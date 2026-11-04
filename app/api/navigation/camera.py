import json
import os
import re
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Literal, Optional

import cv2
from fastapi import APIRouter, File, Form, HTTPException, Query, UploadFile
from pydantic import BaseModel, Field

from app.utils.json_store import load_json, save_json, save_json_locked
from app.utils.expeditions import EXPEDITION_BASE_DIR, get_safe_expedition_dir

router = APIRouter()

IMAGE_DIR = "storage/images"
META_FILE = "storage/metadata.json"
WAYPOINT_FILE = "storage/waypoints.json"



os.makedirs(IMAGE_DIR, exist_ok=True)
os.makedirs("storage/reports", exist_ok=True)


CAMERA_DEVICES = {
    "microscope": "/dev/camera-microscope",
    "kreo": "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Kreo_Owl_Camera_SN0001-video-index0",
    "arm": "/dev/camera-arm",
    "science": "/dev/camera-science",
    "rover": "/dev/camera-rover",
}

# One native capture and encoder owns each physical camera.
class MultiCameraManager:
    def __init__(self):
        self.cameras = {}
        self.camera_info = {}
        self._lifecycle_lock = threading.RLock()

    def _device_path(self, camera_name: str) -> Optional[str]:
        if camera_name in CAMERA_DEVICES:
            return CAMERA_DEVICES[camera_name]
        if re.fullmatch(r"video[0-9]+", camera_name):
            return f"/dev/{camera_name}"
        return None

    def detect_cameras(self):
        available = []
        used = set()
        candidates = list(CAMERA_DEVICES.items()) + [(p.name, str(p)) for p in sorted(Path('/dev').glob('video[0-9]*'))]
        for name, device in candidates:
            real = os.path.realpath(device)
            if not os.path.exists(device) or real in used:
                continue
            formats = self.get_supported_resolutions(name).get('formats', [])
            if not any(f['fourcc'] in ('MJPG', 'YUYV') for f in formats):
                continue  # Ignore V4L2 metadata nodes without video capture formats.
            used.add(real)
            modes = [(r, f['fourcc'], rate) for f in formats for r in f.get('resolutions', [])
                     for rate in r.get('frame_rates', [30]) if 0 < rate <= 60]
            preferred = [m for m in modes if m[0]['width'] == 1280 and m[0]['height'] == 720]
            best = max(preferred or modes, key=lambda m: (m[2], m[1] == 'MJPG'), default=({'width':1280,'height':720},'MJPG',30))
            available.append({'name':name, 'device_path':device, 'backend':'GStreamer H.264',
                              'default_resolution':f"{best[0]['width']}x{best[0]['height']}",
                              'default_fps':best[2], 'active':self.get_camera_status(name)['active'],
                              'is_named':name in CAMERA_DEVICES})
        return available

    def start_camera(self, camera_name, width=1280, height=720, fps=60, pixel_format=None):
        from app.media.gstreamer import GStreamerSource
        with self._lifecycle_lock:
            device = self._device_path(camera_name)
            if not device or not os.path.exists(device):
                return False
            real = os.path.realpath(device)
            for name, source in list(self.cameras.items()):
                if os.path.realpath(self.camera_info[name]['device_path']) == real:
                    if source.status()['active']:
                        if (source.width, source.height, source.fps) != (width, height, fps):
                            raise HTTPException(409, 'Camera already shared at a different mode. Stop it before changing resolution.')
                        self.cameras[camera_name] = source
                        self.camera_info[camera_name] = dict(self.camera_info[name])
                        return True
                    source.stop()
                    self.cameras.pop(name, None)
                    self.camera_info.pop(name, None)
            try:
                if pixel_format is None:
                    formats = self.get_supported_resolutions(camera_name).get('formats', [])
                    pixel_format = 'MJPG' if any(f['fourcc'] == 'MJPG' for f in formats) else 'YUYV'
                source = GStreamerSource(device, width, height, fps, pixel_format)
            except Exception as exc:
                raise HTTPException(503, f'Camera could not start: {exc}') from exc
            self.cameras[camera_name] = source
            self.camera_info[camera_name] = {'width':width, 'height':height, 'fps':fps, 'device_path':device}
            return True

    def stop_camera(self, camera_name):
        with self._lifecycle_lock:
            source = self.cameras.get(camera_name)
            if source is None:
                return False
            source.stop()
            for name in [n for n, s in self.cameras.items() if s is source]:
                self.cameras.pop(name, None)
                self.camera_info.pop(name, None)
            return True

    def stop_all_cameras(self):
        with self._lifecycle_lock:
            for source in set(self.cameras.values()):
                source.stop()
            self.cameras.clear()
            self.camera_info.clear()

    def capture_frame(self, camera_name):
        source = self.cameras.get(camera_name)
        return source.capture_frame() if source else None

    def get_camera_status(self, camera_name):
        source = self.cameras.get(camera_name)
        status = source.status() if source else {'active':False, 'peers':[], 'fps':0}
        info = self.camera_info.get(camera_name, {})
        return {**info, 'active':status['active'], 'streaming':bool(status['peers']),
                'camera_name':camera_name, 'fps':source.fps if source else 0,
                'measured_fps':status['fps'], 'transport':'H264/RTP/UDP',
                'encoder':'x264-ultrafast', 'active_connections':len(status['peers'])}

    def get_all_statuses(self):
        return {name:self.get_camera_status(name) for name in list(self.cameras)}

    def get_supported_resolutions(self, camera_name: str) -> dict:
        """Query supported resolutions for a camera using v4l2-ctl"""
        device_path = CAMERA_DEVICES.get(camera_name)

        # If not a named camera, check if it's a video device
        if device_path is None and camera_name.startswith("video"):
            try:
                video_index = int(camera_name.replace("video", ""))
                device_path = f"/dev/video{video_index}"
            except ValueError:
                return {
                    "success": False,
                    "error": f"Invalid video device name: {camera_name}",
                    "formats": [],
                }

        if not device_path or not os.path.exists(device_path):
            return {
                "success": False,
                "error": f"Camera '{camera_name}' not found",
                "formats": [],
            }

        # Preset fallback resolutions
        FALLBACK_RESOLUTIONS = [
            {"width": 1920, "height": 1080},
            {"width": 1280, "height": 720},
            {"width": 640, "height": 480},
            {"width": 320, "height": 240},
        ]

        try:
            # Try using v4l2-ctl to query supported formats
            result = subprocess.run(
                ["v4l2-ctl", "-d", device_path, "--list-formats-ext"],
                capture_output=True,
                text=True,
                timeout=5,
            )

            if result.returncode != 0:
                print(
                    f"[Camera] v4l2-ctl failed for {camera_name}, using fallback presets"
                )
                return {
                    "success": True,
                    "formats": [
                        {
                            "fourcc": "PRESET",
                            "description": "Common resolutions (v4l2-ctl unavailable)",
                            "resolutions": FALLBACK_RESOLUTIONS,
                        }
                    ],
                }

            # Parse v4l2-ctl output
            formats = []
            current_format = None
            current_format_desc = None
            current_resolutions = []
            current_resolution = None

            for line in result.stdout.split("\n"):
                # Parse format line: [0]: 'MJPG' (Motion-JPEG, compressed)
                format_match = re.match(r"\s*\[\d+\]: '(\w+)' \((.+)\)", line)
                if format_match:
                    # Save previous format
                    if current_format and current_resolutions:
                        formats.append(
                            {
                                "fourcc": current_format,
                                "description": current_format_desc,
                                "resolutions": current_resolutions,
                            }
                        )

                    current_format = format_match.group(1)
                    current_format_desc = format_match.group(2)
                    current_resolutions = []
                    current_resolution = None
                    continue

                # Parse size line: Size: Discrete 1280x720
                size_match = re.match(r"\s*Size: Discrete (\d+)x(\d+)", line)
                if size_match and current_format:
                    width, height = int(size_match.group(1)), int(size_match.group(2))
                    # Avoid duplicates
                    if not any(
                        r["width"] == width and r["height"] == height
                        for r in current_resolutions
                    ):
                        current_resolutions.append({"width": width, "height": height})
                    current_resolution = next(
                        r for r in current_resolutions if r["width"] == width and r["height"] == height
                    )
                    continue

                # Frame rates belong to this pixel format AND resolution.
                interval_match = re.search(r"Interval: Discrete .*?\(([\d.]+) fps\)", line)
                if interval_match and current_resolution is not None:
                    rate = float(interval_match.group(1))
                    if 0 < rate <= 1000:
                        rates = current_resolution.setdefault("frame_rates", [])
                        if rate not in rates:
                            rates.append(rate)
                            rates.sort(reverse=True)

            # Save last format
            if current_format and current_resolutions:
                formats.append(
                    {
                        "fourcc": current_format,
                        "description": current_format_desc,
                        "resolutions": current_resolutions,
                    }
                )

            if not formats:
                print(
                    f"[Camera] No formats parsed from v4l2-ctl for {camera_name}, using fallback"
                )
                return {
                    "success": True,
                    "formats": [
                        {
                            "fourcc": "PRESET",
                            "description": "Common resolutions (parsing failed)",
                            "resolutions": FALLBACK_RESOLUTIONS,
                        }
                    ],
                }

            return {"success": True, "formats": formats}

        except FileNotFoundError:
            print(
                f"[Camera] v4l2-ctl not found, using fallback presets for {camera_name}"
            )
            return {
                "success": True,
                "formats": [
                    {
                        "fourcc": "PRESET",
                        "description": "Common resolutions (v4l2-ctl not installed)",
                        "resolutions": FALLBACK_RESOLUTIONS,
                    }
                ],
            }
        except subprocess.TimeoutExpired:
            print(f"[Camera] v4l2-ctl timeout for {camera_name}, using fallback")
            return {
                "success": True,
                "formats": [
                    {
                        "fourcc": "PRESET",
                        "description": "Common resolutions (query timeout)",
                        "resolutions": FALLBACK_RESOLUTIONS,
                    }
                ],
            }
        except Exception as e:
            print(f"[Camera] Error querying resolutions for {camera_name}: {e}")
            return {
                "success": True,
                "formats": [
                    {
                        "fourcc": "PRESET",
                        "description": "Common resolutions (query error)",
                        "resolutions": FALLBACK_RESOLUTIONS,
                    }
                ],
            }

# Global camera manager instance
camera_manager = MultiCameraManager()


def secure_filename(filename: str) -> str:
    """Simple secure filename implementation"""
    return "".join(c for c in filename if c.isalnum() or c in "._-")



@router.get("/cameras/detect")
def detect_cameras():
    """Detect all available cameras"""
    cameras = camera_manager.detect_cameras()
    return {"status": "ok", "cameras": cameras, "count": len(cameras)}


@router.get("/cameras/{camera_name}/resolutions")
def get_camera_resolutions(camera_name: str):
    """Get supported resolutions for a specific camera"""
    # Validate camera name (named camera or video device)
    is_valid = camera_name in CAMERA_DEVICES or (
        camera_name.startswith("video") and camera_name[5:].isdigit()
    )

    if not is_valid:
        raise HTTPException(
            status_code=404,
            detail=f"Camera '{camera_name}' not found. Available named cameras: {', '.join(CAMERA_DEVICES.keys())}. Also supports video0, video1, etc.",
        )

    result = camera_manager.get_supported_resolutions(camera_name)

    if not result.get("success", False) and result.get("error"):
        raise HTTPException(status_code=404, detail=result["error"])

    # Get device path for response
    device_path = CAMERA_DEVICES.get(camera_name)
    if device_path is None and camera_name.startswith("video"):
        try:
            video_index = int(camera_name.replace("video", ""))
            device_path = f"/dev/video{video_index}"
        except ValueError:
            device_path = "unknown"

    return {
        "success": True,
        "camera_name": camera_name,
        "device_path": device_path,
        "formats": result.get("formats", []),
    }


@router.post("/cameras/{camera_name}/start")
def start_camera(
    camera_name: str,
    width: int = Form(1280, ge=1),
    height: int = Form(720, ge=1),
    fps: int = Form(60, ge=1, le=60),
    pixel_format: Optional[str] = Form(None),
):
    """Start specific camera"""
    # Validate camera name (named camera or video device)
    is_valid = camera_name in CAMERA_DEVICES or (
        camera_name.startswith("video") and camera_name[5:].isdigit()
    )

    if not is_valid:
        raise HTTPException(
            status_code=404,
            detail=f"Camera '{camera_name}' not found. Available named cameras: {', '.join(CAMERA_DEVICES.keys())}. Also supports video0, video1, etc.",
        )

    if pixel_format and pixel_format not in ('MJPG', 'YUYV'):
        raise HTTPException(status_code=400, detail="Invalid camera pixel format")
    success = camera_manager.start_camera(camera_name, width, height, fps, pixel_format)
    if not success:
        raise HTTPException(
            status_code=500, detail=f"Failed to open camera '{camera_name}'"
        )

    return {
        "status": "ok",
        "message": f"Camera '{camera_name}' started",
        "camera": camera_manager.get_camera_status(camera_name),
    }


@router.post("/cameras/{camera_name}/stop")
def stop_camera(camera_name: str):
    """Stop specific camera"""
    success = camera_manager.stop_camera(camera_name)
    if not success:
        raise HTTPException(status_code=404, detail=f"Camera '{camera_name}' not found")
    return {"status": "ok", "message": f"Camera '{camera_name}' stopped"}


@router.post("/cameras/stop_all")
def stop_all_cameras():
    """Stop all cameras"""
    camera_manager.stop_all_cameras()
    return {"status": "ok", "message": "All cameras stopped"}


@router.get("/cameras/status")
def cameras_status():
    """Get status of all cameras"""
    return {"status": "ok", "cameras": camera_manager.get_all_statuses()}


@router.get("/cameras/{camera_name}/status")
def camera_status(camera_name: str):
    """Get specific camera status"""
    return {"status": "ok", "camera": camera_manager.get_camera_status(camera_name)}


class DejelloRequest(BaseModel):
    enabled: Optional[bool] = None
    readout_ms: Optional[float] = Field(None, ge=0, le=100)
    reset_readout: bool = False
    reset_calibration: bool = False
    # imu: gyro via calibration; video: optical flow only, no IMU; auto: imu when calibrated, else video
    source: Optional[Literal["auto", "imu", "video"]] = None


def _dejello_source(camera_name: str):
    source = camera_manager.cameras.get(camera_name)
    if source is None:
        raise HTTPException(404, f"Camera '{camera_name}' is not running")
    if not hasattr(source, "dejello"):
        raise HTTPException(409, "IMU rolling-shutter correction is not available for this camera")
    return source


@router.get("/cameras/{camera_name}/dejello")
def dejello_status(camera_name: str):
    """IMU rolling-shutter correction state. Gyro data never leaves the rover; only this summary does."""
    try:
        return {"status": "ok", "dejello": _dejello_source(camera_name).dejello()}
    except (RuntimeError, ValueError, TimeoutError) as exc:
        raise HTTPException(409, str(exc)) from exc


@router.post("/cameras/{camera_name}/dejello")
def configure_dejello(camera_name: str, request: DejelloRequest):
    """Enable/disable correction, override or reset readout, or discard the calibration."""
    readout = None if request.reset_readout else (request.readout_ms if request.readout_ms is not None else "keep")
    try:
        return {"status": "ok", "dejello": _dejello_source(camera_name).dejello(
            enabled=request.enabled, readout_ms=readout, reset=request.reset_calibration, source=request.source)}
    except (RuntimeError, ValueError, TimeoutError) as exc:
        raise HTTPException(409, str(exc)) from exc


@router.post("/cameras/{camera_name}/capture")
def capture_from_camera(
    camera_name: str,
    latitude: float = Form(0),
    longitude: float = Form(0),
    altitude: float = Form(0),
    heading: float = Form(0),
    speed: float = Form(0),
    battery_level: float = Form(100),
    temperature: float = Form(20),
    humidity: float = Form(50),
    note: str = Form(""),
    mission_id: str = Form("default"),
    rover_id: str = Form("rover_001"),
    tags: str = Form(""),
    expedition_id: str = Form(""),
):
    """Capture image from specific camera"""
    frame = camera_manager.capture_frame(camera_name)
    if frame is None:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to capture from camera '{camera_name}'. Is it started?",
        )

    # Save frame as JPEG

    if not expedition_id:
        return {"status": "error", "message": "Expedition ID must be provided."}

    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{timestamp}_{camera_name}_capture.jpg"

    img_dir_loc = get_safe_expedition_dir(expedition_id)

    filepath = os.path.join(img_dir_loc, filename)

    if not cv2.imwrite(filepath, frame):
        raise HTTPException(status_code=500, detail="Failed to save captured image")
    file_size = os.path.getsize(filepath)

    # Get actual image dimensions
    img_height, img_width = frame.shape[:2]

    # Create metadata entry
    entry = {
        "file": filename,
        "timestamp": timestamp,
        "datetime_iso": datetime.utcnow().isoformat(),
        "location": {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
            "heading": heading,
        },
        "motion": {
            "speed": speed,
            "heading": heading,
        },
        "environment": {
            "temperature": temperature,
            "humidity": humidity,
        },
        "rover_status": {
            "battery_level": battery_level,
            "rover_id": rover_id,
            "mission_id": mission_id,
        },
        "camera": {
            "camera_name": camera_name,
            "device_path": CAMERA_DEVICES.get(camera_name)
            or (
                f"/dev/video{camera_name.replace('video', '')}"
                if camera_name.startswith("video")
                else ""
            ),
            "settings": {
                "resolution": f"{img_width}x{img_height}",
                "hardware_capture": True,
            },
            "file_size_bytes": file_size,
        },
        "note": note,
        "tags": tags.split(",") if tags else [],
        "expedition_id": expedition_id,
    }
    save_json_locked(META_FILE, lambda data: data + [entry])

    return {
        "status": "ok",
        "message": f"Image captured from camera '{camera_name}'",
        "saved": filename,
        "metadata": entry,
        "file_size_mb": round(file_size / (1024 * 1024), 2),
    }


@router.post("/capture")
def capture(
    image: UploadFile = File(...),
    latitude: float = Form(0),
    longitude: float = Form(0),
    altitude: float = Form(0),
    heading: float = Form(0),
    speed: float = Form(0),
    battery_level: float = Form(100),
    temperature: float = Form(20),
    humidity: float = Form(50),
    note: str = Form(""),
    mission_id: str = Form("default"),
    rover_id: str = Form("rover_001"),
    camera_settings: str = Form("{}"),
    tags: str = Form(""),
    expedition_id: str = Form(""),
):
    """Capture camera screenshot with metadata"""
    if not image.filename:
        raise HTTPException(status_code=400, detail="No image selected")

    try:
        camera_settings_dict = json.loads(camera_settings) if camera_settings else {}
    except json.JSONDecodeError:
        camera_settings_dict = {}

    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{timestamp}_{secure_filename(image.filename)}"
    img_dir_loc = get_safe_expedition_dir(expedition_id) if expedition_id else IMAGE_DIR
    os.makedirs(img_dir_loc, exist_ok=True)

    filepath = os.path.join(img_dir_loc, filename)
    print(f"---->0x100 [Capture] Saving uploaded image to {filepath}")

    # Save uploaded file
    import shutil
    with open(filepath, "wb") as f:
        shutil.copyfileobj(image.file, f, length=1024 * 1024)

    file_size = os.path.getsize(filepath)

    entry = {
        "file": filename,
        "timestamp": timestamp,
        "datetime_iso": datetime.utcnow().isoformat(),
        "location": {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
            "heading": heading,
        },
        "motion": {"speed": speed, "heading": heading},
        "environment": {"temperature": temperature, "humidity": humidity},
        "rover_status": {
            "battery_level": battery_level,
            "rover_id": rover_id,
            "mission_id": mission_id,
        },
        "camera": {"settings": camera_settings_dict, "file_size_bytes": file_size},
        "note": note,
        "tags": tags.split(",") if tags else [],
    }
    save_json_locked(META_FILE, lambda data: data + [entry])

    return {
        "status": "ok",
        "saved": filename,
        "metadata": entry,
        "file_size_mb": round(file_size / (1024 * 1024), 2),
    }


@router.post("/waypoint")
def add_waypoint(
    latitude: float = Form(0),
    longitude: float = Form(0),
    altitude: float = Form(0),
    mission_id: str = Form("default"),
    rover_id: str = Form("rover_001"),
    name: str = Form(None),
    category: str = Form("general"),
    description: str = Form(""),
    auto_generated: str = Form("false"),
):
    """Add a waypoint (manual or auto). If 'name' is provided, it's manual; otherwise auto-generated."""
    def update(waypoints):
        nonlocal name, category, description

        is_auto_generated = False

        if not name:
            # Auto-generate name
            waypoint_count = len(
                [wp for wp in waypoints if wp.get("mission_id") == mission_id]
            )
            name = f"Auto Waypoint {waypoint_count + 1}"
            is_auto_generated = True
            category = "auto"
            description = f"Automatically generated waypoint during {mission_id}"
        else:
            # Manual waypoint
            is_auto_generated = auto_generated.lower() == "true"

        # Create waypoint entry
        entry = {
            "name": name,
            "location": {
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
            },
            "category": category,
            "description": description,
            "mission_id": mission_id,
            "rover_id": rover_id,
            "auto_generated": is_auto_generated,
            "timestamp": datetime.utcnow().isoformat(),
            "timestamp_readable": datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S UTC"),
            "waypoint_id": f"wp_{len(waypoints) + 1:03d}",
        }

        waypoints.append(entry)
        return waypoints

    entry = save_json_locked(WAYPOINT_FILE, update)[-1]

    return {"status": "ok", "waypoint": entry}


@router.get("/get_waypoints")
def get_waypoints(mission_id: str = Query(None)):
    """Get waypoints"""
    waypoints = load_json(WAYPOINT_FILE)

    if mission_id:
        waypoints = [wp for wp in waypoints if wp.get("mission_id") == mission_id]

    return {"status": "ok", "waypoints": waypoints, "count": len(waypoints)}


@router.get("/get_metadata")
def get_metadata(mission_id: str = Query(None)):
    """Get image metadata"""
    metadata = load_json(META_FILE)

    if mission_id:
        metadata = [
            md
            for md in metadata
            if md.get("rover_status", {}).get("mission_id") == mission_id
        ]

    return {"status": "ok", "metadata": metadata, "count": len(metadata)}


@router.post("/capture_test_data")
def capture_test_data(
    title: str = Form("Rover Mission Capture"),
    description: str = Form("Camera feed screenshot with metadata"),
    latitude: float = Form(37.7749),
    longitude: float = Form(-122.4194),
    altitude: float = Form(100.5),
    heading: float = Form(45.0),
    speed: float = Form(1.5),
    battery_level: float = Form(85.0),
    temperature: float = Form(22.0),
    humidity: float = Form(50.0),
    mission_id: str = Form("rover_challenge_mission"),
    rover_id: str = Form("rover_challenge_001"),
    note: str = Form("Rover mission data capture"),
):
    """Generate test rover image with metadata - useful for testing and demos"""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        raise HTTPException(
            status_code=500, detail="PIL not installed. Run: pip install pillow"
        )

    # Create test image
    img = Image.new("RGB", (800, 600), color="lightblue")
    draw = ImageDraw.Draw(img)

    try:
        font_large = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 28
        )
        font_medium = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18
        )
    except:
        try:
            font_large = ImageFont.truetype("/System/Library/Fonts/Arial.ttf", 28)
            font_medium = ImageFont.truetype("/System/Library/Fonts/Arial.ttf", 18)
        except:
            font_large = ImageFont.load_default()
            font_medium = ImageFont.load_default()

    # Add content to image
    draw.text((30, 30), title, fill="black", font=font_large)
    draw.text((30, 80), description, fill="darkblue", font=font_medium)
    draw.text(
        (30, 120),
        f"Rover Mission - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        fill="gray",
        font=font_medium,
    )

    # Add rover visual
    draw.rectangle([30, 180, 770, 250], outline="black", width=3)
    draw.text((40, 200), "ROVER MISSION DATA CAPTURED", fill="black", font=font_medium)

    # Add mission info
    draw.text(
        (30, 300),
        f"GPS Coordinates: {latitude}, {longitude}",
        fill="darkgreen",
        font=font_medium,
    )
    draw.text(
        (30, 330),
        f"Battery Level: {battery_level}%",
        fill="darkgreen",
        font=font_medium,
    )
    draw.text(
        (30, 360), f"Temperature: {temperature}°C", fill="darkgreen", font=font_medium
    )
    draw.text(
        (30, 390), f"Mission ID: {mission_id}", fill="darkgreen", font=font_medium
    )

    # Border
    draw.rectangle([10, 10, 790, 590], outline="black", width=4)

    # Save image
    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{timestamp}_rover_test_capture.jpg"
    filepath = os.path.join(IMAGE_DIR, filename)
    img.save(filepath, "JPEG", quality=95)

    file_size = os.path.getsize(filepath)

    # Create metadata entry
    entry = {
        "file": filename,
        "timestamp": timestamp,
        "datetime_iso": datetime.utcnow().isoformat(),
        "location": {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
            "heading": heading,
        },
        "motion": {"speed": speed, "heading": heading},
        "environment": {"temperature": temperature, "humidity": humidity},
        "rover_status": {
            "battery_level": battery_level,
            "rover_id": rover_id,
            "mission_id": mission_id,
        },
        "camera": {
            "settings": {"resolution": "800x600", "test_image": True},
            "file_size_bytes": file_size,
        },
        "note": note,
        "tags": ["rover", "test", "generated"],
    }

    metadata = save_json_locked(META_FILE, lambda data: data + [entry])

    return {
        "status": "ok",
        "message": "Test data captured successfully",
        "saved": filename,
        "filepath": filepath,
        "file_size_bytes": file_size,
        "file_size_kb": round(file_size / 1024, 2),
        "metadata": entry,
        "total_captures": len(metadata),
    }
