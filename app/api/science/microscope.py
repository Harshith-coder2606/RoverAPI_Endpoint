import os
from datetime import datetime

import cv2
from fastapi import APIRouter, Form, HTTPException, Query

from app.utils.json_store import load_json, save_json, save_json_locked
from app.utils.expeditions import EXPEDITION_BASE_DIR, get_safe_expedition_dir

router = APIRouter()

IMAGE_DIR = "storage/images"
META_FILE = "storage/metadata.json"


os.makedirs(IMAGE_DIR, exist_ok=True)


# Microscope hardware management
class MicroscopeManager:
    """The microscope shares the same native H.264 pipeline as navigation cameras."""
    def start_microscope(self, width=1280, height=720, fps=60):
        from app.api.navigation.camera import camera_manager
        return camera_manager.start_camera('microscope', width, height, fps)

    def stop_microscope(self):
        from app.api.navigation.camera import camera_manager
        return camera_manager.stop_camera('microscope')

    def capture_frame(self):
        from app.api.navigation.camera import camera_manager
        return camera_manager.capture_frame('microscope')

    def get_status(self):
        from app.api.navigation.camera import camera_manager
        return camera_manager.get_camera_status('microscope')


# Global microscope manager instance
microscope_manager = MicroscopeManager()




@router.post("/microscope/start")
def start_microscope(
    width: int = Form(640, ge=1), height: int = Form(480, ge=1), fps: int = Form(30, ge=1, le=60)
):
    """Start microscope device"""
    success = microscope_manager.start_microscope(width, height, fps)
    if not success:
        raise HTTPException(
            status_code=500,
            detail="Microscope device unavailable",
        )

    return {
        "success": True,
        "message": "Microscope started",
        "status": microscope_manager.get_status(),
    }


@router.post("/microscope/stop")
def stop_microscope():
    """Stop microscope device"""
    success = microscope_manager.stop_microscope()
    if not success:
        raise HTTPException(status_code=404, detail="Microscope not active")
    return {"success": True, "message": "Microscope stopped"}


@router.get("/microscope/status")
def microscope_status():
    """Get microscope status"""
    return {"success": True, "status": microscope_manager.get_status()}


@router.post("/microscope/capture")
def capture_from_microscope(
    latitude: float = Form(0),
    longitude: float = Form(0),
    altitude: float = Form(0),
    battery_level: float = Form(100),
    mission_id: str = Form("default"),
    rover_id: str = Form("rover_001"),
    note: str = Form(""),
    expedition_id: str = Form("")
):
    """Capture image from microscope with full metadata"""
    frame = microscope_manager.capture_frame()
    if frame is None:
        raise HTTPException(
            status_code=500,
            detail="Failed to capture from microscope. Is it started?",
        )

    # Save frame as JPEG
    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{timestamp}_microscope_capture.jpg"
    img_dir_loc = get_safe_expedition_dir(expedition_id) if expedition_id else IMAGE_DIR

    filepath = os.path.join(img_dir_loc, filename)

    if not cv2.imwrite(filepath, frame):
        raise HTTPException(status_code=500, detail="Failed to save captured image")
    file_size = os.path.getsize(filepath)

    # Get actual image dimensions
    height, width = frame.shape[:2]

    # Create metadata entry
    entry = {
        "file": filename,
        "timestamp": timestamp,
        "datetime_iso": datetime.utcnow().isoformat(),
        "location": {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
        },
        "rover_status": {
            "battery_level": battery_level,
            "rover_id": rover_id,
            "mission_id": mission_id,
        },
        "camera": {
            "device": "microscope",
            "device_path": microscope_manager.get_status().get("device_path", ""),
            "settings": {"resolution": f"{width}x{height}"},
            "file_size_bytes": file_size,
        },
        "note": note,
        "tags": ["microscope", "science"],  # Auto-tags
    }
    save_json_locked(META_FILE, lambda data: data + [entry])

    return {
        "success": True,
        "message": "Image captured from microscope",
        "saved": filename,
        "metadata": entry,
        "file_size_mb": round(file_size / (1024 * 1024), 2),
    }
