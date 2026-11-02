"""H.264 WebRTC signaling for the shared native GStreamer encoder."""

import time


import asyncio

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel
from typing import Optional

from app.api.webrtc_utils import (
    handle_offer,
    get_frame_timing,
    close_all_connections,
    close_peer_connection,
    get_connection_count,
    get_all_webrtc_stats,
    MAX_WEBRTC_CLIENTS_PER_SOURCE,
)

router = APIRouter()


class WebRTCOfferRequest(BaseModel):
    """SDP offer from the client."""

    sdp: str
    type: str = "offer"
    fps: Optional[int] = 60


@router.post("/cameras/{camera_name}/webrtc/offer")
async def webrtc_offer(
    camera_name: str,
    request: WebRTCOfferRequest,
):
    """
    Establish a WebRTC peer connection for camera streaming.

    Send an SDP offer and receive an SDP answer. The server will
    stream H.264 encoded video from the specified camera.

    The camera is captured and encoded once by GStreamer. Each viewer
    receives that H.264 stream over WebRTC UDP without re-encoding.

    Example request:
    ```json
    {
        "sdp": "v=0\\r\\no=- ...",
        "type": "offer",
        "fps": 60
    }
    ```
    """
    from app.api.navigation.camera import camera_manager, CAMERA_DEVICES

    # Validate camera name
    if camera_name not in CAMERA_DEVICES and not (camera_name.startswith('video') and camera_name[5:].isdigit()):
        raise HTTPException(
            status_code=404,
            detail=f"Camera '{camera_name}' not found. "
            f"Available: {', '.join(CAMERA_DEVICES.keys())}",
        )

    # Check if camera is started
    status = await asyncio.to_thread(camera_manager.get_camera_status, camera_name)
    if not status["active"]:
        raise HTTPException(
            status_code=400,
            detail=f"Camera '{camera_name}' not started. "
            f"Call /cameras/{camera_name}/start first",
        )

    # Requested delivery target; actual capture FPS is reported separately by the camera.
    fps = max(1, min(60, request.fps or 60))

    # Create a capture function bound to this camera
    def capture():
        return camera_manager.capture_frame(camera_name)

    source_name = f"camera:{camera_name}"

    try:
        answer = await handle_offer(
            source_name=source_name,
            sdp=request.sdp,
            sdp_type=request.type,
            capture_fn=capture,
            fps=fps,
        )
    except ValueError as e:
        raise HTTPException(status_code=429, detail=str(e))
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to create WebRTC connection: {e}",
        )

    return {
        "success": True,
        "sdp": answer["sdp"],
        "type": answer["type"],
        "peer_id": answer["peer_id"],
        "adaptive_quality": False,
        "codec": "H264",
        "transport": "UDP",
        "encoder": "x264-ultrafast",
        "target_fps": answer["target_fps"],
        "camera_name": camera_name,
        "fps": fps,
    }


@router.delete("/cameras/{camera_name}/webrtc/connections/{peer_id}")
async def webrtc_close_peer(camera_name: str, peer_id: str):
    closed = await close_peer_connection(f"camera:{camera_name}", peer_id)
    return {"success": True, "closed": closed}


@router.delete("/cameras/{camera_name}/webrtc")
async def webrtc_close(camera_name: str):
    """Close all WebRTC connections for a camera."""
    source_name = f"camera:{camera_name}"
    closed = await close_all_connections(source_name)
    return {
        "success": True,
        "message": f"Closed {closed} WebRTC connection(s) for camera '{camera_name}'",
        "closed": closed,
    }


@router.get("/cameras/{camera_name}/webrtc/status")
async def webrtc_status(camera_name: str):
    """Get WebRTC connection status for a camera."""
    source_name = f"camera:{camera_name}"
    count = await get_connection_count(source_name)
    return {
        "success": True,
        "camera_name": camera_name,
        "active_connections": count,
        "max_connections": MAX_WEBRTC_CLIENTS_PER_SOURCE,
    }


@router.get("/cameras/webrtc/status")
async def webrtc_all_status():
    """Get WebRTC connection status for all cameras."""
    stats = await get_all_webrtc_stats()
    return {
        "success": True,
        "sources": stats,
    }


@router.get("/cameras/{camera_name}/webrtc/connections/{peer_id}/timing")
async def frame_timing(camera_name: str, peer_id: str):
    received = time.time_ns() / 1e6
    result = await get_frame_timing(f"camera:{camera_name}", peer_id)
    if result is None:
        raise HTTPException(404, 'Viewer not found')
    return {**result, 'server_receive_ms':received, 'server_send_ms':time.time_ns()/1e6}
