"""H.264 WebRTC signaling for the shared native GStreamer encoder."""

import time


from app.api.ros_image import decode_ros_image
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional

from app.ros.manager import ros_manager
from app.ros.topics import CAMERA_COLOR_IMAGE_RAW_TOPIC
from app.api.webrtc_utils import (
    handle_offer,
    get_frame_timing,
    close_all_connections,
    close_peer_connection,
    get_connection_count,
    MAX_WEBRTC_CLIENTS_PER_SOURCE,
)

router = APIRouter()


class WebRTCOfferRequest(BaseModel):
    """SDP offer from the client."""

    sdp: str
    type: str = "offer"
    topic_name: Optional[str] = None
    fps: Optional[int] = 60


def _make_ros_capture(topic: str):
    """Create a capture function that reads from a ROS camera topic.

    Returns BGR numpy array compatible with OpenCV / av.VideoFrame.
    """

    last_message = None
    last_frame = None

    def capture():
        nonlocal last_message, last_frame
        image_msg = ros_manager.get_latest_camera_image(topic_name=topic)
        if not image_msg:
            return None
        if image_msg is not last_message:
            last_frame = decode_ros_image(image_msg)
            last_message = image_msg
        return last_frame

    return capture


@router.post("/ros/camera/webrtc/offer")
async def webrtc_offer(request: WebRTCOfferRequest, topic_name: Optional[str] = None):
    """
    Establish a WebRTC peer connection for ROS camera streaming.

    Must subscribe to the topic first using POST /api/nav/ros/camera/subscribe.

    Example request:
    ```json
    {
        "sdp": "v=0\\r\\no=- ...",
        "type": "offer",
        "topic_name": "/camera/camera/color/image_raw",
        "fps": 60
    }
    ```
    """
    if not ros_manager.is_connected:
        raise HTTPException(
            status_code=503,
            detail="Not connected to ROS. Ensure rosbridge_server is running.",
        )

    topic = request.topic_name or topic_name or CAMERA_COLOR_IMAGE_RAW_TOPIC
    fps = max(1, min(60, request.fps or 60))
    source_name = f"ros_camera:{topic}"

    capture_fn = _make_ros_capture(topic)

    try:
        answer = await handle_offer(
            source_name=source_name,
            sdp=request.sdp,
            sdp_type=request.type,
            capture_fn=capture_fn,
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
        "topic": topic,
        "fps": fps,
    }


@router.delete("/ros/camera/webrtc/connections/{peer_id}")
async def webrtc_close_peer(peer_id: str, topic_name: Optional[str] = None):
    topic = topic_name or CAMERA_COLOR_IMAGE_RAW_TOPIC
    closed = await close_peer_connection(f"ros_camera:{topic}", peer_id)
    return {"success": True, "closed": closed}


@router.delete("/ros/camera/webrtc")
async def webrtc_close(topic_name: Optional[str] = None):
    """Close all WebRTC connections for a ROS camera topic."""
    topic = topic_name or CAMERA_COLOR_IMAGE_RAW_TOPIC
    source_name = f"ros_camera:{topic}"
    closed = await close_all_connections(source_name)
    return {
        "success": True,
        "message": f"Closed {closed} WebRTC connection(s) for topic '{topic}'",
        "closed": closed,
    }


@router.get("/ros/camera/webrtc/status")
async def webrtc_status(topic_name: Optional[str] = None):
    """Get WebRTC connection status for a ROS camera topic."""
    topic = topic_name or CAMERA_COLOR_IMAGE_RAW_TOPIC
    source_name = f"ros_camera:{topic}"
    count = await get_connection_count(source_name)
    return {
        "success": True,
        "topic": topic,
        "active_connections": count,
        "max_connections": MAX_WEBRTC_CLIENTS_PER_SOURCE,
    }


@router.get("/ros/camera/webrtc/connections/{peer_id}/timing")
async def frame_timing(peer_id: str, topic_name: Optional[str] = None):
    received = time.time_ns() / 1e6
    result = await get_frame_timing(f"ros_camera:{topic_name or CAMERA_COLOR_IMAGE_RAW_TOPIC}", peer_id)
    if result is None:
        raise HTTPException(404, 'Viewer not found')
    return {**result, 'server_receive_ms':received, 'server_send_ms':time.time_ns()/1e6}
