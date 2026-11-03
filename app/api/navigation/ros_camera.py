"""ROS camera image streaming endpoints"""

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel
from typing import Optional
from app.ros.manager import ros_manager
from app.ros.topics import CAMERA_COLOR_IMAGE_RAW_TOPIC


router = APIRouter()


class SubscribeCameraRequest(BaseModel):
    """Request model for camera subscription"""

    topic_name: Optional[str] = CAMERA_COLOR_IMAGE_RAW_TOPIC


@router.post("/ros/camera/subscribe")
async def subscribe_to_camera(request: SubscribeCameraRequest):
    """
    Subscribe to ROS camera image topic.
    After subscribing, use GET /api/nav/ros/camera/latest to retrieve latest frame.

    Example request:
    ```json
    {
        "topic_name": "/camera/camera/color/image_raw"
    }
    ```
    """
    if not ros_manager.is_connected:
        raise HTTPException(
            status_code=503,
            detail="Not connected to ROS. Ensure rosbridge_server is running.",
        )

    topic_name = request.topic_name or CAMERA_COLOR_IMAGE_RAW_TOPIC
    success = ros_manager.subscribe_camera_image(topic_name=topic_name)

    if not success:
        raise HTTPException(
            status_code=500, detail=f"Failed to subscribe to {topic_name}"
        )

    return {
        "success": True,
        "message": f"Subscribed to {topic_name} topic",
        "topic": topic_name,
        "info": "Use GET /api/nav/ros/camera/latest to retrieve latest image data",
    }


@router.get("/ros/camera/latest")
async def get_latest_camera_image(topic_name: Optional[str] = None):
    """
    Get the latest camera image from ROS topic.
    Must subscribe first using POST /api/nav/ros/camera/subscribe.

    Returns image message with base64 encoded image data.
    """
    if not ros_manager.is_connected:
        raise HTTPException(
            status_code=503,
            detail="Not connected to ROS. Ensure rosbridge_server is running.",
        )

    topic = topic_name or CAMERA_COLOR_IMAGE_RAW_TOPIC
    image_msg = ros_manager.get_latest_camera_image(topic_name=topic)

    if image_msg is None:
        return {
            "success": False,
            "message": f"No image data received yet from {topic}. Ensure you've subscribed and messages are being published.",
            "data": None,
        }

    return {
        "success": True,
        "message": "Latest camera image",
        "topic": topic,
        "data": {
            "width": image_msg.get("width"),
            "height": image_msg.get("height"),
            "encoding": image_msg.get("encoding"),
            "data": image_msg.get("data"),  # base64 encoded
        },
    }


@router.post("/ros/camera/unsubscribe")
async def unsubscribe_from_camera(request: SubscribeCameraRequest):
    """
    Unsubscribe from ROS camera topic.

    Example request:
    ```json
    {
        "topic_name": "/camera/camera/color/image_raw"
    }
    ```
    """
    if not ros_manager.is_connected:
        raise HTTPException(
            status_code=503,
            detail="Not connected to ROS. Ensure rosbridge_server is running.",
        )

    topic_name = request.topic_name or CAMERA_COLOR_IMAGE_RAW_TOPIC
    from app.api.webrtc_utils import get_connection_count
    if await get_connection_count(f'ros_camera:{topic_name}'):
        raise HTTPException(409, 'ROS camera still has active viewers. Close them before unsubscribing.')
    ros_manager.unsubscribe(topic_name)

    return {
        "success": True,
        "message": f"Unsubscribed from {topic_name}",
        "topic": topic_name,
    }
