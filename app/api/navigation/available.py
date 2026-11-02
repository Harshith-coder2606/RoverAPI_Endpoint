from fastapi import APIRouter, HTTPException
from app.py_types import listOfStrings

router = APIRouter()


def nav_avi_l_ret() -> listOfStrings:
    # * Add all endpoints here without fail.
    return [
        "available",
        "capture",
        "capture_test_data",
        "waypoint",
        "get_waypoints",
        "get_metadata",
        "generate_report",
        "generate_comprehensive_report",
        "export_data",
        "reports",
        "route_analysis",
        "download/<filename>",
        "cameras/detect",
        "cameras/{camera_name}/start",
        "cameras/{camera_name}/stop",
        "cameras/stop_all",
        "cameras/status",
        "cameras/{camera_name}/status",
        "cameras/{camera_name}/capture",
        "cameras/{camera_name}/webrtc/offer",
        "cameras/{camera_name}/webrtc/status",
        "cameras/{camera_name}/webrtc/connections/{peer_id}",
        "cameras/{camera_name}/webrtc/connections/{peer_id}/timing",
        "cameras/webrtc/status",
        "ros/camera/webrtc/offer",
        "ros/camera/webrtc/status",
        "ros/camera/webrtc/connections/{peer_id}",
        "ros/camera/webrtc/connections/{peer_id}/timing",
        "ros/motor_rpms/subscribe",
        "ros/motor_rpms",
    ]


@router.post("/available")
async def nav_avi():
    return {"success": True, "status": "Success", "endpoints": nav_avi_l_ret()}
