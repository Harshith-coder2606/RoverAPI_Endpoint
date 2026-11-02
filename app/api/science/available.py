from fastapi import APIRouter
from app.py_types import listOfStrings

router = APIRouter()


def sci_avi_l_ret() -> listOfStrings:
    # * Add all endpoints here without fail.
    return [
        "available",
        "reports",
        "get_report",
        "report/:id",
        "report/path/:path",
        "report/:id/path/:path",
        "/reports/ids",
        "/reports/path",
        "microscope/start",
        "microscope/stop",
        "microscope/status",
        "microscope/webrtc/offer",
        "microscope/webrtc/status",
        "microscope/webrtc/connections/{peer_id}",
        "microscope/webrtc/connections/{peer_id}/timing",
        "microscope/capture",
        "sensor_data",
        "control/enable",
        "control/linear_actuator",
        "control/drill",
        "control/barrel",
        "control/servo",
        "control/command",
        "control/status",
        "drill/subscribe",
        "drill/data",
        "warnings/subscribe",
        "warnings",
        "warnings/codes",
    ]


@router.post("/available")
async def sci_avi():
    return {"success": True, "status": "Success", "endpoints": sci_avi_l_ret()}
