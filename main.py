import logging
import threading
import asyncio

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.router import router
from app.ros.manager import ros_manager

# Configure media and connection diagnostics
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

app = FastAPI(
    title="RoverAPI Endpoint",
    description="FastAPI-based Rover Control and Data Collection API with ROS2 Integration",
    version="2.0.0",
)

# Add CORS middleware to allow frontend connections
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins (restrict in production)
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods (GET, POST, OPTIONS, etc.)
    allow_headers=["*"],  # Allow all headers
)

app.include_router(router)


@app.on_event("startup")
async def startup_event():
    """Initialize ROS bridge connection on startup"""
    print("Starting Rover API Server...")

    # Connect to rosbridge in a separate thread to avoid blocking startup
    def connect_ros():
        print("Attempting to connect to rosbridge_server...")
        success = ros_manager.connect()
        if success:
            print(f"Successfully connected to rosbridge at {ros_manager.config.url}")
        else:
            print(f"ERROR: Failed to connect to rosbridge at {ros_manager.config.url}")
            print(
                "  ROS endpoints will not be available until connection is established."
            )
            print(
                "  Ensure rosbridge_server is running: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
            )

    # Start connection attempt in background
    thread = threading.Thread(target=connect_ros, daemon=True)
    thread.start()


@app.on_event("shutdown")
async def shutdown_event():
    """Clean up WebRTC connections and disconnect from ROS bridge on shutdown"""
    print("Shutting down Rover API Server...")

    from app.api.webrtc_utils import shutdown_media
    await shutdown_media()

    from app.api.navigation.camera import camera_manager
    from app.api.science.microscope import microscope_manager
    from app.arduino.manager import arduino_manager

    await asyncio.to_thread(camera_manager.stop_all_cameras)
    await asyncio.to_thread(microscope_manager.stop_microscope)
    await asyncio.to_thread(arduino_manager.disconnect)
    await asyncio.to_thread(ros_manager.disconnect)


@app.get("/")
async def root():
    """Root endpoint to verify server is running"""
    ros_status = "connected" if ros_manager.is_connected else "disconnected"
    return {
        "status": "ok",
        "message": "Rover API Server is running",
        "version": "2.0.0",
        "ros_bridge": ros_status,
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=6767)
