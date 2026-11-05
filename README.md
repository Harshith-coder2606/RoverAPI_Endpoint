# RoverAPI Endpoint
This repository is the API endpoint primarily for the rover's onboard computer with integrated ROS2 support via rosbridge. This document will brief you about accessing, contributing and utilizing this repository.

# Setup
Use uv for dependency management.

Initial setup command:
```sh
uv sync
```

Expose the port over the entire network for other machines to connect to:
```sh
uv run main.py
```

For the dashboard devs, use the URL `http://<IP-Addr>:6767/<endpoint>`

## Camera video

The dashboard uses H.264 over WebRTC UDP. GStreamer captures and encodes each
physical camera once, then shares its encoded stream with up to five viewers.
MJPEG and JPEG-over-WebSocket playback routes have been removed. HTTP handles
camera controls, still images, and SDP signaling, not video frames.

Install the native packages on the onboard Ubuntu computer before starting video:

```sh
sudo apt install python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
  gir1.2-gst-plugins-bad-1.0 gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-nice v4l-utils
```

The media worker runs with `/usr/bin/python3`, independently of the API virtual
environment. Set `GSTREAMER_PYTHON` if the native bindings use a different Python.
Use `GST_PLUGIN_PATH` for an explicit custom plugin installation. GStreamer 1.20
on Ubuntu 22.04 is the tested baseline.

Select **Kreo Owl** in the dashboard and click **Start**. Detection uses its stable
`/dev/v4l/by-id/` path. The 720p60 mode uses MJPEG USB capture, x264 `ultrafast`,
`tune=zerolatency`, no B-frames or lookahead, a one-frame leaky queue, a one-second
keyframe interval, and a 6 Mb/s target. Other cameras use their advertised rates.
The worker temporarily disables dynamic frame-rate reduction above 30 FPS and
restores its previous setting on shutdown. The browser must support H.264.

Closing a viewer releases that peer. **Stop** stops the shared camera and all of
its viewers. A camera already shared at another mode returns HTTP 409 rather than
silently changing other viewers' capture settings. Change its mode after stopping
the camera. Still-image capture reuses the current camera buffer.

The displayed latency matches source-buffer delivery timestamps to browser video
presentation callbacks using RTP timestamps. Clock probes use the peer's data
channel. The value excludes camera exposure, camera-internal buffering, and screen
scanout. It is an estimate, not a camera-to-screen measurement. Video continues if
the browser does not support the timestamp callbacks.

Hardware validation on 3 November 2026 used the Kreo Owl at 1280×720, nominal
60 FPS, and 6 Mb/s. A 30-second steady dashboard run measured 35.4 ms median and
37.5 ms p95 source-buffer-to-browser delay. A run after repeated view changes
measured 45.9 ms median and 77.6 ms p95. The clock-probe half RTT was about
1.3–1.4 ms. These runs reported no RTP packet loss, decoded-frame drops, or
freezes. They do not establish exposure-to-screen latency.

Four fullscreen entry/exit cycles at 480p retained one viewer and the existing
capture mode. Five transitions during delayed offer responses reclaimed their
late peers. A Kreo snapshot saved successfully. Two synthetic ROS image viewers
also shared a topic through the onboard rosbridge; closing one preserved the
other, and global unsubscribe returned HTTP 409 until both closed. Physical
microscope and camera/IMU motion tests remain unverified.

The onboard `uv.service` now runs release
`a4794dcaae0022a793b0b83ae53cc647c1b30856` from
`~/.local/share/rover-api/releases/` on port 6767. Its storage directory links to
the existing checkout's storage. The previous checkout and its local edits are
preserved. The service override is
`/etc/systemd/system/uv.service.d/20-h264.conf`; the original service description
and checkout diff are saved under
`~/.local/share/rover-api/deployment-20261103/`. To return to the original service,
remove that override, run `sudo systemctl daemon-reload`, then
`sudo systemctl restart uv.service`.

Rolling-shutter ("jello") correction can be enabled per camera; it runs on the
rover before encoding, using the BNO055 gyro or the video's own optical flow. See
[README_ROLLING_SHUTTER.md](README_ROLLING_SHUTTER.md).

For local validation with native GStreamer installed:

```sh
uv sync --group test
uv run --group test python -m unittest discover -s tests
```

The `test` group includes aiortc only as a real ICE/RTP test receiver. It is no
longer the runtime video encoder. Hardware-free tests use native test video when
available, and report a skip when native dependencies are missing.


## ROS2 Integration

This endpoint integrates with ROS2 through rosbridge WebSocket for bidirectional communication.

### Prerequisites

1. **ROS2 Installation** (Humble, Iron, or Jazzy recommended)
2. **rosbridge_suite** package:
   ```sh
   sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
   ```

### Starting rosbridge_server

Before starting the API server, launch rosbridge_server in a separate terminal:

```sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

By default, rosbridge runs on `ws://localhost:9090`. You can configure this with environment variables (see below).

### Environment Configuration

Create a `.env` file in the project root (optional, defaults provided):

```env
ROSBRIDGE_HOST=localhost
ROSBRIDGE_PORT=9090
ROSBRIDGE_RETRY_INTERVAL=5
ROSBRIDGE_MAX_RETRIES=10
```

### ROS Topics Used

The API publishes and subscribes to the following standard ROS2 topics:

- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands (published)
- `/odom` (nav_msgs/Odometry) - Odometry data (subscribed)

### Testing the Integration

1. **Start rosbridge_server**:
   ```sh
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. **Start the API server**:
   ```sh
   uv run main.py
   ```

3. **Check ROS connection status**:
   ```sh
   curl http://localhost:6767/api/ros/status
   ```

4. **Send a velocity command** (move forward):
   ```sh
   curl -X POST http://localhost:6767/api/nav/ros/cmd_vel \
     -H "Content-Type: application/json" \
     -d '{"linear_x": 1.0, "angular_z": 0.0}'
   ```

5. **Stop the rover**:
   ```sh
   curl -X POST http://localhost:6767/api/nav/ros/cmd_vel/stop
   ```

6. **Subscribe to odometry and get data**:
   ```sh
   # Subscribe first
   curl -X POST http://localhost:6767/api/nav/ros/odom/subscribe
   
   # Then get latest data
   curl http://localhost:6767/api/nav/ros/odom
   ```

### Monitoring from ROS side

You can monitor the topics from ROS2 command line:

```sh
# List active topics
ros2 topic list

# Echo velocity commands being published
ros2 topic echo /cmd_vel

# Manually publish to test (the example you provided)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```


## Reading/Accessing/Using
Always perform POST requests to specific endpoints. All supported endpoints will be provided by the *available* endpoint.

### End points
* **/api/nav/**: Backend endpoint(s) for navigation subsystem
* **/api/nav/ros/**: ROS2 integration for navigation (cmd_vel, odom)
* **/api/dgt/**: Backend endpoint(s) for diagnostics
* **/api/sci/**: Backend endpoint(s) for science subsystem
* **/api/arm/**: Backend endpoint(s) for arm subsystem
* **/api/o/**: Additional endpoint(s) provided
* **/api/ros/**: ROS bridge status and control endpoints

### New ROS Endpoints

**Navigation ROS Endpoints** (`/api/nav/ros/`):
- `POST /cmd_vel` - Publish velocity commands to move the rover
- `POST /cmd_vel/stop` - Emergency stop (publish zero velocities)
- `POST /odom/subscribe` - Subscribe to odometry topic
- `GET /odom` - Get latest odometry data
- `POST /publish` - Publish custom messages to any topic

**ROS Status Endpoints** (`/api/ros/`):
- `GET /status` - Get rosbridge connection status
- `POST /connect` - Manually connect to rosbridge
- `POST /disconnect` - Disconnect from rosbridge

## Developer/Contributor documentation

Some general guidelines:
* **Always fork the repository and make a pull request**
* Do not write/support *GET* requests for backend unless its required (eg: downloads).
* Type annotations are always better to be provided. Support complex type annotation by adding them to [types files](app/py_types.py) which contains an example.
* Always update available.py files if any new endpoint is added
* Follow the directory structure and update the [main server script](app/__init__.py)
* Provide minimum amount of comments to explain ***why your code exists*** rather than **how it works**. Its easy to understand that certain part of code is a loop as opposed to understanding why we need the loop.

### Directory Structure
Most directory names are self explanatory but here is a quick reference,
* [Arm Directory](app/api/arm/): Backend Files for the arm subsystem.
* [Diagnostics Directory](app/api/diagnostics/): Backend Files for running diagnostics/check-ups on rover.
* [Navigation Directory](app/api/navigation/): Backend Files for the navigation subsystem and most of scouting modules.
* [Science Directory](app/api/science/): Backend Files for the science subsystem and any analytical modules.
* [ROS Directory](app/ros/): ROS2 integration via rosbridge WebSocket.

* [Others Directory](app/api/others/): Additional endpoints for debugging, logging etc.



## BNO055 IMU mounting and calibration (2 November 2026)

### Chassis reference and measured IMU position

The operator chose the geometric centre of the rigid rectangular component chassis
as the proposed `base_link` origin. This is a fixed body reference, independent of
the centre of mass. The suspension arms and wheels are excluded from this box.
These measurements are operator-supplied and have not been checked against the
onboard URDF, TF tree, or odometry configuration.

| Chassis dimension | Measurement | Distance from centre to either face |
| --- | --- | --- |
| Length, front to back | 74 cm | 37 cm |
| Width, left to right | 50 cm | 25 cm |
| Height, bottom to top | 22.5 cm | 11.25 cm |

The body axes are X forward, Y left, and Z up, viewed from the rover's perspective.
The IMU is forward and left of the centre, at exactly the same height.

| IMU offset from the proposed `base_link` origin | Centimetres | Metres for ROS |
| --- | --- | --- |
| X, forward | +22 | +0.220 |
| Y, left | +7.5 | +0.075 |
| Z, same height | 0 | 0.000 |

These offsets place the IMU 15 cm behind the front face, 59 cm ahead of the rear
face, 17.5 cm inward from the left face, and 32.5 cm inward from the right face.
Its height is 11.25 cm above the chassis bottom, not above the ground.

The intended `base_link` to `imu_link` translation is therefore
`xyz="0.220 0.075 0.000"`. The mounting rotation remains provisional. The chassis
height comparison below estimates nonzero roll and pitch, superseding the initial
photo-based zero-rotation estimate. No transform has been configured by this
documentation change.

### Mounting orientation evidence and firmware inspection

The operator's top-down mounting photo labels FR as front-right and FL as
front-left. The front is toward image-left, the rover's left toward image-bottom,
and up toward the camera. The component side of the IMU board faces up.
The operator corrected only the IMU's position from front-right to front-left.
The photo's FR/FL labels remain correct, so this position correction does not
change the inferred axis directions or the recorded native-sensor readings.

The board matches the layout and logo in the
[7Semi ES-12149 manual](https://7semi.com/content/datasheets/7Semi%20ES-12149_BNO055_1.0.pdf),
page 4. Its mounting photo shows the board rotated approximately 180 degrees in
the image plane relative to that reference. The power and I2C header is toward
image-top, and the PS0/PS1/INT/ADD header is toward image-bottom. The chip and
surrounding component layout identify its pin-1 corner toward image-bottom-left.

The [Bosch BNO055 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf),
revision 1.8, section 3.4, defines the native axes relative to the chip's pin-1
corner. Matching that definition to the board gives this inferred mapping:

| Native IMU axis | Direction in the mounting photo | Rover body direction |
| --- | --- | --- |
| +X | Image-left | Forward, +X |
| +Y | Image-bottom | Left, +Y |
| +Z | Out of the component side toward the camera | Up, +Z |

This supports nominal horizontal-axis alignment. Photos do not measure mounting
tilt or establish the sensor's actual output signs. The chassis height comparison
below refines the initial zero-rotation estimate, but does not validate a TF setting.

The local PID checkout was inspected on 2 November 2026 at commit `a844ee3` on
`main`, with a clean working tree. `include/AsyncBno055.h` resets the BNO055,
sets `UNIT_SEL=0x80`, and makes no writes to the axis-map registers. Bosch's reset
mapping is `AXIS_MAP_CONFIG=0x24` and `AXIS_MAP_SIGN=0x00`, with no axis swaps or
sign inversions. The saved calibration JSON records the same mapping.
`include/ImuCalibration.h` checks those values in its stored profile metadata.
The main driver restores only the 22 offset bytes at register `0x55`.
It does not read back or continuously monitor the axis-map registers.

The driver decodes acceleration and angular velocity in register X/Y/Z order.
`src/BasicMotorControl.cpp`, function `updateImu()`, copies those components and
the native quaternion directly into `/imu/data`, with no body-axis rotation or
world-frame conversion. `UNIT_SEL=0x80` does not itself establish ROS ENU
compatibility.

The existing host check, `python3 test/verify_imu.py` in the PID checkout, passed
during this inspection. A read-only SSH observation in ROS domain 0 received no
IMU messages in either of two six-second windows. That observation cannot verify
gravity direction, axis signs, or the flashed firmware version. No sensor reset,
firmware upload, service change, or motor command was performed.

A subsequent read-only transport inspection on 2 November initially found both
micro-ROS agent services running, but unable to open their serial devices:

- `ros-node1.service` waits for `/dev/chassis-teensy`. Its journal reports
  `Serial port not found`, error 2.
- `ros-node2.service` waits for `/dev/ttyACM1` and reports the same error.
- The onboard computer exposes no `/dev/ttyACM*`, `/dev/ttyUSB*`, or serial by-id
  devices. USB enumeration also shows no Teensy device, including a bootloader.
- A fresh ROS observation received no IMU messages and discovered no publisher
  for `/imu/data`. A topic name can appear because the diagnostic subscribes to
  it, so the topic list alone does not establish a working sensor connection.

After the operator reconnected the Teensy, the onboard computer detected its USB
serial interface as `/dev/ttyACM0`, with `/dev/chassis-teensy` pointing to that
device. The existing micro-ROS agent connection recovered without a service
restart or firmware upload. The ROS node `teensy_motor_node` published `/imu/data`
with frame `imu_link`. GPS and motor RPM topic names were also visible, but their
measurements were not validated in this check.

A six-second read-only observation received 179 IMU messages, with a timestamp
rate of approximately 29.69 Hz. The stationary-state measurements were:

| Quantity | Measured value |
| --- | --- |
| Mean acceleration X, Y, Z | -0.736, -0.780, +9.622 m/s² |
| Acceleration vector magnitude | 9.681 m/s² |
| Mean angular velocity X, Y, Z | -0.000347, +0.000049, -0.000116 rad/s |
| Gravity-based roll estimate | -4.63 degrees |
| Gravity-based pitch estimate | +4.36 degrees |
| Total tilt away from level | 6.36 degrees |

The tilt estimate assumes the rover is stationary and the proposed body-axis
mapping is correct. Near-zero mean angular velocity supports the stationary
assumption. The sensor is not reporting a level orientation in this observation.
These angles include chassis inclination and sensor mounting inclination. A level
reference on the chassis is required to separate the two, so these readings are
not a replacement for the candidate fixed-transform rotation.

The observed orientation covariance marker was `0.0`, which this firmware uses
when all calibration levels are 3. It still means the covariance is unknown, not
zero measurement uncertainty. Positive Z acceleration supports the component-side
up interpretation, but this stationary observation does not establish horizontal
axis signs or quaternion ENU compatibility. No motor commands were sent.

### Chassis inclination and estimated mounting tilt

The operator supplied these heights for corresponding corners of the rectangular
metal chassis on 2 November 2026:

| Corner | Height above the ground |
| --- | --- |
| Front-left | 37 cm |
| Front-right | 37 cm |
| Rear-left | 38 cm |
| Rear-right | 38 cm |

Assuming level ground and measurements to matching points on the chassis, the
rear is 1 cm higher than the front. Across the 74 cm chassis length, the pitch is
`asin(1 / 74)`, approximately +0.77 degrees in the proposed ROS body frame.
The chassis is nose-down. Equal left/right heights indicate approximately zero
roll within the measurement precision. These heights do not determine yaw.

A fresh six-second observation received 179 IMU messages at approximately 29.74 Hz.
Mean acceleration was X=-0.747, Y=-0.781, Z=+9.623 m/s², with near-zero mean gyro
readings. The stationary gravity-based estimates were roll=-4.64 degrees and
pitch=+4.43 degrees. Assuming the photo-based horizontal-axis alignment and no
relative mounting yaw, separating the chassis pitch gives:

| Rotation | Chassis relative to level | IMU relative to level | Estimated mounting relative to chassis |
| --- | --- | --- | --- |
| Roll | Approximately 0 degrees | -4.64 degrees | Approximately -4.64 degrees |
| Pitch | +0.77 degrees | +4.43 degrees | Approximately +3.65 degrees |

The candidate parent-to-child mounting rotation is therefore approximately
roll=-0.08095 rad and pitch=+0.06372 rad, with yaw still unverified. This estimate
assumes stationary readings, accurate corner measurements, and the same chassis
pose during measurement. Accelerometer bias also affects the result. Gravity
alone cannot determine mounting yaw. The rotation composition reproduces the
measured acceleration direction under the assumed zero relative yaw.

The fixed transform must describe sensor mounting, so the real +0.77 degree
chassis pitch must remain in the estimated robot pose. No correction has been
applied to firmware or TF. Physical axis tests, quaternion-convention validation,
and a controlled mounting-orientation check remain necessary before fusion.

### Onboard ROS and quaternion-convention inspection

A read-only inspection on 2 November 2026 found the micro-ROS Teensy node,
rosbridge, ROS API nodes, and `rover_kinematics_node` running. `/imu/data`,
`/gps/fix`, and `/motor_rpms` each had a publisher. No publisher was present for
`/tf` or `/tf_static`, and no odometry topic, EKF node, robot-state publisher node,
or Nav2 node was discovered. The diagnostic's own TF subscriptions can create
topic names without any transforms being published.

The home-workspace search found a custom `rover_sensor_pipeline` package with
`RoverEKF.hpp` and `sensor_ingestion_node.cpp`. It is existing work, not an empty
navigation setup, but its node was not running. The source subscribes to wheel
RPMs, GPS, and IMU angular velocity Z, then publishes `/ekf/odom` with frame `map`
and child `base_link`. It does not use the IMU quaternion or apply its mounting
rotation, and it contains no TF broadcaster. Its zero initial heading is not an
established ENU heading. The active kinematics node converts commands to wheel
RPM targets and does not provide measured wheel odometry.

`robot_state_publisher` and joint-state publisher packages are available in the
sourced ROS environment. `robot_localization`, Nav2, and SLAM Toolbox were not
listed there. The searched workspaces contain arm URDFs, but no central six-wheel
rover URDF or localization configuration was found. The custom pipeline needs
review before reuse or replacement. This inspection did not start it or alter
the existing command path.

A ten-second stationary capture received 298 IMU messages at 29.7 Hz. Quaternion
norms were 1.0, timestamps increased monotonically, and all three covariance
matrices remained zero. The mean message timestamp was about 11 ms ahead of the
onboard wall clock at receipt, so clock alignment needs a separate check.

The quaternion was tested as a rotation from sensor coordinates into a Z-up
world. Its predicted stationary specific-force direction, `R(q)^T * [0,0,1]`,
agreed with normalized measured acceleration to a mean angular difference of
0.17 degrees. The inverse interpretation differed by 11.51 degrees, and using a
Z-down world with the same sensor axes differed by 179.83 degrees. This supports
the current component order and Z-up sensor-to-world interpretation at this pose.
It does not replace known-axis movement tests or measure compass accuracy.

The [Bosch quick-start guide](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bno055-an007.pdf),
page 4, defines its Android rotation-vector reference with X east, Y north, and
Z up. It points to the Android rotation-vector definition, whose
[reference frame is ENU](https://developer.android.com/develop/sensors-and-location/sensors/sensors_motion#sensors-motion-rotate).
The inspected firmware selects Android format with `UNIT_SEL=0x80`, uses NDOF
fusion, and assigns the native w/x/y/z quaternion to the matching ROS fields.
Together with the live gravity check, this supports treating the documented
quaternion convention as ENU. No NED-to-ENU rotation, axis swap, or quaternion
conjugation is justified by the current evidence.

Bosch's Euler compass heading is a separate representation, with zero at north
and clockwise-positive values. That convention must not be assigned directly to
ROS yaw, which is east-zero and counter-clockwise-positive. The firmware publishes
quaternions, not those Euler heading registers. The observed mathematical yaw was
approximately -50.9 degrees. Under the documented ENU convention, the sensor-X
magnetic bearing would be approximately 140.9 degrees, but no independent compass
reference was available at that inspection. The sensor-X bearing is not a measured
chassis bearing until the mounting yaw is confirmed.

The operator subsequently supplied a phone compass screenshot showing 138 degrees,
southeast, and confirmed that the phone's top edge pointed toward the rover's
front. The screenshot does not show whether the app uses magnetic or true north.
A fresh six-second capture received 179 messages at approximately 29.69 Hz. Its
latest quaternion gave a mathematical yaw of approximately -50.85 degrees,
equivalent to a sensor-X compass bearing of approximately 140.85 degrees under
the documented ENU convention. The difference from the phone is about 2.85 degrees.
The phone's 138 degree bearing corresponds to ROS ENU yaw of -48 degrees if both
use the same north reference.

Assuming the rover did not rotate between the screenshot and capture, and the
mounting yaw is approximately zero, this is a coarse heading cross-check consistent
with ENU. It provides no evidence for an additional 90 or 180 degree yaw rotation.
Phone alignment, compass accuracy, local magnetic interference, and the unknown
north setting limit the comparison. This single observation is not a basis for
applying a 2.85 degree heading correction or claiming compass accuracy. Dynamic
yaw signs and motor interference remain untested. No conversion was applied.

Stationary gravity is unchanged by a rotation of the world reference about Z.
It therefore cannot distinguish east/north alignment from another Z-up yaw
reference or verify the sign of yaw changes. An independent magnetic bearing can
check heading at the current pose without moving the rover. A later controlled
rotation must still check counter-clockwise-positive yaw and gyro signs. True
north additionally requires accounting for magnetic declination. This inspection
leaves the firmware and TF unchanged and does not establish readiness for fusion.

### Firmware and calibration handoff

The supplied firmware handoff records the following results. These are previous
test results, not new hardware verification during this documentation update.

- Firmware repository: [TeamOdysseyHQ/PID](https://github.com/TeamOdysseyHQ/PID).
  Local checkout: `/home/muthibazz/Projects/Embeded_Projects_Repos/PID`, branch `main`.
- Teensy 4.1 uses asynchronous I2C, BNO055 address `0x28`, SDA pin 18, and SCL pin 19.
- Firmware publishes `/imu/data` with frame `imu_link`. Hardware testing measured
  approximately 29.4 Hz. Motor RPM and GPS topics also published, but GPS fix
  validity was not established.
- Calibration reached SYS=3, GYR=3, ACC=3, and MAG=3. A checksummed profile occupies
  EEPROM bytes 512–543. Existing motor PID settings in bytes 0–75 were verified
  unchanged.
- Main firmware restores and reads back the sensor offsets at startup. After a
  restart and some rotation, 60 fully calibrated IMU messages were received.
- The handoff reported passing build and host tests, with uncommitted changes in
  `include/AsyncBno055.h`, new `include/ImuCalibration.h`,
  `src/BasicMotorControl.cpp`, and `test/imu_async.cpp`. The later local inspection
  above found these files tracked in a clean checkout. The onboard deployment
  status remains unverified.
- Firmware backups, `imu-calibration.json`, and EEPROM dumps are under
  `.scratch/hardware-test/` in the PID checkout. The handoff calls for transferring
  the changes to the onboard checkout before relying on its firmware version.

### Remaining integration checks

The photo-based axis mapping still needs a physical check of acceleration and
gyro signs, including forward/backward and left/right responses. A stationary,
level reading with gravity included can check the vertical axis, but cannot
distinguish horizontal rotations. Any physical tilt or rotation test needs to be
arranged with the operator before moving the rover or sensor.

The firmware publishes native BNO055 orientation. Manufacturer documentation and
the stationary consistency check support the ENU convention, but physical heading,
yaw-change signs, and mounting axes remain unverified. A mounting transform alone
does not establish world-convention compatibility.

The onboard inspection above found an inactive custom EKF prototype and no active
TF or odometry pipeline. The rover's body model, measured wheel odometry, mounting
TF, and navigation integration still need implementation or completion. Known-angle
rotations, gyro signs, independent heading, and motor interference remain to be tested.
Realistic covariances are required before sensor fusion. Full calibration status
does not establish navigation accuracy or resistance to motor interference.
Motor commands require an arranged, controlled test.

## Camera endpoint performance follow-up (4 October 2026)

Camera endpoints no longer require a hard-coded debug directory and frame capture
no longer prints two messages for every read. Blocking camera control, probing and
MJPEG setup handlers use FastAPI's synchronous worker pool. Camera start/stop
operations are serialized, and reads/releases use a lock per physical camera so
independent cameras can still run concurrently. Still captures include microseconds
in their filenames to avoid overwriting rapid captures. Generic detected `videoN`
cameras now work with the WebRTC and WebSocket routes as well as MJPEG.

Validation: `python -m unittest discover -s tests` covers routes with fake cameras,
concurrent starts, capture/stop ownership, generic streaming validation and existing
signaling/adaptation regressions. Hardware ICE validation is skipped when native
media dependencies are unavailable. Physical device open/read latency and aggregate
FPS with multiple viewers still need measurements on the rover; the locks protect
hardware ownership and do not create additional capture capacity.

## API fixes and performance follow-up (9 October 2026)

The changes prioritize latency, then response time, then bandwidth. Blocking
microscope, serial, ROS connection, diagnostic and report handlers now run in
workers. Camera and microscope WebSocket capture/JPEG encoding also run in workers,
and stream pacing accounts for capture, encoding and send time. Where supported by
the camera backend, capture buffers are requested at one frame. WebSocket sends
that stall for two seconds close the viewer connection; clients can reconnect.

Cached science sensor data returns immediately instead of sleeping for half a
second. ROS WebRTC capture reuses decoded images while the cached message is
unchanged. ROS MJPEG sends each new cached message once, avoiding repeated JPEG
encoding and transmission when source updates are slower than the requested FPS.
Requested FPS is a delivery ceiling; default JPEG quality stays the same.

Captures and reports share `EXPEDITION_BASE_DIR` and
`EXPEDITION_PROCESSED_DIR`. The default username is intentionally
`administratror`. The processed directory defaults to a sibling of the configured
unprocessed directory. Export these environment variables to override them.
Expedition IDs are validated, and report templates are resolved from this checkout.
Reports recognize capture names with and without microseconds and mark expeditions
processed only after successful compilation.

JSON updates use a shared thread/process lock and replace complete snapshots.
Waypoint names and IDs are assigned inside that transaction. Uploads copy in
chunks, and rapid captures/uploads use microseconds to avoid overwrites. Failed
image saves return an error. Serial lifecycle changes are serialized with writes,
and ROS subscription creation is serialized. Timed-out microscope opens release
late results instead of leaking devices or queuing repeated open attempts.

Other fixes include padded ROS image rows and byte-array payloads, stream parameter
validation, WebSocket client-limit/cleanup races, stable camera IDs, multiline
diagnostic results, the shadowed science download-by-path route, and the navigation
report download URL. Report downloads are limited to files in configured report
directories. Shutdown releases camera and serial resources.

Validation: `uv run python -m unittest discover -s tests` passes 64 tests, including
actual FastAPI routes, concurrent thread/process storage updates, stream cleanup,
ROS image decoding and real local aiortc ICE/RTP transport. The full app imports,
serves `/`, and generates its OpenAPI schema. All application and test files parse
with Python 3.10 syntax; `git diff --check` passes.

Synthetic comparisons against commit `575f91a` on the Windows development host:

| Scenario | Before median | After median |
| --- | ---: | ---: |
| Cached science sensor handler | 500.588 ms | 0.028 ms |
| Decode an unchanged cached 1280x720 BGR ROS image | 9.053 ms | 0.010 ms |
| Dispatch a simulated 100 ms ROS connect, confirm it started, then complete a health probe | 101.685 ms | 1.231 ms |

Sensor timings use five baseline and thirty updated calls. Image timings use
thirty calls after warming the updated cache. Health timings use five calls per
version after warming the worker pool. These isolate the changed paths; they are
not end-to-end rover measurements. Tests mock physical devices and Typst
compilation. Physical capture latency, multi-viewer FPS, actual report compilation,
and rover-network bandwidth still require validation on the rover.
