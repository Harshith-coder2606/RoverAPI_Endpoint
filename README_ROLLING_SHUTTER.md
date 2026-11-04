# Rolling-shutter ("jello") correction

Rover cameras such as the Logitech C270 use rolling shutters. The sensor reads
rows from top to bottom, so when the camera rotates, each row sees the scene from
a slightly different angle. The picture shears and wobbles: the "jello" effect.

This feature removes that distortion on the rover computer. Each frame is
resampled before H.264 encoding, so every dashboard viewer receives corrected
video. BNO055 gyro data never leaves the rover computer.

> **Status (4 November 2026):** implemented and tested, but **not deployed**. It
> runs only in a test instance (port 6768). See [Status](#status) and
> [Remaining work](#remaining-work).

## How it works

```
BNO055 ──/imu/data──▶ rosbridge (localhost:9090)
                          │  RoverAPI ImuRelay (app/media/imu_relay.py)
                          ▼
camera ──▶ v4l2src ──▶ jpegdec ──▶ I420 frame ──▶ [ correction ] ──▶ x264enc ──▶ WebRTC viewers
                                        │               ▲
                                        └─ optical flow ┘   (app/media/dejello.py, inside gst_worker.py)
```

The pipeline splits each frame at raw I420 (an `appsink`/`appsrc` pair). While
correction is off, frames pass through unchanged.

Row `y` of a frame is exposed at `t_mid + s(y) · t_readout`, where
`s(y) = y/H − 0.5`. If the image moves at velocity `v` (pixels per second),
row `y` is displaced by `s(y) · t_readout · v`. The correction samples each
output pixel from `p + s(y) · t_readout · v`, plus a rotation term for camera
roll. The result is every row as it would look at the middle-row time.

### Motion sources

| Source | Image velocity comes from | Requirements |
| --- | --- | --- |
| `imu` | BNO055 gyro rate, mapped through a calibration | Camera **rigidly fixed to the same body as the IMU** (the chassis); one-time calibration |
| `video` | Optical flow on the frames themselves | Nothing extra; works without an IMU |
| `auto` (default) | `imu` once calibrated and gyro data is current, otherwise `video` | — |

**IMU calibration** runs automatically while correction is on. Optical flow on
the raw frames is regressed against integrated gyro rotation. The fit gives:

- a 3×3 **gain** from IMU rotation to image motion (x, y, roll), which absorbs
  the IMU-to-camera axis mapping, signs and focal length;
- the **time offset** of the middle row relative to the capture timestamp;
- the **readout time**, from differences between top, middle and bottom bands.

Gyro stamps (micro-ROS epoch, synchronized to the rover clock) and frame capture
times (`time.time_ns()` in the worker) use the same clock, so no cross-machine
clock estimate is involved. The calibration is saved per camera in
`~/.local/share/rover-api/dejello/<device>.json`.

**Readout time**, in order of preference:

1. a manual value;
2. the IMU-fitted value;
3. an estimate from video alone;
4. 0.8 × the frame period.

The video estimate regresses the top-vs-bottom band difference against the
rate of change of motion. It reads about 15% high when motion is close to the
analysis rate limit.

## Using it

### Dashboard (rover_ui)

On a named camera (for example **science**), with the stream live:

1. Tick **IMU de-jello**. A status panel appears at the top right.
2. Choose **source**: `auto`, `IMU` or `video only`. The `→` label shows which
   source corrected the latest frame (`off` means it was shown as captured).
3. **Readout:** the slider overrides it; **auto readout** clears the override.
4. **Recalibrate IMU** discards the stored calibration.

### API

`GET /api/nav/cameras/{name}/dejello` returns the status. A camera that isn't
running returns 404. A camera that doesn't support correction returns 409.

`POST /api/nav/cameras/{name}/dejello` takes a JSON body. All fields are
optional:

| Field | Type | Effect |
| --- | --- | --- |
| `enabled` | bool | Turn correction on or off. The gyro relay runs only while it is on. |
| `source` | `"auto"` \| `"imu"` \| `"video"` | Motion source |
| `readout_ms` | number, 0–100 | Manual readout override |
| `reset_readout` | bool | Clear the manual override |
| `reset_calibration` | bool | Delete the stored IMU calibration |

```sh
curl -X POST localhost:6767/api/nav/cameras/science/dejello \
  -H 'Content-Type: application/json' -d '{"enabled": true, "source": "video", "readout_ms": 13}'
```

Main status fields:

- `active_source`: which source corrected the latest frame.
- `calibrated`, `calibration` (`fit`, `offset_ms`, `readout_ms`, `roll_fitted`).
- `last_fit.reason`: why the latest IMU fit was rejected.
- `gyro.rate_hz`, `gyro.age_ms`.
- `readout_ms`, `readout_source`.
- `stats`:
  - `corrected`;
  - `no_gyro`, `no_flow` (frames shown uncorrected);
  - `warp_px`: largest recent shear;
  - `correct_ms`, `video_flow_ms`: per-frame cost.

### Calibrating the IMU source

The gyro must rotate together with the camera. The BNO055 is fixed to the
chassis (see *BNO055 IMU mounting and calibration* in [README.md](README.md)),
so **the camera must be rigidly mounted on the chassis**.

1. Mount the camera rigidly. Start it, enable correction, and set
   `source: auto` or `IMU`.
2. **Move the whole rover**, not the camera: rock it side to side and front to
   back, lift and tilt one end, or drive slowly over bumps. Varied, changing
   motion is best: quick wobbles also let the fit measure the readout time.
3. After 20–30 s the panel shows `fit R² …`. A fit is accepted when the gyro
   explains at least half of the image motion (R² ≥ 0.5).

If the panel keeps showing *"Not enough rotation yet"* while the image is
clearly moving, the IMU is not moving with the camera. On 4 November the gyro
recorded about 0.0001 rad per frame while a loose camera was moved by hand,
which is noise level.

## Status

Verified:

- Unit tests (`tests/test_dejello.py`, 13 tests) pass on the laptop and on the
  rover's system Python (numpy 1.21, OpenCV 4.5). They cover:
  - recovery of a known gain, offset and readout from a 30 Hz gyro;
  - rejection of a still camera;
  - the video readout estimator;
  - optical flow per band;
  - removal of a synthetic rolling-shutter shear;
  - exactness of the warp tables;
  - video-only correction of a synthetic pan;
  - source selection.
- On the rover: the worker held 30 FPS with correction on. Correction costs
  about **6 ms per 1280×720 frame**, using strip `warpAffine` calls on an
  8-thread pool. Video-source optical flow adds about 15 ms per analysed frame.
- Live C270: `/imu/data` reached the worker at 29.5 Hz. Video-only mode
  corrected live frames.
- The C270's readout measured from video is about **11–19 ms** (consistent
  between browser and rover measurements).

Not yet verified:

- IMU calibration on a rigidly mounted camera.
- A visual before/after comparison of either source on the real camera.
- The Kreo Owl at 60 FPS.

## Limitations

- **Only rotation is corrected.** Sideways or forward movement and parallax are
  not corrected.
- **Video source:**
  - it lags the motion by about half a frame;
  - large moving objects or nearby parallax can mislead it;
  - frames with fewer than 30 tracked points pass through uncorrected;
  - the warp is capped at 80 px.
- **IMU rate:** the firmware samples the BNO055 every 33 ms (about 30 Hz), so
  vibration above about 15 Hz is not resolved.
- **Motion blur is not removed.** In indoor light the C270 delivers about
  15 FPS with long exposures, and blur is often the larger artefact.
- **Strong roll:** with very strong roll (80°/s), the strip approximation leaves
  up to about 0.33 px error at strip corners. Without roll the warp is exact.

## Files

| File | Role |
| --- | --- |
| `app/media/dejello.py` | Gyro buffer, optical flow tracker, calibration fit, readout estimators, strip-warp remapper, per-camera `Dejello` state |
| `app/media/imu_relay.py` | Subscribes `/imu/data` once through the ROS manager; forwards samples to camera workers |
| `app/media/gst_worker.py` | Splits the pipeline at I420 when `--dejello-state` is given; `imu` and `dejello` commands |
| `app/media/gstreamer.py` | Passes the calibration path for `/dev/*` cameras; `dejello()` control; gyro forwarding while enabled |
| `app/api/navigation/camera.py` | `GET/POST /cameras/{name}/dejello` |
| `tests/test_dejello.py` | pytest tests (plain asserts; run with pytest, not unittest discovery) |

## Testing

```sh
# Laptop or any machine with numpy, OpenCV and pytest
python -m pytest tests/test_dejello.py

# Rover: the worker's own interpreter (numpy 1.21, OpenCV 4.5)
/usr/bin/python3 -m pytest tests/test_dejello.py
```

## Deployment

The current test instance runs from `~/rover-dejello-test` on the rover:

```sh
~/Projects/RoverAPI_Endpoint/.venv/bin/python -m uvicorn main:app --host 0.0.0.0 --port 6768
```

Port 6768 is not reachable from the laptop. The dashboard reached it through an
SSH tunnel to `localhost:6768`. Only one process can open a camera at a time, so
stop the camera on the live service (port 6767) before starting it on the test
instance.

To deploy like the existing H.264 release (needs sudo and a service restart):

1. Export the commit to `~/.local/share/rover-api/releases/<commit sha>`. Link
   its `storage` directory as the existing release does.
2. Save the current `/etc/systemd/system/uv.service.d/20-h264.conf` under a new
   `~/.local/share/rover-api/deployment-<date>/` folder.
3. Point the drop-in's `WorkingDirectory` at the new release, then run
   `sudo systemctl daemon-reload` and `sudo systemctl restart uv.service`.
4. **Rollback:** restore the saved drop-in, reload and restart.

Correction stays off after deployment until someone enables it per camera, so
deploying does not change existing video.

## Remaining work

1. Calibrate the IMU source with the camera rigidly mounted on the chassis, and
   record the fit.
2. Compare corrected and uncorrected video visually for both sources.
3. Deploy (above), with the team's agreement.
4. Optional firmware change: sample the BNO055 every 10 ms (`nextSampleAt_ = now + 33`
   in the PID repo's `include/AsyncBno055.h`) for about 100 Hz gyro data. This
   needs a reflash.
5. Optional camera setting: a shorter fixed exposure on the C270, to reduce motion
   blur and restore its frame rate.
