"""Bounded control bridge to a native capture/encode process per physical source."""
import base64
from concurrent.futures import Future
import json
import logging
import mmap
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from uuid import uuid4

logger = logging.getLogger(__name__)
# Per-camera rolling-shutter calibrations stay on this computer.
DEJELLO_DIR = Path(os.environ.get('ROVER_DEJELLO_DIR', Path.home() / '.local/share/rover-api/dejello'))


class GStreamerSource:
    def __init__(self, device, width=1280, height=720, fps=60, pixel_format='MJPG', capture_fn=None):
        self.width, self.height, self.fps = width, height, fps
        self.device = device
        self._pending = {}
        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._ready = Future()
        self._frame_file = None
        self._frame_map = None
        self._feeder = None
        self._capture_fn = capture_fn
        self._last_frame = None
        self._last_sent = None
        self._imu_forwarding = False
        args = [os.environ.get('GSTREAMER_PYTHON', '/usr/bin/python3'), '-u',
                str(Path(__file__).with_name('gst_worker.py')), '--device', device,
                '--width', str(width), '--height', str(height), '--fps', str(fps),
                '--pixel-format', pixel_format]
        if device.startswith('/dev/') and capture_fn is None:
            args += ['--dejello-state', str(DEJELLO_DIR / f'{Path(device).name}.json')]
        if capture_fn is not None:
            self._frame_file = tempfile.NamedTemporaryFile(prefix='rover-frame-', dir='/dev/shm')
            self._frame_file.truncate(width * height * 3)
            self._frame_map = mmap.mmap(self._frame_file.fileno(), width * height * 3)
            args += ['--frame-file', self._frame_file.name]
        env = dict(os.environ)
        try:
            self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.PIPE, text=True, bufsize=1, env=env)
        except OSError:
            if self._frame_map is not None:
                self._frame_map.close()
                self._frame_file.close()
            raise
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._error_reader = threading.Thread(target=self._errors, daemon=True)
        self._reader.start()
        self._error_reader.start()
        if capture_fn is not None:
            self._feeder = threading.Thread(target=self._feed, daemon=True)
            self._feeder.start()
        try:
            self._ready.result(timeout=12)
        except Exception:
            self.stop()
            raise

    def _read(self):
        try:
            for line in self.process.stdout:
                try:
                    message = json.loads(line)
                except ValueError:
                    logger.debug('GStreamer stdout: %s', line.rstrip())
                    continue
                with self._lock:
                    future = self._ready if message.get('id') == 'ready' else self._pending.pop(message.get('id'), None)
                if future and not future.done():
                    if message.get('error'):
                        future.set_exception(RuntimeError(message['error']))
                    else:
                        future.set_result(message)
        finally:
            with self._lock:
                futures = [self._ready, *self._pending.values()]
                self._pending.clear()
            for future in futures:
                if not future.done():
                    future.set_exception(RuntimeError('GStreamer source exited; check its media packages and camera connection'))

    def _errors(self):
        for line in self.process.stderr:
            logger.warning('GStreamer %s: %s', self.device, line.rstrip())

    def request(self, cmd, timeout=15, **values):
        request_id = uuid4().hex
        future = Future()
        with self._lock:
            if self.process.poll() is not None:
                raise RuntimeError('GStreamer source is not running')
            self._pending[request_id] = future
            try:
                self.process.stdin.write(json.dumps({'id':request_id, 'cmd':cmd, **values})+'\n')
                self.process.stdin.flush()
            except (OSError, ValueError):
                self._pending.pop(request_id, None)
                raise
        try:
            return future.result(timeout=timeout)
        except BaseException:
            with self._lock:
                self._pending.pop(request_id, None)
            # Reclaim a viewer even if HTTP negotiation timed out or was abandoned.
            if cmd == 'offer' and self.process.poll() is None:
                try:
                    self.request('close', timeout=3, peer_id=values['peer_id'])
                except Exception:
                    logger.exception('Failed to release timed-out viewer')
            raise

    def _send(self, cmd, **values):
        """Fire-and-forget control message (no reply expected)."""
        with self._lock:
            if self.process.poll() is not None:
                return
            try:
                self.process.stdin.write(json.dumps({'id': cmd, 'cmd': cmd, **values}) + '\n')
                self.process.stdin.flush()
            except (OSError, ValueError):
                pass

    def _forward_imu(self, sample):
        self._send('imu', samples=[sample])

    def dejello(self, enabled=None, readout_ms='keep', reset=False, source=None):
        """Configure IMU rolling-shutter correction; gyro samples flow only while it is enabled."""
        from app.media.imu_relay import relay
        values = {'reset': reset}
        if enabled is not None:
            values['enabled'] = enabled
        if readout_ms != 'keep':
            values['readout_ms'] = readout_ms
        if source is not None:
            values['source'] = source
        result = self.request('dejello', timeout=5, **values)
        if result.get('enabled') and not self._imu_forwarding:
            relay.add(self._forward_imu)
            self._imu_forwarding = True
        elif not result.get('enabled') and self._imu_forwarding:
            relay.remove(self._forward_imu)
            self._imu_forwarding = False
        result.pop('id', None)
        return {**result, 'imu_relay': relay.status()}

    def _feed(self):
        while not self._closing.is_set():
            started = time.monotonic()
            try:
                frame = self._capture_fn()
                if frame is not None and frame is not self._last_sent and frame.shape == (self.height, self.width, 3):
                    self._last_frame = frame
                    self._frame_map.seek(0)
                    self._frame_map.write(frame.tobytes())
                    self.request('frame', timeout=3)
                    self._last_sent = frame
            except Exception:
                if not self._closing.is_set():
                    logger.exception('External camera frame could not be delivered')
            self._closing.wait(max(0, 1/self.fps - (time.monotonic()-started)))

    def capture_frame(self):
        if self._capture_fn is not None:
            return self._last_frame
        snapshot = self.request('capture', timeout=3)
        import cv2
        import numpy as np
        if snapshot.get('jpeg'):
            return cv2.imdecode(np.frombuffer(base64.b64decode(snapshot['jpeg']), dtype=np.uint8), cv2.IMREAD_COLOR)
        if snapshot.get('yuy2'):
            raw = np.frombuffer(base64.b64decode(snapshot['yuy2']), dtype=np.uint8).reshape(self.height, self.width, 2)
            return cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUY2)
        return None

    def status(self):
        if self.process.poll() is not None:
            return {'active':False, 'peers':[], 'fps':0}
        return self.request('status', timeout=3)

    def stop(self):
        self._closing.set()
        if self._imu_forwarding:
            from app.media.imu_relay import relay
            relay.remove(self._forward_imu)
            self._imu_forwarding = False
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        if self._feeder and self._feeder is not threading.current_thread():
            self._feeder.join(timeout=4)
        for reader, pipe in ((self._reader, self.process.stdout), (self._error_reader, self.process.stderr)):
            if reader is not threading.current_thread():
                reader.join(timeout=2)
            if not reader.is_alive():
                pipe.close()
        self.process.stdin.close()
        if self._frame_map and (not self._feeder or not self._feeder.is_alive()):
            self._frame_map.close()
            self._frame_file.close()
            self._frame_map = None
