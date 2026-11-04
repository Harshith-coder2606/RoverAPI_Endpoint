"""Forward BNO055 gyro samples from the local rosbridge to camera workers.

The data path is rosbridge (localhost) -> this API process -> worker stdin, all on
the rover computer. Nothing here is exposed over HTTP or WebRTC.
"""
import logging
import math
import threading
import time

logger = logging.getLogger(__name__)
IMU_TOPIC = '/imu/data'


def parse_imu(message):
    """sensor_msgs/Imu from rosbridge -> [epoch ms, wx, wy, wz] in rad/s, or None."""
    try:
        stamp, rate = message['header']['stamp'], message['angular_velocity']
        sec = stamp.get('sec', stamp.get('secs'))
        nsec = stamp.get('nanosec', stamp.get('nsecs'))
        sample = [sec * 1000 + nsec / 1e6, float(rate['x']), float(rate['y']), float(rate['z'])]
    except (KeyError, TypeError, AttributeError, ValueError):
        return None
    # The firmware publishes only after micro-ROS time sync, so stamp 0 is malformed.
    if sec <= 0 or not all(math.isfinite(v) for v in sample):
        return None
    return sample


class ImuRelay:
    def __init__(self, manager_factory=None):
        self._sinks = set()
        self._lock = threading.Lock()
        self._manager_factory = manager_factory
        self._thread = None
        self._subscribed = False
        self.last_sample_monotonic = None
        self.received = 0

    def _manager(self):
        if self._manager_factory is not None:
            return self._manager_factory()
        from app.ros.manager import RosbridgeManager
        return RosbridgeManager()

    def add(self, sink):
        """sink(sample) is called from the rosbridge thread and must not block."""
        with self._lock:
            self._sinks.add(sink)
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._supervise, daemon=True, name='imu-relay')
                self._thread.start()

    def remove(self, sink):
        with self._lock:
            self._sinks.discard(sink)

    def _deliver(self, message):
        sample = parse_imu(message)
        if sample is None:
            return
        self.last_sample_monotonic = time.monotonic()
        self.received += 1
        with self._lock:
            sinks = list(self._sinks)
        for sink in sinks:
            try:
                sink(sample)
            except Exception:
                logger.exception('IMU sample could not reach a camera worker')

    def _supervise(self):
        """Keep one subscription while any worker needs gyro data; recover after rosbridge restarts."""
        while True:
            with self._lock:
                needed = bool(self._sinks)
            manager = self._manager()
            try:
                if not needed:
                    if self._subscribed:
                        manager.unsubscribe(IMU_TOPIC)
                        self._subscribed = False
                    with self._lock:
                        if not self._sinks:
                            self._thread = None
                            return
                    continue
                stale = self.last_sample_monotonic is None or time.monotonic() - self.last_sample_monotonic > 3
                if self._subscribed and stale and not manager.is_connected:
                    self._subscribed = False
                if self._subscribed and stale and self.last_sample_monotonic is not None:
                    # A rosbridge restart can leave a dead subscription object behind.
                    manager.unsubscribe(IMU_TOPIC)
                    self._subscribed = False
                    self.last_sample_monotonic = None
                if not self._subscribed:
                    if not manager.is_connected:
                        manager.connect()
                    if manager.is_connected:
                        self._subscribed = manager.subscribe(IMU_TOPIC, 'sensor_msgs/Imu', self._deliver)
            except Exception:
                logger.exception('IMU relay supervision failed')
                self._subscribed = False
            time.sleep(1)

    def status(self):
        age = None if self.last_sample_monotonic is None else (time.monotonic() - self.last_sample_monotonic) * 1000
        return {'subscribed': self._subscribed, 'received': self.received, 'last_age_ms': age}


relay = ImuRelay()
