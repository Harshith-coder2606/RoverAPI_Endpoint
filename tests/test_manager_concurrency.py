from concurrent.futures import ThreadPoolExecutor
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from app.arduino import manager as arduino
from app.ros import manager as ros
from app.api.science import microscope


class ManagerConcurrencyTests(unittest.TestCase):
    def test_concurrent_serial_connects_open_one_port(self):
        manager = object.__new__(arduino.ArduinoManager)
        manager.__init__()
        port = Mock()
        with patch.object(arduino.serial, "Serial", return_value=port) as open_port, patch.object(arduino.time, "sleep"):
            with ThreadPoolExecutor(max_workers=5) as workers:
                self.assertTrue(all(workers.map(lambda _: manager.connect(), range(10))))
            self.assertEqual(open_port.call_count, 1)

    def test_serial_disconnect_waits_for_an_inflight_command(self):
        manager = object.__new__(arduino.ArduinoManager)
        manager.__init__()
        manager.is_connected = True
        manager.serial_port = port = Mock()
        entered, release = threading.Event(), threading.Event()
        def write(data):
            entered.set()
            if not release.wait(3):
                raise RuntimeError("test did not release write")
        port.write.side_effect = write
        with ThreadPoolExecutor(max_workers=2) as workers:
            sending = workers.submit(manager.send_command, "w:120")
            try:
                self.assertTrue(entered.wait(1))
                disconnecting = workers.submit(manager.disconnect)
                self.assertFalse(port.close.called)
            finally:
                release.set()
                self.assertTrue(sending.result(timeout=2))
                disconnecting.result(timeout=2)
        self.assertTrue(port.close.called)
        self.assertFalse(manager.is_connected)

    def test_concurrent_ros_subscriptions_register_one_topic(self):
        manager = object.__new__(ros.RosbridgeManager)
        manager.__init__()
        manager.is_connected = True
        manager.client = Mock()
        topic = Mock()
        with patch.object(ros.roslibpy, "Topic", return_value=topic) as create_topic:
            with ThreadPoolExecutor(max_workers=5) as workers:
                self.assertTrue(all(workers.map(lambda _: manager.subscribe("/test", "std_msgs/Int32"), range(10))))
            self.assertEqual(create_topic.call_count, 1)
            self.assertEqual(topic.subscribe.call_count, 1)
        manager._latest_messages["/test"] = {"data": 1}
        manager.disconnect()
        self.assertEqual(manager._latest_messages, {})



if __name__ == "__main__":
    unittest.main()
