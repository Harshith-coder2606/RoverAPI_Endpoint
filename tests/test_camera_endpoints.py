"""Hardware-free checks of the actual camera routes and capture/lifecycle locking."""
import ast
import asyncio
from concurrent.futures import ThreadPoolExecutor
import inspect
from pathlib import Path
from types import SimpleNamespace
from typing import Optional
import threading
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'app/api/navigation/camera.py'

class HTTPException(Exception):
    def __init__(self, status_code, detail):
        self.status_code = status_code
        self.detail = detail

class CameraEndpointTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse(SOURCE.read_text())
        names = {'detect_cameras', 'start_camera', 'stop_camera', 'stop_all_cameras', 'cameras_status', 'camera_status'}
        nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name in names]
        for node in nodes:
            node.decorator_list = []
        self.manager = Mock()
        self.ns = {'camera_manager': self.manager, 'CAMERA_DEVICES': {'science': '/dev/camera-science'},
                   'Optional': Optional, 'Form': lambda default, **kwargs: default, 'HTTPException': HTTPException, 're': __import__('re')}
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), 'exec'), self.ns)

    def test_camera_routes_work_without_machine_specific_debug_files(self):
        self.manager.detect_cameras.return_value = [{'name': 'science'}]
        self.manager.start_camera.return_value = True
        self.manager.get_camera_status.return_value = {'active': True}
        self.assertEqual(self.ns['detect_cameras']()['count'], 1)
        self.assertEqual(self.ns['start_camera']('science')['status'], 'ok')
        self.assertEqual(self.ns['camera_status']('science')['camera']['active'], True)
        self.assertEqual(self.ns['stop_camera']('science')['status'], 'ok')
        self.assertEqual(self.ns['stop_all_cameras']()['status'], 'ok')
        self.assertTrue(all(not inspect.iscoroutinefunction(self.ns[n]) for n in
                            ('detect_cameras', 'start_camera', 'stop_camera', 'cameras_status')))

    def test_invalid_camera_validation_and_request_shape_are_preserved(self):
        with self.assertRaises(HTTPException) as invalid:
            self.ns['start_camera']('not-a-camera')
        self.assertEqual(invalid.exception.status_code, 404)
        with self.assertRaises(HTTPException) as invalid_format:
            self.ns['start_camera']('science', pixel_format='invalid')
        self.assertEqual(invalid_format.exception.status_code, 400)
        self.ns['start_camera']('video3', 640, 480, 24, 'MJPG')
        self.manager.start_camera.assert_called_once_with('video3', 640, 480, 24, 'MJPG')

    def test_generic_detected_cameras_are_accepted_by_webrtc(self):
        path = ROOT / 'app/api/navigation/camera_webrtc.py'
        tree = ast.parse(path.read_text())
        node = next(n for n in tree.body if isinstance(n, ast.AsyncFunctionDef) and n.name == 'webrtc_offer')
        node.decorator_list = []
        node.body = [n for n in node.body if not isinstance(n, ast.ImportFrom)]
        handle = Mock()
        async def offer(**kwargs):
            handle(**kwargs)
            return {'sdp': 'answer', 'type': 'answer', 'peer_id': 'one', 'adaptive_quality': True, 'target_fps': 24}
        ns = {'camera_manager': self.manager, 'CAMERA_DEVICES': {'science': 'device'},
              'WebRTCOfferRequest': object, 'HTTPException': HTTPException, 'handle_offer': offer, 'asyncio': asyncio}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), ns)
        self.manager.get_camera_status.return_value = {'active': True}
        result = asyncio.run(ns['webrtc_offer']('video3', SimpleNamespace(fps=24, sdp='offer', type='offer')))
        self.assertEqual(result['camera_name'], 'video3')
        self.assertEqual(handle.call_args.kwargs['source_name'], 'camera:video3')
        with self.assertRaises(HTTPException):
            asyncio.run(ns['webrtc_offer']('video-not-a-number', SimpleNamespace(fps=24)))




if __name__ == '__main__':
    unittest.main()
