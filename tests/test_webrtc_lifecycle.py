"""Exercise native-source reservations and viewer ownership without camera hardware."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
import threading
import unittest
from unittest.mock import AsyncMock, Mock, patch
from app.api import webrtc_utils as rtc


class Source:
    fps = 60
    def __init__(self):
        self.peers = set()
        self.fail = False
        self.stopped = False
    def status(self):
        return {'active':not self.stopped, 'peers':list(self.peers), 'fps':60}
    def request(self, cmd, **values):
        peer_id = values.get('peer_id')
        if cmd == 'offer':
            self.peers.add(peer_id)
            if self.fail:
                raise RuntimeError('Invalid native SDP')
            return {'sdp':'answer', 'type':'answer'}
        if cmd == 'close':
            found = peer_id in self.peers
            self.peers.discard(peer_id)
            return {'closed':found}
        if cmd == 'timing':return {'frames':[]}
    def stop(self):self.stopped = True


class WebRTCLifecycleTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        rtc._peer_connections.clear(); rtc._sources.clear(); rtc._pending_peers.clear()
        rtc._pc_lock = asyncio.Lock()
        self.source = Source()
        self.patcher = patch.object(rtc, '_hardware_source', return_value=self.source)
        self.patcher.start()
    async def asyncTearDown(self):
        for name in list(rtc._peer_connections):await rtc.close_all_connections(name)
        if rtc._source_reaper:
            task = rtc._source_reaper
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        self.patcher.stop()
    async def offer(self, name='camera:kreo'):
        return await rtc.handle_offer(name, 'offer', 'offer')
    async def test_one_viewer_closes_without_stopping_shared_encoder(self):
        a,b = await asyncio.gather(self.offer(),self.offer())
        self.assertEqual(a['target_fps'],60)
        self.assertEqual(a['codec'],'H264')
        self.assertFalse(a['adaptive_quality'])
        self.assertTrue(await rtc.close_peer_connection('camera:kreo',a['peer_id']))
        self.assertEqual(self.source.peers,{b['peer_id']})
        self.assertFalse(self.source.stopped)
    async def test_peer_cannot_close_another_source_and_repeated_close_is_safe(self):
        a = await self.offer()
        self.assertFalse(await rtc.close_peer_connection('microscope',a['peer_id']))
        self.assertEqual(await rtc.get_connection_count('camera:kreo'),1)
        self.assertTrue(await rtc.close_peer_connection('camera:kreo',a['peer_id']))
        self.assertFalse(await rtc.close_peer_connection('camera:kreo',a['peer_id']))
    async def test_capacity_is_atomic_across_device_aliases(self):
        results = await asyncio.gather(*(self.offer('camera:kreo' if i%2 else 'camera:video2') for i in range(12)),return_exceptions=True)
        self.assertEqual(sum(isinstance(r,dict) for r in results),5)
        self.assertTrue(all(isinstance(r,(dict,ValueError)) for r in results))
        self.assertEqual(len(self.source.peers),5)
    async def test_failed_negotiation_reclaims_registration_and_native_peer(self):
        self.source.fail = True
        with self.assertRaisesRegex(RuntimeError,'Invalid native SDP'):await self.offer()
        self.assertEqual(self.source.peers,set())
        self.assertEqual(rtc._peer_connections,{})
        self.assertEqual(rtc._pending_peers,set())
    async def test_transport_failure_releases_capacity_on_next_status(self):
        await self.offer()
        self.source.peers.clear()
        self.assertEqual(await rtc.get_connection_count('camera:kreo'),0)
        self.assertEqual(rtc._peer_connections,{})
        self.assertIsInstance(await self.offer(),dict)
    async def test_close_all_leaves_other_physical_source_running(self):
        await self.offer();await self.offer()
        other = Source()
        with patch.object(rtc,'_hardware_source',return_value=other):
            second = await self.offer('camera:arm')
        self.assertEqual(await rtc.close_all_connections('camera:kreo'),2)
        self.assertEqual(other.peers,{second['peer_id']})

    async def test_cancelled_ros_constructor_stops_its_late_source(self):
        entered, release = threading.Event(), threading.Event()
        def construct(*args):
            entered.set()
            release.wait(2)
            return self.source
        with patch.object(rtc, 'GStreamerSource', side_effect=construct):
            starting = asyncio.create_task(rtc._start_ros_source(lambda: None, 320, 240, 30))
            self.assertTrue(await asyncio.to_thread(entered.wait, 1))
            starting.cancel()
            release.set()
            with self.assertRaises(asyncio.CancelledError):
                await starting
        self.assertTrue(self.source.stopped)
        self.assertEqual(rtc._sources, {})

    async def test_abandoned_ros_source_is_reclaimed_without_status_requests(self):
        rtc._sources['ros_camera:test'] = self.source
        rtc._peer_connections['ros_camera:test'] = {'gone': self.source}
        with patch.object(rtc.asyncio, 'sleep', new=AsyncMock()):
            await rtc._reap_sources()
        self.assertTrue(self.source.stopped)
        self.assertEqual(rtc._sources, {})
        self.assertEqual(rtc._peer_connections, {})


class NativeCameraOwnershipTests(unittest.TestCase):
    def test_concurrent_alias_starts_share_one_encoder_and_stop_releases_all_aliases(self):
        from app.api.navigation.camera import MultiCameraManager
        manager = MultiCameraManager()
        source = Mock(width=1280, height=720, fps=60)
        source.status.return_value = {'active':True, 'peers':[], 'fps':60}
        with patch.object(manager, '_device_path', return_value='/dev/video2'), patch('os.path.exists', return_value=True), patch('app.media.gstreamer.GStreamerSource', return_value=source) as create:
            with ThreadPoolExecutor(max_workers=4) as pool:
                results = list(pool.map(lambda n: manager.start_camera(n, 1280, 720, 60, 'MJPG'), ['kreo','video2']*4))
            self.assertTrue(all(results))
            create.assert_called_once()
            self.assertIs(manager.cameras['kreo'], manager.cameras['video2'])
            self.assertTrue(manager.stop_camera('kreo'))
            source.stop.assert_called_once()
            self.assertEqual(manager.cameras, {})

    def test_mode_conflict_preserves_existing_viewers(self):
        from app.api.navigation.camera import MultiCameraManager
        from fastapi import HTTPException
        manager = MultiCameraManager()
        source = Mock(width=1280, height=720, fps=60)
        source.status.return_value = {'active':True}
        manager.cameras['kreo'] = source
        manager.camera_info['kreo'] = {'device_path':'/dev/video2'}
        with patch.object(manager, '_device_path', return_value='/dev/video2'), patch('os.path.exists', return_value=True):
            with self.assertRaises(HTTPException) as conflict:
                manager.start_camera('video2', 640, 480, 30)
            self.assertEqual(conflict.exception.status_code, 409)
            source.stop.assert_not_called()
