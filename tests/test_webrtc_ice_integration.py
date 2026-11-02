"""Real ICE + RTP test; uses aiortc only as a test receiver and requires native GStreamer, no camera.

Run: python -m unittest discover -s tests -p test_webrtc_ice_integration.py
"""
import asyncio
import unittest
from unittest.mock import patch

try:
    import numpy as np
    from aiortc import RTCBundlePolicy, RTCConfiguration, RTCPeerConnection, RTCSessionDescription
    from app.api import webrtc_utils as rtc
except ImportError:
    rtc = None


@unittest.skipIf(rtc is None, "aiortc/av not installed")
class IceIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_native_h264_ice_connects_and_delivers_frames(self):
        client = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        source = "camera:test"
        from app.media.gstreamer import GStreamerSource
        try:
            native = GStreamerSource('test', 320, 240, 24)
        except (RuntimeError, OSError) as error:
            self.skipTest(str(error))
        track_ready = asyncio.get_running_loop().create_future()
        @client.on("track")
        def on_track(track):
            track_ready.set_result(track)
        try:
            client.addTransceiver("video", direction="recvonly")
            await client.setLocalDescription(await client.createOffer())
            offer = client.localDescription.sdp
            with patch.object(rtc, "_hardware_source", return_value=native):
                answer = await asyncio.wait_for(rtc.handle_offer(source, offer, "offer", lambda: np.zeros((240, 320, 3), dtype=np.uint8), fps=24), 10)
            self.assertIn("a=candidate:", answer["sdp"])
            await client.setRemoteDescription(RTCSessionDescription(sdp=answer["sdp"], type=answer["type"]))
            track = await asyncio.wait_for(track_ready, 5)
            frames = [await asyncio.wait_for(track.recv(), 10) for _ in range(3)]
            self.assertEqual(client.connectionState, "connected")
            self.assertTrue(all(frame.width == 320 for frame in frames))
            self.assertGreater(frames[-1].pts, frames[0].pts)
            self.assertEqual(answer["target_fps"], 24)
            self.assertEqual(await rtc.close_peer_connection(source, answer["peer_id"]), 1)
            self.assertEqual(await rtc.get_connection_count(source), 0)
        finally:
            await client.close()
            await rtc.close_all_connections(source)
            native.stop()


    async def test_closing_data_channel_viewer_preserves_other_viewer(self):
        from app.media.gstreamer import GStreamerSource
        try:
            native = GStreamerSource('test', 320, 240, 24)
        except (RuntimeError, OSError) as error:
            self.skipTest(str(error))
        clients = []
        peer_ids = []
        tracks = []
        source = 'camera:shared-test'
        try:
            for _ in range(2):
                client = RTCPeerConnection(RTCConfiguration(iceServers=[], bundlePolicy=RTCBundlePolicy.MAX_BUNDLE))
                clients.append(client)
                ready = asyncio.get_running_loop().create_future()
                client.on('track', lambda track, ready=ready: ready.set_result(track))
                client.addTransceiver('video', direction='recvonly')
                client.createDataChannel('timing', ordered=False, maxRetransmits=0)
                await client.setLocalDescription(await client.createOffer())
                with patch.object(rtc, '_hardware_source', return_value=native):
                    answer = await rtc.handle_offer(source, client.localDescription.sdp, 'offer')
                peer_ids.append(answer['peer_id'])
                await client.setRemoteDescription(RTCSessionDescription(sdp=answer['sdp'], type='answer'))
                track = await asyncio.wait_for(ready, 5)
                await asyncio.wait_for(track.recv(), 10)
                tracks.append(track)
            await clients[0].close()
            await rtc.close_peer_connection(source, peer_ids[0])
            await asyncio.sleep(0.5)
            status = await asyncio.to_thread(native.status)
            self.assertTrue(status['active'])
            self.assertEqual(status['peers'], [peer_ids[1]])
            self.assertEqual(clients[1].connectionState, 'connected')
            self.assertEqual((await asyncio.wait_for(tracks[1].recv(), 5)).width, 320)
        finally:
            for client in clients:
                await client.close()
            await rtc.close_all_connections(source)
            native.stop()


if __name__ == "__main__":
    unittest.main()
