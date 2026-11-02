"""H.264 WebRTC signaling for native GStreamer sources; video never passes through FastAPI."""
import asyncio
from uuid import uuid4

from app.media.gstreamer import GStreamerSource

MAX_WEBRTC_CLIENTS_PER_SOURCE = 5
_peer_connections = {}
_sources = {}
_pending_peers = set()
_source_reaper = None
_pc_lock = asyncio.Lock()


async def _reap_sources():
    """Reclaim crashed or abandoned ROS sources even when a browser never sends DELETE."""
    global _source_reaper
    try:
        while _peer_connections or _sources:
            await asyncio.sleep(5)
            async with _pc_lock:
                for source in set(_sources.values()) | {s for peers in _peer_connections.values() for s in peers.values()}:
                    active = await _prune(source)
                    reserved = any(peer_id in _pending_peers for entries in _peer_connections.values() for peer_id, current in entries.items() if current is source)
                    if not active and not reserved:
                        for name in [n for n, current in _sources.items() if current is source]:
                            _sources.pop(name, None)
                            await asyncio.to_thread(source.stop)
    finally:
        _source_reaper = None


async def _start_ros_source(capture_fn, width, height, fps):
    task = asyncio.create_task(asyncio.to_thread(GStreamerSource, 'appsrc', width, height, fps, 'BGR', capture_fn))
    try:
        return await asyncio.shield(task)
    except asyncio.CancelledError:
        # Cancelling an HTTP request cannot stop a constructor already running in a thread.
        try:
            source = await task
        except Exception:
            pass
        else:
            await asyncio.to_thread(source.stop)
        raise


async def _prune(source):
    status = await asyncio.to_thread(source.status)
    active = set(status['peers'])
    for name, entries in list(_peer_connections.items()):
        for peer_id, current in list(entries.items()):
            if current is source and peer_id not in active and peer_id not in _pending_peers:
                entries.pop(peer_id, None)
        if not entries:
            _peer_connections.pop(name, None)
    return active


def _hardware_source(source_name):
    from app.api.navigation.camera import camera_manager
    name = source_name.split(':', 1)[1] if source_name.startswith('camera:') else 'microscope'
    source = camera_manager.cameras.get(name)
    if source is None or not source.status()['active']:
        raise RuntimeError('Camera has not been started')
    return source


async def handle_offer(source_name, sdp, sdp_type, capture_fn=None, fps=60):
    global _source_reaper
    if sdp_type != 'offer':
        raise RuntimeError('Expected an SDP offer')
    peer_id = uuid4().hex
    async with _pc_lock:
        if source_name.startswith('ros_camera:'):
            source = _sources.get(source_name)
            if source is None:
                frame = await asyncio.to_thread(capture_fn)
                if frame is None:
                    raise RuntimeError('ROS camera has no image yet')
                height, width = frame.shape[:2]
                source = await _start_ros_source(capture_fn, width, height, fps)
                _sources[source_name] = source
        else:
            source = await asyncio.to_thread(_hardware_source, source_name)
        # Worker enforces the limit across aliases of the same physical camera.
        await _prune(source)
        reserved = sum(current is source for entries in _peer_connections.values() for current in entries.values())
        if reserved >= MAX_WEBRTC_CLIENTS_PER_SOURCE:
            raise ValueError('Maximum of five viewers reached')
        _peer_connections.setdefault(source_name, {})[peer_id] = source
        _pending_peers.add(peer_id)
        if _source_reaper is None or _source_reaper.done():
            _source_reaper = asyncio.create_task(_reap_sources())
    try:
        answer = await asyncio.to_thread(source.request, 'offer', peer_id=peer_id, sdp=sdp)
        return {**answer, 'peer_id':peer_id, 'adaptive_quality':False,
                'target_fps':source.fps, 'codec':'H264', 'transport':'UDP', 'encoder':'x264-ultrafast'}
    except BaseException:
        await close_peer_connection(source_name, peer_id)
        raise
    finally:
        _pending_peers.discard(peer_id)


async def close_peer_connection(source_name, peer_id):
    async with _pc_lock:
        entries = _peer_connections.get(source_name, {})
        source = entries.pop(peer_id, None)
        if not entries:
            _peer_connections.pop(source_name, None)
    if source is None:
        return False
    try:
        await asyncio.to_thread(source.request, 'close', timeout=3, peer_id=peer_id)
    except (RuntimeError, TimeoutError, OSError):
        pass  # A stopped camera has already closed its peers.
    async with _pc_lock:
        if source_name.startswith('ros_camera:') and source_name not in _peer_connections:
            if _sources.get(source_name) is source:
                _sources.pop(source_name, None)
                await asyncio.to_thread(source.stop)
    return True


async def close_all_connections(source_name):
    async with _pc_lock:
        ids = list(_peer_connections.get(source_name, {}))
    return sum(await asyncio.gather(*(close_peer_connection(source_name, peer_id) for peer_id in ids)))


async def get_connection_count(source_name):
    async with _pc_lock:
        entries = dict(_peer_connections.get(source_name, {}))
        if not entries:
            return 0
        source = next(iter(entries.values()))
        active = await _prune(source)
        if not active and source_name.startswith('ros_camera:') and not _pending_peers:
            _sources.pop(source_name, None)
            await asyncio.to_thread(source.stop)
        return len(active)


async def get_all_webrtc_stats():
    async with _pc_lock:
        names = list(_peer_connections)
    return {name:{'active_connections':await get_connection_count(name),
                  'max_connections':MAX_WEBRTC_CLIENTS_PER_SOURCE} for name in names}


async def get_frame_timing(source_name, peer_id):
    async with _pc_lock:
        source = _peer_connections.get(source_name, {}).get(peer_id)
    if source is None:
        return None
    return await asyncio.to_thread(source.request, 'timing', timeout=3, peer_id=peer_id)


async def shutdown_media():
    global _source_reaper
    if _source_reaper:
        task = _source_reaper
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
    for name in list(_peer_connections):
        await close_all_connections(name)
    for source in set(_sources.values()):
        await asyncio.to_thread(source.stop)
    _sources.clear()
