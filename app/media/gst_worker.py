"""Native GStreamer process. JSON on stdin/stdout is control only, never video."""
import argparse
import atexit
import base64
from collections import deque
import json
import mmap
import signal
import struct
import subprocess
import sys
import threading
import time

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstSdp', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import GLib, Gst, GstSdp, GstVideo, GstWebRTC

Gst.init(None)
p = argparse.ArgumentParser()
p.add_argument('--device', required=True)
p.add_argument('--width', type=int, required=True)
p.add_argument('--height', type=int, required=True)
p.add_argument('--fps', type=int, required=True)
p.add_argument('--pixel-format', default='MJPG')
p.add_argument('--frame-file')
p.add_argument('--dejello-state', help='Calibration file; enables IMU rolling-shutter correction support')
a = p.parse_args()
output_lock = threading.Lock()

def reply(request_id, **values):
    with output_lock:
        print(json.dumps({'id': request_id, **values}), flush=True)

for plugin in ('x264enc', 'webrtcbin', 'nicesrc', 'dtlssrtpenc', 'rtph264pay'):
    if not Gst.ElementFactory.find(plugin):
        raise RuntimeError(f'Missing GStreamer plugin {plugin}; install the documented media packages')

old_dynamic = None

def restore_controls():
    global old_dynamic
    if old_dynamic is not None:
        previous, old_dynamic = old_dynamic, None
        try:
            subprocess.run(['v4l2-ctl', '-d', a.device, f'--set-ctrl=exposure_dynamic_framerate={previous}'], capture_output=True, timeout=3)
        except (OSError, subprocess.SubprocessError):
            pass

atexit.register(restore_controls)
if a.device.startswith('/dev/') and a.fps > 30:
    try:
        result = subprocess.run(['v4l2-ctl', '-d', a.device, '--get-ctrl=exposure_dynamic_framerate'], capture_output=True, text=True, timeout=3)
        if result.returncode == 0:
            old_dynamic = int(result.stdout.rsplit(':', 1)[1])
            subprocess.run(['v4l2-ctl', '-d', a.device, '--set-ctrl=exposure_dynamic_framerate=0'], check=True, capture_output=True, timeout=3)
    except (OSError, ValueError, subprocess.SubprocessError):
        pass  # This control is camera-specific; negotiated/measured FPS remains visible.

frame_map = None
frame_file = None
if a.device == 'appsrc':
    frame_file = open(a.frame_file, 'r+b')
    frame_map = mmap.mmap(frame_file.fileno(), a.width * a.height * 3)
    source = f'appsrc name=camera is-live=true format=time do-timestamp=true block=false max-buffers=1 leaky-type=downstream caps="video/x-raw,format=BGR,width={a.width},height={a.height},framerate={a.fps}/1" ! '
elif a.device == 'test':
    source = f'videotestsrc name=camera is-live=true ! video/x-raw,width={a.width},height={a.height},framerate={a.fps}/1 ! '
else:
    caps = (f'image/jpeg,width={a.width},height={a.height},framerate={a.fps}/1' if a.pixel_format == 'MJPG'
            else f'video/x-raw,format=YUY2,width={a.width},height={a.height},framerate={a.fps}/1')
    source = f'v4l2src name=camera device={json.dumps(a.device)} io-mode=mmap ! {caps} ! '
    if a.pixel_format == 'MJPG':
        source += ('tee name=jpeg jpeg. ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! '
                   'appsink name=snapshot emit-signals=true max-buffers=1 drop=true sync=false async=false '
                   'jpeg. ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! jpegdec ! ')

if a.device in ('test', 'appsrc') or a.pixel_format != 'MJPG':
    source += 'queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! '

# IMU rolling-shutter correction needs the decoded frame before encoding. The
# pipeline is split at raw I420; frames pass through untouched while disabled.
dejello = None
if a.dejello_state and a.width % 2 == 0 and a.height % 2 == 0:
    try:
        sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
        from dejello import Dejello
        dejello = Dejello(a.width, a.height, a.fps, a.dejello_state)
    except Exception as exc:  # numpy/OpenCV missing: stream without correction
        print(f'IMU rolling-shutter correction unavailable: {exc}', file=sys.stderr, flush=True)
        dejello = None
split = ('appsink name=dejello_in emit-signals=true max-buffers=1 drop=true sync=false async=false '
         'appsrc name=dejello_out is-live=true format=time do-timestamp=false block=false max-buffers=2 leaky-type=downstream ! '
         if dejello else '')
pipeline = Gst.parse_launch(source +
    'videoconvert ! video/x-raw,format=I420 ! ' + split +
    f'x264enc name=encoder tune=zerolatency speed-preset=ultrafast bitrate={6000 if a.fps > 30 else 4000} '
    f'key-int-max={a.fps} bframes=0 threads=0 sliced-threads=true sync-lookahead=0 rc-lookahead=0 ! '
    'video/x-h264,profile=constrained-baseline,stream-format=byte-stream,alignment=au ! '
    'tee name=encoded allow-not-linked=true ! queue max-size-buffers=1 leaky=downstream ! fakesink sync=false async=false')
loop = GLib.MainLoop()
peers = {}
latest_jpeg = None
latest_raw = None
capture_times = {}
captured = deque(maxlen=120)
fatal_error = None
first_frame = False

def source_probe(pad, info):
    global first_frame, latest_raw
    b = info.get_buffer()
    now = time.time_ns()
    capture_times[b.pts] = now
    captured.append(now)
    if a.device.startswith('/dev/') and a.pixel_format != 'MJPG':
        latest_raw = b.copy_deep()
    while len(capture_times) > 240:
        del capture_times[next(iter(capture_times))]
    if not first_frame:
        first_frame = True
        reply('ready', ready=True)
    return Gst.PadProbeReturn.OK
pipeline.get_by_name('camera').get_static_pad('src').add_probe(Gst.PadProbeType.BUFFER, source_probe)

def save_jpeg(sink):
    global latest_jpeg
    sample = sink.emit('pull-sample')
    if sample:
        b = sample.get_buffer()
        latest_jpeg = b.extract_dup(0, b.get_size())
    return Gst.FlowReturn.OK
if pipeline.get_by_name('snapshot'):
    pipeline.get_by_name('snapshot').connect('new-sample', save_jpeg)

def corrected_frame(sink):
    sample = sink.emit('pull-sample')
    if not sample:
        return Gst.FlowReturn.OK
    out_src = pipeline.get_by_name('dejello_out')
    caps = sample.get_caps()
    if out_src.get_property('caps') is None or not out_src.get_property('caps').is_equal(caps):
        out_src.set_property('caps', caps)
    buf = sample.get_buffer()
    if dejello.enabled:
        info = GstVideo.VideoInfo.new_from_caps(caps)
        capture_ns = capture_times.get(buf.pts)
        ok, mapped = buf.map(Gst.MapFlags.READ)
        if ok:
            try:
                data = np.frombuffer(mapped.data, dtype=np.uint8)
                dims = [(info.width, info.height), (info.width // 2, info.height // 2), (info.width // 2, info.height // 2)]
                planes = [data[info.offset[i]:info.offset[i] + info.stride[i] * h].reshape(h, info.stride[i])[:, :w]
                          for i, (w, h) in enumerate(dims)]
                corrected = dejello.process(planes, capture_ns / 1e6 if capture_ns else None)
            except Exception as exc:
                print(f'Rolling-shutter correction skipped a frame: {exc}', file=sys.stderr, flush=True)
                corrected = None
            finally:
                buf.unmap(mapped)
            if corrected is not None:
                out = np.zeros(info.size, dtype=np.uint8)
                for i, (w, h) in enumerate(dims):
                    out[info.offset[i]:info.offset[i] + info.stride[i] * h].reshape(h, info.stride[i])[:, :w] = corrected[i]
                fixed = Gst.Buffer.new_wrapped(out.tobytes())
                fixed.pts, fixed.dts, fixed.duration = buf.pts, buf.dts, buf.duration
                buf = fixed
    out_src.emit('push-buffer', buf)
    return Gst.FlowReturn.OK
if dejello:
    import numpy as np
    pipeline.get_by_name('dejello_in').connect('new-sample', corrected_frame)

def remove_peer(peer_id):
    peer = peers.pop(peer_id, None)
    if not peer:
        return False
    if peer.get('pending') is not None:
        reply(peer['pending'], error='Viewer closed during negotiation')
    # Native callbacks capture this peer and otherwise retain its RTC resources.
    for obj, handler in peer['handlers']:
        obj.disconnect(handler)
    for pad, probe in peer['probes']:
        pad.remove_probe(probe)
    tee_pad = peer['tee_pad']
    tee_pad.unlink(peer['bin'].get_static_pad('sink'))
    pipeline.get_by_name('encoded').release_request_pad(tee_pad)
    peer['bin'].set_state(Gst.State.NULL)
    pipeline.remove(peer['bin'])
    peer['handlers'].clear()
    peer['probes'].clear()
    return True

def maybe_answer(peer):
    if peer.get('pending') is not None and peer['rtc'].get_property('ice-gathering-state') == GstWebRTC.WebRTCICEGatheringState.COMPLETE:
        desc = peer['rtc'].get_property('local-description')
        if desc:
            reply(peer.pop('pending'), sdp=desc.sdp.as_text(), type='answer')

def add_peer(request):
    peer_id = request['peer_id']
    if len(peers) >= 5:
        raise ValueError('Maximum of five viewers reached')
    # Use the browser's constrained-baseline payload number; never negotiate VP8.
    payload = None
    lines = request['sdp'].splitlines()
    for line in lines:
        if line.startswith('a=rtpmap:') and ' H264/90000' in line:
            pt = line.split(':', 1)[1].split()[0]
            fmtp = next((s for s in lines if s.startswith(f'a=fmtp:{pt} ')), '')
            if 'packetization-mode=1' in fmtp and ('profile-level-id=42' in fmtp.lower() or 'profile-level-id' not in fmtp):
                payload = int(pt)
                break
    if payload is None:
        raise ValueError('Browser must offer H.264 constrained-baseline with packetization-mode=1')
    branch = Gst.parse_bin_from_description(
        'queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! '
        f'rtph264pay name=pay pt={payload} config-interval=-1 mtu=1200 aggregate-mode=zero-latency ! '
        f'application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,payload={payload} ! '
        'webrtcbin name=rtc bundle-policy=max-bundle latency=0', True)
    rtc = branch.get_by_name('rtc')
    transceiver = rtc.emit('get-transceiver', 0)
    if transceiver is not None:
        transceiver.set_property('direction', GstWebRTC.WebRTCRTPTransceiverDirection.SENDONLY)
        transceiver.set_property('codec-preferences', Gst.Caps.from_string(
            f'application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,payload={payload}'))
    pipeline.add(branch)
    tee_pad = pipeline.get_by_name('encoded').request_pad_simple('src_%u')
    peer = {'bin':branch, 'rtc':rtc, 'tee_pad':tee_pad, 'pending':request['id'], 'times':deque(maxlen=240), 'started':time.monotonic(), 'handlers':[], 'probes':[]}
    peers[peer_id] = peer
    # Activate the branch before live buffers can reach its sink pad. Linking
    # an inactive pad can return FLUSHING and stop the shared source task.
    branch.sync_state_with_parent()
    if tee_pad.link(branch.get_static_pad('sink')) != Gst.PadLinkReturn.OK:
        remove_peer(peer_id)
        raise RuntimeError('Unable to link viewer to shared H.264 encoder')
    def packets(pad, info):
        buffers = ([info.get_buffer()] if info.type & Gst.PadProbeType.BUFFER else
                   [info.get_buffer_list().get(i) for i in range(info.get_buffer_list().length())])
        segment = pad.get_sticky_event(Gst.EventType.SEGMENT, 0).parse_segment()
        for b in buffers:
            if b.get_size() >= 12:
                header = b.extract_dup(0, 12)
                if header[1] & 128:
                    pts = segment.to_running_time(Gst.Format.TIME, b.pts)
                    capture_ns = capture_times.get(pts)
                    if capture_ns:
                        peer['times'].append([struct.unpack_from('!I', header, 4)[0], capture_ns / 1e6])
        return Gst.PadProbeReturn.OK
    pay_pad = branch.get_by_name('pay').get_static_pad('src')
    peer['probes'].append((pay_pad, pay_pad.add_probe(Gst.PadProbeType.BUFFER | Gst.PadProbeType.BUFFER_LIST, packets)))
    peer['handlers'].append((rtc, rtc.connect('notify::ice-gathering-state', lambda *_: GLib.idle_add(lambda: (maybe_answer(peer), False)[1]))))
    def connection_changed(*_):
        state = rtc.get_property('connection-state')
        if state in (GstWebRTC.WebRTCPeerConnectionState.FAILED, GstWebRTC.WebRTCPeerConnectionState.CLOSED):
            GLib.idle_add(lambda: (remove_peer(peer_id), False)[1])
    peer['handlers'].append((rtc, rtc.connect('notify::connection-state', connection_changed)))
    def data_channel(_rtc, channel):
        if channel.get_property('label') != 'timing':
            channel.emit('close')
            return
        def message(_channel, text):
            received = time.time_ns() / 1e6
            try:
                if len(text) > 128:
                    return
                data = json.loads(text)
                sent = data['client_sent_ms']
                if not isinstance(sent, (int, float)):
                    return
                channel.emit('send-string', json.dumps({'client_sent_ms':sent, 'server_receive_ms':received,
                    'frames':list(peer['times']) if data.get('include_frames') else [], 'server_send_ms':time.time_ns()/1e6}))
            except (KeyError, ValueError, TypeError):
                return
        peer['handlers'].append((channel, channel.connect('on-message-string', message)))
    peer['handlers'].append((rtc, rtc.connect('on-data-channel', data_channel)))
    status, msg = GstSdp.SDPMessage.new_from_text(request['sdp'])
    if status != GstSdp.SDPResult.OK:
        raise ValueError('Invalid SDP')
    offer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.OFFER, msg)
    def answer_created(promise, *_):
        try:
            result = promise.get_reply()
            answer = result.get_value('answer')
            if answer is None:
                raise RuntimeError(str(result))
            rtc.emit('set-local-description', answer, Gst.Promise.new())
        except Exception as exc:
            reply(peer.pop('pending', request['id']), error=str(exc))
            GLib.idle_add(lambda: (remove_peer(peer_id), False)[1])
    def remote_set(promise, *_):
        rtc.emit('create-answer', None, Gst.Promise.new_with_change_func(answer_created, None, None))
    rtc.emit('set-remote-description', offer, Gst.Promise.new_with_change_func(remote_set, None, None))

def command(request):
    request_id = request['id']
    try:
        cmd = request['cmd']
        if cmd == 'offer':
            add_peer(request)
            return False
        if cmd == 'imu':
            # Gyro samples from the API on this computer; high rate, so never acknowledged.
            if dejello:
                dejello.add_gyro(request.get('samples', []))
            return False
        if cmd == 'close':
            result = {'closed':remove_peer(request['peer_id'])}
        elif cmd == 'status':
            result = {'active':fatal_error is None and first_frame, 'error':fatal_error, 'peers':list(peers),
                      'peer_status':{key:{'state':str(peer['rtc'].get_property('connection-state')), 'recent_rtp_frames':len(peer['times'])} for key,peer in peers.items()},
                      'fps':(len(captured)-1)*1e9/(captured[-1]-captured[0]) if len(captured)>1 else 0,
                      'dejello_supported':dejello is not None}
        elif cmd == 'dejello':
            if not dejello:
                raise ValueError('IMU rolling-shutter correction is not available for this camera')
            readout = request.get('readout_ms', 'keep')
            result = dejello.configure(enabled=request.get('enabled'), readout_ms=readout, reset=bool(request.get('reset')),
                                       source=request.get('source'))
        elif cmd == 'timing':
            peer = peers.get(request['peer_id'])
            result = {'frames':list(peer['times']) if peer else [], 'server_ms':time.time_ns()/1e6}
        elif cmd == 'capture':
            result = {'jpeg':base64.b64encode(latest_jpeg).decode() if latest_jpeg else None}
            if latest_raw is not None:
                result['yuy2'] = base64.b64encode(latest_raw.extract_dup(0, latest_raw.get_size())).decode()
        elif cmd == 'frame':
            b = Gst.Buffer.new_allocate(None, len(frame_map), None)
            b.fill(0, frame_map[:])
            b.duration = Gst.SECOND // a.fps
            pipeline.get_by_name('camera').emit('push-buffer', b)
            result = {}
        elif cmd == 'stop':
            result = {}
            loop.quit()
        else:
            raise ValueError('Unknown media command')
        reply(request_id, **result)
    except Exception as exc:
        if request.get('peer_id') in peers and request.get('cmd') == 'offer':
            remove_peer(request['peer_id'])
        reply(request_id, error=str(exc))
    return False

def read_commands():
    for line in sys.stdin:
        try:
            GLib.idle_add(command, json.loads(line))
        except (ValueError, KeyError):
            continue
    GLib.idle_add(loop.quit)

def bus_message(bus, msg):
    global fatal_error
    if msg.type == Gst.MessageType.ERROR:
        err, debug = msg.parse_error()
        print(f'{err}: {debug}', file=sys.stderr, flush=True)
        # A disconnected viewer can emit an SCTP/transport error. Its branch
        # must not take the shared encoder or other viewers down with it.
        for peer_id, peer in list(peers.items()):
            if msg.src == peer['bin'] or msg.src.has_as_ancestor(peer['bin']):
                remove_peer(peer_id)
                return
        # Messages queued during removal may outlive their detached branch.
        if msg.src != pipeline and not msg.src.has_as_ancestor(pipeline):
            return
        fatal_error = str(err)
        reply('ready', error=fatal_error)
        loop.quit()
bus = pipeline.get_bus()
bus.add_signal_watch()
bus.connect('message', bus_message)
def reap():
    for peer_id, peer in list(peers.items()):
        state = peer['rtc'].get_property('connection-state')
        if time.monotonic()-peer['started']>30 and state != GstWebRTC.WebRTCPeerConnectionState.CONNECTED:
            remove_peer(peer_id)
    return True
GLib.timeout_add_seconds(5, reap)
threading.Thread(target=read_commands, daemon=True).start()
signal.signal(signal.SIGTERM, lambda *_: loop.quit())
signal.signal(signal.SIGINT, lambda *_: loop.quit())
pipeline.set_state(Gst.State.PLAYING)
try:
    loop.run()
finally:
    for peer_id in list(peers):
        remove_peer(peer_id)
    pipeline.set_state(Gst.State.NULL)
    if dejello:
        dejello.stop()
    if frame_map:
        frame_map.close()
        frame_file.close()
    restore_controls()
