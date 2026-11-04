"""Rolling-shutter correction math. Runs anywhere numpy and OpenCV are installed."""
import math

import cv2
import numpy as np
import pytest

from app.media.dejello import (
    Dejello, FlowTracker, GyroBuffer, Remapper, VideoReadoutEstimator, fit_calibration, observe,
    row_warp, source_position, valid_calibration,
)


def rate(t):
    s = t / 1000
    return (0.6 * math.sin(2 * math.pi * 1.3 * s) + 0.3 * math.sin(2 * math.pi * 3.7 * s + 1),
            0.5 * math.sin(2 * math.pi * 0.9 * s + 2) + 0.25 * math.sin(2 * math.pi * 4.3 * s),
            0.4 * math.sin(2 * math.pi * 2.1 * s + 0.5) + 0.2 * math.sin(2 * math.pi * 5.1 * s + 3))


def gyro_at(hz, start, end):
    g = GyroBuffer()
    for t in np.arange(start, end, 1000 / hz):
        g.add(float(t), *rate(t))
    return g


def test_gyro_integrates_exactly_and_rejects_gaps():
    g = GyroBuffer()
    g.add(1000, 0, 1, 2); g.add(1100, 1, 1, 2); g.add(1050, 0.5, 1, 2)
    assert np.allclose(g.integrate(1000, 1100), [0.05, 0.1, 0.2])
    assert g.integrate(900, 1100) is None
    g.add(1500, 0, 0, 0)
    assert g.integrate(1000, 1400) is None
    assert g.mean_rate(1550, 1600) == [0, 0, 0]
    assert g.mean_rate(1700, 1750) is None
    g.add(100, 9, 9, 9)
    assert len(g) == 1


def synthetic_observations(A, offset, readout, truth, interval=1000 / 15, seed=7):
    rng = np.random.default_rng(seed)
    out, frame = [], 0
    for t in np.arange(1000, 60000, interval):
        frame += 1
        at = lambda shift: truth.integrate(t - interval + shift, t + shift)
        g = at(offset)
        bands = []
        for s in (-1 / 3, 0, 1 / 3):
            gb = at(offset + s * readout)
            bands.append({'s': s, 'dx': float(np.dot(A[0], gb) + rng.uniform(-.2, .2)),
                          'dy': float(np.dot(A[1], gb) + rng.uniform(-.2, .2)), 'n': 40})
        out.append({'frame': frame, 'capture_ms': float(t), 'interval_ms': interval,
                    'dx': float(np.dot(A[0], g) + rng.uniform(-.2, .2)), 'dy': float(np.dot(A[1], g) + rng.uniform(-.2, .2)),
                    'roll': float(np.dot(A[2], g) + rng.uniform(-2.5e-4, 2.5e-4)), 'n': 120, 'bands': bands})
    return out


def test_calibration_recovers_mapping_offset_and_readout_with_30hz_gyro():
    A = np.array([[40, -950, 15], [720, 30, -20], [0.02, 0.05, -0.97]])
    truth, gyro = gyro_at(1000, 0, 70000), gyro_at(30, 0, 70000)
    result = fit_calibration(synthetic_observations(A, -64, 14, truth), gyro, 1280, 720, max_readout_ms=16)
    assert result['ok'], result
    c = result['calibration']
    assert abs(c['offset_ms'] + 64) <= 4
    assert c['readout_fitted'] and abs(c['readout_ms'] - 14) <= 3
    assert c['fit'] > 0.98 and c['roll_fitted']
    for r in range(3):
        assert np.allclose(c['gain'][r], A[r], atol=0.05 * np.abs(A[r]).max())
    assert valid_calibration(c)


def test_calibration_refuses_a_still_camera():
    g = GyroBuffer()
    for t in range(0, 30000, 33):
        g.add(t, 0.001, -0.001, 0)
    obs = [{'frame': i, 'capture_ms': t, 'interval_ms': 66, 'dx': math.sin(t), 'dy': math.cos(t), 'roll': 0, 'n': 100, 'bands': []}
           for i, t in enumerate(range(2000, 25000, 66))]
    result = fit_calibration(obs, g, 640, 480)
    assert not result['ok'] and 'rotation' in result['reason']


def test_video_readout_estimate_from_band_skew():
    truth = gyro_at(1000, 0, 70000)
    gain = np.array([[-900, 700, 30], [700, -900, 0]])
    est, rng, interval, readout = VideoReadoutEstimator(), np.random.default_rng(3), 1000 / 15, 14
    motion = lambda t, sh: gain @ np.array(truth.integrate(t - interval + sh, t + sh))
    frame = 0
    for t in np.arange(1000, 60000, interval):
        frame += 2 if t % 997 < 70 else 1
        mid = motion(t, 0)
        bands = [{'s': s, 'dx': m[0] + rng.uniform(-.5, .5), 'dy': m[1] + rng.uniform(-.5, .5), 'n': 30}
                 for s in (-1 / 3, 0, 1 / 3) for m in [motion(t, s * readout)]]
        est.add({'frame': frame, 'capture_ms': t, 'interval_ms': interval, 'dx': mid[0], 'dy': mid[1], 'roll': 0, 'n': 90, 'bands': bands})
    e = est.estimate()
    # Motion near the 7.5 Hz analysis Nyquist limit biases the derivative low, so the estimate reads slightly high.
    assert e and readout - 1 <= e['readout_ms'] <= readout * 1.25 and e['correlation'] > 0.5


def texture(w, h, seed=1):
    rng = np.random.default_rng(seed)
    img = cv2.resize(rng.integers(0, 256, (h // 8, w // 8), dtype=np.uint8), (w, h), interpolation=cv2.INTER_CUBIC)
    return cv2.GaussianBlur(img, (3, 3), 0)


def test_flow_tracker_measures_translation_in_every_band():
    base = texture(400, 320)
    tracker = FlowTracker()
    assert tracker.track(base[20:260, 20:340].copy()) is None
    rows = tracker.track(base[17:257, 25:345].copy())  # content moves left 5, down 3
    assert rows is not None and len(rows) > 60
    assert np.allclose(np.median(rows[:, 2:], axis=0), [-5, 3], atol=0.2)
    o = observe(rows, 1280, 720, 1000.0, 66.7, 2)
    assert len(o['bands']) == 3
    assert abs(o['dx'] + 20) < 1 and abs(o['dy'] - 9) < 1  # 4x and 3x scale to 1280x720


def rolling_shutter_frame(scene, warp, width, height):
    """Forward model: captured(q) = ideal(q - s(q_y) * v), with ideal = scene centre crop."""
    vx, vy, vroll = warp
    ys, xs = np.mgrid[0:height, 0:width].astype(np.float32)
    s = ys / height - 0.5
    px = xs - s * (vx - vroll * (ys - height / 2))
    py = ys - s * (vy + vroll * (xs - width / 2))
    ox, oy = (scene.shape[1] - width) / 2, (scene.shape[0] - height) / 2
    return cv2.remap(scene, (px + ox).astype(np.float32), (py + oy).astype(np.float32), cv2.INTER_LINEAR)


def test_remapper_removes_synthetic_rolling_shutter_shear():
    width, height = 640, 360
    scene = texture(800, 520, seed=5)
    ox, oy = 80, 80
    ideal = scene[oy:oy + height, ox:ox + width]
    warp = (40.0, -18.0, 0.02)
    captured = rolling_shutter_frame(scene, warp, width, height)
    # Chroma planes are processed the same way at half size; check luma accuracy here.
    corrected = Remapper(width, height).apply([captured, captured[::2, ::2].copy(), captured[::2, ::2].copy()], warp)[0]
    inner = (slice(40, -40), slice(60, -60))
    before = np.abs(captured[inner].astype(int) - ideal[inner]).mean()
    after = np.abs(corrected[inner].astype(int) - ideal[inner]).mean()
    assert after < 0.15 * before, (before, after)
    # No warp is an exact identity, edges included.
    same = Remapper(width, height).apply([ideal.copy(), ideal[::2, ::2].copy(), ideal[::2, ::2].copy()], (0.0, 0.0, 0.0))
    assert np.array_equal(same[0], ideal)


def test_row_warp_scales_with_resolution():
    cal = {'version': 1, 'gain': [[0, -900, 0], [700, 0, 0], [0, 0, -1]], 'offset_ms': -60, 'readout_ms': 20,
           'readout_fitted': True, 'roll_fitted': True, 'fit': .9, 'samples': 100, 'width': 1280, 'height': 720}
    assert np.allclose(row_warp(cal, (0.5, -1, 0.2), 1280, 720, 20), (18, 7, -0.004))
    assert np.allclose(row_warp(cal, (0.5, -1, 0.2), 640, 360, 20), (9, 3.5, -0.004))
    assert source_position((0, 0, 0), 10, 20, 1280, 720) == (10, 20)


def test_imu_source_corrects_only_with_calibration_and_fresh_gyro(tmp_path):
    d = Dejello(640, 360, 30, str(tmp_path / 'cal.json'))
    try:
        y = texture(640, 360); u = v = np.full((180, 320), 128, np.uint8)
        assert d.process([y, u, v], 5000.0) is None  # disabled
        d.configure(enabled=True, source='imu')
        assert d.process([y, u, v], 5000.0) is None  # not calibrated
        d.calibration = {'version': 1, 'gain': [[0, -900, 0], [700, 0, 0], [0, 0, 0]], 'offset_ms': -40, 'readout_ms': 20,
                         'readout_fitted': True, 'roll_fitted': False, 'fit': .9, 'samples': 100, 'width': 640, 'height': 360}
        assert d.process([y, u, v], 5000.0) is None and d.stats['no_gyro'] == 1
        d.add_gyro([[4900 + i * 33, 0.0, 1.0, 0.0] for i in range(6)])
        out = d.process([y, u, v], 5000.0)
        assert out is not None and out[0].shape == y.shape and out[1].shape == u.shape
        assert d.status()['readout_source'] == 'fitted' and d.stats['warp_px'] > 10
        d.configure(reset=True)
        assert d.calibration is None and not (tmp_path / 'cal.json').exists()
    finally:
        d.stop()


def test_row_tables_match_exact_inverse():
    r = Remapper(1280, 720)
    for warp in [(30.0, -20.0, 0.0), (25.0, 12.0, 0.01), (-40.0, 30.0, -0.02)]:
        mx, my = r.maps(warp, 1)
        tolerance = 2e-3  # float32 tables vs float64 reference
        for x, y in [(0, 0), (640, 360), (1279, 719), (100, 600), (1200, 50)]:
            qx, qy = source_position(warp, float(x), float(y), 1280, 720)
            assert abs(mx[y, x] - qx) < tolerance and abs(my[y, x] - qy) < tolerance, (warp, x, y, mx[y, x], qx, my[y, x], qy)
        # Half-resolution chroma tables address the same scene points.
        hx, hy = r.maps(warp, 2)
        qx, qy = source_position(warp, 2 * 100 + 0.5, 2 * 50 + 0.5, 1280, 720)
        assert abs(hx[50, 100] - ((qx + 0.5) / 2 - 0.5)) < tolerance and abs(hy[50, 100] - ((qy + 0.5) / 2 - 0.5)) < tolerance


def test_strip_transforms_match_exact_inverse():
    r = Remapper(1280, 720)
    assert Remapper.strips_for(0.0, 1280) == 1
    for warp, limit in [((30.0, -20.0, 0.0), 1e-3), ((25.0, 10.0, 0.004), 0.35), ((-40.0, 30.0, -0.02), 0.35)]:
        for scale in (1, 2):
            h, w = 720 // scale, 1280 // scale
            mx, my = r.maps(warp, scale)
            worst = 0.0
            for r0, r1, M in r.strip_transforms(warp, scale, h, w, Remapper.strips_for(warp[2], w)):
                X, Y = np.meshgrid(np.arange(w), np.arange(r1 - r0))
                px = M[0, 0] * X + M[0, 1] * Y + M[0, 2]
                py = M[1, 0] * X + M[1, 1] * Y + M[1, 2]
                worst = max(worst, np.abs(px - mx[r0:r1]).max(), np.abs(py - my[r0:r1]).max())
            assert worst < limit, (warp, scale, worst)
    # Without roll the strip warp reproduces exact remapping, pixel for pixel (up to rounding).
    y = texture(1280, 720, seed=9)
    fast = r.apply([y, y[::2, ::2].copy(), y[::2, ::2].copy()], (30.0, -20.0, 0.0))[0]
    mx, my = r.maps((30.0, -20.0, 0.0), 1)
    exact = cv2.remap(y, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=16)
    assert np.abs(fast[20:-20, 30:-30].astype(int) - exact[20:-20, 30:-30]).max() <= 2


def test_pooled_strips_match_sequential():
    from concurrent.futures import ThreadPoolExecutor
    r = Remapper(1280, 720)
    y = texture(1280, 720, seed=4); u = y[::2, ::2].copy()
    with ThreadPoolExecutor(4) as pool:
        for warp in [(18.0, 7.0, 0.0), (18.0, 7.0, 0.004)]:
            a = r.apply([y, u, u.copy()], warp)
            b = r.apply([y, u, u.copy()], warp, pool)
            for pa, pb in zip(a, b):
                assert np.abs(pa.astype(int) - pb).max() <= 2  # strip splits differ only by rounding


def test_video_source_corrects_a_rolling_shutter_pan_without_imu(tmp_path):
    width, height, fps, readout_ms, speed = 640, 360, 15, 40.0, 240.0  # content moves 240 px/s
    scene = texture(1600, 520, seed=11)
    oy = 80

    def frame(t, rolling):
        ys, xs = np.mgrid[0:height, 0:width].astype(np.float32)
        s = ys / height - 0.5 if rolling else 0.0
        sx = xs + 400 - speed * (t + s * readout_ms / 1000)   # row y is exposed at t + s * readout
        return cv2.remap(scene, sx.astype(np.float32), (ys + oy).astype(np.float32), cv2.INTER_LINEAR)

    d = Dejello(width, height, fps, str(tmp_path / 'cal.json'))
    try:
        d.configure(enabled=True, source='video', readout_ms=readout_ms)
        chroma = np.full((height // 2, width // 2), 128, np.uint8)
        before, after = [], []
        for k in range(10):
            t = k / fps
            captured, ideal = frame(t, True), frame(t, False)
            out = d.process([captured, chroma, chroma.copy()], t * 1000)
            if k >= 3:  # tracker seeded and velocity extrapolation primed
                assert out is not None and d.status()['active_source'] == 'video'
                inner = (slice(30, -30), slice(60, -60))
                before.append(np.abs(captured[inner].astype(int) - ideal[inner]).mean())
                after.append(np.abs(out[0][inner].astype(int) - ideal[inner]).mean())
        assert np.mean(after) < 0.35 * np.mean(before), (np.mean(before), np.mean(after))
        # Expected shear: speed * readout = 9.6 px across the frame.
        assert 7 < d.stats['warp_px'] < 12
    finally:
        d.stop()


def test_auto_source_uses_video_until_imu_is_calibrated(tmp_path):
    d = Dejello(640, 360, 30, str(tmp_path / 'cal.json'))
    try:
        with pytest.raises(ValueError):
            d.configure(source='gyro')
        d.configure(enabled=True)
        y = texture(640, 360); c = np.full((180, 320), 128, np.uint8)
        d.process([y, c, c.copy()], 1000.0)
        assert d.status()['source'] == 'auto' and d.status()['active_source'] is None  # first frame only seeds flow
        d.calibration = {'version': 1, 'gain': [[0, -900, 0], [700, 0, 0], [0, 0, 0]], 'offset_ms': -40, 'readout_ms': 20,
                         'readout_fitted': True, 'roll_fitted': False, 'fit': .9, 'samples': 100, 'width': 640, 'height': 360}
        d.add_gyro([[900 + i * 33, 0.0, 0.2, 0.0] for i in range(8)])
        assert d.process([y, c, c.copy()], 1033.0) is not None and d.status()['active_source'] == 'imu'
    finally:
        d.stop()
