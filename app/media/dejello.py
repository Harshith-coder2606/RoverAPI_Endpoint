"""IMU-aided rolling-shutter ("jello") correction, run inside the camera worker.

A rolling-shutter camera exposes row y at  t_mid + s(y)*t_readout, s(y) = y/H - 0.5.
When the camera rotates, rows see the scene from different orientations, so the
frame shears and wobbles.

Optical flow on the raw frames supplies what the gyro cannot. Regressing measured
inter-frame motion against integrated gyro rotation gives:
  gain (3x3)  IMU rotation (rad) -> image motion (dx px, dy px, roll rad); this
              absorbs IMU-to-camera axes, signs and focal length
  offset_ms   when the middle row was exposed, relative to the capture stamp
  readout_ms  top-to-bottom readout, from top/middle/bottom flow differences
Each frame is then resampled so every row matches the camera orientation at
mid-frame time:  ideal(p) = captured(p + s(y) * t_r * gain . omega).

Gyro samples and capture stamps both use this computer's clock (micro-ROS
synchronizes the Teensy to it), and both stay on this computer.
Only rotation is corrected; translation and parallax are not.

Must run on the system Python used by the worker (numpy 1.21, OpenCV 4.5).
"""
from bisect import bisect_right
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
import threading
import time

import cv2
import numpy as np

FLOW_W, FLOW_H = 320, 240


# ---------------------------------------------------------------- gyro history
class GyroBuffer:
    """Time-ordered BNO055 rates (ms, rad/s) with exact piecewise-linear integration."""

    def __init__(self, retain_ms=120000):
        self.retain_ms = retain_ms
        self.t, self.w = [], []
        self.lock = threading.Lock()

    def __len__(self):
        return len(self.t)

    def add(self, t, x, y, z):
        if not all(math.isfinite(v) for v in (t, x, y, z)):
            return
        with self.lock:
            # A clock resync or firmware restart moves stamps backwards: old history no longer lines up.
            if self.t and t < self.t[-1] - 1000:
                self.t, self.w = [], []
            i = bisect_right(self.t, t)
            if i and self.t[i - 1] == t:
                return
            self.t.insert(i, t)
            self.w.insert(i, (x, y, z))
            cutoff = self.t[-1] - self.retain_ms
            drop = bisect_right(self.t, cutoff)
            if drop:
                del self.t[:drop], self.w[:drop]

    def span(self):
        with self.lock:
            return (self.t[0], self.t[-1]) if self.t else (math.inf, -math.inf)

    def integrate(self, t0, t1):
        """Rotation over [t0, t1] in radians, or None without full, gap-free coverage."""
        with self.lock:
            t, w = self.t, self.w
            if len(t) < 2 or not t1 > t0 or t0 < t[0] or t1 > t[-1]:
                return None
            out = [0.0, 0.0, 0.0]
            i = max(0, bisect_right(t, t0) - 1)
            while i < len(t) - 1 and t[i] < t1:
                a, b = t[i], t[i + 1]
                if b - a > 200:  # dropped data, not a measured rate
                    return None
                s, e = max(t0, a), min(t1, b)
                if e > s:
                    fs, fe = (s - a) / (b - a), (e - a) / (b - a)
                    wa, wb = w[i], w[i + 1]
                    seconds = (e - s) / 1000
                    for k in range(3):
                        rs = wa[k] + (wb[k] - wa[k]) * fs
                        re = wa[k] + (wb[k] - wa[k]) * fe
                        out[k] += (rs + re) / 2 * seconds
                i += 1
            return out

    def mean_rate(self, t0, t1, hold_ms=120):
        """Mean rate over [t0, t1]. Briefly reuses the newest sample: gyro delivery can trail video."""
        first, last = self.span()
        if not t1 > t0 or t0 > last + hold_ms or t1 < first:
            return None
        s, e = max(t0, first), min(t1, last)
        if e > s:
            integral = self.integrate(s, e)
            if integral is not None:
                return [v / ((e - s) / 1000) for v in integral]
        with self.lock:
            if not self.w:
                return None
            return list(self.w[-1] if t0 >= last else self.w[0])


# --------------------------------------------------------------- optical flow
class FlowTracker:
    """Pyramidal LK on 320x240 gray with grid-spread FAST features and forward/backward checks."""

    def __init__(self):
        self.prev = None
        self.points = None
        self.frame = 0
        self.fast = cv2.FastFeatureDetector_create(5, True, cv2.FAST_FEATURE_DETECTOR_TYPE_9_16)

    def reset(self):
        self.prev = self.points = None

    def detect(self, gray):
        keypoints = self.fast.detect(gray, None)
        cells = {}
        for k in keypoints:
            x, y = k.pt
            if x < 8 or y < 8 or x >= FLOW_W - 8 or y >= FLOW_H - 8:
                continue
            cells.setdefault(int(y * 6 / FLOW_H) * 8 + int(x * 8 / FLOW_W), []).append(k)
        # The strongest corners cluster on one textured object; a grid keeps every band covered.
        chosen = [k for cell in cells.values() for k in sorted(cell, key=lambda k: -k.response)[:5]]
        if not chosen:
            return None
        return np.array([k.pt for k in chosen], dtype=np.float32).reshape(-1, 1, 2)

    def track(self, gray):
        """Returns accepted (x, y, dx, dy) rows at 320x240, or None when only seeding."""
        self.frame += 1
        prev, points = self.prev, self.points
        self.prev = gray
        if prev is None or points is None or len(points) == 0:
            self.points = self.detect(gray)
            return None
        params = dict(winSize=(21, 21), maxLevel=3,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(prev, gray, points, None, **params)
        back, st2, _ = cv2.calcOpticalFlowPyrLK(gray, prev, nxt, None, **params)
        p0, p1, pb = points.reshape(-1, 2), nxt.reshape(-1, 2), back.reshape(-1, 2)
        fb = np.hypot(*(p0 - pb).T)
        d = p1 - p0
        ok = ((st.ravel() == 1) & (st2.ravel() == 1) & np.isfinite(fb) & (fb <= 1.5) &
              np.all(np.isfinite(p1), axis=1) & (p1[:, 0] >= 0) & (p1[:, 0] < FLOW_W) &
              (p1[:, 1] >= 0) & (p1[:, 1] < FLOW_H) & (np.hypot(*d.T) <= 80))
        rows = np.hstack([p1[ok], d[ok]])
        if len(rows):
            # Median/MAD rejects independently moving objects.
            med = np.median(rows[:, 2:], axis=0)
            mad = np.median(np.abs(rows[:, 2:] - med), axis=0)
            keep = np.all(np.abs(rows[:, 2:] - med) <= np.maximum(3 * mad, 1.0), axis=1)
            rows = rows[keep]
        bands = np.minimum(2, (rows[:, 1] * 3 / FLOW_H).astype(int)) if len(rows) else np.array([], int)
        thin = any((bands == b).sum() < 8 for b in range(3))
        if self.frame % 30 == 0 or len(rows) < 30 or thin:
            self.points = self.detect(gray)
        else:
            self.points = rows[:, :2].astype(np.float32).reshape(-1, 1, 2)
        return rows


def observe(rows, width, height, capture_ms, interval_ms, frame):
    """Whole-frame and per-band motion in source pixels, plus in-plane roll."""
    if rows is None or len(rows) < 12 or not 0 < interval_ms <= 500:
        return None
    sx, sy = width / FLOW_W, height / FLOW_H
    x, y = rows[:, 0] * sx - width / 2, rows[:, 1] * sy - height / 2
    fx, fy = rows[:, 2] * sx, rows[:, 3] * sy
    dx, dy = float(fx.mean()), float(fy.mean())
    rx, ry = fx - dx, fy - dy
    radius = float((x * x + y * y).sum())
    roll = float((x * ry - y * rx).sum() / radius) if radius > 0 else 0.0
    bands = []
    index = np.minimum(2, (rows[:, 1] * 3 / FLOW_H).astype(int))
    for b in range(3):
        m = index == b
        if m.sum() >= 4:
            bands.append({'s': float((rows[m, 1] * sy).mean() / height - 0.5),
                          'dx': float(fx[m].mean()), 'dy': float(fy[m].mean()), 'n': int(m.sum())})
    return {'frame': frame, 'capture_ms': capture_ms, 'interval_ms': interval_ms,
            'dx': dx, 'dy': dy, 'roll': roll, 'n': int(len(rows)), 'bands': bands}


# ---------------------------------------------------------------- calibration
def _regress(G, y, wts, ridge):
    """Weighted least squares y ~ [G, 1] with ridge shrinkage on gyro terms.

    An axis the camera never rotated about carries no information; shrinking
    keeps its gain near zero instead of fitting noise that would warp later frames."""
    X = np.hstack([G, np.ones((len(G), 1))])
    W = wts[:, None]
    A = X.T @ (X * W)
    A[:3, :3] += np.eye(3) * ridge * wts.sum()
    try:
        coef = np.linalg.solve(A, X.T @ (y * wts))
    except np.linalg.LinAlgError:
        return None
    pred = X @ coef
    mean = (wts * y).sum() / wts.sum()
    sse = float((wts * (y - pred) ** 2).sum())
    sst = float((wts * (y - mean) ** 2).sum())
    return coef, (1 - sse / sst if sst > 0 else 0.0)


def _rotation(gyro, o, shift):
    return gyro.integrate(o['capture_ms'] - o['interval_ms'] + shift, o['capture_ms'] + shift)


def fit_calibration(observations, gyro, width, height, max_readout_ms=33,
                    offset_range=(-300, 100), min_samples=60, min_fit=0.5):
    lo, hi = offset_range
    first, last = gyro.span()
    usable = [o for o in observations
              if o['capture_ms'] - o['interval_ms'] + lo - max_readout_ms / 2 >= first
              and o['capture_ms'] + hi + max_readout_ms / 2 <= last]
    if len(usable) < min_samples:
        return {'ok': False, 'reason': f'Need {min_samples} flow frames with gyro coverage (have {len(usable)})', 'samples': len(usable)}
    wts = np.sqrt([o['n'] for o in usable]).astype(float)
    targets = {k: np.array([o[k] for o in usable]) for k in ('dx', 'dy', 'roll')}

    def evaluate(offset):
        G = []
        for o in usable:
            g = _rotation(gyro, o, offset)
            if g is None:
                return None
            G.append(g)
        G = np.array(G)
        fits = [_regress(G, targets[k], wts, 4e-6) for k in ('dx', 'dy', 'roll')]
        if any(f is None for f in fits):
            return None
        return {'offset': offset, 'G': G, 'fits': fits, 'score': (fits[0][1] + fits[1][1]) / 2}

    best = None
    for offset in range(lo, hi + 1, 6):  # coarse, then refine around the best
        c = evaluate(offset)
        if c and (best is None or c['score'] > best['score']):
            best = c
    if best:
        centre = best['offset']
        for offset in range(max(lo, centre - 5), min(hi, centre + 5) + 1):
            if offset != centre:
                c = evaluate(offset)
                if c and c['score'] > best['score']:
                    best = c
    if best is None:
        return {'ok': False, 'reason': 'Gyro history has gaps; check /imu/data', 'samples': len(usable)}
    excitation = np.sqrt((best['G'] ** 2).mean(axis=0)).tolist()
    if max(excitation) < 0.006:
        return {'ok': False, 'reason': 'Not enough rotation yet: pan, tilt and rock the camera', 'samples': len(usable), 'excitation': excitation}
    if best['score'] < min_fit:
        return {'ok': False, 'reason': f"Gyro explains too little image motion (R2 {best['score']:.2f}); avoid nearby moving objects",
                'samples': len(usable), 'excitation': excitation}

    ax, ay, ar = (f[0] for f in best['fits'])

    def readout_sse(tr):
        sse = 0.0
        for o in usable:
            for band in o['bands']:
                g = _rotation(gyro, o, best['offset'] + band['s'] * tr)
                if g is None:
                    return None
                px = ax[0] * g[0] + ax[1] * g[1] + ax[2] * g[2] + ax[3]
                py = ay[0] * g[0] + ay[1] * g[1] + ay[2] * g[2] + ay[3]
                sse += band['n'] * ((band['dx'] - px) ** 2 + (band['dy'] - py) ** 2)
        return sse

    base = readout_sse(0)
    readout, best_sse = 0, base if base is not None else math.inf
    for tr in list(range(4, int(max_readout_ms) + 1, 4)):
        sse = readout_sse(tr)
        if sse is not None and sse < best_sse:
            readout, best_sse = tr, sse
    for tr in range(max(1, readout - 3), min(int(max_readout_ms), readout + 3) + 1):
        if tr != readout:
            sse = readout_sse(tr)
            if sse is not None and sse < best_sse:
                readout, best_sse = tr, sse
    # Constant rotation moves every band alike; the readout is then unobservable.
    readout_fitted = bool(base and (base - best_sse) / base >= 0.03 and 0 < readout < max_readout_ms)
    roll_fitted = best['fits'][2][1] >= 0.3
    return {'ok': True, 'excitation': excitation, 'calibration': {
        'version': 1,
        'gain': [list(map(float, ax[:3])), list(map(float, ay[:3])), list(map(float, ar[:3])) if roll_fitted else [0.0, 0.0, 0.0]],
        'offset_ms': float(best['offset']), 'readout_ms': float(readout), 'readout_fitted': readout_fitted,
        'roll_fitted': bool(roll_fitted), 'fit': float(best['score']), 'samples': len(usable),
        'width': width, 'height': height, 'created': time.time()}}


class VideoReadoutEstimator:
    """Readout from video alone: band difference = t_r * (bottom.s - top.s) * dm/dt.

    dm/dt uses a five-frame derivative; three frames underestimate fast-changing
    motion by sin(wD)/(wD) and inflate the readout. Motion close to the analysis
    Nyquist rate still biases it high (about 15% in tests), so the gyro-fitted
    readout is preferred whenever it is observable. Needs changing motion."""

    def __init__(self):
        self.window = deque(maxlen=5)
        self.reset()

    def reset(self):
        self.window.clear()
        self.sx = self.sy = self.sxx = self.sxy = self.syy = 0.0
        self.n = 0

    def add(self, o):
        w = self.window
        if w and w[-1]['frame'] + 1 != o['frame']:
            w.clear()
        w.append(o)
        if len(w) < 5:
            return
        a, b, c, d, e = w
        if len(c['bands']) < 3 or c['bands'][0]['n'] < 5 or c['bands'][-1]['n'] < 5 or c['n'] < 30:
            return
        top, bottom = c['bands'][0], c['bands'][-1]
        span, seconds = bottom['s'] - top['s'], c['interval_ms'] / 1000
        for k in ('dx', 'dy'):
            x = span * (a[k] - 8 * b[k] + 8 * d[k] - e[k]) / (12 * seconds)
            y = bottom[k] - top[k]
            self.sx += x; self.sy += y; self.sxx += x * x; self.sxy += x * y; self.syy += y * y; self.n += 1

    def estimate(self, min_pairs=200, min_correlation=0.25):
        if self.n < min_pairs:
            return None
        vx = self.sxx - self.sx ** 2 / self.n
        vy = self.syy - self.sy ** 2 / self.n
        cov = self.sxy - self.sx * self.sy / self.n
        if vx <= 0 or vy <= 0:
            return None
        r = cov / math.sqrt(vx * vy)
        readout = cov / vx * 1000
        if r < min_correlation or readout <= 0:
            return None
        return {'readout_ms': readout, 'correlation': r, 'pairs': self.n}


def valid_calibration(c):
    try:
        return (c['version'] == 1 and len(c['gain']) == 3 and all(len(g) == 3 for g in c['gain'])
                and all(math.isfinite(v) for g in c['gain'] for v in g)
                and all(math.isfinite(c[k]) for k in ('offset_ms', 'readout_ms', 'fit'))
                and c['width'] > 0 and c['height'] > 0)
    except (KeyError, TypeError):
        return False


# ----------------------------------------------------------------- correction
def row_warp(cal, rate, width, height, readout_ms):
    """Image velocity during readout: displacement at row offset s is s*(vx, vy) plus rotation s*vroll."""
    seconds = readout_ms / 1000
    gx, gy, gr = cal['gain']
    dot = lambda g: g[0] * rate[0] + g[1] * rate[1] + g[2] * rate[2]
    # Focal length in pixels scales with resolution; the axis mapping does not.
    return (seconds * dot(gx) * width / cal['width'], seconds * dot(gy) * height / cal['height'], seconds * dot(gr))


def source_position(warp, x, y, width, height):
    """Captured position sampled for output pixel (x, y); two fixed-point steps invert the row dependence."""
    vx, vy, vroll = warp
    qx, qy = x, y
    for _ in range(2):
        s = qy / height - 0.5
        qx = x + s * (vx - vroll * (y - height / 2))
        qy = y + s * (vy + vroll * (x - width / 2))
    return qx, qy


class Remapper:
    """Resamples I420 planes with tables built from per-row coefficients.

    source_position() is a two-step fixed point whose first step uses the row
    alone, so its result is exactly  row terms + row terms * x (+ x^2 for y).
    Evaluating that form takes a few broadcasts instead of per-pixel iteration
    (about 13 ms -> a few ms at 720p) and matches source_position() exactly."""

    def __init__(self, width, height):
        self.width, self.height = width, height
        self.planes = {}
        for scale in (1, 2):
            w, h = width // scale, height // scale
            xp, yp = np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32)
            # Plane pixel centres in full-resolution coordinates, relative to the image centre for x.
            xc = ((xp + 0.5) * scale - 0.5 - width / 2).astype(np.float32)
            yf = (yp.astype(np.float64) + 0.5) * scale - 0.5
            self.planes[scale] = (xp, yp, xc, xc * xc, yf)

    def maps(self, warp, scale=1):
        vx, vy, vroll = warp
        xp, yp, xc, xc2, yf = self.planes[scale]
        H = self.height
        s_a = yf / H - 0.5                       # first step: row only
        rc = (yf + s_a * vy) / H - 0.5           # second step: s = rc + rk * xc
        rk = s_a * vroll / H
        kx = vx - vroll * (yf - H / 2)           # horizontal velocity of this row
        f = lambda v: (v / scale).astype(np.float32)[:, None]
        mx = xp[None, :] + f(rc * kx)
        my = (yp[:, None] + f(rc * vy))
        if vroll:
            mx = mx + f(rk * kx) * xc[None, :]
            my = my + f(rc * vroll + rk * vy) * xc[None, :] + f(rk * vroll) * xc2[None, :]
        else:
            my = np.broadcast_to(my, mx.shape)
        return mx, np.ascontiguousarray(my, dtype=np.float32)

    def coefficients(self, warp, scale, rows):
        """Exact source coordinates for plane rows `rows`: mx = xp + a + b*xc, my = yp + c + d*xc + e*xc^2."""
        vx, vy, vroll = warp
        H = self.height
        yf = (np.asarray(rows, dtype=np.float64) + 0.5) * scale - 0.5
        s_a = yf / H - 0.5
        rc = (yf + s_a * vy) / H - 0.5
        rk = s_a * vroll / H
        kx = vx - vroll * (yf - H / 2)
        return rc * kx / scale, rk * kx / scale, rc * vy / scale, (rc * vroll + rk * vy) / scale, rk * vroll / scale

    def strip_transforms(self, warp, scale, h, w, strips=32):
        """(r0, r1, M) per strip: M maps strip-local output pixels to source pixels.

        Fitted at each strip's centre: exact without roll; with roll the mapping has
        small x*y and x^2 terms (at most ~0.35 px at strip corners at 80 deg/s)."""
        xc0 = 0.5 * scale - 0.5 - self.width / 2       # xc at plane column 0
        dxc = float(scale)                              # xc step per plane column
        xm = (w - 1) / 2
        xcm = xc0 + dxc * xm
        bounds = np.linspace(0, h, strips + 1).astype(int)
        r0, r1 = bounds[:-1], bounds[1:]
        keep = r1 > r0
        r0, r1 = r0[keep], r1[keep]
        # All strips at once: per-strip Python work would dominate the frame time.
        top, ym, bottom = r0 - 0.5, (r0 + r1 - 1) / 2, r1 - 0.5
        rows = np.concatenate([top, ym, bottom])
        a, b, c, d, e = (v.reshape(3, -1) for v in self.coefficients(warp, scale, rows))
        src_x = xm + a + b * xcm                         # exact source position at the centre column
        src_y = rows.reshape(3, -1) + c + d * xcm + e * xcm * xcm
        gx = 1 + b[1] * dxc                              # d/dx at the strip centre
        gy = (d[1] + 2 * e[1] * xcm) * dxc
        tx = (src_x[2] - src_x[0]) / (bottom - top)      # d/dy at the centre column
        ty = (src_y[2] - src_y[0]) / (bottom - top)
        oy = ym - r0                                     # strip-local row of the centre
        M = np.stack([np.stack([gx, tx, src_x[1] - gx * xm - tx * oy], axis=1),
                      np.stack([gy, ty, src_y[1] - gy * xm - ty * oy], axis=1)], axis=1)
        return list(zip(r0.tolist(), r1.tolist(), M))

    @staticmethod
    def strips_for(vroll, width, tolerance_px=0.25):
        """Strips needed so the roll cross-term error, measured at ~0.41 * |vroll| * width / strips,
        stays under tolerance (capped at 32 strips: ~0.33 px at an extreme 80 deg/s roll).

        Without roll the correction is one affine transform for the whole plane."""
        if abs(vroll) < 1e-9:
            return 1
        return int(min(32, max(2, math.ceil(0.41 * abs(vroll) * width / tolerance_px))))

    def apply(self, planes, warp, pool=None, min_strips=8):
        """planes: [Y, U, V] uint8 arrays (U/V at half size). Returns new contiguous planes.

        Strips are independent, so with a thread pool (and OpenCV single-threaded, which
        avoids its ~0.3 ms per-call thread wake-up) they run in parallel: ~5 ms per 720p
        frame on the rover. Without a pool, strips are only used as roll requires."""
        jobs, out = [], []
        for i, plane in enumerate(planes):
            h, w = plane.shape
            result = np.empty((h, w), dtype=np.uint8)
            out.append(result)
            strips = self.strips_for(warp[2], w)
            if pool is not None:
                strips = max(strips, min_strips if i == 0 else max(1, min_strips // 2))
            for r0, r1, M in self.strip_transforms(warp, 1 if i == 0 else 2, h, w, strips):
                jobs.append((plane, M, w, r0, r1, result, 16 if i == 0 else 128))  # black in limited-range YUV

        def run(job):
            plane, M, w, r0, r1, result, fill = job
            cv2.warpAffine(plane, M, (w, r1 - r0), dst=result[r0:r1], flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                           borderMode=cv2.BORDER_CONSTANT, borderValue=fill)
        if pool is None:
            for job in jobs:
                run(job)
        else:
            list(pool.map(run, jobs))
        return out


# --------------------------------------------------------------- orchestration
SOURCES = ('auto', 'imu', 'video')
MAX_VIDEO_WARP_PX = 80      # beyond this, video-only flow is more likely wrong than the shear is real
MAX_VIDEO_ROLL = 0.05


class Dejello:
    """Per-camera state: gyro history, calibration (flow + gyro), and live correction.

    Motion source per frame:
      imu    gyro rate through the flow+gyro calibration (needs the IMU rigidly on the camera body)
      video  image velocity from optical flow on the frames themselves, no IMU at all; it lags the
             motion by about half a frame and is fooled by large moving objects or parallax
      auto   imu when calibrated and gyro data is current, otherwise video
    """

    def __init__(self, width, height, fps, state_path=None):
        self.width, self.height, self.fps = width, height, fps
        self.state_path = state_path
        self.gyro = GyroBuffer()
        self.remapper = Remapper(width, height)
        # Strip warps run on our own pool; OpenCV's internal pool costs ~0.3 ms per call to wake.
        cv2.setNumThreads(1)
        self.pool = ThreadPoolExecutor(8, thread_name_prefix='dejello-warp')
        self.tracker = FlowTracker()          # calibration flow, flow thread
        self.sync_tracker = FlowTracker()     # video-source flow, streaming thread
        self.video_readout = VideoReadoutEstimator()
        self.video_readout_last = None
        self.source = 'auto'
        self.active_source = None
        self.obs_queue = deque(maxlen=256)
        self.sync_previous = None             # (capture_ms, observation) of the last synchronous flow
        self.video_velocity = None
        self.observations = deque(maxlen=900)
        self.enabled = False
        self.readout_override = None
        self.calibration = self._load()
        self.last_fit = None
        self.lock = threading.Lock()
        self.pending = None
        self.wake = threading.Event()
        self.stopped = False
        self.last_flow_capture = None
        self.since_fit = 0
        self.stats = {'frames': 0, 'corrected': 0, 'no_gyro': 0, 'no_flow': 0, 'warp_px': 0.0,
                      'correct_ms': 0.0, 'flow_ms': 0.0, 'video_flow_ms': 0.0}
        self.thread = threading.Thread(target=self._flow_loop, daemon=True, name='dejello-flow')
        self.thread.start()

    # --- persistence
    def _load(self):
        try:
            with open(self.state_path) as f:
                c = json.load(f)
            return c if valid_calibration(c) else None
        except (OSError, ValueError, TypeError):
            return None

    def _save(self):
        if not self.state_path:
            return
        try:
            os.makedirs(os.path.dirname(self.state_path), exist_ok=True)
            tmp = self.state_path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(self.calibration, f)
            os.replace(tmp, self.state_path)
        except OSError:
            pass

    # --- inputs
    def add_gyro(self, samples):
        for t, x, y, z in samples:
            self.gyro.add(float(t), float(x), float(y), float(z))

    def configure(self, enabled=None, readout_ms='keep', reset=False, source=None):
        with self.lock:
            if source is not None:
                if source not in SOURCES:
                    raise ValueError('source must be one of ' + ', '.join(SOURCES))
                self.source = source
            if enabled is not None:
                self.enabled = bool(enabled)
                if not self.enabled:
                    self.tracker.reset()
                    self.sync_tracker.reset()
                    self.sync_previous = self.video_velocity = None
            if readout_ms != 'keep':
                self.readout_override = None if readout_ms is None else max(0.0, min(self.max_readout_ms, float(readout_ms)))
            if reset:
                self.calibration = None
                self.last_fit = None
                self.observations.clear()
                self.video_readout.reset()
                self.video_readout_last = None
                if self.state_path:
                    try:
                        os.remove(self.state_path)
                    except OSError:
                        pass
        return self.status()

    @property
    def max_readout_ms(self):
        return max(8, min(100, int(1000 / max(1, self.fps))))

    def readout(self):
        if self.readout_override is not None:
            return self.readout_override, 'manual'
        if self.calibration and self.calibration.get('readout_fitted'):
            return self.calibration['readout_ms'], 'fitted'
        video = self.video_readout.estimate()
        if video:
            self.video_readout_last = video
        # Keep the last good estimate: it drops out whenever recent motion was too uniform to measure.
        if self.video_readout_last:
            return min(self.max_readout_ms, self.video_readout_last['readout_ms']), 'video'
        # Webcams at their configured rate usually read out over most of the frame period.
        return 0.8 * self.max_readout_ms, 'estimated'

    # --- per frame (GStreamer streaming thread)
    def process(self, planes, capture_ms):
        """planes: [Y, U, V]. Returns corrected planes, or None to pass the frame through unchanged."""
        if not self.enabled:
            return None
        self.stats['frames'] += 1
        cal = self.calibration
        rate = None
        if cal is not None and capture_ms is not None and self.source != 'video':
            readout, _ = self.readout()
            mid = capture_ms + cal['offset_ms']
            half = max(5.0, readout / 2)
            rate = self.gyro.mean_rate(mid - half, mid + half)
        use_video = self.source == 'video' or (self.source == 'auto' and rate is None)

        if use_video:
            warp = self._video_warp(planes[0], capture_ms)
        else:
            # Calibration flow at <= 15 fps, on the raw frame, off the streaming thread.
            if capture_ms is not None and (self.last_flow_capture is None or capture_ms - self.last_flow_capture >= 1000 / 15 * 0.9
                                           or capture_ms < self.last_flow_capture):
                small = cv2.resize(planes[0], (FLOW_W, FLOW_H), interpolation=cv2.INTER_AREA)
                self.pending = (small, capture_ms)
                self.last_flow_capture = capture_ms
                self.wake.set()
            if rate is None:
                # Never apply a stale shear; show the frame as captured.
                if cal is not None:
                    self.stats['no_gyro'] += 1
                self.active_source = None
                return None
            readout, _ = self.readout()
            warp = row_warp(cal, rate, self.width, self.height, readout)
            self.active_source = 'imu'
        if warp is None:
            self.active_source = None
            return None
        started = time.perf_counter()
        out = self.remapper.apply(planes, warp, self.pool)
        self.stats['corrected'] += 1
        self.stats['warp_px'] = max(self.stats['warp_px'] * 0.98, math.hypot(warp[0], warp[1]))
        elapsed = (time.perf_counter() - started) * 1000
        self.stats['correct_ms'] = elapsed if not self.stats['correct_ms'] else 0.9 * self.stats['correct_ms'] + 0.1 * elapsed
        return out

    def _video_warp(self, luma, capture_ms):
        """Warp from the image's own motion: velocity = flow / interval, displacement over readout."""
        if capture_ms is None:
            return None
        previous = self.sync_previous
        # Above ~30 fps, track every other frame and reuse the latest velocity.
        if previous is not None and 0 < capture_ms - previous[0] < 30 and self.video_velocity is not None:
            self.active_source = 'video'
            return self._velocity_warp(self.video_velocity)
        started = time.perf_counter()
        if previous is not None and not 0 < capture_ms - previous[0] <= 500:
            self.sync_tracker.reset()
            previous = None
        small = cv2.resize(luma, (FLOW_W, FLOW_H), interpolation=cv2.INTER_AREA)
        rows = self.sync_tracker.track(small)
        interval = capture_ms - previous[0] if previous is not None else 0
        o = observe(rows, self.width, self.height, capture_ms, interval, self.sync_tracker.frame)
        elapsed = (time.perf_counter() - started) * 1000
        self.stats['video_flow_ms'] = elapsed if not self.stats['video_flow_ms'] else 0.9 * self.stats['video_flow_ms'] + 0.1 * elapsed
        self.sync_previous = (capture_ms, o)
        if o is None or o['n'] < 30:
            self.stats['no_flow'] += 1
            self.video_velocity = None
            return None
        self.obs_queue.append(o)   # readout and calibration bookkeeping happen on the flow thread
        self.wake.set()
        per_second = lambda ob: [ob['dx'] * 1000 / ob['interval_ms'], ob['dy'] * 1000 / ob['interval_ms'],
                                 ob['roll'] * 1000 / ob['interval_ms']]
        velocity = per_second(o)
        last = previous[1] if previous is not None else None
        if last is not None and last['frame'] + 1 == o['frame']:
            # Flow k-1 -> k is centred half a frame back; extrapolate to this frame.
            velocity = [v + 0.5 * (v - b) for v, b in zip(velocity, per_second(last))]
        self.video_velocity = velocity
        self.active_source = 'video'
        return self._velocity_warp(velocity)

    def _velocity_warp(self, velocity):
        readout, _ = self.readout()
        seconds = readout / 1000
        vx, vy, vroll = (v * seconds for v in velocity)
        size = math.hypot(vx, vy)
        if size > MAX_VIDEO_WARP_PX:
            vx, vy = vx * MAX_VIDEO_WARP_PX / size, vy * MAX_VIDEO_WARP_PX / size
        return vx, vy, max(-MAX_VIDEO_ROLL, min(MAX_VIDEO_ROLL, vroll))

    # --- flow + calibration thread
    def _flow_loop(self):
        previous = None
        while not self.stopped:
            self.wake.wait(0.5)
            self.wake.clear()
            while self.obs_queue:
                self._record(self.obs_queue.popleft())
            job, self.pending = self.pending, None
            if job is None or not self.enabled:
                if not self.enabled:
                    previous = None
                continue
            small, capture_ms = job
            started = time.perf_counter()
            if previous is not None and not 0 < capture_ms - previous <= 500:
                self.tracker.reset()
            rows = self.tracker.track(small)
            interval = capture_ms - previous if previous is not None else 0
            previous = capture_ms
            o = observe(rows, self.width, self.height, capture_ms, interval, self.tracker.frame)
            elapsed = (time.perf_counter() - started) * 1000
            self.stats['flow_ms'] = elapsed if not self.stats['flow_ms'] else 0.9 * self.stats['flow_ms'] + 0.1 * elapsed
            if o is not None:
                self._record(o)

    def _record(self, o):
        """Readout estimate and IMU calibration bookkeeping for one flow observation (flow thread)."""
        self.video_readout.add(o)
        self.observations.append(o)
        self.since_fit += 1
        if self.since_fit >= 30:
            self.since_fit = 0
            fit = fit_calibration(list(self.observations), self.gyro, self.width, self.height, self.max_readout_ms)
            self.last_fit = fit
            # One poor window must not replace a good stored calibration.
            if fit['ok'] and (self.calibration is None or fit['calibration']['fit'] >= self.calibration['fit'] - 0.05):
                self.calibration = fit['calibration']
                self._save()

    def stop(self):
        self.stopped = True
        self.wake.set()
        self.pool.shutdown(wait=False)

    def status(self):
        first, last = self.gyro.span()
        now = time.time() * 1000
        readout, source = self.readout()
        fit = self.last_fit
        return {
            'enabled': self.enabled,
            'source': self.source,
            'active_source': self.active_source,
            'calibrated': self.calibration is not None,
            'calibration': self.calibration,
            'last_fit': None if fit is None else {k: v for k, v in fit.items() if k != 'calibration'},
            'samples': len(self.observations),
            'gyro': {'samples': len(self.gyro), 'age_ms': None if not len(self.gyro) else now - last,
                     'rate_hz': self._gyro_rate()},
            'readout_ms': readout, 'readout_source': source,
            'video_readout': self.video_readout.estimate(),
            'stats': dict(self.stats),
        }

    def _gyro_rate(self):
        with self.gyro.lock:
            t = self.gyro.t[-60:]
        return (len(t) - 1) * 1000 / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0.0
