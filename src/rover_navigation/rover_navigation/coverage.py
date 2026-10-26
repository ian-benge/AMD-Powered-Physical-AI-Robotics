"""Measured camera coverage, independent of terrain memory and launch seeding."""
import math
import numpy as np

from .core import BODY


def image_detail(bgr):
    """Low-cost visual-detail indicator, not an odometry confidence or safety input."""
    pixels = np.asarray(bgr)[::16, ::16].astype(float)
    if pixels.ndim != 3 or pixels.shape[2] != 3 or not pixels.size:
        raise ValueError('Expected a nonempty BGR image')
    gray = pixels @ np.array([.114, .587, .299])
    deviation, span = float(gray.std()), float(np.ptp(gray))
    return dict(low_detail=deviation < 3. and span < 12., sampled_luma_std=deviation)


def ground_view(camera):
    """Flat-ground prediction only. Clip planes bound optical Z, not horizontal range."""
    pitch = float(camera['tilt_rad'])
    height = float(camera['optical_center_height_m'])
    near, far = float(camera['near_m']), float(camera['far_m'])
    hfov = float(camera['horizontal_fov_rad'])
    width, rows = int(camera['width']), int(camera['height'])
    if (not all(math.isfinite(v) for v in (pitch, height, near, far, hfov)) or
            not 0 <= pitch < math.pi/2 or not height > 0 or not 0 < near < far or
            not 0 < hfov < math.pi or min(width, rows) <= 0):
        raise ValueError('Invalid camera geometry for ground coverage')
    vertical = math.tan(hfov/2)*rows/width
    s, c = math.sin(pitch), math.cos(pitch)
    lo, hi = max(-vertical, (height/far-s)/c), min(vertical, (height/near-s)/c)
    if lo > hi:
        return dict(pitch_rad=pitch, model='level rover on a flat plane', near_base_m=None, far_base_m=None)

    def x(t):
        return float(camera['mount_xyz'][0])+height*(c-s*t)/(s+c*t)

    return dict(pitch_rad=pitch, pitch_deg=math.degrees(pitch), model='level rover on a flat plane',
                near_base_m=x(hi), far_base_m=x(lo), camera_height_m=height)


def coverage_profile(frame, base_pose, length=2.5, spacing=.05, margin=.025):
    """Sample a padded-width forward corridor in THIS depth frame, before memory/seed.

    A strip is observed only when every sample is known. Hazard strips stay
    distinct from unobserved strips. This diagnostic never authorizes movement.
    """
    x, y, yaw = base_pose
    if not all(math.isfinite(v) for v in base_pose) or length <= 0 or spacing <= 0:
        raise ValueError('Invalid coverage pose or dimensions')
    front, half = BODY[0]+margin, BODY[2]+margin
    count = int(math.ceil(length/spacing))
    edges = np.linspace(front, front+length, count+1)
    # Use the same fine sampling scale as the guard; a narrow unknown hole must
    # not disappear simply because the display's strips are five cm long.
    lateral = np.linspace(-half, half, int(math.ceil(2*half/(frame.resolution/2)))+1)
    fractions, bands, first = [], [], None
    c, s = math.cos(yaw), math.sin(yaw)

    def moved(dx, dy):
        return x+c*dx-s*dy, y+s*dx+c*dy

    for low, high in zip(edges[:-1], edges[1:]):
        forward = np.linspace(low, high, max(2, int(math.ceil((high-low)/(frame.resolution/2)))+1))
        dx, dy = np.meshgrid(forward, lateral)
        px, py = moved(dx, dy)
        cols = np.floor((px-frame.x)/frame.resolution).astype(int)
        rows = np.floor((py-frame.y)/frame.resolution).astype(int)
        inside = (rows >= 0) & (rows < frame.data.shape[0]) & (cols >= 0) & (cols < frame.data.shape[1])
        values = np.full(rows.shape, -1, np.int16)
        values[inside] = frame.data[rows[inside], cols[inside]]
        known, hazard = values >= 0, values >= 90
        if known.all() and first is None:
            first = float(low-front)
        kind = 'hazard' if hazard.any() else 'unknown' if not known.all() else 'clear'
        fractions.append([float(known.mean()), float(hazard.mean())])
        # Merge adjacent strips of the same kind to bound the live JSON overlay.
        if bands and bands[-1]['kind'] == kind:
            bands[-1]['far_m'] = float(high)
        else:
            bands.append(dict(kind=kind, near_m=float(low), far_m=float(high)))
    for band in bands:
        band['polygon'] = [list(moved(dx, dy)) for dx, dy in
                           ((band['near_m'], half), (band['far_m'], half),
                            (band['far_m'], -half), (band['near_m'], -half))]
    return dict(frame='odom', source='single_depth_frame_before_memory_and_seed',
                base_pose=list(base_pose), corridor_width_m=2*half, length_m=length,
                first_observed_strip_ahead_of_front_m=first,
                observed_sample_fraction=float(np.mean(fractions, axis=0)[0]),
                hazard_sample_fraction=float(np.mean(fractions, axis=0)[1]), bands=bands)
