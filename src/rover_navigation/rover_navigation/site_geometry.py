"""Bounded RGB-D evidence for object tracks; never produces navigation costs."""
import math
import numpy as np

from .core import rotation


def depth_image(data, encoding, width, height, step, bigendian):
    kinds = {'32FC1': ('f4', 1.), '16UC1': ('u2', .001)}
    if encoding not in kinds:
        raise ValueError('Unsupported depth encoding')
    kind, scale = kinds[encoding]
    dtype = np.dtype(('>' if bigendian else '<') + kind)
    if min(width, height) <= 0 or step < width*dtype.itemsize or len(data) != step*height:
        raise ValueError('Invalid depth buffer')
    return np.ndarray((height, width), dtype=dtype, buffer=bytes(data),
                      strides=(step, dtype.itemsize)).astype(float)*scale


def covariance_floor(covariance, floor=.05):
    c = np.asarray(covariance, float)
    if c.shape != (3, 3) or not np.isfinite(c).all():
        raise ValueError('Invalid position covariance')
    eigenvalues, vectors = np.linalg.eigh((c+c.T)/2)
    if eigenvalues.min() < -1e-8:
        raise ValueError('Negative position covariance')
    return (vectors*np.maximum(eigenvalues, floor**2)) @ vectors.T


def pose_covariance(values):
    """Validate the full covariance, including uncertainty unavailable to TF."""
    c = np.asarray(values, float)
    if c.size != 36 or not np.isfinite(c).all():
        raise ValueError('Invalid localization covariance')
    c = c.reshape(6, 6)
    if not np.allclose(c, c.T, atol=1e-7) or np.linalg.eigvalsh(c).min() < -1e-8:
        raise ValueError('Invalid localization covariance')
    if np.linalg.eigvalsh(c[:3, :3]).max() > .10**2 or c[5, 5] > math.radians(5)**2:
        raise ValueError('Localization uncertain (position >0.10 m or yaw >5 degrees)')
    return c


def color_histogram(bgr, box):
    h, w = bgr.shape[:2]
    x1, y1, x2, y2 = np.asarray(box, int)
    patch = bgr[max(0, y1):min(h, y2):2, max(0, x1):min(w, x2):2].astype(float)
    if patch.size == 0:
        raise ValueError('Empty appearance patch')
    # Chromaticity makes this small descriptor less sensitive to exposure.
    rgb = patch[..., ::-1]
    normalized = rgb/np.maximum(1., rgb.sum(axis=2, keepdims=True))
    bins = np.clip((normalized[..., :2]*8).astype(int), 0, 7)
    histogram = np.bincount((bins[..., 0]*8+bins[..., 1]).ravel(), minlength=64).astype(float)
    return (histogram/histogram.sum()).tolist()


def _points(depth, k, box, expand=0.):
    h, w = depth.shape
    x1, y1, x2, y2 = box
    dx, dy = (x2-x1)*expand, (y2-y1)*expand
    left, right = max(0, math.floor(x1-dx)), min(w, math.ceil(x2+dx))
    top, bottom = max(0, math.floor(y1-dy)), min(h, math.ceil(y2+dy))
    stride = max(1, math.ceil(math.sqrt(max(1, (right-left)*(bottom-top))/2048)))
    v, u = np.mgrid[top:bottom:stride, left:right:stride]
    z = depth[v, u]
    valid = np.isfinite(z) & (z >= .3) & (z <= 8.)
    xyz = np.stack(((u-k[2])*z/k[0], (v-k[5])*z/k[4], z), axis=-1)
    inside = (u >= x1) & (u < x2) & (v >= y1) & (v < y2)
    return xyz[valid], inside[valid], int(inside.sum()), valid, u, v


def supporting_plane(points):
    """Deterministic RANSAC, rejecting steep, sparse or ambiguous support."""
    if len(points) < 30:
        return None
    rng = np.random.default_rng(1021)
    best = None
    for _ in range(48):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal = normal/norm*(1 if normal[2] >= 0 else -1)
        if normal[2] < math.cos(math.radians(20)):
            continue
        offset = -float(normal @ a)
        mask = np.abs(points @ normal+offset) <= .015
        if best is None or mask.sum() > best.sum():
            best = mask
    if best is None or best.sum() < 30 or best.mean() < .5:
        return None
    center = np.mean(points[best], axis=0)
    _, singular, axes = np.linalg.svd(points[best]-center, full_matrices=False)
    if singular[1] < .02:
        return None
    normal = axes[-1]*(1 if axes[-1, 2] >= 0 else -1)
    offset = -float(normal @ center)
    fraction = float((np.abs(points @ normal+offset) <= .015).mean())
    if normal[2] < math.cos(math.radians(20)) or fraction < .5:
        return None
    return normal, offset, fraction


def measure(depth, k, box, translation, quaternion, localization_covariance, surface=False):
    """Return odom measurement, conservative covariance and independent relief test."""
    k, box = np.asarray(k, float), np.asarray(box, float)
    h, w = depth.shape
    if (k.shape != (9,) or not np.isfinite(k).all() or min(k[0], k[4]) <= 0 or
            box.shape != (4,) or not np.isfinite(box).all() or
            not (0 <= box[0] < box[2] <= w and 0 <= box[1] < box[3] <= h)):
        raise ValueError('Invalid aligned box/calibration')
    camera_rotation = rotation(quaternion)
    translation = np.asarray(translation, float)
    points, inside, total, _, _, _ = _points(depth, k, box, .5)
    core = points[inside]
    if len(core) < 9 or len(core) < .2*total:
        raise ValueError('Insufficient valid depth inside detection')
    world = points @ camera_rotation.T+translation
    plane = supporting_plane(world[~inside])
    evidence = dict(supported=False, contradiction=False, reason='insufficient surrounding ground geometry')
    selected = core
    envelope = None
    if surface:
        if plane is None or len(core) < .8*total or min(box[0], box[1], w-box[2], h-box[3]) < 3:
            raise ValueError('Soil boundary is clipped or lacks dense ground support')
        normal, offset, fraction = plane
        flat = np.abs(world[inside] @ normal+offset) <= .015
        if normal[2] < math.cos(math.radians(8)) or flat.mean() < .8:
            raise ValueError('Soil requires near-planar ground below 8 degrees')
        rays = np.array([[(u-k[2])/k[0], (v-k[5])/k[4], 1.]
                         for u,v in ((box[0],box[1]),(box[2],box[1]),(box[2],box[3]),(box[0],box[3]))]) @ camera_rotation.T
        denominator = rays @ normal
        if np.any(np.abs(denominator) < 1e-8):
            raise ValueError('Soil boundary has an unbounded ground projection')
        distances = -(normal @ translation+offset)/denominator
        if np.any((distances < .3) | (distances > 8.)):
            raise ValueError('Soil boundary outside usable depth')
        envelope = (translation+rays*distances[:,None])[:,:2].tolist()
        evidence.update(supported=True, contradiction=False, plane_inlier_fraction=fraction,
                        flat_fraction=float(flat.mean()), reason='multi-view ground-surface support')
        selected = core[flat]
    elif plane is not None:
        normal, offset, fraction = plane
        heights = world[inside] @ normal+offset
        raised = heights >= .03
        evidence.update(plane_inlier_fraction=fraction, raised_fraction=float(raised.mean()),
                        supported=bool(raised.mean() >= .2), contradiction=bool(raised.mean() < .2),
                        reason='raised geometry supported' if raised.mean() >= .2 else 'insufficient raised geometry')
        if evidence['supported']:
            selected = core[raised]
    # Median surface location, not a collection pose or inferred object center.
    optical = np.median(selected, axis=0)
    point = camera_rotation @ optical+translation
    spread = 1.4826*np.median(np.abs(selected-np.median(selected, axis=0)), axis=0)
    z = optical[2]
    u = k[0]*optical[0]/z+k[2]
    v = k[4]*optical[1]/z+k[5]
    jacobian = np.array([[z/k[0], 0., (u-k[2])/k[0]],
                         [0., z/k[4], (v-k[5])/k[4]], [0., 0., 1.]])
    optical_cov = jacobian @ np.diag([4., 4., max(.01, spread[2])**2]) @ jacobian.T
    optical_cov += np.diag(spread**2)  # object surfaces vary with viewpoint
    relative = point-translation
    x, y, z = relative
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    c = np.asarray(localization_covariance, float)
    pose_jacobian = np.column_stack((np.eye(3), -skew))
    cov = camera_rotation @ optical_cov @ camera_rotation.T+pose_jacobian @ c @ pose_jacobian.T
    extent = np.maximum(.03, np.percentile(selected, 95, axis=0)-np.percentile(selected, 5, axis=0))
    result = dict(odom_position=point.tolist(), covariance=covariance_floor(cov).tolist(),
                  extent=extent.tolist(), geometry=evidence)
    if envelope is not None:
        result['envelope_odom'] = envelope
        result['envelope_radius_m'] = max(np.linalg.norm(np.asarray(envelope)-point[:2],axis=1))
    return result


def visible_region(item, depth, k, translation, quaternion):
    """Missing detection is evidence only in a fully visible, depth-supported region."""
    r = rotation(quaternion)
    point = (np.asarray(item['odom_position'])-translation) @ r
    c = r.T @ np.asarray(item['covariance']) @ r
    radius = max(.03, float(np.linalg.norm(item['extent']))/2)
    uncertainty = 2*math.sqrt(max(0., np.linalg.eigvalsh(c).max()))
    extent = radius+uncertainty
    if point[2]-extent < .3 or point[2]+extent > 8.:
        return False, 'outside depth range'
    # Project all corners of a conservative enclosing cube.
    corners = np.array([point+[x, y, z] for x in (-extent, extent)
                        for y in (-extent, extent) for z in (-extent, extent)])
    us, vs = k[0]*corners[:, 0]/corners[:, 2]+k[2], k[4]*corners[:, 1]/corners[:, 2]+k[5]
    h, w = depth.shape
    if us.min() < 3 or vs.min() < 3 or us.max() >= w-3 or vs.max() >= h-3:
        return False, 'outside full camera view'
    u, v = k[0]*point[0]/point[2]+k[2], k[4]*point[1]/point[2]+k[5]
    # Object-scale patch, not the uncertainty-expanded background around it.
    du, dv = max(2, k[0]*radius/point[2]), max(2, k[4]*radius/point[2])
    patch = depth[max(0, int(v-dv)):min(h, int(v+dv)+1), max(0, int(u-du)):min(w, int(u+du)+1)]
    valid = patch[np.isfinite(patch) & (patch >= .3) & (patch <= 8.)]
    if len(valid) < 9 or len(valid) < .8*patch.size:
        return False, 'insufficient visibility depth'
    if np.percentile(valid, 10) < point[2]-extent-.03:
        return False, 'occluded'
    return True, 'visible with healthy depth'
