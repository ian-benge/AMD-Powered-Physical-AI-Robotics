"""Read-only map overlays. These values never participate in navigation decisions."""
import numpy as np
import math


def localization_health(sim_time, transform_stamp):
    # Controls deliberately uses wall time for HTTP leases. Compare TF only to
    # /clock, never this node's wall-clock ROS timestamp.
    age = sim_time-transform_stamp if sim_time is not None else None
    return dict(available=True, transform_age_seconds=age,
                healthy=age is not None and math.isfinite(age) and -.05 <= age <= .6)


def map_overlay(status, transform, coverage=None):
    """Move odom geometry into the map's CURRENT correction, including frozen rejections."""
    def points(values):
        p = np.asarray(values, dtype=float)
        return transform(np.column_stack([p, np.zeros(len(p))]))[:, :2].tolist()

    geometry = status.get('geometry') or {}
    result = dict(frame='map', source_frame='odom', stamp=geometry.get('stamp'))
    if coverage and coverage.get('frame') == 'odom':
        result['coverage_stamp'] = coverage['stamp']
        result['coverage'] = [dict(kind=b['kind'], polygon=points(b['polygon'])) for b in coverage['bands']]
    if geometry.get('frame') == 'odom':
        result['footprint'] = points(geometry['footprint'])
        result['sweep'] = [points(p) for p in geometry['sweep']]
    fault = status.get('fault') or {}
    detail = fault.get('detail', {})
    if detail.get('frame') == 'odom' and 'cell_center' in detail:
        result['rejection'] = dict(reason=fault['reason'], causes=detail.get('causes', []),
                                   stamp=detail.get('grid_stamp'), resolution=detail['resolution'],
                                   point=points([[detail['x'], detail['y']]])[0],
                                   cell_center=points([detail['cell_center']])[0],
                                   sweep=[points(p) for p in detail.get('sweep', [])])
        # Transform cell corners too: map/odom corrections can include rotation.
        x, y = detail['cell_center']
        h = detail['resolution']/2
        result['rejection']['cell'] = points([[x-h, y-h], [x+h, y-h], [x+h, y+h], [x-h, y+h]])
    return result


def reason_colors(data):
    """OpenCV BGR colors, fixed legend: gray unknown, orange slope, purple span, pink both."""
    colors = np.full((*data.shape, 3), (99, 85, 74), np.uint8)
    colors[data == 0] = (200, 223, 203)
    colors[(data > 0) & ((data & 2) != 0)] = (45, 165, 255)
    colors[(data > 0) & ((data & 4) != 0)] = (220, 95, 185)
    colors[(data > 0) & ((data & 6) == 6)] = (125, 70, 255)
    colors[(data > 0) & ((data & 8) != 0)] = (70, 75, 225)
    return colors
