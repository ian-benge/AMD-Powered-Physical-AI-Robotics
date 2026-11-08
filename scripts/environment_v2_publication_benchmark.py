"""CPU-only ROS serialization identity and conversion timing at 20/40 m."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message, serialize_message
from rover_navigation.core import Grid
from rover_navigation.ros_utils import grid_message


def retained_message(grid, frame, stamp):
    msg = OccupancyGrid()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.info.resolution = grid.resolution
    msg.info.height, msg.info.width = grid.data.shape
    msg.info.origin.position.x, msg.info.origin.position.y = float(grid.x), float(grid.y)
    msg.info.origin.orientation.w = 1.
    msg.data = grid.data.ravel().tolist()
    return msg


def main(output):
    stamp = Time(sec=7, nanosec=123456789)
    report = dict(result='RUNNING', actual_ros_serialization=True, cases=[], motion_requested=False,
        cdr_padding_bytes_ignored=[40, 41, 42, 43],
        padding_scope='Only the four alignment bytes before the Pose in this fixed map-frame CDR layout; all message fields are compared independently.')
    for n in (400, 800):
        values = np.random.RandomState(n).choice([-1, 0, 15, 80, 100], size=(n, n)).astype(np.int8)
        for label, data in [('contiguous', values), ('noncontiguous', values.T[::-1])]:
            grid = Grid(data, .05, -n*.025, -n*.025)
            expected_message = retained_message(grid, 'map', stamp)
            actual_message = grid_message(grid, 'map', stamp)
            assert actual_message == expected_message, 'Publication fields or cells changed'
            expected, actual = map(serialize_message, (expected_message, actual_message))
            expected_decoded = deserialize_message(expected, OccupancyGrid)
            assert deserialize_message(actual, OccupancyGrid) == expected_decoded
            # CDR alignment padding has no field value and is not initialized by
            # this serializer. Compare every other byte and every decoded field.
            canonical = lambda raw: raw[:40]+raw[44:]
            assert canonical(actual) == canonical(expected), 'Encoded publication fields changed'
            times = {}
            for name, function in [('retained_list_setter', retained_message), ('native_byte_array', grid_message)]:
                samples = []
                for _ in range(15):
                    start = time.perf_counter()
                    message = function(grid, 'map', stamp)
                    samples.append(time.perf_counter()-start)
                assert deserialize_message(serialize_message(message), OccupancyGrid) == expected_decoded
                times[name] = dict(median_seconds=float(np.median(samples)), p95_seconds=float(np.percentile(samples, 95)))
            speedup = times['retained_list_setter']['median_seconds']/times['native_byte_array']['median_seconds']
            assert speedup >= 2., 'Conversion optimization did not improve the measured path'
            report['cases'].append(dict(side=n, layout=label, encoded_field_parity=True,
                serialized_fields_sha256=hashlib.sha256(canonical(actual)).hexdigest(),
                decoded_message_and_cell_parity=True, timing=times, median_speedup=speedup))
    report['result'] = 'EXACT_ROS_FIELDS_AND_GRID_PAYLOAD_PARITY_AND_CONVERSION_TIMING_PASS'
    with output.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    main(parser.parse_args().output)
