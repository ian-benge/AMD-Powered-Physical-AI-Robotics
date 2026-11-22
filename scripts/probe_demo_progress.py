#!/usr/bin/env python3
"""Build and replay the installed Nav2 progress plugins without GPU or motion."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--events', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    events = [json.loads(line) for line in args.events.read_text().splitlines()]
    trajectory = [e for e in events if e.get('kind') == 'trajectory' and e.get('phase') == 'exploration_return']
    if len(trajectory) < 50:
        raise ValueError('Preserve a sufficiently sampled recorded return')
    with tempfile.TemporaryDirectory(prefix='nav2-progress-') as temporary:
        root = Path(temporary)
        shutil.copyfile(Path(__file__).with_suffix('.cpp'), root / 'probe.cpp')
        (root / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(progress_probe)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
add_executable(probe probe.cpp)
ament_target_dependencies(probe rclcpp rclcpp_lifecycle geometry_msgs nav2_core pluginlib)
''')
        first = trajectory[0]['wall_monotonic']
        csv = root / 'return.txt'
        csv.write_text(''.join(' '.join(str(v) for v in
            (entry['wall_monotonic'] - first, *entry['odom_pose'])) + '\n' for entry in trajectory))
        subprocess.run(['cmake', '-S', str(root), '-B', str(root / 'build')], check=True)
        subprocess.run(['cmake', '--build', str(root / 'build'), '-j', '2'], check=True)
        result = subprocess.run([str(root / 'build/probe'), str(csv)], check=True, capture_output=True, text=True)
        report = json.loads(result.stdout.splitlines()[-1])
        report.update(events=str(args.events), events_sha256=hashlib.sha256(args.events.read_bytes()).hexdigest(),
                      actual_installed_plugins=True, gpu_used=False, motion_requested=False,
                      final_pose_hold_seconds=.25, scope='Plugin replay, not a live home-return result')
        with args.output.open('x') as stream:
            json.dump(report, stream, indent=2)
            stream.write('\n')
        print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
