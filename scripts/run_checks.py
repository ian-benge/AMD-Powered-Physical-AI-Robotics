#!/usr/bin/env python3
"""Run isolated CPU source suites without ROS, model weights, or AMD hardware."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SUITES = {
    'navigation': ('src/rover_navigation/test', ROOT),
    'perception': ('src/rover_perception/test', ROOT),
    'environment': ('src/mars_training_environment/test', ROOT),
    'training': ('src/rover_training/test', ROOT),
    'webcam': ('tests', ROOT / 'examples/webcam'),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=list(SUITES), action='append',
                        help='Repeat to select suites; default runs all.')
    parser.add_argument('--output-dir', type=Path, help='Optional directory for text logs.')
    args = parser.parse_args()
    env = os.environ.copy()
    paths = [str(ROOT / 'src' / name) for name in
             ('rover_navigation', 'rover_perception', 'mars_training_environment', 'rover_training')]
    if env.get('PYTHONPATH'):
        paths.append(env['PYTHONPATH'])
    env['PYTHONPATH'] = os.pathsep.join(paths)
    env['PYTHONUTF8'] = '1'
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    failed = []
    for name in args.suite or SUITES:
        tests, cwd = SUITES[name]
        print(f'\nRunning {name} source checks', flush=True)
        result = subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', tests],
                                cwd=cwd, env=env, text=True, encoding='utf-8',
                                errors='replace', stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        print(result.stdout, end='', flush=True)
        if args.output_dir:
            (args.output_dir / (name + '.txt')).write_text(result.stdout, encoding='utf-8')
        if result.returncode:
            failed.append(name)
    print('\nSource checks ' + ('failed: ' + ', '.join(failed) if failed else 'passed.'))
    print('Hardware inference, ROS middleware and driving require separate acceptance.')
    return bool(failed)


if __name__ == '__main__':
    raise SystemExit(main())
