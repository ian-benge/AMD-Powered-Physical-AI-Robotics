#!/usr/bin/env python3
"""Exact-stamp RGB-D and observed poses; isolated evaluator fixture only."""
import argparse
from collections import OrderedDict, deque
import datetime as dt
import json
import math
import os
from pathlib import Path
import re
import selectors
import signal
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
from phase1_pose_sync import measured_pose_bracket


def stamp(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nsec


def camera_intrinsics(document):
    k=document['intrinsics']['k']
    if len(k)!=9 or not all(math.isfinite(v) for v in k) or min(k[0],k[4])<=0:
        raise ValueError('Invalid fixture calibration')
    if any(document.get('distortion',{}).get('k',[])):
        raise ValueError('Fixture requires a calibrated pinhole camera without distortion')
    width,height=int(document['width']),int(document['height'])
    if min(width,height)<=0: raise ValueError('Invalid calibrated image dimensions')
    return dict(width=width,height=height,fx=k[0],fy=k[4],cx=k[2],cy=k[5])


def decode(rgb, depth):
    # gz-msgs10 PixelFormatType: RGB_INT8=3, R_FLOAT32=13. Reject silently
    # misinterpreted channel orders / element types and padded-row mistakes.
    if rgb.pixel_format_type != 3 or depth.pixel_format_type != 13:
        raise ValueError('Expected RGB_INT8 / R_FLOAT32 fixture output')
    if (rgb.width, rgb.height) != (depth.width, depth.height):
        raise ValueError('RGB and depth dimensions differ')
    if rgb.step < rgb.width*3 or depth.step < depth.width*4 or depth.step % 4:
        raise ValueError('Invalid RGB-D row strides')
    pixels = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.step)
    pixels = pixels[:, :rgb.width*3].reshape(rgb.height, rgb.width, 3)
    values = np.frombuffer(depth.data, '<f4').reshape(depth.height, depth.step//4)[:, :depth.width]
    return pixels, values


def trajectory(mode, elapsed):
    x = .9*(1-math.cos(2*math.pi*elapsed/120)) if mode == 'approach_retreat' else 0.
    yaw = .35*math.sin(2*math.pi*elapsed/60) if mode == 'rotation' else 0.
    return x, yaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--mode', choices=['stationary', 'approach_retreat', 'rotation'], default='stationary')
    args = parser.parse_args()
    if not 0 < args.seconds <= 660:
        parser.error('duration must be in (0, 660]')
    if not re.fullmatch(r'rover-phase1-[a-z0-9-]+', os.environ.get('GZ_PARTITION', '')):
        parser.error('An isolated rover-phase1-* GZ_PARTITION is required')
    from gz.msgs10.camera_info_pb2 import CameraInfo
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
    from google.protobuf.json_format import MessageToJson

    args.output.mkdir(parents=True, exist_ok=False)
    node, lock = Node(), threading.Lock()
    images, depths, poses = OrderedDict(), OrderedDict(), deque(maxlen=300)
    calibration=[]
    counts = dict(rgb=0, depth=0, uniform_rgb=0, empty_depth=0, matched=0,
                  unsynchronized_pose=0, unstable_pose_bracket=0)
    interrupted = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: interrupted.set())

    def image_callback(message, buffer, key):
        with lock:
            counts[key] += 1
            buffer[stamp(message)] = message
            while len(buffer) > 60:
                buffer.popitem(last=False)

    def pose(message):
        with lock:
            for value in message.pose:
                if value.name == 'fixture_camera':
                    p, q = value.position, value.orientation
                    poses.append((stamp(value) or stamp(message), [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))

    def info(message):
        with lock:
            if not calibration: calibration.append(MessageToJson(message))

    def place(x, yaw):
        # Keep synchronous RPC out of the process receiving high-rate Python
        # image callbacks. A separate transport-only client prevents callback
        # workers from starving the request/reply path while it waits.
        request=dict(x=x,yaw=yaw)
        controller.stdin.write(json.dumps(request)+'\n'); controller.stdin.flush()
        with selectors.DefaultSelector() as selector:
            selector.register(controller.stdout,selectors.EVENT_READ)
            if not selector.select(timeout=4):
                response=dict(accepted=False,error='Pose client response timeout')
            else:
                response=json.loads(controller.stdout.readline())
        requests.write(json.dumps(dict(request=request,response=response))+'\n'); requests.flush()
        if not response.get('accepted'):
            raise RuntimeError('Isolated fixture pose request failed: '+json.dumps(response))

    frames, started, failure, complete = 0, None, None, False
    controller=controller_log=requests=None
    try:
        controller_log=(args.output/'pose-controller.log').open('x')
        requests=(args.output/'pose-requests.jsonl').open('x')
        controller=subprocess.Popen([sys.executable,str(Path(__file__).with_name('phase1_pose_controller.py'))],
                                    stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=controller_log,text=True)
        if not all((node.subscribe(Image, '/fixture/image', lambda m: image_callback(m, images, 'rgb')),
                    node.subscribe(Image, '/fixture/depth_image', lambda m: image_callback(m, depths, 'depth')),
                    node.subscribe(CameraInfo, '/fixture/camera_info', info),
                    node.subscribe(Pose_V, '/model/fixture_camera/pose', pose))):
            raise RuntimeError('Fixture subscriptions failed')
        place(0., 0.)
        warmup = time.monotonic()+30
        while time.monotonic() < warmup:
            if interrupted.is_set():
                raise RuntimeError('Interrupted during warmup')
            with lock:
                ready_stamp=next((t for t in sorted(set(images).intersection(depths))
                                  if (bracket:=measured_pose_bracket(poses,t)) and bracket['stable']),None)
                ready = bool(calibration and ready_stamp is not None)
            if ready:
                break
            time.sleep(.01)
        else:
            raise RuntimeError('No synchronized fixture RGB-D / measured poses during warmup')
        intrinsics=camera_intrinsics(json.loads(calibration[0]))
        if (intrinsics['width'],intrinsics['height'])!=(960,540):
            raise ValueError('Unexpected fixture calibration dimensions')
        (args.output/'camera-info.json').write_text(calibration[0])
        started = next_save = last_pair_wall = time.monotonic()
        last_stamp, last_action = ready_stamp-1, -1.
        with (args.output/'frames.jsonl').open('x') as records, \
             (args.output/'actions.jsonl').open('x') as actions, \
             (args.output/'frame-health.jsonl').open('x') as health:
            while time.monotonic()-started < args.seconds:
                if interrupted.is_set():
                    raise RuntimeError('Interrupted; trial incomplete')
                elapsed = time.monotonic()-started
                if args.mode != 'stationary' and elapsed-last_action >= .1:
                    x, yaw = trajectory(args.mode, elapsed)
                    place(x, yaw)
                    actions.write(json.dumps(dict(wall_elapsed=elapsed, x=x, yaw=yaw, accepted=True))+'\n')
                    actions.flush(); last_action = elapsed
                with lock:
                    common = sorted(t for t in images if t in depths and t > last_stamp)
                    pairs = [(t, images[t], depths[t]) for t in common]
                    pose_copy = list(poses)
                for ts, rgb_message, depth_message in pairs:
                    if ts <= last_stamp:
                        continue
                    bracket=measured_pose_bracket(pose_copy,ts)
                    if bracket is None:
                        break  # Wait for the measured pose after this image.
                    if last_stamp >= 0 and (ts-last_stamp)*1e-9 > 1.:
                        raise RuntimeError('More than one simulation second missing from RGB-D')
                    last_stamp = ts; last_pair_wall = time.monotonic()
                    pixels, values = decode(rgb_message, depth_message)
                    detail = float(pixels[::8, ::8].std(axis=(0, 1)).max())
                    usable = np.isfinite(values) & (values > .3) & (values <= 8)
                    observed,gap=bracket['pose'],bracket['gap_s']
                    counts['matched'] += 1
                    counts['uniform_rgb'] += int(detail < 1)
                    counts['empty_depth'] += int(not usable.any())
                    counts['unsynchronized_pose'] += int(gap > .05)
                    counts['unstable_pose_bracket'] += int(not bracket['stable'])
                    health.write(json.dumps(dict(stamp_ns=ts, wall_elapsed=elapsed, luma_detail=detail,
                                                 usable_depth_fraction=float(usable.mean()), pose_gap_s=gap,
                                                 stable_pose_bracket=bracket['stable']))+'\n')
                    if detail < 1 or not usable.any() or gap > .05:
                        health.flush(); os.fsync(health.fileno())
                        raise RuntimeError('Uniform RGB, empty usable depth, or missing synchronized pose')
                    if time.monotonic() < next_save:
                        continue
                    if not bracket['stable']:
                        if time.monotonic()-next_save>1:
                            raise RuntimeError('No unambiguous measured pose for the next saved frame')
                        continue
                    record = dict(index=frames, stamp_ns=ts, utc=dt.datetime.now(dt.timezone.utc).isoformat(),
                                  rgb_format=int(rgb_message.pixel_format_type), depth_format=int(depth_message.pixel_format_type),
                                  luma_detail=detail, finite_depth_fraction=float(usable.mean()), pose=observed,
                                  pose_gap_s=gap, synchronized_pose=True)
                    record.update(pose_bracket=[bracket['before'],bracket['after']],stable_pose_bracket=True)
                    if not cv2.imwrite(str(args.output/f'rgb-{frames:05d}.png'), cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)):
                        raise OSError('RGB file write failed')
                    np.savez_compressed(args.output/f'depth-{frames:05d}.npz', depth=values)
                    records.write(json.dumps(record)+'\n'); records.flush(); os.fsync(records.fileno())
                    health.flush(); os.fsync(health.fileno())
                    frames += 1; next_save += 1
                if time.monotonic()-last_pair_wall > 2:
                    raise RuntimeError('RGB-D capture stalled for two wall seconds')
                time.sleep(.005)
        if interrupted.is_set() or not frames:
            raise RuntimeError('Interrupted or empty recording')
        complete = True
    except Exception as error:
        failure = str(error)
    finally:
        if controller:
            try:
                controller.stdin.close()
            except BrokenPipeError:
                pass  # Preserve the recording summary if the pose client died.
            try: controller.wait(timeout=5)
            except subprocess.TimeoutExpired: controller.kill(); controller.wait(timeout=5)
        if controller_log: controller_log.close()
        if requests: requests.close()
        result = dict(counts=counts, saved_frames=frames, requested_seconds=args.seconds,
                      duration_wall_s=time.monotonic()-started if started else 0., mode=args.mode,
                      capture_complete=complete, failure=failure,
                      pose_policy='equal_measured_poses_bracketing_frame',
                      evaluator_pose_replay=args.mode != 'stationary', guarded_manual_motion=False)
        with (args.output/'summary.json').open('x') as stream:
            json.dump(result, stream, indent=2); stream.flush(); os.fsync(stream.fileno())
        print(json.dumps(result), flush=True)
    return 0 if complete else 1


if __name__ == '__main__':
    raise SystemExit(main())
