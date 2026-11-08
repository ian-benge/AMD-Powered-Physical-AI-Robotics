#!/usr/bin/env python3
"""Transport-only fixture pose client; no camera subscriptions or navigation."""
import json
import math
import os
import re
import sys


def validate_pose(x,yaw):
    x,yaw=float(x),float(yaw)
    if not (math.isfinite(x) and math.isfinite(yaw) and 0<=x<=1.8 and abs(yaw)<=.35):
        raise ValueError('Pose is outside the isolated evaluator trajectory')
    return x,yaw


def main():
    if not re.fullmatch(r'rover-phase1-[a-z0-9-]+',os.environ.get('GZ_PARTITION','')):
        raise RuntimeError('Isolated fixture partition required')
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node
    node=Node()
    for line in sys.stdin:
        try:
            values=json.loads(line); x,yaw=validate_pose(values['x'],values['yaw'])
            request=Pose(); request.name='fixture_camera'
            request.position.x=x; request.position.z=.762
            request.orientation.x=-math.sin(yaw/2)*math.sin(.2)
            request.orientation.y=math.cos(yaw/2)*math.sin(.2)
            request.orientation.z=math.sin(yaw/2)*math.cos(.2)
            request.orientation.w=math.cos(yaw/2)*math.cos(.2)
            ok,reply=node.request('/world/visibility_fixture/set_pose',request,Pose,Boolean,3000)
            result=dict(accepted=bool(ok and reply.data),transport_ok=bool(ok),reply_data=bool(reply.data),x=x,yaw=yaw)
        except Exception as error:
            result=dict(accepted=False,error=str(error))
        print(json.dumps(result),flush=True)
    return 0


if __name__=='__main__': raise SystemExit(main())
