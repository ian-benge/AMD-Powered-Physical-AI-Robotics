"""Measured-pose brackets for geometry-qualified isolated camera frames."""
import math


def measured_pose_bracket(poses, timestamp):
    before=max((p for p in poses if p[0]<timestamp),key=lambda p:p[0],default=None)
    after=min((p for p in poses if p[0]>timestamp),key=lambda p:p[0],default=None)
    if before is None or after is None:
        return None
    observed=min((before,after),key=lambda p:abs(p[0]-timestamp))
    a,b=before[1]+before[2],after[1]+after[2]
    stable=(len(a)==len(b)==7 and all(math.isfinite(v) for v in a+b)
            and all(abs(x-y)<=1e-9 for x,y in zip(a,b))
            and max(timestamp-before[0],after[0]-timestamp)*1e-9<=.05)
    return dict(pose=observed,gap_s=abs(observed[0]-timestamp)*1e-9,
                before=before,after=after,stable=stable)
