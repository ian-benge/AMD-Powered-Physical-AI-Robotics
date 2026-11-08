"""A bounded browser control session. It never rearms the rover by itself."""
import math
import threading
import time
import uuid

from .safety import SafetyGate


SPEED_LIMITS = {
    'linear': dict(min=.01, max=SafetyGate.MAX_LINEAR_SPEED, step=.01, default=.2),
    'angular': dict(min=.05, max=SafetyGate.MAX_ANGULAR_SPEED, step=.05, default=.3),
}


class ManualInput:
    def __init__(self, timeout=.25, max_linear=None, max_angular=None, initial_linear=None):
        self.limits = {axis:dict(rule) for axis,rule in SPEED_LIMITS.items()}
        for axis,maximum in [('linear',max_linear),('angular',max_angular)]:
            if maximum is None:
                continue
            rule=self.limits[axis]
            if (isinstance(maximum,bool) or not isinstance(maximum,(int,float)) or
                    not math.isfinite(maximum) or not rule['min']<=maximum<=rule['max']):
                raise ValueError('Manual speed profile must stay within retained guard limits')
            rule['max']=float(maximum)
            rule['default']=min(rule['default'],rule['max'])
        if initial_linear is not None:
            rule=self.limits['linear']
            if (isinstance(initial_linear,bool) or not isinstance(initial_linear,(int,float)) or
                    not math.isfinite(initial_linear) or not rule['min']<=initial_linear<=rule['max']):
                raise ValueError('Initial manual speed must lie within the selected profile')
            rule['default']=float(initial_linear)
        self.lock = threading.Lock()
        self.timeout = timeout
        self.token = None
        self.sequence = -1
        self.received = 0.
        self.moving = False

    def arm(self):
        with self.lock:
            self.token = uuid.uuid4().hex
            self.sequence, self.received = -1, time.monotonic()
            self.moving = False
            return self.token

    def release(self, token=None):
        with self.lock:
            if token is None or token == self.token:
                self.token = None

    def command(self, data, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if not self.token or data.get('session') != self.token:
                raise ValueError('Enable manual control again')
            # Idle ownership tolerates short UI/render pauses. Once a nonzero
            # command is accepted, the original strict input lease applies,
            # including to a delayed zero. This cannot revive expired motion.
            if now-self.received > (self.timeout if self.moving else 10.):
                self.token = None
                raise ValueError('Control input expired; enable manual control again')
            sequence = data.get('sequence')
            v, w = data.get('linear'), data.get('angular')
            if (isinstance(sequence, bool) or not isinstance(sequence, int) or sequence <= self.sequence or
                    any(isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) for x in (v, w)) or
                    not 0 <= v <= self.limits['linear']['max'] or abs(w) > self.limits['angular']['max']):
                self.token = None
                raise ValueError(f"Commands require increasing sequence, forward speed [0,{self.limits['linear']['max']:.2f}], "
                                 f"and turn rate [-{self.limits['angular']['max']:.2f},{self.limits['angular']['max']:.2f}]")
            self.sequence, self.received = sequence, now
            self.moving = bool(v or w)
            return float(v), float(w)
