"""A bounded browser control session. It never rearms the rover by itself."""
import math
import threading
import time
import uuid


class ManualInput:
    def __init__(self, timeout=.25):
        self.lock = threading.Lock()
        self.timeout = timeout
        self.token = None
        self.sequence = -1
        self.received = 0.

    def arm(self):
        with self.lock:
            self.token = uuid.uuid4().hex
            self.sequence, self.received = -1, time.monotonic()
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
            if now-self.received > (10. if self.sequence == -1 else self.timeout):
                self.token = None
                raise ValueError('Control input expired; enable manual control again')
            sequence = data.get('sequence')
            v, w = data.get('linear'), data.get('angular')
            if (isinstance(sequence, bool) or not isinstance(sequence, int) or sequence <= self.sequence or
                    any(isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) for x in (v, w)) or
                    not 0 <= v <= .05 or abs(w) > .3):
                self.token = None
                raise ValueError('Commands require increasing sequence, forward speed [0,0.05], and turn rate [-0.3,0.3]')
            self.sequence, self.received = sequence, now
            return float(v), float(w)
