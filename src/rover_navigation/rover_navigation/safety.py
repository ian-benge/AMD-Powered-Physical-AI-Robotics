"""Wall-clock control lease and latched faults. No ROS dependency."""
import math


class SafetyGate:
    MAX_LINEAR_SPEED = .5
    MAX_ANGULAR_SPEED = .6

    def __init__(self, command_timeout=.35, sensor_timeout=1., clock_timeout=.5):
        self.command_timeout, self.sensor_timeout, self.clock_timeout = command_timeout, sensor_timeout, clock_timeout
        self.mode, self.reason = 'stopped', 'startup'
        # First non-operator stop since the last successful selection. A later
        # operator Stop changes the reason but must not erase this evidence.
        self.fault = None
        self.commands = {}
        self.sensors = {}
        self.clock_received = -math.inf
        self.sim_stamp = None
        self.clock_reset = False

    def clock(self, stamp, now):
        if self.sim_stamp is not None and stamp < self.sim_stamp:
            self.clock_reset = True
            self.stop('simulation_clock_reset')
            self.sensors.clear()
        if self.sim_stamp != stamp:
            self.clock_received = now
        self.sim_stamp = stamp

    def health(self, sensor, now):
        self.sensors[sensor] = now

    def ready(self, now):
        return (not self.clock_reset and now-self.clock_received <= self.clock_timeout and
                all(now-self.sensors.get(key, -math.inf) <= self.sensor_timeout for key in ('depth', 'pose')))

    def select(self, mode, now):
        if self.clock_reset:
            self.stop('simulation_clock_reset_requires_restart')
            return False
        if mode not in ('manual', 'autonomous') or not self.ready(now):
            self.stop('not_ready')
            return False
        self.mode, self.reason = mode, ''
        self.fault = None
        self.commands.clear()  # takeover requires a NEW command; never replay a lease
        return True

    def command(self, source, v, w, now):
        if not all(math.isfinite(x) for x in (v, w)):
            self.stop('invalid_command')
            return
        self.commands[source] = (max(0., min(self.MAX_LINEAR_SPEED, v)),
                                 max(-self.MAX_ANGULAR_SPEED, min(self.MAX_ANGULAR_SPEED, w)), now)

    def stop(self, reason):
        self.mode, self.reason = 'stopped', reason
        self.commands.clear()
        if reason != 'operator_stop' and self.fault is None:
            self.fault = dict(reason=reason, sim_time=self.sim_stamp)

    def output(self, now, clear):
        if self.mode == 'stopped':
            return 0., 0.
        if not self.ready(now):
            self.stop('sensor_or_clock_timeout')
            return 0., 0.
        v, w, received = self.commands.get(self.mode, (0., 0., -math.inf))
        if now-received > self.command_timeout:
            if v or w:
                self.stop('command_timeout')
            return 0., 0.
        if (v or w) and not clear(v, w):
            self.stop('unsafe_or_unobserved_ground')
            return 0., 0.
        return v, w
