import unittest
from unittest.mock import patch
from rover_navigation.manual_input import ManualInput, SPEED_LIMITS
from rover_navigation.safety import SafetyGate


class ManualInputTests(unittest.TestCase):
    def test_larger_world_speed_cap_keeps_conservative_initial_value(self):
        q=ManualInput(max_linear=.2,max_angular=.3,initial_linear=.05)
        self.assertEqual(q.limits['linear']['default'],.05)
        token=q.arm();self.assertEqual(q.command(dict(session=token,sequence=1,linear=.2,angular=0)),(.2,0.))
        for value in (.21,float('nan'),True):
            with self.assertRaises(ValueError):ManualInput(max_linear=.2,initial_linear=value)
    def test_qualification_profile_rejects_cached_higher_speed_without_changing_defaults(self):
        qualified=ManualInput(max_linear=.05,max_angular=.3)
        token=qualified.arm()
        self.assertEqual(qualified.command(dict(session=token,sequence=1,linear=.05,angular=.3)),(.05,.3))
        with self.assertRaises(ValueError):
            qualified.command(dict(session=token,sequence=2,linear=.2,angular=0.))
        self.assertIsNone(qualified.token)
        self.assertEqual(qualified.limits['linear']['default'],.05)
        self.assertEqual(ManualInput().limits['linear']['max'],SafetyGate.MAX_LINEAR_SPEED)
        self.assertEqual(SPEED_LIMITS['linear']['default'],.2)

    def test_manual_profile_cannot_expand_retained_guard_limits(self):
        for limits in [dict(max_linear=.51),dict(max_angular=.61),dict(max_linear=float('nan')),
                       dict(max_linear=True),dict(max_linear=.001),dict(max_angular=.01)]:
            with self.assertRaises(ValueError):ManualInput(**limits)

    def test_held_commands_expire_and_cannot_replay_after_release(self):
        control = ManualInput()
        with patch('rover_navigation.manual_input.time.monotonic', return_value=10.):
            token = control.arm()
        message = dict(session=token, sequence=1, linear=.05, angular=.3)
        self.assertEqual(control.command(message, 11.), (.05, .3))
        message['sequence'] += 1
        with self.assertRaises(ValueError):
            control.command(message, 11.3)
        with self.assertRaises(ValueError):
            control.command(message, 11.31)

    def test_new_selection_invalidates_old_browser_and_reordered_input(self):
        control = ManualInput()
        old = control.arm()
        new = control.arm()
        with self.assertRaises(ValueError):
            control.command(dict(session=old, sequence=1, linear=.05, angular=0.))
        command = dict(session=new, sequence=1, linear=.05, angular=0.)
        control.command(command)
        with self.assertRaises(ValueError):
            control.command(command)
        control.release(new)
        with self.assertRaises(ValueError):
            control.command(dict(command, sequence=2))

    def test_idle_grace_survives_ui_pause_but_moving_lease_remains_strict(self):
        control = ManualInput()
        with patch('rover_navigation.manual_input.time.monotonic', return_value=10.):
            token = control.arm()
        idle = dict(session=token, sequence=1, linear=0., angular=0.)
        control.command(idle, 10.1)
        self.assertEqual(control.command(dict(idle, sequence=2), 10.8), (0., 0.))
        self.assertEqual(control.token, token)
        control.command(dict(idle, sequence=3, linear=.05), 10.9)
        # An idle packet cannot renew a token after motion has expired.
        with self.assertRaisesRegex(ValueError, 'expired'):
            control.command(dict(idle, sequence=4), 11.2)
        self.assertIsNone(control.token)

    def test_idle_grace_is_bounded_and_release_still_revokes_it(self):
        control = ManualInput()
        with patch('rover_navigation.manual_input.time.monotonic', return_value=10.):
            token = control.arm()
        with self.assertRaisesRegex(ValueError, 'expired'):
            control.command(dict(session=token, sequence=1, linear=0., angular=0.), 20.1)
        token = control.arm()
        control.release(token)
        with self.assertRaises(ValueError):
            control.command(dict(session=token, sequence=1, linear=0., angular=0.))

    def test_reverse_nonfinite_and_excess_speed_rejected(self):
        for v, w in ((-.01, 0.), (.501, 0.), (0., .601), (0., -.601),
                     (float('nan'), 0.), (0., float('inf')), (True, 0.)):
            control = ManualInput()
            token = control.arm()
            with self.assertRaises(ValueError):
                control.command(dict(session=token, sequence=1, linear=v, angular=w))

    def test_adjustable_speeds_preserve_session_and_guard_limits(self):
        control = ManualInput()
        token = control.arm()
        self.assertEqual(SPEED_LIMITS['linear']['max'], SafetyGate.MAX_LINEAR_SPEED)
        self.assertEqual(SPEED_LIMITS['angular']['max'], SafetyGate.MAX_ANGULAR_SPEED)
        for sequence, (v, w) in enumerate(((.05, .3), (.5, .6), (0., 0.), (.01, -.6)), 1):
            self.assertEqual(control.command(dict(session=token, sequence=sequence, linear=v, angular=w)), (v, w))
            self.assertEqual(control.token, token)
