import unittest
from unittest.mock import patch
from rover_navigation.manual_input import ManualInput


class ManualInputTests(unittest.TestCase):
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

    def test_reverse_nonfinite_and_excess_speed_rejected(self):
        for v, w in ((-.01, 0.), (.051, 0.), (0., .31), (float('nan'), 0.)):
            control = ManualInput()
            token = control.arm()
            with self.assertRaises(ValueError):
                control.command(dict(session=token, sequence=1, linear=v, angular=w))
