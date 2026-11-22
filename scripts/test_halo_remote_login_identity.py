import unittest
from halo_remote_login_identity import validate


class RemoteLoginIdentityTests(unittest.TestCase):
    def test_observed_system_login_identity(self):
        validate('LISTEN *:3389', dict(ActiveState='active', User='gnome-remote-desktop', MainPID='336889'),
                 988, 988, [b'/usr/libexec/gnome-remote-desktop-daemon', b'--system'])

    def test_missing_listener_wrong_unit_owner_or_process_refused(self):
        unit = dict(ActiveState='active', User='gnome-remote-desktop', MainPID='336889')
        args = [b'/usr/libexec/gnome-remote-desktop-daemon', b'--system']
        for listener, service, uid, argv in [('', unit, 988, args), ('LISTEN', dict(unit, ActiveState='inactive'), 988, args),
             ('LISTEN', dict(unit, User='ian'), 988, args), ('LISTEN', dict(unit, MainPID='0'), 988, args),
             ('LISTEN', unit, 1001, args), ('LISTEN', unit, 988, [b'/usr/libexec/gnome-remote-desktop-daemon'])]:
            with self.subTest(service=service, uid=uid), self.assertRaises(RuntimeError):
                validate(listener, service, uid, 988, argv)


if __name__ == '__main__':
    unittest.main()
