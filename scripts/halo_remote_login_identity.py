"""Read-only verification of GNOME's system Remote Login listener.

Used only by a separately pinned native monitoring kit. Existing graphical
session selection and per-case session identity checks remain in remote_session.
"""
from pathlib import Path


def validate(listener, unit, process_uid, expected_uid, argv):
    if not listener.strip():
        raise RuntimeError('Remote Login listener is absent')
    if unit.get('ActiveState') != 'active' or unit.get('User') != 'gnome-remote-desktop':
        raise RuntimeError('Unexpected Remote Login service identity')
    if not unit.get('MainPID', '').isdecimal() or int(unit['MainPID']) <= 1:
        raise RuntimeError('Remote Login MainPID is absent')
    if process_uid != expected_uid or argv != [b'/usr/libexec/gnome-remote-desktop-daemon', b'--system']:
        raise RuntimeError('Remote Login process identity mismatch')


def verify_listener(listener, command):
    import pwd

    raw = command(['systemctl', 'show', 'gnome-remote-desktop.service',
                   '-p', 'ActiveState', '-p', 'MainPID', '-p', 'User'])
    unit = dict(row.split('=', 1) for row in raw.splitlines() if '=' in row)
    pid = unit.get('MainPID', '')
    if not pid.isdecimal() or int(pid) <= 1:
        raise RuntimeError('Remote Login MainPID is absent')
    process = Path('/proc') / pid
    before = (process / 'stat').read_text().rsplit(')', 1)[1].split()[19]
    argv = [arg for arg in (process / 'cmdline').read_bytes().split(b'\0') if arg]
    validate(listener, unit, process.stat().st_uid, pwd.getpwnam('gnome-remote-desktop').pw_uid, argv)
    after = (process / 'stat').read_text().rsplit(')', 1)[1].split()[19]
    if before != after:
        raise RuntimeError('Remote Login PID changed during inspection')
    return dict(unit=unit, start_ticks=after)
