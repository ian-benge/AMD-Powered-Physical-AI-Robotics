"""Require a live desktop output without creating GL contexts or changing displays."""
from contextlib import contextmanager
import ctypes
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading


class DesktopUnavailable(RuntimeError):
    pass


def query_active_output():
    """Run only in the bounded probe child: Xlib calls can block on a dead server."""
    display = os.environ.get('DISPLAY')
    if not display:
        raise DesktopUnavailable('DISPLAY is absent')
    x11 = ctypes.CDLL('libX11.so.6')
    randr = ctypes.CDLL('libXrandr.so.2')
    pointer, integer, window = ctypes.c_void_p, ctypes.c_int, ctypes.c_ulong
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = pointer
    x11.XDefaultRootWindow.argtypes = [pointer]
    x11.XDefaultRootWindow.restype = window
    x11.XCloseDisplay.argtypes = [pointer]
    x11.XCloseDisplay.restype = integer
    randr.XRRQueryVersion.argtypes = [pointer, ctypes.POINTER(integer), ctypes.POINTER(integer)]
    randr.XRRQueryVersion.restype = integer
    randr.XRRGetMonitors.argtypes = [pointer, window, integer, ctypes.POINTER(integer)]
    randr.XRRGetMonitors.restype = pointer
    randr.XRRFreeMonitors.argtypes = [pointer]
    randr.XRRFreeMonitors.restype = None
    connection = x11.XOpenDisplay(display.encode())
    if not connection:
        raise DesktopUnavailable('X11 desktop connection failed')
    monitors = None
    try:
        major, minor, count = integer(), integer(), integer()
        if not randr.XRRQueryVersion(connection, ctypes.byref(major), ctypes.byref(minor)) or (major.value, minor.value) < (1, 5):
            raise DesktopUnavailable('RandR 1.5 active-output query is unavailable')
        monitors = randr.XRRGetMonitors(connection, x11.XDefaultRootWindow(connection), 1, ctypes.byref(count))
        if not monitors or count.value < 1:
            raise DesktopUnavailable('X11 is reachable but has no active desktop output')
        return dict(version=1, display=display, active_monitors=count.value,
                    randr=[major.value, minor.value])
    finally:
        if monitors:
            randr.XRRFreeMonitors(monitors)
        x11.XCloseDisplay(connection)


def active_output():
    try:
        result = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--probe'],
            capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise DesktopUnavailable('Desktop active-output query failed or timed out') from error
    try:
        value = json.loads(result.stdout)
    except (ValueError, TypeError) as error:
        raise DesktopUnavailable('Desktop active-output query returned no valid evidence') from error
    if not isinstance(value, dict):
        raise DesktopUnavailable('Desktop active-output evidence is not an object')
    if result.returncode:
        raise DesktopUnavailable(value.get('error', 'Desktop active-output query failed'))
    if (value.get('version') != 1 or
            value.get('display') != os.environ.get('DISPLAY') or
            type(value.get('active_monitors')) is not int or value['active_monitors'] < 1 or
            not isinstance(value.get('randr'), list) or len(value['randr']) != 2 or
            any(type(part) is not int for part in value['randr']) or tuple(value['randr']) < (1, 5)):
        raise DesktopUnavailable('Desktop active-output evidence is incomplete or mismatched')
    return value


def stop_path(root, invocation):
    ident = hashlib.sha256(Path(invocation).read_bytes()).hexdigest()
    return Path(root)/'desktop-state'/ident/'STOP.json'


def require_desktop(stop=None):
    try:
        return active_output()
    except Exception as error:
        reason = 'DESKTOP_RECONNECT_REQUIRED: '+str(error)+'; reconnect the Halo graphical desktop, then resume. Outputs retained.'
        if stop is not None:
            stop = Path(stop)
            stop.parent.mkdir(parents=True, exist_ok=True)
            # One immutable stop per invocation; a later attempt cannot erase it.
            try:
                with stop.open('x') as stream:
                    json.dump(dict(stage='DESKTOP_RECONNECT_REQUIRED', reason=reason,
                        utc=dt.datetime.now(dt.timezone.utc).isoformat()), stream, indent=2)
                    stream.write('\n')
            except FileExistsError:
                pass
        raise DesktopUnavailable(reason) from error


@contextmanager
def watch_desktop(process, stop, interval=1):
    """Stop only the held launch process; its existing context owns child cleanup."""
    finished = threading.Event()
    failures = []

    def observe():
        while not finished.wait(interval) and process.poll() is None:
            try:
                require_desktop(stop)
            except Exception as error:
                failures.append(error)
                try:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
                return

    thread = threading.Thread(target=observe, name='desktop-output-guard', daemon=True)
    thread.start()
    try:
        yield
    finally:
        finished.set()
        thread.join(timeout=7)
        if thread.is_alive():
            raise DesktopUnavailable('Desktop output monitor did not finish; capture refused')
        if failures:
            raise DesktopUnavailable(str(failures[0])) from failures[0]


if __name__ == '__main__':
    if sys.argv[1:] != ['--probe']:
        raise SystemExit('Use --probe for the isolated active-output query')
    try:
        print(json.dumps(query_active_output()))
    except Exception as error:
        print(json.dumps(dict(error=str(error))))
        raise SystemExit(1)
