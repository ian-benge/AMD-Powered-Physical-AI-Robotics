#!/usr/bin/env python3
"""Forward terminal stop to the supervisor, keeping health queries uninterrupted."""
import signal
import subprocess
import sys


def run_python(arguments):
    if not arguments:
        raise ValueError('Specify the Python supervisor and its arguments')
    child=None
    stopping=False

    def stop(*_):
        nonlocal stopping
        stopping=True
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)

    original={sig:signal.signal(sig,stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        child=subprocess.Popen([sys.executable,*arguments],start_new_session=True)
        if stopping:stop()
        return child.wait()
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGTERM)
            child.wait(timeout=45)
        for sig,handler in original.items():signal.signal(sig,handler)


if __name__=='__main__':raise SystemExit(run_python(sys.argv[1:]))
