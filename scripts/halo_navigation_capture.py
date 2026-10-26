#!/usr/bin/env python3
"""Capture only Gazebo/RViz client windows on the Halo desktop (host, not Docker)."""
import argparse
import os
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if not os.environ.get('DISPLAY'):
        desktop = subprocess.check_output(['systemctl', '--user', 'show-environment'], text=True)
        for line in desktop.splitlines():
            name, _, value = line.partition('=')
            if name in ('DISPLAY', 'XAUTHORITY'):
                os.environ[name] = value
    import gi
    gi.require_version('Gdk', '3.0')
    gi.require_version('GdkX11', '3.0')
    from gi.repository import Gdk, GdkX11
    display = Gdk.Display.open(os.environ.get('DISPLAY', ''))
    if display is None:
        raise RuntimeError('An active Ubuntu desktop is required')
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    windows = {}
    for line in tree.splitlines():
        match = re.search(r'(0x[0-9a-f]+).*\("(rviz2|gz-sim-gui)" ', line)
        if match and re.search(r'\b(?:[1-9][0-9]{2,})x(?:[1-9][0-9]{2,})', line):
            windows.setdefault(match[2], int(match[1], 16))
    if not windows:
        raise RuntimeError('No Gazebo/RViz client windows found')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    for name, identifier in windows.items():
        window = GdkX11.X11Window.foreign_new_for_display(display, identifier)
        width, height = window.get_width(), window.get_height()
        image = Gdk.pixbuf_get_from_window(window, 0, 0, width, height)
        if image is None:
            raise RuntimeError('Could not capture '+name)
        target = args.output_dir/(name+'.png')
        image.savev(str(target), 'png', [], [])
        print(target)


if __name__ == '__main__':
    main()
