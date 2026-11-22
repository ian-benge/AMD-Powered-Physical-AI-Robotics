#!/usr/bin/env python3
"""Archive closed GPU repair evidence with a hash for every regular member."""
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import zipfile

BASE = Path('/home/ian/capstone-training/rover-navigation/navigation_sessions')
DEMO = BASE / 'autonomous-demo-20261109'


def main():
    folder = DEMO / 'gpu-repair-20261117'
    selection = json.loads(Path('/home/ian/capstone-training/rover-sim/CURRENT.json').read_text())
    pointer = Path('/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103') / ('capture-' + selection['label'] + '.path')
    case = Path(pointer.read_text().strip())
    state = json.loads((case / 'LATEST_CAPTURE_STATE.json').read_text())
    if state['state'] != 'stopped' or not (case / 'POST_TEARDOWN.json').is_file():
        raise RuntimeError('Keep monitoring through protected finish, then stop it before final packaging')
    for port in (8775, 8782):
        with socket.socket() as probe:
            probe.settimeout(1.)
            if probe.connect_ex(('127.0.0.1', port)) == 0:
                raise RuntimeError('Stop the NPU and terrain workloads before final packaging')
    containers = subprocess.run(['docker', 'ps', '--format', '{{.Names}}'],
                                capture_output=True, text=True, check=True)
    if containers.stdout.strip():
        raise RuntimeError('Containers remain active; review workload teardown before final packaging')
    paths = {}
    excluded = []
    def add_tree(root, prefix):
        for path in sorted(root.rglob('*')):
            if path.is_file():
                relative = path.relative_to(root)
                if path.is_symlink() or '__pycache__' in path.parts or path.suffix == '.pyc' or path.name == 'rtabmap.db':
                    excluded.append(dict(path=str(path), reason='cache, symlink or retained large localization database'))
                else:
                    paths[prefix + '/' + str(relative).replace(os.sep, '/')] = path
    add_tree(folder, 'gpu-repair')
    add_tree(case, 'native-current')
    kit = Path(selection['kit'])
    add_tree(kit, 'native-kit')
    for old in sorted(BASE.glob('oem617-7141-rocm-manual-20261117t19*')):
        if old != case:
            add_tree(old, 'native-predecessors/' + old.name)
    versions = sorted(int(path.name.removeprefix('application-v'))
                      for path in DEMO.glob('application-v*')
                      if path.name.removeprefix('application-v').isdigit()
                      and int(path.name.removeprefix('application-v')) >= 23)
    for version in versions:
        label = 'application-v' + str(version)
        app = DEMO / label
        for name in ('DEMO_CONFIGURATION.json', 'ASSESSMENT_RELEASE.json', 'BUILD_RECEIPT.json'):
            paths[label + '/' + name] = app / name
        for name in ('checks', 'navigation_sessions', 'supervisor-sessions'):
            if (app / name).is_dir():
                add_tree(app / name, label + '/' + name)
        add_tree(DEMO / ('runtime-v' + str(version)), 'runtime-v' + str(version))
    stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = DEMO / ('HALO_GPU_REPAIR_' + stamp + '.zip')
    manifest = {}
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name, path in sorted(paths.items()):
            before = path.stat()
            payload = path.read_bytes()
            after = path.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise RuntimeError('Evidence changed while archiving: ' + str(path))
            manifest[name] = hashlib.sha256(payload).hexdigest()
            archive.writestr(name, payload)
        index = dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(), selection=selection,
                     current_case=str(case), application_versions=versions, excluded=excluded,
                     original_v22_archive_sha256='c75580af574e44fb786649c809e5430422cba760bbd4cf6b7705a4309468c65a',
                     original_incident_flags_preserved=True, full_demo_acceptance=False)
        payload = (json.dumps(index, indent=2) + '\n').encode()
        archive.writestr('INDEX.json', payload)
        manifest['INDEX.json'] = hashlib.sha256(payload).hexdigest()
        archive.writestr('SHA256.json', json.dumps(manifest, indent=2) + '\n')
    with output.open('rb') as stream:
        checksum = hashlib.file_digest(stream, 'sha256').hexdigest()
    identity = dict(archive=str(output), sha256=checksum, verified_files=len(manifest), bytes=output.stat().st_size)
    with output.with_suffix('.identity.json').open('x') as stream:
        json.dump(identity, stream, indent=2)
    print(json.dumps(identity), flush=True)


if __name__ == '__main__':
    main()
