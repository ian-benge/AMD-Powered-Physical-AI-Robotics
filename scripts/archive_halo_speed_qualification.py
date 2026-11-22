#!/usr/bin/env python3
"""Package a closed November 17 speed test; acceptance is reviewed separately."""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import zipfile

BASE = Path('/home/ian/capstone-training/rover-navigation/navigation_sessions')
DEMO = BASE / 'autonomous-demo-20261109'
ROOT = DEMO / 'qualification-010-20261117'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, default=ROOT)
    args = parser.parse_args()
    allowed = (ROOT, DEMO / 'qualification-010-resume-20261117')
    if args.folder.is_symlink() or args.folder.resolve() not in allowed:
        raise RuntimeError('Use an explicitly named speed qualification evidence folder')
    folder = args.folder.resolve()
    selection = json.loads(Path('/home/ian/capstone-training/rover-sim/CURRENT.json').read_text())
    expected = json.loads((folder / 'FRESH_CAPTURE_DESKTOP_CHECK.json').read_text())
    if selection != expected['selection']:
        raise RuntimeError('Native selection changed; preserve and review this test separately')
    case = Path(expected['case'])
    if case.parent != BASE or case.is_symlink():
        raise RuntimeError('Unexpected native case')
    state = json.loads((case / 'LATEST_CAPTURE_STATE.json').read_text())
    if state['state'] != 'stopped' or not (case / 'POST_TEARDOWN.json').is_file():
        raise RuntimeError('Complete protected finish and stop monitoring before packaging')
    for name in ('PROTECTED_TEARDOWN_SCOPED_REVIEW_FINAL.json', 'OBSERVER_FINAL_STOP.json'):
        if not (folder / name).is_file():
            raise RuntimeError('Missing final evidence review: ' + name)
    for port in (8774, 8775, 8777, 8778, 8782):
        with socket.socket() as probe:
            probe.settimeout(1.)
            if probe.connect_ex(('127.0.0.1', port)) == 0:
                raise RuntimeError('A workload listener remains active')
    if subprocess.check_output(['docker', 'ps', '-q'], text=True).strip():
        raise RuntimeError('Containers remain active')
    files, excluded = {}, []

    def add_tree(root, prefix):
        for path in sorted(root.rglob('*')):
            if path.is_file():
                if path.is_symlink() or '__pycache__' in path.parts or path.suffix == '.pyc' or path.name == 'rtabmap.db':
                    excluded.append(dict(path=str(path), reason='symlink, cache or retained localization database'))
                else:
                    files[prefix + '/' + path.relative_to(root).as_posix()] = path

    add_tree(folder, 'qualification')
    add_tree(case, 'native-current')
    add_tree(Path(selection['kit']), 'native-kit')
    add_tree(DEMO / 'application-v26', 'application-v26')
    add_tree(DEMO / 'runtime-v26', 'runtime-v26')
    add_tree(DEMO / 'gpu-repair-20261117/terrain-sdma-off-v2', 'terrain-sdma-off-v2')
    stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    prefix = 'HALO_SPEED_010_' if folder == ROOT else 'HALO_SPEED_010_RESUMED_'
    output = DEMO / (prefix + stamp + '.zip')
    manifest = {}
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name, path in sorted(files.items()):
            before = path.stat()
            payload = path.read_bytes()
            after = path.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise RuntimeError('Evidence changed during packaging: ' + str(path))
            manifest[name] = hashlib.sha256(payload).hexdigest()
            archive.writestr(name, payload)
        index = dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(), selection=selection, evidence_folder=str(folder),
                     case=str(case), excluded=excluded, acceptance_claimed_by_packaging=False,
                     predecessor_archive_sha256='9d8a162cf8cb9e45b24a8858be52f62b03fbde5536885486274e16386cd96120')
        payload = (json.dumps(index, indent=2) + '\n').encode()
        archive.writestr('INDEX.json', payload)
        manifest['INDEX.json'] = hashlib.sha256(payload).hexdigest()
        archive.writestr('SHA256.json', json.dumps(manifest, indent=2) + '\n')
    with output.open('rb') as stream:
        checksum = hashlib.file_digest(stream, 'sha256').hexdigest()
    identity = dict(archive=str(output), sha256=checksum, verified_files=len(manifest), bytes=output.stat().st_size)
    with output.with_suffix('.identity.json').open('x') as stream:
        json.dump(identity, stream, indent=2)
        stream.write('\n')
    print(json.dumps(identity), flush=True)


if __name__ == '__main__':
    main()
