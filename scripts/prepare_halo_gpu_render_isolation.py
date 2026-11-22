#!/usr/bin/env python3
"""Create a bounded ten-Hz rendering-only fixture; never launch it here."""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from prepare_halo_gpu_candidate import once

BASE = Path('/home/ian/capstone-training/rover-navigation/navigation_sessions')


def main():
    parent = BASE / 'rendering-phase1-resume-20261104T202728Z/qualification-v10'
    target = BASE / 'autonomous-demo-20261109/gpu-repair-20261117/render-only-v1'
    target.mkdir()  # Existing attempts are never overwritten.
    for tree in ('source', 'fixture20'):
        shutil.copytree(parent / tree, target / tree)
    fixture = target / 'fixture20'
    world = fixture / 'world.sdf'
    world.write_text(once(world.read_text(), '<update_rate>30</update_rate>', '<update_rate>10</update_rate>'))
    metadata = json.loads((fixture / 'fixture.json').read_text())
    assert metadata['camera']['rate_hz'] == 30
    metadata['camera']['rate_hz'] = 10
    (fixture / 'fixture.json').write_text(json.dumps(metadata, indent=2) + '\n')
    assert float(ET.parse(world).findtext('.//sensor/update_rate')) == 10
    source = target / 'source/scripts'
    runner = source / 'phase1_run_fixture.py'
    runner.write_text(once(runner.read_text(), '    if not -20 <= tctl < 90:', '    if not -20 <= tctl < 87:').replace(
        'Tctl reached the 90 C experiment threshold', 'Tctl reached the 87 C qualification threshold'))
    container = source / 'phase1_fixture_container.sh'
    container.write_text(once(container.read_text(), 'docker run --rm --init --name',
        'docker run --cpuset-cpus=0,1,2,3,4,5,6,7 -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=2 --rm --init --name'))
    starter = target / 'run_render_only.sh'
    starter.write_text('#!/usr/bin/env bash\nset -euo pipefail\n'
        'ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)\n'
        'python3 -B "$ROOT/check_inputs.py"\n'
        'eval "$(python3 -I /home/ian/capstone-training/rover-sim/manage.py context)"\n'
        'CASE=$(cat "/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103/capture-$SIM_LABEL.path")\n'
        'exec python3 -B "$ROOT/source/scripts/phase1_run_fixture.py" "$ROOT" render-only-300s '
        '--image sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa '
        '--fixture fixture20 --kit "$SIM_KIT" --case "$CASE" --mode stationary --seconds 300 --attended\n')
    checker = target / 'check_inputs.py'
    checker.write_text("import hashlib,json\nfrom pathlib import Path\nr=Path(__file__).resolve().parent\n"
        "v=json.loads((r/'INPUTS.json').read_text())\n"
        "for n,h in v['files'].items():\n"
        " if hashlib.sha256((r/n).read_bytes()).hexdigest()!=h:raise RuntimeError('Rendering input changed: '+n)\n"
        "print('RENDER_ONLY_INPUT_PINS_PASS')\n")
    for path in (runner, checker):
        ast.parse(path.read_text())
    for path in (container, starter):
        subprocess.run(['bash', '-n', str(path)], check=True)
    files = {str(p.relative_to(target)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(target.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
    with (target / 'INPUTS.json').open('x') as stream:
        json.dump(dict(parent=str(parent), files=files, camera_rate_hz=10, qualification_stop_c=87,
                       fixture_only=True, navigation_motion=False, rocm_mapping=False,
                       note='Camera fixture differs from rare rover scene; comparison cannot isolate scene effects'), stream, indent=2)
    subprocess.run(['python3', '-B', str(checker)], check=True)
    print('RENDER_ONLY_CANDIDATE_READY: ' + str(starter))


if __name__ == '__main__':
    main()
