#!/usr/bin/env python3
"""Install a new checked demo wrapper after a candidate build; preserve rollback."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('application',type=Path)
    p.add_argument('runtime',type=Path);a=p.parse_args();app=a.application.resolve();runtime=a.runtime.resolve()
    if runtime.parent!=app.parent or runtime.exists():raise ValueError('Use a new sibling runtime directory')
    receipt=json.loads((app/'BUILD_RECEIPT.json').read_text())
    if receipt.get('installed_interface') is not True:raise ValueError('Candidate has not passed build/import checks')
    release=json.loads((app/'ASSESSMENT_RELEASE.json').read_text());config=json.loads(Path(release['configuration']).read_text())
    for name,expected in config['files'].items():
        if (app/name).is_symlink() or sha(app/name)!=expected:raise ValueError('Candidate changed: '+name)
    runtime.mkdir()
    for name in ('autonomous_demo_assess.py','start_autonomous_demo.sh',
                 'demo_exterior_view.py','view_autonomous_demo.sh'):
        source=app/'source/scripts'/name;body=source.read_bytes().replace(b'\r\n',b'\n')
        target=runtime/name;target.write_bytes(body)
        release['files'][str(target)]=sha(target)
    with (runtime/'ASSESSMENT_RELEASE.json').open('x') as f:json.dump(release,f,indent=2);f.write('\n')
    subprocess.run(['python3','-B',str(runtime/'autonomous_demo_assess.py'),'--check'],check=True)
    print('DEMO_RUNTIME_STATIC_CHECK_PASS: '+str(runtime/'start_autonomous_demo.sh'))


if __name__=='__main__':main()
