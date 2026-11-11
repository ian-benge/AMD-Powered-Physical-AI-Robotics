"""Install training-only dependencies beside the unchanged qualified ROCm stack."""
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import subprocess
import sys

from capture_training_dataset import sha, save

BASE = Path('/home/ian/capstone-training/venvs/terrain-rocm7141')
SITE = BASE/'lib/python3.12/site-packages'
HERE = Path(__file__).resolve().parent


def main():
    env = HERE/'training-venv'
    if env.exists(): raise RuntimeError('Training environment already exists; verify it instead of modifying it')
    before = {d.metadata['Name']:d.version for d in metadata.distributions(path=[str(SITE)])}
    if before.get('torch')!='2.12.0+rocm7.14.1' or before.get('rocm')!='7.14.1':
        raise RuntimeError('Qualified parent environment changed')
    save(HERE/'QUALIFIED_PACKAGES_BEFORE.json',before)
    subprocess.run([str(BASE/'bin/python'),'-m','venv',str(env)],check=True)
    child_site=env/'lib/python3.12/site-packages'
    (child_site/'qualified-rocm.pth').write_text('import site; site.addsitedir('+repr(str(SITE))+')\n')
    constraints=HERE/'training-constraints.txt'
    constraints.write_text(''.join(f'{name}=={version}\n' for name,version in sorted(before.items()))+
        'torchvision==0.27.0+rocm7.14.1\nultralytics==8.4.175\n')
    python=env/'bin/python'
    environment=dict(os.environ,PIP_CONFIG_FILE='/dev/null',PIP_CONSTRAINT=str(constraints),
        PIP_DISABLE_PIP_VERSION_CHECK='1',PIP_NO_INPUT='1')
    for key in ['PIP_EXTRA_INDEX_URL','PIP_INDEX_URL','PYTHONPATH']:environment.pop(key,None)
    commands=[
        [str(python),'-m','pip','install','--index-url','https://repo.amd.com/rocm/whl-multi-arch/',
         'torchvision[device-gfx1151]==0.27.0+rocm7.14.1'],
        [str(python),'-m','pip','install','--index-url','https://pypi.org/simple','ultralytics==8.4.175'],
        [str(python),'-m','pip','check']]
    with (HERE/'ENVIRONMENT_SETUP.log').open('xb') as log:
        for command in commands:
            subprocess.run(command,env=environment,stdout=log,stderr=subprocess.STDOUT,check=True)
    after={d.metadata['Name']:d.version for d in metadata.distributions(path=[str(SITE)])}
    if after!=before:raise RuntimeError('Qualified parent environment was modified')
    result=subprocess.check_output([str(python),'-m','pip','freeze','--all'],env=environment,text=True)
    (HERE/'TRAINING_ENVIRONMENT_FREEZE.txt').write_text(result)
    save(HERE/'TRAINING_ENVIRONMENT.json',dict(python=str(python),qualified_parent=str(BASE),
        shared_qualified_packages_unchanged=True,torch='2.12.0+rocm7.14.1',
        torchvision='0.27.0+rocm7.14.1',ultralytics='8.4.175',
        freeze_sha256=sha(HERE/'TRAINING_ENVIRONMENT_FREEZE.txt'),
        gpu_probe_passed=False,system_drivers_changed=False))
    print('TRAINING_ENVIRONMENT_PREPARED_NO_GPU_WORK',flush=True)


if __name__=='__main__':main()
