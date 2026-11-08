"""Bound Linux capture cleanup to a launch's unique Gazebo partition.

Gazebo children may detach from ROS launch's process group. Ownership therefore
uses the inherited launch partition, UID and process start time, with pidfds for
signalling. A previous scene can never satisfy current-scene renderer proof.
"""
import os
import hashlib
import json
from pathlib import Path
import shlex
import signal
import time


def process_identity(pid, proc=Path('/proc')):
    directory = proc/str(pid)
    fields = (directory/'stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=int(pid), start_ticks=int(fields[19]), state=fields[0],
                uid=directory.stat().st_uid)


def partition_for(pid, proc=Path('/proc')):
    values = (proc/str(pid)/'environ').read_bytes().split(b'\0')
    return next((v.split(b'=', 1)[1].decode() for v in values if v.startswith(b'GZ_PARTITION=')), None)


class CaptureProcessScope:
    def __init__(self, process):
        self.leader = process_identity(process.pid)
        # /proc/environ can be transiently empty while the launch shim execs.
        # Wait only for ownership evidence; never infer it from a process name.
        self.partition = None
        for _ in range(20):
            self.partition = partition_for(process.pid)
            if self.partition or process.poll() is not None:
                break
            time.sleep(.05)
        if not self.partition or not self.partition.startswith('mars_capture_'):
            raise RuntimeError('Capture launch has no unique owned partition')
        current = process_identity(process.pid)
        if (current['uid'], current['start_ticks']) != (self.leader['uid'], self.leader['start_ticks']):
            raise RuntimeError('Capture launch identity changed during startup')
        if self.leader['uid'] != os.getuid():
            raise RuntimeError('Capture launch UID differs from its supervisor')

    def processes(self):
        owned = []
        for directory in Path('/proc').glob('[0-9]*'):
            try:
                identity = process_identity(int(directory.name))
                if identity['uid'] != self.leader['uid'] or identity['state'] == 'Z':
                    continue
                if partition_for(identity['pid']) == self.partition:
                    owned.append(identity)
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue
        return owned

    def signal(self, identity, sig):
        """The pidfd stays bound to this process even if its PID is later reused."""
        try:
            fd = os.pidfd_open(identity['pid'])
        except ProcessLookupError:
            return
        try:
            current = process_identity(identity['pid'])
            if (current['start_ticks'], current['uid']) != (identity['start_ticks'], identity['uid']):
                raise RuntimeError('Capture PID identity changed before cleanup')
            if partition_for(identity['pid']) != self.partition:
                raise RuntimeError('Refusing cleanup outside the owned capture partition')
            signal.pidfd_send_signal(fd, sig)
        except (FileNotFoundError, ProcessLookupError):
            pass
        finally:
            os.close(fd)

    def cleanup(self, stage_seconds=3.):
        actions = []
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            deadline = time.monotonic()+stage_seconds
            sent = set()
            while True:
                owned = self.processes()
                if not owned:
                    return dict(version=2, partition=self.partition, leader=self.leader,
                                actions=actions, remaining=[])
                for identity in owned:
                    key = (identity['pid'], identity['start_ticks'])
                    if key not in sent:
                        self.signal(identity, sig); sent.add(key)
                        actions.append(dict(identity=identity, signal=int(sig)))
                if time.monotonic() >= deadline:
                    break
                time.sleep(.05)
        remaining = self.processes()
        if remaining:
            raise RuntimeError('Owned Gazebo/ROS processes survived bounded cleanup: '+str(remaining))
        return dict(version=2, partition=self.partition, leader=self.leader,
                    actions=actions, remaining=[])


def verify_scene_renderer(value, scope, world, launch_wall_time, renderer_log=Path('/tmp/.gz/rendering/ogre.log')):
    processes = value.get('processes', [])
    if len(processes) != 1:
        raise RuntimeError('Expected exactly one mapped renderer for the current scene')
    server = processes[0]
    if str(world) not in shlex.split(server['command']):
        raise RuntimeError('Mapped renderer belongs to a different scene')
    identity = next((p for p in scope.processes() if p['pid'] == server['pid']), None)
    if identity is None:
        raise RuntimeError('Mapped renderer is outside the current launch partition')
    if renderer_log.stat().st_mtime < launch_wall_time:
        raise RuntimeError('Renderer identity log predates the current launch')
    identities = value.get('renderer_identities', [])
    if not identities or not all('Radeon 8060S' in line and 'radeonsi' in line for line in identities):
        raise RuntimeError('Current renderer hardware identity is missing')
    libraries = server.get('libraries', [])
    if not any('libgz-rendering8-ogre.so' in lib['path'] and lib['sha256'] ==
               'f6e8842247198a2b34b25cfdb40a8f4bc4f156c2e90a54135a68698dc239a3e6' for lib in libraries):
        raise RuntimeError('Qualified Ogre mapping is missing')
    if not any('libgallium' in lib['path'] or 'radeonsi_dri.so' in lib['path'] for lib in libraries):
        raise RuntimeError('Executable Mesa mapping is missing')
    return dict(version=2, world=str(world), renderer=identity, partition=scope.partition)


def prepare_dataset_output(root):
    """Retain legacy evidence and keep weaker proofs out of a resumed dataset."""
    root = Path(root).resolve(); output = root/'dataset'
    contract = output/'RUNTIME_CONTRACT.json'
    if output.is_symlink():
        raise ValueError('Capture dataset must not be a symlink')
    if output.exists() and not contract.exists():
        preserved = root/'dataset-before-process-cleanup-fix'
        if preserved.exists():
            raise ValueError('Legacy preservation directory already exists; refusing to replace it')
        frames = len(list((output/'metadata').glob('*/*.json')))
        output.rename(preserved)
        migration = dict(version=2, preserved=str(preserved), preserved_metadata_frames=frames,
            reason='Previous renderer checks were not scene-bound; originals retained and recaptured',
            generated_bundle_unchanged=True)
        with (root/'PROCESS_FIX_MIGRATION.json').open('x') as stream:
            json.dump(migration, stream, indent=2)
    output.mkdir(exist_ok=True)
    if not contract.exists():
        with contract.open('x') as stream:
            json.dump(dict(version=2, owned_partition_cleanup=True, scene_bound_renderer_proof=True), stream)
    if json.loads(contract.read_text()) != dict(version=2, owned_partition_cleanup=True, scene_bound_renderer_proof=True):
        raise ValueError('Capture runtime contract differs from the required version')
    return output


def verify_runtime_qualification(root, output, report):
    def inside(base, relative):
        path = (base/relative).resolve()
        if not path.is_relative_to(base.resolve()):
            raise ValueError('Runtime evidence path escapes its root')
        return path

    def sha(path):
        with path.open('rb') as stream:
            return hashlib.file_digest(stream, 'sha256').hexdigest()

    root, output = Path(root), Path(output)
    value = json.loads((inside(output, report['attempt'])/'RUNTIME_QUALIFICATION.json').read_text())
    expected_world = str(root/'bundle/scenes'/report['scene_id']/'world.sdf')
    if value.get('version') != 2 or value.get('world') != expected_world or value.get('scene_sha256') != report['scene_sha256']:
        raise ValueError('Runtime qualification belongs to a different scene')
    proof = inside(root, value['proof'])
    for file, field in [('PASS.json', 'proof_sha256'), ('CLEANUP.json', 'cleanup_sha256')]:
        if sha(proof/file) != value[field]:
            raise ValueError('Saved runtime qualification changed')
    passed = json.loads((proof/'PASS.json').read_text()); cleanup = json.loads((proof/'CLEANUP.json').read_text())
    if passed.get('version') != 2 or passed.get('world') != expected_world or cleanup.get('remaining') != [] or cleanup.get('partition') != passed.get('partition'):
        raise ValueError('Scoped renderer proof or clean teardown is missing')
    if sha(proof/f'attempt-{passed["attempt"]:03d}.json') != passed['observation_sha256']:
        raise ValueError('Renderer mapping observation changed')
    return value
