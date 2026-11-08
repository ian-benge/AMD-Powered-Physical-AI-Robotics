"""The host supervisor must reject changed scene contracts before GPU startup."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts'))
from phase2_supervise_live import validate_scene, inspect_container, belongs_to_candidate
from mars_training_environment.common import digest
from mars_training_environment.serialization import digest as portable_digest


class Phase2SupervisorTests(unittest.TestCase):
    def test_inspection_does_not_resolve_the_retained_image_as_a_container(self):
        with patch('phase2_supervise_live.subprocess.run',return_value=SimpleNamespace(returncode=1)) as run:
            self.assertIsNone(inspect_container('rover-navigation'))
            self.assertEqual(run.call_args.args[0],['docker','container','inspect','rover-navigation'])
        with patch('phase2_supervise_live.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout='[{"Id":"image-only"}]')):
            with self.assertRaises(RuntimeError):inspect_container('rover-navigation')

    def test_container_identity_requires_both_image_and_checkout(self):
        root=Path.cwd().resolve();config=dict(image='sha256:candidate')
        good=dict(Image=config['image'],Mounts=[dict(Destination='/work',Source=str(root))])
        self.assertTrue(belongs_to_candidate(good,config,root))
        self.assertFalse(belongs_to_candidate(dict(good,Image='baseline'),config,root))
        self.assertFalse(belongs_to_candidate(dict(good,Mounts=[]),config,root))

    def test_portable_manifest_hash_preserves_cross_platform_number_policy(self):
        value=dict(x=-0.00000000001,y=[1.2345678901234,2.],label='terrain')
        self.assertEqual(digest(value),portable_digest(value))

    def test_prelaunch_rejects_missing_truth_isolation_and_modified_scene(self):
        scene=dict(terrain=dict(profile='mixed_reachable',profile_version=1,shared_surface=True),
                   config=dict(capture=dict(views_per_scene=0)),
                   reachability=dict(evaluator_only=True,truth_to_navigation=False,
                        routes=[dict(category=c) for c in ('ordinary_rock','pale_soil','crystal_rock')]))
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);target=root/'scenes/scene_000001/scene.json';target.parent.mkdir(parents=True)
            def write(value):
                value=deepcopy(value);value['content_sha256']=digest(value)
                target.write_text(json.dumps(value))
            write(scene)
            validate_scene({},root,'scene_000001')
            for section,changes in [('terrain',dict(shared_surface=False)),
                                    ('reachability',dict(truth_to_navigation=True)),
                                    ('reachability',dict(evaluator_only=False))]:
                bad=deepcopy(scene);bad[section].update(changes);write(bad)
                with self.assertRaises(ValueError):validate_scene({},root,'scene_000001')
            write(scene)
            value=json.loads(target.read_text());value['config']['capture']['views_per_scene']=1
            target.write_text(json.dumps(value))
            with self.assertRaises(ValueError):validate_scene({},root,'scene_000001')


if __name__=='__main__':unittest.main()
