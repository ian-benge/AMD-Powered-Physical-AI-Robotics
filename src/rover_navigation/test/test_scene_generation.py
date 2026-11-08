import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

from rover_navigation.scene_generation import generate


class GenerationTests(unittest.TestCase):
    def test_v2_generation_retains_density_camera_and_profile(self):
        from mars_training_environment.common import read_config
        from mars_training_environment.environment_v2 import configuration
        with TemporaryDirectory() as directory:
            root=Path(directory);(root/'assets').mkdir();(root/'assets/catalog.json').write_text('[]')
            selected=configuration(read_config(),'cluttered','high')
            template=root/'template.json';template.write_text(json.dumps(dict(id='v2',config=selected)))
            make_scene=Mock(return_value=dict(content_sha256='generated',geometry_sha256='geometry'))
            with patch.dict(sys.modules,{'mars_training_environment.generate':SimpleNamespace(make_scene=make_scene)}):
                result=generate(root,template,4311017)
            actual=make_scene.call_args.args[1]
            self.assertEqual(actual['terrain'],selected['terrain'])
            self.assertEqual(actual['rocks'],selected['rocks'])
            self.assertEqual(actual['camera'],selected['camera'])
            self.assertEqual(result['terrain_profile_version'],2)
            self.assertFalse(result['capture_requested'])

    def test_navigation_generation_preserves_terrain_constraints_and_template(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'assets').mkdir()
            (root/'assets/catalog.json').write_text('[]')
            from mars_training_environment.common import read_config
            config=read_config()
            config['capture'].update(views_per_scene=40,target_class_views={'rock':40})
            template = root/'template.json'
            source = dict(id='scene_050000', config=config)
            template.write_text(json.dumps(source))
            make_scene = Mock(return_value=dict(content_sha256='generated-hash'))
            with patch.dict(sys.modules, {'mars_training_environment.generate': SimpleNamespace(make_scene=make_scene)}):
                result = generate(root, template, 1000017)
            selected = make_scene.call_args.args[1]
            self.assertEqual(selected['terrain']['profile'],'mixed_reachable')
            self.assertEqual(selected['terrain']['profile_version'],1)
            self.assertEqual(selected['terrain']['spawn_clearance_m'],1.3)
            self.assertEqual(selected['terrain']['grid_size'],selected['terrain']['visual_grid_size'])
            self.assertEqual(selected['rocks'], config['rocks'])
            self.assertEqual(selected['capture']['views_per_scene'], 0)
            self.assertNotIn('target_class_views', selected['capture'])
            self.assertEqual(json.loads(template.read_text()), source)
            self.assertFalse(result['capture_requested'])
            self.assertEqual(result['source_scene'], source['id'])
            self.assertEqual(result['terrain_profile'],'mixed_reachable')
            (root/'scenes/scene_1000017').mkdir(parents=True)
            with self.assertRaisesRegex(ValueError, 'already exists'):
                generate(root, template, 1000017)

    def test_invalid_seed_is_rejected_before_any_files_are_read(self):
        for seed in (True, -1, 1_000_000_000, '1000'):
            with self.subTest(seed=seed), self.assertRaises(ValueError):
                generate(Path('absent'), Path('absent'), seed)

    def test_generation_clearance_contract_contains_the_actual_rover_turn(self):
        from rover_navigation.core import FOOTPRINT,turn_radius,DRIVE_CENTER_X
        from mars_training_environment.reachability import FOOTPRINT as policy,TURN_RADIUS,CORRIDOR_RADIUS,START
        for actual,expected in zip(FOOTPRINT,policy):self.assertAlmostEqual(actual,expected)
        self.assertAlmostEqual(turn_radius(),TURN_RADIUS)
        self.assertGreaterEqual(CORRIDOR_RADIUS,turn_radius())
        self.assertEqual(START,(DRIVE_CENTER_X,0))
