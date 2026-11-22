import json
from pathlib import Path
import tempfile
import unittest

from mars_training_environment.common import digest
from prepare_demo_rare_scenes import counts, geometry_record


class RareSceneEvidenceTests(unittest.TestCase):
    def test_render_materials_do_not_change_the_pre_material_geometry_identity(self):
        geometry=dict(seed=1,layout_seed=1,layout_attempt=2,config_sha256='config',
                      terrain={'size_m':40},objects=[{'name':'rock','xyz':[1.,2.,0.]}],regions=[])
        scene=dict(geometry,geometry_sha256=digest(geometry))
        scene['objects']=[dict(geometry['objects'][0],material_maps={'albedo':'new.png'})]
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);(folder/'layout-attempts').mkdir()
            saved=folder/'layout-attempts/attempt-02.json'
            saved.write_text(json.dumps(geometry))
            self.assertEqual(geometry_record(scene,folder),geometry)
            altered=dict(geometry,objects=[{'name':'rock','xyz':[3.,2.,0.]}])
            saved.write_text(json.dumps(altered))
            with self.assertRaisesRegex(ValueError,'geometry identity'):geometry_record(scene,folder)
    def test_counting_uses_export_metadata_without_reinterpreting_legacy_class_ids(self):
        scene=dict(objects=[{'class_name':'rock'},{'class_name':'pale_stone'},
                            {'class_name':'sulfur_rock'},{'class_name':'cheyava_rock'},
                            {'class_name':'veined_lookalike'}],
                   regions=[{'class_name':'silica_soil'},{'class_name':'pale_lookalike'}])
        self.assertEqual(counts(scene),[2,2,1,2])


if __name__=='__main__':
    unittest.main()
