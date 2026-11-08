"""Physical surface, full-corridor connectivity and deterministic rejection."""
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

from mars_training_environment.common import read_config, digest
from mars_training_environment.generate import make_scene
from mars_training_environment.meshes import build_library, read_collada
from mars_training_environment.reachability import LayoutError, mixed_reachable_config, validate_layout
from mars_training_environment.reachability_audit import audit_layout
from mars_training_environment.terrain import Terrain, generate_terrain


def targets():
    objects=[dict(name='rock',xyz=[4,0,0],radius_m=.15),
             dict(name='crystal',class_name='sulfur_rock',xyz=[0,4,0],radius_m=.1)]
    regions=[dict(name='soil',class_name='silica_soil',xyz=[-4,0,0],radius_m=.3,
                  polygon_xy=[[-4.3,-.3],[-3.7,-.3],[-3.7,.3],[-4.3,.3]])]
    return objects,regions


class ReachableTerrainTests(unittest.TestCase):
    def test_shared_mesh_and_clear_pad_are_reproducible(self):
        config=mixed_reachable_config(read_config())['terrain']
        with tempfile.TemporaryDirectory() as folder:
            first,base,record=generate_terrain(42,config,Path(folder)/'a')
            again,_,other=generate_terrain(42,config,Path(folder)/'b')
            np.testing.assert_array_equal(first.h,base.h)
            np.testing.assert_array_equal(first.h,again.h)
            self.assertEqual(digest(record),digest(other))
            angles=np.linspace(0,math.tau,1000)
            np.testing.assert_allclose(first.height(1.3*np.cos(angles),1.3*np.sin(angles)),0,atol=1e-12)
            self.assertLessEqual(np.ptp(first.h),.8+1e-9)
            self.assertGreater(np.ptp(first.h),.3)
            self.assertTrue(record['shared_surface'])
            av,af=read_collada(Path(folder)/'a/terrain.dae')
            bv,bf=read_collada(Path(folder)/'a/terrain_collision.dae')
            np.testing.assert_array_equal(av,bv);np.testing.assert_array_equal(af,bf)

    def test_full_turning_corridors_connect_three_distinct_staging_locations(self):
        objects,regions=targets();field=Terrain(np.zeros((129,129)),20)
        result=validate_layout(field,objects,regions)
        self.assertEqual({r['category'] for r in result['routes']},{'ordinary_rock','crystal_rock','pale_soil'})
        self.assertGreaterEqual(result['corridor_width_m'],1)
        self.assertGreaterEqual(result['corridor_width_m']/2,result['padded_turn_radius_m'])
        self.assertFalse(result['truth_to_navigation'])
        for route in result['routes']:
            self.assertTrue(3<=route['launch_distance_m']<=7)
            for point in route['path_xy']:
                self.assertTrue(all(np.linalg.norm(np.asarray(point)-o['xyz'][:2])>o['radius_m']+.5 for o in objects))

    def test_objects_placed_after_terrain_can_disconnect_a_layout(self):
        objects,regions=targets()
        objects[0]['xyz']=[4,0,0];objects[1]['xyz']=[4,4,0]
        regions[0].update(xyz=[4,-4,0],polygon_xy=[[3.7,-4.3],[4.3,-4.3],[4.3,-3.7],[3.7,-3.7]])
        field=Terrain(np.zeros((129,129)),20)
        validate_layout(field,objects,regions)
        wall=[dict(name=f'wall{i}',xyz=[2,float(y),0],radius_m=.2,class_name='obstacle')
              for i,y in enumerate(np.arange(-10,10.01,.5))]
        with self.assertRaises(LayoutError):validate_layout(field,objects+wall,regions)

    def test_a_point_gap_is_not_a_one_metre_passage(self):
        objects,regions=targets();objects[0]['xyz']=[4,0,0];objects[1]['xyz']=[4,4,0]
        regions[0].update(xyz=[4,-4,0],polygon_xy=[[3.7,-4.3],[4.3,-4.3],[4.3,-3.7],[3.7,-3.7]])
        ys=list(np.arange(-10,-.54,.4))+list(np.arange(.55,10.01,.4))
        wall=[dict(name=f'wall{i}',xyz=[2,float(y),0],radius_m=.2,class_name='obstacle') for i,y in enumerate(ys)]
        with self.assertRaises(LayoutError):validate_layout(Terrain(np.zeros((129,129)),20),objects+wall,regions)

    def test_unknown_profile_and_disagreeing_grids_are_rejected(self):
        config=mixed_reachable_config(read_config())['terrain']
        for changes in [dict(profile='future'),dict(profile_version=2),dict(visual_grid_size=257),dict(layout_attempts=0)]:
            with tempfile.TemporaryDirectory() as folder,self.assertRaises(ValueError):
                generate_terrain(1,dict(config,**changes),folder)

    def test_independent_audit_checks_saved_geometry_and_entire_segments(self):
        from mars_training_environment.meshes import write_dae
        objects,regions=targets();field=Terrain(np.zeros((129,129)),20)
        certificate=validate_layout(field,objects,regions)
        v,f=field.mesh()
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);scene=root/'scenes/scene_000001';scene.mkdir(parents=True)
            for name in ('terrain.dae','terrain_collision.dae'):write_dae(scene/name,v,f)
            mesh=np.array([[-.05,-.05,0],[.05,-.05,0],[0,.05,.08]])
            write_dae(root/'rock.dae',mesh,[[0,1,2]])
            assets=[dict(id='rock',mesh='rock.dae',collision='rock.dae')]
            for obj in objects:obj.update(asset_id='rock',scale=1,rpy=[0,0,0])
            geometry=dict(seed=1,terrain=dict(heights=field.h.tolist(),base_heights=field.h.tolist(),size_m=20),
                          objects=objects,regions=regions)
            self.assertEqual(audit_layout(root,geometry,certificate,assets)['result'],'PASS')
            bad=deepcopy(certificate)
            bad['routes'][0]['path_xy']=[bad['routes'][0]['path_xy'][0],[0,4],bad['routes'][0]['staging_xy']]
            with self.assertRaisesRegex(ValueError,'corridor'):audit_layout(root,geometry,bad,assets)
            # A physical hull omitted from the declared object radius is rejected.
            write_dae(root/'rock.dae',mesh*10,[[0,1,2]])
            with self.assertRaisesRegex(ValueError,'excludes mesh'):audit_layout(root,geometry,certificate,assets)

    def test_retries_change_the_whole_layout_and_preserve_io_errors(self):
        config=mixed_reachable_config(read_config());config['terrain']['layout_attempts']=2
        with patch('mars_training_environment.generate._make_scene',side_effect=[LayoutError('blocked'),{'id':'ok'}]) as make:
            record=make_scene('unused',config,123,[],validation_only=True)
        self.assertEqual(len(record['layout_failures']),1)
        self.assertNotEqual(make.call_args_list[0].args[5],make.call_args_list[1].args[5])
        with patch('mars_training_environment.generate._make_scene',side_effect=LayoutError('blocked')) as make:
            with self.assertRaisesRegex(LayoutError,'after 2 layouts'):make_scene('unused',config,123,[],validation_only=True)
            self.assertEqual(make.call_count,2)
        with patch('mars_training_environment.generate._make_scene',side_effect=OSError('disk')):
            with self.assertRaises(OSError):make_scene('unused',config,123,[],validation_only=True)


if __name__=='__main__':unittest.main()
