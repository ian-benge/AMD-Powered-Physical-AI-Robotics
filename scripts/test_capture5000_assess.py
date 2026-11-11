import copy
import json
from pathlib import Path
import tempfile
import unittest
from capture5000_assess import candidate_settings,check_terrain_result,verify_status,NAMES


class CandidateAssessment(unittest.TestCase):
    def example(self):
        candidate=dict(checkpoint='/new/weights.pt',checkpoint_sha256='new-source',model_sha256='new-onnx',
            manifest='/new/manifest.json',cache_key='cache',compile_report_sha256='compile',runtime_fingerprint={'kernel':'qualified'})
        model=dict(backend='npu',precision='a16w8',output_format='yolo_dfl',model_family='YOLOv8',
            source_checkpoint=candidate['checkpoint'],source_checkpoint_sha256='new-source',model_sha256='new-onnx',
            manifest=candidate['manifest'],class_names=list(NAMES),input_size=[576,960],proof_state='npu_executed',
            computation_offload_verified=True,cache_key='cache',compile_evidence={'verified':True,'sha256':'compile'},
            runtime_fingerprint={'kernel':'qualified'})
        return dict(ready=True,error=None,model=model),dict(candidate=candidate)

    def test_baseline_wrong_model_runtime_or_unproven_npu_never_pass(self):
        status,release=self.example();verify_status(status,release)
        for key,value in [('source_checkpoint','/old/weights.pt'),('source_checkpoint_sha256','old-source'),
            ('model_sha256','old-onnx'),('backend','torch'),('proof_state','cpu_only'),
            ('computation_offload_verified',False),('runtime_fingerprint',{'kernel':'other'}),
            ('compile_evidence',{'verified':False,'sha256':'compile'})]:
            changed=copy.deepcopy(status);changed['model'][key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):verify_status(changed,release)
        status['ready']=False
        with self.assertRaises(RuntimeError):verify_status(status,release)

    def test_browser_profile_preserves_camera_mapping_and_motion_settings(self):
        original=dict(map_size_m=40.,manual_max_linear_speed=.2,manual_initial_linear_speed=.05,
            manual_max_angular_speed=.3,sensor_render_engine='ogre',adaptive_terrain='true',
            detector_url='http://127.0.0.1:8775',detector_model='/new/weights.pt',rviz='true',gui='true',
            sensor_headless='false',camera=dict(width=960,height=540,rgb_far_m=20.,far_m=8.))
        configured=candidate_settings(lambda *_:dict(original),dict(model_assessment=dict(camera_rate_hz=15)), {})
        self.assertEqual(configured.pop('detector_url'),'http://127.0.0.1:8778')
        self.assertEqual(configured.pop('rviz'),'false')
        self.assertEqual(configured.pop('camera_rate_hz'),15)
        self.assertEqual(configured,{k:v for k,v in original.items() if k not in ('detector_url','rviz')})

    def test_terrain_thermal_stop_is_explicit_and_never_retried_as_pid_race(self):
        with tempfile.TemporaryDirectory() as directory:
            case=Path(directory);check_terrain_result(case)
            evidence=case/'environment-v2-terrain-final.json'
            evidence.write_text(json.dumps(dict(failure='Stop: Tctl reached the 90 C experiment threshold or is invalid',
                operator_stop=False,worker_exit=-15)))
            with self.assertRaisesRegex(RuntimeError,'Terrain service stopped: Stop: Tctl reached the 90 C'):
                check_terrain_result(case)
            evidence.write_text(json.dumps(dict(failure=None,operator_stop=True,worker_exit=0)))
            with self.assertRaisesRegex(RuntimeError,'Terrain service stopped: operator stop'):
                check_terrain_result(case)


if __name__=='__main__':unittest.main()
