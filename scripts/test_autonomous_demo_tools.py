import copy
import unittest
from edit_autonomous_demo import validate,srt_time
from prepare_demo_diagnostics import reserve
from bulk10000_plan import split_for_seed
from autonomous_demo_assess import demo_settings


class DemoToolTests(unittest.TestCase):
    def test_contact_fixture_cannot_start_an_autonomous_mission_or_change_sensor_profile(self):
        original=lambda c,s:dict(camera_rate_hz=15,terrain_backend='rocm_http',sensor_render_engine='ogre')
        contact=demo_settings(original,{},None,'rehearsal',.05,'flat')
        self.assertFalse(contact['demo_mode']);self.assertEqual(contact['fixture'],'flat')
        self.assertEqual((contact['camera_rate_hz'],contact['sensor_render_engine']),(15,'ogre'))
        with self.assertRaises(ValueError):demo_settings(original,{},None,'rehearsal',.10,'flat')
        normal=demo_settings(original,{},None,'rehearsal',.05)
        self.assertTrue(normal['demo_mode']);self.assertEqual(normal['fixture'],'')
    def test_browser_demo_disables_only_display_windows_and_keeps_sensor_contract(self):
        retained=dict(gui='true',rviz='true',camera_rate_hz=15,sensor_headless='false',sensor_render_engine='ogre',
            camera=dict(width=960,height=540,far_m=8,rgb_far_m=20,tilt_rad=.4),terrain_backend='rocm_http',
            map_size_m=40,adaptive_terrain='true',detector_url='http://127.0.0.1:8778')
        configured=demo_settings(lambda *_:copy.deepcopy(retained),{},None,'rehearsal',.05)
        self.assertEqual((configured['gui'],configured['rviz']),('false','false'))
        for key,value in retained.items():
            if key not in ('gui','rviz'):self.assertEqual(configured[key],value,key)
        self.assertEqual(retained['gui'],'true')
    def test_single_accepted_original_can_produce_a_captioned_four_minute_edit(self):
        plan=dict(mission_id='one',original_sha256='hash',clips=[
            dict(start_seconds=0,end_seconds=220,speed=1,caption='Exploration and return'),
            dict(start_seconds=220,end_seconds=230,speed=1,caption='Simulated collection',simulated_collection=True),
            dict(start_seconds=230,end_seconds=250,speed=2,caption='Final return')])
        review=dict(passed=True,mission_id='one',original_sha256='hash')
        self.assertEqual(validate(plan,review,300),240)
        for change in ('wrong_original','wrong_mission','too_short','overlap','compressed_collection'):
            p,r=copy.deepcopy(plan),dict(review)
            if change=='wrong_original':r['original_sha256']='other'
            elif change=='wrong_mission':r['mission_id']='other'
            elif change=='too_short':p['clips'][0]['speed']=8
            elif change=='overlap':p['clips'][1]['start_seconds']=210
            else:p['clips'][1]['speed']=2
            with self.subTest(change=change),self.assertRaises(ValueError):validate(p,r,300)
        self.assertEqual(srt_time(3661.125),'01:01:01,125')
    def test_reservation_is_deterministic_and_does_not_generate_training_images(self):
        p=reserve();self.assertEqual(p,reserve());self.assertEqual(sum(s['views'] for s in p['scenes']),100)
        self.assertEqual(sum(s['background'] for s in p['scenes']),1)
        self.assertEqual(len({s['scene_group'] for s in p['scenes']}),5)
        self.assertTrue(all(split_for_seed(s['seed'])=='test' for s in p['scenes']))
        self.assertFalse(p['captured']);self.assertFalse(p['training_permitted'])


if __name__=='__main__':unittest.main()
