import math
import unittest
import numpy as np
from rover_navigation.core import Grid
from rover_navigation.demo_core import DemoPolicy, DemoRun, collection_poses, observation_frontiers, return_reserve, target_signature
from rover_navigation.demo_coverage import MeasuredCoverage
from rover_navigation.soil_sites import SoilStore


class DemoTests(unittest.TestCase):
    def grid(self):return Grid(np.zeros((160,160),np.int8),.05,-4.,-4.)
    def new_run(self):
        r=DemoRun('scene-one',DemoPolicy.rehearsal())
        r.start(0.,'operator-token-one',[0.,0.,0.],0,5.)
        return r
    def at_home(self):
        r=self.new_run();r.transition('exploration_return',10.);r.home_arrived(20.)
        return r
    def preview(self):
        r=self.at_home();r.select(21.,dict(source='operator_selected',odom_position=[2.,0.,0.]))
        outward=[[0.,0.,0.],[1.,0.,0.]];back=[[1.,0.,0.],[0.,0.,0.]]
        r.set_preview(22.,outward,back,[1.,0.,0.],[0.,0.,0.],self.grid(),target_signature(r.target))
        return r
    def test_policy_bounds_and_wall_budget(self):
        for kw in (dict(speed=.2),dict(heartbeat_seconds=2.),dict(total_seconds=2000.),dict(collection_seconds=0.),dict(speed=float('nan'))):
            with self.assertRaises(ValueError):DemoPolicy(**kw)
        self.assertEqual(DemoPolicy.rehearsal().total_seconds,720.)
    def test_first_home_return_waits_without_sample_or_motion(self):
        r=self.at_home();self.assertEqual(r.phase,'awaiting_target');self.assertIsNone(r.sample)
        self.assertEqual(r.home,[0.,0.,0.])
    def test_wrong_owner_mission_session_and_epoch_rejected(self):
        r=self.new_run()
        for args in [('other',r.mission_id,r.owner,0),('scene-one','wrong',r.owner,0),('scene-one',r.mission_id,'someone-else-token',0),('scene-one',r.mission_id,r.owner,1)]:
            with self.assertRaises(ValueError):r.verify(*args)
    def test_duplicate_heartbeat_does_not_refresh_lease(self):
        r=self.new_run();r.heartbeat(.2,1)
        with self.assertRaises(ValueError):r.heartbeat(.9,1)
        self.assertEqual(r.heartbeat_wall,.2)
        r.deadlines(1.21);self.assertEqual(r.phase,'stopped')
    def test_stop_invalidates_preview_and_collection(self):
        r=self.preview();ident=r.preview['id'];r.stop(23.,'operator Stop')
        with self.assertRaises(ValueError):r.approve(24.,ident,[0.,0.,0.],self.grid(),target_signature(r.target))
        self.assertIsNone(r.preview)
    def test_preview_expires_at_thirty_seconds(self):
        r=self.preview()
        with self.assertRaises(ValueError):r.approve(52.,r.preview['id'],[0.,0.,0.],self.grid(),target_signature(r.target))
    def test_changed_pose_target_and_map_reject_approval(self):
        for kind in ('pose','target','hazard','unknown'):
            r=self.preview();g=self.grid();pose=[0.,0.,0.];signature=target_signature(r.target)
            if kind=='pose':pose[0]=.1
            if kind=='target':signature='changed'
            if kind in ('hazard','unknown'):g.data[g.cell(.5,.1)]=100 if kind=='hazard' else -1
            with self.subTest(kind=kind),self.assertRaises(ValueError):r.approve(23.,r.preview['id'],pose,g,signature)
    def test_collection_requires_ten_continuous_seconds(self):
        r=self.preview();r.approve(23.,r.preview['id'],[0.,0.,0.],self.grid(),target_signature(r.target))
        self.assertFalse(r.collect(24.,True,True,True));self.assertFalse(r.collect(33.99,True,True,True))
        self.assertTrue(r.collect(34.,True,True,True));self.assertEqual(r.phase,'sample_return')
        self.assertFalse(r.sample['delivered']);r.home_arrived(60.)
        self.assertTrue(r.sample['delivered']);self.assertEqual(r.phase,'complete')
    def test_interrupted_collection_commits_nothing(self):
        for stationary,fresh,healthy in ((False,True,True),(True,False,True),(True,True,False)):
            r=self.preview();r.approve(23.,r.preview['id'],[0.,0.,0.],self.grid(),target_signature(r.target))
            r.collect(24.,True,True,True);r.collect(30.,stationary,fresh,healthy)
            self.assertEqual(r.phase,'stopped');self.assertIsNone(r.sample)
    def test_no_duplicate_collection_or_delivery_before_home(self):
        r=self.preview();r.approve(23.,r.preview['id'],[0.,0.,0.],self.grid(),target_signature(r.target))
        r.collect(24.,True,True,True);r.collect(34.,True,True,True)
        with self.assertRaises(ValueError):r.collect(35.,True,True,True)
        r.stop(36.,'return blocked');self.assertIsNotNone(r.sample);self.assertFalse(r.sample['delivered'])
    def test_selection_and_overall_deadlines_never_depart(self):
        r=self.at_home();r.deadlines(81.);self.assertEqual(r.phase,'stopped');self.assertIsNone(r.sample)
        r=self.new_run();r.heartbeat(719.,1);r.deadlines(720.);self.assertEqual(r.phase,'stopped')
    def test_three_distinct_low_gain_visits_trigger_return(self):
        r=self.new_run();r.transition('exploring',1.)
        for i in range(3):r.visited(2.+i,[i,0.,0.],5.+.2*i)
        self.assertEqual(r.exploration_reason(10.,[[0.,0.,0.]],1.,[True]),'Three distinct low-gain frontier visits')
    def test_repeated_pose_not_counted_as_distinct_frontier(self):
        r=self.new_run()
        for i in range(5):r.visited(i,[0.,0.,0.],5.)
        self.assertEqual(len(r.frontier_visits),1)
    def test_return_reserve_accounts_for_rtf_turns_and_distance(self):
        path=[[0.,0.,0.],[1.,0.,math.pi]]
        self.assertGreater(return_reserve(path,.1,.8),return_reserve(path,.1,1.))
        self.assertTrue(math.isinf(return_reserve([],.1)))
    def test_manual_pre_mapping_cannot_start_accepted_mission(self):
        r=DemoRun('one')
        with self.assertRaises(ValueError):r.start(0.,'long-owner-token',[0.,0.,0.],0,0.,manual_used=True)
    def test_target_pose_outside_envelope_with_full_turn_clearance(self):
        poses=collection_poses(self.grid(),[0.,0.],.3,np.eye(3)*.0025)
        self.assertEqual(len(poses),8)
        self.assertTrue(all(math.hypot(*p[:2])>=1.5-1e-9 for p in poses))
        g=self.grid();g.data[:]=-1
        with self.assertRaises(ValueError):collection_poses(g,[0.,0.])
    def test_frontiers_face_unknown_and_do_not_use_world_boundaries(self):
        g=self.grid();self.assertEqual(observation_frontiers(g,[0.,0.,0.]),[])
        g.data[:,100:]=-1
        goals=observation_frontiers(g,[0.,0.,0.]);self.assertTrue(goals)
        self.assertTrue(all(g.at(*p['pose'][:2])>=0 for p in goals))
        self.assertTrue(all(abs(p['pose'][2])<.8 for p in goals))
    def test_event_phase_override_does_not_break_trajectory_logging(self):
        r=self.new_run();r.event(2.,'trajectory',phase='exploring',wall=2.)
        self.assertEqual(r.events[-1]['phase'],'exploring')


class CoverageTests(unittest.TestCase):
    def test_seeded_clearance_and_apron_never_add_measured_area(self):
        c=MeasuredCoverage(4.,.025);g=Grid(np.zeros((80,80),np.int8),.05,-2.,-2.)
        _,s=c.project(g,[0,0,0],np.eye(3));self.assertEqual(s['measured_area_m2'],0.)
        c.remember(Grid(np.zeros((10,10),np.int8),.025,3.,3.),1.)
        self.assertFalse(c.seen.any())
    def test_observed_terrain_and_correction_are_reprojected(self):
        c=MeasuredCoverage(4.,.025);c.remember(Grid(np.zeros((40,40),np.int8),.025,-.5,-.5),1.)
        g=Grid(np.zeros((80,80),np.int8),.05,-2.,-2.)
        a,s=c.project(g,[0,0,0],np.eye(3));b,t=c.project(g,[.5,0,0],np.eye(3))
        self.assertAlmostEqual(s['measured_area_m2'],1.)
        self.assertAlmostEqual(t['measured_area_m2'],1.)
        self.assertEqual(a.at(0.,0.),0);self.assertEqual(b.at(-.7,0.),0)
        self.assertEqual(b.at(.3,0.),-1)
    def test_unsupported_and_repeated_frames_do_not_inflate_coverage(self):
        c=MeasuredCoverage(4.,.025)
        c.remember(Grid(np.full((20,20),-1,np.int8),.025,0.,0.),1.)
        self.assertFalse(c.seen.any())
        c.remember(Grid(np.zeros((20,20),np.int8),.025,0.,0.),1.)
        self.assertFalse(c.seen.any())


class SoilTests(unittest.TestCase):
    def observation(self):
        return dict(class_name='pale_soil_patch',confidence=.9,odom_position=[1.,0.,0.],covariance=(np.eye(3)*.0025).tolist(),
                    extent=[.3,.3,.01],appearance=None,geometry=dict(supported=True,contradiction=False),
                    envelope_odom=[[.7,-.3],[1.3,-.3],[1.3,.3],[.7,.3]])
    def test_three_translated_ground_views_confirm_soil_and_persist(self):
        s=SoilStore()
        for i in range(3):s.update([self.observation()],float(i),[.21*i,0.,0.],[.21*i,0.,.7])
        self.assertEqual(s.objects[0]['state'],'confirmed')
        s.update([],100.,[0.,0.,0.],[0.,0.,.7]);self.assertEqual(s.objects[0]['state'],'confirmed')
        self.assertGreater(s.objects[0]['envelope_radius_m'],.4)
    def test_rotation_or_repeated_frames_do_not_confirm_soil(self):
        s=SoilStore()
        for i in range(6):s.update([self.observation()],float(i),[0.,0.,0.],[.21*i,0.,.7])
        self.assertNotEqual(s.objects[0]['state'],'confirmed')

    def test_front_axle_pivot_does_not_count_base_link_arc_as_translation(self):
        from rover_navigation.soil_sites import surface_observer
        from rover_navigation.core import DRIVE_CENTER_X
        s=SoilStore()
        for i,angle in enumerate((0.,2*math.pi/3,4*math.pi/3)):
            base=[-DRIVE_CENTER_X*math.cos(angle),-DRIVE_CENTER_X*math.sin(angle),0.,0.,0.,math.sin(angle/2),math.cos(angle/2)]
            observer=surface_observer(base)
            np.testing.assert_allclose(observer[:2],[0.,0.],atol=1e-10)
            s.update([self.observation()],float(i),observer,[math.cos(angle),math.sin(angle),.7])
        self.assertEqual(s.objects[0]['independent_view_count'],1)

    def test_envelope_still_encloses_old_views_after_track_center_moves(self):
        s=SoilStore();s.update([self.observation()],0.,[0,0,0],[0,0,.7])
        old=np.array(s.objects[0]['odom_position'][:2]);radius=s.objects[0]['envelope_radius_m']
        obs=self.observation();obs['odom_position'][0]+=.1
        s.update([obs],1.,[.21,0,0],[.21,0,.7])
        item=s.objects[0]
        self.assertGreaterEqual(item['envelope_radius_m'],radius+np.linalg.norm(np.array(item['odom_position'][:2])-old)-1e-9)

    def test_surface_evidence_accepts_ground_and_rejects_raised_clipped_or_missing_depth(self):
        from rover_navigation.site_geometry import measure
        depth=np.ones((120,120));k=[100,0,60,0,100,60,0,0,1]
        args=([45,45,75,75],[0,0,1],[1,0,0,0],np.zeros((6,6)))
        result=measure(depth,k,*args,surface=True)
        self.assertTrue(result['geometry']['supported']);self.assertEqual(len(result['envelope_odom']),4)
        for value in (.9,float('nan')):
            other=depth.copy();other[45:75,45:75]=value
            with self.assertRaises(ValueError):measure(other,k,*args,surface=True)
        with self.assertRaises(ValueError):measure(depth,k,[0,45,30,75],*args[1:],surface=True)


if __name__=='__main__':unittest.main()
