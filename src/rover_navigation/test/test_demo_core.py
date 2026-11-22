import math
import unittest
import numpy as np
from rover_navigation.core import Grid, turn_radius
from rover_navigation.demo_core import (DemoPolicy, DemoRun, SampleBudgetError, collection_poses,
                                        follow_recorded_retreat, frontier_stop_clear, observation_frontiers,
                                        path_length, recorded_retreat, return_reserve, target_signature)
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
        outward=[[0.,0.,0.],[1.,0.,0.]];back=[[.5,0.,0.],[0.,0.,0.]]
        r.set_preview(22.,outward,back,[1.,0.,0.],[0.,0.,0.],self.grid(),target_signature(r.target),
                      home=[0.,0.,0.],target_map=[2.,0.],keepout=[2.,0.,.2])
        return r
    def test_policy_bounds_and_wall_budget(self):
        for kw in (dict(speed=.2),dict(heartbeat_seconds=2.),dict(total_seconds=2000.),dict(collection_seconds=0.),dict(speed=float('nan'))):
            with self.assertRaises(ValueError):DemoPolicy(**kw)
        self.assertEqual(DemoPolicy.rehearsal().total_seconds,720.)

    def test_survey_has_a_mapping_deadline_and_time_for_both_returns(self):
        r=DemoRun('survey',DemoPolicy.survey())
        r.start(0.,'operator-token-one',[0.,0.,0.],0,5.)
        r.transition('exploring',26.)
        r.low_gain_visits=3
        home=[[5.,0.,0.],[0.,0.,0.]]
        self.assertEqual(r.status(130.)['mapping_remaining_seconds'],1370.)
        self.assertEqual(r.exploration_reason(1499.,home,1.,[True]),'')
        self.assertEqual(r.exploration_reason(1499.,home,1.,[]),'No safe useful frontier remains')
        self.assertEqual(r.exploration_reason(1500.,home,1.,[True]),'Survey mapping time reached')
        self.assertEqual(r.exploration_reason(1499.,[[42.,0.,0.],[0.,0.,0.]],1.,[True]),'Return reserve reached')
        r.transition('exploration_return',1500.)
        r.home_arrived(1900.)
        self.assertEqual(r.phase,'awaiting_target')
        self.assertIsNone(r.status(1900.)['mapping_remaining_seconds'])
        self.assertEqual(r.sample_time_left(1900.),500.)
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

    def test_early_home_return_funds_a_longer_checked_sample_trip(self):
        r = self.new_run()
        r.transition('exploration_return', 110.)
        r.home_arrived(230.)
        r.select(244., dict(source='operator_selected', odom_position=[5.9,0.,0.]))
        grid = Grid(np.zeros((240,240),np.int8), .05, -6., -6.)
        home, stop = [0.,0.,0.], [4.4,0.,0.]
        signature = target_signature(r.target)
        with self.assertRaisesRegex(SampleBudgetError, 'Clear round trip needs about'):
            r.set_preview(245., [home,[5.3,0.,0.]], [[4.8,0.,0.],home],
                          [5.3,0.,0.], home, grid, signature,
                          home=home,target_map=[5.9,0.],keepout=[5.9,0.,.2])
        r.set_preview(245., [home,stop], [[3.9,0.,0.],home], stop, home, grid, signature,
                      home=home,target_map=[5.9,0.],keepout=[5.9,0.,.2])
        self.assertGreater(r.preview['estimated_seconds'], r.policy.sampling_seconds)
        self.assertEqual(r.sample_time_left(245.), 370.)
        r.approve(251., r.preview['id'], home, grid, signature)
        r.heartbeat(620., 1)
        r.deadlines(620.)
        self.assertEqual(r.phase, 'sample_outbound')
        r.deadlines(621.)
        self.assertEqual(r.phase, 'stopped')
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
        self.assertTrue(r.collect(34.,True,True,True));self.assertEqual(r.phase,'awaiting_sample_return')
        with self.assertRaises(ValueError):r.home_arrived(59.)
        r.request_sample_return(58.)
        self.assertEqual(r.phase,'sample_backing')
        r.transition('sample_return',59.)
        self.assertFalse(r.sample['delivered']);r.home_arrived(60.)
        self.assertTrue(r.sample['delivered']);self.assertEqual(r.phase,'complete')
    def test_interrupted_collection_commits_nothing(self):
        for stationary,fresh,healthy in ((False,True,True),(True,True,False)):
            r=self.preview();r.approve(23.,r.preview['id'],[0.,0.,0.],self.grid(),target_signature(r.target))
            r.collect(24.,True,True,True);r.collect(30.,stationary,fresh,healthy)
            self.assertEqual(r.phase,'stopped');self.assertIsNone(r.sample)
    def test_sample_evidence_gap_pauses_timer_then_allows_explicit_return(self):
        r=self.preview();r.approve(23.,r.preview['id'],[0.,0.,0.],self.grid(),target_signature(r.target))
        r.collect(24.,True,True,True);r.collect(32.,True,True,True)
        r.collect(32.2,True,False,True)
        self.assertEqual(r.phase,'collecting')
        self.assertEqual(r.collection_elapsed,8.)
        r.collect(32.4,True,True,True)
        r.collect(34.3,True,True,True)
        self.assertIsNone(r.sample)
        self.assertTrue(r.collect(34.5,True,True,True))
        self.assertEqual(r.phase,'awaiting_sample_return')
    def test_lost_sample_evidence_can_return_home_without_collection(self):
        r=self.preview();r.approve(23.,r.preview['id'],[0.,0.,0.],self.grid(),target_signature(r.target))
        r.collect(24.,True,True,True);r.collect(25.6,True,False,True)
        self.assertEqual(r.phase,'awaiting_sample_return')
        r.request_sample_return(26.)
        r.transition('sample_return',37.)
        r.home_arrived(55.)
        self.assertEqual(r.phase,'complete')
        self.assertIsNone(r.sample)
        self.assertIn('without a confirmed sample',r.reason)
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
        poses=collection_poses(self.grid(),[0.,0.],.3,np.eye(3)*.0025,raised_object=True)
        self.assertEqual(len(poses),24)
        self.assertTrue(all(math.hypot(*p[:2])>=1.5-1e-9 for p in poses))
        g=self.grid();g.data[:]=-1
        with self.assertRaises(ValueError):collection_poses(g,[0.,0.])
    def test_confirmed_crystal_can_be_selected_when_center_is_unknown(self):
        g=self.grid()
        g.data[76:84,76:84]=-1
        with self.assertRaisesRegex(ValueError,'measured traversable ground'):
            collection_poses(g,[0.,0.])
        poses=collection_poses(g,[0.,0.],.18,raised_object=True)
        self.assertTrue(any(np.allclose(p[:2],[-1.5,0.]) for p in poses))
        self.assertTrue(all(math.hypot(*pose[:2])>=1.5-1e-9 for pose in poses))
        g.data[:,52:]=-1
        poses=collection_poses(g,[0.,0.],.18,raised_object=True)
        self.assertTrue(any(np.allclose(p[:2],[-2.3,0.]) for p in poses))
        self.assertTrue(all(math.hypot(*pose[:2])>1.5 for pose in poses))
    def test_preview_rejects_return_that_crosses_selected_sample(self):
        r=self.at_home()
        r.select(21.,dict(source='operator_selected',odom_position=[2.,0.,0.]))
        home,stop=[0.,0.,0.],[1.,0.,0.]
        with self.assertRaisesRegex(ValueError,'crosses the selected sample site'):
            r.set_preview(22.,[home,stop],[[.5,0.,0.],[2.,0.,0.],home],stop,home,
                          self.grid(),target_signature(r.target),home=home,target_map=[2.,0.],
                          keepout=[2.,0.,.2])
    def test_reverse_retraces_a_curved_measured_approach(self):
        history=[[2*math.sin(t),2*(1-math.cos(t)),t] for t in np.linspace(0.,.4,21)]
        g=self.grid()
        route=recorded_retreat(g,history[-1],history)
        self.assertAlmostEqual(path_length(route),.50)
        self.assertNotAlmostEqual(route[0][2],route[-1][2])
        progress,look=follow_recorded_retreat(route,route[0],0.)
        self.assertAlmostEqual(progress,0.)
        self.assertLess(look[0],route[0][0])
        self.assertIsNone(follow_recorded_retreat(route,route[-1],.45)[1])
        with self.assertRaisesRegex(ValueError,'gap'):
            recorded_retreat(g,history[-1],history[:8])
        g.data[:,:]=-1
        with self.assertRaisesRegex(ValueError,'hazardous or unknown'):
            recorded_retreat(g,history[-1],history)
    def test_soil_preview_prefers_closer_stops_with_clear_wider_fallbacks(self):
        covariance=np.array([[.06063,.00869,0.],[.00869,.11127,0.],[0.,0.,0.]])
        envelope=.92469
        g=self.grid()
        poses=collection_poses(g,[0.,0.],envelope,covariance)
        ranges=[math.hypot(*pose[:2]) for pose in poses]
        self.assertAlmostEqual(min(ranges),1.2)
        previous=envelope+2*math.sqrt(np.linalg.eigvalsh(covariance[:2,:2]).max())+turn_radius()+.30
        self.assertAlmostEqual(max(ranges),previous)
        self.assertTrue(any(np.allclose(pose[:2],[1.2,0.]) for pose in poses))
        # A hazard beside the eastern preferred pose lies within its turn sweep.
        g.data[80:85,104:107]=100
        poses=collection_poses(g,[0.,0.],envelope,covariance)
        self.assertFalse(any(np.allclose(pose[:2],[1.2,0.]) for pose in poses))
        self.assertTrue(any(np.allclose(pose[:2],[-1.2,0.]) for pose in poses))
        self.assertTrue(any(np.allclose(pose[:2],[previous,0.]) for pose in poses))
    def test_frontiers_face_unknown_and_do_not_use_world_boundaries(self):
        g=self.grid();self.assertEqual(observation_frontiers(g,[0.,0.,0.]),[])
        g.data[:,100:]=-1
        goals=observation_frontiers(g,[0.,0.,0.]);self.assertTrue(goals)
        self.assertTrue(all(g.at(*p['pose'][:2])>=0 for p in goals))
        self.assertTrue(all(abs(p['pose'][2])<.8 for p in goals))
    def test_connected_frontier_has_distinct_safe_alternatives_without_changing_map(self):
        from rover_navigation.core import footprint_clear
        g=self.grid();g.data[:,100:]=-1;original=g.data.copy()
        goals=observation_frontiers(g,[0.,0.,0.],speed=.05)
        self.assertGreater(len(goals),1)
        self.assertLessEqual(len(goals),24)
        groups={p['cluster_id'] for p in goals}
        self.assertEqual(len(groups),1)
        for i,goal in enumerate(goals):
            self.assertTrue(all(footprint_clear(g,*goal['pose'][:2],yaw)
                                for yaw in np.linspace(-math.pi,math.pi,73)))
            self.assertTrue(all(math.dist(goal['pose'][:2],other['pose'][:2])>=.65
                                for other in goals[:i]))
        np.testing.assert_array_equal(g.data,original)
        again=observation_frontiers(g,[0.,0.,0.],speed=.05)
        self.assertEqual(goals,again)
        attempted=[p['pose'][:2] for p in goals]
        for goal in observation_frontiers(g,[0.,0.,0.],attempted,speed=.05):
            self.assertTrue(all(math.dist(goal['pose'][:2],p)>=.65 for p in attempted))
    def test_frontier_ranking_uses_actual_speed_and_turn_limit(self):
        g=self.grid();g.data[:,100:]=-1
        goals=observation_frontiers(g,[0.,0.,math.pi],speed=.05,angular_speed=.15)
        self.assertTrue(goals)
        for goal in goals:
            turn=abs(math.atan2(math.sin(goal['pose'][2]-math.pi),math.cos(goal['pose'][2]-math.pi)))
            bearing=math.atan2(goal['pose'][1],goal['pose'][0])
            outward=.85+.15*math.cos(bearing-math.pi)
            self.assertAlmostEqual(goal['score'],goal['expected_gain_m2']*outward/(1.+goal['distance_m']/.05+turn/.15))
        for values in (dict(speed=0.),dict(speed=.2),dict(speed=math.nan),dict(angular_speed=.4)):
            with self.assertRaises(ValueError):observation_frontiers(g,[0.,0.,0.],**values)
    def test_frontier_keeps_accessible_viewpoints_with_a_cheap_route_home(self):
        g=self.grid()
        g.data[:,100:]=-1
        pose=[-2.,0.,0.]
        home=[-.5,2.,0.]
        goals=observation_frontiers(g,pose,speed=.05,home=home,max_goals=6)
        self.assertGreater(len(goals),2)
        self.assertTrue(any(p['pose'][0]<.45 for p in goals))
        self.assertTrue(any(p['pose'][1]>1. for p in goals))
        self.assertTrue(all(p['travel_seconds']>=p['distance_m']/.05 for p in goals))
    def test_frontier_favors_ahead_and_avoids_obstacles_at_the_stop(self):
        g=self.grid()
        g.data[:,:40]=-1
        g.data[:,120:]=-1
        goals=observation_frontiers(g,[0.,0.,0.])
        self.assertGreater(goals[0]['pose'][0],0.)
        stop=goals[0]['pose']
        g.data[g.cell(stop[0],stop[1]+.46)]=100
        self.assertFalse(frontier_stop_clear(g,stop))
        self.assertTrue(all(frontier_stop_clear(g,p['pose']) for p in observation_frontiers(g,[0.,0.,0.])))

    def test_wider_arrival_requires_room_for_the_full_turn_at_the_edge_of_tolerance(self):
        g=self.grid()
        g.data[g.cell(0.,.58)]=-1
        self.assertTrue(frontier_stop_clear(g,[0.,0.,0.],.12))
        self.assertFalse(frontier_stop_clear(g,[0.,0.,0.]))

    def test_frontier_viewpoint_priority_accounts_for_the_driven_trail(self):
        g=self.grid()
        g.data[:,:40]=-1
        g.data[:,120:]=-1
        trail=[(x,y) for x in (1.1,1.5,1.9) for y in np.arange(-3.8,3.9,.2)]
        ranked=observation_frontiers(g,[0.,0.,0.],trail=trail)
        near_trail=[p['trail_novelty'] for p in ranked if p['pose'][0]>0.]
        self.assertTrue(near_trail)
        self.assertGreater(ranked[0]['trail_novelty'],max(near_trail))
        self.assertLess(ranked[0]['pose'][0],0.)
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
        for i in range(1,15):
            obs=self.observation()
            obs['odom_position'][0]+=.1*(-1)**i
            s.update([obs],float(i),[.21*i,0,0],[.21*i,0,.7])
            item=s.objects[0]
            measured=np.linalg.norm(np.asarray(obs['envelope_odom'])-item['odom_position'][:2],axis=1)
            self.assertAlmostEqual(item['envelope_radius_m'],max(measured))
            self.assertLess(item['envelope_radius_m'],.53)

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
