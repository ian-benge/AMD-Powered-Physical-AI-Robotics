import unittest
from rover_navigation.metrics import summarize, summarize_trials


class MetricsTests(unittest.TestCase):
    def test_return_uses_mission_start_not_earlier_manual_mapping(self):
        records = [dict(kind='truth', sim_time=t, pose=[x, 0., 0.]) for t, x in ((0., -2.), (1., 0.), (2., 1.), (3., 0.))]
        records += [dict(kind='mission_status', sim_time=1., mission='waypoints', result=''),
                    dict(kind='mission_status', sim_time=3., mission='idle', result='Home reached')]
        report = summarize(records)
        self.assertTrue(report['return_within_tolerance'])
        self.assertEqual(report['final_home_error_m'], 0.)
        self.assertEqual(report['collision_acceptance'], 'not_established_by_trajectory_logging')

    def test_odometry_alignment_removes_frame_offset_not_trajectory_error(self):
        records = [dict(kind='truth', sim_time=t, pose=[x, 0., 0.]) for t, x in ((0., 0.), (1., 1.), (2., 0.))]
        records += [dict(kind='estimate', sim_time=t, pose=[x+10., 20., 0.]) for t, x in ((0., 0.), (1., 1.5), (2., 0.))]
        records += [dict(kind='mission_status', sim_time=0., mission='exploring')]
        self.assertAlmostEqual(summarize(records)['aligned_odometry_max_error_m'], .5)

    def test_incomplete_recording_does_not_claim_home_success(self):
        self.assertFalse(summarize([])['home_reference_available'])

    def test_repeated_trials_keep_original_home_and_require_outward_success(self):
        records = [dict(kind='truth', sim_time=t, pose=[x, 0., 0.])
                   for t, x in ((0., 0.), (1., .5), (2., .1), (3., .1), (4., .6), (5., .2))]
        for trial, start, end in ((1, 0., 2.), (2, 3., 5.)):
            records += [dict(kind='mission_status', trial=trial, mission='waypoints', sim_time=start),
                        dict(kind='mission_status', trial=trial, mission='idle', sim_time=end,
                             outbound_reached=True, result='Home reached')]
        reports = summarize_trials(records)
        self.assertTrue(reports[0]['passed'])
        self.assertFalse(reports[1]['passed'])
        self.assertAlmostEqual(reports[1]['final_home_error_m'], .2)

    def test_startup_status_without_a_clock_does_not_break_trial_report(self):
        records = [dict(kind='status', sim_time=None, mode='stopped', ready=False)]
        records += [dict(kind='truth', sim_time=t, pose=[x, 0., 0.])
                    for t, x in ((0., 0.), (1., .5), (2., 0.))]
        records += [dict(kind='mission_status', trial=1, mission='waypoints', sim_time=0.),
                    dict(kind='mission_status', trial=1, mission='idle', sim_time=2.,
                         outbound_reached=True, result='Home reached')]
        self.assertTrue(summarize_trials(records)[0]['passed'])
        records.append(dict(kind='status', sim_time=1., mode='stopped', ready=False,
                            reason='sensor_or_clock_timeout'))
        self.assertFalse(summarize_trials(records)[0]['passed'])

    def test_return_wander_does_not_count_as_outward_travel(self):
        records = [dict(kind='truth', sim_time=t, pose=[x, y, 0.])
                   for t, x, y in ((0., 0., 0.), (1., .2, 0.), (2., .6, -.4), (3., 0., 0.))]
        records += [dict(kind='mission_status', trial=1, mission='waypoints', sim_time=0., outbound_reached=False),
                    dict(kind='mission_status', trial=1, mission='returning', sim_time=1., outbound_reached=True),
                    dict(kind='mission_status', trial=1, mission='idle', sim_time=3., outbound_reached=True,
                         result='Home reached')]
        report = summarize_trials(records)[0]
        self.assertAlmostEqual(report['outward_displacement_m'], .2)
        self.assertFalse(report['passed'])

    def test_guard_fault_masked_by_a_later_stop_still_fails_its_trial(self):
        records = [dict(kind='truth', sim_time=t, pose=[x, 0., 0.]) for t, x in ((0., 0.), (1., .5), (2., 0.))]
        records += [dict(kind='mission_status', trial=1, mission='waypoints', sim_time=0.),
                    dict(kind='mission_status', trial=1, mission='idle', sim_time=2.,
                         outbound_reached=True, result='Home reached')]
        stale = dict(kind='status', sim_time=.5, mode='autonomous', ready=True, reason='',
                     fault=dict(reason='not_ready', sim_time=-5.))
        self.assertTrue(summarize_trials(records+[stale])[0]['passed'])
        masked = dict(kind='status', sim_time=1.5, mode='stopped', ready=True, reason='operator_stop',
                      fault=dict(reason='map_localization_stale', sim_time=1.4))
        report = summarize_trials(records+[masked])[0]
        self.assertFalse(report['passed'])
        self.assertEqual(report['guard_faults'], ['map_localization_stale'])
