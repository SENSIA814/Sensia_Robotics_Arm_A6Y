"""Contact discrimination, anti-windup, release, torque limits and fresh-data gates."""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from a6_y_teleop.gripper_haptics import GripperHaptics, GripperHapticConfig
from a6_y_teleop.gripper_haptics_ros import GripperHapticBridge


class HapticTests(unittest.TestCase):
    def setUp(self):
        self.h = GripperHaptics(GripperHapticConfig())

    def contact(self):
        result = None
        for i in range(16):
            result = self.h.step(1+i*.01, -.01, -.011, -.015, 0., .2)
        self.assertTrue(result['contact'])
        return result

    def test_empty_friction_motion_or_short_spike_never_latches(self):
        for torque, velocity, sent in ((.099, 0., -.011), (.2, .003, -.011), (.2, 0., -.015)):
            self.h.reset()
            for i in range(100):
                r = self.h.step(1+i*.01, -.01, sent, -.015, velocity, torque)
                self.assertFalse(r['contact']); self.assertEqual(r['teacher_torque'], 0.)
        self.h.reset()
        for i in range(10):
            self.assertFalse(self.h.step(1+i*.01, -.01, -.011, -.015, 0., .2)['contact'])
        self.assertFalse(self.h.step(1.1, -.01, -.011, -.015, 0., 0.)['contact'])

    def test_capture_stays_fixed_and_torque_bounded_under_continued_squeeze(self):
        r = self.contact(); hold = r['target']; old_torque = r['teacher_torque']
        for i in range(1,100):
            r = self.h.step(1.15+i*.01, -.009+i*.00005, -.005, -.015, 0., .4)
            self.assertEqual(r['target'], hold)
            self.assertTrue(-2.0 <= r['teacher_torque'] <= 0.)
            self.assertLessEqual(abs(r['teacher_torque']-old_torque), .00800001)
            old_torque = r['teacher_torque']

    def test_opening_immediately_releases_and_rebases_accumulated_hand_travel(self):
        self.contact()
        self.h.step(1.16, -.005, -.0149, -.015, 0., .2)
        r = self.h.step(1.17, -.0054, -.0149, -.015, 0., .2)
        self.assertFalse(r['contact']); self.assertEqual(r['teacher_torque'], 0.)
        self.assertLess(r['target'], -.0149)  # Opens now; no unwinding to -.015 first.
        previous = r['target']
        for i in range(1,101):
            raw = -.0054 + (-.04+.0054)*i/100
            r = self.h.step(1.17+i*.01, raw, previous, previous, -.003, 0.)
            self.assertLessEqual(r['target'], previous+1e-12)
            previous = r['target']
        self.assertAlmostEqual(previous, -.04)
        self.assertIsNone(self.h.rebase)

    def test_reverse_during_contact_candidate_cancels_dwell(self):
        for i in range(12):
            self.h.step(1+i*.01, -.01, -.011, -.015, 0., .2)
        self.assertFalse(self.h.step(1.13, -.0105, -.011, -.015, 0., .2)['contact'])

    def test_pause_disable_and_nonfinite_data_zero_output(self):
        for invalid in ('pause','disable','nan'):
            self.h.reset();self.contact()
            if invalid == 'nan':
                with self.assertRaises(ValueError):self.h.step(1.16, math.nan, -.011, -.015, 0., .2)
            else:
                r=self.h.step(2. if invalid=='pause' else 1.16, -.01, -.011, -.015, 0., .2,
                              active=invalid!='disable')
                self.assertEqual(r['teacher_torque'],0.)
            self.assertEqual(self.h.torque,0.);self.assertFalse(self.h.contact)

    def test_configuration_rejects_unsafe_trial_caps(self):
        for params in ({'max_teacher_torque':2.01},{'contact_torque':.01},{'timeout':1.},
                       {'leader_scale':0.},{'feedback_gain':math.nan},{'contact_dwell':0.},
                       {'free_damping_limit':.03},{'load_torque':.1},{'load_dwell':.01}):
            with self.assertRaises(ValueError):GripperHapticConfig(**params)

    def test_free_damping_is_small_passive_and_has_no_stiction_or_sign_memory(self):
        for i,velocity in enumerate((.001, .0239, .15, -.0239, -.15, 0., .0002918)):
            r=self.h.step(1+i*.01, -.02, -.02, -.02, 0., .05, leader_velocity=velocity)
            self.assertFalse(r['contact'])
            self.assertLessEqual(abs(r['teacher_torque']), .015)
            self.assertLessEqual(r['teacher_torque']*velocity, 0.)
            self.assertEqual(r['reflected_torque'], 0.)
            if abs(velocity)<.000956:self.assertEqual(r['teacher_torque'], 0.)
        r=self.h.step(1.07, -.02, -.02, -.02, 0., .05, active=False, leader_velocity=.0239)
        self.assertEqual(r['teacher_torque'],0.)

    def test_soft_object_load_with_tight_tracking_and_motion_still_triggers(self):
        for i in range(10):
            # Recorded failure: substantial load with only micrometres of error.
            actual=-.02+i*.00005
            r=self.h.step(1+i*.01, actual+.00001, actual+.00001, actual, .005, .4)
        self.assertTrue(r['contact']);self.assertEqual(r['contact_reason'],'load')

    def test_jitter_no_longer_resets_stall_but_short_high_load_and_opening_do_not_latch(self):
        for i in range(20):
            r=self.h.step(1+i*.01, -.01+(i%2)*.00001, -.011, -.015, 0., .2)
        self.assertTrue(r['contact'])
        self.h.reset()
        for i in range(5):
            r=self.h.step(1+i*.01, -.01, -.011, -.011, .01, .4)
            self.assertFalse(r['contact'])
        self.assertFalse(self.h.step(1.05, -.01, -.011, -.011, 0., .05)['contact'])
        self.h.reset()
        for i in range(50):
            raw=-.01-i*.0002
            r=self.h.step(1+i*.01, raw, raw, raw, -.02, .4)
            self.assertFalse(r['contact'])

    def test_contact_plus_damping_share_cap_and_opening_removes_wall_component(self):
        self.contact()
        for i in range(300):
            r=self.h.step(1.16+i*.01, -.005, -.015, -.015, 0., 4., leader_velocity=.1)
            self.assertGreaterEqual(r['teacher_torque'], -2.0)
        self.assertAlmostEqual(r['teacher_torque'], -2.0)
        r=self.h.step(4.16, -.0051, -.015, -.015, 0., 4., leader_velocity=-.0239)
        self.assertEqual(r['reflected_torque'],0.)
        self.assertTrue(0 < r['teacher_torque'] <= .015)  # Only small opening damping remains.

    def test_bridge_uses_native_sample_age_not_repeated_publication_age(self):
        bridge=GripperHapticBridge.__new__(GripperHapticBridge);bridge.engine=self.h
        row=dict(position=-.01,velocity=0.,effort=0.,feedback_age=.01,position_age=.01,
                 position_valid=1.,has_feedback=1.,motor_error=1.,fault=0.,source_age=.001)
        bridge.rows={'leader':dict(row),'follower':dict(row)}
        bridge.received={'leader':1.,'follower':1.}
        self.assertEqual(bridge.errors(1.01),[])
        for key,value in (('feedback_age',.081),('position_age',.081),('effort',math.nan),
                          ('motor_error',0.),('fault',1.),('position_valid',0.),('source_age',.081)):
            bridge.rows['follower']=dict(row,**{key:value})
            self.assertTrue(bridge.errors(1.01),key)
        bridge.rows['follower']=dict(row)
        bridge.rows['follower'].update(feedback_age=.05, source_age=.04)
        self.assertTrue(bridge.errors(1.001),'Motor age and transport delay must be added')
        bridge.rows['follower']=dict(row)
        self.assertTrue(bridge.errors(1.09))
        bridge.started=True
        self.assertTrue(bridge.errors(1.09))
        bridge.received={'leader':1.09,'follower':1.09}
        self.assertTrue(bridge.errors(1.10),'A recovered sample must not automatically restart after a fault')

    def test_bridge_only_sends_j7_generalized_effort_and_zeroes_on_stop(self):
        b=GripperHapticBridge.__new__(GripperHapticBridge);b.engine=self.h
        b.effort=Mock();b.gate=Mock();b.status=Mock()
        b.publish(-.06, True, {'state':'contact'})
        data=b.effort.publish.call_args.args[0].data
        self.assertEqual(list(data[:6]),[0.]*6)
        self.assertAlmostEqual(data[6]*.0239,-.06)
        b.stop()
        self.assertEqual(list(b.effort.publish.call_args.args[0].data),[0.]*7)
        self.assertFalse(b.gate.publish.call_args.args[0].data)


if __name__=='__main__':unittest.main()
