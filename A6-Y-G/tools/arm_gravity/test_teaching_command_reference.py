"""Offline regressions for published-target progression and interruption recovery."""
import math
import random
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from a6_y_teleop.leader_follower_teleop import LeaderFollowerTeleop, TeleopConfig, validate_config


def mapper():
    m=LeaderFollowerTeleop.__new__(LeaderFollowerTeleop)
    m.config=TeleopConfig(control_hz=100.,step_reference='previous_command',
                         min_position=[-12.5]*6,max_position=[12.5]*6,
                         limit_margin=[0.]*6,max_velocity=[7.5]*3+[5.]*3)
    m.last_safe=m.last_command_time=None
    m.soft_start_begin=m.soft_start_target=None;m.soft_start_time=0.
    m.previous_raw=None;m.previous_raw_time=0.;m.filtered_raw_velocity=None
    m.enable=True;m.previous_enable=False;m.emergency_stop=False
    m.feedback_errors=Mock(return_value=[])
    m.follower_positions=Mock(return_value=[.2]*6)
    m.map_leader_to_follower=Mock(return_value=[1.2]*6)
    m.raw_pub=object();m.safe_pub=object()
    m.publish_action=Mock();m.publish_command=Mock();m.publish_status=Mock()
    return m


def tick(m,now):
    with patch('a6_y_teleop.leader_follower_teleop.time.monotonic',return_value=now):
        m.control_tick()
    return list(m.publish_command.call_args.args[0])


class CommandReferenceTests(unittest.TestCase):
    def test_recorded_j4_startup_uses_gentle_transition_then_normal_following(self):
        m=mapper();m.config.soft_start_duration=1.2
        begin=[0.,0.,0.,math.radians(-20.184883),0.,0.]
        target=[0.,0.,0.,math.radians(2.043624),0.,0.]
        m.follower_positions.return_value=begin;m.map_leader_to_follower.return_value=target
        self.assertEqual(tick(m,0.),begin)
        output=[begin]
        for i in range(1,121):output.append(tick(m,i*.01))
        # Only a small part of the 22-degree gap should close in the first 0.1 s.
        self.assertLess(math.degrees(output[10][3]-begin[3]),.5)
        self.assertAlmostEqual(output[60][3],(begin[3]+target[3])*.5)
        self.assertAlmostEqual(output[-1][3],target[3])
        speeds=[abs(b[3]-a[3])/.01 for a,b in zip(output,output[1:])]
        self.assertLess(max(speeds),math.radians(28.))
        self.assertIsNone(m.soft_start_begin)
        # No extra startup smoothing remains after the handoff.
        m.map_leader_to_follower.return_value=[v+.03 for v in target]
        for q,goal in zip(tick(m,1.21),m.map_leader_to_follower.return_value):self.assertAlmostEqual(q,goal)

    def test_moving_teacher_is_continuous_across_startup_handoff(self):
        m=mapper();m.config.soft_start_duration=1.2
        previous=tick(m,0.);output=[]
        for i in range(1,131):
            now=i*.01;m.map_leader_to_follower.return_value=[1.2+.3*now]*6
            q=tick(m,now);output.append(q)
            for j in range(6):self.assertLessEqual(abs(q[j]-previous[j]),m.config.max_velocity[j]*.01+1e-12)
            previous=q
        self.assertAlmostEqual(output[119][0],1.56)
        self.assertLess(abs(output[119][0]-output[118][0]),.004)
        self.assertAlmostEqual(output[120][0]-output[119][0],.003)

    def test_abrupt_teacher_change_at_handoff_still_respects_rate_limit(self):
        m=mapper();m.config.soft_start_duration=1.2
        for i in range(120):previous=tick(m,i*.01)
        m.map_leader_to_follower.return_value=[10.]*6
        q=tick(m,1.2)
        for j in range(6):self.assertLessEqual(abs(q[j]-previous[j]),m.config.max_velocity[j]*.01+1e-12)

    def test_disable_mid_startup_restarts_from_new_pose_for_full_duration(self):
        m=mapper();m.config.soft_start_duration=1.2
        tick(m,0.);tick(m,.6)
        m.on_enable(SimpleNamespace(data=False));m.on_enable(SimpleNamespace(data=True))
        m.follower_positions.return_value=[-.4]*6
        self.assertEqual(tick(m,1.),[-.4]*6)
        for i in range(1,61):q=tick(m,1.+i*.01)
        for v in q:self.assertAlmostEqual(v,.4)

    def test_legacy_startup_keeps_frozen_endpoint(self):
        m=mapper();m.config.step_reference='feedback'
        m.soft_start_begin=[.2]*6;m.soft_start_target=[1.2]*6;m.soft_start_time=0.
        self.assertEqual(m.apply_soft_start([2.]*6,.1),[1.2]*6)
        self.assertEqual(m.apply_soft_start([2.]*6,.11),[2.]*6)

    def test_start_holds_nonzero_pose_then_reaches_target_with_stationary_feedback(self):
        m=mapper();begin=[-.3,-.02,.1,-.25,.04,-.06]
        m.follower_positions.return_value=begin
        self.assertEqual(tick(m,0.),begin)
        previous=begin
        for i in range(1,101):
            q=tick(m,i*.01)
            for j in range(6):
                self.assertLessEqual(abs(q[j]-previous[j]),m.config.max_velocity[j]*.01+1e-12)
                self.assertGreaterEqual(q[j],begin[j]);self.assertLessEqual(q[j],1.2)
            previous=q
        self.assertEqual(q,[1.2]*6)
        self.assertEqual(m.follower_positions.return_value,begin)

    def test_changing_feedback_after_initialization_does_not_distort_targets(self):
        a,b=mapper(),mapper();tick(a,0.);tick(b,0.)
        for i in range(1,100):
            b.follower_positions.return_value=[.2+.1*math.sin(i+j) for j in range(6)]
            self.assertEqual(tick(a,i*.01),tick(b,i*.01))

    def test_jitter_and_long_pause_never_accumulate_a_large_step(self):
        m=mapper();m.config.soft_start_duration=1e-6
        previous=tick(m,0.);last=0.
        for now in [.006,.020,.070,.071,2.,2.010,2.010,2.009]:
            q=tick(m,now)
            allowed=min(max(now-last,0.),.01)
            for j in range(6):self.assertLessEqual(abs(q[j]-previous[j]),m.config.max_velocity[j]*allowed+1e-12)
            previous=q;last=now

    def test_reversals_and_position_boundaries_keep_steps_bounded(self):
        m=mapper();m.config.min_position=[-1.]*6;m.config.max_position=[1.]*6
        m.config.soft_start_duration=1e-6;rng=random.Random(512)
        previous=tick(m,0.)
        for i in range(1,501):
            m.map_leader_to_follower.return_value=[rng.uniform(-4.,4.) for _ in range(6)]
            q=tick(m,i*.01)
            for j in range(6):
                self.assertTrue(-1.<=q[j]<=1.)
                self.assertLessEqual(abs(q[j]-previous[j]),m.config.max_velocity[j]*.01+1e-12)
            previous=q

    def test_disable_and_reenable_between_ticks_discards_old_reference(self):
        m=mapper();tick(m,0.);tick(m,.1)
        m.on_enable(SimpleNamespace(data=False));m.on_enable(SimpleNamespace(data=True))
        m.follower_positions.return_value=[-.4]*6
        self.assertEqual(tick(m,.11),[-.4]*6)

    def test_emergency_stop_holds_and_restart_initializes_at_feedback(self):
        m=mapper();tick(m,0.);tick(m,.1)
        m.on_emergency_stop(SimpleNamespace(data=True));m.follower_positions.return_value=[.3]*6
        self.assertEqual(tick(m,.11),[.3]*6);self.assertIsNone(m.last_command_time)
        m.on_emergency_stop(SimpleNamespace(data=False));m.follower_positions.return_value=[-.1]*6
        self.assertEqual(tick(m,.12),[-.1]*6)

    def test_feedback_timeout_then_recovery_cannot_resume_old_target(self):
        m=mapper();tick(m,0.);tick(m,.1)
        m.feedback_errors.return_value=['leader feedback timeout']
        self.assertEqual(tick(m,.11),[.2]*6);self.assertIsNone(m.last_command_time)
        m.feedback_errors.return_value=[];m.follower_positions.return_value=[-.5]*6
        self.assertEqual(tick(m,.12),[-.5]*6)

    def test_invalid_leader_and_follower_reset_without_target_catch_up(self):
        for invalid in ['leader','follower']:
            with self.subTest(invalid=invalid):
                m=mapper();tick(m,0.);tick(m,.1)
                method=m.map_leader_to_follower if invalid=='leader' else m.follower_positions
                method.return_value=None
                count=m.publish_command.call_count;tick(m,.11)
                if invalid=='follower':self.assertEqual(count,m.publish_command.call_count)
                self.assertIsNone(m.last_command_time)
                m.map_leader_to_follower.return_value=[1.2]*6;m.follower_positions.return_value=[-.2]*6
                self.assertEqual(tick(m,.12),[-.2]*6)

    def test_raw_generated_and_command_logs_keep_distinct_values(self):
        m=mapper();tick(m,0.);q=tick(m,.01)
        raw,safe=m.publish_action.call_args_list[-2:]
        self.assertIs(raw.args[0],m.raw_pub);self.assertEqual(raw.args[1],[1.2]*6)
        self.assertIs(safe.args[0],m.safe_pub);self.assertEqual(safe.args[1],q)
        self.assertEqual(m.last_safe,q);self.assertNotEqual(q,[1.2]*6)

    def test_failed_publish_does_not_commit_an_unsent_reference(self):
        m=mapper();previous=tick(m,0.)
        m.publish_command.side_effect=RuntimeError('publisher failed')
        with self.assertRaises(RuntimeError):tick(m,.01)
        self.assertEqual(m.last_safe,previous);self.assertEqual(m.last_command_time,0.)

    def test_legacy_configs_keep_original_feedback_relative_behavior(self):
        m=mapper();m.config.step_reference='feedback';m.last_safe=[4.]*6
        q=m.apply_safety([1.]*6,[0.]*6,now=1.)
        for value,expected in zip(q,[.1125]*3+[.075]*3):self.assertAlmostEqual(value,expected)

    def test_invalid_command_reference_limits_are_rejected(self):
        for bad in [0.,-1.,math.inf,math.nan]:
            m=mapper();m.config.max_velocity[4]=bad
            with self.assertRaises(ValueError):validate_config(m.config)
            m=mapper();m.config.max_joint_step=bad
            with self.assertRaises(ValueError):validate_config(m.config)
        m=mapper();m.config.step_reference='typo'
        with self.assertRaises(ValueError):validate_config(m.config)


if __name__=='__main__':unittest.main()
