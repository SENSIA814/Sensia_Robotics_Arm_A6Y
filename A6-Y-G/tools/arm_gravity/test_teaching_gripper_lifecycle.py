"""Offline lifecycle checks: gripper hold, seven-axis validity and action logging."""
import csv
import io
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from gravity_config import JOINTS
from teaching_gripper import GRIPPER_JOINT
from test_teaching_gripper import model as calibrated_model
from teaching_port import TeachingCoordinator, UpstreamRuntime, action_positions
from teaching_port_session import (NativeHome, PortArmSession, home_targets,
                                   verify_gripper_hold, GRIPPER_HOLD_TOLERANCE)


def model(role='motion_left'):
    return calibrated_model(role)


def fresh_sample(gripper_enabled=1.):
    now=time.monotonic()
    rows={name:dict(has_feedback=1.,feedback_age=.01,fault=0.,motor_error=1.,
                    position_valid=1.,position_age=.01) for name in JOINTS}
    rows[GRIPPER_JOINT]['motor_error']=gripper_enabled
    return dict(q=[0.]*6+[-.012],velocity=[0.]*7,rows=rows,
                sample_time=now,diagnostic_time=now,sample_source_time=now,diagnostic_source_time=now)


class GripperLifecycleTests(unittest.TestCase):
    def test_home_zeroes_only_arm_axes_and_preserves_follower_finger(self):
        q=[.2]*6+[-.012]
        targets=home_targets(model(),q)
        self.assertEqual(list(targets),list(JOINTS))
        self.assertEqual(list(targets.values())[:6],[0.]*6)
        self.assertEqual(targets[GRIPPER_JOINT],q[6])
        self.assertEqual(q,[.2]*6+[-.012])
        self.assertEqual(home_targets(model('teaching_left'),q)[GRIPPER_JOINT],q[6])

    def test_home_refuses_invalid_finger_feedback(self):
        for value in (float('nan'),float('inf'),.1,-.1):
            with self.assertRaises(ValueError):home_targets(model(),[0.]*6+[value])
        with self.assertRaises(ValueError):home_targets(model(),[0.]*6)

    def test_home_accepts_bounded_feedback_tolerance_without_clamping_measured_hold(self):
        from teaching_gripper import gripper_profile
        profile=gripper_profile(model())
        value=profile['measured_closed_position_m']+profile['feedback_tolerance_m']*.5
        self.assertEqual(home_targets(model(),[0.]*6+[value])[GRIPPER_JOINT],value)

    def test_gripper_hold_is_checked_in_metres_not_arm_rad_tolerance(self):
        targets=home_targets(model(),[0.]*6+[-.012])
        verify_gripper_hold([0.]*6+[-.012+GRIPPER_HOLD_TOLERANCE*.5],targets)
        with self.assertRaises(RuntimeError):verify_gripper_hold([0.]*6+[0.],targets)
        with self.assertRaises(RuntimeError):verify_gripper_hold([0.]*6+[float('nan')],targets)

    def test_native_home_explicitly_passes_seven_joints_to_original_client(self):
        with tempfile.TemporaryDirectory() as folder:
            child=SimpleNamespace(poll=lambda:0,returncode=0)
            q=np.array([0.]*6+[-.012])
            session=SimpleNamespace(model=model(),namespace='/motion',
                feedback=SimpleNamespace(measured=lambda:q),
                release_event=threading.Event(),stop_event=threading.Event(),cancel_home=threading.Event(),
                _select=Mock(),update=Mock(),_event=Mock(),
                _fresh=lambda **_:True,_wait=lambda predicate,**_:self.assertTrue(predicate()),
                stack=SimpleNamespace(directory=Path(folder),spawn=Mock(return_value=child)))
            home=NativeHome.__new__(NativeHome);home.session=session;home.cancel=Mock()
            home.execute(exiting=True)
            args=session.stack.spawn.call_args.args[3]
            specified=[args[i+1] for i,value in enumerate(args[:-1]) if value=='--joint']
            self.assertEqual(specified,list(JOINTS))
            self.assertIn('value="-0.012"',(Path(folder)/'native_home.srdf').read_text())
            home.cancel.assert_called_once()

    def arm(self,role='motion_left',sample=None):
        arm=PortArmSession.__new__(PortArmSession)
        arm.model=model(role);arm.lock=threading.Lock();arm.state={'mock':False}
        arm.feedback=SimpleNamespace(snapshot=lambda:sample or fresh_sample())
        return arm

    def test_enabled_follower_and_haptic_teacher_require_gripper_enabled(self):
        self.assertTrue(self.arm()._fresh(enabled=True,sample=fresh_sample(1.)))
        self.assertFalse(self.arm()._fresh(enabled=True,sample=fresh_sample(0.)))
        teacher=self.arm('teaching_left')
        self.assertFalse(teacher._fresh(enabled=True,sample=fresh_sample(0.)))
        self.assertTrue(teacher._fresh(enabled=True,sample=fresh_sample(1.)))

    def test_disabled_confirmation_includes_the_seventh_motor(self):
        sample=fresh_sample(0.)
        for row in sample['rows'].values():row['motor_error']=0.
        arm=self.arm()
        self.assertTrue(arm._fresh(disabled=True,sample=sample))
        sample['rows'][GRIPPER_JOINT]['motor_error']=1.
        self.assertFalse(arm._fresh(disabled=True,sample=sample))

    def test_stale_or_invalid_gripper_cannot_authorize_follow(self):
        arm=self.arm()
        for key,value in (('position_valid',0.),('position_age',.51)):
            sample=fresh_sample();sample['rows'][GRIPPER_JOINT][key]=value
            self.assertFalse(arm._fresh(enabled=True,sample=sample))

    def test_recorded_feedback_beyond_calibration_does_not_block_start_or_hold(self):
        from teaching_gripper import gripper_profile
        recorded = .00010540215444589034  # teaching_888c6fdad1 startup failure.
        for role in ('teaching_left','teaching_right','motion_left','motion_right'):
            profile=gripper_profile(model(role))
            for value in (recorded,profile['measured_closed_position_m']+.001,
                          profile['measured_open_position_m']-.001):
                q=[0.]*6+[value]
                bounds={n:(-12.5,12.5) for n in JOINTS}
                with patch('teaching_port_session.position_bounds',return_value=bounds):
                    self.arm(role)._validate_enable_positions(q)
                targets=home_targets(model(role),q)
                self.assertEqual(q[-1],value)
                self.assertEqual(targets[GRIPPER_JOINT],value)

    def test_enable_still_rejects_nonfinite_or_nondecodable_gripper_position(self):
        bounds={n:(-12.5,12.5) for n in JOINTS}
        with patch('teaching_port_session.position_bounds',return_value=bounds):
            self.arm()._validate_enable_positions([0.]*6+[-.02])
            for value in (float('nan'),float('inf'),.1,-.1):
                with self.assertRaisesRegex(ValueError,'夹爪协议范围'):
                    self.arm()._validate_enable_positions([0.]*6+[value])

    def test_seven_axis_action_log_uses_joint_names_and_keeps_gripper(self):
        message=SimpleNamespace(name=list(reversed(JOINTS)),position=[-.012,6.,5.,4.,3.,2.,1.])
        self.assertEqual(action_positions(message),[1.,2.,3.,4.,5.,6.,-.012])
        with self.assertRaisesRegex(ValueError,'缺少'):
            action_positions(SimpleNamespace(name=list(JOINTS[:6]),position=[0.]*6))
        runtime=UpstreamRuntime.__new__(UpstreamRuntime)
        runtime.count={'motion_left':0};runtime.last_action={};runtime.raw={}
        runtime.owner=SimpleNamespace(sessions={
            role:SimpleNamespace(feedback=SimpleNamespace(snapshot=lambda:{'q':[0.]*6+[-.02]}))
            for role in ('teaching_left','motion_left')})
        stream=io.StringIO();runtime.writer=csv.writer(stream)
        runtime.on_action('teaching_left','motion_left',message)
        row=next(csv.reader(io.StringIO(stream.getvalue())))
        self.assertEqual(len(row),4+7*4)
        self.assertEqual(float(row[4+7+6]),-.012)

    def test_sample_validates_finger_velocity_in_motor_protocol_units(self):
        c=TeachingCoordinator.__new__(TeachingCoordinator)
        c.roles={'motion_left':{'name':'motion'}}
        c.limit_records={'motion_left':{'feedback_velocity_limits':[10.]*6,'motors':{7:{'VMAX':30.}}}}
        sample=fresh_sample()
        session=self.arm(sample=sample)
        session.control_state=lambda **_:dict(role='motion_left',active=True,hardware=sample)
        self.assertEqual(c._sample(session).shape,(7,))
        sample['velocity'][6]=.1  # Above command slew .015 but valid encoder speed.
        c._sample(session)
        sample['velocity'][6]=1.  # Exceeds 30 rad/s * .0239 m/rad.
        with self.assertRaisesRegex(RuntimeError,'VMAX'):c._sample(session)


if __name__=='__main__':unittest.main()
