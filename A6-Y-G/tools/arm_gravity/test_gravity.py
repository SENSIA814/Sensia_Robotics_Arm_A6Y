import math
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch, Mock
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import yaml

from gravity_config import (ROOT, ROLE_ORDER, JOINTS, VECTOR_KEYS, GravityModel, base_parameters,
    load_model, load_roles, validate_parameters, save_profile, load_profile, control_description, write_config,
    motor_limit_report, MOTOR_LIMITS_SOURCE)
from gravity_session import GravitySession, GravityStack, EnableBlocked


class ConfigurationTests(unittest.TestCase):
    def test_motor_feedback_limits_match_decode_ranges_for_every_role(self):
        for role in ROLE_ORDER:
            model=load_model(role);record=motor_limit_report(model)
            expected=[10.]*3+([50.]*3 if role.endswith('left') else [30.]*3)
            self.assertEqual(record['feedback_velocity_limits'],expected)
            self.assertEqual(record['usb_serial'],model['arm']['usb_serial'])
            with tempfile.TemporaryDirectory() as folder:
                config=yaml.safe_load(write_config(model,base_parameters(),folder,'/test','manager',True,True).read_text())
                self.assertEqual(config['/test/a6_y_gravity_comp_controller']['ros__parameters']['feedback_velocity_limits'],expected)
                xml=ET.fromstring(config['/test/manager']['ros__parameters']['robot_description'])
                decoded=[float(j.find("param[@name='protocol_vmax']").text) for j in xml.findall('ros2_control/joint')[:6]]
                self.assertEqual(decoded,expected)
            # Coordinate scaling applies once, and sign does not negate a bound.
            model['joints']['joint4']['position_scale']=.5
            self.assertEqual(motor_limit_report(model)['feedback_velocity_limits'][3],expected[3]*.5)

    def test_motor_record_never_falls_back_to_an_invented_limit(self):
        model=load_model('teaching_left');model['arm']['usb_serial']='unrecorded'
        with self.assertRaisesRegex(ValueError,'USB'):motor_limit_report(model)
        model=load_model('teaching_left')
        for bad in (0.,-1.,float('nan')):
            data=json.loads(MOTOR_LIMITS_SOURCE.read_text())
            data['arms']['teaching_left']['motors'][3]['parameters']['VMAX']=bad
            with tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'limits.json';path.write_text(json.dumps(data))
                with patch('gravity_config.MOTOR_LIMITS_SOURCE',path),self.assertRaises(ValueError):
                    motor_limit_report(model)

    def test_four_roles_preserve_verified_mapping_and_pmax(self):
        for role in ROLE_ORDER:
            model=load_model(role)
            for active in (False,True):
                xml=ET.fromstring(control_description(model,False,active))
                hardware={p.get('name'):p.text for p in xml.findall('ros2_control/hardware/param')}
                self.assertEqual(hardware['can_channel'],model['arm']['port'])
                self.assertEqual(hardware['control_mode'],'mit')
                self.assertEqual(hardware['mit_kp'],'0.0'); self.assertEqual(hardware['mit_kd'],'0.0')
                for i,joint in enumerate(xml.findall('ros2_control/joint')):
                    p={p.get('name'):p.text for p in joint.findall('param')}
                    entry=model['joints'][JOINTS[i]]
                    self.assertEqual(float(p['protocol_pmax']),entry['position_scale_evidence']['pmax_rad'])
                    self.assertEqual(float(p['sign']),entry['sign'])
                    self.assertEqual(p['read_only'],'true' if not active or i == 6 else 'false')

    def test_profile_roundtrip_isolated_by_role_and_serial(self):
        with tempfile.TemporaryDirectory() as folder:
            p=base_parameters(); p['torque_scales'][1]=.77
            save_profile('motion_left',p,directory=folder)
            self.assertEqual(load_profile('motion_left',directory=folder)['torque_scales'][1],.77)
            self.assertEqual(load_profile('motion_right',directory=folder),base_parameters())
            roles=load_roles(); roles['motion_left']['usb_serial']='wrong'
            with self.assertRaises(ValueError): load_profile('motion_left',roles,directory=folder)

    def test_reject_unsafe_parameters(self):
        cases=[('torque_scales',0,float('nan')),('effort_signs',2,0),('damping_gains',0,-1),
               ('torque_limits',0,29),('torque_limits',6,1),('friction_compensations',2,21)]
        for key,index,value in cases:
            p=base_parameters(); p[key][index]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError): validate_parameters(p)

    def test_exact_original_parameters_except_readonly_gripper(self):
        original=yaml.safe_load((ROOT/'src/a6_y_controllers/config/a5_gravity_controllers.yaml').read_text())['/arm6/a6_y_gravity_comp_controller']['ros__parameters']
        p=base_parameters()
        for key in VECTOR_KEYS: self.assertEqual(p[key][:6],original[key][:6])
        self.assertEqual(p['torque_limits'][6],0.)

    def test_scoped_config_pins_model_and_native_plugin(self):
        model=load_model('teaching_right')
        with tempfile.TemporaryDirectory() as f:
            path=write_config(model,base_parameters(),f,'/test','manager',True,False)
            config=yaml.safe_load(path.read_text())
            p=config['/test/a6_y_gravity_comp_controller']['ros__parameters']
            self.assertEqual(p['urdf_path'],model['urdf_path'])
            self.assertEqual(p['hardware_enable_interface'],'motor_enable')
            self.assertFalse(p['haptic_enable_output'])
            self.assertEqual(set(config),{'/test/manager','/test/a6_y_gravity_comp_controller'})


class DynamicsTests(unittest.TestCase):
    def test_gravity_is_potential_gradient_with_mimic(self):
        model=load_model('motion_left'); math_model=GravityModel(model)
        full=pin.buildModelFromXML(model['description'],False)
        fd=full.createData()
        rng=np.random.default_rng(25); maximum=0.
        for _ in range(40):
            q=np.array([rng.uniform(model['joints'][n]['lower'],model['joints'][n]['upper']) for n in JOINTS])
            g=math_model.gravity(q)
            for j in range(7):
                eps=1e-6
                plus=q.copy(); minus=q.copy(); plus[j]+=eps; minus[j]-=eps
                fullp=np.r_[plus,-plus[-1]]; fullm=np.r_[minus,-minus[-1]]
                grad=(pin.computePotentialEnergy(full,fd,fullp)-pin.computePotentialEnergy(full,fd,fullm))/(2*eps)
                maximum=max(maximum,abs(grad-g[j]))
        self.assertLess(maximum,2e-7)

    def test_invalid_position_never_computes_torque(self):
        model=GravityModel(load_model('teaching_left'))
        for q in ([0]*6,[0]*6+[float('inf')]):
            with self.assertRaises(ValueError): model.gravity(q)


class SessionTests(unittest.TestCase):
    def test_home_merges_only_near_stationary_submillisecond_terminal_sample(self):
        from gravity_home import home_trajectory, HOME_SPEED, HOME_ACCEL
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        msg=JointTrajectory(joint_names=list(JOINTS[:6]))
        for seconds,q,v in ((0.,.2,0.),(2.5,6.4e-8,-.00016),(2.5008,0.,0.)):
            point=JointTrajectoryPoint(positions=[q]*6,velocities=[v]*6)
            point.time_from_start.sec=int(seconds)
            point.time_from_start.nanosec=round((seconds-int(seconds))*1e9)
            msg.points.append(point)
        plan=home_trajectory(msg,load_model('teaching_right'))
        self.assertEqual(len(msg.points),3)  # original planner evidence unchanged
        self.assertEqual(len(plan.times),2)
        np.testing.assert_array_equal(plan.q[[0,-1]],[[.2]*6,[0.]*6])
        self.assertIsNotNone(plan.merged_terminal_sample)
        self.assertLessEqual(max(plan.peak_velocity),HOME_SPEED)
        self.assertLessEqual(max(plan.peak_acceleration),HOME_ACCEL)
        for bad in (float('nan'),.01):
            msg.points[-2].positions=[bad]*6
            with self.assertRaises(ValueError):home_trajectory(msg,load_model('teaching_right'))
        msg.points[-2].positions=[6.4e-8]*6
        msg.points[-1].velocities=[.01]*6
        with self.assertRaises(ValueError):home_trajectory(msg,load_model('teaching_right'))
        msg.points[-1].velocities=[0.]*6
        msg.points[-1].time_from_start.nanosec=500000000
        with self.assertRaises(ValueError):home_trajectory(msg,load_model('teaching_right'))

    def test_home_cubic_retiming_enforces_new_absolute_limits(self):
        from gravity_home import home_trajectory, HOME_SPEED, HOME_ACCEL
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        msg=JointTrajectory(joint_names=list(JOINTS[:6]))
        for seconds,q in ((0.,.2),(.3,0.)):
            point=JointTrajectoryPoint(positions=[q]*6,velocities=[0.]*6)
            point.time_from_start.sec=int(seconds)
            point.time_from_start.nanosec=round((seconds-int(seconds))*1e9)
            msg.points.append(point)
        plan=home_trajectory(msg,load_model('teaching_right'))
        self.assertLessEqual(max(plan.peak_velocity),HOME_SPEED)
        self.assertLessEqual(max(plan.peak_acceleration),HOME_ACCEL)
        # New velocity-limited path is faster than the old .075 rad/s request.
        self.assertLess(plan.duration,1.5*.2/.075)
        self.assertGreater(plan.duration,.3)

    def test_each_home_plan_keeps_exact_execution_path_and_parameters(self):
        from gravity_home import GravityHome, HOME_STIFFNESS_RAMP
        with tempfile.TemporaryDirectory() as folder:
            home=GravityHome.__new__(GravityHome);home.attempt=0
            home.s=SimpleNamespace(stack=SimpleNamespace(directory=Path(folder)),params=base_parameters())
            plan=SimpleNamespace(times=np.array([0.,4.]),q=np.array([[.1]*6,[0.]*6]),
                                 v=np.zeros((2,6)),peak_velocity=np.full(6,.0375),peak_acceleration=np.full(6,.0375))
            home.record_plan(plan)
            first=Path(folder,'home_plan_001.json').read_text()
            plan.q[0,0]=.15;home.record_plan(plan)
            self.assertEqual(Path(folder,'home_plan_001.json').read_text(),first)
            latest=json.loads(Path(folder,'home_plan.json').read_text())
            self.assertEqual(latest,json.loads(Path(folder,plan.home_file).read_text()))
            self.assertEqual(latest['execution_times'],[0.,HOME_STIFFNESS_RAMP,4.+HOME_STIFFNESS_RAMP])
            self.assertEqual(latest['execution_positions'][0],latest['execution_positions'][1])
            self.assertEqual(latest['execution_velocities'][0],[0.]*6)
            self.assertEqual(latest['parameters'],base_parameters())

    def test_home_planning_tolerance_preserves_actual_feedback_and_rejects_larger_residual(self):
        from gravity_home import planning_start
        model=load_model('teaching_right');q=[0.,-.001335,.00553,0.,0.,0.,-.01]
        planned,residuals=planning_start(model,q)
        self.assertEqual(q[1],-.001335)
        self.assertEqual(planned[1],0.)
        self.assertEqual(residuals[0]['joint'],'joint2')
        q[1]=-.006
        with self.assertRaises(ValueError):planning_start(model,q)
        q[1]=float('nan')
        with self.assertRaises(ValueError):planning_start(model,q)

    def feedback_session(self, enabled_ids=()):
        from native_stack import NativeFeedback
        s=GravitySession(); s.feedback=NativeFeedback({},False)
        s.update(mock=False,ready=True,running=True)
        now=time.monotonic()
        rows={n:dict(has_feedback=1.,feedback_age=0.,fault=0.,enabled=0.,
                     motor_error=1. if i+1 in enabled_ids else 0.) for i,n in enumerate(JOINTS)}
        s.feedback.update(q=[0.]*7,velocity=[0.]*7,sample_time=now,diagnostic_time=now,
                          sample_source_time=now,diagnostic_source_time=now,rows=rows)
        s.stack=Mock();s.output_started=False
        return s

    def test_enabled_feedback_is_not_a_timeout_and_cannot_acquire_output(self):
        s=self.feedback_session(range(1,7))
        self.assertTrue(s._fresh())
        self.assertFalse(s._fresh(disabled=True))
        with patch.object(s,'_wait',side_effect=lambda predicate,**kwargs:self.assertTrue(predicate())):
            with self.assertRaisesRegex(EnableBlocked,'J1.*J6.*仍报告已使能'):
                s._enable()
        s.stack.stop_control.assert_not_called()
        s.stack.start_control.assert_not_called()
        self.assertFalse(s.output_started)

    def test_gripper_enabled_or_stale_feedback_also_blocks_start(self):
        s=self.feedback_session([7])
        self.assertIn('夹爪',s.enable_block_reason())
        s=self.feedback_session()
        self.assertEqual(s.enable_block_reason(),'')
        s.feedback.update(diagnostic_time=time.monotonic()-1.)
        self.assertIn('反馈',s.enable_block_reason())

    def test_ui_uses_reported_motor_state_and_recovers_when_disabled(self):
        from PyQt5.QtWidgets import QApplication
        from gravity_view import GravityWindow
        app=QApplication.instance() or QApplication([])
        w=GravityWindow();w.select_role('teaching_right')
        try:
            w.session=self.feedback_session(range(1,7));w.sessions[w.role]=w.session;w.refresh()
            self.assertTrue(w.enable_button.isEnabled())
            self.assertTrue(w.release_button.isEnabled())
            self.assertIn('仍报告已使能',w.status.text())
            self.assertNotIn('超时',w.status.text())
            rows=w.session.feedback.snapshot()['rows']
            for row in rows.values(): row['motor_error']=0.
            now=time.monotonic()
            w.session.feedback.update(rows=rows,sample_time=now,diagnostic_time=now)
            w.refresh()
            self.assertTrue(w.enable_button.isEnabled())
            self.assertNotIn('仍报告已使能',w.status.text())
        finally:
            w.session.update(running=False);w.close();app.processEvents()

    def test_dead_planner_is_reported_without_stopping_gravity_control(self):
        stack=GravityStack.__new__(GravityStack)
        planner=Mock();planner.poll.return_value=134
        controller=Mock();controller.poll.return_value=None
        stack.children=[('move_group',planner),('controller_manager',controller)]
        stack.check()
        self.assertEqual(stack.children,[('controller_manager',controller)])
        self.assertFalse(stack.home_planner_started)
        self.assertIn('重力补偿保持',stack.home_planner_error)
        controller.terminate.assert_not_called()

    def test_release_and_close_immediately_block_another_enable(self):
        for method in ('release','stop'):
            s=GravitySession();s.update(ready=True,running=True)
            getattr(s,method)()
            with self.assertRaises(ValueError):s.enable()
            self.assertEqual(s.commands.qsize(),0)

    def test_normal_close_never_acquires_external_enabled_motors(self):
        s=self.feedback_session(range(1,7));s.home=None;s.close_event.set()
        s._close_normally()
        self.assertFalse(s.stop_event.is_set())
        self.assertFalse(s.close_event.is_set())
        self.assertTrue(s.snapshot()['close_failed'])
        s.stack.start_control.assert_not_called()
        s.stack.stop_control.assert_not_called()

    def test_home_failure_keeps_session_and_repeated_close_is_idempotent(self):
        s=self.feedback_session();s.update(active=True);s.home=Mock()
        s.feedback.state['q']=[.2]*6+[0.]
        s.home.plan.side_effect=ValueError('collision')
        s.stop();s.stop();s._close_normally()
        s.home.plan.assert_called_once()
        s.home.execute.assert_not_called()
        s.home.cancel.assert_called_once()
        self.assertTrue(s.snapshot()['active'])
        self.assertFalse(s.stop_event.is_set())
        self.assertTrue(s.snapshot()['close_failed'])

    def test_disarm_only_configuration_cannot_enable_or_send_motion(self):
        root=ET.fromstring(control_description(load_model('teaching_right'),False,True,disarm_only=True))
        hp={p.get('name'):p.text for p in root.findall('ros2_control/hardware/param')}
        self.assertEqual(hp['auto_enable'],'false')
        self.assertEqual(hp['set_control_mode_on_configure'],'false')
        self.assertEqual(hp['disable_on_deactivate'],'true')
        for i,j in enumerate(root.findall('ros2_control/joint')):
            self.assertEqual(j.find("param[@name='read_only']").text,'true' if i==6 else 'false')

    def test_ui_construction_and_all_arm_selection_no_transport(self):
        from PyQt5.QtWidgets import QApplication
        from gravity_view import GravityWindow
        app=QApplication.instance() or QApplication([])
        with patch('gravity_session.GravityStack.start',side_effect=AssertionError('must not connect')):
            w=GravityWindow()
            self.assertFalse(w.enable_button.isEnabled())
            for role in ROLE_ORDER:
                w.buttons[role].click()
                self.assertEqual(w.role,role)
                actual,expected=w.parameters(),load_profile(role)
                for key in VECTOR_KEYS:
                    self.assertTrue(np.allclose(actual[key],expected[key],atol=.0005,rtol=0.))
                self.assertEqual({k:v for k,v in actual.items() if k not in VECTOR_KEYS},
                                 {k:v for k,v in expected.items() if k not in VECTOR_KEYS})
            w.reset.click()
            self.assertEqual(w.parameters(),base_parameters())
            self.assertIn('六轴参数一致',w.parameter_summary.text())
            w.buttons[role].click()
            self.assertEqual(w.parameters(),base_parameters())
            other=next(r for r in ROLE_ORDER if r!=role)
            w.buttons[other].click();w.buttons[role].click()
            self.assertEqual(w.parameters(),base_parameters())
            w.close()
        app.processEvents()

    def test_enable_cannot_queue_twice(self):
        s=GravitySession(); s.update(ready=True)
        s.enable()
        with self.assertRaises(ValueError): s.enable()
        self.assertEqual(s.commands.qsize(),1)

    def test_live_tuning_keeps_direction_and_limits_and_rejects_duplicate_queue(self):
        s=GravitySession(); p=base_parameters()
        s.update(ready=True,applied_parameters=p)
        changed=base_parameters(); changed['effort_signs'][2]=-1.
        with self.assertRaises(ValueError): s.tune(changed)
        self.assertFalse(s.snapshot()['busy'])
        changed=base_parameters(); changed['torque_limits'][2]=19.
        with self.assertRaises(ValueError): s.tune(changed)
        changed=base_parameters(); changed['torque_scales'][2]=1.1
        s.tune(changed)
        self.assertTrue(s.snapshot()['tuning_pending'])
        with self.assertRaises(ValueError): s.tune(changed)
        self.assertEqual(s.commands.qsize(),1)
        self.assertEqual(s.snapshot()['applied_parameters'],p)

    def test_stale_diagnostics_block_start(self):
        from native_stack import NativeFeedback
        s=GravitySession(); s.feedback=NativeFeedback({},False)
        s.update(mock=False)
        now=time.monotonic()
        s.feedback.update(q=[0.]*7,sample_time=now,diagnostic_time=now-1.)
        self.assertFalse(s._fresh())


if __name__ == '__main__': unittest.main()
