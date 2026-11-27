from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import tempfile
import threading
import unittest
import xml.etree.ElementTree as ET
import yaml

from gravity_config import ROOT, CONTROLLER, JOINTS, load_model, load_profile
from teaching_port_config import HOME, FOLLOW, write_port_control, write_mapper_config
from a6_y_teleop.leader_follower_teleop import load_config, LeaderFollowerTeleop
from teaching_port import TeachingCoordinator
from teaching_port_session import PortArmSession, PortStack
from teaching_port_config import port_parameters, position_bounds
from gravity_session import GravitySession


class ArchivePortTests(unittest.TestCase):
    def test_recorded_negative_j2_is_not_a_zero_pose_requirement(self):
        arm=PortArmSession();arm.model=load_model('motion_right')
        q=[-.091363394,-.011635004,.00171664,-.353437095,.061226825,.031090257,-.000114568]
        original=list(q)
        arm._validate_enable_positions(q)
        self.assertEqual(q,original)
        # The change is scoped to power teaching, not standalone gravity/MoveIt.
        standalone=GravitySession();standalone.model=arm.model
        with self.assertRaisesRegex(ValueError,'模型范围'):standalone._validate_enable_positions(q)
        for bad in (float('nan'),float('inf'),12.6):
            q[1]=bad
            with self.assertRaisesRegex(ValueError,'joint2'):arm._validate_enable_positions(q)
        with self.assertRaises(ValueError):arm._validate_enable_positions([0.]*6)

    def test_protocol_position_bounds_follow_encoder_scale_and_offset(self):
        model=deepcopy(load_model('motion_right'))
        model['joints']['joint2'].update(sign=-1.,position_scale=.5,offset=.2)
        bounds=position_bounds(model)
        self.assertAlmostEqual(bounds['joint2'][0],-6.05)
        self.assertAlmostEqual(bounds['joint2'][1],6.45)

    def test_native_and_mapper_keep_negative_and_zero_teacher_targets(self):
        with tempfile.TemporaryDirectory() as temp:
            follower=SimpleNamespace(namespace='/follower',model=load_model('motion_right'))
            path,_=write_mapper_config(temp,SimpleNamespace(namespace='/teacher',model=load_model('teaching_left')),follower)
            config=load_config(str(path))
            # Exercise the actual mapper and the same protocol position bounds.
            mapper=LeaderFollowerTeleop.__new__(LeaderFollowerTeleop)
            mapper.config=config;mapper.last_safe=None;mapper.last_command_time=None
            for j2 in (-.002479591,0.,.000572213):
                teacher=[-.077630274,j2,.00171664,.001335164,.001335164,.060463874,-.01]
                self.assertEqual(mapper.apply_safety(teacher,teacher),teacher)
            path=write_port_control(follower.model,port_parameters('motion_right'),temp,'/follower','cm',False,True)
            data=yaml.safe_load(path.read_text())
            robot=ET.fromstring(data['/follower/cm']['ros__parameters']['robot_description'])
            for i,joint in enumerate(robot.findall('ros2_control/joint')[:6]):
                self.assertEqual(float(joint.find("param[@name='min_position']").text),config.min_position[i])
                self.assertEqual(float(joint.find("param[@name='max_position']").text),config.max_position[i])
                self.assertEqual(config.limit_margin[i],0.)

    def test_all_poses_checked_before_any_arm_enable(self):
        c=TeachingCoordinator();c._event=Mock()
        left=SimpleNamespace(enable_block_reason=lambda:'',feedback=SimpleNamespace(snapshot=lambda:{'q':[.1]*7}),
            _validate_enable_positions=Mock())
        right=SimpleNamespace(enable_block_reason=lambda:'',feedback=SimpleNamespace(snapshot=lambda:{'q':[float('nan')]*7}),
            _validate_enable_positions=Mock(side_effect=ValueError('joint2 无效')))
        c.sessions={'motion_left':left,'motion_right':right}
        with self.assertRaisesRegex(ValueError,'joint2'):c._preflight_start()
        left._validate_enable_positions.assert_called_once()
        self.assertNotIn('startup_positions_checked',[x.args[0] for x in c._event.call_args_list])

    def test_home_exit_between_polls_is_not_a_persistent_process_failure(self):
        stack=PortStack.__new__(PortStack)
        stack.directory=Path('/tmp')
        home=SimpleNamespace(poll=Mock(side_effect=[None,0]))
        stack.children=[('native_home',home)]
        stack.check()
        home.poll.assert_not_called()  # NativeHome owns this child's result.
        stack.children.append(('controller_manager',SimpleNamespace(poll=lambda:1)))
        with self.assertRaisesRegex(RuntimeError,'controller_manager'):
            stack.check()

    def test_fixed_target_rate_rejected_before_starting_hardware(self):
        c=TeachingCoordinator()
        with self.assertRaisesRegex(ValueError,'100 Hz'):
            c.start(frequency=50.)
        self.assertEqual(c.sessions,{})

    def test_motion_arm_rejects_gravity_before_any_ros_call(self):
        arm=PortArmSession();arm.model={'role':'motion_left'}
        with self.assertRaisesRegex(ValueError,'禁止开启重力补偿'):arm._select(CONTROLLER)
        with self.assertRaises(ValueError):arm.submit('gravity')
        with self.assertRaises(ValueError):arm.submit('home')
        self.assertIsNone(arm._create_math())
        with patch('teaching_port_config.load_profile',side_effect=AssertionError('motion profile read')):
            self.assertIsInstance(port_parameters('motion_left'),dict)

    def test_exit_requests_all_arms_before_waiting_and_has_group_timeout(self):
        c=TeachingCoordinator();c.close_timeout=0.;c._hold_sessions=lambda:None
        requested=[]
        class Arm:
            def __init__(self,role):self.role=role;self.cancel_home=threading.Event()
            def snapshot(self):return dict(role=self.role,running=True,close_failed=False)
            def stop(self):requested.append(self.role)
        c.sessions={r:Arm(r) for r in ('teaching_left','motion_left')}
        c._close_sessions()
        self.assertEqual(requested,list(c.sessions))
        self.assertTrue(all(s.cancel_home.is_set() for s in c.sessions.values()))
        self.assertTrue(c.snapshot()['close_failed'])
        self.assertFalse(c.snapshot()['running'])

    def test_close_after_release_visible_before_worker_finishes_is_not_lost(self):
        c=TeachingCoordinator()
        finished=[];allow_worker_exit=threading.Event()
        c.alive=lambda:not finished
        c._retry_close=lambda:finished.append(True)
        c.update(phase='released',running=False)
        c.emergency.set()
        old=threading.Thread(target=lambda:allow_worker_exit.wait(2.))
        c.worker=old;old.start()
        try:
            c.stop()
            allow_worker_exit.set()
            c.worker.join(2.)
            self.assertFalse(c.worker.is_alive())
            self.assertEqual(finished,[True])
        finally:
            allow_worker_exit.set();old.join(2.)

    def test_actual_upstream_loader_accepts_seven_axis_cross_pair_config(self):
        with tempfile.TemporaryDirectory() as temp:
            leader=SimpleNamespace(namespace='/left_teacher',model=load_model('teaching_left'))
            follower=SimpleNamespace(namespace='/right_motion',model=load_model('motion_right'))
            path,_=write_mapper_config(temp,leader,follower,100.)
            config=load_config(str(path))
            self.assertEqual(config.joint_names,list(JOINTS))
            self.assertEqual(config.leader_joint_state_topic,'/left_teacher/joint_states')
            self.assertEqual(config.command_topic,'/right_motion/direct_position_controller/commands')
            self.assertEqual(config.max_velocity,[7.5]*3+[5.]*3+[.100])
            self.assertEqual(config.max_joint_step,.35)
            self.assertEqual(config.step_reference,'previous_command')
            self.assertEqual(config.soft_start_duration,1.2)
            self.assertFalse(config.prediction_enabled)
            self.assertEqual(config.control_hz,100.)
            self.assertEqual(config.feedback_timeout,.5)

    def test_native_home_and_follow_cannot_use_the_old_torque_planner(self):
        with tempfile.TemporaryDirectory() as temp:
            for role in ('teaching_left','motion_left','teaching_right','motion_right'):
                path=write_port_control(load_model(role),load_profile(role),temp,'/test','cm',False,True)
                data=yaml.safe_load(path.read_text())
                self.assertEqual(data['/test/cm']['ros__parameters']['update_rate'],250)
                teacher=role.startswith('teaching_')
                if teacher:
                    gravity=data['/test/'+CONTROLLER]['ros__parameters']
                    for flag in ('planned_home_enabled','teaching_follow_enabled','teaching_position_output','home_mock_follow'):
                        self.assertFalse(gravity[flag])
                else:
                    self.assertNotIn('/test/'+CONTROLLER,data)
                    self.assertNotIn(CONTROLLER,data['/test/cm']['ros__parameters'])
                self.assertEqual(data['/test/'+HOME]['ros__parameters']['command_interfaces'],['position'])
                self.assertEqual(data['/test/'+FOLLOW]['ros__parameters']['joints'],list(JOINTS))
                robot=ET.fromstring(data['/test/cm']['ros__parameters']['robot_description'])
                hardware=robot.find('ros2_control/hardware')
                self.assertEqual(hardware.find("param[@name='can_bitrate']").text,'1000000')
                self.assertEqual(hardware.find("param[@name='diagnostic_update_rate']").text,'250')
                self.assertEqual(hardware.find("param[@name='control_mode']").text,'mit' if teacher else 'pos_vel')
                self.assertEqual(hardware.find("param[@name='defer_enable_until_controller_ready']").text,'true')
                self.assertEqual(robot.findall('ros2_control/joint')[-1].find("param[@name='read_only']").text,'false')
                self.assertEqual(float(robot.findall('ros2_control/joint')[-1].find("param[@name='max_velocity']").text),.100)


if __name__=='__main__':unittest.main()
