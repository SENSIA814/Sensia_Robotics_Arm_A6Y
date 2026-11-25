#!/usr/bin/env python3
"""Real MoveIt services, Pinocchio target IK and simulated execution only."""
import json
import time
from unittest.mock import patch

import numpy as np

from motion_model import Kinematics, JOINTS, planning_model
from planning_session import PlanningSession


def wait(session, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = session.snapshot()
        if state['error']:
            raise AssertionError(state['error'] + '\nLogs: ' + state.get('directory', ''))
        if predicate(state):
            return state
        time.sleep(.05)
    raise AssertionError('timed out: ' + str(session.snapshot()))


def main():
    session = PlanningSession()
    with patch('hardware.MotionControl.open', side_effect=AssertionError('physical hardware forbidden')):
        session.start('motion_left', mock=True, with_rviz=False)
        try:
            state = wait(session, lambda s: s['ready'], 35)
            from moveit_msgs.srv import GetStateValidity
            zero = GetStateValidity.Request()
            zero.group_name = 'arm'
            zero.robot_state.joint_state.name = list(JOINTS)
            zero.robot_state.joint_state.position = [0.] * 7
            zero_future = session.valid_client.call_async(zero)
            wait(session, lambda _: zero_future.done(), 5)
            zero_result = zero_future.result()
            zero_contacts = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in zero_result.contacts})
            print('URDF zero collision check:', zero_result.valid, zero_contacts, flush=True)
            q = np.array(state['hardware']['q'])
            # Prove world geometry actually participates in collision checking.
            from moveit_msgs.msg import PlanningScene, CollisionObject
            from shape_msgs.msg import SolidPrimitive
            from geometry_msgs.msg import Pose
            scene_pub = session.node.create_publisher(PlanningScene, 'planning_scene', 1)
            obstacle = CollisionObject()
            obstacle.id, obstacle.header.frame_id = 'verification_obstacle', 'base_link'
            box = SolidPrimitive()
            box.type, box.dimensions = SolidPrimitive.BOX, [3., 3., 3.]
            pose = Pose()
            pose.orientation.w = 1.
            obstacle.primitives, obstacle.primitive_poses = [box], [pose]
            for operation, expected_valid in [(CollisionObject.ADD, False), (CollisionObject.REMOVE, True)]:
                obstacle.operation = operation
                scene = PlanningScene()
                scene.is_diff = True
                scene.world.collision_objects = [obstacle]
                until = time.monotonic() + 5
                while True:
                    scene_pub.publish(scene)
                    request = GetStateValidity.Request()
                    request.group_name = 'arm'
                    request.robot_state.joint_state.name = list(JOINTS)
                    request.robot_state.joint_state.position = q.tolist()
                    check = session.valid_client.call_async(request)
                    wait(session, lambda _: check.done(), 3)
                    if check.result().valid == expected_valid:
                        break
                    if time.monotonic() > until:
                        raise AssertionError('planning scene collision update was not observed')
                    time.sleep(.05)
            session.node.destroy_publisher(scene_pub)
            target_q = q.copy()
            target_q[:3] += [.06, .04, .03]
            target = Kinematics(planning_model('motion_left')).fk(target_q)
            session.send('target', target)
            state = wait(session, lambda s: s['target_valid'] and s.get('target') == target.tolist(), 8)
            preview = np.array(state['preview_q'])
            assert np.linalg.norm(Kinematics(session.model).fk(preview)[:3, 3] - target[:3, 3]) < .0005
            assert not state['hardware']['enabled']
            invalid = target.copy()
            invalid[0, 3] = 3.
            session.send('target', invalid)
            wait(session, lambda s: not s['target_valid'] and '工作空间' in s['preview_status'], 8)
            assert np.allclose(session.snapshot()['preview_q'], preview)
            session.send('target', target)
            session.send('plan', {'velocity_scaling': .6, 'acceleration_scaling': .2})
            state = wait(session, lambda s: s['plan_ready'], 25)
            assert session.plan.velocity_scaling == .6 and session.plan.acceleration_scaling == .2
            assert np.max(session.plan.peak_velocity) <= .25 * .6
            assert np.max(session.plan.peak_acceleration) <= .5 * .2
            print('Plan preview ready:', state['duration'], flush=True)
            session.send('execute')
            state = wait(session, lambda s: s['hardware']['completed'] == 1, 25)
            pose = Kinematics(planning_model('motion_left')).fk(state['hardware']['q'])
            assert np.linalg.norm(pose[:3, 3] - target[:3, 3]) < .001
            assert state['hardware']['enabled'] is True
            # Exercise the same ROS command/status interface as the RViz panel.
            from std_msgs.msg import String
            from visualization_msgs.srv import GetInteractiveMarkers
            marker_client = session.node.create_client(GetInteractiveMarkers, 'tcp_target/get_interactive_markers')
            assert marker_client.wait_for_service(timeout_sec=3)
            markers = marker_client.call_async(GetInteractiveMarkers.Request())
            wait(session, lambda _: markers.done(), 5)
            goal_marker = next(m for m in markers.result().markers if m.name == 'tcp_goal')
            assert len(goal_marker.controls) == 7  # XYZ arrows and Roll/Pitch/Yaw rings by default
            assert goal_marker.controls[0].always_visible
            assert goal_marker.controls[0].interaction_mode == 7  # MOVE_3D
            ui = session.node.create_publisher(String, 'ui/command', 10)
            wait(session, lambda _: ui.get_subscription_count() > 0, 5)
            before = session.runtime.measured()
            percent = float((-before[6] + .001) / .035 * 100)
            ui.publish(String(data=json.dumps({'command': 'gripper_target', 'value': percent})))
            wait(session, lambda s: abs(s.get('gripper_percent', -100) - percent) < .01, 3)
            assert np.allclose(session.runtime.measured(), before)
            ui.publish(String(data=json.dumps({'command': 'gripper_plan', 'value': .5})))
            state = wait(session, lambda s: s['plan_ready'], 25)
            assert state['plan_group'] == 'gripper'
            ui.publish(String(data=json.dumps({'command': 'execute_gripper'})))
            state = wait(session, lambda s: s['hardware']['completed'] == 2, 25)
            after = session.runtime.measured()
            assert np.allclose(after[:6], before[:6])
            assert abs(after[6] - (before[6] - .001)) < .00005
            session.send('emergency_release')
            wait(session, lambda s: not s['hardware']['enabled'], 3)
        finally:
            session.abort()  # Test teardown, not a user-requested normal exit.
            session.worker.join(10)
        assert not session.worker.is_alive()
        state = session.snapshot()
        assert not state['error'], state['error']
        assert state['hardware']['tx_count'] == 0
        print(json.dumps({'ok': True, 'planning': 'MoveIt OMPL', 'kinematics': 'Pinocchio',
                          'mock_execution_completed': state['hardware']['completed'],
                          'hardware_connected': False, 'can_frames': 0, 'rviz_render_verified': False,
                          'urdf_zero_collision_free': zero_result.valid, 'zero_contacts': zero_contacts,
                          'world_obstacle_collision_verified': True,
                          'rviz_command_topic_verified': True, 'tcp_marker_service_verified': True,
                          'gripper_plan_and_execution_verified': True,
                          'live_bounded_preview_verified': True, 'unreachable_preview_holds_last_valid_model': True,
                          'independent_velocity_acceleration_verified': True,
                          'report': state['report']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
