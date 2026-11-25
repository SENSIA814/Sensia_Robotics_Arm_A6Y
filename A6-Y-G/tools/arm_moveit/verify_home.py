#!/usr/bin/env python3
"""MoveIt home/exit regression with mock motors; physical transport is forbidden."""
import json
import time
from unittest.mock import patch

import numpy as np

from motion_model import Kinematics
from planning_session import PlanningSession


def wait(session, predicate, timeout=35, allow_error=False):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        s = session.snapshot()
        if s['hardware']['error'] or (s['error'] and not allow_error):
            raise AssertionError(s['error'] or s['hardware']['error'])
        if predicate(s):
            return s
        time.sleep(.03)
    raise AssertionError('Home test timed out: ' + json.dumps(session.snapshot(), ensure_ascii=False))


def main():
    s = PlanningSession()
    initial = [.04, .12, .12, .01, 0., 0., -.008]
    with patch('hardware.MOCK_Q', initial), patch('hardware.MotionControl.open', side_effect=AssertionError('physical transport forbidden')):
        s.start('motion_left', mock=True, with_rviz=False)
        try:
            wait(s, lambda v: v['ready'] and v['target_valid'])
            assert not s.snapshot()['position_only']
            # A blocked path must keep the session open and never enable motors.
            from moveit_msgs.msg import CollisionObject
            from moveit_msgs.srv import ApplyPlanningScene
            from shape_msgs.msg import SolidPrimitive
            from geometry_msgs.msg import Pose
            client = s.node.create_client(ApplyPlanningScene, 'apply_planning_scene')
            assert client.wait_for_service(timeout_sec=5)
            obstacle = CollisionObject()
            obstacle.id, obstacle.header.frame_id = 'home_blocker', 'base_link'
            box, pose = SolidPrimitive(), Pose()
            box.type, box.dimensions, pose.orientation.w = SolidPrimitive.BOX, [3., 3., 3.], 1.
            obstacle.primitives, obstacle.primitive_poses = [box], [pose]
            for operation in (CollisionObject.ADD, CollisionObject.REMOVE):
                obstacle.operation = operation
                request = ApplyPlanningScene.Request()
                request.scene.is_diff = True
                request.scene.world.collision_objects = [obstacle]
                future = client.call_async(request)
                wait(s, lambda _: future.done(), allow_error=True)
                assert future.result().success
                if operation == CollisionObject.ADD:
                    s.stop()
                    blocked = wait(s, lambda v: v['close_failed'] and not v['home_active'], allow_error=True)
                    assert blocked['running'] and not blocked['hardware']['enabled']
                    assert blocked['hardware']['completed'] == 0 and not s.cancel.is_set()
            # Stop must cancel a pending normal exit, including a planned home.
            s.stop()
            wait(s, lambda v: v['home_stage'] == 'preview', allow_error=True)
            s.send('stop')
            wait(s, lambda v: not v['home_active'] and v['close_failed'], allow_error=True)
            time.sleep(.3)
            assert s.snapshot()['running'] and not s.cancel.is_set()
            assert not s.snapshot()['hardware']['executing']
            # Manual home uses exact joint zeros, independent of the TCP IK branch.
            s.send('home')
            wait(s, lambda v: v['home_active'], allow_error=True)
            result = wait(s, lambda v: v['home_stage'] == 'complete')
            assert np.max(np.abs(s.runtime.measured()[:6])) <= .005
            assert abs(s.runtime.measured()[6] - initial[6]) < 1e-9
            assert result['hardware']['enabled'] and result['hardware']['completed'] == 1
            # Move away again; closing must execute another planned home, then
            # confirm release before the ROS processes and serial owner terminate.
            target = Kinematics(s.model).fk(initial)
            s.send('target', target)
            s.send('plan')
            wait(s, lambda v: v['plan_ready'])
            s.send('execute_arm')
            wait(s, lambda v: v['hardware']['completed'] == 2)
            s.stop()
            # Repeated window-close events must not restart/cancel the home.
            wait(s, lambda v: v['home_active'])
            s.stop()
            wait(s, lambda v: not v['running'])
            s.worker.join(10)
            result = s.snapshot()
            assert result['close_allowed'] and not result['close_failed']
            assert result['hardware']['completed'] == 3 and not result['hardware']['enabled']
            assert np.max(np.abs(result['home_result']['joint_positions'][:6])) <= .005
            assert result['home_result']['joint_positions'][6] == initial[6]
            assert result['hardware']['tx_count'] == 0
            print(json.dumps({'ok': True, 'manual_home': True, 'stop_cancels_exit': True,
                              'blocked_path_prevents_exit_and_enabling': True,
                              'repeated_close_does_not_restart_home': True, 'zero_before_disable_and_exit': True,
                              'gripper_held': True, 'physical_can_frames': 0, 'report': result['report']}, ensure_ascii=False, indent=2))
        finally:
            s.abort()
            s.worker.join(10)


if __name__ == '__main__':
    main()
