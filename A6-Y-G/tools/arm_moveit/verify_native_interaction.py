#!/usr/bin/env python3
"""Native MoveIt interaction/failure regressions, C++ dry-run only."""
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from visualization_msgs.srv import GetInteractiveMarkers
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

import native_model
from native_session import PlanningSession
from verify_native import wait


def main():
    # Avoid the wrist singularity of J4=J5=J6=0 when testing independent RPY.
    native_model.MOCK_Q = [0., 1.2, 1.2, .1, .12, .05, -.01]
    s = PlanningSession()
    rclpy.init()
    node = rclpy.create_node('native_interaction_verifier')
    s.start('motion_left', mock=True, with_rviz=False)
    def call(kind, name, request):
        client = node.create_client(kind, s.namespace + '/' + name)
        assert client.wait_for_service(timeout_sec=10)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        assert future.done()
        result = future.result()
        node.destroy_client(client)
        return result
    def target(matrix):
        previous = s.revision
        s.send('target', matrix)
        wait(s, lambda v: s.revision > previous and s.preview_revision == s.revision)
    try:
        state = wait(s, lambda v: v['ready'] and v['target_valid'])
        markers = call(GetInteractiveMarkers, 'tcp_target/get_interactive_markers', GetInteractiveMarkers.Request())
        assert len(markers.markers) == 1
        names = {c.name for c in markers.markers[0].controls}
        assert names == {'move_tcp', 'move_x', 'move_y', 'move_z', 'rotate_x', 'rotate_y', 'rotate_z'}
        current = s.kin.fk(s.runtime.measured())
        for axis in range(3):
            goal = current.copy()
            angle = np.zeros(3)
            angle[axis] = .008
            goal[:3, :3] = Rotation.from_rotvec(angle).as_matrix() @ current[:3, :3]
            target(goal)
            assert s.snapshot()['target_valid'], s.snapshot()['preview_status']
            preview = s.kin.fk(s.preview_q)
            assert np.linalg.norm(preview[:3, 3] - goal[:3, 3]) < .0001
            assert np.linalg.norm(Rotation.from_matrix(preview[:3, :3] @ goal[:3, :3].T).as_rotvec()) < .001
        good = s.preview_q.copy()
        bad = current.copy()
        bad[0, 3] = 8.
        target(bad)
        assert not s.snapshot()['target_valid']
        assert np.allclose(s.preview_q, good)
        # A position-only goal ignores an incompatible requested orientation.
        target(current)
        s.send('settings', {'position_only': True})
        wait(s, lambda v: v['position_only'] and v['target_valid'])
        goal = current.copy()
        goal[:3, :3] = Rotation.from_euler('xyz', [2., 1., 2.]).as_matrix()
        target(goal)
        assert s.snapshot()['target_valid'], s.snapshot()['preview_status']
        assert np.linalg.norm(s.kin.fk(s.preview_q)[:3, 3]-goal[:3, 3]) < .001
        s.send('settings', {'position_only': False})
        target(current)
        # Acquire the dry-run hardware with an actual native execution first.
        # Permission alone must never force a readonly session to home on exit.
        s.send('permit', True)
        q = s.runtime.measured()
        q[0] += .03
        target(s.kin.fk(q))
        s.send('plan')
        wait(s, lambda v: v['plan_ready'])
        s.send('execute_arm')
        wait(s, lambda v: v['hardware']['completed'] == 1)
        assert s.snapshot()['hardware']['enabled']
        # A collision blocks owned-motor home/exit; no direct-command fallback.
        box = CollisionObject(id='blocked_home_test')
        box.header.frame_id = 'base_link'
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[5., 5., 5.])]
        box.primitive_poses = [Pose()]
        box.primitive_poses[0].orientation.w = 1.
        box.operation = CollisionObject.ADD
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.world.collision_objects = [box]
        assert call(ApplyPlanningScene, 'apply_planning_scene', req).success
        s.send('permit', True)
        s.stop()
        failed = wait(s, lambda v: v['close_failed'], allow_error=True)
        assert failed['running'] and not failed['close_allowed'] and failed['hardware']['enabled']
        box.operation = CollisionObject.REMOVE
        assert call(ApplyPlanningScene, 'apply_planning_scene', req).success
        s.send('reset')
        wait(s, lambda v: v['target_valid'] and not v['error'], allow_error=True)
        # Stop during the one-second home preview cancels shutdown entirely.
        s.stop()
        wait(s, lambda v: v['home_stage'] == 'preview')
        s.send('stop')
        wait(s, lambda v: not v['home_active'] and v['close_failed'], allow_error=True)
        assert s.snapshot()['hardware']['enabled']
        assert s.snapshot()['running'] and not s.snapshot()['closing_requested']
        # MoveIt rejects a malformed execution start, without ending the UI.
        s.send('reset')
        wait(s, lambda v: v['target_valid'] and not v['error'], allow_error=True)
        q = s.runtime.measured()
        q[0] += .04
        target(s.kin.fk(q))
        s.send('plan')
        wait(s, lambda v: v['plan_ready'])
        s.plan.joint_trajectory.points[0].positions[0] += .5  # Deliberately invalid, mock only.
        s.send('execute_arm')
        failed = wait(s, lambda v: bool(v['error']) and not v['busy'], allow_error=True)
        assert failed['running'] and failed['ready'] and not failed['plan_ready']
        assert failed['hardware']['completed'] == 1
        assert all(p.poll() is None for n, p in s.display.children if not n.startswith('spawner_'))
        s.send('emergency_release')
        wait(s, lambda v: not v['hardware']['enabled'] and not v['permitted'], allow_error=True)
        s.stop()
        wait(s, lambda v: v['close_allowed'], allow_error=True)
        s.worker.join(15)
        assert not s.worker.is_alive()
        result = dict(result='PASS', xyz_rpy_controls=7, fixed_tcp_rpy=True, invalid_target_retains_model=True,
                      position_only=True, collision_blocks_home_and_exit=True, stop_cancels_home_exit=True,
                      native_execution_rejection_keeps_ui=True, physical_devices=0, report=s.snapshot()['report'])
        Path(state['directory'], 'interaction_verification.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    finally:
        s.abort()
        s.worker.join(15)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
