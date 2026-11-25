#!/usr/bin/env python3
"""Replay the reported posture in upstream C++ dry_run; no physical devices."""
import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

import native_model
from motion_model import JOINTS, ROOT
from native_session import PlanningSession
from verify_native import wait


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rviz', action='store_true')
    args = parser.parse_args()
    source = ROOT.parent/'diagnostics/arm_moveit_native_reports/motion_left_bcaiwqdg/live_status.json'
    recorded = json.loads(source.read_text())
    initial = recorded['hardware']['q']
    goal_q = recorded['preview_q']
    native_model.MOCK_Q = initial
    rclpy.init()
    node = rclpy.create_node('bare_collision_verifier')
    results = {}
    try:
        for profile in ('full', 'bare'):
            s = PlanningSession()
            s.start('motion_left', mock=True, with_rviz=args.rviz and profile == 'bare', collision_profile=profile)
            def call(kind, name, req):
                client = node.create_client(kind, s.namespace+'/'+name)
                assert client.wait_for_service(timeout_sec=10)
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=15)
                assert future.done()
                result = future.result()
                node.destroy_client(client)
                return result
            def valid(q):
                req = GetStateValidity.Request(group_name='arm')
                req.robot_state = RobotState(joint_state=JointState(name=list(JOINTS),position=list(q)),is_diff=True)
                return call(GetStateValidity, 'check_state_validity', req)
            try:
                state = wait(s, lambda v:v['ready'])
                check = valid(s.runtime.measured())
                contacts = sorted({tuple(sorted((c.contact_body_1,c.contact_body_2))) for c in check.contacts})
                assert check.valid == (profile == 'bare'), (profile,contacts)
                s.send('target', s.kin.fk(np.array(goal_q)))
                wait(s, lambda v:v['target_valid'] and s.preview_revision == s.revision)
                s.send('plan')
                if profile == 'full':
                    failed = wait(s, lambda v:bool(v['error']) and not v['busy'],allow_error=True)
                    assert not failed['plan_ready'] and not failed['hardware']['enabled']
                    results[profile] = dict(start_valid=False,contacts=contacts,error=failed['error'])
                else:
                    plan = wait(s, lambda v:v['plan_ready'])
                    assert not plan['hardware']['enabled']
                    s.send('execute_arm')
                    wait(s, lambda v:v['hardware']['completed']==1)
                    assert np.max(abs(s.runtime.measured()[:6]-np.array(goal_q[:6]))) < .002
                    s.send('gripper_target',20.)
                    s.send('gripper_plan')
                    wait(s, lambda v:v['plan_ready'] and v['plan_group']=='gripper')
                    s.send('execute_gripper')
                    wait(s, lambda v:v['hardware']['completed']==2)
                    # A real environment obstacle remains invalid with bare mode.
                    box = CollisionObject(id='bare_environment_guard')
                    box.header.frame_id = 'base_link'
                    box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[5.,5.,5.])]
                    box.primitive_poses = [Pose()]
                    box.primitive_poses[0].orientation.w = 1.
                    box.operation = CollisionObject.ADD
                    req = ApplyPlanningScene.Request()
                    req.scene.is_diff = True
                    req.scene.world.collision_objects = [box]
                    assert call(ApplyPlanningScene,'apply_planning_scene',req).success
                    blocked = valid(s.runtime.measured())
                    assert not blocked.valid
                    assert any('bare_environment_guard' in (c.contact_body_1,c.contact_body_2) for c in blocked.contacts)
                    s.send('plan')
                    wait(s,lambda v:bool(v['error']) and not v['busy'],allow_error=True)
                    assert not s.snapshot()['plan_ready'] and s.snapshot()['hardware']['completed']==2
                    box.operation = CollisionObject.REMOVE
                    assert call(ApplyPlanningScene,'apply_planning_scene',req).success
                    s.send('reset')
                    wait(s,lambda v:v['target_valid'] and not v['error'],allow_error=True)
                    results[profile] = dict(start_valid=True,recorded_goal_planned=True,
                                           arm_and_gripper_dry_run_executed=True,environment_collision_still_blocks=True)
                s.stop()
                wait(s,lambda v:v['close_allowed'],timeout=90,allow_error=True)
                s.worker.join(15)
                assert not s.worker.is_alive()
                if profile == 'bare':
                    assert s.snapshot()['home_result'] is not None
                    assert not s.snapshot()['hardware']['enabled']
                    results[profile]['home_and_release_confirmed'] = True
                else:
                    assert s.snapshot()['home_result'] is None
                results[profile]['report'] = s.snapshot()['report']
                assert 'Opened SLCAN bus' not in (Path(state['directory'])/'controller_manager.log').read_text()
            finally:
                s.abort()
                s.worker.join(15)
        result = dict(result='PASS',physical_devices=0,source_snapshot=str(source),rviz=args.rviz,profiles=results)
        path=ROOT.parent/'diagnostics/arm_moveit_bare_collision_20261125.json'
        path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
