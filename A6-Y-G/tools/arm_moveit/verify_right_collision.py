#!/usr/bin/env python3
"""Replay the right-arm report with C++ dry_run, planning only; no serial I/O."""
import json
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

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
    source = ROOT.parent/'diagnostics/arm_moveit_native_reports/motion_right_9tzfeso0/events.jsonl'
    failures = [e for line in source.read_text().splitlines()
                if (e := json.loads(line)).get('kind') == 'plan_result' and e['moveit_error_code'] == -2]
    assert len(failures) == 2
    initial = failures[-1]['start']
    native_model.MOCK_Q = initial
    original_model = native_model.native_model
    def legacy_model(*args, **kwargs):
        model = original_model(*args, **kwargs)
        semantic = ET.fromstring(model['semantic'])
        for element in list(semantic.findall('disable_collisions')):
            if {element.get('link1'), element.get('link2')} == {'link2', 'link4'}:
                semantic.remove(element)
        model['semantic'] = ET.tostring(semantic, encoding='unicode')
        model['collision_policy']['disabled_pairs'] = [['link1', 'link4'], ['link1', 'link5']]
        model['collision_policy']['role_extra_pairs'] = []
        model['collision_policy']['description'] = '验证旧裸机规则：仍检查 link2↔link4'
        return model
    results = {}
    rclpy.init()
    node = rclpy.create_node('right_collision_replay')
    try:
        for variant in ('previous_bare', 'right_bare'):
            s = PlanningSession()
            with patch('native_session.native_model', legacy_model if variant == 'previous_bare' else original_model):
                s.start('motion_right', mock=True, with_rviz=False, collision_profile='bare')
            def call(kind, name, request):
                client = node.create_client(kind, s.namespace+'/'+name)
                try:
                    assert client.wait_for_service(timeout_sec=10)
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
                    assert future.done()
                    return future.result()
                finally:
                    node.destroy_client(client)
            def valid(q):
                return call(GetStateValidity, 'check_state_validity', GetStateValidity.Request(
                    group_name='arm', robot_state=RobotState(is_diff=True,
                    joint_state=JointState(name=list(JOINTS), position=list(q)))))
            try:
                ready = wait(s, lambda v:v['ready'])
                folder = Path(ready['directory'])
                check = valid(s.runtime.measured())
                contacts = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in check.contacts})
                results[variant] = dict(session=str(folder), start_contacts=contacts, start_valid=check.valid)
                if variant == 'previous_bare':
                    assert ('link2', 'link4') in contacts, contacts
                else:
                    assert not contacts, contacts
                for index, failure in enumerate(failures):
                    if index and variant == 'previous_bare':
                        break
                    target = s.kin.fk(np.asarray(failure['target']))
                    s.send('target', target)
                    wait(s, lambda v:v['target_valid'] and v.get('target') == target.tolist() and
                         s.preview_revision == s.revision, allow_error=True)
                    s.send('plan', {'velocity_scaling': failure['velocity_scaling'],
                                    'acceleration_scaling': failure['acceleration_scaling']})
                    if variant == 'previous_bare':
                        failed = wait(s, lambda v:bool(v['error']) and not v['busy'], allow_error=True)
                        assert 'link2 ↔ link4' in failed['error'] and '（-2）' in failed['error']
                        results[variant]['error'] = failed['error']
                    else:
                        planned = wait(s, lambda v:v['plan_ready'])
                        assert not planned['hardware']['enabled']
                        results[variant][f'target_{index+1}'] = dict(planned=True, duration_s=planned['duration'],
                            points=len(s.plan.joint_trajectory.points))
                if variant == 'right_bare':
                    box = CollisionObject(id='right_environment_guard')
                    box.header.frame_id = 'base_link'
                    box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[5.,5.,5.])]
                    box.primitive_poses = [Pose()]
                    box.primitive_poses[0].orientation.w = 1.
                    box.operation = CollisionObject.ADD
                    request = ApplyPlanningScene.Request()
                    request.scene.is_diff = True
                    request.scene.world.collision_objects = [box]
                    assert call(ApplyPlanningScene, 'apply_planning_scene', request).success
                    blocked = valid(s.runtime.measured())
                    assert not blocked.valid
                    assert any('right_environment_guard' in (c.contact_body_1, c.contact_body_2) for c in blocked.contacts)
                    s.send('plan')
                    failed = wait(s, lambda v:bool(v['error']) and not v['busy'], allow_error=True)
                    assert not failed['plan_ready']
                    results[variant]['environment_still_blocks'] = True
                s.stop()
                s.worker.join(15)
                assert not s.worker.is_alive()
                final = s.snapshot()
                assert final['hardware']['completed'] == 0 and not final['hardware']['enabled']
                assert final['home_stage'] == 'not_required'
                assert 'Opened SLCAN bus' not in (folder/'controller_manager.log').read_text()
                results[variant]['readonly_close'] = True
                print(variant, json.dumps(results[variant], ensure_ascii=False), flush=True)
            finally:
                s.abort()
                s.worker.join(15)
        result = dict(result='PASS', physical_devices=0, source_events=str(source), initial=initial, variants=results)
        path = ROOT.parent/'diagnostics/arm_moveit_right_collision_20261125.json'
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        print(path, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
