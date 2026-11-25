#!/usr/bin/env python3
"""Original C++ SlcanBus/SystemInterface on a PTY; never a physical device."""
import errno
import argparse
import json
import os
from pathlib import Path
import pty
import select
import struct
import threading
import time

import numpy as np

from motion_model import JOINTS, MOCK_Q, ROLE_ORDER, load_roles
from native_model import native_model
from native_session import PlanningSession
from verify_native import wait


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--role', choices=ROLE_ORDER, default='motion_left')
    parser.add_argument('--readonly-close-test', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    args = parser.parse_args()
    role = args.role
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    assert port.startswith('/dev/pts/')
    os.close(slave)
    roles = load_roles()
    roles[role]['port'] = port
    model = native_model(role, roles)
    q = np.array(MOCK_Q)
    enabled, frames = set(), []
    stopped = threading.Event()

    def emulate():
        buffer = b''
        def reply(i, data):
            try:
                os.write(master, f't{i+16:03X}8{data.hex()}\r'.encode())
            except OSError:
                pass
        def feedback(i):
            e = model['joints'][JOINTS[i-1]]
            pmax = e['position_scale_evidence']['pmax_rad']
            motor = e['sign'] * (q[i-1] - e['offset']) / e['position_scale']
            raw = round((motor + pmax) / (2*pmax) * 65535)
            # Quantized zero velocity/effort, as on the physical protocol.
            reply(i, bytes([(16 if i in enabled else 0) | i, raw >> 8, raw & 255, 0x80, 0x08, 0x00, 25, 25]))
        while not stopped.is_set():
            if not select.select([master], [], [], .01)[0]:
                continue
            try:
                buffer += os.read(master, 8192)
            except OSError as exc:
                if exc.errno == errno.EIO:
                    stopped.wait(.01)
                    continue
                raise
            while b'\r' in buffer:
                line, buffer = buffer.split(b'\r', 1)
                if len(line) != 21 or not line.startswith(b't'):
                    continue
                cid, data = int(line[1:4], 16), bytes.fromhex(line[5:].decode())
                frames.append((cid, data))
                if cid == 0x7ff and data[2] == 0xcc:
                    feedback(data[0])
                elif cid == 0x7ff and data[2:4] == b'\x55\x0a':
                    reply(data[0], data)
                elif cid in range(1, 8) and data == b'\xff'*7+b'\xfc':
                    enabled.add(cid)
                    feedback(cid)
                elif cid in range(1, 8) and data == b'\xff'*7+b'\xfd':
                    enabled.discard(cid)
                    feedback(cid)
                elif cid in range(0x101, 0x108):
                    i = cid - 0x100
                    p, velocity = struct.unpack('<ff', data)
                    if i in enabled:
                        e = model['joints'][JOINTS[i-1]]
                        q[i-1] = e['sign'] * e['position_scale'] * p + e['offset']
                    feedback(i)

    thread = threading.Thread(target=emulate, daemon=True)
    thread.start()
    s = PlanningSession(roles=roles)
    s.start(role, mock=False, with_rviz=args.rviz)
    try:
        state = wait(s, lambda v: v['ready'] and v['target_valid'])
        assert not enabled
        assert frames and all(cid == 0x7ff and data[2] == 0xcc for cid, data in frames)
        s.send('permit', True)
        if args.readonly_close_test:
            # The exact reported branch: permission checked, native planning
            # fails, no Execute click, close with joints away from home.
            import rclpy
            from moveit_msgs.msg import CollisionObject
            from moveit_msgs.srv import ApplyPlanningScene
            from shape_msgs.msg import SolidPrimitive
            from geometry_msgs.msg import Pose
            rclpy.init()
            node = rclpy.create_node('readonly_exit_verifier')
            try:
                client = node.create_client(ApplyPlanningScene, s.namespace+'/apply_planning_scene')
                assert client.wait_for_service(timeout_sec=10)
                box = CollisionObject(id='blocked_readonly_exit_test')
                box.header.frame_id = 'base_link'
                box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[5., 5., 5.])]
                box.primitive_poses = [Pose()]
                box.primitive_poses[0].orientation.w = 1.
                box.operation = CollisionObject.ADD
                req = ApplyPlanningScene.Request()
                req.scene.is_diff = True
                req.scene.world.collision_objects = [box]
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=15)
                assert future.done() and future.result().success
                s.send('plan')
                failed = wait(s, lambda v: bool(v['error']) and not v['busy'], allow_error=True)
                assert failed['permitted'] and failed['hardware']['completed'] == 0
                assert not failed['hardware']['enabled'] and not failed['hardware']['executing']
                before = q.copy()
                started = time.monotonic()
                s.stop()
                wait(s, lambda v: v['close_allowed'], timeout=8, allow_error=True)
                s.worker.join(10)
                elapsed = time.monotonic()-started
                assert not s.worker.is_alive() and elapsed < 10
                assert all(p.poll() is not None for _, p in s.display.children)
                assert not enabled and np.array_equal(q, before)
                assert frames and all(cid == 0x7ff and data[2] == 0xcc for cid, data in frames)
                assert not (Path(state['directory'])/'controllers_active.yaml').exists()
                assert s.snapshot()['home_result'] is None and s.snapshot()['home_stage'] == 'not_required'
                result = dict(result='PASS', role=role, physical_devices=0, transport=port,
                              permitted_planning_failure_closed=True, planning_error=failed['error'],
                              cpp_slcan_frames=len(frames), all_frames_feedback_queries=True,
                              unchanged_joints=True, no_home_or_enable=True, rviz=args.rviz,
                              all_children_exited=True, close_seconds=elapsed, report=s.snapshot()['report'])
                Path(state['directory'], 'readonly_exit_verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
                print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
            finally:
                node.destroy_node()
                rclpy.shutdown()
            return
        target = s.runtime.measured()
        target[:3] += [.03, .025, .02]
        s.send('target', s.kin.fk(target))
        s.send('plan')
        wait(s, lambda v: v['plan_ready'])
        assert not enabled
        s.send('execute_arm')
        wait(s, lambda v: v['hardware']['completed'] == 1)
        assert enabled == set(range(1, 8))
        s.send('gripper_target', 20.)
        s.send('gripper_plan')
        wait(s, lambda v: v['plan_ready'] and v['plan_group'] == 'gripper')
        s.send('execute_gripper')
        wait(s, lambda v: v['hardware']['completed'] == 2)
        s.stop()
        wait(s, lambda v: v['close_allowed'], timeout=90)
        s.worker.join(15)
        assert not s.worker.is_alive() and not enabled
        assert not any(data == b'\xff'*7+b'\xfe' for _, data in frames)
        assert all(data[2] in (0xcc, 0x55) for cid, data in frames if cid == 0x7ff)
        result = {'result': 'PASS', 'role': role, 'rmw': s.snapshot().get('rmw'), 'transport': port, 'physical_devices': 0,
                  'cpp_slcan_frames': len(frames), 'readonly_before_execute': True,
                  'arm_and_gripper_native_execution': True, 'home_and_disable_confirmed': True,
                  'report': s.snapshot()['report']}
        Path(state['directory'], 'serial_verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    finally:
        s.abort()
        s.worker.join(15)
        stopped.set()
        thread.join(2)
        os.close(master)


if __name__ == '__main__':
    main()
