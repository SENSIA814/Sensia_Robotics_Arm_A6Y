"""Hardware-free checks of kinematics, trajectories, command gates and lifecycle."""
from contextlib import contextmanager
from copy import deepcopy
import errno
import os
import pty
import select
import struct
import threading
import time
import unittest
from unittest.mock import patch

import can
import numpy as np

from motion_model import ARM_JOINTS, GRIPPER_JOINTS, JOINTS, MOCK_Q, Kinematics, load_roles, planning_model, bounded_start
from trajectory import Trajectory, gripper_profile, gripper_motion, from_ros
from hardware import MotionControl, ArmRuntime, tracking_speeds
from scanner import status_payload


def wait_for(predicate, timeout=5):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('condition timed out')


def small_trajectory(model, q=None, duration=1.):
    initial = list(MOCK_Q[:6] if q is None else q[:6])
    final = initial.copy()
    final[0] += .015
    return Trajectory(list(ARM_JOINTS), [
        {'time': 0., 'positions': initial, 'velocities': [0.]*6},
        {'time': duration, 'positions': final, 'velocities': [0.]*6}], model)


def gripper_trajectory(model, q=None, duration=1.):
    initial = MOCK_Q[6] if q is None else q[6]
    return Trajectory(list(GRIPPER_JOINTS), [
        {'time': 0., 'positions': [initial], 'velocities': [0.]},
        {'time': duration, 'positions': [initial - .001], 'velocities': [0.]}], model)


class ModelTests(unittest.TestCase):
    def test_position_only_from_zero_boundaries_reaches_without_forcing_orientation(self):
        kin = Kinematics(planning_model('motion_left'))
        target = kin.fk([.03, .12, .08, .02, .01, 0., 0.])
        target[:3, :3] = np.diag([-1., -1., 1.])
        solved = kin.ik(target, [0.]*7, position_only=True)
        self.assertLess(np.linalg.norm(kin.fk(solved)[:3, 3] - target[:3, 3]), .0005)
        self.assertTrue(np.all(solved[:6] >= kin.lower))
        self.assertTrue(np.all(solved[:6] <= kin.upper))

    def test_boundary_planning_copy_never_changes_raw_feedback(self):
        raw = np.array([*MOCK_Q[:6], .00038952970121311915])
        bounded, warning = bounded_start(planning_model('teaching_left'), raw)
        self.assertEqual(bounded[6], 0.)
        self.assertAlmostEqual(raw[6], .00038952970121311915)
        self.assertIn('+0.390 mm', warning)
        raw[6] = .0006
        with self.assertRaisesRegex(ValueError, '零位标定'):
            bounded_start(planning_model('teaching_left'), raw)

    def test_fk_ik_roundtrip_uses_tcp_and_preserves_gripper(self):
        kin = Kinematics(planning_model('motion_left'))
        q = np.array(MOCK_Q)
        target_q = q + [.05, .04, -.03, .02, .01, -.02, 0.]
        target = kin.fk(target_q)
        solved = kin.ik(target, q)
        self.assertLess(np.linalg.norm(kin.fk(solved) - target), .01)
        self.assertEqual(solved[6], q[6])

    def test_unreachable_and_invalid_pose_fail(self):
        kin = Kinematics(planning_model('motion_left'))
        target = kin.fk(MOCK_Q)
        target[0, 3] = 4
        with self.assertRaises(ValueError):
            kin.ik(target, MOCK_Q)
        target[0, 0] = float('nan')
        with self.assertRaises(ValueError):
            kin.ik(target, MOCK_Q)

    def test_only_adjacent_and_finger_contacts_excluded(self):
        import xml.etree.ElementTree as ET
        m = planning_model('motion_left')
        semantic = ET.fromstring(m['semantic'])
        excluded = {frozenset((p.attrib['link1'], p.attrib['link2'])) for p in semantic.findall('disable_collisions')}
        self.assertNotIn(frozenset(('link1', 'link4')), excluded)
        self.assertEqual(semantic.find('group/chain').attrib['tip_link'], 'tool_tcp')
        self.assertEqual(len(semantic.findall('disable_collisions')), 10)


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.model = planning_model('motion_left')

    def test_sample_and_endpoints(self):
        plan = small_trajectory(self.model)
        q, v = plan.sample(.5)
        self.assertAlmostEqual(q[0], .0075)
        self.assertLessEqual(max(abs(v)), .25)
        self.assertAlmostEqual(plan.sample(2)[0][0], .015)
        self.assertTrue(np.all(plan.sample(2)[1] == 0))

    def test_logged_gripper_residual_can_only_recover_to_bounded_target(self):
        plan = gripper_motion(.00038952970121311915, -.001, self.model, .4, .2)
        q = [plan.sample(t)[0][0] for t in np.linspace(0, plan.duration, 200)]
        self.assertTrue(np.all(np.diff(q) <= 0))
        self.assertLessEqual(plan.peak_velocity[0], .003 * .4)
        self.assertLessEqual(plan.peak_acceleration[0], .006 * .2)
        with self.assertRaises(ValueError):
            gripper_motion(.0006, 0., self.model, .5, .5)
        with self.assertRaises(ValueError):
            gripper_motion(0., .0001, self.model, .5, .5)

    def test_independent_acceleration_retimes_same_positions(self):
        points = [{'time': 0., 'positions': list(MOCK_Q[:6]), 'velocities': [0.]*6},
                  {'time': 1., 'positions': [MOCK_Q[0]+.1, *MOCK_Q[1:6]], 'velocities': [0.]*6}]
        fast = Trajectory(list(ARM_JOINTS), points, self.model, .5, .8, retime=True)
        slow = Trajectory(list(ARM_JOINTS), points, self.model, .5, .1, retime=True)
        self.assertGreater(slow.duration, fast.duration)
        np.testing.assert_allclose(slow.q, fast.q)
        self.assertLessEqual(max(slow.peak_velocity), .125)
        self.assertLessEqual(max(slow.peak_acceleration), .05)

    def test_drift_expiry_bad_names_and_speed_rejected(self):
        plan = small_trajectory(self.model)
        wrong = np.array(MOCK_Q)
        wrong[0] = .1
        with self.assertRaises(ValueError):
            plan.check_start(wrong)
        plan.created -= 31
        with self.assertRaises(ValueError):
            plan.check_start(MOCK_Q)
        with self.assertRaises(ValueError):
            small_trajectory(self.model, duration=.01)
        with self.assertRaises(ValueError):
            Trajectory(['joint1']*6, [], self.model)

    def test_gripper_has_prismatic_speed_bounds_and_start_tolerance(self):
        plan = gripper_trajectory(self.model)
        self.assertEqual(plan.indices, [6])
        with self.assertRaises(ValueError):
            gripper_trajectory(self.model, duration=.1)
        with self.assertRaises(ValueError):
            gripper_trajectory(self.model, [*MOCK_Q[:6], -.036])
        moved = np.array(MOCK_Q)
        moved[6] -= .001
        with self.assertRaises(ValueError):
            plan.check_start(moved)

    def test_gripper_retiming_removes_sharp_end_deceleration(self):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        msg = JointTrajectory()
        msg.joint_names = list(GRIPPER_JOINTS)
        for q, v, ns in [(-.01, 0., 0), (-.011, -.0015, 900000000), (-.011007, 0., 921000000)]:
            p = JointTrajectoryPoint()
            p.positions, p.velocities = [q], [v]
            p.time_from_start.nanosec = ns
            msg.points.append(p)
        with self.assertRaisesRegex(ValueError, '加速度超限'):
            from_ros(msg, self.model)
        plan = gripper_profile(msg, self.model, .5)
        self.assertEqual(plan.group, 'gripper')
        self.assertAlmostEqual(plan.q[-1, 0], -.011007)
        self.assertGreater(plan.duration, 1.4)
        for elapsed in np.linspace(0, plan.duration, 100):
            q, v = plan.sample(elapsed)
            self.assertLessEqual(abs(v[0]), .0015)
            self.assertTrue(-.011007 - 1e-9 <= q[0] <= -.01 + 1e-9)

    def test_passive_joint_drift_invalidates_gripper_start(self):
        plan = gripper_trajectory(self.model)
        plan.passive_source = np.array(MOCK_Q)
        measured = np.array(MOCK_Q)
        measured[0] += .04
        with self.assertRaisesRegex(ValueError, '未规划的关节'):
            plan.check_start(measured)

    def test_nan_reversal_and_cubic_overshoot_rejected(self):
        base = [{'time': 0., 'positions': list(MOCK_Q[:6]), 'velocities': [0.]*6},
                {'time': 1., 'positions': list(MOCK_Q[:6]), 'velocities': [0.]*6}]
        for kind in ('nan', 'time', 'velocity', 'position'):
            p = deepcopy(base)
            if kind == 'nan':
                p[1]['positions'][0] = float('nan')
            elif kind == 'time':
                p[1]['time'] = 0.
            elif kind == 'velocity':
                p[1]['velocities'][2] = 1.
            else:
                p[1]['positions'][3] = 2.
            with self.assertRaises(ValueError):
                Trajectory(list(ARM_JOINTS), p, self.model)


class Bus:
    def __init__(self):
        self.sent = []
    def send(self, msg, timeout):
        self.sent.append(msg)


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.bus = Bus()
        self.control = MotionControl(bus=self.bus, model=planning_model('motion_left'))

    def test_readonly_start_forbids_motion_mode_enable_and_zero(self):
        commands = [(1, b'\xff'*7+b'\xfc'), (1, b'\xff'*7+b'\xfe'),
                    (0x7ff, bytes.fromhex('0100550a02000000')), (0x101, struct.pack('<ff', 0., .1))]
        for can_id, data in commands:
            with self.assertRaises(RuntimeError):
                self.control.send_std(can_id, data)
        self.assertEqual(self.bus.sent, [])
        self.control.send_std(0x7ff, status_payload(7))
        self.assertEqual(len(self.bus.sent), 1)

    def test_execute_allowlist_never_commands_gripper_or_saves_parameters(self):
        self.control.phase = 'preparing'
        self.control.owned.add(1)
        for can_id, data in [(7, b'\xff'*7+b'\xfc'), (0x107, struct.pack('<ff', 0., .1)),
                             (1, b'\xff'*7+b'\xfe'), (0x7ff, bytes.fromhex('0100aa0000000000')),
                             (0x7ff, bytes.fromhex('0100550802000000')),
                             (0x101, struct.pack('<ff', 0., 9.)),
                             (0x101, struct.pack('<ff', float('nan'), .1)),
                             (0x101, struct.pack('<ff', 9., .1))]:
            with self.assertRaises(RuntimeError):
                self.control.send_std(can_id, data)
        self.assertEqual(self.bus.sent, [])
        self.control.command(MOCK_Q[:6], [.1]*6)
        self.assertEqual([m.arbitration_id for m in self.bus.sent], list(range(0x101, 0x107)))
        self.assertAlmostEqual(struct.unpack('<ff', self.bus.sent[1].data)[0], 1.2, places=5)

    def test_batch_validated_before_any_send(self):
        self.control.phase = 'armed'
        q = list(MOCK_Q[:6])
        q[-1] = 99.
        with self.assertRaises(ValueError):
            self.control.command(q, [.1]*6)
        self.assertEqual(self.bus.sent, [])

    def test_mode_accepts_matching_read_response_without_enabling(self):
        class ModeBus(Bus):
            pending = None
            def send(self, msg, timeout):
                super().send(msg, timeout)
                data = bytearray(msg.data)
                data[2] = 0x33
                self.pending = can.Message(arbitration_id=data[0]+16, data=data, is_extended_id=False)
            def recv(self, timeout):
                result, self.pending = self.pending, None
                return result
        bus = ModeBus()
        control = MotionControl(bus=bus, model=planning_model('motion_left'))
        control.prepare(threading.Event())
        self.assertEqual(len(bus.sent), 6)
        self.assertFalse(control.owned)


@contextmanager
def simulated_arm(bad_mode=False, finite_speed=False, initial=None, response_ratio=.7):
    master, slave = pty.openpty()
    roles = load_roles()
    roles['motion_left']['port'] = os.ttyname(slave)
    os.close(slave)
    model = planning_model('motion_left', roles)
    q = np.array(MOCK_Q if initial is None else initial)
    target = q.copy()
    speed = np.zeros(7)
    enabled = set()
    frames = []
    stopped = threading.Event()
    def worker():
        buffer = b''
        last = time.monotonic()
        while not stopped.is_set():
            now = time.monotonic()
            if finite_speed:
                for i in enabled.copy():
                    # Slower than requested, making terminal lag observable.
                    step = speed[i-1] * response_ratio * (now - last)
                    q[i-1] += np.clip(target[i-1] - q[i-1], -step, step)
            last = now
            if not select.select([master], [], [], .01)[0]:
                continue
            try:
                buffer += os.read(master, 4096)
            except OSError as exc:
                if exc.errno == errno.EIO:
                    stopped.wait(.01)
                    continue
                return
            while b'\r' in buffer:
                command, buffer = buffer.split(b'\r', 1)
                if len(command) != 21 or not command.startswith(b't'):
                    continue
                can_id, data = int(command[1:4], 16), bytes.fromhex(command[5:].decode())
                frames.append((can_id, data))
                def reply(i, payload):
                    os.write(master, f't{i+16:03X}8{payload.hex()}\r'.encode())
                if can_id == 0x7ff and data[2] == 0xcc:
                    i = data[0]
                    entry = model['joints'][JOINTS[i-1]]
                    pmax = entry['position_scale_evidence']['pmax_rad']
                    motor = entry['sign'] * (q[i-1] - entry['offset']) / entry['position_scale']
                    raw = round((motor + pmax) / (2*pmax) * 65535)
                    payload = bytes([(16 if i in enabled else 0) | i, raw >> 8, raw & 255, 0, 0, 0, 24, 25])
                    reply(i, payload)
                elif can_id == 0x7ff and data[2:4] == b'\x55\x0a':
                    if not bad_mode:
                        reply(data[0], data)
                elif can_id in range(1, 8) and data == b'\xff'*7+b'\xfc':
                    enabled.add(can_id)
                elif can_id in range(1, 8) and data == b'\xff'*7+b'\xfd':
                    enabled.discard(can_id)
                elif can_id in range(0x101, 0x108) and can_id - 0x100 in enabled:
                    i = can_id-0x100
                    entry = model['joints'][JOINTS[i-1]]
                    p, v = struct.unpack('<ff', data)
                    target[i-1] = entry['sign'] * entry['position_scale'] * p + entry['offset']
                    speed[i-1] = v * entry['position_scale']
                    if not finite_speed:
                        q[i-1] = target[i-1]
    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    try:
        yield model, frames, enabled
    finally:
        stopped.set()
        thread.join(1)
        os.close(master)


class RuntimeTests(unittest.TestCase):
    def test_stalled_motor_aborts_path_and_endpoint_but_keeps_feedback_and_hold(self):
        for distance, duration, end_tolerance, reason in [
                (.3, 4., .02, '轨迹跟踪'), (.015, 1., .004, '轨迹终点')]:
            with self.subTest(reason=reason), simulated_arm(finite_speed=True, response_ratio=0.) as (model, frames, enabled):
                runtime = ArmRuntime(model, mock=False)
                runtime.start()
                try:
                    wait_for(lambda: runtime.snapshot()['q'] is not None)
                    q = runtime.measured()
                    end = q[:6].copy()
                    end[0] += distance
                    plan = Trajectory(list(ARM_JOINTS), [
                        {'time': 0., 'positions': q[:6], 'velocities': [0.]*6},
                        {'time': duration, 'positions': end, 'velocities': [0.]*6}], model, .5, .5)
                    plan.end_tolerance[:] = end_tolerance
                    generation = runtime.generation()
                    runtime.submit(plan, generation)
                    wait_for(lambda: runtime.snapshot()['failed'] == 1, 8)
                    state = runtime.snapshot()
                    self.assertIsNone(state['error'])
                    self.assertFalse(state['executing'])
                    self.assertEqual(state['completed'], 0)
                    self.assertTrue(runtime.worker.is_alive())
                    self.assertEqual(enabled, set(range(1, 7)))
                    self.assertIn(reason, state['motion_failure']['message'])
                    failed = [j for j in state['motion_failure']['joints'] if j['exceeded']]
                    self.assertEqual([j['name'] for j in failed], ['joint1'])
                    self.assertGreater(abs(failed[0]['error']), failed[0]['limit'])
                    self.assertGreater(runtime.generation(), generation)
                    with self.assertRaisesRegex(ValueError, '过期执行'):
                        runtime.submit(small_trajectory(model, runtime.measured()), generation)
                    frame_count = len(frames)
                    time.sleep(.2)
                    self.assertGreater(runtime.snapshot()['sample_time'], state['sample_time'])
                    self.assertFalse(any(data == b'\xff'*7+b'\xfd' for _, data in frames))
                    commands = [struct.unpack('<ff', data)[0] for cid, data in frames[frame_count:] if cid == 0x101]
                    self.assertTrue(commands)
                    self.assertTrue(all(abs(model['joints']['joint1']['sign']*p-q[0]) < .001 for p in commands))
                    # Explicitly submit a new, short trajectory after the fault.
                    runtime.submit(small_trajectory(model, runtime.measured()))
                    wait_for(lambda: runtime.snapshot()['completed'] == 1)
                finally:
                    self.stop(runtime)
                self.assertFalse(enabled)
                self.assertTrue(runtime.snapshot()['release_confirmed'])

    def test_unconfirmed_release_is_never_reported_as_disabled(self):
        with simulated_arm() as (model, _, enabled):
            runtime = ArmRuntime(model, mock=False)
            runtime.start()
            try:
                wait_for(lambda: runtime.snapshot()['q'] is not None)
                runtime.submit(small_trajectory(model, runtime.measured()))
                wait_for(lambda: runtime.snapshot()['completed'] == 1)
                with patch.object(MotionControl, 'release', side_effect=RuntimeError('模拟失能回读失败')):
                    self.stop(runtime)
                self.assertTrue(runtime.snapshot()['enabled'])
                self.assertFalse(runtime.snapshot()['release_confirmed'])
                self.assertIn('失能回读失败', runtime.snapshot()['error'])
            finally:
                self.stop(runtime)

    def test_terminal_error_correction_respects_independent_speed_acceleration(self):
        previous = np.zeros(2)
        caps, acceleration = np.array([.125, .0015]), np.array([.1, .0012])
        for _ in range(40):
            new = tracking_speeds([0., 0.], [.049, .001], previous, caps, acceleration, .05)
            self.assertTrue(np.all(new <= caps))
            self.assertTrue(np.all(np.abs(new - previous) <= acceleration * .05 + 1e-12))
            previous = new
        self.assertAlmostEqual(previous[0], .098)  # Does not collapse to old .02 floor.

    def test_finite_speed_motor_reaches_arm_then_recovers_reported_gripper_residual(self):
        with simulated_arm(finite_speed=True, initial=[*MOCK_Q[:6], .00038952970121311915]) as (model, frames, enabled):
            runtime = ArmRuntime(model, mock=False)
            runtime.start()
            try:
                wait_for(lambda: runtime.snapshot()['q'] is not None)
                q = runtime.measured()
                end = q[:6].copy()
                end[1:3] += [.12, .14]
                plan = Trajectory(list(ARM_JOINTS), [
                    {'time': 0., 'positions': q[:6], 'velocities': [0.]*6},
                    {'time': 2., 'positions': end, 'velocities': [0.]*6}], model, .5, .3, retime=True)
                runtime.submit(plan)
                wait_for(lambda: runtime.snapshot()['completed'] == 1, 12)
                self.assertLess(max(abs(runtime.measured()[:6]-end)), .02)
                runtime.submit(gripper_motion(runtime.measured()[6], -.001, model, .5, .3))
                wait_for(lambda: runtime.snapshot()['completed'] == 2, 12)
                self.assertLess(abs(runtime.measured()[6] + .001), .0005)
            finally:
                self.stop(runtime)
            self.assertIsNone(runtime.snapshot()['error'], runtime.snapshot()['error'])
            self.assertFalse(enabled)

    def stop(self, runtime):
        runtime.stop_motion.set()
        runtime.shutdown.set()
        runtime.worker.join(5)
        self.assertFalse(runtime.worker.is_alive())

    def test_mock_never_opens_serial_executes_holds_and_releases(self):
        model = planning_model('motion_left')
        with patch('hardware.open_arm_control', side_effect=AssertionError('opened hardware')):
            runtime = ArmRuntime(model, mock=True)
            runtime.start()
            try:
                wait_for(lambda: runtime.snapshot()['q'] is not None)
                runtime.submit(small_trajectory(model))
                wait_for(lambda: runtime.snapshot()['completed'] == 1)
                self.assertTrue(runtime.snapshot()['enabled'])
                runtime.release_requested.set()
                wait_for(lambda: not runtime.snapshot()['enabled'])
            finally:
                self.stop(runtime)
        self.assertIsNone(runtime.snapshot()['error'])
        self.assertEqual(runtime.snapshot()['tx_count'], 0)

    def test_real_transport_pty_is_readonly_until_execute_then_confirms_release(self):
        with simulated_arm() as (model, frames, enabled):
            runtime = ArmRuntime(model, mock=False)
            runtime.start()
            try:
                wait_for(lambda: runtime.snapshot()['q'] is not None)
                self.assertTrue(all(cid == 0x7ff and data == status_payload(data[0]) for cid, data in frames))
                runtime.submit(small_trajectory(model, runtime.measured()))
                wait_for(lambda: runtime.snapshot()['completed'] == 1)
                self.assertEqual(enabled, set(range(1, 7)))
                runtime.release_requested.set()
                wait_for(lambda: not runtime.snapshot()['enabled'])
                self.assertFalse(enabled)
            finally:
                self.stop(runtime)
            self.assertIsNone(runtime.snapshot()['error'], runtime.snapshot())
            for cid, data in frames:
                self.assertNotEqual(cid, 7)
                self.assertNotEqual(cid, 0x107)
                self.assertNotEqual(data, b'\xff'*7+b'\xfe')
                if cid == 0x7ff:
                    self.assertIn(data[2], (0xcc, 0x55))
                    if data[2] == 0x55:
                        self.assertEqual(data[3:], b'\x0a\x02\x00\x00\x00')

    def test_mode_failure_never_enables(self):
        with simulated_arm(bad_mode=True) as (model, frames, _):
            runtime = ArmRuntime(model, mock=False)
            runtime.start()
            try:
                wait_for(lambda: runtime.snapshot()['q'] is not None)
                runtime.submit(small_trajectory(model, runtime.measured()))
                wait_for(lambda: runtime.snapshot()['error'] is not None)
            finally:
                self.stop(runtime)
            self.assertIn('位置模式未回读', runtime.snapshot()['error'])
            self.assertFalse(any(data == b'\xff'*7+b'\xfc' for _, data in frames))

    def test_stale_feedback_and_repeated_submit_rejected(self):
        runtime = ArmRuntime(planning_model('motion_left'))
        runtime.update(q=list(MOCK_Q), sample_time=time.monotonic()-1)
        with self.assertRaises(ValueError):
            runtime.submit(small_trajectory(runtime.model))
        runtime.update(sample_time=time.monotonic())
        runtime.submit(small_trajectory(runtime.model))
        with self.assertRaises(ValueError):
            runtime.submit(small_trajectory(runtime.model))

    def test_stop_invalidates_a_late_collision_validation_result(self):
        runtime = ArmRuntime(planning_model('motion_left'))
        runtime.update(q=list(MOCK_Q), sample_time=time.monotonic())
        generation = runtime.generation()
        runtime.cancel_motion()
        with self.assertRaisesRegex(ValueError, '过期执行请求'):
            runtime.submit(small_trajectory(runtime.model), expected_generation=generation)
        self.assertIsNone(runtime.pending)

    def test_stop_during_motion_holds_and_close_confirms_disable(self):
        with simulated_arm() as (model, frames, enabled):
            runtime = ArmRuntime(model, mock=False)
            runtime.start()
            try:
                wait_for(lambda: runtime.snapshot()['q'] is not None)
                runtime.submit(small_trajectory(model, runtime.measured(), duration=2.))
                wait_for(lambda: runtime.snapshot()['enabled'])
                runtime.cancel_motion()
                wait_for(lambda: not runtime.snapshot()['executing'])
                held = runtime.measured()
                time.sleep(.15)
                self.assertLess(np.max(np.abs(runtime.measured() - held)), .001)
                self.assertEqual(enabled, set(range(1, 7)))
            finally:
                self.stop(runtime)
            self.assertFalse(enabled)
            self.assertIsNone(runtime.snapshot()['error'])

    def test_gripper_only_enables_id7_then_arm_keeps_gripper_position(self):
        with simulated_arm() as (model, frames, enabled):
            runtime = ArmRuntime(model, mock=False)
            runtime.start()
            try:
                wait_for(lambda: runtime.snapshot()['q'] is not None)
                before = runtime.measured()
                runtime.submit(gripper_trajectory(model, before))
                wait_for(lambda: runtime.snapshot()['completed'] == 1)
                self.assertEqual(enabled, {7})
                self.assertTrue(np.allclose(runtime.measured()[:6], before[:6]))
                self.assertAlmostEqual(runtime.measured()[6], before[6] - .001, places=4)
                self.assertFalse(any(cid in range(1, 7) or cid in range(0x101, 0x107) for cid, _ in frames))
                jaw = runtime.measured()[6]
                runtime.submit(small_trajectory(model, runtime.measured()))
                wait_for(lambda: runtime.snapshot()['completed'] == 2)
                self.assertEqual(enabled, set(range(1, 8)))
                self.assertAlmostEqual(runtime.measured()[6], jaw, places=4)
                runtime.cancel_motion(release=True)
                wait_for(lambda: not runtime.snapshot()['enabled'])
                self.assertFalse(enabled)
            finally:
                self.stop(runtime)
            self.assertIsNone(runtime.snapshot()['error'], runtime.snapshot()['error'])

    def test_gripper_stop_holds_and_shutdown_disables_only_id7(self):
        with simulated_arm() as (model, frames, enabled):
            runtime = ArmRuntime(model, mock=False)
            runtime.start()
            try:
                wait_for(lambda: runtime.snapshot()['q'] is not None)
                runtime.submit(gripper_trajectory(model, runtime.measured(), duration=2))
                wait_for(lambda: runtime.snapshot()['enabled'])
                runtime.cancel_motion()
                wait_for(lambda: not runtime.snapshot()['executing'])
                held = runtime.measured()
                time.sleep(.15)
                self.assertTrue(np.allclose(runtime.measured(), held, atol=.0001))
            finally:
                self.stop(runtime)
            self.assertFalse(enabled)
            self.assertIsNone(runtime.snapshot()['error'])
            self.assertEqual({cid for cid, data in frames if data == b'\xff'*7+b'\xfd'}, {7})

    def test_cancel_before_start_never_opens_serial(self):
        with patch('hardware.open_arm_control', side_effect=AssertionError('opened hardware')):
            runtime = ArmRuntime(planning_model('motion_left'), mock=False)
            runtime.cancel_motion(shutdown=True)
            runtime.start()
            runtime.worker.join(2)
            self.assertIsNone(runtime.snapshot()['error'])


if __name__ == '__main__':
    unittest.main()
