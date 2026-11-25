"""Single owner of the serial bus; execution is separate from planning/dragging."""
from collections import deque
from contextlib import ExitStack
from copy import deepcopy
import struct
import threading
import time

import numpy as np

from motion_model import ARM_JOINTS, JOINTS, MAX_SPEED, MOCK_Q, joint_speed, joint_accel, joint_tolerance
from scanner import MotorControl, open_arm_control, status_payload, timestamp
from feedback import read_cycle, complete_positions
from operation_lock import exclusive_operation


class MotionControl(MotorControl):
    """Allow only status, RAM mode=POS_VEL, bounded positions and owned enables."""
    def __init__(self, *args, model, tx=None, **kwargs):
        self.model = model
        self.entries = {model['joints'][n]['motor_id']: model['joints'][n] for n in JOINTS}
        self.names_by_id = {model['joints'][n]['motor_id']: n for n in JOINTS}
        self.command_bounds = {n: (model['joints'][n]['lower'], model['joints'][n]['upper']) for n in JOINTS}
        self.prepared = {model['joints'][n]['motor_id'] for n in ARM_JOINTS}
        self.phase = 'readonly'
        self.owned = set()
        self.tx = tx if tx is not None else deque(maxlen=100)
        self.tx_count = 0
        super().__init__(*args, **kwargs)

    def send_std(self, arbitration_id, payload, *, pad_to_8=True):
        data = bytes(payload)
        allowed = arbitration_id == 0x7ff and data in [status_payload(i) for i in range(1, 8)]
        if self.phase in ('preparing', 'armed') and len(data) == 8:
            if arbitration_id == 0x7ff:
                allowed |= any(data == bytes([i, 0, 0x55, 10, 2, 0, 0, 0]) for i in self.prepared)
            if arbitration_id in self.owned:
                allowed |= data in (b'\xff' * 7 + b'\xfc', b'\xff' * 7 + b'\xfd')
            motor_id = arbitration_id - 0x100
            if motor_id in self.prepared:
                p, v = struct.unpack('<ff', data)
                e = self.entries[motor_id]
                name = self.names_by_id[motor_id]
                lower, upper = self.command_bounds[name]
                q = e['sign'] * e['position_scale'] * p + e['offset']
                allowed |= (np.isfinite([p, v]).all() and 0 < v <= joint_speed(name) / e['position_scale'] * 1.001
                            and abs(p) <= e['position_scale_evidence']['pmax_rad']
                            and lower - joint_tolerance(name) <= q <= upper + joint_tolerance(name))
        # Cleanup after a failure is permitted for motors we attempted to enable.
        if arbitration_id in self.owned and data == b'\xff' * 7 + b'\xfd':
            allowed = True
        if not allowed:
            raise RuntimeError('已拦截未授权电机指令或超限位置/速度')
        super().send_std(arbitration_id, data, pad_to_8=pad_to_8)
        self.tx_count += 1
        self.tx.append({'can_id': arbitration_id, 'data': data.hex(), 'at': timestamp()})

    def prepare(self, cancel, names=ARM_JOINTS):
        ids = [self.model['joints'][n]['motor_id'] for n in names]
        self.prepared = set(ids) | self.owned
        self.phase = 'preparing'
        for i in ids:
            if cancel.is_set():
                raise RuntimeError('已取消电机准备')
            for _ in range(512):
                if self.bus.recv(timeout=0) is None:
                    break
            else:
                raise RuntimeError('准备期间总线持续繁忙')
            request = bytes([i, 0, 0x55, 10, 2, 0, 0, 0])
            self.send_std(0x7ff, request)
            deadline = time.monotonic() + .5
            confirmed = False
            while time.monotonic() < deadline and not cancel.is_set():
                msg = self.bus.recv(timeout=.01)
                if msg is None:
                    continue
                if (msg.arbitration_id == i + 16 and len(msg.data) == 8
                        and not any(getattr(msg, f, False) for f in ('is_extended_id', 'is_error_frame', 'is_remote_frame', 'is_fd'))
                        and bytes(msg.data[:2]) == request[:2] and msg.data[2] in (0x33, 0x55)
                        and bytes(msg.data[3:]) == request[3:]):
                    confirmed = True
                    break
            if not confirmed:
                raise RuntimeError(f'ID {i} 的位置模式未回读确认；没有继续使能')

    def command(self, q, velocity, names=ARM_JOINTS):
        # Validate the entire batch before sending its first frame.
        q, velocity = np.asarray(q, float), np.asarray(velocity, float)
        if q.shape != (len(names),) or velocity.shape != (len(names),) or not np.isfinite(q).all() or not np.isfinite(velocity).all():
            raise ValueError('位置与速度指令必须包含匹配关节数量的有限值')
        frames = []
        for index, name in enumerate(names):
            e = self.model['joints'][name]
            motor_q = e['sign'] * (float(q[index]) - e['offset']) / e['position_scale']
            # POS_VEL's second field is a positive speed limit. A tiny transport
            # floor avoids zero semantics without overriding the user's limit.
            speed = min(joint_speed(name), max(1e-8, abs(float(velocity[index])))) / e['position_scale']
            lower, upper = self.command_bounds[name]
            if (e['motor_id'] not in self.prepared or not np.isfinite([motor_q, speed]).all() or abs(motor_q) > e['position_scale_evidence']['pmax_rad']
                    or not lower - joint_tolerance(name) <= q[index] <= upper + joint_tolerance(name)):
                raise ValueError(f'{name} 指令超限')
            frames.append((e['motor_id'] + 0x100, struct.pack('<ff', motor_q, speed)))
        for can_id, data in frames:
            self.send_std(can_id, data)

    def allow_trajectory(self, trajectory):
        for j, name in enumerate(trajectory.names):
            self.command_bounds[name] = (trajectory.command_lower[j], trajectory.command_upper[j])

    def enable(self, cancel, names=ARM_JOINTS):
        for i in [self.model['joints'][n]['motor_id'] for n in names]:
            if cancel.is_set():
                raise RuntimeError('已取消使能')
            self.owned.add(i)  # Include uncertain sends in later cleanup.
            self.send_std(i, b'\xff' * 7 + b'\xfc')
        self.phase = 'armed'

    def release(self):
        if not self.owned:
            self.phase = 'readonly'
            return
        failures = []
        for i in sorted(self.owned):
            try:
                self.send_std(i, b'\xff' * 7 + b'\xfd')
            except Exception as exc:
                failures.append(f'ID {i}: {exc}')
        self.phase = 'readonly'
        if failures:
            # Retain IDs for the final best-effort close retry; never claim success.
            raise RuntimeError('失能发送未完成：' + '; '.join(failures))
        deadline = time.monotonic() + .4
        while time.monotonic() < deadline:
            rows = read_cycle(self, self.model, threading.Event(), timeout=.02)
            if all(rows.get(n, {}).get('feedback') and rows[n]['feedback']['state'] == 0 for n in JOINTS if self.model['joints'][n]['motor_id'] in self.owned):
                self.owned.clear()
                return
        raise RuntimeError('已发送失能，但回读未确认；请检查硬件并支撑机械臂')


class ArmRuntime:
    def __init__(self, model, mock=True):
        self.model, self.mock = model, mock
        self.lock = threading.Lock()
        self.shutdown = threading.Event()
        self.stop_motion = threading.Event()
        self.release_requested = threading.Event()
        self.pending = None
        self.cancel_generation = 0
        self.worker = None
        self.state = {'q': None, 'sample_time': None, 'rows': {}, 'status': '正在连接',
                      'error': None, 'enabled': False, 'executing': False, 'completed': 0,
                      'failed': 0, 'motion_failure': None, 'release_confirmed': True}

    def update(self, **values):
        with self.lock:
            self.state.update(values)

    def snapshot(self):
        with self.lock:
            return deepcopy(self.state)

    def measured(self):
        s = self.snapshot()
        if s['error'] or s['q'] is None or s['sample_time'] is None or time.monotonic() - s['sample_time'] > .25:
            raise ValueError('当前反馈缺失、过期或存在故障')
        return np.array(s['q'])

    def cancel_motion(self, release=False, shutdown=False):
        with self.lock:
            self.cancel_generation += 1
            self.pending = None
            self.stop_motion.set()
        if release:
            self.release_requested.set()
        if shutdown:
            self.shutdown.set()

    def generation(self):
        with self.lock:
            return self.cancel_generation

    def submit(self, trajectory, expected_generation=None):
        q = self.measured()
        trajectory.check_start(q)
        with self.lock:
            if expected_generation is not None and expected_generation != self.cancel_generation:
                raise ValueError('执行检查期间已请求停止，已丢弃过期执行请求')
            if self.pending is not None or self.state['executing']:
                raise ValueError('已有轨迹正在执行')
            self.stop_motion.clear()
            self.pending = trajectory
            self.state['executing'] = True

    def start(self):
        self.worker = threading.Thread(target=self._run, daemon=False)
        self.worker.start()

    def _reject_trajectory(self, trajectory, q, desired, elapsed, kind, indices, limits):
        """Abort one motion, not the feedback worker or its position hold.

        Only called after fresh, healthy feedback. Bus/drive/hold faults still
        take the fatal cleanup path. Do not resume or retry the failed motion.
        """
        joints = []
        for j, i in enumerate(indices):
            joints.append({'name': JOINTS[i], 'unit': 'm' if i == 6 else 'rad',
                           'actual': float(q[i]), 'desired': float(desired[j]),
                           'error': float(q[i] - desired[j]), 'limit': float(limits[j]),
                           'exceeded': bool(abs(q[i] - desired[j]) > limits[j])})
        failures = [j for j in joints if j['exceeded']]
        details = '；'.join(f'{j["name"]}: 目标 {j["desired"]:.6f} / 反馈 {j["actual"]:.6f} '
                           f'/ 偏差 {j["error"]:+.6f} / 限值 {j["limit"]:.6f} {j["unit"]}' for j in failures)
        message = f'{kind}（{elapsed:.2f}/{trajectory.duration:.2f} s）；{details}。本次轨迹已中止，保持当前位置，请检查后重新规划'
        with self.lock:
            self.cancel_generation += 1  # Reject late execute/validation results.
            self.pending = None
            sequence = self.state['failed'] + 1
            self.state.update(executing=False, failed=sequence, status='轨迹已中止 · 保持当前位置',
                              motion_failure={'sequence': sequence, 'at': time.time(), 'kind': kind,
                                              'message': message, 'group': trajectory.group,
                                              'elapsed': elapsed, 'duration': trajectory.duration,
                                              'velocity_scaling': trajectory.velocity_scaling,
                                              'acceleration_scaling': trajectory.acceleration_scaling,
                                              'joints': joints})

    def _run(self):
        control = None
        tx = deque(maxlen=100)
        trajectory = hold = None
        active = set()
        speed_caps = np.array([joint_speed(n) for n in JOINTS])
        accel_caps = np.array([joint_accel(n) for n in JOINTS])
        command_speeds = np.zeros(7)
        q = np.array(MOCK_Q)
        start_time = last_loop = time.monotonic()
        try:
            if self.shutdown.is_set():
                return
            with ExitStack() as stack:
                if not self.mock:
                    stack.enter_context(exclusive_operation())
                    arm = {**self.model['arm'], 'tx': tx}
                    def factory(**kwargs):
                        return MotionControl(model=self.model, **kwargs)
                    control = stack.enter_context(open_arm_control(arm, factory))
                    # Cleanup must run before the context closes the serial bus.
                    stack.callback(self._release, control)
                while not self.shutdown.is_set():
                    now = time.monotonic()
                    dt = min(.1, max(.001, now - last_loop))
                    if control and control.owned and now - last_loop > .25:
                        raise RuntimeError('控制循环超时，停止执行')
                    last_loop = now
                    if control:
                        rows = read_cycle(control, self.model, self.shutdown, timeout=.025)
                        if self.shutdown.is_set():
                            break  # An intentional teardown may interrupt a read cycle.
                        positions = complete_positions(rows, max_age=.25)
                        faulty = [n for n, r in rows.items() if r['feedback'] and r['feedback']['state'] not in (0, 1)]
                        if positions is None or faulty:
                            self.update(q=None, rows=rows, status='反馈缺失或电机状态异常')
                            if control.owned or trajectory:
                                raise RuntimeError('执行期间反馈缺失或电机故障，已停止')
                            self.shutdown.wait(.05)
                            continue
                        q = np.array(positions)
                        if control.owned and any(rows[n]['feedback']['state'] != 1 for n in JOINTS if self.model['joints'][n]['motor_id'] in control.owned):
                            raise RuntimeError('执行电机意外失能')
                        self.update(rows=rows)
                    self.update(q=q.tolist(), sample_time=time.monotonic())
                    if self.release_requested.is_set():
                        self.release_requested.clear()
                        if control:
                            control.release()
                        hold = trajectory = None
                        active.clear()
                        command_speeds[:] = 0.
                        with self.lock:
                            self.pending = None
                        self.update(enabled=False, executing=False, release_confirmed=True, status='已释放电机 · 只读反馈')
                    if self.stop_motion.is_set():
                        trajectory = None
                        hold = q.copy() if active else None
                        with self.lock:
                            self.pending = None
                        self.update(executing=False, status='轨迹已停止，保持当前位置' if hold is not None else '轨迹已取消')
                        self.stop_motion.clear()
                    with self.lock:
                        pending, self.pending = self.pending, None
                    if pending is not None:
                        if self.stop_motion.is_set() or self.shutdown.is_set():
                            continue
                        pending.check_start(q)
                        speed_caps[pending.indices] = [joint_speed(n) * pending.velocity_scaling for n in pending.names]
                        accel_caps[pending.indices] = [joint_accel(n) * pending.acceleration_scaling for n in pending.names]
                        if control:
                            control.allow_trajectory(pending)
                        new_names = [n for n in pending.names if JOINTS.index(n) not in active]
                        if control and new_names:
                            if any(rows[n]['feedback']['state'] != 0 for n in new_names):
                                raise RuntimeError('检测到其他程序已使能目标电机，拒绝接管')
                            self.update(status='准备位置模式并确认反馈')
                            control.prepare(self.stop_motion, new_names)
                            if self.stop_motion.is_set() or self.shutdown.is_set():
                                raise RuntimeError('使能前已取消执行')
                            rows = read_cycle(control, self.model, self.shutdown, timeout=.025)
                            positions = complete_positions(rows, max_age=.25)
                            if positions is None or any(rows[n]['feedback']['state'] != 0 for n in new_names):
                                raise RuntimeError('使能前反馈不完整或状态变化')
                            q = np.array(positions)
                            pending.check_start(q)
                            if self.stop_motion.is_set() or self.shutdown.is_set():
                                raise RuntimeError('使能前已取消执行')
                            indices = [JOINTS.index(n) for n in new_names]
                            control.command(q[indices], np.zeros(len(indices)), new_names)
                            control.enable(self.stop_motion, new_names)
                            rows = read_cycle(control, self.model, self.shutdown, timeout=.025)
                            positions = complete_positions(rows, max_age=.25)
                            if positions is None or any(rows[n]['feedback']['state'] != 1 for n in new_names):
                                raise RuntimeError('使能回读未确认')
                            q = np.array(positions)
                            pending.check_start(q)
                            self.update(q=q.tolist(), sample_time=time.monotonic(), rows=rows)
                        active.update(pending.indices)
                        hold = q.copy()
                        passive_reference = getattr(pending, 'passive_source', q).copy()
                        trajectory, start_time = pending, time.monotonic()
                        last_loop = start_time
                        self.update(enabled=True, executing=True, release_confirmed=False,
                                    status='正在执行' + ('夹爪' if pending.group == 'gripper' else '机械臂') + '规划轨迹')
                    velocities = np.zeros(7)
                    if trajectory is not None:
                        elapsed = time.monotonic() - start_time
                        desired, velocity = trajectory.sample(elapsed)
                        indices = trajectory.indices
                        passive = [i for i in range(7) if i not in indices]
                        passive_limits = [joint_tolerance(JOINTS[i], .03, .0005) for i in passive]
                        failure = None
                        if np.any(np.abs(q[indices] - desired) > trajectory.tracking_tolerance):
                            failure = ('轨迹跟踪偏差超过允许范围', indices, desired, trajectory.tracking_tolerance)
                        elif any(abs(q[i] - passive_reference[i]) > limit for i, limit in zip(passive, passive_limits)):
                            failure = ('未规划的关节已移动', passive, passive_reference[passive], passive_limits)
                        elif elapsed > trajectory.duration + 3 and np.any(np.abs(q[indices] - desired) > trajectory.end_tolerance):
                            failure = ('轨迹终点未在容差内到达', indices, desired, trajectory.end_tolerance)
                        if failure is not None:
                            kind, checked, reference, limits = failure
                            self._reject_trajectory(trajectory, q, reference, elapsed, kind, checked, limits)
                            trajectory = None
                            hold = q.copy()  # Fresh feedback, never continue the failed target.
                        else:
                            hold[indices], velocities[indices] = desired, velocity
                            if not control:
                                q[indices] = desired
                            if elapsed >= trajectory.duration and np.all(np.abs(q[indices] - desired) <= trajectory.end_tolerance):
                                trajectory = None
                                self.update(executing=False, completed=self.snapshot()['completed'] + 1,
                                            status='轨迹完成 · 保持目标位置')
                    if hold is not None and control and not self.stop_motion.is_set() and not self.release_requested.is_set():
                        indices = sorted(active)
                        names = [JOINTS[i] for i in indices]
                        if any(abs(q[i] - hold[i]) > joint_tolerance(JOINTS[i], .15, .003) for i in indices):
                            raise RuntimeError('保持位置期间偏差过大')
                        command_speeds = tracking_speeds(velocities, hold - q, command_speeds, speed_caps, accel_caps, dt)
                        control.command(hold[indices], command_speeds[indices], names)
                    elif not self.snapshot()['enabled']:
                        self.update(status='模拟状态已就绪' if self.mock else '只读反馈已就绪，规划不会运动')
                    self.shutdown.wait(max(0., .05 - (time.monotonic() - last_loop)))
        except Exception as exc:
            cleanup_error = self.snapshot()['error']
            self.update(error=f'{type(exc).__name__}: {exc}' + (f'；{cleanup_error}' if cleanup_error else ''),
                        q=None, status='执行已停止，请检查')
        finally:
            # A failed disable acknowledgement must never masquerade as a
            # released arm and permit the normal close path to bypass homing.
            uncertain = bool(control and control.owned)
            self.update(executing=False, enabled=uncertain, release_confirmed=not uncertain, tx_tail=list(tx),
                        tx_count=control.tx_count if control else 0)

    def _release(self, control):
        try:
            control.release()
        except Exception as exc:
            self.update(error=f'{self.snapshot()["error"] or ""}；{exc}')


def tracking_speeds(feedforward, error, previous, speed_caps, accel_caps, dt):
    """Bounded POS_VEL speed envelope, including terminal position correction."""
    desired = np.minimum(speed_caps, np.abs(feedforward) + 2. * np.abs(error))
    delta = np.asarray(accel_caps) * dt
    return np.clip(np.clip(desired, previous - delta, previous + delta), 0., speed_caps)
