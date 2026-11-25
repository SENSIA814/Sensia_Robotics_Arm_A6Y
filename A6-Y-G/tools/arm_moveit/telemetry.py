"""Read-only feedback sampling and bounded asynchronous session recording."""
from collections import deque
from copy import deepcopy
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import threading
import time
from zoneinfo import ZoneInfo

import numpy as np
import pinocchio as pin
from motion_model import JOINTS, Kinematics
from gripper_policy import percent_from_position


def finite(value):
    return float(value) if value is not None and math.isfinite(value) else None


def gripper_open_percent(position, entry):
    # Preserve out-of-range feedback for diagnosis; 0/100 are model endpoints.
    return None if position is None else 100*(entry['upper']-position)/(entry['upper']-entry['lower'])


class FeedbackSampler:
    def __init__(self, model):
        self.model = model
        self.kin = Kinematics(model)  # This thread owns its Pinocchio data.
        self.started = time.monotonic()

    def sample(self, hardware, now=None):
        now = time.monotonic() if now is None else now
        stamp = hardware.get('sample_time')
        age = None if stamp is None else max(0., now - stamp)
        diagnostic_stamp = hardware.get('diagnostic_time')
        diagnostic_age = None if diagnostic_stamp is None else max(0., now - diagnostic_stamp)
        q, velocity = hardware.get('q'), hardware.get('velocity')
        fresh = q is not None and age is not None and age <= .5
        valid = []
        rows = hardware.get('rows', {})
        for name in JOINTS:
            row = rows.get(name, {})
            valid.append(bool(fresh and (hardware.get('mock', False) or
                         (diagnostic_age is not None and diagnostic_age <= .5 and
                          row.get('has_feedback') == 1 and row.get('feedback_age', math.inf) <= .5 and
                          row.get('fault', 1) == 0))))
        q = list(q) if q is not None else [None] * 7
        velocity = list(velocity) if velocity is not None else [None] * 7
        position = [finite(v) if valid[i] else None for i, v in enumerate(q)]
        speed = [finite(v) if valid[i] else None for i, v in enumerate(velocity)]
        motor_q, motor_v = [None]*7, [None]*7
        for i, name in enumerate(JOINTS):
            entry = self.model['joints'][name]
            factor = entry['sign'] / entry['position_scale']
            motor = entry['motor_id']-1
            motor_q[motor] = None if position[i] is None else math.degrees((position[i] - entry['offset']) * factor)
            motor_v[motor] = None if speed[i] is None else math.degrees(speed[i] * factor)
        xyz, rpy = [None]*3, [None]*3
        absolute_xyz, absolute_rpy = [None]*3, [None]*3
        # tool_tcp does not depend on finger position; retain TCP when only the gripper is missing.
        if all(v is not None for v in position[:6]):
            pose = self.kin.fk(position[:6] + [position[6] if position[6] is not None else 0.])
            absolute_xyz = (pose[:3, 3] * 1000).tolist()
            absolute_rpy = np.rad2deg(pin.rpy.matrixToRpy(pose[:3, :3])).tolist()
            reference = hardware.get('plot_zero_pose')
            if hardware.get('plot_relative'):
                if reference is not None:
                    reference = np.asarray(reference, float)
                    xyz = ((pose[:3, 3] - reference[:3, 3])*1000).tolist()
                    rpy = np.rad2deg(pin.rpy.matrixToRpy(reference[:3, :3].T @ pose[:3, :3])).tolist()
            else:
                xyz, rpy = absolute_xyz, absolute_rpy
        return dict(time_unix=time.time(), elapsed_s=now-self.started, feedback_age_s=age,
                    diagnostic_age_s=diagnostic_age, ros_stamp_s=hardware.get('ros_stamp'),
                    valid=valid, raw_position=q, raw_velocity=velocity, rows=deepcopy(rows),
                    tcp_xyz_mm=xyz, tcp_rpy_deg=rpy,
                    tcp_absolute_xyz_mm=absolute_xyz, tcp_absolute_rpy_deg=absolute_rpy,
                    plot_relative=hardware.get('plot_relative', False), plot_zero_at=hardware.get('plot_zero_at'),
                    joint_deg=[None if v is None else math.degrees(v) for v in position[:6]],
                    joint_deg_s=[None if v is None else math.degrees(v) for v in speed[:6]],
                    motor_deg=motor_q, motor_deg_s=motor_v,
                    gripper_mm=[None if position[6] is None else position[6]*1000],
                    gripper_mm_s=[None if speed[6] is None else speed[6]*1000],
                    gripper_model_percent=gripper_open_percent(position[6], self.model['joints'][JOINTS[6]]),
                    gripper_feedback_percent=percent_from_position(self.model['gripper_limits'], position[6]),
                    gripper_open_percent=[percent_from_position(self.model['gripper_limits'], position[6]),
                                          hardware.get('gripper_target_percent')],
                    gripper_target_percent=hardware.get('gripper_target_percent'),
                    gripper_target_mm=hardware.get('gripper_target_mm'),
                    gripper_motion=('unknown' if speed[6] is None else 'opening' if speed[6] < -.00005 else
                                    'closing' if speed[6] > .00005 else 'stationary'),
                    execution_group=hardware.get('execution_group', ''),
                    enabled=hardware.get('enabled', False), executing=hardware.get('executing', False))


GROUPS = {'tcp_xyz_mm': ('x', 'y', 'z'), 'tcp_rpy_deg': ('roll', 'pitch', 'yaw'),
          'tcp_absolute_xyz_mm': ('x', 'y', 'z'), 'tcp_absolute_rpy_deg': ('roll', 'pitch', 'yaw'),
          'joint_deg': tuple(f'j{i}' for i in range(1, 7)),
          'joint_deg_s': tuple(f'j{i}' for i in range(1, 7)),
          'motor_deg': tuple(f'm{i}' for i in range(1, 8)),
          'motor_deg_s': tuple(f'm{i}' for i in range(1, 8)),
          'gripper_mm': ('finger',), 'gripper_mm_s': ('finger',),
          'gripper_open_percent': ('feedback', 'target')}


def flatten(sample):
    row = {k: sample[k] for k in ('time_unix', 'elapsed_s', 'ros_stamp_s', 'feedback_age_s', 'diagnostic_age_s', 'enabled', 'executing')}
    row['time_iso'] = datetime.fromtimestamp(sample['time_unix'], timezone.utc).isoformat()
    for key in ('gripper_feedback_percent', 'gripper_model_percent', 'gripper_target_percent', 'gripper_target_mm', 'gripper_motion', 'execution_group',
                'plot_relative', 'plot_zero_at'):
        row[key] = sample[key]
    for key, labels in GROUPS.items():
        row.update({f'{key}_{label}': sample[key][i] for i, label in enumerate(labels)})
    for i, name in enumerate(JOINTS):
        row[f'{name}_valid'] = sample['valid'][i]
        row[f'{name}_position_si'] = finite(sample['raw_position'][i])
        row[f'{name}_velocity_si'] = finite(sample['raw_velocity'][i])
        for field in ('motor_error', 'fault', 'has_feedback', 'feedback_age', 'enabled'):
            row[f'{name}_{field}'] = finite(sample['rows'].get(name, {}).get(field))
    return row


class SessionRecorder:
    """No file I/O in ROS callbacks/UI. CSV remains active when plots are hidden."""
    def __init__(self, folder, model, runtime, hz=20):
        self.folder, self.model, self.runtime = Path(folder), model, runtime
        self.hz = hz
        self.queue = queue.Queue(maxsize=4096)
        self.lock = threading.Lock()
        self.history = deque(maxlen=hz*120)
        self.closed = threading.Event()
        self.error, self.dropped, self.sequence = '', 0, 0
        self.thread = threading.Thread(target=self._run, name='a6y-session-log', daemon=True)
        self.thread.start()

    def event(self, kind, message='', **details):
        if self.closed.is_set():
            return
        record = dict(time=time.time(), kind=kind, message=message, **details)
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            with self.lock:
                self.dropped += 1

    def snapshot(self, after=0):
        with self.lock:
            return {'error': self.error, 'dropped_events': self.dropped,
                    'samples': [s for s in self.history if s['sequence'] > after]}

    def close(self):
        self.closed.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            with self.lock:
                self.error = '日志线程未在 2 秒内完成写入，请检查磁盘；未阻塞 ROS 退出'

    def _run(self):
        try:
            sampler = FeedbackSampler(self.model)
            metadata = dict(schema=3, role=self.model['role'], mock=self.runtime.mock,
                            sample_hz=self.hz, history_seconds=120, frame='base_link', tip='tool_tcp',
                            orientation='R = Rz(yaw) Ry(pitch) Rx(roll); canonical RPY, wrap discontinuities shown as gaps',
                            plot_reference='When plot_relative=True, XYZ=p-p0 in base axes and RPY=RPY(R0.T@R). Reference is measured after successful home; absolute FK always logged separately. Missing reference gives blank plots.',
                            motor_angles='Derived from upstream filtered joint feedback using sign, position_scale, offset; indexed by motor ID, not raw CAN frames',
                            velocity='JointState velocity from original driver; not position finite differences (except upstream dry_run)',
                            missing='Blank CSV fields and plot gaps. Raw SI feedback retained with validity flags.',
                            gripper='feedback_percent: 0 closed / 100 open electronic endpoints, unclamped. model_percent preserves original URDF range. Target is a requested goal, not a motion command. motion: velocity threshold 0.05 mm/s; unknown when missing.',
                            gripper_limits=self.model['gripper_limits'],
                            columns=GROUPS, raw_si='J1-J6 rad/rad_s; gripper m/m_s',
                            urdf_sha256=self.model['urdf_sha256'], joints=self.model['joints'],
                            collision_policy=self.model['collision_policy'])
            (self.folder/'telemetry_metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
            with (self.folder/'telemetry.csv').open('w', newline='', encoding='utf-8') as csvfile, \
                 (self.folder/'events.jsonl').open('a', encoding='utf-8') as events, \
                 (self.folder/'application.log').open('a', encoding='utf-8') as readable, \
                 (self.folder/'gripper.log').open('a', encoding='utf-8') as gripper_log:
                writer = None
                next_sample, last_flush = time.monotonic(), time.monotonic()
                last_hardware = None
                while not self.closed.is_set() or not self.queue.empty():
                    now = time.monotonic()
                    if not self.closed.is_set() and now >= next_sample:
                        h = self.runtime.snapshot()
                        h['mock'] = self.runtime.mock
                        sample = sampler.sample(h, now)
                        row = flatten(sample)
                        if writer is None:
                            writer = csv.DictWriter(csvfile, fieldnames=list(row))
                            writer.writeheader()
                        writer.writerow(row)
                        with self.lock:
                            self.sequence += 1
                            sample['sequence'] = self.sequence
                            self.history.append(sample)
                        hardware_state = {key: h.get(key) for key in ('status', 'error', 'enabled', 'executing', 'completed', 'control_ready')}
                        if hardware_state != last_hardware:
                            self.event('hardware', hardware_state['status'], hardware=hardware_state)
                            last_hardware = hardware_state
                        next_sample = now + 1/self.hz
                    # Bounded batch prevents dragging/ROS events starving telemetry.
                    for _ in range(200):
                        try:
                            item = self.queue.get_nowait()
                        except queue.Empty:
                            break
                        item['time_iso'] = datetime.fromtimestamp(item['time'], timezone.utc).isoformat()
                        if item['kind'] == 'plan_saved' and 'trajectory' in item:
                            (self.folder/item['trajectory_file']).write_text(json.dumps(item.pop('trajectory'), indent=2)+'\n')
                        encoded = json.dumps(item, ensure_ascii=False, default=lambda v: v.tolist() if hasattr(v, 'tolist') else str(v))
                        events.write(encoded+'\n')
                        local_time = datetime.fromtimestamp(item['time'], ZoneInfo('Asia/Shanghai')).isoformat(timespec='milliseconds')
                        line = f"{local_time} [{item['kind']}] {item['message']} {encoded}\n"
                        readable.write(line)
                        if (item['kind'].startswith('gripper_') or item.get('group') == 'gripper' or
                                item.get('command') in ('gripper_target', 'gripper_plan', 'execute_gripper')):
                            gripper_log.write(line)
                    if now - last_flush >= 1. or self.closed.is_set():
                        csvfile.flush(); events.flush(); readable.flush(); gripper_log.flush()
                        last_flush = now
                    if not self.closed.is_set():
                        self.closed.wait(max(0., min(.02, next_sample-time.monotonic())))
        except Exception as exc:
            with self.lock:
                self.error = f'日志/遥测记录失败：{type(exc).__name__}: {exc}'
