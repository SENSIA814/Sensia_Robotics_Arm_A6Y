"""Bounded, asynchronous observation of manual step tests; never sends commands."""
import csv
import json
import math
from pathlib import Path
import queue
import threading
import time

from gravity_config import JOINTS


ARM_FIELDS = ('position', 'velocity', 'effort', 'motor_position', 'motor_velocity',
              'sent_position', 'sent_motor_position', 'sent_velocity_limit',
              'send_begin_time', 'send_complete_time', 'feedback_receive_time', 'sent_mode',
              'command_count', 'feedback_count', 'feedback_age', 'position_valid',
              'position_age', 'fault', 'enabled', 'error')
CYCLE_FIELDS = ('control_time', 'control_cycle_count', 'control_period', 'control_overrun_count', 'dry_run')


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def finite(value):
    return value is not None and math.isfinite(value)


def atomic_json(path, data):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
    temp.replace(path)


class StepRecorder:
    def __init__(self, folder, pairs, on_error, capacity=4096, analyze=True, devices=None):
        self.folder = Path(folder)
        self.on_error, self.analyze = on_error, analyze
        self.queue = queue.Queue(maxsize=capacity)
        self.finished = threading.Event()
        self.done = threading.Event()
        self.failed = False
        self.analysis_complete = False
        self.received, self.dropped, self.written, self.cycles = {}, {}, {}, {}
        self.metadata = dict(schema=1, pairs=pairs, devices=devices or {}, nominal_arm_hz=250, nominal_target_hz=100,
            units=dict(angle='J1-J6: rad; J7 single-finger displacement: m',
                velocity='J1-J6: rad/s; J7: m/s', time='s', error='sent_position - position'),
            timebases=dict(control_time='host steady clock at native read start',
                send_begin_time='host steady clock before serial write',
                send_complete_time='host steady clock after serial accepted frame; NOT CAN ACK',
                feedback_receive_time='host steady clock when decoded; NOT motor sampling time',
                source_ros_time='publisher ROS clock, system clock in this local deployment',
                receive_monotonic='observer callback time; NOT source time'),
            notes=['One latest native snapshot per observed control cycle, not every CAN frame.',
                   'Followers command J1-J6 and J7 gripper; teacher J7 uses MIT contact haptics.',
                   'Teacher MIT gravity has no servo position target; empty values are intentional.',
                   'Positive error means target exceeds actual in calibrated joint coordinates.',
                   'Only following phase is captured. Exit homing is excluded.',
                   'Queue drops and skipped native cycles invalidate precise latency estimates.'])
        atomic_json(self.folder/'step_quality.json', self.quality(False))
        self.thread = threading.Thread(target=self._run, name='step-log-writer', daemon=True)
        self.thread.start()

    def put(self, kind, role, msg, leader=''):
        # A single ROS observer produces records. Never wait for disk or analysis.
        if self.finished.is_set() or self.failed:
            return
        key = kind + ':' + role
        self.received[key] = self.received.get(key, 0) + 1
        item = (kind, role, leader, time.time(), time.monotonic(), msg)
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.dropped[key] = self.dropped.get(key, 0) + 1

    def close(self):
        # Motor shutdown must never wait for a logger thread or disk flush.
        self.finished.set()

    def quality(self, complete, error=None):
        return dict(self.metadata, complete=complete, received=dict(self.received),
                    analysis_complete=self.analysis_complete,
                    written=dict(self.written), queue_dropped=dict(self.dropped),
                    cycles=dict(self.cycles), error=error)

    def _arm_row(self, role, wall, mono, msg):
        interfaces = {name: dict(zip(v.interface_names, v.values))
                      for name, v in zip(msg.joint_names, msg.interface_values)}
        first = interfaces.get(JOINTS[0], {})
        row = dict(role=role, receive_wall=wall, receive_monotonic=mono, source_ros_time=stamp(msg))
        row.update({key: first.get(key) for key in CYCLE_FIELDS})
        cycle = first.get('control_cycle_count')
        stat = self.cycles.setdefault(role, dict(first=None, last=None, skipped=0, duplicates=0, resets=0))
        if finite(cycle):
            previous = stat['last']
            if previous is None:
                stat['first'] = cycle
            elif cycle > previous:
                stat['skipped'] += max(0, int(cycle - previous - 1))
            elif cycle == previous:
                stat['duplicates'] += 1
            else:
                stat['resets'] += 1
            stat['last'] = cycle
        for index, joint in enumerate(JOINTS, 1):
            values = interfaces.get(joint, {})
            for name in ARM_FIELDS:
                row[f'{name}_j{index}'] = values.get(name)
            target, actual = values.get('sent_position'), values.get('position')
            if (values.get('sent_mode') == 2 and values.get('command_count', 0) > 0
                    and values.get('position_valid') == 1 and finite(target) and finite(actual)):
                row[f'error_j{index}'] = target - actual
        return row

    def _run(self):
        error = None
        try:
            with (self.folder/'step_arms.csv').open('w', newline='') as arm_file, \
                 (self.folder/'step_targets.csv').open('w', newline='') as target_file:
                arms = csv.DictWriter(arm_file, ['role', 'receive_wall', 'receive_monotonic',
                    'source_ros_time', *CYCLE_FIELDS] +
                    [f'{name}_j{i}' for i in range(1, 8) for name in ARM_FIELDS])
                targets = csv.DictWriter(target_file, ['kind', 'leader', 'follower', 'receive_wall',
                    'receive_monotonic', 'source_ros_time'] + [f'q_j{i}' for i in range(1, 8)])
                arms.writeheader(); targets.writeheader()
                last_flush = time.monotonic()
                while not self.finished.is_set() or not self.queue.empty():
                    try:
                        kind, role, leader, wall, mono, msg = self.queue.get(timeout=.1)
                    except queue.Empty:
                        continue
                    if kind == 'arm':
                        row = self._arm_row(role, wall, mono, msg)
                        writer = arms
                    else:
                        row = dict(kind=kind, leader=leader, follower=role, receive_wall=wall,
                                   receive_monotonic=mono, source_ros_time=stamp(msg))
                        # Use names when present; never silently interchange joint order.
                        positions = dict(zip(msg.name, msg.position))
                        row.update({f'q_j{i}': positions.get(name) for i, name in enumerate(JOINTS, 1)})
                        writer = targets
                    writer.writerow({k: (format(v, '.9f') if finite(v) else '')
                                     if isinstance(v, (float, int)) else v for k, v in row.items()})
                    key = kind + ':' + role
                    self.written[key] = self.written.get(key, 0) + 1
                    if time.monotonic() - last_flush >= 1:
                        arm_file.flush(); target_file.flush()
                        atomic_json(self.folder/'step_quality.json', self.quality(False))
                        last_flush = time.monotonic()
            atomic_json(self.folder/'step_quality.json', self.quality(True))
            if self.analyze:
                from analyze_teaching_step import analyze_folder
                analyze_folder(self.folder)
                self.analysis_complete = True
                atomic_json(self.folder/'step_quality.json', self.quality(True))
        except Exception as exc:
            error = str(exc)
            self.failed = True
            try:
                atomic_json(self.folder/'step_quality.json', self.quality(False, error))
            except OSError:
                pass
            self.on_error(RuntimeError('阶跃日志/分析失败：' + error))
        finally:
            self.done.set()
