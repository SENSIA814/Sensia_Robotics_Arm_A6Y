"""Write the chosen arm's present pose as motor zero; never command motion.

The existing dm_canable2 driver uses seven FF bytes followed by FE for zero.
Zero commands are not retried. Every write must be followed by fresh feedback;
a transport error cannot prove that the physical write did not happen.
"""
from copy import deepcopy
import threading
import time

from operation_lock import exclusive_operation
from position_feedback import load_position_scales
from scanner import (IDS, MotorControl, decode_status, empty_motor,
                     open_arm_control, status_payload, timestamp)


ZERO_PAYLOAD = bytes.fromhex('fffffffffffffffe')
STATUS_TIMEOUT = .06
SETTLE_SECONDS = .1
STILL_TOLERANCE_RAD = .01
ZERO_TOLERANCE_RAD = .005


class ZeroCalibrationControl(MotorControl):
    """A separate allowlist: the read-only scanner cannot send this command."""
    def __init__(self, *args, tx=None, **kwargs):
        self.tx = tx if tx is not None else []
        self.zero_attempts = set()
        super().__init__(*args, **kwargs)

    def send_std(self, arbitration_id, payload, *, pad_to_8=True):
        data = bytes(payload)
        is_status = arbitration_id == 0x7FF and data in {status_payload(i) for i in IDS}
        is_zero = type(arbitration_id) is int and arbitration_id in IDS and data == ZERO_PAYLOAD
        if not (is_status or is_zero):
            raise RuntimeError('标定仅允许状态查询与保存零点，已拦截其他电机命令')
        if is_zero:
            if arbitration_id in self.zero_attempts:
                raise RuntimeError('同一电机的写零位指令不能自动重发')
            self.zero_attempts.add(arbitration_id)
        entry = {'can_id': arbitration_id, 'data': data.hex(), 'at': timestamp(), 'sent': False}
        self.tx.append(entry)
        super().send_std(arbitration_id, data, pad_to_8=pad_to_8)
        entry['sent'] = True


class CalibrationStopped(Exception):
    pass


def check_cancel(cancel):
    if cancel.is_set():
        raise CalibrationStopped()


def query_status(control, arm, motor, cancel=None):
    if cancel is not None:
        check_cancel(cancel)
    for _ in range(512):
        if control.bus.recv(timeout=0) is None:
            break
    else:
        raise RuntimeError('接收队列持续繁忙，禁止写零位')
    motor['attempts'] += 1
    control.send_std(0x7FF, status_payload(motor['id']))
    deadline = time.monotonic() + STATUS_TIMEOUT
    samples = []
    while time.monotonic() < deadline:
        if cancel is not None:
            check_cancel(cancel)
        msg = control.bus.recv(timeout=min(.01, max(0, deadline - time.monotonic())))
        if msg is None:
            continue
        feedback = decode_status(msg, motor['id'], motor['position_scale'])
        if feedback is None:
            arm['unexpected_frames'] += 1
            raise RuntimeError(f"ID {motor['id']} 收到未匹配反馈，已停止标定")
        samples.append(feedback)
        motor['frames'] += 1
        motor['last'] = feedback
        motor['states'] = sorted(set(motor['states']) | {feedback['state']})
        if feedback['state'] != 0:
            reason = '电机仍处于使能状态，请支撑机械臂并停止控制、失能后再标定' if feedback['state'] == 1 else '电机状态异常'
            raise RuntimeError(f"ID {motor['id']}：{reason}（0x{feedback['state']:X}）")
        if abs(feedback['velocity_raw'] - 2047.5) > 2.5:
            raise RuntimeError(f"ID {motor['id']} 尚未静止，请固定零位姿态后重试")
    if not samples:
        raise TimeoutError(f"ID {motor['id']} 无状态反馈，无法确认标定结果")
    # Multiple replies can indicate duplicate CAN IDs. Do not select one and
    # accidentally calibrate multiple devices that share the same address.
    if len(samples) != 1:
        raise RuntimeError(f"ID {motor['id']} 一次查询收到多条反馈，请检查重复 ID 或其他总线通信")
    motor['hits'] += 1
    return samples[0]


def calibrate_zero(role, roles, cancel=None, emit=None):
    if not isinstance(role, str) or role not in roles:
        raise ValueError('写零位必须明确选择一条机械臂')
    cancel = cancel if cancel is not None else threading.Event()
    emit = emit or (lambda event: None)
    arm = {**roles[role], 'motors': [], 'tx': [], 'error': None,
           'cancelled': False, 'unexpected_frames': 0, 'device': None}
    for i in IDS:
        arm['motors'].append({**empty_motor(i), 'result': 'zero_pending',
                              'position_scale': None, 'before_samples': [], 'after_samples': [],
                              'prewrite': None, 'zero_command_attempted': False,
                              'zero_command_sent': False, 'verified': False, 'error': None})
    report = {'schema_version': 1, 'operation': 'zero', 'started_at': timestamp(),
              'status_requests_only': False, 'selected_roles': [role], 'motor_ids': list(IDS),
              'zero_tolerance_rad': ZERO_TOLERANCE_RAD,
              'still_tolerance_rad': STILL_TOLERANCE_RAD, 'arms': [arm],
              'note': '保存当前姿态为电机零位；成功表示写入后两次反馈接近零，未验证断电重启保持。停止不撤回已发出的写零位。',
              'power_cycle_verified': False}
    active = None

    def progress(motor, message):
        emit({'event': 'zero_progress', 'role': role, 'motor': deepcopy(motor),
              'message': f"{arm['name']} · ID {motor['id']} · {message}"})

    try:
        with exclusive_operation():
            check_cancel(cancel)
            scales = load_position_scales().get(arm['usb_serial'], {})
            if set(scales) != set(IDS):
                raise ValueError('所选臂缺少已核对的七电机量程，禁止写零位')
            for motor in arm['motors']:
                motor['position_scale'] = scales[motor['id']]
            with open_arm_control(arm, ZeroCalibrationControl) as control:
                # Check the entire arm twice before issuing any persistent write.
                for _ in range(2):
                    for motor in arm['motors']:
                        active = motor
                        motor['result'] = 'zero_checking'
                        feedback = query_status(control, arm, motor, cancel)
                        motor['before_samples'].append(feedback)
                        if len(motor['before_samples']) == 2:
                            if abs(feedback['position_rad'] - motor['before_samples'][0]['position_rad']) > STILL_TOLERANCE_RAD:
                                raise RuntimeError(f"ID {motor['id']} 姿态变化，请固定零位姿态后重试")
                            motor['result'] = 'zero_ready'
                        progress(motor, '检查通信、失能和静止状态')
                for motor in arm['motors']:
                    active = motor
                    check_cancel(cancel)
                    motor['prewrite'] = query_status(control, arm, motor, cancel)
                    if abs(motor['prewrite']['position_rad'] - motor['before_samples'][-1]['position_rad']) > STILL_TOLERANCE_RAD:
                        raise RuntimeError(f"ID {motor['id']} 写入前姿态发生变化，已停止标定")
                    check_cancel(cancel)
                    # Mark the attempt before sending: an IO error may happen
                    # after bytes reached the device. Never claim rollback.
                    motor['zero_command_attempted'] = True
                    motor['result'] = 'zero_unverified'
                    control.send_std(motor['id'], ZERO_PAYLOAD)
                    motor['zero_command_sent'] = True
                    progress(motor, '已发送写零位，正在回读')
                    time.sleep(SETTLE_SECONDS)
                    # Finish bounded verification of this motor even if Stop was
                    # clicked after its write. The next motor will not be written.
                    for _ in range(2):
                        feedback = query_status(control, arm, motor)
                        motor['after_samples'].append(feedback)
                        tolerance = max(ZERO_TOLERANCE_RAD, 2 * scales[motor['id']]['pmax_rad'] / 65535)
                        if abs(feedback['position_rad']) > tolerance:
                            raise RuntimeError(f"ID {motor['id']} 写入后位置未接近零，结果未确认；不会自动重写")
                    motor['verified'] = True
                    motor['result'] = 'zero_ok'
                    progress(motor, '零位写入并回读通过')
                # A later adapter-close failure is an arm-level error; it must
                # not relabel the last successfully verified motor as unknown.
                active = None
    except CalibrationStopped:
        arm['cancelled'] = True
    except Exception as exc:
        arm['error'] = f'{type(exc).__name__}: {exc}'
        if active:
            active['error'] = arm['error']
            active['result'] = 'zero_unverified' if active['zero_command_attempted'] else 'zero_failed'
    finally:
        arm['cancelled'] = arm['cancelled'] or cancel.is_set()
        for motor in arm['motors']:
            if motor['result'] in ('zero_pending', 'zero_checking', 'zero_ready'):
                motor['result'] = 'zero_not_written'
        arm['responding'] = sum(m['hits'] > 0 for m in arm['motors'])
        arm['written_ids'] = [m['id'] for m in arm['motors'] if m['zero_command_sent']]
        arm['uncertain_ids'] = [m['id'] for m in arm['motors'] if m['zero_command_attempted'] and not m['verified']]
        arm['verified_ids'] = [m['id'] for m in arm['motors'] if m['verified']]
        arm['not_written_ids'] = [m['id'] for m in arm['motors'] if not m['zero_command_attempted']]
        arm['ok'] = len(arm['verified_ids']) == 7 and not arm['error'] and not arm['cancelled']
        report.update(finished_at=timestamp(), cancelled=arm['cancelled'], ok=arm['ok'])
        emit({'event': 'arm_done', 'arm': arm})
    return report
