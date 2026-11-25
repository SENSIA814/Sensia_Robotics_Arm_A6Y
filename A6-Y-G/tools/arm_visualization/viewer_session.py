"""One selected arm, a bounded background reader, and isolated ROS display."""
from collections import deque
from contextlib import ExitStack
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import threading
import time
import uuid

from model import JOINTS, ROOT, load_model, load_roles
from feedback import complete_positions, read_cycle
from scanner import StatusOnlyControl, open_arm_control, timestamp
from operation_lock import exclusive_operation
from ros_display import RosDisplay

REPORTS = ROOT.parent / 'diagnostics/arm_rviz_reports'


class QueryAudit(deque):
    def __init__(self):
        super().__init__(maxlen=70)
        self.count = 0

    def append(self, value):
        self.count += 1
        super().append(value)


class ViewSession:
    def __init__(self, roles=None, report_dir=REPORTS, display_factory=RosDisplay):
        self.roles = roles if roles is not None else load_roles()
        self.report_dir = Path(report_dir)
        self.display_factory = display_factory
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.worker = None
        self.state = {'running': False, 'role': None, 'status': '请选择机械臂',
                      'error': None, 'rows': {}, 'preview': False, 'report': None}

    def update(self, **values):
        with self.lock:
            self.state.update(values)

    def snapshot(self):
        with self.lock:
            return deepcopy(self.state)

    def start(self, role, preview=False, with_rviz=True):
        if self.worker is not None and self.worker.is_alive():
            raise RuntimeError('请先停止当前显示，再切换机械臂')
        model = load_model(role, self.roles)
        self.cancel.clear()
        with self.lock:
            self.state = {'running': True, 'role': role, 'status': '正在启动显示',
                          'error': None, 'rows': {}, 'preview': preview, 'report': None,
                          'model': {k: v for k, v in model.items() if k != 'description'},
                          'started_at': timestamp(), 'publish_count': 0, 'cycles': 0,
                          'last_complete_at': None, 'last_complete_monotonic': None}
        self.worker = threading.Thread(target=self._run, args=(model, preview, with_rviz), daemon=False)
        self.worker.start()

    def stop(self):
        self.cancel.set()

    def _run(self, model, preview, with_rviz):
        tx = QueryAudit()
        directory = None
        try:
            self.report_dir.mkdir(parents=True, exist_ok=True)
            directory = Path(tempfile.mkdtemp(prefix=model['role'] + '_', dir=self.report_dir))
            namespace = '/a6y_check/' + model['role'] + '/s' + uuid.uuid4().hex[:10]
            self.update(directory=str(directory), namespace=namespace)
            with ExitStack() as stack:
                if not preview:
                    stack.enter_context(exclusive_operation())
                    arm = {**model['arm'], 'tx': tx}
                    control = stack.enter_context(open_arm_control(arm, StatusOnlyControl))
                display = stack.enter_context(self.display_factory(model, directory, namespace, with_rviz))
                count = cycles = 0
                while not self.cancel.is_set() and display.alive():
                    start = time.monotonic()
                    if preview:
                        display.publish([0.] * 7, 'URDF ZERO PREVIEW / NO HARDWARE')
                        self.update(status='URDF 零姿态预览 · 没有连接硬件', publish_count=count + 1)
                        count += 1
                        self.cancel.wait(.1)
                        continue
                    rows = read_cycle(control, model, self.cancel)
                    if self.cancel.is_set():
                        break
                    positions = complete_positions(rows)
                    cycles += 1
                    missing = [name for name in JOINTS if name not in rows or rows[name]['q'] is None]
                    faults = [name for name, row in rows.items()
                              if row['feedback'] and row['feedback']['state'] not in (0, 1)]
                    enabled = [name for name, row in rows.items()
                               if row['feedback'] and row['feedback']['state'] == 1]
                    limits = [name for name, row in rows.items() if row['out_of_limits']]
                    if positions is None:
                        status = '反馈缺失／超时，姿态已暂停：' + ', '.join(missing or ['采样过期'])
                        english = 'FEEDBACK LOST / POSE FROZEN'
                    else:
                        count += 1
                        status = '只读反馈 7 / 7 · 历史方向与比例待实机核对'
                        english = 'LIVE FEEDBACK 7/7 / MAPPING UNVERIFIED'
                        self.update(last_complete_at=timestamp(), last_complete_monotonic=time.monotonic())
                    if faults:
                        status += '；电机状态异常：' + ', '.join(faults)
                        english += '\nMOTOR FAULT'
                    if enabled:
                        status += '；检测到已使能：' + ', '.join(enabled)
                        english += '\nMOTOR ALREADY ENABLED'
                    if limits:
                        status += '；超出 URDF 范围：' + ', '.join(limits)
                        english += '\nOUTSIDE URDF LIMITS'
                    display.publish(positions, english, positions is None or bool(faults or enabled or limits))
                    self.update(rows=rows, status=status, cycles=cycles, publish_count=count,
                                tx_count=tx.count, cycle_hz=1 / max(.05, time.monotonic() - start))
                    self.cancel.wait(max(0, .05 - (time.monotonic() - start)))
            self.update(status='已停止 · 串口已释放，可返回零位标定')
        except Exception as exc:
            self.update(error=f'{type(exc).__name__}: {exc}', status='显示已停止，请检查错误信息')
        finally:
            self.update(finished_at=timestamp(), tx_count=tx.count, tx_tail=list(tx),
                        status_requests_only=True, calibration_verified=False)
            if directory is not None:
                path = directory / 'session.json'
                try:
                    self.update(report=str(path))
                    report = self.snapshot()
                    report['running'] = False
                    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
                except OSError as exc:
                    previous = self.snapshot()['error']
                    self.update(report=None, error=f'{previous or ""} 保存报告失败：{exc}')
            self.update(running=False)
