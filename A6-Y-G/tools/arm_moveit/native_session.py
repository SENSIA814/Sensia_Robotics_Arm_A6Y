"""UI orchestration only: MoveGroup -> ExecuteTrajectory -> ros2_control.

No trajectory interpolation, custom speed envelope, Python CAN writes, or
Pinocchio inverse kinematics are used in this entry point.
"""
from contextlib import ExitStack
from copy import deepcopy
import json
import os
import re
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import time
import traceback
import uuid

import numpy as np

from motion_model import ARM_JOINTS, JOINTS, ROOT, Kinematics
from native_model import native_model
from native_stack import NativeFeedback, NativeStack
from operation_lock import exclusive_operation
from usb_identity import verify_usb_identity
from pose_utils import pose_message, pose_matrix
from preview import ModelPreview
from telemetry import SessionRecorder
from gripper_policy import position_from_percent, percent_from_position, validate_trajectory


class Cancelled(RuntimeError):
    pass


class PlanningSession:
    def __init__(self, roles=None, report_dir=None):
        self.roles = roles
        self.report_dir = Path(report_dir or ROOT.parent / 'diagnostics/arm_moveit_native_reports')
        self.lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.home_queued = False
        self.cancel = threading.Event()
        self.commands = queue.Queue(maxsize=40)
        self.worker = self.runtime = self.current_goal = None
        self.recorder = None
        self.state = dict(running=False, ready=False, busy=False, mock=True, permitted=False,
                          status='请选择机械臂', error=None, warning='', events=[],
                          plan_ready=False, position_only=False, velocity_scaling=.1, acceleration_scaling=.1,
                          home_active=False, home_stage='', closing_requested=False, close_failed=False,
                          close_allowed=False, target_valid=False, preview_status='',
                          backend='MoveIt / FollowJointTrajectory / ros2_control (upstream port)')

    def update(self, **values):
        with self.lock:
            watched = ('status', 'error', 'warning', 'preview_status', 'permitted', 'plan_ready', 'home_stage',
                       'close_failed', 'collision_status', 'velocity_scaling', 'acceleration_scaling', 'position_only',
                       'target', 'gripper_target_mm', 'startup_home', 'home_on_start', 'gripper_limit_status')
            changes = {k: v for k, v in values.items() if k in watched and v != self.state.get(k)}
            if values.get('error') and values['error'] != self.state.get('error'):
                event = {'time': time.time(), 'error': values['error']}
                self.state['events'].append(event)
                self.state['events'] = self.state['events'][-200:]
            self.state.update(values)
        if changes:
            self.record('state', '；'.join(str(changes[k]) for k in ('error', 'warning', 'status', 'preview_status') if changes.get(k)), changes=changes)

    def record(self, kind, message='', **details):
        if self.recorder:
            self.recorder.event(kind, message, **details)

    def telemetry_snapshot(self, after=0):
        return self.recorder.snapshot(after) if self.recorder else {'error': '', 'dropped_events': 0, 'samples': []}

    def snapshot(self):
        with self.lock:
            result = deepcopy(self.state)
        if self.runtime:
            result['hardware'] = self.runtime.snapshot()
        started = result.get('action_started')
        result['wait_note'] = (f"{result.get('action_label', '')}：已等待 {time.monotonic()-started:.1f} 秒，"
                               f"超时上限 {result.get('action_timeout', 0):.1f} 秒") if started else ''
        return result

    def start(self, role, mock=True, with_rviz=True, collision_profile=None, home_on_start=False):
        if self.worker and self.worker.is_alive():
            raise ValueError('请先关闭当前会话')
        self.recorder = None
        self.model = native_model(role, self.roles, collision_profile)
        self.runtime = NativeFeedback(self.model, mock)
        self.startup_generation = self.runtime.generation()
        self.runtime.update(plot_relative=bool(home_on_start), plot_zero_pose=None, plot_zero_at=None)
        self.cancel.clear()
        self.commands = queue.Queue(maxsize=40)
        self.home_queued = False
        self.plan = self.plan_source = self.target = self.preview_q = None
        self.preview_future = self.preview_pending = None
        self.preview_group = 'arm'
        self.preview_revision, self.revision = -1, 0
        self.gripper_target = 0.
        self.display = self.node = self.executor = None
        self.current_goal = None
        self.update(running=True, ready=False, busy=False, mock=mock, role=role, permitted=False,
                    status='启动原版 MoveIt 与 ros2_control', error=None, events=[], directory=None,
                    report=None, plan_ready=False, target=None, target_valid=False, preview_status='', warning='',
                    home_active=False, home_stage='', home_result=None, closing_requested=False,
                    close_failed=False, close_allowed=False, velocity_scaling=.1, acceleration_scaling=.1,
                    position_only=False, action_started=None, cleanup_errors=[])
        self.update(home_on_start=bool(home_on_start), startup_home='pending' if home_on_start else 'disabled')
        from rclpy.utilities import get_rmw_implementation_identifier
        self.update(rmw=get_rmw_implementation_identifier())
        policy = self.model['collision_policy']
        limits = self.model['gripper_limits']
        self.update(collision_profile=policy['profile'],
                    collision_status=policy['label'] + '：' + policy['description'],
                    gripper_limit_status=f'夹爪电子限位：0% = {limits["closed_position_m"]*1000:.2f} mm，'
                                         f'100% = {limits["open_position_m"]*1000:.2f} mm（单指位置）')
        self.worker = threading.Thread(target=self._run, args=(with_rviz,), daemon=False)
        self.worker.start()

    def send(self, command, value=None):
        if not self.snapshot()['running']:
            return
        self.record('command', command, command=command, value=value)
        with self.command_lock:
            homing = command in ('home', 'release', 'shutdown')
            if homing and (self.home_queued or self.snapshot()['home_active']):
                if command == 'shutdown':
                    self.update(closing_requested=True)
                return
            if command == 'permit':
                self.update(permitted=bool(value))
            priority = homing or command in ('stop', 'emergency_release') or (command == 'permit' and not value)
            if priority:
                # Stop/Home/Close supersede queued movement, including old home
                # requests. They must never resume motion after a Stop click.
                while True:
                    try:
                        self.commands.get_nowait()
                    except queue.Empty:
                        break
                self.home_queued = False
                if not homing:
                    was_closing = self.snapshot()['closing_requested']
                    self.update(closing_requested=False,
                                close_failed=self.snapshot()['close_failed'] or was_closing)
                self.runtime.cancel_motion()
                self._stop_action()
            if homing:
                self.home_queued = True
                self.update(closing_requested=command == 'shutdown', close_failed=False)
            if homing or command in ('execute', 'execute_arm', 'execute_gripper'):
                value = self.runtime.generation()
            try:
                self.commands.put_nowait((command, value))
            except queue.Full:
                self.update(error='操作队列繁忙，请稍后重试')

    def stop(self):
        self.send('shutdown')

    def abort(self):
        self.cancel.set()
        if self.runtime:
            self.runtime.cancel_motion()
            self._stop_action()

    def _stop_action(self):
        if self.display and self.display.context and not self.display.context.ok():
            return
        errors = []
        try:
            if self.current_goal:
                self.current_goal.cancel_goal_async()
        except Exception as exc:
            errors.append(str(exc))
        try:
            if getattr(self, 'stop_pub', None):
                from std_msgs.msg import String
                self.stop_pub.publish(String(data='stop'))
        except Exception as exc:
            errors.append(str(exc))
        if errors:
            self.update(warning='停止消息发送异常：' + '；'.join(errors))
            if self.runtime and self.runtime.snapshot()['executing']:
                self.runtime.update(error='停止请求未确认；禁止继续执行，请检查控制器或紧急失能')

    def _spin(self):
        self.executor.spin_once(timeout_sec=.01)
        if time.monotonic() - getattr(self, 'published', 0.) > .1:
            self._publish_ui()
            self.published = time.monotonic()

    def _wait(self, predicate, timeout, generation=None):
        deadline = time.monotonic() + timeout
        while not predicate():
            if self.cancel.is_set() or (generation is not None and generation != self.runtime.generation()):
                raise Cancelled('操作已取消，未继续执行')
            if time.monotonic() > deadline:
                raise TimeoutError('等待 ROS 响应超时，请查看本次日志')
            self.display.check()
            self._spin()
        if generation is not None and generation != self.runtime.generation():
            raise Cancelled('操作已取消，已丢弃迟到结果')

    def _future(self, future, timeout=20, generation=None):
        self._wait(future.done, timeout, generation)
        return future.result()

    def _run(self, with_rviz):
        folder = None
        try:
            from rclpy.executors import SingleThreadedExecutor
            self.report_dir.mkdir(parents=True, exist_ok=True)
            folder = Path(tempfile.mkdtemp(prefix=self.model['role'] + '_', dir=self.report_dir))
            self.namespace = '/a6y_native/' + self.model['role'] + '/s' + uuid.uuid4().hex[:8]
            self.update(directory=str(folder), namespace=self.namespace)
            self.recorder = SessionRecorder(folder, self.model, self.runtime)
            self.record('session_start', '启动会话', role=self.model['role'], mock=self.runtime.mock,
                        namespace=self.namespace, collision_policy=self.model['collision_policy'],
                        home_on_start=self.snapshot()['home_on_start'])
            with ExitStack() as ownership:
                if not self.runtime.mock:
                    ownership.enter_context(exclusive_operation())
                    self._verify_device()
                self.display = NativeStack(self.model, folder, self.namespace, self.runtime.mock, with_rviz)
                try:
                    self.display.start()
                    self.node = self.display.node
                    self.executor = SingleThreadedExecutor(context=self.display.context)
                    self.executor.add_node(self.node)
                    self._setup_ros()
                    self.display.pump = self._spin
                    self._controllers_ready(active=False)
                    self._wait(lambda: self.plan_client.server_is_ready() and self.execute_client.server_is_ready() and
                               self.ik_client.service_is_ready() and self.runtime.snapshot()['q'] is not None, 35)
                    self.reset_target()
                    self.runtime.update(status='ros2_control 只读反馈（正常）；点击执行时自动激活轨迹控制器')
                    self.update(ready=True, busy=self.snapshot()['home_on_start'],
                                status='原版 MoveIt 已就绪；拖动目标，规划后执行')
                    self._startup_home()
                    while not self.cancel.is_set():
                        self.display.check()
                        self._spin()
                        self._poll_preview()
                        if self.display.close_requested and not self.snapshot()['closing_requested']:
                            self.display.close_requested = False
                            self.send('shutdown')
                        try:
                            command, value = self.commands.get_nowait()
                            command_group = ('gripper' if command in ('gripper_target', 'gripper_plan', 'execute_gripper') else
                                             self.snapshot().get('plan_group') if command == 'execute' else None)
                            try:
                                self._command(command, value)
                                self.record('command_result', '操作处理完成', command=command, group=command_group)
                            finally:
                                if command in ('home', 'release', 'shutdown'):
                                    with self.command_lock:
                                        self.home_queued = False
                        except queue.Empty:
                            pass
                        except Exception as exc:
                            self.record('command_result', str(exc), command=command, success=False,
                                        group=command_group, exception=type(exc).__name__, traceback=traceback.format_exc())
                            self._discard_plan()
                            self.update(busy=False, error=str(exc), status='操作未完成；会话保留，请检查错误')
                            if command == 'shutdown':
                                self.update(close_failed=True, closing_requested=False, close_allowed=False)
                        if self.plan is not None and time.monotonic() - self.plan_created > 30:
                            self._discard_plan()
                            self.update(status='规划已超过 30 秒，请重新规划')
                finally:
                    self._cleanup()
        except Exception as exc:
            self.record('session_error', str(exc), traceback=traceback.format_exc())
            self.update(error=f'{type(exc).__name__}: {exc}', status='ROS 会话启动或进程异常，请查看日志')
            if self.snapshot()['closing_requested']:
                self.update(close_failed=True, closing_requested=False)
        finally:
            self.update(running=False, ready=False, busy=False, plan_ready=False, permitted=False, home_active=False)
            if folder:
                path = folder / 'session.json'
                self.update(report=str(path))
                data = self.snapshot()
                data['model'] = {k: v for k, v in self.model.items() if k not in ('description', 'semantic')}
                try:
                    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
                except OSError as exc:
                    self.update(warning=f'会话报告写入失败：{exc}')
            if self.recorder:
                self.record('session_end', '会话已关闭', process_exit_codes=self.snapshot().get('process_exit_codes'),
                            error=self.snapshot().get('error'))
                self.recorder.close()

    def _cleanup(self):
        # Each cleanup step is independent: an invalid ROS publisher must not
        # skip process cleanup and leave an unclosable orphan RViz behind.
        failures = []
        def attempt(label, operation):
            try:
                operation()
            except Exception as exc:
                failures.append(f'{label}: {exc}')
        attempt('cancel action', self._stop_action)
        if self.display:
            self.update(busy=True, status='正在关闭 ROS 子进程，最长等待 6 秒')
            attempt('native processes', lambda: self.display.terminate(self.display.children, self.display.pump))
        if getattr(self, 'marker_server', None):
            attempt('interactive markers', self.marker_server.shutdown)
        if self.executor:
            attempt('executor', lambda: self.executor.shutdown(timeout_sec=1.))
        if self.display:
            attempt('ROS resources', self.display.close)
        codes = {name: child.poll() for name, child in self.display.children} if self.display else {}
        self.update(cleanup_errors=failures, process_exit_codes=codes)

    def _startup_home(self):
        if not self.snapshot().get('home_on_start'):
            return
        with self.command_lock:
            if self.cancel.is_set() or self.runtime.generation() != self.startup_generation or self.snapshot()['closing_requested']:
                self.update(startup_home='cancelled', busy=False, status='启动回零已取消，等待下一步指令')
                return
            # The labelled Start-and-home action authorizes this one automatic
            # home. A prior Stop/revocation must never be overridden.
            self.update(permitted=True, startup_home='running', busy=True)
        self.record('startup_home', '启动自动规划回零并保持', generation=self.startup_generation)
        try:
            self._home(generation=self.startup_generation)
        except Exception as exc:
            # Includes a Stop racing the initial generation/permission checks.
            # Keep the session and its pending Stop/Close command alive.
            self.update(startup_home='failed', busy=False, error=str(exc),
                        status='启动回零未完成；会话保留，不自动重试')
            return
        state = self.snapshot()
        if state['close_allowed']:
            self.update(startup_home='cancelled')
        elif state['home_stage'] == 'complete':
            self.update(startup_home='complete', status='启动回零完成 · 已保持零位，等待下一步指令')
        else:
            self.update(startup_home='failed', status='启动回零未完成；会话保留，不自动重试')

    def _set_plot_zero(self, q):
        if self.runtime.snapshot().get('plot_zero_pose') is not None:
            return
        pose = self.kin.fk(q).tolist()
        stamp = time.time()
        self.runtime.update(plot_relative=True, plot_zero_pose=pose, plot_zero_at=stamp)
        self.record('plot_zero', '曲线 XYZ/RPY 参考已设为本次回零后的末端位姿',
                    reference_pose=pose, joint_positions=q.tolist(), reference_time=stamp)

    def _verify_device(self):
        verify_usb_identity(self.model['arm'])
        port = self.model['arm']['port']
        result = subprocess.run(['fuser', port], capture_output=True, timeout=3)
        if result.returncode == 0:
            raise ValueError('所选机械臂串口已被占用，请停止其他控制程序')
        if not Path(port).exists():
            raise ValueError('所选机械臂 USB 串口不存在')

    def _setup_ros(self):
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup, ExecuteTrajectory
        from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetStateValidity
        from moveit_msgs.msg import DisplayTrajectory
        from controller_manager_msgs.srv import ListControllers
        from sensor_msgs.msg import JointState
        from control_msgs.msg import DynamicJointState
        from control_msgs.action import FollowJointTrajectory
        from std_msgs.msg import String
        from interactive_markers.interactive_marker_server import InteractiveMarkerServer
        self.plan_client = ActionClient(self.node, MoveGroup, 'move_action')
        self.execute_client = ActionClient(self.node, ExecuteTrajectory, 'execute_trajectory')
        self.controller_actions = [ActionClient(self.node, FollowJointTrajectory, name + '/follow_joint_trajectory')
                                   for name in ('arm_controller', 'gripper_controller')]
        self.ik_client = self.node.create_client(GetPositionIK, 'compute_ik')
        self.preview_plan_client = self.node.create_client(GetMotionPlan, 'plan_kinematic_path')
        self.valid_client = self.node.create_client(GetStateValidity, 'check_state_validity')
        self.controllers = None
        self.controller_service_name = None
        self.node.create_subscription(JointState, 'joint_states', self.runtime.joint_states, 10)
        self.node.create_subscription(DynamicJointState, 'dynamic_joint_states', self.runtime.diagnostics, 10)
        self.ui_pub = self.node.create_publisher(String, 'ui/status', 1)
        self.stop_pub = self.node.create_publisher(String, 'trajectory_execution_event', 10)
        self.path_pub = self.node.create_publisher(DisplayTrajectory, 'display_planned_path', 1)
        self.node.create_subscription(String, 'ui/command', self._ui_command, 10)
        self.marker_server = InteractiveMarkerServer(self.node, 'tcp_target')
        self.kin = Kinematics(self.model)  # FK/model rendering only.
        self.visual = ModelPreview(self.node, self.model)

    def _controllers_ready(self, active):
        from controller_manager_msgs.srv import ListControllers
        name = self.display.manager_name + '/list_controllers'
        if self.controller_service_name != name:
            if self.controllers:
                self.node.destroy_client(self.controllers)
            self.controllers = self.node.create_client(ListControllers, name)
            self.controller_service_name = name
        self._wait(self.controllers.service_is_ready, 30)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            result = self._future(self.controllers.call_async(ListControllers.Request()), 5)
            states = {c.name: c.state for c in result.controller}
            expected = 'active' if active else 'inactive'
            if states.get('joint_state_broadcaster') == 'active' and all(states.get(n) == expected for n in ('arm_controller', 'gripper_controller')):
                self.update(controllers=states)
                return
            self._spin()
        raise RuntimeError('机械臂／夹爪轨迹控制器未按预期就绪')

    def _switch_control(self, enabled, generation=None):
        if enabled and generation is not None and generation != self.runtime.generation():
            raise Cancelled('接管请求已取消，未使能电机')
        hardware = self.runtime.snapshot()
        if hardware['enabled'] == enabled and (hardware['control_ready'] if enabled else hardware['release_confirmed']):
            return
        initial = self.runtime.measured() if enabled else self.runtime.snapshot()['q']
        self.update(busy=True, status='准备原版位置控制器' if enabled else '释放位置控制器并恢复只读反馈')
        self.display.stop_control()  # Original driver deactivation/disable path.
        self.runtime.update(q=None, velocity=None, sample_time=None, diagnostic_time=None, rows={}, control_ready=False)
        if not self.runtime.mock:
            self._verify_device()
        if enabled and generation is not None and generation != self.runtime.generation():
            raise Cancelled('接管准备期间已取消，未启动使能控制器')
        # Original auto_enable is a startup parameter; restart only controller
        # manager, keeping MoveIt/RViz alive. No new control-loop gate is added.
        if enabled:
            self.runtime.update(enabled=True, release_confirmed=False, status='接管进行中；使能状态待回读')
        self.display.start_control(enabled, initial)
        self._controllers_ready(active=enabled)
        if enabled:
            self._wait(lambda: all(c.server_is_ready() for c in self.controller_actions), 15)
            # Like the upstream bringup's delayed move_group start, allow DDS
            # clients in the already-running move_group to discover new servers.
            ready_at = time.monotonic() + 1.
            self._wait(lambda: time.monotonic() >= ready_at, 2)
        self._wait(lambda: self.runtime.snapshot()['q'] is not None, 15)
        self.runtime.measured()
        if not enabled and not self.runtime.mock:
            self._wait(lambda: all(self.runtime.snapshot()['rows'].get(n, {}).get('motor_error') == 0. for n in JOINTS), 5)
        self.runtime.update(enabled=enabled, release_confirmed=not enabled, control_ready=enabled, error=None,
                            status='原版位置控制已激活' if enabled else '已确认失能 · ros2_control 只读反馈')
        if generation is not None and generation != self.runtime.generation():
            raise Cancelled('准备期间收到停止请求，轨迹没有提交')

    def _ui_command(self, message):
        try:
            data = json.loads(message.data)
            self.send(data['command'], data.get('value'))
        except Exception as exc:
            self.update(error=str(exc))

    def _publish_ui(self):
        if not hasattr(self, 'ui_pub'):
            return
        from std_msgs.msg import String
        s = self.snapshot()
        h = s['hardware']
        payload = {k: v for k, v in s.items() if k not in ('hardware', 'events', 'target')}
        payload.update(role=self.model['arm']['name'], executing=h['executing'], enabled=h['enabled'],
                       hardware_status=h['status'], gripper_current_mm=None if h['q'] is None else h['q'][6] * 1000.)
        self.ui_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if time.monotonic() - getattr(self, 'last_status_write', 0.) >= 1.:
            self.last_status_write = time.monotonic()
            try:
                path = Path(s['directory']) / 'live_status.json'
                pending = path.with_suffix('.tmp')
                pending.write_text(json.dumps(dict(s, backend_pid=os.getpid(), updated_at=time.time()), ensure_ascii=False))
                pending.replace(path)
            except OSError as exc:
                self.update(warning=f'状态快照写入失败：{exc}')
        self.visual.publish(self.preview_q, self.target, s['target_valid'], None, None)

    def _discard_plan(self):
        self.plan = None
        self.update(plan_ready=False)

    def _command(self, command, value):
        if command == 'permit':
            self.update(permitted=bool(value))
            return
        if command in ('home', 'release', 'shutdown'):
            self._home(close=command == 'shutdown' or self.snapshot()['closing_requested'], generation=value)
            return
        if command in ('stop', 'emergency_release'):
            self._discard_plan()
            if command == 'emergency_release':
                self._switch_control(False)
                self.update(permitted=False)
            self.update(busy=False, status='已停止轨迹，控制器保持当前位置' if command == 'stop' else '已请求失能，未执行回零')
            return
        if not self.snapshot()['ready']:
            raise ValueError('等待 MoveIt 和 ros2_control 就绪')
        if command == 'reset':
            self.reset_target()
        elif command == 'target':
            self._set_target(np.asarray(value, float))
            self._make_marker()
        elif command == 'settings':
            self.set_settings(value)
        elif command == 'gripper_target':
            self.set_gripper_target(value)
        elif command in ('plan', 'gripper_plan'):
            self.set_settings(value or {})
            self._plan('gripper' if command == 'gripper_plan' else 'arm')
        elif command in ('execute', 'execute_arm', 'execute_gripper'):
            if command != 'execute' and self.snapshot().get('plan_group') != command[8:]:
                raise ValueError('请先规划对应的机械臂／夹爪轨迹')
            self._execute(value)
        else:
            raise ValueError('未知操作：' + command)

    def set_settings(self, values):
        current = self.snapshot()
        changed = {}
        for key in ('velocity_scaling', 'acceleration_scaling'):
            value = float(values.get(key, current[key]))
            if not np.isfinite(value) or not .01 <= value <= 1:
                raise ValueError('速度／加速度比例必须在 1%～100%')
            if value != current[key]:
                changed[key] = value
        mode = values.get('position_only', current['position_only'])
        if not isinstance(mode, bool):
            raise ValueError('仅位置参数必须为布尔值')
        if mode != current['position_only']:
            changed['position_only'] = mode
        if changed:
            self._discard_plan()
            self.update(**changed)
            if 'position_only' in changed and self.target is not None:
                self._set_target(self.target)
                self._make_marker()

    def reset_target(self):
        q = self.runtime.measured()
        self.set_gripper_target(float(np.clip(percent_from_position(self.model['gripper_limits'], q[6]), 0, 100)),
                                source='feedback_sync')
        self._set_target(self.kin.fk(q))
        self._make_marker()

    def set_gripper_target(self, percent, source='user'):
        percent = float(percent)
        if not np.isfinite(percent) or not 0 <= percent <= 100:
            raise ValueError('夹爪开度需在 0～100%')
        self.gripper_target = position_from_percent(self.model['gripper_limits'], percent)
        self._discard_plan()
        self.revision += 1
        self.preview_group = 'gripper'
        self.preview_pending = self.revision
        self.preview_q = self.runtime.measured()
        self.preview_q[6] = self.gripper_target
        self.update(gripper_percent=percent, gripper_target_mm=1000*self.gripper_target,
                    target_valid=False, preview_status='正在检查夹爪目标碰撞')
        self.runtime.update(gripper_target_percent=percent, gripper_target_mm=1000*self.gripper_target)
        label = '闭合目标' if percent == 0 else '张开目标' if percent == 100 else '中间开度目标'
        self.record('gripper_target', f'夹爪{label} {percent:.1f}%（仅设置目标，未执行）',
                    group='gripper', target_percent=percent, target_mm=1000*self.gripper_target, source=source,
                    electronic_limits=self.model['gripper_limits'])

    def _set_target(self, target):
        if self.runtime.snapshot()['executing']:
            raise ValueError('执行期间请先停止，再修改目标')
        if target.shape != (4, 4) or not np.isfinite(target).all():
            raise ValueError('末端目标不是有限 4×4 位姿')
        self.target = target.copy()
        self.preview_group = 'arm'
        self.revision += 1
        self._discard_plan()
        self.preview_pending = self.revision
        self.update(target=target.tolist(), target_valid=False, error=None, preview_status='MoveIt CachedKDL 正在检查目标…')

    def _state_msg(self, q):
        from moveit_msgs.msg import RobotState
        state = RobotState()
        state.joint_state.name, state.joint_state.position = list(JOINTS), list(map(float, q))
        return state

    def _request(self, group, joint_goal=None, position_only=False, planning_time=5.):
        from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionConstraint
        from shape_msgs.msg import SolidPrimitive
        request = MotionPlanRequest()
        s = self.snapshot()
        request.group_name, request.pipeline_id = group, 'ompl'
        request.allowed_planning_time = planning_time
        request.num_planning_attempts = 1  # MoveGroupInterface default in the reference.
        request.max_velocity_scaling_factor = s['velocity_scaling']
        request.max_acceleration_scaling_factor = s['acceleration_scaling']
        request.start_state = self._state_msg(self.runtime.measured())
        constraint = Constraints()
        if position_only:
            pos = PositionConstraint()
            pos.header.frame_id, pos.link_name, pos.weight = 'base_link', 'tool_tcp', 1.
            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.0002]*3)
            pose = pose_message(self.target)
            pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.
            pose.orientation.w = 1.
            pos.constraint_region.primitives = [box]
            pos.constraint_region.primitive_poses = [pose]
            constraint.position_constraints = [pos]
        else:
            for i in (range(6) if group == 'arm' else [6]):
                c = JointConstraint(joint_name=JOINTS[i], position=float(joint_goal[i]),
                                    tolerance_above=.0001, tolerance_below=.0001, weight=1.)
                if group == 'gripper':
                    limits = self.model['gripper_limits']
                    c.tolerance_above = max(0., min(.00001, limits['closed_position_m']-c.position))
                    c.tolerance_below = max(0., min(.00001, c.position-limits['open_position_m']))
                constraint.joint_constraints.append(c)
        request.goal_constraints = [constraint]
        return request

    def _poll_preview(self):
        from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetStateValidity
        if self.preview_future is not None and time.monotonic() > self.preview_deadline:
            self.preview_future.cancel()
            self.preview_future = None
            self.preview_revision = self.preview_work
            self.update(target_valid=False, preview_status='MoveIt 目标检查超时，可重新拖动目标')
        if self.preview_future is not None and self.preview_future.done():
            future, self.preview_future = self.preview_future, None
            if self.preview_work == self.revision:
                try:
                    result = future.result()
                    if self.preview_work_group == 'gripper':
                        if not result.valid:
                            raise ValueError('夹爪目标发生碰撞')
                        names, positions = list(JOINTS), self.preview_gripper_q
                    elif self.preview_position_only:
                        result = result.motion_plan_response
                        if result.error_code.val != 1 or not result.trajectory.joint_trajectory.points:
                            raise ValueError(f'MoveIt 位置目标不可达或碰撞（{result.error_code.val}）')
                        names = result.trajectory.joint_trajectory.joint_names
                        positions = result.trajectory.joint_trajectory.points[-1].positions
                    else:
                        if result.error_code.val != 1:
                            raise ValueError(f'MoveIt IK 无解或目标碰撞（{result.error_code.val}）')
                        names, positions = result.solution.joint_state.name, result.solution.joint_state.position
                    q = self.runtime.measured()
                    solution = dict(zip(names, positions))
                    for i, n in enumerate(JOINTS):
                        q[i] = solution.get(n, q[i])
                    self.preview_q = q
                    self.preview_revision = self.revision
                    self.update(target_valid=True, preview_q=q.tolist(), preview_status='绿色模型：MoveIt 逆解与当前碰撞配置检查通过')
                except Exception as exc:
                    self.preview_revision = self.revision
                    self.update(target_valid=False, preview_status=str(exc))
        if self.preview_future is None and self.preview_pending is not None:
            self.preview_work, self.preview_pending = self.preview_pending, None
            try:
                # Let the original IK/planning adapters enforce model bounds;
                # do not apply the old Python backend's extra boundary gate.
                q = self.runtime.measured()
                self.preview_position_only = self.snapshot()['position_only']
                self.preview_work_group = self.preview_group
                self.preview_deadline = time.monotonic() + 4
                if self.preview_group == 'gripper':
                    q[6] = self.gripper_target
                    self.preview_gripper_q = q.copy()
                    req = GetStateValidity.Request(robot_state=self._state_msg(q))
                    self.preview_future = self.valid_client.call_async(req)
                elif self.preview_position_only:
                    req = GetMotionPlan.Request()
                    req.motion_plan_request = self._request('arm', position_only=True, planning_time=1.)
                    self.preview_future = self.preview_plan_client.call_async(req)
                else:
                    req = GetPositionIK.Request()
                    req.ik_request.group_name, req.ik_request.ik_link_name = 'arm', 'tool_tcp'
                    req.ik_request.pose_stamped.header.frame_id = 'base_link'
                    req.ik_request.pose_stamped.pose = pose_message(self.target)
                    if self.preview_q is not None:
                        q[:6] = self.preview_q[:6]
                    req.ik_request.robot_state = self._state_msg(q)
                    req.ik_request.avoid_collisions = True
                    req.ik_request.timeout.nanosec = 5000000
                    self.preview_future = self.ik_client.call_async(req)
            except Exception as exc:
                self.preview_revision = self.revision
                self.update(target_valid=False, preview_status=str(exc))

    def _action(self, client, goal, timeout, generation):
        if generation != self.runtime.generation():
            raise Cancelled('操作已取消，没有提交新目标')
        future = client.send_goal_async(goal)
        # Keep the acceptance future until resolved so cancellation cannot lose
        # a goal that the server accepted concurrently with a Stop click.
        handle = self._future(future, 10)
        if not handle.accepted:
            raise RuntimeError('MoveIt 拒绝了本次请求')
        self.current_goal = handle
        result = handle.get_result_async()
        self.update(action_started=time.monotonic(), action_timeout=timeout,
                    action_label='等待 MoveIt 执行结果' if client is self.execute_client else '等待 MoveIt 规划结果')
        try:
            self._wait(result.done, timeout, generation)
            return result.result()
        except Exception:
            handle.cancel_goal_async()
            self._stop_action()
            try:
                self._wait(result.done, 8)
            except Exception:
                if client is self.execute_client:
                    self.runtime.update(error='原生动作取消结果未确认；禁止再次执行或自动回零，请检查控制器并紧急失能')
                raise
            raise
        finally:
            self.current_goal = None
            self.update(action_started=None)

    def _plan(self, group='arm', joint_goal=None, generation=None):
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import DisplayTrajectory
        generation = self.runtime.generation() if generation is None else generation
        self.update(busy=True, error=None, status='MoveIt 原生规划与时间参数化')
        if joint_goal is None:
            if group == 'arm':
                if self.preview_group != 'arm':
                    self._set_target(self.target)
                deadline = time.monotonic() + 8
                while self.preview_revision != self.revision and time.monotonic() < deadline:
                    self._poll_preview()
                    self._spin()
                if not self.snapshot()['target_valid']:
                    raise ValueError(self.snapshot()['preview_status'])
                joint_goal = self.preview_q.copy()
            else:
                joint_goal = self.runtime.measured()
                joint_goal[6] = self.gripper_target
        revision = self.revision
        goal = MoveGroup.Goal()
        goal.request = self._request(group, joint_goal)
        goal.planning_options.plan_only = True
        self.plan_source = self.runtime.measured()
        self._discard_plan()
        response = self._action(self.plan_client, goal, 20, generation)
        if revision != self.revision:
            raise Cancelled('目标已改变，旧规划已丢弃')
        result = response.result
        self.record('plan_result', 'MoveIt 规划返回', group=group, action_status=response.status,
                    moveit_error_code=result.error_code.val, target=joint_goal,
                    start=self.plan_source, velocity_scaling=self.snapshot()['velocity_scaling'],
                    acceleration_scaling=self.snapshot()['acceleration_scaling'])
        if response.status != 4 or result.error_code.val != 1:
            raise ValueError(self._planning_error(result.error_code.val))
        trajectory = result.planned_trajectory
        if not trajectory.joint_trajectory.points:
            raise ValueError('MoveIt 返回空轨迹')
        if group == 'gripper':
            validate_trajectory(self.model['gripper_limits'], trajectory.joint_trajectory, self.plan_source[6])
        self.plan, self.plan_start = trajectory, result.trajectory_start
        self.plan_created = time.monotonic()
        last = trajectory.joint_trajectory.points[-1].time_from_start
        duration = last.sec + last.nanosec * 1e-9
        display = DisplayTrajectory(model_id=self.kin.model.name, trajectory_start=self.plan_start, trajectory=[self.plan])
        self.path_pub.publish(display)
        from rosidl_runtime_py.convert import message_to_ordereddict
        (Path(self.snapshot()['directory']) / 'last_native_plan.json').write_text(json.dumps(message_to_ordereddict(self.plan), indent=2))
        archive = f'plan_{time.time_ns()}_{group}.json'
        self.record('plan_saved', '规划通过，轨迹已保存', group=group, trajectory_file=archive,
                    points=len(trajectory.joint_trajectory.points), duration_s=duration,
                    trajectory=message_to_ordereddict(self.plan))
        self.update(busy=False, plan_ready=True, plan_group=group, duration=duration,
                    status=f'MoveIt 规划通过 · 轨迹预计用时 {duration:.2f} 秒；预览后点击执行')

    def _execute(self, generation=None):
        from moveit_msgs.action import ExecuteTrajectory
        generation = self.runtime.generation() if generation is None else generation
        state = self.snapshot()
        if self.plan is None or not state['plan_ready']:
            raise ValueError('请先规划并预览')
        if not state['mock'] and not state['permitted']:
            raise ValueError('请先核对机械臂并授予真机执行许可')
        if generation != self.runtime.generation():
            raise Cancelled('执行请求已取消')
        plan = deepcopy(self.plan)
        duration = state['duration']
        if time.monotonic() - self.plan_created > 30:
            raise ValueError('规划已过期，请重新规划')
        before = self.runtime.measured()
        if np.any(np.abs(before - self.plan_source) > np.array([.01]*6 + [.0005])):
            raise ValueError('反馈已偏离规划起点，请重新规划')
        if state.get('plan_group') == 'gripper':
            validate_trajectory(self.model['gripper_limits'], plan.joint_trajectory, self.plan_source[6])
        self._switch_control(True, generation)
        # Collision checks were performed by the native planner. MoveIt's
        # execution manager validates the start; pass its trajectory unchanged.
        before = self.runtime.measured()
        if np.any(np.abs(before - self.plan_source) > np.array([.01]*6 + [.0005])):
            raise ValueError('接管后反馈变化，请重新规划')
        self._discard_plan()
        self.runtime.update(executing=True, execution_group=state.get('plan_group', ''),
                            status='MoveIt → FollowJointTrajectory → ros2_control 执行中')
        self.update(busy=True, error=None, status='标准轨迹控制器执行中')
        goal = ExecuteTrajectory.Goal(trajectory=plan)
        if state.get('plan_group') == 'gripper':
            self.record('gripper_execution_start', f'开始执行夹爪轨迹，目标开度 {state["gripper_percent"]:.1f}%',
                        group='gripper', target_percent=state['gripper_percent'], duration_s=duration)
        try:
            response = self._action(self.execute_client, goal, max(15., duration * 2 + 10), generation)
            self.record('execute_result', 'MoveIt 执行返回', group=state.get('plan_group'),
                        action_status=response.status, moveit_error_code=response.result.error_code.val)
            if response.status != 4 or response.result.error_code.val != 1:
                raise ValueError(f'MoveIt 执行未成功（{response.result.error_code.val}）；控制器结果见日志')
            self.runtime.update(completed=self.runtime.snapshot()['completed'] + 1, status='轨迹完成 · 原版位置控制器保持')
            self.update(status='执行完成', busy=False)
            if state.get('plan_group') == 'gripper':
                self.record('gripper_execution_complete', '夹爪轨迹执行完成（MoveIt 返回成功）',
                            group='gripper', target_percent=state['gripper_percent'], moveit_error_code=1)
        except Exception as exc:
            if state.get('plan_group') == 'gripper':
                self.record('gripper_execution_failed', '夹爪未完成：'+str(exc), group='gripper',
                            target_percent=state['gripper_percent'])
            raise
        finally:
            self.runtime.update(executing=False, execution_group='')
            self.update(busy=False)

    def _planning_error(self, code):
        if code == -2:
            pairs = []
            path = Path(self.snapshot()['directory']) / 'move_group.log'
            if path.exists():
                with path.open('rb') as stream:
                    stream.seek(max(0, path.stat().st_size - 16000))
                    pairs = re.findall(r"contact between '([^']+)' .*? and '([^']+)'", stream.read().decode(errors='replace'))
            detail = '；碰撞连杆：' + '、'.join(sorted({a+' ↔ '+b for a, b in pairs})) if pairs else ''
            self.record('collision', '规划后复核失败'+detail, moveit_error_code=code,
                        pairs=sorted({tuple(pair) for pair in pairs}), source='move_group.log recent tail')
            return 'MoveIt 轨迹未通过规划后复核（-2），本次动作已取消' + detail
        return f'MoveIt 规划失败（{code}），本次动作已取消；请检查目标与碰撞'

    def _home(self, close=False, generation=None):
        state = self.snapshot()
        generation = self.runtime.generation() if generation is None else generation
        if generation != self.runtime.generation():
            raise Cancelled('回零请求已取消')
        hardware = state['hardware']
        # Permission does not mean we acquired the motors. Closing an unowned/released session must not
        # initiate motion, including after a failed plan with permission set.
        if close and not hardware['enabled'] and not hardware['executing'] and hardware['release_confirmed']:
            self._discard_plan()
            self.update(close_allowed=True, close_failed=False, closing_requested=True,
                        home_active=False, home_stage='not_required', busy=False,
                        status='关闭只读会话：未接管电机或已确认释放，无需回零')
            self.cancel.set()
            return
        if not state['mock'] and not state['permitted']:
            raise ValueError('回零需要真机执行许可；不能回零时可支撑后紧急失能')
        self.update(home_active=True, home_stage='planning', closing_requested=close, close_failed=False,
                    close_allowed=False, busy=True, error=None)
        try:
            q = self.runtime.measured()
            if np.any(np.abs(q[:6]) > .005):
                goal = q.copy()
                goal[:6] = 0.
                self.target = self.kin.fk(goal)
                self.preview_q = goal.copy()
                self.revision += 1
                self.preview_pending = None
                self.update(target=self.target.tolist(), target_valid=True, preview_status='关节零位目标，由 MoveIt 检查路径')
                self._make_marker()
                self._plan('arm', joint_goal=goal, generation=generation)
                self.update(home_stage='preview', status='回零规划通过，1 秒后执行；可点击停止取消')
                until = time.monotonic() + 1
                self._wait(lambda: time.monotonic() >= until, 2, generation)
                self.update(home_stage='executing')
                self._execute(generation)
                self._wait(lambda: np.all(np.abs(self.runtime.measured()[:6]) <= .005), 5, generation)
            elif not close:
                # At zero already: acquire position hold without moving to a
                # different pose. Closing a read-only session still bypasses this.
                self._switch_control(True, generation)
                self._wait(lambda: np.all(np.abs(self.runtime.measured()[:6]) <= .005), 5, generation)
            self.update(home_result={'joint_positions': self.runtime.measured().tolist(), 'arm_tolerance_rad': .005, 'at': time.time()},
                        home_stage='complete', busy=False, status='J1～J6 已回零，夹爪保持原开度')
            if self.snapshot()['closing_requested']:
                self.update(home_stage='releasing')
                self._switch_control(False)
                if generation != self.runtime.generation():
                    raise Cancelled('已完成失能；收到停止请求，取消关闭窗口')
                self.update(home_stage='closing', close_allowed=True)
                self._publish_ui()
                self.cancel.set()
            else:
                self._set_plot_zero(self.runtime.measured())
                self.reset_target()
        except Exception as exc:
            self._discard_plan()
            failed_close = close or self.snapshot()['closing_requested'] or self.snapshot()['close_failed']
            self.update(home_stage='failed', close_failed=failed_close, closing_requested=False,
                        close_allowed=False, error=str(exc), status='回零未完成，退出已取消')
        finally:
            self.update(home_active=False, busy=False)

    def _make_marker(self):
        from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
        marker = InteractiveMarker()
        marker.header.frame_id, marker.name, marker.description, marker.scale = 'base_link', 'tcp_goal', 'XYZ + Roll / Pitch / Yaw', .25
        marker.pose = pose_message(self.target)
        center = InteractiveMarkerControl(name='move_tcp', always_visible=True, interaction_mode=InteractiveMarkerControl.MOVE_3D)
        center.orientation.w = 1.
        sphere = Marker(type=Marker.SPHERE)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = .06
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = .2, 1., .65, 1.
        sphere.pose.orientation.w = 1.
        center.markers = [sphere]
        marker.controls = [center]
        for axis, component in [('x', 'x'), ('y', 'z'), ('z', 'y')]:
            for kind, mode in [('rotate', InteractiveMarkerControl.ROTATE_AXIS), ('move', InteractiveMarkerControl.MOVE_AXIS)]:
                if kind == 'rotate' and self.snapshot()['position_only']:
                    continue
                control = InteractiveMarkerControl(name=kind+'_'+axis, interaction_mode=mode)
                control.orientation.w = 2**-.5
                setattr(control.orientation, component, 2**-.5 * (-1 if axis == 'z' else 1))
                marker.controls.append(control)
        self.marker_server.insert(marker, feedback_callback=self._marker_feedback)
        self.marker_server.applyChanges()

    def _marker_feedback(self, message):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        if message.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            try:
                if self.snapshot()['home_active']:
                    raise ValueError('回零期间不能修改目标')
                self._set_target(pose_matrix(message.pose))
            except Exception as exc:
                self.update(error=str(exc))
                self._make_marker()
