"""MoveIt OMPL planning + Pinocchio IK + RViz target, with explicit execution."""
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
import uuid

import numpy as np
import pinocchio as pin
import yaml

from motion_model import ARM_JOINTS, GRIPPER_JOINTS, JOINTS, ROOT, Kinematics, planning_model, moveit_parameters, joint_tolerance, bounded_start
from hardware import ArmRuntime
from trajectory import from_ros, gripper_motion
from preview import ModelPreview
from ros_display import RosDisplay, rviz_config


def pose_message(transform):
    from geometry_msgs.msg import Pose
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, transform[:3, 3])
    q = pin.Quaternion(transform[:3, :3]).coeffs()
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = map(float, q)
    return result


def pose_matrix(pose):
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    xyz = np.array([pose.position.x, pose.position.y, pose.position.z])
    if not np.isfinite(np.r_[q, xyz]).all() or np.linalg.norm(q) < .001:
        raise ValueError('RViz 目标位姿无效')
    q /= np.linalg.norm(q)
    result = np.eye(4)
    result[:3, :3] = pin.Quaternion(q[3], q[0], q[1], q[2]).matrix()
    result[:3, 3] = xyz
    return result


class PlanningDisplay(RosDisplay):
    def alive(self):
        # Closing only RViz must not tear down MoveIt or the serial owner before
        # the parent has finished the requested return-to-zero sequence.
        for name, child in self.children:
            code = child.poll()
            if code is not None:
                if name == 'rviz2' and code == 0:
                    if not getattr(self, 'close_reported', False):
                        self.close_requested = self.close_reported = True
                    continue
                raise RuntimeError(f'{name} 已退出（{code}），详见 {self.directory / (name + ".log")}')
        return True

    def __enter__(self):
        requested_rviz = self.with_rviz
        self.with_rviz = False
        super().__enter__()
        try:
            from ament_index_python.packages import get_package_prefix
            config = self.directory / 'move_group.yaml'
            config.write_text(yaml.safe_dump({'/**': {'ros__parameters': moveit_parameters(self.model)}}))
            executable = Path(get_package_prefix('moveit_ros_move_group')) / 'lib/moveit_ros_move_group/move_group'
            self._spawn('move_group', [str(executable), *self.remaps, '--params-file', str(config)])
            if requested_rviz:
                rviz = rviz_config(self.namespace)
                rviz['Panels'] = [p for p in rviz['Panels'] if p['Class'] != 'rviz_common/Views']
                rviz['Panels'].insert(0, {'Class': 'a6y_rviz_panel/PlanningPanel', 'Name': 'A6Y 规划与夹爪'})
                rviz['Window Geometry'].update(Width=1440, Height=950)
                view = rviz['Visualization Manager']['Views']['Current']
                view.update(Distance=1.65, **{'Focal Point': {'X': .18, 'Y': 0., 'Z': .3}})
                displays = rviz['Visualization Manager']['Displays']
                for display in displays:
                    if display['Name'] == 'Feedback status and joint axes':
                        display.update(Enabled=False, Value=False)
                displays.append({'Class': 'rviz_default_plugins/InteractiveMarkers', 'Name': 'Drag TCP target',
                                 'Enabled': True, 'Value': True, 'Show Axes': False, 'Show Descriptions': False,
                                 'Interactive Markers Namespace': self.namespace + '/tcp_target'})
                displays.append({'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Goal and planned motion',
                                 'Enabled': True, 'Value': True,
                                 'Topic': {'Value': self.namespace + '/model_preview', 'Depth': 1,
                                           'Reliability Policy': 'Reliable', 'Durability Policy': 'Volatile'}})
                rviz['Visualization Manager']['Tools'].append({'Class': 'rviz_default_plugins/FocusCamera'})
                path = self.directory / 'moveit.rviz'
                path.write_text(yaml.safe_dump(rviz, sort_keys=False))
                executable = Path(get_package_prefix('rviz2')) / 'lib/rviz2/rviz2'
                self._spawn('rviz2', [str(executable), '-d', str(path), *self.remaps, '--params-file', str(config)])
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise


class PlanningSession:
    def __init__(self, roles=None, report_dir=None):
        self.roles = roles
        self.report_dir = Path(report_dir or ROOT.parent / 'diagnostics/arm_moveit_reports')
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.commands = queue.Queue(maxsize=20)
        self.worker = None
        self.runtime = None
        self.home = None
        self.state = {'running': False, 'ready': False, 'status': '请选择机械臂', 'error': None,
                      'plan_ready': False, 'busy': False, 'mock': True, 'permitted': False,
                      'velocity_scaling': .5, 'acceleration_scaling': .5, 'position_only': False,
                      'home_active': False, 'home_stage': '', 'closing_requested': False,
                      'close_failed': False, 'close_allowed': False,
                      'runtime_fault': None, 'handled_motion_failures': 0,
                      'preview_status': '', 'target_valid': False, 'warning': '', 'events': []}

    def update(self, **values):
        with self.lock:
            if values.get('error') and values['error'] != self.state.get('error'):
                event = {'time': time.time(), 'error': values['error']}
                if values.get('execution_failure'):
                    event['execution_failure'] = values['execution_failure']
                self.state['events'].append(event)
                if self.state.get('directory'):
                    try:
                        with (Path(self.state['directory']) / 'events.jsonl').open('a') as stream:
                            stream.write(json.dumps(event, ensure_ascii=False) + '\n')
                    except OSError:
                        pass  # The same event remains in the final session report.
            self.state.update(values)

    def snapshot(self):
        with self.lock:
            result = deepcopy(self.state)
        if self.runtime:
            result['hardware'] = self.runtime.snapshot()
        return result

    def start(self, role, mock=True, with_rviz=True):
        if self.worker and self.worker.is_alive():
            raise RuntimeError('请先关闭当前会话')
        self.model = planning_model(role, self.roles)
        self.cancel.clear()
        self.commands = queue.Queue(maxsize=20)
        self.runtime = ArmRuntime(self.model, mock)
        self.home = None
        self.update(running=True, ready=False, role=role, mock=mock, permitted=False,
                    error=None, status='启动 MoveIt、Pinocchio 与模型显示', plan_ready=False,
                    busy=False, target=None, report=None, events=[], directory=None, target_valid=False,
                    preview_status='', warning='', position_only=False, home_active=False, home_stage='',
                    closing_requested=False, close_failed=False, close_allowed=False, home_result=None)
        self.update(runtime_fault=None, handled_motion_failures=0, execution_failure=None)
        self.worker = threading.Thread(target=self._run, args=(with_rviz,), daemon=False)
        self.worker.start()

    def send(self, command, value=None):
        if command in ('execute', 'execute_arm', 'execute_gripper') and self.runtime:
            value = self.runtime.generation()
        if command == 'stop' and self.runtime:
            self.runtime.cancel_motion()
        if command in ('home', 'release', 'shutdown') and self.runtime and not self.snapshot()['home_active']:
            self.runtime.cancel_motion()
        if command == 'emergency_release' and self.runtime:
            self.runtime.cancel_motion(release=True)
        if command == 'permit' and not value and self.runtime:
            self.update(permitted=False)
            self.runtime.cancel_motion()
        if not self.snapshot()['running']:
            return
        try:
            self.commands.put_nowait((command, value))
        except queue.Full:
            self.update(error='操作队列繁忙，请稍后重试')

    def stop(self):
        """Normal exit: plan and execute the home motion before teardown."""
        self.send('shutdown')

    def abort(self):
        """Fault/test teardown only; never claim a home motion was completed."""
        self.cancel.set()
        if self.runtime:
            self.runtime.cancel_motion(shutdown=True)

    def _run(self, with_rviz):
        directory = executor = interface = None
        try:
            from rclpy.executors import SingleThreadedExecutor
            import rclpy
            self.report_dir.mkdir(parents=True, exist_ok=True)
            directory = Path(tempfile.mkdtemp(prefix=self.model['role'] + '_', dir=self.report_dir))
            self.namespace = '/a6y_moveit/' + self.model['role'] + '/s' + uuid.uuid4().hex[:8]
            self.update(directory=str(directory), namespace=self.namespace)
            self.runtime.start()
            with PlanningDisplay(self.model, directory, self.namespace, with_rviz) as self.display:
                try:
                    self.node = rclpy.create_node('planning_interface', namespace=self.namespace, context=self.display.context)
                    interface = self.node
                    executor = SingleThreadedExecutor(context=self.display.context)
                    executor.add_node(self.node)
                    self._setup_ros()
                    deadline, published = time.monotonic() + 25., 0.
                    while not self.cancel.is_set() and self.display.alive():
                        hardware = self.runtime.snapshot()
                        self._handle_runtime_status(hardware)
                        if not hardware['error'] and not self.snapshot()['ready']:
                            if self.plan_client.service_is_ready() and self.valid_client.service_is_ready() and hardware['q'] is not None:
                                self.reset_target()
                                self.update(ready=True, status='拖动 RViz 末端坐标轴，再点击规划')
                            elif time.monotonic() > deadline:
                                raise RuntimeError('MoveIt 服务或电机反馈未就绪；请查看本次日志')
                        executor.spin_once(timeout_sec=.005)
                        if getattr(self.display, 'close_requested', False):
                            self.display.close_requested = False
                            if not self.snapshot()['closing_requested']:
                                self.send('shutdown')
                        if not hardware['error']:
                            self._poll_preview()
                        self._poll_future()
                        try:
                            command, value = self.commands.get_nowait()
                            self._command(command, value)
                        except queue.Empty:
                            pass
                        except Exception as exc:
                            if self.home is not None or command in ('home', 'release', 'shutdown'):
                                self._fail_home(str(exc), closing=command == 'shutdown')
                            if command in ('execute', 'execute_arm', 'execute_gripper'):
                                self.plan = None
                                self.update(plan_ready=False)
                            self.update(error=str(exc), busy=False)
                        self._poll_home()
                        if self.plan is not None and self.snapshot()['plan_ready'] and time.monotonic() - self.plan.created > 30:
                            self.plan = None
                            self.update(plan_ready=False, status='预览已超过 30 秒，请重新规划')
                        if time.monotonic() - published > .05:
                            try:
                                q = self.runtime.measured()
                            except ValueError:
                                q = None
                            label = ('MOCK / NO HARDWARE' if self.runtime.mock else 'REAL ARM / ' + ('EXECUTING' if hardware['executing'] else 'HOLDING' if hardware['enabled'] else 'READONLY'))
                            self.display.publish(q, label if q is not None else 'FEEDBACK LOST / POSE FROZEN', q is None)
                            self._publish_ui()
                            published = time.monotonic()
                finally:
                    self.runtime.cancel_motion(shutdown=True)
                    self._destroy_ros(executor, interface)
                    executor = interface = None
            self.update(status='会话已关闭，电机已请求失能并释放串口')
        except Exception as exc:
            if self.snapshot()['closing_requested']:
                self.update(close_failed=True, closing_requested=False)
            self.update(error=f'{type(exc).__name__}: {exc}', status='会话已停止，请检查日志')
        finally:
            self.runtime.cancel_motion(shutdown=True)
            if self.runtime.worker:
                self.runtime.worker.join(5)
                if self.runtime.worker.is_alive():
                    self.update(error='通信线程未结束，设备状态未确认；请检查硬件')
            if executor or interface:
                self._destroy_ros(executor, interface)
            if self.runtime.snapshot()['error']:
                self.update(error=self.runtime.snapshot()['error'])
            self.update(running=False, ready=False, plan_ready=False, busy=False, permitted=False, home_active=False)
            if directory:
                try:
                    path = directory / 'session.json'
                    self.update(report=str(path))
                    report = self.snapshot()
                    report['model'] = {k: v for k, v in self.model.items() if k not in ('description', 'semantic')}
                    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
                except OSError as exc:
                    self.update(error=f'{self.snapshot()["error"] or ""}；保存记录失败：{exc}')

    def _handle_runtime_status(self, hardware):
        """A rejected trajectory is an execution result, not a ROS shutdown."""
        state = self.snapshot()
        failure = hardware.get('motion_failure')
        new_failure = failure and failure['sequence'] > state['handled_motion_failures']
        fatal = hardware['error']
        new_fatal = fatal and fatal != state['runtime_fault']
        if not new_failure and not new_fatal:
            return
        reason = fatal if new_fatal else failure['message']
        self.update(error=reason, execution_failure=failure)
        if self.home is not None:
            self._fail_home(reason)
        else:
            self._invalidate_plan()
        # No queued home/execute command may restart motion after an abort.
        # Stop/release requests have already been applied by send().
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
        self.update(busy=False, permitted=False, target_valid=False,
                    handled_motion_failures=hardware.get('failed', 0),
                    execution_failure=failure, error=reason,
                    preview_status='旧规划已丢弃；检查反馈后重新设置目标、规划和执行',
                    status=('硬件故障，已停止执行；界面保留，请检查日志' if fatal else
                            '本次执行已中止；反馈与界面保留，需要重新规划'))
        if fatal:
            self.update(runtime_fault=fatal, ready=False,
                        preview_status='反馈不可用，禁止运动；确认失能后关闭会话并重新连接')

    def _destroy_ros(self, executor, interface):
        if getattr(self, 'ik_pool', None):
            self.ik_pool.shutdown(wait=True, cancel_futures=True)
            self.ik_pool = None
        if executor:
            executor.shutdown()
        if getattr(self, 'marker_server', None):
            self.marker_server.shutdown()
            self.marker_server = None
        if interface:
            interface.destroy_node()

    def _setup_ros(self):
        from moveit_msgs.srv import GetMotionPlan, GetStateValidity
        from std_msgs.msg import String
        from interactive_markers.interactive_marker_server import InteractiveMarkerServer
        from interactive_markers.menu_handler import MenuHandler
        self.kin = Kinematics(self.model)
        self.preview_kin = Kinematics(self.model)
        self.ik_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='tcp-preview')
        self.preview_task = self.preview_check = self.preview_request = None
        self.preview_q = self.preview_source = None
        self.preview_revision = -1
        self.plan_after_preview = None
        self.visual = ModelPreview(self.node, self.model)
        self.plan_client = self.node.create_client(GetMotionPlan, 'plan_kinematic_path')
        self.valid_client = self.node.create_client(GetStateValidity, 'check_state_validity')
        self.ui_pub = self.node.create_publisher(String, 'ui/status', 1)
        self.ui_sub = self.node.create_subscription(String, 'ui/command', self._ui_command, 10)
        self.marker_server = InteractiveMarkerServer(self.node, 'tcp_target')
        self.menu = MenuHandler()
        self.menu.insert('Plan to this pose', callback=lambda _: self.send('plan'))
        self.menu.insert('Execute previewed plan', callback=lambda _: self.send('execute'))
        self.menu.insert('Stop trajectory / hold', callback=lambda _: self.send('stop'))
        self.menu.insert('Plan and execute return to joint zero', callback=lambda _: self.send('home'))
        self.menu.insert('Reset target to measured TCP', callback=lambda _: self.send('reset'))
        self.target = None
        self.gripper_target = None
        self.revision = 0
        self.plan = self.future = None
        self.plan_source = None
        self.preview_group = 'arm'
        self.pending_kind = None

    def _ui_command(self, message):
        try:
            data = json.loads(message.data)
            command, value = data['command'], data.get('value')
            if command not in ('plan', 'execute_arm', 'execute_gripper', 'reset', 'stop', 'release',
                               'permit', 'gripper_target', 'gripper_plan', 'settings', 'home', 'shutdown', 'emergency_release', 'release'):
                raise ValueError('未知面板操作')
            if command == 'permit' and not isinstance(value, bool):
                raise ValueError('执行许可必须是布尔值')
            self.send(command, value)
        except (KeyError, TypeError, ValueError) as exc:
            self.update(error=f'RViz 操作无效：{exc}')

    def _publish_ui(self):
        from std_msgs.msg import String
        state = self.snapshot()
        h = state['hardware']
        payload = {key: state.get(key) for key in ('ready', 'busy', 'mock', 'permitted', 'plan_ready',
                   'status', 'error', 'plan_group', 'gripper_percent', 'gripper_target_mm',
                   'velocity_scaling', 'acceleration_scaling', 'position_only', 'preview_status',
                   'target_valid', 'warning', 'home_active', 'home_stage', 'closing_requested', 'close_failed', 'close_allowed')}
        payload.update(role=self.model['arm']['name'], executing=h['executing'], enabled=h['enabled'],
                       hardware_status=h['status'], gripper_current_mm=None if h['q'] is None else h['q'][6] * 1000)
        self.ui_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        goal = None if self.preview_q is None else self.preview_q.copy()
        if self.preview_group == 'gripper' and h['q'] is not None:
            goal = np.array(h['q'])
            goal[6] = self.gripper_target
        self.visual.publish(goal, self.target, state['target_valid'],
                            self.plan if state['plan_ready'] else None, self.plan_source)

    def set_gripper_target(self, percent):
        if self.runtime.snapshot()['executing']:
            raise ValueError('请等待当前执行结束，再修改夹爪目标')
        percent = float(percent)
        if not np.isfinite(percent) or not 0 <= percent <= 100:
            raise ValueError('夹爪开度比例需要在 0～100% 之间')
        entry = self.model['joints'][GRIPPER_JOINTS[0]]
        self.gripper_target = entry['upper'] + percent / 100 * (entry['lower'] - entry['upper'])
        self.preview_group = 'gripper'
        self.revision += 1
        self.plan = None
        self.plan_after_preview = None
        if self.future is None:
            self.update(busy=False)
        self.update(plan_ready=False, gripper_percent=percent, gripper_target_mm=self.gripper_target * 1000,
                    error=None, status='夹爪目标已预览；点击夹爪规划，再执行')

    def reset_target(self):
        if self.gripper_target is None:
            q = self.runtime.measured()
            entry = self.model['joints'][GRIPPER_JOINTS[0]]
            self.set_gripper_target(np.clip((entry['upper'] - q[6]) / (entry['upper'] - entry['lower']) * 100, 0, 100))
        self._set_target(self.kin.fk(self.runtime.measured()))
        self._make_marker()

    def _set_target(self, target):
        if self.runtime.snapshot()['executing'] or self.snapshot()['home_active']:
            raise ValueError('执行期间不能修改目标；请先停止轨迹')
        self.target = np.array(target)
        self.revision += 1
        self.plan = None
        self.preview_group = 'arm'
        self.plan_after_preview = None
        if self.future is None:
            self.update(busy=False)
        self._queue_preview()
        self.update(plan_ready=False, target=self.target.tolist(), error=None, status='目标已更新，正在检查可达性')

    def _queue_preview(self):
        raw = self.runtime.measured()
        seed, warning = bounded_start(self.model, raw)
        if self.preview_q is not None:
            seed[:6] = self.preview_q[:6]
        self.preview_request = (self.revision, self.target.copy(), seed, raw.copy(), self.snapshot()['position_only'])
        self.update(target_valid=False, preview_status='正在求解并检查碰撞…', warning=warning)

    def _poll_preview(self):
        """Coalesce mouse events; run only the latest IK and full-robot check."""
        from moveit_msgs.srv import GetStateValidity
        try:
            if self.preview_task is not None and self.preview_task.done():
                task, self.preview_task = self.preview_task, None
                if self.preview_work[0] == self.revision:
                    self.preview_candidate = task.result()
                    req = GetStateValidity.Request()
                    req.robot_state.joint_state.name = list(JOINTS)
                    req.robot_state.joint_state.position = self.preview_candidate.tolist()
                    self.preview_check = self.valid_client.call_async(req)
                    self.preview_deadline = time.monotonic() + 3
            if self.preview_check is not None and self.preview_work[0] != self.revision:
                self.preview_check.cancel()
                self.preview_check = None
            if self.preview_check is not None:
                if time.monotonic() > self.preview_deadline:
                    self.preview_check.cancel()
                    self.preview_check = None
                    raise ValueError('目标碰撞检查超时')
                if self.preview_check.done():
                    check, self.preview_check = self.preview_check, None
                    if self.preview_work[0] == self.revision:
                        response = check.result()
                        if not response.valid:
                            pairs = ', '.join(f'{c.contact_body_1}/{c.contact_body_2}' for c in response.contacts[:3])
                            raise ValueError('目标模型发生碰撞' + (f'：{pairs}' if pairs else ''))
                        self.preview_q = self.preview_candidate.copy()
                        self.preview_source = self.preview_work[3]
                        self.preview_revision = self.revision
                        self.update(target_valid=True, preview_status='绿色模型：目标可达且通过碰撞检查',
                                    preview_q=self.preview_q.tolist())
                        if self.preview_group == 'arm' and self.future is None:
                            self.update(status='拖动预览有效，可以规划')
                        if self.plan_after_preview and self.plan_after_preview[1] == self.revision:
                            options, _ = self.plan_after_preview
                            self.plan_after_preview = None
                            self.request_plan(options)
        except Exception as exc:
            self.preview_revision = self.revision
            waiting_for_preview = self.plan_after_preview is not None
            self.plan_after_preview = None
            self.update(target_valid=False, preview_status=str(exc))
            if waiting_for_preview or self.future is None:
                self.update(busy=False)
            if self.preview_group == 'arm' and self.future is None:
                self.update(status='红色目标不可用；模型保留上一次有效姿态')
        if self.preview_request is not None and self.preview_task is None and self.preview_check is None:
            self.preview_work, self.preview_request = self.preview_request, None
            _, target, seed, _, position_only = self.preview_work
            self.preview_task = self.ik_pool.submit(self.preview_kin.ik, target, seed, position_only)

    def _make_marker(self):
        from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
        marker = InteractiveMarker()
        marker.header.frame_id = 'base_link'
        marker.name, marker.description, marker.scale = 'tcp_goal', 'XYZ + Roll / Pitch / Yaw', .25
        marker.pose = pose_message(self.target)
        center = InteractiveMarkerControl()
        center.name = 'move_tcp'
        center.orientation.w = 1.
        center.always_visible = True
        center.interaction_mode = InteractiveMarkerControl.MOVE_3D
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = .06
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = .2, 1., .65, 1.
        sphere.pose.orientation.w = 1.
        center.markers.append(sphere)
        marker.controls.append(center)
        for axis, component in [('x', 'x'), ('y', 'z'), ('z', 'y')]:
            for kind, interaction in [('rotate', InteractiveMarkerControl.ROTATE_AXIS), ('move', InteractiveMarkerControl.MOVE_AXIS)]:
                if kind == 'rotate' and self.snapshot()['position_only']:
                    continue
                control = InteractiveMarkerControl()
                control.name = kind + '_' + axis
                control.description = ({'x': 'Roll', 'y': 'Pitch', 'z': 'Yaw'}[axis] if kind == 'rotate' else axis.upper())
                control.orientation.w = 2 ** -.5
                setattr(control.orientation, component, 2 ** -.5)
                if axis == 'z':
                    control.orientation.y *= -1
                control.interaction_mode = interaction
                marker.controls.append(control)
        self.marker_server.insert(marker, feedback_callback=self._marker_feedback)
        self.menu.apply(self.marker_server, marker.name)
        self.marker_server.applyChanges()

    def _marker_feedback(self, feedback):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            try:
                self._set_target(pose_matrix(feedback.pose))
            except ValueError as exc:
                self.marker_server.setPose('tcp_goal', pose_message(self.target))
                self.marker_server.applyChanges()
                self.update(error=str(exc))

    def _command(self, command, value):
        if command == 'permit':
            if value and self.runtime.snapshot()['error']:
                raise ValueError('硬件故障未解除，不能授予执行许可')
            self.update(permitted=bool(value))
            if not value and self.home is not None:
                self._fail_home('执行许可已撤销，回零与退出已取消')
            return
        if command in ('home', 'release', 'shutdown'):
            self.request_home(close=command == 'shutdown')
            return
        if command in ('stop', 'emergency_release'):
            if self.runtime.snapshot()['error']:
                raise ValueError('通信线程因故障停止；失能状态请查看错误与日志，不能宣称已发送新的停止指令')
            if self.home is not None:
                self._fail_home('回零与退出已由用户取消')
            self._invalidate_plan()
            if command == 'emergency_release':
                self.update(permitted=False)
            self.update(busy=False, plan_ready=False, status='已请求停止轨迹' if command == 'stop' else '已请求紧急失能；未完成回零')
            return
        if self.snapshot()['home_active']:
            raise ValueError('回零／退出正在进行；修改目标前请先停止回零')
        if not self.snapshot()['ready']:
            raise ValueError('等待模型与反馈就绪')
        if command == 'reset':
            self.reset_target()
        elif command == 'target':
            self._set_target(value)
            self._make_marker()
        elif command == 'plan':
            self.request_plan(value)
        elif command == 'settings':
            self.set_settings(value)
        elif command == 'gripper_target':
            self.set_gripper_target(value)
        elif command == 'gripper_plan':
            self.request_plan(value, group='gripper')
        elif command in ('execute', 'execute_arm', 'execute_gripper'):
            if command != 'execute' and (self.plan is None or self.plan.group != command[8:]):
                raise ValueError('请先规划对应的机械臂／夹爪轨迹')
            self.request_execute(value)

    def _invalidate_plan(self):
        self.revision += 1
        self.plan = self.plan_after_preview = self.preview_request = None
        if self.future is not None:
            self.future.cancel()
            self.future = None
        self.update(plan_ready=False)

    def _fail_home(self, reason, closing=False):
        closing = closing or self.snapshot()['closing_requested']
        self.home = None
        self.runtime.cancel_motion()
        self._invalidate_plan()
        self.update(home_active=False, home_stage='failed', closing_requested=False, close_failed=closing,
                    close_allowed=False, busy=False, error=reason,
                    status='回零未完成，退出已取消' if closing else '回零未完成，已停止轨迹')

    def request_home(self, close=False):
        state = self.snapshot()
        if state['hardware']['error']:
            worker = self.runtime.worker
            if close and (worker is None or not worker.is_alive()) and state['hardware'].get('release_confirmed', False):
                self.update(close_allowed=True, close_failed=False, status='故障会话关闭：已确认释放，未执行回零')
                self.abort()
                return
            raise ValueError('硬件故障，不能回零；需等待通信线程结束并确认失能状态，界面与日志保留')
        if self.home is not None:
            if close:
                self.update(closing_requested=True)
            return
        # A read-only session never enables motors merely because a window is
        # closed. An owned arm cannot silently bypass its return-to-zero step.
        if close and not state['hardware']['enabled'] and (not state['ready'] or (not state['mock'] and not state['permitted'])):
            self.update(close_allowed=True, close_failed=False, status='只读会话关闭：未使能电机')
            self.abort()
            return
        if not state['ready']:
            raise ValueError('模型与反馈尚未就绪，不能规划回零')
        if not state['mock'] and not state['permitted']:
            raise ValueError('回零需要真机执行许可；请核对后勾选，或支撑机械臂后使用紧急失能')
        self.runtime.cancel_motion()
        self._invalidate_plan()
        self.home = {'stage': 'stopping', 'generation': self.runtime.generation(), 'deadline': time.monotonic() + 300}
        self.update(home_active=True, home_stage='stopping', closing_requested=close, close_failed=False,
                    close_allowed=False, busy=True, error=None, status='回零：停止当前轨迹并获取起点')

    def _poll_home(self):
        if self.home is None:
            return
        try:
            home = self.home
            hardware = self.runtime.snapshot()
            if hardware['error']:
                raise ValueError(hardware['error'])
            if time.monotonic() > home['deadline']:
                raise ValueError('回零流程超时')
            stage = home['stage']
            if stage in ('stopping', 'planning', 'preview', 'executing') and home['generation'] != self.runtime.generation():
                raise ValueError('回零期间收到停止或失能请求，已取消回零')
            if stage == 'stopping' and not hardware['executing']:
                current = self.runtime.measured()
                bounded_start(self.model, current)
                goal = current.copy()
                goal[:6] = 0.
                for name in ARM_JOINTS:
                    if not self.model['joints'][name]['lower'] <= 0 <= self.model['joints'][name]['upper']:
                        raise ValueError(f'{name} 的零点不在关节范围内，拒绝回零')
                home['goal'] = goal
                if np.all(np.abs(current[:6]) <= .005):
                    self._finish_home(current)
                    return
                home['stage'] = 'planning'
                self.target = self.kin.fk(goal)
                self.preview_group = 'arm'
                self.update(target=self.target.tolist(), target_valid=False, preview_status='关节零位目标正在规划')
                self._make_marker()
                self.update(home_stage='planning')
                self.request_plan(joint_goal=goal)
                self.update(status='回零：MoveIt 规划 J1～J6 = 0，夹爪保持当前开度')
            elif stage == 'planning' and self.snapshot()['plan_ready']:
                home['stage'], home['execute_at'] = 'preview', time.monotonic() + 1.
                self.preview_q = home['goal'].copy()
                self.update(home_stage='preview', target_valid=True, preview_status='零位路径检查通过',
                            status='回零路径通过 · 1 秒后执行，可点击停止取消')
            elif stage == 'preview' and time.monotonic() >= home['execute_at']:
                # Verify actual zero more tightly than ordinary point-to-point
                # execution; no motor PID or firmware register is changed.
                self.plan.end_tolerance[:] = .004
                home['completed'] = hardware['completed']
                home['stage'] = 'executing'
                self.update(home_stage='executing')
                self.request_execute(home['generation'])
                self.update(status='回零：复核路径并执行；可随时点击停止')
            elif stage == 'executing' and hardware['completed'] > home['completed'] and not hardware['executing']:
                measured = self.runtime.measured()
                if not np.all(np.abs(measured[:6]) <= .005):
                    raise ValueError('回零反馈未达到 ±0.005 rad，退出已取消')
                self._finish_home(measured)
            elif stage == 'releasing' and not hardware['enabled'] and not hardware['executing']:
                home['stage'], home['close_at'] = 'closing', time.monotonic() + .25
                self.update(home_stage='closing', close_allowed=True, status='已回零并确认失能，正在退出')
            elif stage == 'closing' and time.monotonic() >= home['close_at']:
                self.abort()
        except Exception as exc:
            self._fail_home(str(exc))

    def _finish_home(self, measured):
        self.update(home_result={'joint_positions': measured.tolist(), 'at': time.time(), 'arm_tolerance_rad': .005})
        if self.snapshot()['closing_requested']:
            self.home['stage'] = 'releasing'
            self.update(home_stage='releasing', status='回零反馈通过，正在失能并确认')
            self.runtime.cancel_motion(release=True)
        else:
            self.home = None
            self.update(home_active=False, home_stage='complete', busy=False, plan_ready=False,
                        status='已回到关节零位 · 保持当前位置' if self.runtime.snapshot()['enabled'] else '当前已在关节零位 · 只读反馈')
            self.reset_target()
            self.update(status='关节回零完成（J1～J6），夹爪保持原开度')

    def set_settings(self, options):
        if self.runtime.snapshot()['executing']:
            raise ValueError('执行中不能修改速度／加速度，请先停止并重新规划')
        state = self.snapshot()
        changed = {}
        for key in ('velocity_scaling', 'acceleration_scaling'):
            value = float(options.get(key, state[key]))
            if not np.isfinite(value) or not .1 <= value <= 1.:
                raise ValueError('速度和加速度比例需在 10%～100% 之间')
            if value != state[key]:
                changed[key] = value
        mode = options.get('position_only', state['position_only'])
        if not isinstance(mode, bool):
            raise ValueError('仅位置选项必须为布尔值')
        if mode != state['position_only']:
            changed['position_only'] = mode
        if changed:
            previous_revision = self.revision
            self.revision += 1
            self.plan = self.plan_after_preview = None
            if self.future is None:
                self.update(busy=False)
            self.update(**changed, plan_ready=False, status='参数已改变，请重新规划')
            if 'position_only' in changed:
                self.preview_group = 'arm'
                self._queue_preview()
                self._make_marker()
            elif self.preview_revision == previous_revision:
                self.preview_revision = self.revision
            else:
                self._queue_preview()

    def request_plan(self, options=None, group='arm', joint_goal=None):
        from moveit_msgs.srv import GetMotionPlan
        from moveit_msgs.msg import Constraints, JointConstraint
        if self.future is not None or self.runtime.snapshot()['executing']:
            raise ValueError('请等待当前规划／执行结束')
        if isinstance(options, (int, float)):
            options = {'velocity_scaling': options, 'acceleration_scaling': options}
        self.set_settings(options or {})
        state = self.snapshot()
        speed, acceleration = state['velocity_scaling'], state['acceleration_scaling']
        current = self.runtime.measured()
        start, warning = bounded_start(self.model, current)
        if joint_goal is not None:
            goal = np.array(joint_goal, float)
        elif group == 'arm':
            if self.preview_revision != self.revision:
                if self.preview_request is None and self.preview_task is None and self.preview_check is None:
                    self._queue_preview()
                self.plan_after_preview = (options, self.revision)
                self.update(busy=True, status='等待当前拖动目标的逆解与碰撞检查')
                return
            if not state['target_valid']:
                raise ValueError(state['preview_status'])
            # The IK branch shown in green is exactly the planned target branch.
            goal = self.preview_q.copy()
            goal[6] = current[6]
        else:
            goal = current.copy()
            goal[6] = self.gripper_target
        self.plan = None
        self.update(plan_ready=False, busy=True, plan_group=group,
                    warning=warning, status='夹爪限速轨迹与全机碰撞检查' if group == 'gripper' else 'MoveIt 规划到绿色目标模型', error=None)
        self.plan_source = current.copy()
        self.plan_goal = goal.copy()
        self.plan_speed, self.plan_acceleration = speed, acceleration
        self.future_revision = self.revision
        if group == 'gripper':
            self.plan = gripper_motion(current[6], goal[6], self.model, speed, acceleration)
            self.plan.passive_source = current.copy()
            self._begin_validation('plan')
            return
        names = ARM_JOINTS if group == 'arm' else GRIPPER_JOINTS
        request = GetMotionPlan.Request()
        req = request.motion_plan_request
        req.group_name, req.pipeline_id, req.planner_id = group, 'ompl', 'RRTConnect'
        req.num_planning_attempts, req.allowed_planning_time = 5, 8.
        req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor = speed, acceleration
        req.workspace_parameters.header.frame_id = 'base_link'
        req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -2.
        req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z = 2.
        req.start_state.joint_state.name, req.start_state.joint_state.position = list(JOINTS), start.tolist()
        constraints = Constraints()
        for name in names:
            angle = goal[JOINTS.index(name)]
            constraint = JointConstraint()
            constraint.joint_name, constraint.position = name, float(angle)
            constraint.tolerance_above = constraint.tolerance_below = joint_tolerance(name, .0005, .00001)
            constraint.weight = 1.
            constraints.joint_constraints.append(constraint)
        req.goal_constraints = [constraints]
        self.future = self.plan_client.call_async(request)
        self.pending_kind, self.future_deadline = 'plan', time.monotonic() + 15

    def request_execute(self, generation=None):
        if self.future is not None or not self.plan:
            raise ValueError('请先完成有效规划并预览')
        if not self.runtime.mock and not self.snapshot()['permitted']:
            raise ValueError('请在选择窗口勾选“已核对本臂零位与方向，允许真机执行”')
        if generation is not None and generation != self.runtime.generation():
            raise ValueError('已取消排队中的执行请求')
        q = self.runtime.measured()
        self.plan.check_start(q)
        self._check_passive(q)
        self.validation_generation = self.runtime.generation() if generation is None else generation
        self.future_revision = self.revision
        self.update(busy=True, status='执行前复核插值路径碰撞与起点', error=None)
        self._begin_validation('execute')

    def _begin_validation(self, mode):
        self.validation_mode = mode
        self.validation_times = iter(np.linspace(0., self.plan.duration, int(np.ceil(self.plan.duration / .04)) + 1))
        self._next_validation()

    def _check_passive(self, q):
        if any(abs(q[i] - self.plan_source[i]) > joint_tolerance(n, .03, .0005)
               for i, n in enumerate(JOINTS) if i not in self.plan.indices):
            raise ValueError('未规划的关节位置已变化，请重新规划碰撞检查')

    def _next_validation(self):
        from moveit_msgs.srv import GetStateValidity
        try:
            elapsed = next(self.validation_times)
        except StopIteration:
            if self.validation_mode == 'plan':
                self.plan.created = time.monotonic()
                self.update(busy=False, plan_ready=True, duration=self.plan.duration,
                            status=f'规划通过 · {self.plan.duration:.1f} 秒；蓝色模型预览后点击执行', error=None)
                return
            # Freshness, drift, gripper and permission are checked again after all service replies.
            if not self.runtime.mock and not self.snapshot()['permitted']:
                raise ValueError('执行许可已撤销')
            q = self.runtime.measured()
            self._check_passive(q)
            self.runtime.submit(self.plan, expected_generation=self.validation_generation)
            self.plan = None
            self.update(busy=False, plan_ready=False, status='已提交轨迹执行')
            return
        q = self.plan_source.copy()
        q[self.plan.indices], _ = self.plan.sample(elapsed)
        request = GetStateValidity.Request()
        request.group_name = ''  # Check the whole robot, including both mimic fingers.
        request.robot_state.joint_state.name = list(JOINTS)
        request.robot_state.joint_state.position = q.tolist()
        self.future = self.valid_client.call_async(request)
        self.pending_kind, self.future_deadline = 'validate', time.monotonic() + 3

    def _poll_future(self):
        if self.future is None:
            return
        try:
            if time.monotonic() > self.future_deadline:
                self.future.cancel()
                self.future = None
                raise RuntimeError('MoveIt 服务请求超时')
            if not self.future.done():
                return
            future, self.future = self.future, None
            if self.future_revision != self.revision:
                self.update(busy=False, plan_ready=False, status='目标或操作已变化，请重新规划')
                return
            response = future.result()
            if self.pending_kind == 'validate':
                if not response.valid:
                    self.plan = None
                    raise ValueError('插值路径存在碰撞或约束冲突，拒绝执行')
                self._next_validation()
                return
            result = response.motion_plan_response
            if result.error_code.val != 1:
                raise ValueError(f'MoveIt 规划失败（代码 {result.error_code.val}），检查起点碰撞、范围和目标可达性')
            plan = from_ros(result.trajectory.joint_trajectory, self.model,
                            velocity_scaling=self.plan_speed, acceleration_scaling=self.plan_acceleration, retime=True)
            if np.any(np.abs(plan.q[-1] - self.plan_goal[plan.indices]) >
                      [joint_tolerance(n, .0006, .00002) for n in plan.names]):
                raise ValueError('MoveIt 返回的路径终点未达到目标，请重新规划')
            plan.passive_source = self.plan_source.copy()
            self.plan = plan
            self.update(status='规划已返回，检查重定时后的插值路径')
            self._begin_validation('plan')
        except Exception as exc:
            self.plan = None
            if self.home is not None:
                self._fail_home(str(exc))
            else:
                self.update(busy=False, plan_ready=False, error=str(exc), status='规划／执行检查未通过')
