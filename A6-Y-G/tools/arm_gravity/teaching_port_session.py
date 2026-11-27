"""Native POS_VEL home, stock direct-position follow, verified MIT gravity."""
from concurrent.futures import Future
import time
import threading
import traceback
import xml.etree.ElementTree as ET
import numpy as np

from gravity_config import JOINTS, CONTROLLER
from gravity_session import GravitySession, GravityStack
from teaching_port_config import HOME, FOLLOW, HOME_TOLERANCE, HOME_TIMEOUT, write_port_control, port_parameters, position_bounds
from teaching_gripper import GRIPPER_JOINT, controlled_joints, gripper_profile


GRIPPER_HOLD_TOLERANCE = .002  # Metres of one finger, not radians or jaw width.


def home_targets(model, positions):
    """Return only six axes to zero; preserve each gripper's measured opening."""
    names = controlled_joints(model)
    targets = {name: 0. for name in JOINTS[:6]}
    if GRIPPER_JOINT in names:
        if positions is None or len(positions) != len(JOINTS):
            raise ValueError('回零保持夹爪需要完整七轴反馈')
        value = float(positions[6])
        limits = gripper_profile(model)
        if not np.isfinite(value) or not limits['hardware_min_position_m'] <= value <= limits['hardware_max_position_m']:
            raise ValueError('夹爪实际位置非有限或超出电机协议范围，不能生成回零保持目标')
        targets[GRIPPER_JOINT] = value
    return targets


def verify_gripper_hold(positions, targets):
    if GRIPPER_JOINT not in targets:
        return
    if positions is None or len(positions) != len(JOINTS):
        raise RuntimeError('夹爪保持反馈缺失')
    actual = float(positions[6])
    if not np.isfinite(actual) or abs(actual-targets[GRIPPER_JOINT]) > GRIPPER_HOLD_TOLERANCE:
        raise RuntimeError('退出回零期间夹爪未保持原开度（单指位移容差 0.002 m）')


class PortStack(GravityStack):
    # NativeHome.execute checks the exit code AND the measured final pose.
    # Exclude it before polling; it may exit between any two polls.
    separately_supervised_children = ('native_home',)

    def start_control(self, enabled, initial=None, disarm_only=False):
        self.manager_name = 'controller_manager_'+str(self.control_generation)
        self.control_generation += 1
        self.path = write_port_control(self.model,self.params,self.directory,self.namespace,
            self.manager_name,self.mock,enabled,initial,disarm_only)
        self.spawn('controller_manager','controller_manager','ros2_control_node',
            ['--ros-args','-r','controller_manager:__node:='+self.manager_name],control=True,params=self.path)
        # FOLLOW also provides measured-position hold when cancelling a teacher's
        # exit trajectory. Only teachers may load a gravity controller.
        self.controller_names = ['joint_state_broadcaster',HOME,FOLLOW]
        if self.model['role'].startswith('teaching_'):self.controller_names.append(CONTROLLER)
        for name in self.controller_names:
            args = [name,'--controller-manager',self.namespace+'/'+self.manager_name,
                    '--controller-manager-timeout','25','--param-file',str(self.path)]
            if name != 'joint_state_broadcaster':args += ['--inactive']
            self.spawn('spawner_'+name,'controller_manager','spawner',args,control=True)

class NativeHome:
    def __init__(self, session):
        from action_msgs.srv import CancelGoal
        self.session = session
        self.child = None
        self.cancel_client = session.stack.node.create_client(CancelGoal,
            session.namespace+'/'+HOME+'/follow_joint_trajectory/_action/cancel_goal')

    def cancel(self):
        if self.child is None:return
        from action_msgs.srv import CancelGoal
        s = self.session
        # Killing the action client alone does not cancel an accepted trajectory.
        unsuccessful = self.child.poll() != 0
        try:
            s.stack.terminate([('native_home',self.child)],s._pump)
            if not self.cancel_client.wait_for_service(timeout_sec=1.):
                raise RuntimeError('原生回零取消服务不可用')
            future = self.cancel_client.call_async(CancelGoal.Request())
            s._wait(future.done,timeout=2.,abortable=False)
            if future.exception() or future.result().return_code not in (0,2):
                raise RuntimeError('原生回零取消未确认')
            if unsuccessful and not s.release_event.is_set() and not s.stop_event.is_set():
                # Retire the old action server state as well: an in-flight goal
                # from the terminated client must not restart a cancelled home.
                # Both controllers use POS_VEL; hardware keeps position support.
                s._select(FOLLOW)
                s._select(HOME)
        finally:
            s.stack.terminate([('native_home',self.child)],s._pump)
            s.stack.children = [(n,p) for n,p in s.stack.children if p is not self.child]
            self.child = None

    def execute(self, exiting=False):
        s = self.session
        if exiting:s.cancel_home.clear()
        if s.release_event.is_set() or s.stop_event.is_set():raise InterruptedError('回零已取消')
        s._select(HOME)
        targets = home_targets(s.model, s.feedback.measured())
        robot = ET.Element('robot',name='native_home')
        group = ET.SubElement(robot,'group_state',name='home',group='arm')
        for name,value in targets.items():ET.SubElement(group,'joint',name=name,value=str(value))
        srdf = s.stack.directory/'native_home.srdf'
        srdf.write_text(ET.tostring(robot,encoding='unicode'))
        s.update(busy=True,home_stage='executing',status='电机原生 POS_VEL 位置模式回零')
        s._event('native_home_started',controller=HOME,mode='pos_vel',move_duration=20.,hold_duration=2.,
                 timeout=HOME_TIMEOUT,exiting=exiting,targets=targets,
                 gripper_policy='hold_measured_opening_then_disable')
        joint_args = [value for name in targets for value in ('--joint',name)]
        self.child = s.stack.spawn('native_home','a6_y_ros2_control','move_to_group_state.py',
            ['--srdf',str(srdf),'--action',s.namespace+'/'+HOME+'/follow_joint_trajectory',
             '--joint-state-topic',s.namespace+'/joint_states',*joint_args])
        try:
            started = time.monotonic();deadline = started+HOME_TIMEOUT;next_progress=started
            while self.child.poll() is None:
                s._pump();s.stack.check()
                if s.cancel_home.is_set() or s.release_event.is_set() or s.stop_event.is_set() or (s.close_event.is_set() and not exiting):
                    raise InterruptedError('原生回零已请求取消')
                if not s._fresh_after_pending(enabled=True):raise RuntimeError('原生回零期间反馈异常')
                verify_gripper_hold(s.feedback.measured(), targets)
                if time.monotonic()>deadline:raise TimeoutError('原生回零工具超时，见 native_home.log')
                if time.monotonic()>=next_progress:
                    elapsed=time.monotonic()-started
                    error=float(np.max(np.abs(s.feedback.measured()[:6])))
                    s.update(home_elapsed=elapsed,home_remaining=max(0.,deadline-time.monotonic()),
                        status=f'退出回零 {elapsed:.0f}s / 最长 {HOME_TIMEOUT:.0f}s；距零位 {error:.3f} rad')
                    next_progress=time.monotonic()+.5
                s._sample(s.csv_writer)
            code = self.child.returncode
            if code != 0:raise RuntimeError(f'原生位置回零失败（{code}），见 native_home.log')
            s._wait(lambda:s._fresh(enabled=True),timeout=2.)
            if np.max(np.abs(s.feedback.measured()[:6])) > HOME_TOLERANCE:
                raise RuntimeError('原生位置回零结果与实际反馈不符（容差 0.03 rad）')
            verify_gripper_hold(s.feedback.measured(), targets)
            s._event('native_home_reached',positions=s.feedback.measured().tolist(),tolerance=HOME_TOLERANCE)
        finally:
            self.cancel()


class PortArmSession(GravitySession):
    pump_callbacks = 6

    def start(self, role, mock=True, with_rviz=False, parameters=None):
        self.cancel_home = threading.Event()
        self.selected_controller = None
        p = dict(parameters or port_parameters(role,self.roles))
        p.update(planned_home_enabled=False,home_mock_follow=False,
                 teaching_follow_enabled=False,teaching_position_output=False)
        self.update(follow_phase=0,follow_message='',startup_homed=False,backend='disabled',
                    gravity_enabled=False,home_elapsed=0.,home_remaining=0.)
        super().start(role,mock,with_rviz,p)

    def _create_stack(self, folder, with_rviz):
        return PortStack(self.model,self.params,folder,self.namespace,self.snapshot()['mock'],with_rviz)

    def _create_home(self):return NativeHome(self)

    def _create_math(self):
        return super()._create_math() if self.model['role'].startswith('teaching_') else None

    def _startup_controller(self):
        return CONTROLLER if self.model['role'].startswith('teaching_') else FOLLOW

    def _validate_enable_positions(self, positions):
        if positions is None or len(positions) != len(JOINTS):
            raise ValueError('启动需要完整的七轴实际反馈')
        bounds = position_bounds(self.model)
        grip = gripper_profile(self.model)
        bounds[GRIPPER_JOINT] = (grip['hardware_min_position_m'],grip['hardware_max_position_m'])
        for name,q in zip(JOINTS,positions):
            low,high = bounds[name]
            if not np.isfinite(q) or not low <= q <= high:
                label = '夹爪协议范围/m' if name == GRIPPER_JOINT else '电机 PMAX 换算范围/rad'
                raise ValueError(f'{name} 实际位置 {q} 超出{label} [{low}, {high}]')

    def _fresh(self, disabled=False, enabled=False, sample=None):
        state = self.feedback.snapshot() if sample is None else sample
        if not super()._fresh(disabled=disabled,enabled=enabled,sample=state):return False
        with self.lock:mock=self.state['mock']
        if mock:return True
        row = state['rows'].get(GRIPPER_JOINT,{})
        if row.get('position_valid') != 1 or not 0 <= row.get('position_age',-1) < .5:
            return False
        if enabled:
            if row.get('motor_error') != 1.:return False
        return True

    def _ready_status(self):
        return '反馈就绪，等待六轴重力补偿与夹爪零刚度力反馈' if self.model['role'].startswith('teaching_') else '反馈就绪，等待六轴及夹爪 POS_VEL 跟随'

    def _enabled_status(self):
        return '示教臂六轴 MIT 重力补偿，夹爪轻阻尼、接触反馈' if self.model['role'].startswith('teaching_') else '运动臂六轴及夹爪 POS_VEL 位置跟随（无重力补偿）'

    def _controllers_ready(self):
        children = [p for n,p in self.stack.control_children if n.startswith('spawner_')]
        return len(children)==len(self.stack.controller_names) and all(p.poll()==0 for p in children)

    def _feedback_qos(self):
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        return QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)

    def control_state(self, feedback=False):
        with self.lock:
            state={k:self.state.get(k) for k in ('role','running','active','error','follow_phase','follow_message')}
        if feedback:state['hardware']=self.feedback.snapshot()
        return state

    def _switch(self, activate):self._select(self._startup_controller() if activate else None)

    def _select(self, name):
        if name==CONTROLLER and not self.model['role'].startswith('teaching_'):
            raise ValueError('运动臂禁止开启重力补偿，只允许 POS_VEL 位置控制')
        if name == self.selected_controller:return
        from controller_manager_msgs.srv import SwitchController
        client = self.stack.node.create_client(SwitchController,self.namespace+'/'+self.stack.manager_name+'/switch_controller')
        old = self.selected_controller
        try:
            if not client.wait_for_service(timeout_sec=2.):raise TimeoutError('原生控制器切换服务不可用')
            request = SwitchController.Request()
            request.activate_controllers = [name] if name else []
            request.deactivate_controllers = [old] if old else []
            request.strictness = SwitchController.Request.STRICT
            request.timeout.sec = 3
            future = client.call_async(request)
            self._wait(future.done,timeout=5.,abortable=False)
            if future.exception() or not future.result().ok:
                raise RuntimeError(f'原生控制器切换失败：{old} → {name}')
            self.selected_controller = name
            self.controller_requested = name is not None
            confirmed_after=time.monotonic()
            self._wait(lambda:self._fresh(enabled=bool(name),disabled=not name) and
                (self.feedback.snapshot().get('diagnostic_source_time') or 0)>=confirmed_after,
                timeout=4.,abortable=False)
            self.update(backend='mit_gravity' if name==CONTROLLER else 'pos_vel' if name else 'disabled',
                        gravity_enabled=name==CONTROLLER)
            self._event('controller_switch_confirmed',previous=old,controller=name,backend=self.snapshot()['backend'])
        finally:self.stack.node.destroy_client(client)

    def submit(self, operation):
        allowed = ('gravity','hold') if self.model['role'].startswith('teaching_') else ('follow','hold')
        if operation not in allowed:raise ValueError('该机械臂不支持此操作：'+operation)
        with self.lock:
            if not self.state['ready'] or self.state['busy'] or self.state['closing_requested']:
                raise ValueError('机械臂未就绪或正在处理其他操作')
            self.state.update(busy=True,notice='',close_failed=False)
        self.cancel_home.clear()
        future = Future()
        self.commands.put_nowait(('port',operation,future))
        return future

    def _extra_command(self, command):
        _,operation,future = command
        try:
            if self.close_event.is_set() or self.release_event.is_set():raise InterruptedError('操作已取消')
            if not self.snapshot()['active']:
                if operation=='hold':raise ValueError('尚未接管的机械臂不能进入保持')
                self._enable()
            self._select({'gravity':CONTROLLER,'follow':FOLLOW,'hold':HOME}[operation])
            self.update(follow_phase={'gravity':0,'follow':1,'hold':2}[operation],
                status={'gravity':'示教臂 MIT 重力补偿','follow':'运动臂 POS_VEL 跟随（无重力补偿）','hold':'原生位置保持'}[operation])
            self._event('teaching_'+operation)
            future.set_result(True)
        except Exception as exc:
            self.update(notice=str(exc))
            self._event('teaching_operation_failed',operation=operation,message=str(exc),traceback=traceback.format_exc())
            future.set_exception(exc)
        finally:self.update(busy=False)

    def _release(self):
        if self.home:
            try:self.home.cancel()
            except Exception as exc:self._event('home_cancel_unverified',message=str(exc))
        super()._release()
        self.update(follow_phase=0,status='控制输出已停止，已核验七台电机失能，反馈保持连接')

    def _close_normally(self):
        self.update(busy=True,notice='')
        homed=False
        try:
            if self.release_event.is_set():self._release()
            elif self.snapshot()['active']:
                self.home.execute(exiting=True)
                homed=True
                self._release()
            elif self._fresh_after_pending(disabled=True):self.update(release_confirmed=True)
            else:raise RuntimeError('电机失能未确认，退出已取消')
            self.update(home_stage='complete' if homed else 'skipped',
                        status='原生位置回零及失能核验完成' if homed else '已确认失能，正在关闭会话')
            self.stop_event.set()
        except Exception as exc:
            self.close_event.clear()
            self.update(busy=False,closing_requested=False,close_failed=True,home_stage='failed',
                notice=str(exc),status='原生回零或失能未完成，保留窗口')
            self._event('close_cancelled',message=str(exc))
