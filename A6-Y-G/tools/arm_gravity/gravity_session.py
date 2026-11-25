"""Role-selected lifecycle around native ros2_control gravity compensation.

Qt never writes motor frames. Numerical previews run in Python; all real effort
commands use the existing C++ controller and the paired MIT hardware plugin.
"""
from contextlib import ExitStack
from copy import deepcopy
import csv
import json
from pathlib import Path
import queue
import threading
import time
import traceback
import uuid

import numpy as np
import yaml

from gravity_config import (ROOT, JOINTS, CONTROLLER, LIVE_KEYS, GravityModel, load_model,
                            load_roles, load_profile, validate_parameters, write_config, motor_limit_report)
from native_stack import NativeStack, NativeFeedback
from operation_lock import arm_operation
from usb_identity import verify_usb_identity
from ros_display import rviz_config


class EnableBlocked(RuntimeError):
    """Start preconditions failed before acquiring output; keep feedback alive."""


class GravityStack(NativeStack):
    def __init__(self, model, params, folder, namespace, mock, with_rviz):
        super().__init__(model, folder, namespace, mock, with_rviz)
        self.params = params

    def start(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.signals import SignalHandlerOptions
        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('gravity_session', namespace=self.namespace, context=self.context)
        path = self.directory/'robot_description.yaml'
        path.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':self.model['description']}}}))
        self.spawn('robot_state_publisher','robot_state_publisher','robot_state_publisher',params=path)
        self.start_control(False)
        if self.with_rviz:
            config = rviz_config(self.namespace)
            config['Visualization Manager']['Displays'] = [d for d in config['Visualization Manager']['Displays']
                                                          if d['Name'] != 'Feedback status and joint axes']
            config['Visualization Manager']['Views']['Current'].update(Distance=1.65, **{'Focal Point':{'X':.2,'Y':0.,'Z':.3}})
            rp = self.directory/'gravity.rviz'
            rp.write_text(yaml.safe_dump(config, sort_keys=False))
            self.spawn('rviz2','rviz2','rviz2',['-d',str(rp)], params=path)

    def start_control(self, enabled, initial=None, disarm_only=False):
        self.manager_name = 'controller_manager_'+str(self.control_generation)
        self.control_generation += 1
        self.path = write_config(self.model,self.params,self.directory,self.namespace,self.manager_name,
                                 self.mock,enabled,initial,disarm_only)
        self.spawn('controller_manager','controller_manager','ros2_control_node',
                   ['--ros-args','-r','controller_manager:__node:='+self.manager_name],control=True,params=self.path)
        for name in ('joint_state_broadcaster',CONTROLLER):
            args = [name,'--controller-manager',self.namespace+'/'+self.manager_name,
                    '--controller-manager-timeout','20','--param-file',str(self.path)]
            if name == CONTROLLER: args += ['--inactive']
            self.spawn('spawner_'+name,'controller_manager','spawner',args,control=True)

    def check(self):
        # A failed planner must cancel the return, not drop gravity support.
        # Retire only reaped planner processes; all control processes remain critical.
        dead=[(n,p) for n,p in self.children if n=='move_group' and p.poll() is not None]
        if dead:
            self.home_planner_error='MoveIt 规划器退出，回零已取消，重力补偿保持；请查看 move_group.log'
            self.home_planner_started=False
            self.children=[c for c in self.children if c not in dead]
        super().check()


class GravitySession:
    pump_callbacks = 32
    def __init__(self, roles=None, report_dir=None):
        self.roles = roles or load_roles()
        self.report_dir = Path(report_dir or ROOT.parent/'diagnostics/arm_gravity_reports')
        self.lock = threading.Lock()
        self.worker = self.stack = self.feedback = self.executor = None
        self.stop_event = threading.Event()
        self.release_event = threading.Event()
        self.close_event = threading.Event()
        self.commands = queue.Queue(maxsize=1)
        self.state = dict(running=False, ready=False, active=False, busy=False, mock=True,
                          status='请选择机械臂', error='', directory=None, release_confirmed=None,
                          home_stage='',closing_requested=False,close_failed=False,notice='')

    def update(self, **values):
        with self.lock: self.state.update(values)

    def snapshot(self):
        with self.lock: result = deepcopy(self.state)
        if self.feedback: result['hardware'] = self.feedback.snapshot()
        return result

    def start(self, role, mock=True, with_rviz=True, parameters=None):
        if self.worker and self.worker.is_alive(): raise ValueError('请先断开当前机械臂')
        self.model = load_model(role,self.roles)
        self.params = validate_parameters(parameters or load_profile(role,self.roles))
        self.motor_limits = motor_limit_report(self.model)
        self.params['feedback_velocity_limits'] = self.motor_limits['feedback_velocity_limits']
        self.feedback = NativeFeedback(self.model,mock)
        self.feedback.update(controller_backend=self._startup_controller(),status='等待机械臂反馈',
                             sample_source_time=None,diagnostic_source_time=None)
        self.stack = None
        self.tuning_client = None
        self.tuning_client_generation = None
        self.stop_event.clear()
        self.release_event.clear()
        self.close_event.clear()
        self.home = None
        self.commands = queue.Queue(maxsize=1)
        self.output_started = False
        self.controller_requested = False
        self.update(running=True,ready=False,active=False,busy=True,mock=mock,role=role,
                    status='正在建立反馈连接',error='',directory=None,release_confirmed=None,
                    gravity=None,commanded=None,effort=None,sequence=0,
                    applied_parameters=deepcopy(self.params),tuning_revision=0,tuning_error='',
                    tuning_status='启动参数已准备',tuning_pending=False,
                    home_stage='',closing_requested=False,close_failed=False,notice='')
        self.worker = threading.Thread(target=self._run,args=(with_rviz,),daemon=False)
        self.worker.start()

    def enable(self):
        with self.lock:
            if not self.state['ready'] or self.state['busy'] or self.state['active'] or self.state['closing_requested']:
                raise ValueError('反馈就绪且补偿停止时才能开启')
            self.state.update(busy=True, status='正在准备重力补偿',notice='',error='',close_failed=False)
        self.release_event.clear()
        self.commands.put_nowait('enable')

    def release(self):
        if self.snapshot()['running']:
            self.update(busy=True,status='正在请求立即失能，跳过回零')
        self.release_event.set()

    def tune(self, parameters):
        p = validate_parameters(parameters)
        with self.lock:
            if not self.state['ready'] or self.state['busy'] or self.state['closing_requested']:
                raise ValueError('请等待连接或当前操作完成')
            previous = self.state['applied_parameters']
            if any(p[k] != previous[k] for k in p if k not in LIVE_KEYS):
                raise ValueError('在线仅可调整重力系数、阻尼与摩擦助力；方向和限幅需断开后修改')
            self.state.update(busy=True,tuning_pending=True,tuning_error='',tuning_status='正在提交参数')
        self.commands.put_nowait(('tune',p))

    def _tuning_call(self, client, request):
        future=client.call_async(request)
        self._wait(future.done,timeout=3)
        response=future.result()
        if not response.success: raise ValueError(response.message)
        return response

    def _tune(self, p):
        from a6_y_teleop_msgs.srv import GravityTuning
        if self.tuning_client_generation != self.stack.control_generation:
            if self.tuning_client: self.stack.node.destroy_client(self.tuning_client)
            self.tuning_client=self.stack.node.create_client(GravityTuning,self.namespace+'/'+CONTROLLER+'/tune')
            self.tuning_client_generation=self.stack.control_generation
        client=self.tuning_client
        acknowledged=False
        try:
            if not client.wait_for_service(timeout_sec=2): raise TimeoutError('实时调参服务不可用，请重启界面')
            req=GravityTuning.Request(apply=True)
            for key in LIVE_KEYS: setattr(req,key,p[key])
            response=self._tuning_call(client,req)
            revision=response.revision
            for key in LIVE_KEYS:
                value=getattr(response,key)
                if key != 'friction_compensation_enabled': value=list(value)
                if value != p[key]: raise RuntimeError('控制器参数回读不一致：'+key)
            # Keep the latest accepted target for reactivation, even if the user
            # stops a transition. The interrupted revision is not shown as applied.
            self.params=deepcopy(p); self.stack.params=self.params
            acknowledged=True
            self.update(tuning_status='控制器已接受，正在平滑应用')
            self._event('tuning_accepted',revision=revision,parameters=p)
            if self.snapshot()['active']:
                deadline=time.monotonic()+3
                while response.applied_revision != revision:
                    if time.monotonic()>deadline: raise TimeoutError('控制循环未确认参数生效')
                    self._pump()
                    if not self._fresh_after_pending(enabled=True): raise RuntimeError('调参期间电机反馈异常')
                    response=self._tuning_call(client,GravityTuning.Request(apply=False))
                    if response.revision != revision: raise RuntimeError('参数被其他客户端修改，请重新连接核对')
                status='已生效（0.3 秒平滑过渡完成）'
            else:
                status='已准备，下次开启补偿生效'
            self._event('tuning_confirmed',revision=revision,active=self.snapshot()['active'],parameters=p)
            (Path(self.snapshot()['directory'])/'parameters_latest.yaml').write_text(yaml.safe_dump(p,sort_keys=False))
            self.update(applied_parameters=deepcopy(p),tuning_revision=revision,tuning_pending=False,
                        tuning_status=status,tuning_error='',busy=False)
        except InterruptedError:
            if not acknowledged:
                raise RuntimeError('参数确认前停止，正在关闭会话；重新连接后使用界面参数')
            raise

    def stop(self):
        # Normal close is a request, not cancellation of the torque controller.
        # Repeated window-close events must not restart a return trajectory.
        if self.close_event.is_set(): return
        self.close_event.set()
        self.update(closing_requested=True,close_failed=False,busy=self.snapshot()['running'])
        if not self.snapshot()['ready']:
            self.release_event.set();self.stop_event.set()

    def _close_normally(self):
        from gravity_home import GravityHome
        self.update(busy=True,notice='',home_stage='planning')
        self.stack.home_planner_error=''
        try:
            if self.snapshot()['active']:
                if self.home is None: self.home=GravityHome(self)
                self.update(status='退出：正在规划回零，当前仍有重力补偿支撑')
                self._event('home_planning')
                q=self.feedback.measured()
                if np.max(np.abs(q[:6]))<=.005:
                    end=time.monotonic()+.3
                    while time.monotonic()<end:
                        self.home.pump()
                        if np.max(np.abs(self.feedback.measured()[:6]))>.005:
                            raise ValueError('零位未保持稳定，退出已取消')
                    if np.max(np.abs(self.feedback.snapshot()['velocity'][:6]))>.03:
                        raise ValueError('机械臂仍在运动，退出已取消')
                    self._event('home_reached',positions=self.feedback.measured().tolist(),already_at_zero=True)
                else:
                    plan=self.home.plan()
                    self.home.execute(plan)
                self._release()
            elif not self._fresh_after_pending(disabled=True):
                raise ValueError('电机失能状态未确认，退出已取消。请托住机械臂，点击“立即失能”；本会话未接管，不自动回零。')
            else:
                self.update(release_confirmed=True)
            self.update(home_stage='complete',busy=True,status='已确认失能，正在关闭会话')
            self.stop_event.set()
        except Exception as exc:
            if self.home:
                try: self.home.cancel()
                except Exception as cancel_exc:
                    self._event('home_cancel_unverified',message=str(cancel_exc))
                    # Unknown controller state must not be presented as gravity mode.
                    self.release_event.set()
            self.close_event.clear()
            self.update(busy=False,closing_requested=False,close_failed=True,home_stage='failed',
                        notice=str(exc),status='回零或失能未完成，退出已取消')
            self._event('close_cancelled',message=str(exc))

    def _event(self, kind, **data):
        if hasattr(self,'events') and not self.events.closed:
            self.events.write(json.dumps(dict(time=time.time(),kind=kind,**data),ensure_ascii=False)+'\n')
            self.events.flush()

    def _pump(self):
        if self.executor and self.stack.context.ok():
            # Destroying another ROS context can briefly hold the interpreter.
            # Drain both depth-10 feedback queues before checking freshness, so
            # an updated JointState cannot be paired with an old diagnostic.
            # Each wait after the first is nonblocking; keep cancellation bounded.
            self.executor.spin_once(timeout_sec=.01)
            deadline = time.monotonic()+.02
            for _ in range(self.pump_callbacks):
                if time.monotonic() >= deadline: break
                self.executor.spin_once(timeout_sec=0.)

    @staticmethod
    def _source_time(msg):
        # Publishers use this host's system clock (no simulated ROS time).
        # Replayed queued messages must not become fresh just because their
        # callback finally runs. Invalid/future stamps cannot authorize output.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wall, monotonic = time.time(), time.monotonic()
        if not np.isfinite(stamp) or stamp <= 0 or stamp > wall+.05:
            return None
        return monotonic-max(0.,wall-stamp)

    def _joint_states(self, msg):
        positions=dict(zip(msg.name,msg.position));velocities=dict(zip(msg.name,msg.velocity))
        if all(n in positions and np.isfinite(positions[n]) for n in JOINTS):
            # Publish values and their source timestamp atomically. A sender
            # must never see new positions carrying the previous sample's age.
            self.feedback.update(q=[positions[n] for n in JOINTS],sample_time=time.monotonic(),
                velocity=[float(velocities[n]) if n in velocities and np.isfinite(velocities[n]) else None for n in JOINTS],
                ros_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                sample_source_time=self._source_time(msg))

    def _diagnostics(self, msg):
        rows={name:dict(zip(values.interface_names,values.values)) for name,values in zip(msg.joint_names,msg.interface_values)}
        self.feedback.update(rows=rows,diagnostic_time=time.monotonic(),diagnostic_source_time=self._source_time(msg))

    def _fresh_after_pending(self, **requirements):
        """Worker-only recheck of DDS arrivals before declaring feedback lost.

        Another context's shutdown may pause Python after our normal pump.
        Consume already published messages once more; source timestamps and
        driver feedback_age still have to satisfy the unchanged 0.5 s limit.
        Never call this from the UI or another thread sharing this executor.
        """
        if self._fresh(**requirements): return True
        self._pump()
        return self._fresh(**requirements)

    def _wait(self, predicate, timeout=15, abortable=True, timeout_message='等待控制器或新鲜反馈超时，请查看本次日志'):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self._pump()
            self.stack.check()
            if abortable and (self.stop_event.is_set() or self.release_event.is_set()):
                raise InterruptedError('启动已取消')
            if predicate(): return
        raise TimeoutError(timeout_message)

    def _controllers_ready(self):
        spawners=[p for n,p in self.stack.control_children if n.startswith('spawner_')]
        return len(spawners) == 2 and all(p.poll() == 0 for p in spawners)

    def _fresh(self, disabled=False, enabled=False, sample=None):
        s = self.feedback.snapshot() if sample is None else sample
        now = time.monotonic()
        if s['q'] is None or any(now-(s.get(k) or 0) > .5 for k in ('sample_time','diagnostic_time')):
            return False
        with self.lock: mock=self.state['mock']
        if mock: return True
        for key in ('sample_source_time','diagnostic_source_time'):
            if s.get(key) is None or not 0 <= now-s[key] < .5:
                return False
        for name in JOINTS:
            row = s['rows'].get(name,{})
            if row.get('has_feedback') != 1 or not 0 <= row.get('feedback_age',-1) < .5 or row.get('fault') != 0:
                return False
            error = row.get('motor_error')
            if error not in (0.,1.): return False
            if disabled and error != 0.: return False
            if enabled and name != JOINTS[6] and error != 1.: return False
        return True

    def _switch(self, activate):
        from controller_manager_msgs.srv import SwitchController
        client = self.stack.node.create_client(SwitchController,self.namespace+'/'+self.stack.manager_name+'/switch_controller')
        try:
            if not client.wait_for_service(timeout_sec=2): raise TimeoutError('控制器切换服务不可用')
            req = SwitchController.Request()
            req.activate_controllers = [CONTROLLER] if activate else []
            req.deactivate_controllers = [] if activate else [CONTROLLER]
            req.strictness = SwitchController.Request.STRICT
            req.timeout.sec = 3
            f = client.call_async(req)
            self._wait(f.done,timeout=5,abortable=False)
            if f.exception() or not f.result().ok: raise RuntimeError('控制器启停失败')
        finally:
            self.stack.node.destroy_client(client)

    def enable_block_reason(self):
        if self.feedback is None or not self._fresh():
            return '电机反馈缺失、过期或报告故障，暂不能开启补偿'
        if self.snapshot()['mock']:
            return ''
        rows = self.feedback.snapshot()['rows']
        # The driver's "enabled" field tracks this process's enable commands.
        # After a PC restart it can be zero while motors still report state 1.
        enabled = [('J'+str(i+1) if i < 6 else '夹爪') for i,n in enumerate(JOINTS)
                   if rows.get(n,{}).get('motor_error') == 1.]
        if enabled:
            return ('、'.join(enabled)+' 仍报告已使能，本会话尚未开启补偿。请托住机械臂，点击“立即失能”，'
                    '确认失能后再手动开启补偿。')
        return ''

    def _validate_enable_positions(self, positions):
        for q,n in zip(positions,JOINTS):
            entry = self.model['joints'][n]
            tolerance = .01 if n != JOINTS[6] else .001
            if q < entry['lower']-tolerance or q > entry['upper']+tolerance:
                raise ValueError(n+' 反馈超出模型范围，请复核零位或姿态')

    def _enable(self):
        if self.close_event.is_set(): raise InterruptedError('退出期间不能开启补偿')
        self._wait(lambda:self._fresh(),timeout=2,
                   timeout_message='开启补偿前未取得有效电机反馈，请检查连接和电机状态')
        reason = self.enable_block_reason()
        if reason:
            raise EnableBlocked(reason)
        s = self.feedback.snapshot()
        if not self.snapshot()['mock']:
            verify_usb_identity(self.model['arm'])
        self._validate_enable_positions(s['q'])
        self.update(release_confirmed=False)
        if not self.output_started:
            # Retire the read-only manager's client while motors are disabled.
            # Destroying/recreating DDS clients during active tuning can stall
            # feedback dispatch; keep one client for each manager generation.
            if self.tuning_client:
                self.stack.node.destroy_client(self.tuning_client)
                self.tuning_client = None
                self.tuning_client_generation = None
            self.stack.stop_control()
            if self.stop_event.is_set() or self.release_event.is_set(): raise InterruptedError('启动已取消')
            self.feedback.update(q=None,sample_time=None,diagnostic_time=None,rows={})
            self.output_started = True  # cleanup must attempt a release even if startup fails
            self.stack.start_control(True, initial=s['q'])
            self.update(tuning_revision=0)
        self._wait(lambda:self._controllers_ready() and self._fresh(disabled=True))
        if self.stop_event.is_set() or self.release_event.is_set() or self.close_event.is_set(): raise InterruptedError('启动已取消')
        self.controller_requested = True
        self._switch(True)
        self._wait(lambda:self._fresh(enabled=True),timeout=5)
        self.update(active=True,busy=False,status=self._enabled_status(),release_confirmed=False,
                    applied_parameters=deepcopy(self.params),tuning_status='当前参数已生效',tuning_error='')
        self._event('enabled', role=self.model['role'])

    def _release(self):
        if self.output_started:
            self.update(busy=True,status='正在停止控制输出并核对失能反馈')
            if self.controller_requested:
                self._switch(False)
                self.controller_requested = False
            self._wait(lambda:self._fresh(disabled=True),timeout=4,abortable=False)
        elif not self._fresh_after_pending(disabled=True):
            self._disarm_external()
        self.feedback.update(enabled=False,release_confirmed=True)
        self.update(active=False,busy=False,status='控制输出已停止，已确认六轴失能，反馈保持连接',release_confirmed=True,notice='')
        self._event('released',confirmed=True)

    def _disarm_external(self):
        # Explicit release only: replace the read-only manager with a manager
        # that may DISABLE J1..J6 on teardown, but can never enable/send MIT.
        if not self._fresh(): raise RuntimeError('反馈不可用，无法核验失能；请使用硬件急停')
        if not self.snapshot()['mock']: verify_usb_identity(self.model['arm'])
        q=self.feedback.snapshot()['q']
        self.update(busy=True,release_confirmed=False,status='正在清除电机残留使能；不会回零或开启补偿')
        self._event('external_disarm_requested')
        if self.tuning_client:
            self.stack.node.destroy_client(self.tuning_client);self.tuning_client=None;self.tuning_client_generation=None
        self.stack.stop_control()
        self.feedback.update(q=None,sample_time=None,diagnostic_time=None,rows={})
        self.stack.start_control(True,initial=q,disarm_only=True)
        self._wait(lambda:self._controllers_ready() and self._fresh(),abortable=False)
        self.stack.stop_control()  # driver sends repeated FD; auto_enable=false throughout
        self.feedback.update(q=None,sample_time=None,diagnostic_time=None,rows={})
        self.stack.start_control(False,initial=q)
        self._wait(lambda:self._controllers_ready() and self._fresh(disabled=True),abortable=False)

    def _sample(self, csv_writer):
        s = self.feedback.snapshot()
        if s['q'] is None: return
        g = self.math.gravity(s['q']).tolist() if self.math is not None else [None]*len(JOINTS)
        measured = [s['rows'].get(n,{}).get('effort') for n in JOINTS]
        commanded = [s['rows'].get(n,{}).get('commanded_effort') for n in JOINTS]
        fresh = self._fresh()
        self.feedback.update(enabled=(self.snapshot()['active'] if self.snapshot()['mock'] else
                                      fresh and any(s['rows'].get(n,{}).get('motor_error') == 1 for n in JOINTS[:6])),
                             release_confirmed=self.snapshot().get('release_confirmed') is True)
        seq = self.snapshot()['sequence']+1
        self.update(gravity=g,commanded=commanded,effort=measured,sequence=seq,fresh=fresh)
        state=self.snapshot()
        csv_writer.writerow([time.time(),state['active'],fresh,*s['q'],*g,*commanded,*measured,
                             state['tuning_revision'],state['tuning_pending'],*s['velocity']])

    def _run(self, with_rviz):
        import rclpy
        from sensor_msgs.msg import JointState
        from control_msgs.msg import DynamicJointState
        from gravity_home import GravityHome
        self.report_dir.mkdir(parents=True,exist_ok=True)
        folder = self.report_dir/(self.model['role']+'_'+uuid.uuid4().hex[:10])
        folder.mkdir()
        self.namespace = '/gravity_'+uuid.uuid4().hex[:10]
        self.update(directory=str(folder))
        self.math = self._create_math()
        self.events = (folder/'events.jsonl').open('w')
        (folder/'motor_limits.json').write_text(json.dumps(self.motor_limits,ensure_ascii=False,indent=2))
        self._event('motor_feedback_limits',limits_rad_s=self.motor_limits['feedback_velocity_limits'],
                    source=self.motor_limits['source'],observed_at=self.motor_limits['observed_at'])
        csv_file = (folder/'telemetry.csv').open('w',newline='')
        writer = csv.writer(csv_file)
        self.csv_writer = writer
        writer.writerow(['time','active','fresh']+[f'{kind}_{n}' for kind in ('q','gravity','command','feedback_effort') for n in JOINTS]+
                        ['tuning_revision','tuning_pending']+[f'velocity_{n}' for n in JOINTS])
        (folder/'parameters.yaml').write_text(yaml.safe_dump(self.params,sort_keys=False))
        (folder/'model.json').write_text(json.dumps({k:self.model[k] for k in ('role','arm','urdf_path','urdf_sha256','joints')},ensure_ascii=False,indent=2))
        # Keep the cross-window device lock until all native processes have exited.
        with ExitStack() as resources:
            try:
                if not self.snapshot()['mock']:
                    resources.enter_context(arm_operation(self.model['arm']))
                    verify_usb_identity(self.model['arm'])
                self.stack = self._create_stack(folder,with_rviz)
                self.stack.start()
                # Import service types/create DDS clients while still read-only.
                # These clients belong to the persistent session node/namespace,
                # so they survive replacement of its controller manager.
                self.home = self._create_home()
                self._prepare_extra_clients()
                from rclpy.executors import SingleThreadedExecutor
                self.executor = SingleThreadedExecutor(context=self.stack.context)
                self.executor.add_node(self.stack.node)
                self.stack.pump = self._pump
                self.stack.node.create_subscription(JointState,self.namespace+'/joint_states',self._joint_states,self._feedback_qos())
                self.stack.node.create_subscription(DynamicJointState,self.namespace+'/dynamic_joint_states',self._diagnostics,self._feedback_qos())
                self._wait(lambda:self._controllers_ready() and self._fresh())
                self.update(ready=True,busy=False,status=self._ready_status(),release_confirmed=self._fresh(disabled=True))
                self._event('ready',mock=self.snapshot()['mock'])
                last_sample = 0.
                while not self.stop_event.is_set():
                    self._pump()
                    self.stack.check()
                    if self.stack.close_requested:
                        self.stack.close_requested=False;self.stop()
                    if self.close_event.is_set():
                        self._close_normally()
                        if self.stop_event.is_set(): break
                    if self.release_event.is_set():
                        while not self.commands.empty(): self.commands.get_nowait()
                        self._release()
                        if self.snapshot().get('tuning_pending'):
                            self.update(tuning_status='参数提交已取消')
                        self.release_event.clear()
                        self.update(busy=False,tuning_pending=False)
                    try: command = self.commands.get_nowait()
                    except queue.Empty: command = None
                    if command == 'enable':
                        try: self._enable()
                        except EnableBlocked as exc:
                            self.update(busy=False,status='尚未开启补偿，反馈保持连接',notice=str(exc))
                            self._event('enable_rejected',message=str(exc))
                        except InterruptedError:
                            if self.output_started: self._release()
                            self.update(busy=False)
                    elif isinstance(command,tuple) and command[0] == 'tune':
                        try: self._tune(command[1])
                        except ValueError as exc:
                            self.update(busy=False,tuning_pending=False,tuning_error=str(exc),tuning_status='参数被拒绝，原参数继续生效')
                            self._event('tuning_rejected',message=str(exc))
                        except InterruptedError:
                            if self.output_started: self._release()
                            self.update(busy=False,tuning_pending=False,applied_parameters=deepcopy(self.params),
                                        tuning_status='已停止补偿；最新接受参数将在再次开启时生效')
                    elif command is not None:
                        self._extra_command(command)
                    self._idle_extra()
                    if time.monotonic()-last_sample > .05:
                        self._sample(writer)
                        csv_file.flush()
                        last_sample = time.monotonic()
                        if self.snapshot()['active'] and not self._fresh_after_pending(enabled=True):
                            raise RuntimeError('控制期间电机反馈超时或报告故障，正在停止输出')
            except InterruptedError:
                self._event('cancelled')
            except Exception as exc:
                self.update(error=str(exc),status='会话异常，正在释放控制器')
                self._event('error',message=str(exc),traceback=traceback.format_exc())
            finally:
                if self.stack:
                    if self.output_started and not self.snapshot().get('release_confirmed'):
                        try: self._release()
                        except Exception as exc:
                            self.update(release_confirmed=False,error=self.snapshot()['error']+'；停止核验：'+str(exc))
                            self._event('release_unverified',message=str(exc))
                    if self.executor:
                        self.executor.shutdown()
                        self.executor = None
                    try: self.stack.close()
                    except Exception as exc:
                        self.update(error=self.snapshot()['error']+'；进程退出：'+str(exc))
                if self.snapshot().get('release_confirmed') is not True:
                    self.update(close_failed=True,notice='会话已退出，但电机失能未确认；请托住机械臂并使用硬件急停或断开电机电源')
                self.update(running=False,ready=False,active=False,busy=False,tuning_pending=False,
                            status='会话已结束' if not self.snapshot()['error'] else '会话已结束，请查看错误与日志')
                self._event('closed',release_confirmed=self.snapshot().get('release_confirmed'))
                csv_file.close()
                self.events.close()
                (folder/'result.json').write_text(json.dumps(self.snapshot(),ensure_ascii=False,indent=2,default=str))

    def _create_stack(self, folder, with_rviz):
        return GravityStack(self.model,self.params,folder,self.namespace,self.snapshot()['mock'],with_rviz)

    def _create_math(self):return GravityModel(self.model)

    def _startup_controller(self):return CONTROLLER

    def _ready_status(self):return '仅反馈连接，尚未开启补偿；点击“开启重力补偿”开始'

    def _enabled_status(self):
        return '离线补偿计算运行中' if self.snapshot()['mock'] else '重力补偿运行中，可手动拖动所选机械臂'

    def _create_home(self):
        from gravity_home import GravityHome
        return GravityHome(self)

    def _prepare_extra_clients(self):
        pass

    def _feedback_qos(self):
        return 10

    def _extra_command(self, command):
        raise ValueError('未知会话命令')

    def _idle_extra(self):
        pass
