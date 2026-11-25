"""Desktop lifecycle for the supplied, unmodified leader_follower_teleop node."""
import csv
import json
import os
from pathlib import Path
import time
import threading
import traceback
import uuid
import numpy as np

from gravity_config import ROOT, load_model
from teaching import TeachingCoordinator as LifecycleCoordinator, DEFAULT_PAIRS
from teaching_logs import SessionLog, software_manifest, write_json
from teaching_port_config import port_parameters, write_mapper_config, CONTROL_HZ, TARGET_HZ, CAN_BITRATE, POSITION_LIMIT_POLICY, position_bounds
from teaching_frequency_log import FrequencyLog
from teaching_step_observer import StepObserverProcess
from teaching_port_session import PortArmSession
from native_stack import NativeStack


class UpstreamRuntime:
    """Observe and enable the archived mapper; never calculate/send joint targets."""
    def __init__(self, owner, pairs, frequency, folder):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, String
        self.owner, self.folder = owner, folder
        self.children, self.streams, self.publishers = [], [], {}
        self.status, self.raw, self.count, self.last_action = {}, {}, {}, {}
        self.enabled = False
        self.step = None
        self.context = Context()
        rclpy.init(args=[],context=self.context,signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('teaching_port_observer',context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.file = (folder/'references.csv').open('w',newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['wall_time','leader','follower','sequence']+
            [f'{kind}_j{i}' for kind in ('raw','safe','leader_q','follower_q') for i in range(1,7)])
        try:
            topics = {}
            for leader,follower in pairs:
                path,config = write_mapper_config(folder,owner.sessions[leader],owner.sessions[follower],frequency)
                t = config['teleop'];self.count[follower]=0
                topics[follower] = {kind:t[kind+'_action_topic'] for kind in ('raw','safe')}
                self.publishers[follower] = self.node.create_publisher(Bool,t['enable_topic'],10)
                self.node.create_subscription(String,t['status_topic'],
                    lambda msg,r=follower:self.on_status(r,msg),10)
                self.node.create_subscription(JointState,t['raw_action_topic'],
                    lambda msg,l=leader,r=follower:self.on_raw(l,r,msg),10)
                self.node.create_subscription(JointState,t['safe_action_topic'],
                    lambda msg,l=leader,r=follower:self.on_action(l,r,msg),10)
                # A separate process uses the upstream node's default ROS context.
                # This stack owns only mapper children, never a hardware manager.
                stack = NativeStack(owner.sessions[follower].model,folder,
                    owner.sessions[follower].namespace+'/upstream_teleop',False,False)
                self.streams.append(stack)
                child = stack.spawn(follower+'_mapper','a6_y_teleop','leader_follower_teleop',
                    ['--ros-args','-p','config_file:='+str(path)])
                self.children.append((follower,child))
            try:
                self.step = StepObserverProcess(folder,dict(pairs=pairs,topics=topics,
                    devices={r:owner.roles[r]['port'] for r in owner.sessions},
                    namespaces={r:s.namespace for r,s in owner.sessions.items()}),owner._log_failure)
            except OSError as exc:
                owner._log_failure(exc)
        except BaseException:
            self.close();raise

    def on_status(self, follower, msg):
        if self.status.get(follower)!=msg.data:
            self.owner._event('upstream_status',role=follower,status=msg.data)
        self.status[follower] = msg.data

    def on_raw(self, leader, follower, msg):
        self.raw[follower] = list(msg.position)

    def on_action(self, leader, follower, msg):
        self.count[follower] += 1
        self.last_action[follower] = time.monotonic()
        source = self.owner.sessions[leader].feedback.snapshot()['q']
        target = self.owner.sessions[follower].feedback.snapshot()['q']
        self.writer.writerow([time.time(),leader,follower,self.count[follower],
            *self.raw.get(follower,[None]*6),*msg.position,*source[:6],*target[:6]])

    def pump(self):
        if self.step:self.step.check()
        self.executor.spin_once(timeout_sec=.005)
        for _ in range(12):self.executor.spin_once(timeout_sec=0.)
        for role,child in self.children:
            if child.poll() is not None:raise RuntimeError(role+' 原程序跟随进程退出，见 mapper 日志')

    def enable(self, value):
        from std_msgs.msg import Bool
        self.enabled = value
        if self.step:self.step.enable(value)
        for publisher in self.publishers.values():publisher.publish(Bool(data=value))

    def ready(self):
        return all(p.get_subscription_count()>0 and self.status.get(r)=='disabled; holding follower'
                   for r,p in self.publishers.items())

    def close(self):
        self.enable(False)
        if self.step:self.step.close()
        for stack in self.streams:stack.close()
        self.executor.shutdown()
        self.node.destroy_node()
        if self.context.ok():self.context.shutdown()
        self.file.close()


class TeachingCoordinator(LifecycleCoordinator):
    close_timeout = 80.  # One deadline for the group, including process cleanup.

    def __init__(self, roles=None, session_factory=PortArmSession, report_dir=None):
        super().__init__(roles,session_factory,report_dir)
        self.release_closer = None

    def _parameters_for_role(self, role):return port_parameters(role,self.roles)

    def start(self, pairs=DEFAULT_PAIRS, frequency=TARGET_HZ, mock=True, with_rviz=False):
        if frequency != TARGET_HZ:
            raise ValueError(f'当前动力示教目标发送频率固定为 {TARGET_HZ} Hz')
        return super().start(pairs,frequency,mock,with_rviz)

    def _close_sessions(self):
        self.update(running=True,phase='closing',status='停止跟随；各臂同时使用原生位置模式回零并核验失能',
                    close_failed=False,close_error='')
        self._hold_sessions()
        pending={};errors={};started=time.monotonic();next_progress=started
        for role,s in self.sessions.items():
            state=s.snapshot()
            if state['running']:
                s.stop();pending[role]=s
            elif state.get('role') and state.get('release_confirmed') is not True:
                errors[role]=state.get('error') or state.get('notice') or '失能未确认'
        while pending:
            if self.emergency.is_set():self._finish_release();return
            for role,s in list(pending.items()):
                state=s.snapshot()
                if state.get('close_failed'):
                    errors[role]=state.get('notice') or state.get('error') or '回零或失能失败'
                elif not state['running']:
                    if state.get('release_confirmed') is not True:
                        errors[role]=state.get('error') or state.get('notice') or '失能未确认'
                    else:self._event('arm_closed',role=role,release_confirmed=True)
                else:continue
                del pending[role]
            elapsed=time.monotonic()-started
            if pending and elapsed>self.close_timeout:
                for role,s in pending.items():
                    s.cancel_home.set()
                    errors[role]='退出等待超时，已请求取消回零；可查看日志或托住后立即失能'
                self._event('close_timeout',level='ERROR',roles=list(pending),elapsed=elapsed)
                break
            if pending and time.monotonic()>=next_progress:
                self.update(status=f'退出回零：{len(pending)} 条臂处理中，已用 {elapsed:.0f}s / 最长 {self.close_timeout:.0f}s；可立即失能')
                next_progress=time.monotonic()+5.
            time.sleep(.02)
        if errors:
            self.update(running=False,phase='error',close_failed=True,
                status='退出未完成；已停止等待，可查看日志、重试回零或托住后立即失能',
                close_error='；'.join(self.roles[r]['name']+'：'+e for r,e in errors.items()))
        else:
            self.update(running=False,phase='stopped',status='示教会话已安全结束，已连接机械臂均确认失能')

    def stop(self):
        previous = self.worker
        if not self.emergency.is_set() or not previous or not previous.is_alive():
            return super().stop()
        # 'released' becomes visible before the releasing worker finishes its
        # final log write. A close in that interval still needs a worker to act
        # on it, even if _finish_release already checked close_after_release.
        self.close_after_release.set()
        self.stop_requested.set()
        self._event('stop_requested')
        if self.release_closer and self.release_closer.is_alive():return
        def finish_close():
            previous.join()
            if self.alive() and self.snapshot()['phase']=='released':self._retry_close()
        self.release_closer = threading.Thread(target=finish_close,daemon=False)
        self.worker = self.release_closer
        self.worker.start()

    def _sample(self, session):
        state = session.control_state(feedback=True)
        hw = state['hardware']
        if not state['active'] or not session._fresh(enabled=True,sample=hw):
            raise RuntimeError(self.roles[state['role']]['name']+'：反馈过期或电机异常，停止整组跟随')
        q=np.asarray(hw['q'][:6],float);v=np.asarray(hw['velocity'][:6],float)
        limits=np.asarray(self.limit_records[state['role']]['feedback_velocity_limits'])
        if not np.isfinite(q).all() or not np.isfinite(v).all() or np.any(np.abs(v)>limits):
            raise RuntimeError(state['role']+'：反馈无效或超过该电机 VMAX 协议范围')
        return q

    def _preflight_start(self):
        # Validate the whole group while every arm is still read-only. Each
        # _enable rechecks fresh feedback again immediately before takeover.
        positions={}
        for role,session in self.sessions.items():
            reason=session.enable_block_reason()
            if reason:raise RuntimeError(self.roles[role]['name']+'：'+reason)
            sample=session.feedback.snapshot()
            try:session._validate_enable_positions(sample['q'])
            except ValueError as exc:raise ValueError(self.roles[role]['name']+'：'+str(exc)) from exc
            positions[role]=dict(position=sample['q'],source_time=sample.get('sample_source_time'))
        self._event('startup_positions_checked',startup_home=False,position_limit_policy=POSITION_LIMIT_POLICY,
            positions=positions)

    def _run(self,pairs,frequency,mock,with_rviz,parameters):
        folder=None;failed=False;runtime=None;timing=None
        try:
            folder=self.report_dir/('teaching_'+uuid.uuid4().hex[:10]);folder.mkdir(parents=True)
            self.update(directory=str(folder))
            self.log=SessionLog(folder)
            rates=dict(control_hz=CONTROL_HZ,per_axis_command_hz=CONTROL_HZ,
                       target_hz=TARGET_HZ,can_bitrate=CAN_BITRATE)
            self.update(configured_rates=rates)
            self._event('session_started',pairs=pairs,frequency=frequency,mock=mock,
                configured_rates=rates,
                implementation='archive_teleop_direct_start',startup_home=False,exit_home=True,home_backend='pos_vel',
                position_limit_policy=POSITION_LIMIT_POLICY,
                leader_backend='mit_gravity',follower_backend='stock_direct_position')
            write_json(folder/'software.json',software_manifest(ROOT))
            write_json(folder/'motor_limits.json',self.limit_records)
            write_json(folder/'session.json',dict(pairs=pairs,frequency=frequency,mock=mock,
                configured_rates=rates,
                roles={r:self.roles[r] for r in self.sessions},startup_home=False,exit_home=True,home_backend='pos_vel',
                position_limit_policy=POSITION_LIMIT_POLICY,
                position_bounds={r:position_bounds(load_model(r,self.roles)) for r in self.sessions},
                leader_backend='mit_gravity',follower_backend='stock_direct_position',
                application_log=os.environ.get('A6Y_TEACHING_APP_LOG')))
            try:timing=FrequencyLog(folder)
            except OSError as exc:self._log_failure(exc)
            for role,s in self.sessions.items():
                if self.stop_requested.is_set():raise InterruptedError()
                s.start(role,mock,with_rviz,parameters[role])
            self._await(lambda:all(s.snapshot()['ready'] for s in self.sessions.values()),45.)
            self._save()
            self._preflight_start()
            self.update(phase='preparing',status='启动不回零：运动臂准备位置跟随，示教臂准备重力补偿')
            # The mapper first holds disabled; only enable after ALL modes are ready.
            for _,follower in pairs:self._operation(self.sessions[follower],'follow')
            for leader,_ in pairs:self._operation(self.sessions[leader],'gravity')
            # Start the original node after both endpoints finish connecting.
            # Replacing a read-only manager must not look like a teaching timeout.
            runtime=UpstreamRuntime(self,pairs,frequency,folder)
            deadline=time.monotonic()+15.
            while not runtime.ready():
                self._check();runtime.pump()
                if time.monotonic()>deadline:raise TimeoutError('原程序跟随进程未就绪，见 mapper 日志')
            runtime.count={r:0 for r in runtime.count}
            self._event('startup_follow_reference',startup_home=False,
                references=[dict(leader=leader,follower=follower,
                    teacher_position=self.sessions[leader].feedback.snapshot()['q'][:6],
                    follower_position=self.sessions[follower].feedback.snapshot()['q'][:6]) for leader,follower in pairs])
            runtime.enable(True)
            started=time.monotonic();last_flush=started
            self.update(phase='following',status='示教臂：MIT 重力补偿；运动臂：POS_VEL 跟随示教臂当前位置')
            while not self.stop_requested.is_set():
                self._check();runtime.pump()
                now=time.monotonic()
                for s in self.sessions.values():self._sample(s)
                for _,follower in pairs:
                    status=runtime.status.get(follower,'')
                    # Startup DDS delivery gets one original feedback timeout.
                    if now-started>.5 and status!='running':
                        raise RuntimeError(follower+' 原程序跟随停止：'+status)
                    if now-started>.5 and now-runtime.last_action.get(follower,0)>.5:
                        raise RuntimeError(follower+' 原程序目标发布超时')
                if now-last_flush>.2:
                    runtime.file.flush()
                    if timing:
                        try:timing.sample(self.sessions,'following')
                        except Exception as exc:
                            self._log_failure(exc)
                            try:timing.close()
                            except Exception:pass
                            timing=None
                    self.update(packets=min(runtime.count.values()),actual_hz=min(runtime.count.values())/max(.02,now-started))
                    last_flush=now
        except InterruptedError:self._event('cancelled')
        except Exception as exc:
            failed=True
            self._event('first_failure',level='ERROR',message=str(exc),traceback=traceback.format_exc(),phase=self.snapshot()['phase'])
            self.update(error=str(exc),status='操作失败，停止整组跟随；保留反馈与日志')
            if folder:
                try:write_json(folder/'first_failure.json',dict(message=str(exc),traceback=traceback.format_exc(),state=self.snapshot()))
                except Exception as log_exc:self._log_failure(log_exc)
        finally:
            # Hold before stopping DDS processes; no queued mapper command may
            # reach an active direct-position controller during protected exit.
            if runtime:
                try:runtime.enable(False)
                except Exception as exc:self._event('mapper_disable_failed',level='ERROR',message=str(exc))
            for session in self.sessions.values():
                if hasattr(session,'cancel_home'):session.cancel_home.set()
            # Do not wait for a faulted arm's release timeout before freezing
            # healthy peers. Repeat only for an arm finishing an operation.
            self._hold_sessions()
            deadline=time.monotonic()+4.
            while any(s.snapshot()['busy'] for s in self.sessions.values()) and time.monotonic()<deadline:
                time.sleep(.01)
            self._hold_sessions()
            if runtime:
                try:runtime.close()
                except Exception as exc:self._event('mapper_close_failed',level='ERROR',message=str(exc))
            if self.emergency.is_set():self._finish_release()
            elif failed:
                for s in self.sessions.values():
                    if s.snapshot()['running'] and not s.snapshot()['active'] and not s.snapshot()['busy']:s.stop()
                unresolved=any(s.snapshot()['active'] or (s.snapshot().get('role') and s.snapshot().get('release_confirmed') is not True)
                               for s in self.sessions.values())
                self.update(running=False,phase='error',close_failed=bool(unresolved),
                    status='整组已停止，未自动重试回零；可导出日志、回零退出或托住后立即失能')
            else:self._close_sessions()
            self._save()
            if timing:
                try:timing.close()
                except Exception as exc:self._log_failure(exc)
