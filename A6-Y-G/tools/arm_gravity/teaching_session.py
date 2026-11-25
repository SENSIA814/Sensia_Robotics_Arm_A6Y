"""Teaching lifecycle using the existing gravity driver, home planner and exit gate.

Each session owns its ROS executor. The coordinator only publishes immutable
targets and queues operations; it never spins another worker's executor.
"""
from concurrent.futures import Future
import time
import traceback
import numpy as np

from gravity_config import CONTROLLER, load_profile
from gravity_session import GravitySession


class TeachingArmSession(GravitySession):
    # Two latest-only feedback streams plus a service response. Avoid 32 empty
    # wait-set rebuilds per tick per arm competing for the Python interpreter.
    pump_callbacks=6

    def _feedback_qos(self):
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        return QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)

    def control_state(self, feedback=False):
        # The 50 Hz sender does not need to copy tuning profiles, plots or
        # telemetry summaries. Keep its read path small and internally coherent.
        with self.lock:
            state={k:self.state.get(k) for k in ('role','running','active','error','follow_phase','follow_message')}
        if feedback:state['hardware']=self.feedback.snapshot()
        return state

    def start(self, role, mock=True, with_rviz=False, parameters=None):
        p=dict(parameters or load_profile(role,self.roles))
        p['teaching_follow_enabled']=True
        # Match the supplied working teleop: teacher MIT gravity, follower
        # POS_VEL. The motor's position loop owns follower tracking and homing.
        p['teaching_position_output']=role.startswith('motion_')
        self.follow_revision=0
        self.follow_client=self.reference_publisher=None
        self.next_follow_status=0.
        self.update(follow_phase=0,follow_message='',startup_homed=False,
                    backend='pos_vel' if p['teaching_position_output'] else 'mit_gravity')
        super().start(role,mock,with_rviz,p)

    def _prepare_extra_clients(self):
        from a6_y_teleop_msgs.srv import TeachingControl
        from a6_y_teleop_msgs.msg import TeachingReference
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.follow_client=self.stack.node.create_client(TeachingControl,self.namespace+'/'+CONTROLLER+'/teaching')
        self.reference_publisher=self.stack.node.create_publisher(TeachingReference,
            self.namespace+'/'+CONTROLLER+'/teaching_reference',
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))

    def submit(self, operation):
        with self.lock:
            if not self.state['ready'] or self.state['busy'] or self.state['closing_requested']:
                raise ValueError('机械臂未就绪或正在处理其他操作')
            self.state.update(busy=True,notice='',close_failed=False)
        future=Future()
        try: self.commands.put_nowait(('teaching',operation,future))
        except Exception:
            self.update(busy=False)
            raise
        return future

    def _follow_call(self, command):
        from a6_y_teleop_msgs.srv import TeachingControl
        if not self.follow_client.wait_for_service(timeout_sec=.2):
            raise RuntimeError('动力示教服务未就绪')
        request=TeachingControl.Request(command=command,revision=self.follow_revision)
        future=self.follow_client.call_async(request)
        self._wait(future.done,timeout=.8)
        response=future.result()
        if not response.success: raise RuntimeError(response.message)
        self.follow_revision=response.revision
        if command:
            expected={1:1,2:2,3:0}[command]
            deadline=time.monotonic()+.6
            while response.applied_revision!=self.follow_revision or response.phase!=expected:
                if time.monotonic()>deadline: raise TimeoutError('动力示教切换未获控制循环确认')
                future=self.follow_client.call_async(TeachingControl.Request(revision=self.follow_revision))
                self._wait(future.done,timeout=.4)
                response=future.result()
                if not response.success or response.revision!=self.follow_revision or response.phase==3:
                    raise RuntimeError('动力示教切换失败：'+response.message)
        self.update(follow_phase=response.phase,follow_message=response.message)
        return response

    def _extra_command(self, command):
        _,operation,future=command
        try:
            if self.close_event.is_set() or self.release_event.is_set():
                raise InterruptedError('操作已取消')
            if operation=='home':
                if not self.snapshot()['active']: self._enable()
                self.update(busy=True,home_stage='planning',status='启动：规划并执行回零')
                if np.max(np.abs(self.feedback.measured()[:6]))<=.005:
                    # MoveIt returns a one-point no-op when already at zero.
                    # Apply the same stable-arrival check as normal exit instead
                    # of inventing motion or accepting a degenerate trajectory.
                    deadline=time.monotonic()+.3
                    while time.monotonic()<deadline:
                        self.home.pump()
                        if (np.max(np.abs(self.feedback.measured()[:6]))>.005 or
                            np.max(np.abs(self.feedback.snapshot()['velocity'][:6]))>.03):
                            raise ValueError('零位未保持稳定，取消启动示教')
                    self._event('home_reached',positions=self.feedback.measured().tolist(),already_at_zero=True)
                else:
                    self.home.execute(self.home.plan())
                    self.home.cancel()
                self._follow_call(2)
                self.update(startup_homed=True,home_stage='complete',status='启动回零完成，保持零位等待其他臂')
                self._event('startup_homed')
            else:
                self._follow_call({'follow':1,'hold':2,'gravity':3}[operation])
                self.update(status={'follow':'运动臂 POS_VEL 增量位置跟随','hold':'位置保持，跟随已停止',
                                    'gravity':'示教臂重力补偿，可手动拖动'}[operation])
                self._event('teaching_'+operation,revision=self.follow_revision)
            future.set_result(True)
        except Exception as exc:
            if self.home:
                try: self.home.cancel()
                except Exception: self.release_event.set()
            self.update(notice=str(exc),startup_homed=False if operation=='home' else self.snapshot().get('startup_homed',False))
            self._event('teaching_operation_failed',operation=operation,message=str(exc),traceback=traceback.format_exc())
            future.set_exception(exc)
        finally:
            self.update(busy=False)

    def publish_reference(self, positions, sequence, stamp_ns):
        from a6_y_teleop_msgs.msg import TeachingReference
        self.reference_publisher.publish(TeachingReference(revision=self.follow_revision,
            sequence=sequence,stamp_ns=stamp_ns,positions=list(map(float,positions))))

    def _idle_extra(self):
        state=self.control_state()
        if state['active'] and state.get('follow_phase',0) and time.monotonic()>=self.next_follow_status:
            self._follow_call(0)
            self.next_follow_status=time.monotonic()+(.1 if state['follow_phase']==1 else .5)

    def _close_normally(self):
        try:
            if self.snapshot()['active'] and self.snapshot().get('follow_phase',0):
                self._follow_call(2)
                self._follow_call(3)
        except Exception as exc:
            self.close_event.clear()
            self.update(busy=False,closing_requested=False,close_failed=True,notice=str(exc))
            return
        super()._close_normally()
