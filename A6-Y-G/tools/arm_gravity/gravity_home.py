"""MoveIt plans/checks only; the existing C++ gravity controller executes torque."""
from copy import deepcopy
import json
from pathlib import Path
import time

import numpy as np
import yaml

from gravity_config import JOINTS, CONTROLLER
from native_model import native_model, CONFIG
from trajectory import from_ros
from motion_model import MAX_SPEED, MAX_ACCEL

# Absolute requested limits, also enforced on the cubic sent to the controller.
# Previously MoveIt used .15/.3 with 50% scaling: effective .075/.15.
HOME_SPEED = .10  # rad/s
HOME_ACCEL = .20  # rad/s²
HOME_STIFFNESS_RAMP = 2.  # stationary seconds, matched by the C++ controller


def home_trajectory(message, model):
    message=deepcopy(message)
    merged=None
    if len(message.points)>=3:
        previous,end=message.points[-2:]
        stamp=lambda p:p.time_from_start.sec+p.time_from_start.nanosec*1e-9
        gap=stamp(end)-stamp(previous)
        times=np.array([stamp(p) for p in message.points])
        values=[previous.positions,end.positions,previous.velocities,end.velocities]
        # TOTG samples every 0.1 s and may append a <1 ms final remainder.
        # Merge only a near-stationary terminal sample; retain both endpoints.
        # The resulting cubic is revalidated and collision-checked below.
        if (0<gap<.001 and np.isfinite(times).all() and np.all(np.diff(times[:-1])>=.001)
                and all(len(v)==6 and np.isfinite(v).all() for v in values)
                and max(abs(np.asarray(end.positions)-previous.positions))<1e-6
                and max(abs(np.asarray(end.positions)))<=1e-4
                and max(abs(np.asarray(previous.velocities)))<.001
                and max(abs(np.asarray(end.velocities)))<1e-6):
            merged=dict(time=stamp(previous),positions=list(previous.positions),
                        velocities=list(previous.velocities),terminal_interval=gap)
            del message.points[-2]
    plan=from_ros(message, model, velocity_scaling=HOME_SPEED/MAX_SPEED,
                  acceleration_scaling=HOME_ACCEL/MAX_ACCEL, retime=True)
    plan.merged_terminal_sample=merged
    return plan


def home_failure_text(message):
    reasons={'start_position_mismatch':'起点位置偏移', 'start_velocity':'开始回零时仍在运动',
             'invalid_control_period':'控制周期异常', 'heartbeat_timeout':'回零心跳超时',
             'tracking_error':'关节跟踪误差超限', 'joint_speed':'关节反馈速度超限',
             'joint_lower_limit':'关节低于下限', 'joint_upper_limit':'关节高于上限',
             'impedance_solver_invalid':'回零阻抗求解失败', 'zero_settling_timeout':'零位稳定等待超时'}
    for code, explanation in reasons.items():
        if message.startswith('Return aborted: '+code+' '):
            return explanation+'；'+message.removeprefix('Return aborted: ')
    return message


def planning_start(model, measured):
    """Only normalize encoder residuals within the return's arrival tolerance."""
    q=np.asarray(measured,dtype=float).copy()
    if q.shape!=(7,) or not np.isfinite(q).all(): raise ValueError('回零起点反馈无效')
    residuals=[]
    for i,n in enumerate(JOINTS):
        entry=model['joints'][n];bounded=float(np.clip(q[i],entry['lower'],entry['upper']))
        tolerance=.005 if i<6 else .0005
        if abs(q[i]-bounded)>tolerance: raise ValueError(n+' 超出模型范围，不能回零；请复核零位标定')
        if bounded!=q[i]:
            residuals.append(dict(joint=n,measured=float(q[i]),planning=bounded,tolerance=tolerance))
            q[i]=bounded
    return q,residuals


class GravityHome:
    def __init__(self, session):
        from a6_y_teleop_msgs.srv import GravityHome as Service
        from moveit_msgs.srv import GetMotionPlan, GetStateValidity
        self.s = session
        self.revision = 0
        self.unconfirmed = False
        self.attempt = 0
        self.planning_attempt = 0
        self.client = session.stack.node.create_client(Service, session.namespace+'/'+CONTROLLER+'/home')
        self.plan_client = session.stack.node.create_client(GetMotionPlan, session.namespace+'/plan_kinematic_path')
        self.valid_client = session.stack.node.create_client(GetStateValidity, session.namespace+'/check_state_validity')

    def close(self):
        for c in (self.client, self.plan_client, self.valid_client):
            self.s.stack.node.destroy_client(c)

    def pump(self):
        self.s._pump()
        self.s.stack.check()
        if getattr(self.s.stack,'home_planner_error',''):
            raise RuntimeError(self.s.stack.home_planner_error)
        if self.s.release_event.is_set(): raise InterruptedError('回零已取消，请求立即失能')
        if not self.s._fresh_after_pending(enabled=True): raise RuntimeError('回零期间反馈失效，禁止继续运动')
        if time.monotonic()-getattr(self,'sample_at',0)>.05:
            self.s._sample(self.s.csv_writer);self.sample_at=time.monotonic()

    def call(self, client, request, timeout=5, check=True):
        if not client.wait_for_service(timeout_sec=.1): raise RuntimeError('回零服务尚未就绪')
        future=client.call_async(request); deadline=time.monotonic()+timeout
        while not future.done():
            if check: self.pump()
            else: self.s._pump()
            if time.monotonic()>deadline: raise TimeoutError('回零服务响应超时')
        if future.exception(): raise future.exception()
        return future.result()

    def cancel(self):
        from a6_y_teleop_msgs.srv import GravityHome as Service
        if self.unconfirmed: raise RuntimeError('回零启动结果不确定，必须停用控制器并失能')
        if self.revision:
            response=self.call(self.client,Service.Request(command=2,revision=self.revision),check=False)
            if not response.success: raise RuntimeError('回零取消被拒绝：'+response.message)
            revision=response.revision;deadline=time.monotonic()+.5
            while response.applied_revision!=revision or response.phase!=0:
                if time.monotonic()>deadline: raise TimeoutError('回零取消未获控制循环确认')
                response=self.call(self.client,Service.Request(command=0,revision=revision),timeout=.4,check=False)
            self.revision=0

    @staticmethod
    def robot_state(q):
        from moveit_msgs.msg import RobotState
        state=RobotState();state.joint_state.name=list(JOINTS);state.joint_state.position=list(map(float,q))
        return state

    def plan(self):
        from ament_index_python.packages import get_package_share_directory
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.srv import GetMotionPlan, GetStateValidity
        s=self.s
        model=native_model(s.model['role'],s.roles)
        if not getattr(s.stack,'home_planner_started',False):
            s.stack.home_planner_error=''
            ompl=yaml.safe_load((Path(get_package_share_directory('moveit_configs_utils'))/'default_configs/ompl_planning.yaml').read_text())
            ompl.update(planner_configs={'RRTConnect':{'type':'geometric::RRTConnect','range':0.}},
                        arm={'planner_configs':['RRTConnect'],'longest_valid_segment_fraction':.005})
            limits=deepcopy(model['native_limits'])
            for n in JOINTS[:6]: limits['joint_limits'][n].update(max_velocity=HOME_SPEED,max_acceleration=HOME_ACCEL)
            params=dict(robot_description=model['description'],robot_description_semantic=model['semantic'],
                        robot_description_kinematics=yaml.safe_load((CONFIG/'kinematics.yaml').read_text()),
                        robot_description_planning=limits,planning_pipelines=['ompl'],default_planning_pipeline='ompl',
                        ompl=ompl,allow_trajectory_execution=False,publish_robot_description_semantic=True,
                        disable_capabilities='move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction')
            path=s.stack.directory/'home_move_group.yaml'
            path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
            (s.stack.directory/'home_collision_policy.json').write_text(json.dumps(model['collision_policy'],ensure_ascii=False,indent=2))
            s.stack.spawn('move_group','moveit_ros_move_group','move_group',params=path)
            s.stack.home_planner_started=True
        deadline=time.monotonic()+15
        while not self.plan_client.service_is_ready() or not self.valid_client.service_is_ready():
            self.pump()
            if time.monotonic()>deadline: raise TimeoutError('MoveIt 回零规划器未就绪')
        q=s.feedback.measured();qplan,residuals=planning_start(model,q)
        if residuals: s._event('home_planning_boundary_residual',joints=residuals)
        goal=qplan.copy();goal[:6]=0.
        for values in (qplan,goal):
            req=GetStateValidity.Request(group_name='arm',robot_state=self.robot_state(values))
            if not self.call(self.valid_client,req).valid: raise ValueError('当前位置或零位发生碰撞，退出已取消')
        req=GetMotionPlan.Request();r=req.motion_plan_request
        r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnect';r.allowed_planning_time=5.
        r.num_planning_attempts=1;r.max_velocity_scaling_factor=1.;r.max_acceleration_scaling_factor=1.
        r.start_state=self.robot_state(qplan)
        r.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=0.,
            tolerance_above=.00001,tolerance_below=.00001,weight=1.) for n in JOINTS[:6]])]
        response=self.call(self.plan_client,req,timeout=8).motion_plan_response
        if response.error_code.val!=1: raise ValueError('MoveIt 回零规划失败：'+str(response.error_code.val))
        self.planning_attempt+=1
        raw_file=f'home_planner_response_{self.planning_attempt:03d}.json'
        trajectory=response.trajectory.joint_trajectory
        (s.stack.directory/raw_file).write_text(json.dumps(dict(
            joint_names=list(trajectory.joint_names),points=[dict(
                time=p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
                positions=list(p.positions),velocities=list(p.velocities)) for p in trajectory.points]),indent=2))
        plan=home_trajectory(response.trajectory.joint_trajectory,model)
        plan.home_planner_file=raw_file
        if np.max(abs(plan.q[-1]))>1e-4: raise ValueError('回零规划终点不是关节零位')
        # Check the SAME cubic interpolation sent to the torque controller.
        for t in np.linspace(0,plan.duration,int(np.ceil(plan.duration/.05))+1):
            state=qplan.copy();state[:6]=plan.sample(t)[0]
            req=GetStateValidity.Request(group_name='arm',robot_state=self.robot_state(state))
            if not self.call(self.valid_client,req).valid: raise ValueError('回零插值路径碰撞，退出已取消')
        plan.passive_source=qplan.copy();plan.check_start(s.feedback.measured())
        self.record_plan(plan)
        return plan

    def record_plan(self, plan):
        self.attempt+=1
        plan.home_file=f'home_plan_{self.attempt:03d}.json'
        data=dict(attempt=self.attempt,planned_at=time.time(),times=plan.times.tolist(),
                  positions=plan.q.tolist(),velocities=plan.v.tolist(),
                  goal='J1..J6=0',gripper_command=False,stiffness_ramp_seconds=HOME_STIFFNESS_RAMP,
                  max_velocity=HOME_SPEED,max_acceleration=HOME_ACCEL,
                  peak_velocity=plan.peak_velocity.tolist(),peak_acceleration=plan.peak_acceleration.tolist(),
                  parameters=self.s.params,
                  planner_file=getattr(plan,'home_planner_file',None),
                  merged_terminal_sample=getattr(plan,'merged_terminal_sample',None),
                  execution_times=[0.]+(plan.times+HOME_STIFFNESS_RAMP).tolist(),
                  execution_positions=np.vstack([plan.q[0],plan.q]).tolist(),
                  execution_velocities=np.vstack([np.zeros(6),plan.v]).tolist())
        encoded=json.dumps(data,ensure_ascii=False,indent=2)
        (self.s.stack.directory/plan.home_file).write_text(encoded)
        (self.s.stack.directory/'home_plan.json').write_text(encoded)

    def execute(self, plan):
        from a6_y_teleop_msgs.srv import GravityHome as Service
        s=self.s
        s.update(home_stage='preview',status='回零路径已检查，1 秒后执行；可点击立即失能取消')
        until=time.monotonic()+1.
        while time.monotonic()<until: self.pump()
        plan.check_start(s.feedback.measured())
        # Two stationary seconds smoothly introduce position stiffness.
        req=Service.Request(command=1,times=[0.]+(plan.times+HOME_STIFFNESS_RAMP).tolist(),
                            positions=np.vstack([plan.q[0],plan.q]).ravel().tolist(),
                            velocities=np.vstack([np.zeros(6),plan.v]).ravel().tolist())
        self.unconfirmed=True
        result=self.call(self.client,req);self.revision=result.revision;self.unconfirmed=False
        if not result.success: raise ValueError(result.message)
        s.update(home_stage='executing',status='正在缓慢回零，夹爪保持只读；可点击立即失能')
        s._event('home_executing',revision=self.revision,duration=plan.duration+HOME_STIFFNESS_RAMP,
                 plan_file=plan.home_file,peak_velocity=plan.peak_velocity.tolist(),
                 peak_acceleration=plan.peak_acceleration.tolist())
        deadline=time.monotonic()+plan.duration+15.;next_status=0.
        while time.monotonic()<deadline:
            self.pump()
            if time.monotonic()<next_status: continue
            next_status=time.monotonic()+.1
            result=self.call(self.client,Service.Request(command=0,revision=self.revision),timeout=.4)
            if result.revision!=self.revision or result.phase==3:
                s._event('home_aborted',revision=self.revision,plan_file=plan.home_file,message=result.message,
                         positions=s.feedback.measured().tolist(),velocity=list(s.feedback.snapshot()['velocity']))
                raise RuntimeError('回零控制器中止：'+home_failure_text(result.message))
            if result.phase==2:
                q=s.feedback.measured();v=np.asarray(s.feedback.snapshot()['velocity'])
                if np.max(abs(q[:6]))>.005 or np.max(abs(v[:6]))>.03: raise RuntimeError('回零状态与实测位置不一致')
                s._event('home_reached',positions=q.tolist(),tolerance_rad=.005)
                return
        raise TimeoutError('回零未在规定时间到位，退出已取消')
