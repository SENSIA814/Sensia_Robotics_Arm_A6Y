"""Validated cubic interpolation, independent of ROS and motor transport."""
import time
import xml.etree.ElementTree as ET
import numpy as np

from motion_model import ARM_JOINTS, GRIPPER_JOINTS, JOINTS, joint_speed, joint_accel, joint_tolerance


class Trajectory:
    def __init__(self, names, points, model, velocity_scaling=1., acceleration_scaling=1.,
                 recover_boundary=False, retime=False):
        if not all(np.isfinite(v) and .1 <= v <= 1 for v in (velocity_scaling, acceleration_scaling)):
            raise ValueError('速度与加速度比例需在 10%～100% 之间')
        self.velocity_scaling, self.acceleration_scaling = velocity_scaling, acceleration_scaling
        if len(names) != len(set(names)) or set(names) not in (set(ARM_JOINTS), set(GRIPPER_JOINTS)) or len(points) < 2:
            raise ValueError(f'轨迹必须包含 J1～J6 或夹爪，且至少两个时间点：{names}, 点数 {len(points)}')
        self.names = list(ARM_JOINTS if len(names) == 6 else GRIPPER_JOINTS)
        self.indices = [JOINTS.index(n) for n in self.names]
        self.group = 'arm' if len(names) == 6 else 'gripper'
        self.tolerance = np.array([joint_tolerance(n) for n in self.names])
        self.start_tolerance = np.array([joint_tolerance(n, .03, .0005) for n in self.names])
        self.tracking_tolerance = np.array([joint_tolerance(n, .15, .003) for n in self.names])
        self.end_tolerance = np.array([joint_tolerance(n, .02, .0005) for n in self.names])
        order = [names.index(n) for n in self.names]
        self.times = np.asarray([p['time'] for p in points], float)
        if (not np.isfinite(self.times).all() or abs(self.times[0]) > 1e-6
                or np.any(np.diff(self.times) < .001) or self.times[-1] > 120):
            raise ValueError('轨迹时间必须从 0 单调增加，持续时间不超过 120 秒')
        if any(len(p['positions']) != len(names) or len(p['velocities']) != len(names) for p in points):
            raise ValueError('轨迹点位置与速度长度必须匹配关节数')
        self.q = np.asarray([p['positions'] for p in points], float)[:, order]
        self.v = np.asarray([p['velocities'] for p in points], float)[:, order]
        if not np.isfinite(self.q).all() or not np.isfinite(self.v).all():
            raise ValueError('轨迹包含非有限数值')
        if np.any(np.abs(self.v[[0, -1]]) > self.tolerance * 2):
            raise ValueError('轨迹必须从静止开始并以静止结束')
        self.lower = np.array([model['joints'][n]['lower'] for n in self.names])
        self.upper = np.array([model['joints'][n]['upper'] for n in self.names])
        self.command_lower, self.command_upper = self.lower.copy(), self.upper.copy()
        if recover_boundary:
            # Only a one-axis inward recovery from an observed <=0.5 mm residual.
            # This does not widen legal targets or change calibration/feedback.
            if (self.group != 'gripper' or len(points) != 2 or
                    np.any(self.q[-1] < self.lower) or np.any(self.q[-1] > self.upper) or
                    np.any(self.q[0] < self.lower - .0005) or np.any(self.q[0] > self.upper + .0005) or
                    np.any(self.v != 0)):
                raise ValueError('夹爪边界恢复必须从 ±0.5 mm 内的反馈单调返回合法目标')
            self.command_lower = np.minimum(self.lower, self.q[0])
            self.command_upper = np.maximum(self.upper, self.q[0])
        h = np.diff(self.times)[:, None]
        self.a = (2 * self.q[:-1] - 2 * self.q[1:] + h * (self.v[:-1] + self.v[1:])) / h**3
        self.b = (-3 * self.q[:-1] + 3 * self.q[1:] - h * (2 * self.v[:-1] + self.v[1:])) / h**2
        self.c, self.d = self.v[:-1], self.q[:-1]
        self.peak_velocity = np.zeros(len(self.names))
        self.peak_acceleration = np.zeros(len(self.names))
        # Evaluate polynomial extrema, not just the supplied knot positions.
        for i, width in enumerate(h[:, 0]):
            for j, name in enumerate(self.names):
                a, b, c, d = self.a[i, j], self.b[i, j], self.c[i, j], self.d[i, j]
                ts = [0., width]
                ts += [float(r.real) for r in np.roots([3*a, 2*b, c]) if abs(r.imag) < 1e-9 and 0 < r.real < width]
                values = [((a*t+b)*t+c)*t+d for t in ts]
                if min(values) < self.command_lower[j] - self.tolerance[j] or max(values) > self.command_upper[j] + self.tolerance[j]:
                    raise ValueError('轨迹插值超出关节范围')
                vt = [0., width] + ([-b/(3*a)] if abs(a) > 1e-12 and 0 < -b/(3*a) < width else [])
                self.peak_velocity[j] = max(self.peak_velocity[j], max(abs(3*a*t*t+2*b*t+c) for t in vt))
                self.peak_acceleration[j] = max(self.peak_acceleration[j], max(abs(6*a*t+2*b) for t in (0., width)))
        v_ratio = max(self.peak_velocity / [joint_speed(n) * velocity_scaling for n in self.names])
        a_ratio = max(self.peak_acceleration / [joint_accel(n) * acceleration_scaling for n in self.names])
        if retime:
            factor = max(1., v_ratio, np.sqrt(a_ratio)) * 1.001
            self.times *= factor
            self.v /= factor
            self.a /= factor**3
            self.b /= factor**2
            self.c = self.v[:-1]
            self.peak_velocity /= factor
            self.peak_acceleration /= factor**2
            if self.times[-1] > 120:
                raise ValueError('按当前速度／加速度重定时后超过 120 秒，请缩短路径')
        elif v_ratio > 1.01:
            raise ValueError('轨迹速度超限')
        elif a_ratio > 1.05:
            raise ValueError('轨迹加速度超限')
        self.created = time.monotonic()
        self.duration = float(self.times[-1])

    def sample(self, elapsed):
        if elapsed >= self.duration:
            return self.q[-1].copy(), np.zeros(len(self.names))
        i = max(0, int(np.searchsorted(self.times, elapsed, side='right')) - 1)
        t = max(0., elapsed - self.times[i])
        return ((self.a[i]*t + self.b[i])*t + self.c[i])*t + self.d[i], (3*self.a[i]*t+2*self.b[i])*t+self.c[i]

    def check_start(self, measured):
        if time.monotonic() - self.created > 30.:
            raise ValueError('规划已超过 30 秒，请重新规划')
        if np.any(np.abs(np.asarray(measured)[self.indices] - self.q[0]) > self.start_tolerance):
            raise ValueError('真机已偏离规划起点，请重新规划')
        if hasattr(self, 'passive_source') and any(
                abs(measured[i] - self.passive_source[i]) > joint_tolerance(n, .03, .0005)
                for i, n in enumerate(JOINTS) if i not in self.indices):
            raise ValueError('未规划的关节偏离碰撞检查时的位置，请重新规划')


def from_ros(message, model, **options):
    names = list(message.joint_names)
    points = [
        {'time': p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
         'positions': list(p.positions), 'velocities': list(p.velocities)} for p in message.points]
    # MoveIt includes the passive mimic finger in gripper trajectories. Validate
    # its relation before dropping it from the motor command path (one motor).
    for joint in ET.fromstring(model['description']).findall('joint'):
        mimic = joint.find('mimic')
        name = joint.attrib['name']
        if mimic is None or name not in names:
            continue
        if mimic.attrib['joint'] not in names:
            raise ValueError('轨迹包含没有主关节的 mimic 关节')
        i, j = names.index(name), names.index(mimic.attrib['joint'])
        multiplier, offset = float(mimic.get('multiplier', 1)), float(mimic.get('offset', 0))
        for point in points:
            if len(point['positions']) != len(names) or len(point['velocities']) != len(names):
                raise ValueError('mimic 轨迹点长度错误')
            for key, shift in (('positions', offset), ('velocities', 0)):
                values = point[key]
                if not np.isfinite(values).all() or abs(values[i] - (multiplier * values[j] + shift)) > 1e-6:
                    raise ValueError('MoveIt 夹爪 mimic 关系不一致')
                values.pop(i)
        names.pop(i)
    return Trajectory(names, points, model, **options)


def gripper_motion(start, end, model, speed, acceleration):
    if not all(np.isfinite(x) for x in (start, end, speed, acceleration)) or not all(.1 <= x <= 1 for x in (speed, acceleration)):
        raise ValueError('夹爪位置或速度／加速度参数无效')
    distance = abs(end - start)
    duration = 1.05 * max(.2, 1.5 * distance / (joint_speed(GRIPPER_JOINTS[0]) * speed),
                          np.sqrt(6 * distance / (joint_accel(GRIPPER_JOINTS[0]) * acceleration)))
    return Trajectory(list(GRIPPER_JOINTS), [
        {'time': 0., 'positions': [start], 'velocities': [0.]},
        {'time': duration, 'positions': [end], 'velocities': [0.]}], model,
        velocity_scaling=speed, acceleration_scaling=acceleration, recover_boundary=True)


def gripper_profile(message, model, speed, acceleration=None):
    """Retiming a one-dimensional path cannot add new positions to its swept interval.

    Humble TOTG can leave a sharp final deceleration on millimetre-scale paths.
    Use a bounded cubic with zero endpoint velocity; preview the same samples
    that the driver interpolates, and still collision-check before execution.
    """
    if list(message.joint_names) != list(GRIPPER_JOINTS) or not message.points or not .1 <= speed <= 1:
        raise ValueError('夹爪路径格式或速度比例无效')
    if any(len(p.positions) != 1 or not np.isfinite(p.positions).all() for p in message.points):
        raise ValueError('夹爪路径包含无效位置')
    start, end = message.points[0].positions[0], message.points[-1].positions[0]
    plan = gripper_motion(start, end, model, speed, speed if acceleration is None else acceleration)
    from trajectory_msgs.msg import JointTrajectoryPoint
    message.points = []
    for elapsed in np.linspace(0, plan.duration, int(np.ceil(plan.duration / .05)) + 1):
        point = JointTrajectoryPoint()
        q, v = plan.sample(elapsed)
        point.positions, point.velocities = q.tolist(), v.tolist()
        ns = round(float(elapsed) * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
        message.points.append(point)
    return plan
