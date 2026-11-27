"""J7 contact latch and bounded teacher torque; no ROS or hardware side effects.

Positions are single-finger model metres (closing is positive). Torque values
here are motor Nm, NOT measured fingertip force. The follower remains POS_VEL;
capturing a reference prevents pursuit, but is not a follower force controller.
"""
from dataclasses import dataclass, asdict
from collections import deque
import math


@dataclass(frozen=True)
class GripperHapticConfig:
    leader_scale: float = .0239
    follower_scale: float = .0239
    follower_open: float = -.04
    follower_closed: float = -.0007
    contact_torque: float = .12
    contact_error: float = .00025
    contact_speed: float = .0015
    contact_dwell: float = .12
    load_torque: float = .25
    load_dwell: float = .06
    release_travel: float = .0003
    hold_compression: float = 0.
    feedback_gain: float = .5
    wall_stiffness: float = 18.0  # Nm per mapped metre of operator penetration
    max_teacher_torque: float = 2.0
    torque_slew: float = .8  # Nm/s, reflected contact component only
    free_damping: float = .008  # Motor Nm / (motor rad/s), opposes hand motion
    free_damping_limit: float = .015
    free_damping_deadband: float = .04  # Motor rad/s, above zero-speed codec noise
    timeout: float = .08

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('Non-finite gripper haptic parameter')
        if any(v <= 0 for k, v in asdict(self).items()
               if k not in ('follower_open', 'follower_closed', 'hold_compression')):
            raise ValueError('Gripper haptic gains, scales and thresholds must be positive')
        if not self.follower_open < self.follower_closed:
            raise ValueError('Gripper haptics requires increasing closing coordinates')
        if not (0 <= self.hold_compression <= .0002 and
                self.max_teacher_torque <= 2.0 and self.contact_torque >= .1 and
                self.torque_slew <= 1. and .04 <= self.timeout <= .1 and
                self.contact_dwell >= .08 and self.load_torque >= .25 and
                self.load_torque > self.contact_torque and self.load_dwell >= .06 and
                self.free_damping <= .01 and self.free_damping_limit <= .02):
            raise ValueError('Gripper haptic parameters exceed bounded trial settings')


class GripperHaptics:
    def __init__(self, config):
        self.c = config
        self.reset()

    def reset(self):
        self.contact = False
        self.candidate_since = None
        self.load_since = None
        self.hold = None
        self.peak = None
        self.rebase = None
        self.torque = 0.
        self.last_time = None
        self.last_raw = None
        self.raw_history = deque()
        self.contact_reason = ''

    def target_after_release(self, raw):
        if self.rebase is None:
            return raw
        anchor, held = self.rebase
        # A release immediately opens from the captured pose, even if the
        # operator had squeezed farther into the wall. Rejoin the original
        # percentage mapping continuously at either software endpoint.
        lo, hi = self.c.follower_open, self.c.follower_closed
        if raw <= lo or raw >= hi:
            self.rebase = None
            return raw
        if raw <= anchor:
            return lo + (raw-lo) * (held-lo) / max(anchor-lo, 1e-9)
        return held + (raw-anchor) * (hi-held) / max(hi-anchor, 1e-9)

    def step(self, now, raw, sent, actual, velocity, motor_torque, active=True,
             leader_velocity=0.):
        c = self.c
        if not all(math.isfinite(v) for v in (now, raw, sent, actual, velocity, motor_torque, leader_velocity)):
            self.reset()
            raise ValueError('Invalid gripper feedback')
        raw = min(c.follower_closed, max(c.follower_open, raw))
        dt = .01 if self.last_time is None else now-self.last_time
        if not active or dt <= 0 or dt > c.timeout:
            self.reset()
            self.last_time = now
            return dict(state='inactive', target=sent, teacher_torque=0., contact=False)
        self.last_time = now
        dt = min(dt, .02)
        self.raw_history.append((now, raw))
        while self.raw_history and self.raw_history[0][0] < now-.06:
            self.raw_history.popleft()
        # Reject deliberate opening; individual encoder steps must not reset a
        # contact dwell. The previous 1 um/tick test missed loaded soft objects.
        opening = raw < max(q for _, q in self.raw_history)-c.release_travel
        self.last_raw = raw
        target = self.target_after_release(raw)
        if not self.contact:
            closing_stall = (not opening and target-actual >= c.contact_error and
                             sent-actual >= c.contact_error and
                             abs(velocity) <= c.contact_speed and
                             motor_torque >= c.contact_torque)
            if closing_stall:
                if self.candidate_since is None:
                    self.candidate_since = now
            else:
                self.candidate_since = None
            # A tightly tracking servo can load a soft object without a large
            # position error or a complete stall. High load has its own dwell.
            loaded = (not opening and motor_torque >= c.load_torque and
                      min(target, sent) >= actual-c.contact_error)
            if loaded:
                if self.load_since is None:
                    self.load_since = now
            else:
                self.load_since = None
            stalled = self.candidate_since is not None and now-self.candidate_since >= c.contact_dwell
            sustained_load = self.load_since is not None and now-self.load_since >= c.load_dwell
            if stalled or sustained_load:
                self.contact = True
                self.contact_reason = 'load' if sustained_load else 'stall'
                self.hold = min(sent, actual+c.hold_compression)
                self.peak = raw
        requested = 0.
        if self.contact:
            self.peak = max(self.peak, raw)
            # Opening wins immediately; zero haptic torque bypasses ramp-down.
            if raw <= self.peak-c.release_travel:
                self.rebase = (self.peak, self.hold)
                self.contact = False
                self.candidate_since = None
                self.load_since = None
                self.contact_reason = ''
                self.torque = 0.
                target = self.target_after_release(raw)
            else:
                target = self.hold  # Fixed at capture: never integrate closure.
                penetration = max(0., raw-self.hold)
                requested = -min(c.max_teacher_torque,
                    c.feedback_gain*max(0., motor_torque-c.contact_torque*.5) +
                    c.wall_stiffness*penetration)
                if leader_velocity < -c.free_damping_deadband*c.leader_scale:
                    requested = self.torque = 0.  # Withdraw wall force on opening intent.
        delta = c.torque_slew*dt
        self.torque += max(-delta, min(delta, requested-self.torque))
        motor_velocity = leader_velocity/c.leader_scale
        damping = -math.copysign(min(c.free_damping_limit,
            c.free_damping*max(0., abs(motor_velocity)-c.free_damping_deadband)), motor_velocity)
        # Damping is memoryless and passive (tau * velocity <= 0), including
        # reversals. Do not slew it through the previous sign and create assist.
        output = max(-c.max_teacher_torque, min(c.max_teacher_torque, self.torque+damping))
        return dict(state='contact' if self.contact else 'free', target=target,
                    teacher_torque=output, contact=self.contact, contact_reason=self.contact_reason,
                    reflected_torque=self.torque, damping_torque=damping)
