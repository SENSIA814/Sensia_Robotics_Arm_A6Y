"""Deterministic teleoperation state machine, independent of ROS and sockets."""
import time
import numpy as np
import pinocchio as pin
from .input_protocol import decode
from .pose_mapping import relative_target, limit_target


class VirtualArm:
    def __init__(self, kinematics, config):
        self.kin = kinematics
        self.cfg = config
        self.q = np.asarray(config['home'], dtype=float)
        if self.q.shape != (7,) or not np.isfinite(self.q).all() or np.any(self.q < self.kin.lower) or np.any(self.q > self.kin.upper):
            raise ValueError('home must contain seven finite in-range coordinates')
        self.B = np.asarray(config['vr_to_base'], dtype=float)
        if self.B.shape != (3,3) or not np.allclose(self.B.T@self.B,np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(self.B),1):
            raise ValueError('vr_to_base must be a proper rotation')
        for key in ['input_timeout','position_scale','max_control_period','compute_budget',
                    'target_linear_speed','target_angular_speed','input_translation_jump',
                    'input_rotation_jump','limited_timeout','position_tolerance','orientation_tolerance',
                    'orientation_length_scale','update_rate']:
            if not np.isfinite(config[key]) or config[key] <= 0:
                raise ValueError('invalid positive setting: '+key)
        self.state = 'WAITING'
        self.reason = 'waiting_for_released_grip'
        self.last = None
        self.retired_sessions = set()
        self.raw_target = self.kin.fk(self.q)
        self.target = self.raw_target.copy()
        self.anchor_tcp = self.target.copy()
        self.anchor_frame = None
        self.velocity = np.zeros(7)
        self.sigma = 0.
        self.limited = False
        self.stalled_time = 0.
        self.accepted = self.rejected = self.overruns = 0
        self.last_compute_s = 0.

    def pause(self, reason):
        self.state, self.reason = 'PAUSED', reason
        self.anchor_frame = None
        self.target = self.kin.fk(self.q)
        self.raw_target = self.target.copy()
        self.velocity[:] = 0
        self.stalled_time = 0.

    def receive(self, packet, now_ns):
        try:
            frame = decode(packet, now_ns)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.rejected += 1
            self.pause('invalid_packet')
            return False
        old = self.last
        if old and frame.session != old.session:
            if frame.session in self.retired_sessions or old.fresh(now_ns,self.cfg['input_timeout']):
                self.rejected += 1
                return False
            self.retired_sessions.add(old.session)
            self.pause('session_changed')
            old = None
        if old:
            if frame.sequence <= old.sequence:
                self.rejected += 1
                return False
            if not old.fresh(now_ns,self.cfg['input_timeout']):
                self.pause('input_timeout')
            invalid_order = (frame.event_ns < old.event_ns or frame.pose_ns < old.pose_ns or
                frame.buttons_ns < old.buttons_ns or frame.pose_sequence < old.pose_sequence or
                frame.clutch_epoch < old.clutch_epoch)
            cached_changed = frame.pose_sequence == old.pose_sequence and (
                frame.pose_ns != old.pose_ns or not np.array_equal(frame.position,old.position) or
                not np.allclose(frame.rotation,old.rotation,atol=1e-12))
            if invalid_order or cached_changed:
                self.rejected += 1
                self.pause('inconsistent_sequence')
                return False
        self.last = frame
        self.accepted += 1
        if not frame.fresh(now_ns,self.cfg['input_timeout']):
            self.pause('input_invalid_or_expired')
            return True
        if old:
            edges = frame.clutch_epoch-old.clutch_epoch
            if edges > 1 or (edges == 1) != (frame.squeeze != old.squeeze):
                self.pause('missed_clutch_edge')
                return True
            if frame.pose_sequence > old.pose_sequence and old.valid and (
                np.linalg.norm(frame.position-old.position) > self.cfg['input_translation_jump'] or
                np.linalg.norm(pin.log3(frame.rotation@old.rotation.T)) > self.cfg['input_rotation_jump']):
                self.pause('tracking_jump')
                return True
        # Release is handled BEFORE any IK; even a release/repress within one tick resets the anchor.
        if not frame.squeeze:
            self.state, self.reason = 'HOLD', 'grip_released'
            self.anchor_frame = None
            self.target = self.kin.fk(self.q)
            self.raw_target = self.target.copy()
            self.stalled_time = 0.
            return True
        if self.state == 'HOLD' and old is not None and not old.squeeze:
            self.anchor_frame = frame
            self.anchor_tcp = self.kin.fk(self.q)
            self.target = self.anchor_tcp.copy()
            self.state, self.reason = 'TELEOP', 'following'
        if self.state == 'TELEOP':
            self.raw_target = relative_target(self.anchor_tcp, self.anchor_frame, frame,
                                              self.B, self.cfg['position_scale'])
        return True

    def tick(self, now_ns, dt, enforce_budget=True):
        start = time.perf_counter()
        before = self.q.copy()
        self.velocity[:] = 0
        if not np.isfinite(dt) or dt <= 0 or dt > self.cfg['max_control_period']:
            self.overruns += 1
            self.pause('control_period_exceeded')
            return
        if self.last is not None and not self.last.fresh(now_ns,self.cfg['input_timeout']):
            self.pause('input_timeout')
        if self.state != 'TELEOP':
            self.last_compute_s = time.perf_counter()-start
            return
        self.target = limit_target(self.target, self.raw_target, dt,
                                   self.cfg['target_linear_speed'], self.cfg['target_angular_speed'])
        try:
            candidate, self.sigma = self.kin.step(before,self.target,dt)
            gripper = -0.03*(1.-self.last.trigger)
            candidate[6] = np.clip(gripper,before[6]-self.kin.speed[6]*dt,before[6]+self.kin.speed[6]*dt)
            candidate = np.clip(candidate,self.kin.lower,self.kin.upper)
            e = self.kin.error(candidate,self.target)
            if not np.isfinite(candidate).all() or not np.isfinite(e).all():
                raise ValueError('nonfinite candidate')
            error_large = np.linalg.norm(e[:3])>self.cfg['position_tolerance'] or np.linalg.norm(e[3:])>self.cfg['orientation_tolerance']
            self.limited = bool(error_large and np.linalg.norm(candidate[:6]-before[:6])<1e-6)
            self.stalled_time = self.stalled_time+dt if self.limited else 0.
            if self.stalled_time > self.cfg['limited_timeout']:
                self.pause('target_unreachable_regrip')
                return
        except (ValueError, np.linalg.LinAlgError, RuntimeError):
            self.pause('ik_failed')
            return
        self.last_compute_s = time.perf_counter()-start
        if enforce_budget and self.last_compute_s > self.cfg['compute_budget']:
            self.overruns += 1
            self.pause('compute_budget_exceeded')
            return
        self.q = candidate
        self.velocity = (self.q-before)/dt

    def diagnostics(self, now_ns):
        error = self.kin.error(self.q,self.target)
        return dict(state=self.state, reason=self.reason, limited=self.limited,
            accepted=self.accepted,rejected=self.rejected,overruns=self.overruns,
            pose_age_s=(now_ns-self.last.pose_ns)/1e9 if self.last else -1.,
            compute_ms=self.last_compute_s*1000, sigma_min=self.sigma,
            position_error_m=float(np.linalg.norm(error[:3])),
            orientation_error_rad=float(np.linalg.norm(error[3:])))
