"""Physical-reference computation. This module never enables hardware or arms VR."""
import json
from pathlib import Path
import secrets
import time
import numpy as np
import pinocchio as pin
from .input_protocol import decode
from .pose_mapping import relative_target, limit_target

DISABLED, HOLD, ARMED_HOLD, TELEOP, STOPPING, PAUSED, FAULT = range(7)
STATE_NAMES = ('DISABLED', 'HOLD', 'ARMED_HOLD', 'TELEOP', 'STOPPING', 'PAUSED', 'FAULT')
REASON_NAMES = ('NONE', 'NOT_PREPARED', 'PREPARED', 'ARMED', 'FOLLOWING', 'RELEASED',
    'INPUT_TIMEOUT', 'REFERENCE_TIMEOUT', 'INVALID_REFERENCE', 'SESSION_CHANGED', 'MISSED_EDGE',
    'TRACKING_ERROR', 'ACTUAL_LIMIT', 'ACTUAL_SPEED', 'STOP_TIMEOUT', 'STOPPED',
    'PAUSE_REQUEST', 'DISARM_REQUEST', 'BAD_FEEDBACK', 'BAD_PERIOD', 'REQUEST_REJECTED',
    'WAITING_FIRST_REFERENCE', 'SATURATION', 'COMPUTE_FAILURE', 'SESSION_TRAVEL_LIMIT', 'AUTO_REARMED',
    'TRACKING_LIMITED')


def identifier(value):
    result = 14695981039346656037
    for byte in value.encode(): result = ((result ^ byte)*1099511628211) & ((1 << 64)-1)
    return result or 1


def clock_id():
    return identifier(Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def age_ok(now, stamp, limit):
    return 0 < stamp <= now and now-stamp <= limit*1e9


class RealBridge:
    def __init__(self, kin, geometry, config, mode, clock=None):
        if mode not in ('readonly', 'mock', 'execute'): raise ValueError('invalid mode')
        self.kin, self.geometry, self.cfg, self.mode = kin, geometry, config, mode
        self.clock = clock_id() if clock is None else clock
        self.producer = secrets.randbits(63) or 1
        self.B = np.asarray(config['vr_to_base'], dtype=float)
        if self.B.shape != (3, 3) or not np.allclose(self.B.T@self.B, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(self.B), 1):
            raise ValueError('invalid vr_to_base rotation')
        for key in ('input_timeout', 'feedback_timeout', 'max_control_period', 'compute_budget',
                    'position_scale', 'target_linear_speed', 'target_angular_speed', 'max_joint_lead',
                    'input_translation_jump', 'input_rotation_jump',
                    'limited_timeout', 'update_rate'):
            if not np.isfinite(config[key]) or config[key] <= 0: raise ValueError('invalid '+key)
        radius=config['local_workspace_radius']
        if radius is not None and (not np.isfinite(radius) or radius<=0): raise ValueError('invalid local_workspace_radius')
        self.gripper_control=bool(config.get('gripper_control',False))
        for key, default in (('stop_on_compute_overrun', True), ('stop_on_ik_stall', True),
                             ('geometry_checking', True), ('hold_on_ik_failure', False),
                             ('limit_reference_lead', False)):
            if type(config.get(key, default)) is not bool: raise ValueError('invalid '+key)
        self.geometry_checking = config.get('geometry_checking', True)
        self.hold_on_ik_failure = config.get('hold_on_ik_failure', False)
        self.limit_reference_lead = config.get('limit_reference_lead', False)
        self.joint_goal_horizon = config.get('joint_goal_horizon', 0.)
        if not np.isfinite(self.joint_goal_horizon) or self.joint_goal_horizon < 0:
            raise ValueError('invalid joint_goal_horizon')
        if self.joint_goal_horizon and self.limit_reference_lead:
            raise ValueError('trajectory goals cannot use the executable-reference lead filter')
        if self.geometry_checking and geometry is None: raise ValueError('geometry checker required')
        self.stop_on_compute_overrun = config.get('stop_on_compute_overrun', True)
        self.gripper_trigger_active=False
        if self.gripper_control:
            for key in ('gripper_speed','gripper_max_lead'):
                if not np.isfinite(config[key]) or config[key]<=0: raise ValueError('invalid '+key)
        self.frame = self.previous = self.state = None
        self.live = False
        self.retired = set()
        self.sequence = self.first_sequence = self.takeover = 0
        self.anchor_frame = self.anchor_tcp = self.target = self.raw_target = self.preview = None
        self.latched = True
        self.reason = 'waiting_for_feedback_and_input'
        self.accepted = self.rejected = 0
        self.compute_s = self.stalled_s = 0.
        self.last_capture_ns = 0

    def pause(self, reason):
        self.latched, self.reason = True, reason
        self.anchor_frame = None
        self.stalled_s = 0.

    def receive(self, packet, now):
        try:
            frame = decode(packet, now, expected_schema=2)
            live = json.loads(packet)['source_kind'] == 'live'
            if not live and self.mode != 'mock': raise ValueError('test input rejected')
            old = self.frame
            if old and frame.session != old.session:
                if frame.session in self.retired or old.fresh(now, self.cfg['input_timeout']):
                    raise ValueError('concurrent/retired session')
                self.retired.add(old.session)
                self.pause('session_changed')
                old = None
            if old:
                if frame.sequence <= old.sequence: raise ValueError('old_sequence')
                if (frame.pose_sequence < old.pose_sequence or frame.pose_ns < old.pose_ns or
                        frame.buttons_ns < old.buttons_ns or frame.event_ns < old.event_ns or
                        frame.clutch_epoch < old.clutch_epoch): raise ValueError('input_order')
                if frame.pose_sequence == old.pose_sequence and (frame.pose_ns != old.pose_ns or
                        not np.array_equal(frame.position, old.position) or
                        not np.allclose(frame.rotation, old.rotation, atol=1e-12)):
                    raise ValueError('changed_cached_pose')
                edges = frame.clutch_epoch-old.clutch_epoch
                if edges > 1 or (edges == 1) != (frame.squeeze != old.squeeze):
                    self.pause('missed_clutch_edge')
                if old.valid and frame.pose_sequence > old.pose_sequence and (
                        np.linalg.norm(frame.position-old.position) > self.cfg['input_translation_jump'] or
                        np.linalg.norm(pin.log3(frame.rotation@old.rotation.T)) > self.cfg['input_rotation_jump']):
                    self.pause('tracking_jump')
            self.previous, self.frame, self.live = old, frame, live
            self.accepted += 1
            if not frame.fresh(now, self.cfg['input_timeout']): self.pause('input_expired')
            # Release clears only the input latch. The C++ gate decides whether
            # the configured stream may rearm or still needs a local ARM.
            elif not frame.squeeze and (self.mode == 'readonly' or self.state is None or
                    self.state.state in (DISABLED, HOLD, PAUSED, ARMED_HOLD)):
                self.latched = False
                self.anchor_frame = None
                self.reason = 'grip_released'
            return True
        except (ValueError, KeyError, TypeError, OverflowError):
            self.rejected += 1
            self.pause('invalid_packet')
            return False

    def observe(self, state, now):
        if self.state and state.controller_epoch == self.state.controller_epoch and state.monotonic_ns <= self.state.monotonic_ns:
            return
        if state.clock_id != self.clock: self.pause('clock_mismatch')
        if self.state and state.controller_epoch != self.state.controller_epoch:
            self.pause('controller_restarted')
            self.takeover = self.first_sequence = 0
        self.state = state
        if self.geometry_checking and self.cfg.get('zero_pose_collision_tolerance', {}).get('enabled', False):
            if state.clock_id == self.clock and state.feedback_valid and age_ok(now, state.monotonic_ns, self.cfg['feedback_timeout']):
                self.geometry.observe_controller(state.controller_epoch, state.state, state.session_origin_positions)

    def feedback_ok(self, now):
        s = self.state
        return bool(s is not None and s.clock_id == self.clock and s.feedback_valid and
                    age_ok(now, s.monotonic_ns, self.cfg['feedback_timeout']) and
                    np.isfinite(s.actual_positions).all() and np.isfinite(s.actual_velocities).all() and
                    all(np.isfinite(a) and 0 <= a <= self.cfg['feedback_timeout'] for a in
                        list(s.position_age)+list(s.feedback_age)))

    def tick(self, now, dt, enforce_budget=True):
        """Return (reference fields, optional CAPTURE request fields).

        readonly returns no control/reference message; preview is separate state.
        PREPARE and ARM are deliberately absent from this module.
        """
        start = time.perf_counter()
        if self.geometry_checking and self.cfg.get('zero_pose_collision_tolerance', {}).get('enabled', False):
            self.geometry.last_tolerated_pairs = []
        if not self.feedback_ok(now):
            self.pause('feedback_stale_or_invalid')
            return None, None
        s, frame = self.state, self.frame
        actual = np.asarray(s.actual_positions).copy()
        self.preview = actual.copy()
        if self.target is None: self.target = self.raw_target = self.kin.fk(actual)
        if not np.isfinite(dt) or dt <= 0 or dt > self.cfg['max_control_period']:
            self.pause('bridge_period_exceeded')
        fresh = frame is not None and frame.fresh(now, self.cfg['input_timeout'])
        if not fresh: self.pause('input_expired')
        if not self.kin.feedback_in_bounds(actual):
            self.pause('actual_outside_model_limits_calibration_required')
        self.sequence += 1
        command = None
        positions = actual[:6].copy()
        gripper_position=float(actual[6])
        moving = False
        if fresh and not self.latched:
            if self.mode == 'readonly':
                if frame.squeeze:
                    if self.anchor_frame is None:
                        self.anchor_frame, self.anchor_tcp = frame, self.kin.fk(actual)
                        self.target = self.anchor_tcp.copy()
                    moving = True
            else:
                if s.state == ARMED_HOLD and frame.squeeze and s.producer_id == self.producer and s.session_id == identifier(frame.session):
                    if now-self.last_capture_ns > 30_000_000:
                        command = dict(action=5, controller_epoch=s.controller_epoch,
                                       request_id=now, generated_monotonic_ns=now, producer_id=self.producer,
                                       session_id=identifier(frame.session), clutch_epoch=frame.clutch_epoch)
                        self.last_capture_ns = now
                if s.state == TELEOP and frame.squeeze and s.producer_id == self.producer and s.session_id == identifier(frame.session):
                    if s.takeover_id != self.takeover:
                        self.takeover, self.first_sequence = s.takeover_id, self.sequence
                        self.gripper_trigger_active=False
                        self.anchor_frame = None
                        q_anchor = np.r_[s.anchor_positions, s.gripper_anchor_position if self.gripper_control else actual[6]]
                        self.anchor_tcp = self.kin.fk(q_anchor)
                        self.target = self.anchor_tcp.copy()
                    # Discard motion before the controller's actual-pose capture.
                    if self.anchor_frame is None and frame.pose_ns >= s.anchor_monotonic_ns:
                        self.anchor_frame = frame
                    if self.anchor_frame is None or s.accepted_sequence < self.first_sequence:
                        positions = np.asarray(s.anchor_positions).copy()
                        if self.gripper_control: gripper_position=float(s.gripper_anchor_position)
                    else:
                        moving = True
        try:
            if moving:
                # A slow solve must not increase the next swept path and make
                # that solve even slower. Drop catch-up motion; keep real wall
                # time above for freshness and period checks.
                step_dt = min(dt, 1/self.cfg['update_rate'])
                before = actual.copy()
                if self.mode != 'readonly': before[:6] = s.reference_positions
                if self.gripper_control and self.mode != 'readonly': before[6]=s.gripper_reference_position
                self.raw_target = relative_target(self.anchor_tcp, self.anchor_frame, frame,
                                                  self.B, self.cfg['position_scale'], self.cfg['rotation_enabled'])
                if self.cfg['local_workspace_radius'] is not None and np.linalg.norm(self.raw_target[:3, 3]-self.anchor_tcp[:3, 3]) > self.cfg['local_workspace_radius']:
                    raise ValueError('local_workspace_exceeded')
                lo, hi = self.cfg.get('workspace_lower'), self.cfg.get('workspace_upper')
                if lo is not None and (np.any(self.raw_target[:3, 3] < lo) or np.any(self.raw_target[:3, 3] > hi)):
                    raise ValueError('workspace_exceeded')
                target = limit_target(self.target, self.raw_target, step_dt, self.cfg['target_linear_speed'], self.cfg['target_angular_speed'])
                ik_failure = ''
                try:
                    # For native full position control, solve a nearby future
                    # goal. Using only an 8 ms step here makes the independent
                    # C++ braking limiter slow down again at every update.
                    # TCP target slew still uses the real step_dt.
                    goal_dt = self.joint_goal_horizon or step_dt
                    candidate, sigma = self.kin.step(before, target, goal_dt, allow_zero_tolerance=True,
                        tracking_position=actual[:6] if self.limit_reference_lead else None,
                        tracking_limit=self.cfg['max_joint_lead'] if self.limit_reference_lead else None)
                    candidate = np.asarray(candidate, dtype=float)
                    if candidate.shape != before.shape or not np.isfinite(candidate).all() or not np.isfinite(sigma):
                        raise ValueError('invalid IK candidate')
                except (TypeError, ValueError, OverflowError, RuntimeError, np.linalg.LinAlgError) as error:
                    if not self.hold_on_ik_failure: raise
                    # Piper-style retry: keep the controller's last reference,
                    # not lagging measured feedback, and discard target windup.
                    candidate, sigma = before.copy(), 1.
                    target = self.kin.fk(before)
                    ik_failure = str(error) or type(error).__name__
                if self.gripper_control and not ik_failure:
                    # Keep captured opening until a deliberate trigger change.
                    # Then absolute trigger maps release=open, full press=closed.
                    self.gripper_trigger_active |= abs(frame.trigger-self.anchor_frame.trigger)>.02
                    desired=(self.kin.lower[6]+frame.trigger*(self.kin.upper[6]-self.kin.lower[6])
                             if self.gripper_trigger_active else before[6])
                    gripper_step=self.cfg['gripper_speed']*(self.joint_goal_horizon or step_dt)
                    candidate[6]=before[6]+np.clip(desired-before[6],-gripper_step,gripper_step)
                    if not self.joint_goal_horizon and abs(candidate[6]-actual[6])>self.cfg['gripper_max_lead']:
                        raise ValueError('gripper_reference_lead_exceeded')
                if self.gripper_control: gripper_position=float(candidate[6])
                lead_limited = bool(not ik_failure and self.limit_reference_lead and self.kin.last_tracking_limited)
                # The bounded solver may land exactly on the band; ignore only
                # floating-point roundoff here, not physical tracking error.
                lead_epsilon = 1e-12 if self.kin.bounded_ik else 0.
                if not self.joint_goal_horizon and np.max(np.abs(candidate[:6]-actual[:6])) > self.cfg['max_joint_lead']+lead_epsilon:
                    if not self.limit_reference_lead:
                        raise ValueError('reference_lead_exceeded')
                    # Planning backpressure: shorten the IK step instead of
                    # latching every time a valid reference outruns feedback.
                    # One scale preserves the joint-space step direction.
                    # If measured feedback moved outside the soft band, never
                    # increase that existing gap or snap the reference to it;
                    # C++ still independently stops on its tracking-error cap.
                    lead = before[:6]-actual[:6]
                    delta = candidate[:6]-before[:6]
                    envelope = np.maximum(self.cfg['max_joint_lead'], np.abs(lead))
                    scale = 1.
                    for gap, change, bound in zip(lead, delta, envelope):
                        if change > 0: scale = min(scale, max(0., (bound-gap)/change))
                        elif change < 0: scale = min(scale, max(0., (bound+gap)/(-change)))
                    candidate[:6] = before[:6]+scale*delta
                    # Discard unreachable target accumulation while waiting for
                    # the motors. The live relative hand target is retried next tick.
                    target = self.kin.fk(candidate)
                    lead_limited = True
                if lead_limited:
                    target = self.kin.fk(candidate)
                # Both the commanded reference and the actual-to-candidate path
                # must fit the envelope; the physical path is measured later.
                if self.geometry_checking:
                    for begin in (before, actual):
                        valid, why = self.geometry.check(begin, candidate)
                        if not valid: raise ValueError(why)
                error = self.kin.error(candidate, target)
                stalled = (np.linalg.norm(error[:3]) > self.cfg['position_tolerance'] and
                           (np.linalg.norm(candidate[:6]-before[:6]) < 1e-6 or sigma < 1e-5))
                self.stalled_s = self.stalled_s+dt if stalled else 0.
                if self.cfg.get('stop_on_ik_stall', True) and self.stalled_s > self.cfg['limited_timeout']:
                    raise ValueError('ik_stalled')
                self.target, positions, self.preview = target, candidate[:6].copy(), candidate
                self.reason = ('holding_ik:'+ik_failure if ik_failure else
                               'limited_reference_lead' if lead_limited else
                               'preview' if self.mode == 'readonly' else 'following')
            elif fresh and not self.latched and self.geometry_checking:
                valid, why = self.geometry.check(actual, actual)
                if not valid: raise ValueError(why)
            self.compute_s = time.perf_counter()-start
            if enforce_budget and self.stop_on_compute_overrun and self.compute_s > self.cfg['compute_budget']:
                raise ValueError('compute_budget_exceeded')
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
            self.pause(str(error))
            command = None
            positions, self.preview = actual[:6].copy(), actual.copy()
            gripper_position=float(actual[6])
        if self.mode == 'readonly': return None, None
        ref = dict(clock_id=self.clock, controller_epoch=s.controller_epoch, takeover_id=s.takeover_id,
                   producer_id=self.producer, session_id=identifier(frame.session) if frame else 0,
                   sequence=self.sequence, pose_sequence=frame.pose_sequence if frame else 0,
                   clutch_epoch=frame.clutch_epoch if frame else 0, generated_monotonic_ns=now,
                   pose_monotonic_ns=frame.pose_ns if frame else 0,
                   buttons_monotonic_ns=frame.buttons_ns if frame else 0, feedback_monotonic_ns=s.monotonic_ns,
                   valid=bool(fresh and not self.latched), squeeze=bool(frame and frame.squeeze),
                   live=self.live, positions=positions.tolist(),gripper_control=self.gripper_control,
                   gripper_position=gripper_position)
        return ref, command
