import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
from a6_y_vr_teleop.a5_kinematics import A5Kinematics
from a6_y_vr_teleop.geometry_guard import GeometryGuard
from a6_y_vr_teleop.input_protocol import make_packet, decode
from a6_y_vr_teleop.real_bridge import RealBridge, identifier, DISABLED, ARMED_HOLD, TELEOP, PAUSED
from a6_y_vr_teleop.commissioning import commissioning_issues, reference_limit_issues

ROOT = Path(__file__).resolve().parents[3]
Q = np.array([0., 1., 1., 0., .4, 0., -.015])
NOW = 10**12


@pytest.fixture(scope='module')
def resources():
    cfg = yaml.safe_load((ROOT/'src/a6_y_vr_teleop/config/a5_vr_real.yaml').read_text())
    urdf = ROOT/'src/A5/urdf/A5.urdf'
    kin = A5Kinematics(urdf, ROOT/'src/a6_y_controllers/config/a5_motion_limits.yaml', cfg)
    return kin, GeometryGuard(kin, urdf, cfg), cfg


def state(now=NOW, **extra):
    s = dict(clock_id=22, controller_epoch=11, monotonic_ns=now, feedback_valid=True,
             actual_positions=Q.copy(), actual_velocities=np.zeros(7), position_age=np.zeros(7),
             feedback_age=np.zeros(7), state=DISABLED, producer_id=0, session_id=0, takeover_id=1,
             anchor_positions=Q[:6].copy(), reference_positions=Q[:6].copy(),
             anchor_monotonic_ns=now, accepted_sequence=0)
    s.update(extra); return SimpleNamespace(**s)


def packet(now=NOW, seq=1, held=False, edge=0, pos=(0, 0, 0), **extra):
    return make_packet('live-test', seq, now, pos, squeeze=held, clutch_epoch=edge,
                       schema_version=2, purpose='real', source_kind='live', **extra)


def bridge(resources, mode='execute'):
    arm = RealBridge(*resources, mode, clock=22)
    arm.observe(state(), NOW); arm.receive(packet(), NOW)
    return arm


def takeover(arm):
    arm.receive(packet(NOW+8_000_000, 2, True, 1), NOW+8_000_000)
    s = state(NOW+8_000_000, state=TELEOP, producer_id=arm.producer, session_id=identifier('live-test'),
              takeover_id=9, anchor_monotonic_ns=NOW+8_000_000)
    arm.observe(s, s.monotonic_ns)
    ref, _ = arm.tick(s.monotonic_ns, .008, False)
    s.accepted_sequence = ref['sequence']; return s, ref


def test_readonly_never_publishes_controller_inputs(resources):
    arm = bridge(resources, 'readonly')
    assert arm.tick(NOW, .008, False) == (None, None)
    arm.receive(packet(NOW+8_000_000, 2, True, 1), NOW+8_000_000)
    assert arm.tick(NOW+8_000_000, .008, False) == (None, None)
    assert arm.preview is not None


def test_no_automatic_prepare_arm_and_capture_requires_binding(resources):
    arm = bridge(resources); ref, ctrl = arm.tick(NOW, .008, False)
    assert ref['valid'] and ctrl is None
    arm.receive(packet(NOW+8_000_000, 2, True, 1), NOW+8_000_000)
    arm.state.state = ARMED_HOLD
    assert arm.tick(NOW+8_000_000, .008, False)[1] is None
    arm.state.producer_id = arm.producer; arm.state.session_id = identifier('live-test')
    assert arm.tick(NOW+8_000_000, .008, False)[1]['action'] == 5


def test_first_reference_equals_measured_anchor_and_keeps_gripper(resources):
    arm = bridge(resources); s, ref = takeover(arm)
    np.testing.assert_array_equal(ref['positions'], Q[:6])
    assert arm.anchor_frame.pose_ns >= s.anchor_monotonic_ns
    for i in range(3, 10):
        now = NOW+i*8_000_000; s.monotonic_ns = now
        arm.receive(packet(now, i, True, 1, (0, 0, -.001*i)), now)
        ref, _ = arm.tick(now, .008, False)
        assert ref['valid'], arm.reason
        assert np.max(np.abs(np.array(ref['positions'])-s.reference_positions)) <= .1*.008+1e-10
        assert np.max(np.abs(np.array(ref['positions'])-Q[:6])) <= .03
        assert arm.preview[6] == Q[6]
        s.reference_positions = ref['positions']; s.accepted_sequence = ref['sequence']
    assert np.linalg.norm(np.asarray(ref['positions'])-Q[:6]) > 1e-6


def test_handshake_discards_motion_before_anchor(resources):
    arm = bridge(resources)
    arm.receive(packet(NOW+8_000_000, 2, True, 1, (.05, 0, 0)), NOW+8_000_000)
    arm.observe(state(NOW+9_000_000, state=TELEOP, producer_id=arm.producer,
        session_id=identifier('live-test'), takeover_id=9, anchor_monotonic_ns=NOW+9_000_000), NOW+9_000_000)
    ref, _ = arm.tick(NOW+9_000_000, .008, False)
    assert arm.anchor_frame is None
    arm.state.accepted_sequence = ref['sequence']
    arm.receive(packet(NOW+16_000_000, 3, True, 1, (.06, 0, 0)), NOW+16_000_000)
    ref, _ = arm.tick(NOW+16_000_000, .008, False)
    np.testing.assert_allclose(ref['positions'], Q[:6], atol=1e-12)


def test_cached_pose_cannot_get_fresh_timestamp_and_timeout_latches(resources):
    arm = bridge(resources); s, _ = takeover(arm)
    assert not arm.receive(packet(NOW+16_000_000, 3, True, 1, pose_sequence=2), NOW+16_000_000)
    assert not arm.tick(NOW+16_000_000, .008, False)[0]['valid']
    s.monotonic_ns = NOW+200_000_000
    assert not arm.tick(s.monotonic_ns, .008, False)[0]['valid']


def test_stale_motor_sample_overrules_fresh_ros_state(resources):
    arm = bridge(resources); arm.state.position_age[3] = .1
    assert arm.tick(NOW, .008, False) == (None, None)
    assert arm.latched


def test_replay_virtual_unknown_clock_and_tracking_jump_rejected(resources):
    arm = bridge(resources)
    assert not arm.receive(make_packet('virtual', 2, NOW, (0, 0, 0)), NOW)
    test = json.loads(packet()); test['source_kind'] = 'test'
    assert not arm.receive(json.dumps(test).encode(), NOW)
    arm = bridge(resources); arm.state.clock_id = 99
    assert arm.tick(NOW, .008, False) == (None, None)
    arm = bridge(resources); takeover(arm)
    arm.receive(packet(NOW+16_000_000, 3, True, 1, (1, 0, 0)), NOW+16_000_000)
    assert not arm.tick(NOW+16_000_000, .008, False)[0]['valid']


def test_input_fault_cannot_auto_resume_while_controller_teleops(resources):
    arm = bridge(resources); takeover(arm); arm.pause('test_fault')
    arm.receive(packet(NOW+16_000_000, 3, False, 2), NOW+16_000_000)
    assert arm.latched
    arm.state.state = PAUSED
    arm.receive(packet(NOW+24_000_000, 4, False, 2), NOW+24_000_000)
    assert not arm.latched
    assert arm.tick(NOW+24_000_000, .008, False)[1] is None


def test_geometry_clearance_floor_and_swept_path(resources):
    _, geometry, _ = resources
    assert geometry.check(Q, Q)[0]
    candidate = Q.copy(); candidate[0] += .0005
    assert geometry.check(Q, candidate)[0]
    old = geometry.cfg.get('floor_z_m'); geometry.cfg['floor_z_m'] = 1.
    try: assert not geometry.check(Q, Q)[0]
    finally: geometry.cfg['floor_z_m'] = old
    candidate[0] += .5
    assert geometry.check(Q, candidate) == (False, 'path_too_long')


def test_pending_commissioning_cannot_be_enabled_by_status_flag(resources):
    kin, _, cfg = resources
    ctrl = ROOT/'src/a6_y_controllers/config'
    paths = {name: ctrl/file for name, file in dict(motors='a5_motors.yaml', calibration='a5_calibration.yaml',
        limits='a5_motion_limits.yaml', gravity='a5_gravity_controllers.yaml', controller='a5_vr_joint_impedance_controllers.yaml').items()}
    issues = commissioning_issues({'status': 'verified'}, paths, kin, cfg)
    # The user confirmed joint mapping and waived zero/scale testing. These
    # decisions do not waive gripper geometry, TCP or physical stop acceptance.
    assert not any(i.startswith('joint1:') for i in issues)
    assert any('gripper_gap:' in i for i in issues)
    assert any('fk_spot_checks:' in i for i in issues)
    assert any('axis_mapping:' in i for i in issues)
    assert any('stopping' in i for i in issues)
    assert any('snapshot' in i for i in issues)


@pytest.mark.parametrize('joint,position', [(1, -.035), (2, -.035), (6, .000835), (6, -.040772303)])
def test_real_feedback_beyond_zero_tolerance_is_rejected_without_clipping(resources, joint, position):
    arm = bridge(resources); arm.state.actual_positions[joint] = position
    original = arm.state.actual_positions.copy()
    ref, ctrl = arm.tick(NOW, .008, False)
    assert not ref['valid'] and ctrl is None
    assert arm.reason == 'actual_outside_model_limits_calibration_required'
    np.testing.assert_array_equal(arm.state.actual_positions, original)
    np.testing.assert_array_equal(arm.preview, original)


def test_measured_zero_error_is_preserved_through_bridge_takeover(resources):
    kin, _, cfg = resources
    # Isolate position acceptance from the separately tested geometry envelope.
    clear_geometry = SimpleNamespace(check=lambda begin, end: (True, 'clear'))
    arm = bridge((kin, clear_geometry, cfg))
    q = Q.copy(); q[1] = -.004768444342717615; q[6] = .0003895297016861095
    arm.state.actual_positions = q.copy()
    ref, _ = arm.tick(NOW, .008, False)
    assert ref['valid'], arm.reason
    np.testing.assert_array_equal(arm.preview, q)
    now = NOW+8_000_000
    arm.receive(packet(now, 2, True, 1), now)
    s = state(now, state=TELEOP, producer_id=arm.producer, session_id=identifier('live-test'),
              takeover_id=9, actual_positions=q.copy(), anchor_positions=q[:6].copy(),
              reference_positions=q[:6].copy(), anchor_monotonic_ns=now)
    arm.observe(s, now)
    ref, _ = arm.tick(now, .008, False)
    assert ref['valid'], arm.reason
    np.testing.assert_array_equal(ref['positions'], q[:6])
    s.accepted_sequence = ref['sequence']
    now += 8_000_000; s.monotonic_ns = now
    arm.receive(packet(now, 3, True, 1), now)
    ref, _ = arm.tick(now, .008, False)
    assert ref['valid'], arm.reason
    np.testing.assert_allclose(ref['positions'], q[:6], atol=1e-12, rtol=0.)
    np.testing.assert_array_equal(arm.state.actual_positions, q)
    assert arm.preview[6] == q[6]


def test_zero_tolerance_only_extends_zero_endpoints(resources):
    kin, _, _ = resources
    np.testing.assert_array_equal(kin.zero_tolerance[:6], np.full(6, np.deg2rad(2)))
    assert kin.zero_tolerance[6] == np.deg2rad(2)*.0239
    for i in range(7):
        for direction in (-1, 1):
            q = Q.copy(); q[i] = direction*kin.zero_tolerance[i]
            assert kin.feedback_in_bounds(q)
        for bound in (kin.feedback_lower[i], kin.feedback_upper[i]):
            q = Q.copy(); q[i] = bound
            assert kin.feedback_in_bounds(q)
        q = Q.copy(); q[i] = kin.feedback_lower[i]-1e-9
        assert not kin.feedback_in_bounds(q)
        q[i] = kin.feedback_upper[i]+1e-9
        assert not kin.feedback_in_bounds(q)
        if kin.lower[i] != 0: assert kin.feedback_lower[i] == kin.lower[i]
        if kin.upper[i] != 0: assert kin.feedback_upper[i] == kin.upper[i]


def test_tolerated_anchor_ik_holds_and_moves_inward_without_jump(resources):
    kin, _, _ = resources
    for joint in (1, 2):
        q = Q.copy(); q[joint] = -.03; q[6] = .00038953
        held, _ = kin.step(q, kin.fk(q), .008, allow_zero_tolerance=True)
        np.testing.assert_allclose(held, q, atol=1e-12, rtol=0.)
        with pytest.raises(ValueError, match='nominal limits'):
            kin.step(q, kin.fk(q), .008)
        goal = q.copy(); goal[joint] = .01
        target = kin.fk(goal)
        initial = q[joint]
        for _ in range(20):
            next_q, _ = kin.step(q, target, .008, allow_zero_tolerance=True)
            assert next_q[joint] >= q[joint]
            assert np.max(np.abs(next_q[:6]-q[:6])) <= .1*.008+1e-12
            assert next_q[6] == q[6]
            q = next_q
        assert q[joint] > initial
        lo, hi = kin.reference_bounds(q, True)
        assert lo[joint] == q[joint]  # May not revisit an older, more negative anchor.
        assert hi[6] == q[6]
        outward = q.copy(); outward[joint] -= .005
        next_q, _ = kin.step(q, kin.fk(outward), .008, allow_zero_tolerance=True)
        assert next_q[joint] >= q[joint]
        q[joint] = .001
        assert kin.reference_bounds(q, True)[0][joint] == 0.


@pytest.mark.parametrize('bad', [-.01, float('nan'), float('inf')])
def test_invalid_zero_tolerance_rejected(resources, tmp_path, bad):
    _, _, cfg = resources
    limits = yaml.safe_load((ROOT/'src/a6_y_controllers/config/a5_motion_limits.yaml').read_text())
    limits['joint_limits']['joint2']['zero_tolerance'] = bad
    path = tmp_path/'limits.yaml'; path.write_text(yaml.safe_dump(limits))
    with pytest.raises(ValueError, match='zero tolerance'):
        A5Kinematics(ROOT/'src/A5/urdf/A5.urdf', path, cfg)


def test_zero_tolerance_requires_scalar_per_joint(resources, tmp_path):
    _, _, cfg = resources
    limits = yaml.safe_load((ROOT/'src/a6_y_controllers/config/a5_motion_limits.yaml').read_text())
    for item in limits['joint_limits'].values(): item['zero_tolerance'] = [.001]
    path = tmp_path/'limits.yaml'; path.write_text(yaml.safe_dump(limits))
    with pytest.raises(ValueError, match='zero tolerance'):
        A5Kinematics(ROOT/'src/A5/urdf/A5.urdf', path, cfg)


def test_controller_limits_and_tolerances_must_match_bridge(resources):
    kin, _, _ = resources
    controller = yaml.safe_load((ROOT/'src/a6_y_controllers/config/a5_vr_joint_impedance_controllers.yaml').read_text())
    params = controller['/arm6/a6_y_joint_impedance_controller']['ros__parameters']
    assert reference_limit_issues(params, kin) == []
    for key in ('vr_zero_tolerance', 'vr_lower', 'vr_upper', 'vr_gripper_lower', 'vr_gripper_upper', 'vr_feedback_tolerance'):
        changed = dict(params); changed[key] = None
        assert reference_limit_issues(changed, kin) == ['VR limit mismatch: '+key]


def test_initial_held_grip_cannot_start_readonly_preview(resources):
    arm = RealBridge(*resources, 'readonly', clock=22)
    arm.observe(state(), NOW); arm.receive(packet(held=True, edge=1), NOW)
    arm.tick(NOW, .008, False)
    assert arm.latched and arm.anchor_frame is None


def test_full_position_orientation_and_gripper_preserve_capture_then_follow_intent(resources):
    from a6_y_vr_teleop.position_profile import position_profile
    from scipy.spatial.transform import Rotation
    _,_,cfg=resources
    _,cfg=position_profile('follow',{},cfg,'full')
    urdf=ROOT/'src/A5/urdf/A5.urdf'
    kin=A5Kinematics(urdf,ROOT/'src/a6_y_controllers/config/a5_motion_limits.yaml',cfg)
    arm=RealBridge(kin,GeometryGuard(kin,urdf,cfg),cfg,'execute',clock=22)
    extra=dict(gripper_control=True,gripper_anchor_position=Q[6],gripper_reference_position=Q[6])
    arm.observe(state(**extra),NOW);arm.receive(packet(trigger=.8),NOW)
    now=NOW+8_000_000
    arm.receive(packet(now,2,True,1,trigger=.8),now)
    s=state(now,state=TELEOP,producer_id=arm.producer,session_id=identifier('live-test'),
            takeover_id=9,anchor_monotonic_ns=now,**extra)
    arm.observe(s,now);ref,_=arm.tick(now,.008,False)
    assert ref['valid'] and ref['gripper_control']
    assert ref['gripper_position']==Q[6]
    s.accepted_sequence=ref['sequence']
    for i,trigger in [(3,.8),(4,.9)]:
        now=NOW+i*8_000_000;s.monotonic_ns=now
        arm.receive(packet(now,i,True,1,trigger=trigger,quaternion=Rotation.from_rotvec([0,0,.01]).as_quat()),now)
        ref,_=arm.tick(now,.008,False)
        assert ref['valid'],arm.reason
        if i==3:assert ref['gripper_position']==Q[6]
        else:assert 0<ref['gripper_position']-Q[6]<=cfg['gripper_speed']*cfg['joint_goal_horizon']+1e-12
        s.reference_positions=ref['positions'];s.accepted_sequence=ref['sequence']
    assert Rotation.from_matrix(arm.raw_target[:3,:3]@arm.anchor_tcp[:3,:3].T).magnitude()>.005
    assert np.linalg.norm(np.asarray(ref['positions'])-Q[:6])>1e-6


def test_full_position_short_compute_overrun_does_not_latch_but_stale_input_does(resources, monkeypatch):
    from a6_y_vr_teleop.position_profile import position_profile
    import a6_y_vr_teleop.real_bridge as module
    kin, geometry, cfg = resources
    _, cfg = position_profile('follow', {}, cfg, 'full')
    arm = RealBridge(kin, geometry, cfg, 'execute', clock=22)
    arm.observe(state(), NOW); arm.receive(packet(), NOW)
    # Deterministic 10 ms compute time: over the nominal 8 ms cycle, still
    # within the existing reference/feedback freshness windows.
    readings = iter((1., 1.010))
    monkeypatch.setattr(module.time, 'perf_counter', lambda: next(readings))
    ref, _ = arm.tick(NOW, .008)
    assert ref['valid'] and not arm.latched
    readings = iter((2., 2.001))
    now = NOW+160_000_000
    arm.observe(state(now), now)
    ref, _ = arm.tick(now, .008)
    assert not ref['valid'] and arm.reason == 'input_expired'


@pytest.mark.parametrize('stop_on_stall', [True, False])
def test_unreachable_target_holds_bounded_reference_with_optional_stall_stop(resources, monkeypatch, stop_on_stall):
    from copy import deepcopy
    kin, geometry, cfg = resources
    cfg = deepcopy(cfg)
    cfg.update(stop_on_ik_stall=stop_on_stall, limited_timeout=.008)
    arm = RealBridge(kin, geometry, cfg, 'execute', clock=22)
    arm.observe(state(), NOW); arm.receive(packet(), NOW)
    s, _ = takeover(arm)
    monkeypatch.setattr(kin, 'step', lambda q, target, dt, **kw: (q.copy(), 0.))
    monkeypatch.setattr(kin, 'error', lambda q, target: np.array([.01, 0, 0, 0, 0, 0]))
    for i in (3, 4):
        now = NOW+i*8_000_000; s.monotonic_ns = now
        arm.receive(packet(now, i, True, 1), now)
        ref, _ = arm.tick(now, .008, False)
    assert ref['valid'] is (not stop_on_stall)
    np.testing.assert_array_equal(ref['positions'], Q[:6])
    if stop_on_stall: assert arm.reason == 'ik_stalled'


def test_late_bridge_cycle_does_not_expand_goal_horizon_or_tcp_slew(resources, monkeypatch):
    from a6_y_vr_teleop.position_profile import position_profile
    kin,geometry,cfg=resources
    _,cfg=position_profile('follow',{},cfg,'full')
    arm=RealBridge(kin,geometry,cfg,'execute',clock=22)
    extra=dict(gripper_anchor_position=Q[6],gripper_reference_position=Q[6])
    arm.observe(state(**extra),NOW);arm.receive(packet(),NOW)
    now=NOW+8_000_000
    arm.receive(packet(now,2,True,1),now)
    s=state(now,state=TELEOP,producer_id=arm.producer,session_id=identifier('live-test'),
        takeover_id=9,anchor_monotonic_ns=now,**extra)
    arm.observe(s,now);ref,_=arm.tick(now,.008,False);s.accepted_sequence=ref['sequence']
    intervals=[];original=kin.step
    def step(q,target,dt,**kw):
        intervals.append(dt);return original(q,target,dt,**kw)
    monkeypatch.setattr(kin,'step',step)
    now+=24_000_000;s.monotonic_ns=now
    arm.receive(packet(now,3,True,1,(0,0,-.01),trigger=1.),now)
    ref,_=arm.tick(now,.024,False)
    assert ref['valid'],arm.reason
    assert intervals==[cfg['joint_goal_horizon']]
    assert np.linalg.norm(arm.target[:3,3]-arm.anchor_tcp[:3,3])<=cfg['target_linear_speed']/cfg['update_rate']+1e-12
    assert 0<ref['gripper_position']-Q[6]<=cfg['gripper_speed']*cfg['joint_goal_horizon']+1e-12


@pytest.mark.parametrize('failure', ['exception', 'nonfinite', 'malformed'])
def test_full_ik_failure_holds_controller_reference_then_retries(resources, monkeypatch, failure):
    from a6_y_vr_teleop.position_profile import position_profile
    kin, geometry, cfg = resources
    _, cfg = position_profile('follow', {}, cfg, 'full')
    arm = RealBridge(kin, geometry, cfg, 'execute', clock=22)
    arm.observe(state(), NOW); arm.receive(packet(), NOW)
    # This test explicitly exercises the arm and gripper full-mode handshake.
    extra = dict(gripper_anchor_position=Q[6], gripper_reference_position=Q[6])
    now = NOW+8_000_000
    arm.receive(packet(now, 2, True, 1), now)
    s = state(now, state=TELEOP, producer_id=arm.producer, session_id=identifier('live-test'),
        takeover_id=9, anchor_monotonic_ns=now, **extra)
    arm.observe(s, now); first, _ = arm.tick(now, .008, False)
    s.accepted_sequence = first['sequence']
    # Actual motor feedback lags the reference. Holding actual would reverse it.
    s.reference_positions = Q[:6]+.001
    before = np.r_[s.reference_positions, s.gripper_reference_position]
    original_step = kin.step
    def fail(q, target, dt, **kwargs):
        if failure == 'exception': raise np.linalg.LinAlgError('singular test solve')
        if failure == 'malformed': return {'bad': 'candidate'}, 0.
        return np.full(7, np.nan), 0.
    monkeypatch.setattr(kin, 'step', fail)
    now += 8_000_000; s.monotonic_ns = now
    arm.receive(packet(now, 3, True, 1, (0, 0, -.002)), now)
    ref, _ = arm.tick(now, .008, False)
    assert ref['valid'] and not arm.latched, arm.reason
    assert arm.reason.startswith('holding_ik:')
    np.testing.assert_array_equal(ref['positions'], before[:6])
    assert ref['gripper_position'] == before[6]
    np.testing.assert_allclose(arm.target, kin.fk(before))
    monkeypatch.setattr(kin, 'step', original_step)
    now += 8_000_000; s.monotonic_ns = now
    arm.receive(packet(now, 4, True, 1, (0, 0, -.002)), now)
    ref, _ = arm.tick(now, .008, False)
    assert ref['valid'] and not arm.latched and arm.reason == 'following'
    assert np.max(np.abs(np.asarray(ref['positions'])-before[:6])) > 1e-8
