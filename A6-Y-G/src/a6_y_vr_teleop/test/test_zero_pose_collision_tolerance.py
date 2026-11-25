from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
from a6_y_vr_teleop.a5_kinematics import A5Kinematics
from a6_y_vr_teleop.geometry_guard import GeometryGuard, DistanceCertificates
from a6_y_vr_teleop.real_bridge import RealBridge, identifier
from a6_y_vr_teleop.input_protocol import make_packet

ROOT = Path(__file__).resolve().parents[3]
FOLDED = np.array([.103189, -.00476844, .0055314, -.306134, -.059701, -.021935, .000261214])


def resources(enabled=True, pairs=None):
    cfg = yaml.safe_load((ROOT/'src/a6_y_vr_teleop/config/a5_vr_real.yaml').read_text())
    cfg['zero_pose_collision_tolerance'] = dict(enabled=enabled,
        zero_band_rad=float(np.deg2rad(2.)), anchor_band_rad=float(np.deg2rad(2.)),
        pairs=pairs if pairs is not None else [['link1', 'link4'], ['link1', 'link5'], ['link2', 'link4']])
    urdf = ROOT/'src/A5/urdf/A5.urdf'
    kin = A5Kinematics(urdf, ROOT/'src/a6_y_controllers/config/a5_motion_limits.yaml', cfg)
    return kin, GeometryGuard(kin, urdf, cfg), cfg


def test_measured_folded_pose_needs_explicit_tolerance_and_preserves_coordinates():
    _, strict, _ = resources(False)
    assert not strict.check(FOLDED, FOLDED)[0]
    _, tolerant, _ = resources()
    original = FOLDED.copy()
    assert tolerant.check(FOLDED, FOLDED)[0]
    np.testing.assert_array_equal(FOLDED, original)
    assert len(tolerant.last_tolerated_pairs) == 3


def test_tolerance_requires_controller_anchor_for_movement_and_does_not_reanchor():
    _, guard, _ = resources()
    target = FOLDED.copy(); target[2] += .0001
    assert not guard.check(FOLDED, target)[0]
    guard.observe_controller(100, 1, FOLDED[:6])
    assert guard.check(FOLDED, target)[0]
    outside = FOLDED.copy(); outside[3] += np.deg2rad(3.)
    assert not guard.check(outside, outside)[0]
    # DISARM, a new PREPARE and grip recapture cannot accumulate an exception.
    for state in (0, 1, 2, 3): guard.observe_controller(100, state, outside[:6])
    assert not guard.check(outside, outside)[0]


def test_leaving_zero_band_reinstates_pair_check_and_other_pairs_stay_checked():
    _, guard, _ = resources()
    guard.observe_controller(100, 1, FOLDED[:6])
    assert guard.check(FOLDED, FOLDED)[0]
    outside = FOLDED.copy(); outside[2] = np.deg2rad(2.)+.0001
    assert not guard.check(outside, outside)[0]  # link1-link5 still intersects here.
    assert guard.last_tolerated_pairs == []
    _, incomplete, _ = resources(pairs=[['link1', 'link4']])
    assert not incomplete.check(FOLDED, FOLDED)[0]  # Remaining intersecting pairs still block.
    guard.cfg['floor_z_m'] = 1.
    assert not guard.check(FOLDED, FOLDED)[0]


def test_invalid_or_overwide_tolerance_is_rejected():
    _, _, cfg = resources()
    urdf = ROOT/'src/A5/urdf/A5.urdf'
    for key, value in [('zero_band_rad', np.deg2rad(3.)), ('anchor_band_rad', float('nan')),
                       ('pairs', [['base_link', 'link6']]), ('enabled', 'false')]:
        bad = deepcopy(cfg); bad['zero_pose_collision_tolerance'][key] = value
        kin = A5Kinematics(urdf, ROOT/'src/a6_y_controllers/config/a5_motion_limits.yaml', bad)
        with pytest.raises(ValueError): GeometryGuard(kin, urdf, bad)


def test_folded_pose_bridge_takeover_preserves_anchor_then_allows_tiny_reference():
    kin, guard, cfg = resources()
    arm = RealBridge(kin, guard, cfg, 'execute', clock=22)
    now = 10**12
    s = SimpleNamespace(clock_id=22, controller_epoch=11, monotonic_ns=now, feedback_valid=True,
        actual_positions=FOLDED.copy(), actual_velocities=np.zeros(7), position_age=np.zeros(7),
        feedback_age=np.zeros(7), state=0, producer_id=arm.producer, session_id=identifier('folded-test'),
        takeover_id=1, anchor_positions=FOLDED[:6].copy(), reference_positions=FOLDED[:6].copy(),
        anchor_monotonic_ns=now, accepted_sequence=0, session_origin_positions=FOLDED[:6].copy())
    def update(i, held=False, pos=(0., 0., 0.)):
        s.monotonic_ns = now+i*8_000_000
        # Each ROS receive has a separate message object and monotonic stamp.
        arm.observe(SimpleNamespace(**vars(s)), s.monotonic_ns)
        arm.receive(make_packet('folded-test', i+1, s.monotonic_ns, pos, squeeze=held,
            clutch_epoch=int(held), schema_version=2, purpose='real', source_kind='live'), s.monotonic_ns)
        return arm.tick(s.monotonic_ns, .008, False)[0]
    assert update(0)['valid']
    s.state = 1
    assert update(1)['valid']
    s.state = 3; s.takeover_id = 9; s.anchor_monotonic_ns = now+16_000_000
    first = update(2, True)
    assert first['valid']
    np.testing.assert_array_equal(first['positions'], FOLDED[:6])
    s.accepted_sequence = first['sequence']
    for i in range(3, 15):
        ref = update(i, True, (0., .001, 0.))
        assert ref['valid'], arm.reason
        s.reference_positions = ref['positions']; s.accepted_sequence = ref['sequence']
    assert np.max(np.abs(np.array(ref['positions'])-FOLDED[:6])) > 1e-6
    np.testing.assert_array_equal(s.actual_positions, FOLDED)


def test_recorded_folded_path_reuses_certified_distances_between_samples(monkeypatch):
    import pinocchio as pin
    # Actual reference-to-candidate segment from the 2026-10-08 live trace.
    start = np.array([.0646601052872507, -.006294346532387252, .0036240177004653873,
                      -.3065156023498883, .0005722133211259336, -.02269779507133589,
                      .0003620334874494734])
    end = np.array([.06521610217436058, -.005485191332387252, .003559504687807745,
                    -.30596335475997494, .0008866182857001598, -.023130679012296422,
                    .0003620334874494734])
    _, guard, _ = resources()
    guard.observe_controller(100, 1, start[:6])
    assert guard.check(start, start)[0]
    original = pin.computeDistance
    expensive = []
    def distance(model, data, index):
        if model is guard.geom:
            pair = model.collisionPairs[index]
            names = {model.geometryObjects[j].name for j in (pair.first, pair.second)}
            if names == {'link4_0', 'link6_0'}: expensive.append(index)
        return original(model, data, index)
    monkeypatch.setattr(pin, 'computeDistance', distance)
    assert guard.check(start, end) == (True, 'clear')
    assert len(expensive) <= 3
    # Cached certificates must not certify a stricter clearance they don't meet.
    guard.cfg['mesh_clearance_m'] = .002
    valid, reason = guard.check(start, end)
    assert not valid and reason.startswith('mesh_clearance:')


def test_recorded_motor_lag_path_keeps_certificates_for_both_ends(monkeypatch):
    import pinocchio as pin
    # 18:22 live run: the real motor lagged its reference by several mrad.
    # Rechecking this corridor discarded all but the final sample's distance,
    # repeating expensive link4/link6 mesh queries every bridge update.
    start = np.array([.07267109178301645, -.006675822079804661, .0005722133211261138,
                      -.2988860914015401, -.029564354924849212, -.05321583886472858,
                      .0003620334874494734])
    end = np.array([.0693999544547649, -.00667582207980456, 0.,
                    -.3000202077302088, -.034218337815643325, -.05427682777559584,
                    .0003620334874494734])
    _, guard, _ = resources()
    guard.observe_controller(100, 1, start[:6])
    assert guard.check(start, end) == (True, 'clear')
    original = pin.computeDistance
    queries = []
    def distance(model, data, index):
        if model is guard.geom: queries.append(index)
        return original(model, data, index)
    monkeypatch.setattr(pin, 'computeDistance', distance)
    for _ in range(4):
        assert guard.check(end, end) == (True, 'clear')
        assert guard.check(start, end) == (True, 'clear')
    assert len(queries) <= 2, 'previously certified corridor was recomputed'
    guard.cfg['mesh_clearance_m'] = .002
    assert not guard.check(start, end)[0], 'cached distances bypassed stricter clearance'


def test_distance_history_bounds_whole_segments_and_evicts_without_weakening():
    cache = DistanceCertificates([1., 0., 0., 0., 0., 0., 2.], capacity=2)
    origin = np.zeros(7)
    other = origin.copy(); other[0] = 1.
    assert cache.lower_bound(origin) == -float('inf')
    cache.remember(origin, .1)
    cache.remember(other, .1)
    assert cache.lower_bound(origin) == pytest.approx(.1)
    assert cache.lower_bound(other) == pytest.approx(.1)
    # Safe endpoints from different certificates do not certify their interior.
    assert cache.lower_bound(origin, other) == pytest.approx(-.9)
    shifted = origin.copy(); shifted[1] = 100.; shifted[6] = .01
    assert cache.lower_bound(shifted) == pytest.approx(.08)
    third = other.copy(); third[0] = 2.
    cache.remember(third, .1)
    assert cache.count == 2
    assert cache.lower_bound(origin) == pytest.approx(-.9)
