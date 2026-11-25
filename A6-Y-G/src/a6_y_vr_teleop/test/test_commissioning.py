from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
import yaml
from a6_y_vr_teleop.a5_kinematics import A5Kinematics
from a6_y_vr_teleop.commissioning import commissioning_profile, commissioning_stage_issues, commissioning_issues
from a6_y_vr_teleop.commissioning_metrics import hold_metrics, disarm_metrics

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def profile():
    ctrl = ROOT/'src/a6_y_controllers/config'
    paths = {key: ctrl/value for key, value in dict(motors='a5_motors.yaml', calibration='a5_calibration.yaml',
        limits='a5_motion_limits.yaml', gravity='a5_gravity_controllers.yaml', controller='a5_vr_joint_impedance_controllers.yaml').items()}
    cfg = yaml.safe_load((ROOT/'src/a6_y_vr_teleop/config/a5_vr_real.yaml').read_text())
    manifest = yaml.safe_load((ROOT/'src/a6_y_vr_teleop/config/a5_vr_commissioning.yaml').read_text())
    kin = A5Kinematics(ROOT/'src/A5/urdf/A5.urdf', paths['limits'], cfg)
    return paths, cfg, manifest, kin


def test_hold_does_not_require_its_own_future_evidence(profile):
    paths, cfg, manifest, kin = profile
    manifest['supported_enable'] = dict(status='pending')
    assert commissioning_stage_issues('hold', manifest, paths, kin, cfg) == []
    assert commissioning_issues(manifest, paths, kin, cfg)
    issues = commissioning_stage_issues('follow', manifest, paths, kin, cfg)
    assert any('supported_enable' in value for value in issues)
    assert any('axis_mapping' in value for value in issues)
    assert any('geometry' in value for value in issues)


def test_trial_profile_cannot_enable_test_input_or_rotation(profile):
    paths, cfg, _, _ = profile
    params = yaml.safe_load(paths['controller'].read_text())['/arm6/a6_y_joint_impedance_controller']['ros__parameters']
    for stage in ('hold', 'follow'):
        controller, bridge = commissioning_profile(stage, params, cfg)
        assert controller['vr_allow_arm'] == (stage == 'follow')
        assert not controller['vr_allow_test_input']
        assert controller['vr_ramp_gravity_on_prepare'] and controller['impedance_ramp_duration'] >= 2.
        assert controller['vr_session_reference_travel'] == np.deg2rad(1.)
        assert controller['vr_session_actual_travel'] == np.deg2rad(2.)
        assert not bridge['rotation_enabled'] and bridge['target_linear_speed'] == .005
        assert bridge['local_workspace_radius'] == .005
        assert controller['torque_limits'] == params['torque_limits']
    assert cfg['target_linear_speed'] == .02
    with pytest.raises(ValueError): commissioning_profile('invalid', params, cfg)


def test_hold_still_requires_confirmed_mapping_and_matching_torque_limits(profile, tmp_path):
    paths, cfg, manifest, kin = profile
    paths = dict(paths)
    calibration = yaml.safe_load(paths['calibration'].read_text())
    calibration['joint_mapping']['joint2']['direction_verified'] = False
    file = tmp_path/'calibration.yaml'; file.write_text(yaml.safe_dump(calibration)); paths['calibration'] = file
    assert any('joint2:' in issue for issue in commissioning_stage_issues('hold', manifest, paths, kin, cfg))
    controller = yaml.safe_load(paths['controller'].read_text())
    controller['/arm6/a6_y_joint_impedance_controller']['ros__parameters']['torque_limits'][2] = 999.
    file = tmp_path/'controller.yaml'; file.write_text(yaml.safe_dump(controller)); paths['controller'] = file
    assert any('torque_limits' in issue for issue in commissioning_stage_issues('hold', manifest, paths, kin, cfg))


def test_follow_can_collect_stopping_results_after_other_prerequisites(profile, monkeypatch):
    paths, cfg, manifest, kin = profile
    # Synthetic decisions test stage ordering only; never written to physical records.
    monkeypatch.setattr('a6_y_vr_teleop.commissioning.field_issues', lambda *a, **kw: [])
    for name in ('supported_enable', 'independent_stop', 'motor_command_loss', 'axis_mapping', 'actual_quest'):
        manifest[name] = dict(status='verified', date='2026-10-08', operator='test fixture', evidence='synthetic')
    manifest['stopping'].update(max_distance_m=.002, max_joint_travel_rad=.01, max_stop_time_s=.5)
    manifest['geometry'].update(status='verified', model_and_tracking_margin_m=.001, evidence='synthetic')
    cfg.update(mesh_clearance_m=.004, workspace_lower=[-1., -1., 0.], workspace_upper=[1., 1., 1.], floor_z_m=-.5)
    assert commissioning_stage_issues('follow', manifest, paths, kin, cfg) == []
    assert commissioning_issues(manifest, paths, kin, cfg)  # No stop trials or approved snapshot yet.


def test_explicit_operator_trial_defers_evidence_without_approving_it(profile):
    paths, cfg, manifest, kin = profile
    before = deepcopy(manifest)
    assert commissioning_stage_issues('follow', manifest, paths, kin, cfg, operator_trial=True) == []
    assert manifest == before
    assert commissioning_stage_issues('follow', manifest, paths, kin, cfg)
    assert commissioning_issues(manifest, paths, kin, cfg)
    assert commissioning_stage_issues('hold', manifest, paths, kin, cfg, operator_trial=True)
    manifest['supported_enable']['status'] = 'pending'
    assert any('supported_enable' in s for s in commissioning_stage_issues(
        'follow', manifest, paths, kin, cfg, operator_trial=True))
    manifest['independent_stop'].pop('device_operator_confirmed')
    assert any('power cutoff' in s for s in commissioning_stage_issues(
        'follow', manifest, paths, kin, cfg, operator_trial=True))


def test_operator_trial_cannot_waive_mapping_or_torque_cap_errors(profile, tmp_path):
    paths, cfg, manifest, kin = profile
    paths = dict(paths)
    calibration = yaml.safe_load(paths['calibration'].read_text())
    calibration['joint_mapping']['joint2']['direction_verified'] = False
    file = tmp_path/'calibration.yaml'; file.write_text(yaml.safe_dump(calibration)); paths['calibration'] = file
    controller = yaml.safe_load(paths['controller'].read_text())
    controller['/arm6/a6_y_joint_impedance_controller']['ros__parameters']['torque_limits'][2] = 999.
    file = tmp_path/'controller.yaml'; file.write_text(yaml.safe_dump(controller)); paths['controller'] = file
    issues = commissioning_stage_issues('follow', manifest, paths, kin, cfg, operator_trial=True)
    assert any('joint2:' in s for s in issues)
    assert any('torque_limits' in s for s in issues)


def hold_samples():
    return [dict(monotonic_ns=10**12+i*8_000_000, actual=[0.]*7, velocity=[0.]*7,
                 reference=[0.]*6, position_age=[.02]*7, feedback_age=[.02]*7,
                 state=1, feedback_valid=True, follow_permitted=False, saturated=[False]*6) for i in range(1300)]


def test_hold_metrics_preserve_failures_and_insufficient_duration():
    samples = hold_samples(); assert hold_metrics(samples)['passed']
    assert not hold_metrics(samples[:10])['passed']
    for key, value in [('state', 6), ('feedback_valid', False), ('follow_permitted', True),
                       ('actual', [.02]*7), ('velocity', [.04]*7), ('reference', [.001]*6),
                       ('position_age', [.051]*7), ('saturated', [True]*6), ('actual', [float('nan')]*7)]:
        bad = deepcopy(samples); bad[100][key] = value
        assert not hold_metrics(bad)['passed'], key


def test_disarm_requires_sustained_disabled_state_after_acknowledgement():
    samples = [dict(monotonic_ns=10**12+i*8_000_000, state=0, reason=17) for i in range(40)]
    assert disarm_metrics(samples)['passed']
    assert not disarm_metrics(samples[:2])['passed']
    samples[6].update(state=6, reason=23)
    assert not disarm_metrics(samples)['passed']
