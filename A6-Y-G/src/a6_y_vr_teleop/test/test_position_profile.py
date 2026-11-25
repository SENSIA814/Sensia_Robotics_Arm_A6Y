from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml
import xacro
from a6_y_vr_teleop.a5_kinematics import A5Kinematics
from a6_y_vr_teleop.position_profile import position_paths, position_profile, position_issues, CONTROLLER
from a6_y_vr_teleop.pose_mapping import limit_target

ROOT = Path(__file__).resolve().parents[3]

@pytest.fixture
def profile():
    ctrl=ROOT/'src/a6_y_controllers/config'
    paths=position_paths(dict(controller=ctrl/'a5_vr_joint_impedance_controllers.yaml', calibration=ctrl/'a5_calibration.yaml'))
    cfg=yaml.safe_load((ROOT/'src/a6_y_vr_teleop/config/a5_vr_real.yaml').read_text())
    kin=A5Kinematics(ROOT/'src/A5/urdf/A5.urdf',ctrl/'a5_motion_limits.yaml',cfg)
    params=yaml.safe_load(paths['controller'].read_text())['/arm6/'+CONTROLLER]['ros__parameters']
    manifest=yaml.safe_load((ROOT/'src/a6_y_vr_teleop/config/a5_vr_commissioning.yaml').read_text())
    return paths,kin,params,manifest,cfg


def test_position_profile_does_not_inherit_effort_hold_proof(profile):
    paths,kin,params,manifest,cfg=profile
    manifest['supported_enable']={'status':'pending'}
    before=deepcopy(manifest)
    assert position_issues(paths,kin,params,manifest)==[]
    assert manifest==before
    assert not any(k in params for k in ('torque_scales','joint_stiffness','vr_ramp_gravity_on_prepare'))
    hold,bridge=position_profile('hold',params,cfg)
    assert not hold['vr_allow_arm'] and not bridge['rotation_enabled']
    assert bridge['target_linear_speed']==.005
    assert bridge['geometry_checking'] and not bridge['hold_on_ik_failure']
    assert not bridge['limit_reference_lead']
    assert not bridge['bounded_ik']
    follow,_=position_profile('follow',params,cfg)
    assert follow['vr_allow_arm']


@pytest.mark.parametrize('change', ['zero_snap','gripper_enable','motor_mapping','driver_speed','controller_speed'])
def test_position_contract_rejects_unsafe_divergence(profile,tmp_path,change):
    paths,kin,params,manifest,_=profile
    motors=yaml.safe_load(paths['motors'].read_text())
    if change=='zero_snap':motors['joints']['joint2']['min_position']=0.
    if change=='gripper_enable':motors['joints']['gripper_finger_joint1']['read_only']=False
    if change=='motor_mapping':motors['joints']['joint4']['sign']=1.
    if change=='driver_speed':motors['joints']['joint4']['max_velocity']=.5
    if change=='controller_speed':params['position_velocity_limit'][0]=0.
    path=tmp_path/'motors.yaml';path.write_text(yaml.safe_dump(motors));paths['motors']=path
    assert position_issues(paths,kin,params,manifest)


def test_position_hardware_description_has_native_mode_readonly_gripper_and_zero_capture(profile):
    paths,kin,_,_,_=profile
    doc=xacro.process_file(str(ROOT/'src/a6_y_controllers/config/a5.urdf.xacro'),mappings=dict(
        model_file=str(ROOT/'src/A5/urdf/A5.urdf'),motor_config_file=str(paths['motors']),control_mode='pos_vel'))
    root=ET.fromstring(doc.toxml())
    assert root.find("ros2_control/hardware/param[@name='control_mode']").text=='pos_vel'
    assert root.find("ros2_control/joint[@name='gripper_finger_joint1']/param[@name='read_only']").text.lower()=='true'
    lo=float(root.find("ros2_control/joint[@name='joint2']/param[@name='min_position']").text)
    assert lo<-.0048<0 and lo==kin.lower[1]-kin.zero_tolerance[1]


def test_full_profile_opens_orientation_gripper_and_removes_only_trial_travel(profile):
    paths,kin,_,manifest,cfg=profile
    paths=position_paths(paths,'full')
    params=yaml.safe_load(paths['controller'].read_text())['/arm6/'+CONTROLLER]['ros__parameters']
    assert position_issues(paths,kin,params,manifest,'full')==[]
    p,b=position_profile('follow',params,cfg,'full')
    assert b['rotation_enabled'] and b['gripper_control'] and b['local_workspace_radius'] is None
    assert b['stop_on_compute_overrun'] is False and b['stop_on_ik_stall'] is False
    assert b['geometry_checking'] is False and b['hold_on_ik_failure'] is True
    assert b['limit_reference_lead'] is False
    assert b['joint_goal_horizon']==.25 and p['vr_trajectory_goals'] is True
    assert p['vr_tracking_governor'] is True
    assert b['bounded_ik'] is True
    assert not b['zero_pose_collision_tolerance']['enabled']
    assert p['vr_session_reference_travel']==p['vr_session_actual_travel']==0
    assert p['vr_actual_velocity']==[1.25]*6 and p['vr_gripper_actual_speed']==.01
    assert p['vr_auto_rearm_after_release'] is True
    assert p['vr_lower']==kin.lower[:6].tolist() and p['vr_upper']==kin.upper[:6].tolist()
    assert not cfg.get('gripper_control',False)
    params['vr_gripper_speed']=1.
    assert position_issues(paths,kin,params,manifest,'full')


def test_full_speed_reaches_ik_and_driver_without_changing_other_mode_bounds(profile):
    paths,_,_,manifest,cfg=profile
    paths=position_paths(paths,'full')
    params=yaml.safe_load(paths['controller'].read_text())['/arm6/'+CONTROLLER]['ros__parameters']
    params,cfg=position_profile('follow',params,cfg,'full')
    kin=A5Kinematics(ROOT/'src/A5/urdf/A5.urdf',paths['limits'],cfg)
    assert position_issues(paths,kin,params,manifest,'full')==[]
    # Exercise a saturated IK step: the shared 0.5 rad/s cap must not silently
    # halve the new full-mode request, and the gripper must stay fixed.
    q=np.array([0.,1.,1.,.1,.4,.1,-.015])
    goal=q.copy();goal[0]+=.2
    dt=1/cfg['update_rate']
    candidate,_=kin.step(q,kin.fk(goal),dt)
    peak=float(np.max(np.abs(candidate[:6]-q[:6]))/dt)
    assert .5 < peak <= 1.+1e-12
    assert candidate[6]==q[6]
    assert np.all(candidate>=kin.lower) and np.all(candidate<=kin.upper)
    doc=xacro.process_file(str(ROOT/'src/a6_y_controllers/config/a5.urdf.xacro'),mappings=dict(
        model_file=str(ROOT/'src/A5/urdf/A5.urdf'),motor_config_file=str(paths['motors']),control_mode='pos_vel'))
    root=ET.fromstring(doc.toxml())
    for name in ('joint1','joint2','joint3','joint4','joint5','joint6'):
        assert float(root.find(f"ros2_control/joint[@name='{name}']/param[@name='max_velocity']").text)==1.
    base=yaml.safe_load((paths['limits'].parent/'a5_motion_limits.yaml').read_text())['joint_limits']
    full=yaml.safe_load(paths['limits'].read_text())['joint_limits']
    for name,limits in base.items():
        expected=dict(limits)
        if name.startswith('joint'): expected['max_velocity']=1.
        assert full[name]==expected
        if name.startswith('joint'): assert limits['max_velocity']==.5


def test_full_tcp_target_slew_is_ten_times_previous_operator_settings(profile):
    _,_,params,_,cfg=profile
    _,cfg=position_profile('follow',params,cfg,'full')
    previous=np.eye(4);desired=np.eye(4)
    desired[:3,3]=[1.,0.,0.]
    desired[:3,:3]=Rotation.from_rotvec([0.,0.,1.]).as_matrix()
    dt=1/cfg['update_rate']
    old=limit_target(previous,desired,dt,.18,.75)
    new=limit_target(previous,desired,dt,cfg['target_linear_speed'],cfg['target_angular_speed'])
    assert np.linalg.norm(new[:3,3])==pytest.approx(10*np.linalg.norm(old[:3,3]))
    assert Rotation.from_matrix(new[:3,:3]).magnitude()==pytest.approx(
        10*Rotation.from_matrix(old[:3,:3]).magnitude())


@pytest.mark.parametrize('change', ['stale_ik_cap','stale_driver_cap','zero_monitor','nan_monitor','insufficient_monitor','goal_mode_disabled','governor_disabled'])
def test_full_speed_contract_rejects_mismatched_caps(profile,tmp_path,change):
    paths,kin,_,manifest,_=profile
    paths=position_paths(paths,'full')
    params=yaml.safe_load(paths['controller'].read_text())['/arm6/'+CONTROLLER]['ros__parameters']
    if change in ('stale_ik_cap','stale_driver_cap'):
        key='limits' if change=='stale_ik_cap' else 'motors'
        root='joint_limits' if key=='limits' else 'joints'
        data=yaml.safe_load(paths[key].read_text())
        data[root]['joint3']['max_velocity']=.5
        path=tmp_path/(key+'.yaml');path.write_text(yaml.safe_dump(data));paths[key]=path
    elif change=='goal_mode_disabled':
        params['vr_trajectory_goals']=False
    elif change=='governor_disabled':
        params['vr_tracking_governor']=False
    else:
        params['vr_actual_velocity'][0]={'zero_monitor':0.,'nan_monitor':float('nan'),'insufficient_monitor':.9}[change]
    assert position_issues(paths,kin,params,manifest,'full')
