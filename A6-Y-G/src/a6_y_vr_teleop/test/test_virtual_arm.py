import json
from pathlib import Path
import numpy as np
import pinocchio as pin
import pytest
import yaml
from a6_y_vr_teleop.a5_kinematics import A5Kinematics
from a6_y_vr_teleop.input_protocol import decode,make_packet
from a6_y_vr_teleop.quest_input import RightControllerDecoder
from a6_y_vr_teleop.virtual_arm import VirtualArm

ROOT=Path(__file__).resolve().parents[3]


@pytest.fixture
def arm():
    cfg=yaml.safe_load((ROOT/'src/a6_y_vr_teleop/config/a5_vr.yaml').read_text())
    return VirtualArm(A5Kinematics(ROOT/'src/A5/urdf/A5.urdf',
        ROOT/'src/a6_y_controllers/config/a5_motion_limits.yaml',cfg),cfg)


class Operator:
    def __init__(self,arm):
        self.arm=arm;self.n=0;self.now=10**12;self.epoch=0;self.held=False
    def send(self,p=(0,0,0),held=False,trigger=.5,quat=(0,0,0,1),session='test',**kw):
        self.now+=8_000_000;self.n+=1
        if held!=self.held: self.epoch+=1
        self.held=held
        packet=make_packet(session,self.n,self.now,p,quat,held,trigger,self.epoch,**kw)
        self.arm.receive(packet,self.now)
        return packet
    def tick(self): self.arm.tick(self.now,.008,enforce_budget=False)


def test_requires_release_on_initial_connection_and_timeout(arm):
    op=Operator(arm);op.send(held=True);op.tick();assert arm.state!='TELEOP'
    op.send();op.send(held=True);assert arm.state=='TELEOP'
    frozen=arm.q.copy()
    arm.tick(op.now+160_000_000,.008,False);assert arm.state=='PAUSED'
    op.now+=160_000_000;op.send(held=True);op.tick();assert arm.state=='PAUSED'
    assert np.array_equal(arm.q,frozen)
    op.send();op.send(held=True);assert arm.state=='TELEOP'


def test_release_freezes_same_tick_and_regrip_uses_actual_fk(arm):
    op=Operator(arm);op.send();op.send(held=True)
    for _ in range(20):
        op.send((0,0,-.08),True);op.tick()
        op.send((0,0,-.15),False)
        frozen=arm.q.copy();op.tick();assert np.array_equal(frozen,arm.q)
        op.send((.02,.03,-.15),True)
        assert np.allclose(arm.raw_target,arm.kin.fk(frozen),atol=1e-12)


@pytest.mark.parametrize('axis',range(3))
def test_translation_and_rotation_mapping(arm,axis):
    op=Operator(arm);op.send();op.send(held=True)
    anchor=arm.kin.fk(arm.q)
    p=np.zeros(3);p[axis]=.02
    w=np.zeros(3);w[axis]=.1
    quat=pin.Quaternion(pin.exp3(w)).coeffs()
    op.send(p,True,quat=quat)
    assert np.allclose(arm.raw_target[:3,3],anchor[:3,3]+.5*arm.B@p)
    assert np.allclose(arm.raw_target[:3,:3],arm.B@pin.exp3(w)@arm.B.T@anchor[:3,:3])


def test_lost_release_packet_detected_by_epoch(arm):
    op=Operator(arm);op.send();op.send(held=True)
    op.epoch+=2;op.send((.01,0,0),True)
    assert arm.state=='PAUSED' and arm.reason=='missed_clutch_edge'
    op.send(held=True);assert arm.state=='PAUSED'
    op.send();op.send(held=True);assert arm.state=='TELEOP'


def test_out_of_order_duplicate_and_competing_session(arm):
    op=Operator(arm);packet=op.send();op.send(held=True)
    n=arm.accepted
    arm.receive(packet,op.now);assert arm.accepted==n
    op.send(held=True,session='other');assert arm.last.session=='test'
    assert arm.rejected==2


def test_stale_pose_cannot_be_refreshed_by_buttons(arm):
    op=Operator(arm);op.send();op.send(held=True)
    last=arm.last
    op.now+=160_000_000
    op.send(held=True,pose_sequence=last.pose_sequence,pose_received_monotonic_ns=last.pose_ns)
    assert arm.state=='PAUSED'


def test_new_session_requires_release_and_rejects_retired_session(arm):
    op=Operator(arm);op.send();op.send(held=True)
    op.now+=200_000_000
    op.send(held=True,session='new');assert arm.state=='PAUSED'
    op.send(session='new');op.send(held=True,session='new');assert arm.state=='TELEOP'
    op.now+=200_000_000;op.send(session='test');assert arm.last.session=='new'


def test_nan_jump_and_long_period_hold(arm):
    op=Operator(arm);op.send();op.send(held=True)
    q=arm.q.copy();op.send((1,0,0),True);assert arm.state=='PAUSED'
    arm.receive(b'{"schema_version":1,"position_m":[NaN,0,0]}',op.now)
    assert arm.state=='PAUSED'
    op.send();op.send(held=True);arm.tick(op.now,.1,False)
    assert np.array_equal(arm.q,q) and arm.state=='PAUSED'


@pytest.mark.parametrize('trigger,expected',[(0,-.03),(.5,-.015),(1,0)])
def test_gripper_units_limits_and_mimic(arm,trigger,expected):
    op=Operator(arm);op.send();op.send(held=True)
    for _ in range(60): op.send(held=True,trigger=trigger);op.tick()
    assert arm.q[6]==pytest.approx(expected)
    m=arm.kin.model;d=m.createData()
    pin.framesForwardKinematics(m,d,arm.kin.configuration(arm.q))
    assert m.nq==7
    # Fixed TCP stays independent of hand opening.
    q=arm.q.copy();q[6]=-.03
    assert np.allclose(arm.kin.fk(q),arm.kin.fk(arm.q))


@pytest.mark.parametrize('axis',range(6))
def test_ik_reachable_targets_and_velocity_bounds(arm,axis):
    q_goal=arm.q.copy();q_goal[axis]+=.08
    target=arm.kin.fk(q_goal)
    q=arm.q.copy()
    for _ in range(300):
        before=q.copy();q,sigma=arm.kin.step(q,target,.008)
        assert np.all(np.abs(q-before)<=arm.kin.speed*.008+1e-12)
        assert np.all(q>=arm.kin.lower) and np.all(q<=arm.kin.upper)
    e=arm.kin.error(q,target)
    assert np.linalg.norm(e[:3])<.002 and np.linalg.norm(e[3:])<.035


def test_unreachable_target_stays_finite_and_bounded(arm):
    target=arm.target.copy();target[:3,3]=[10,10,10]
    q=arm.q.copy()
    for _ in range(300):
        previous=q.copy();q,_=arm.kin.step(q,target,.008)
        assert np.isfinite(q).all() and np.all(np.abs(q-previous)<=arm.kin.speed*.008+1e-12)
        assert np.all(q>=arm.kin.lower) and np.all(q<=arm.kin.upper)


def test_decoder_column_major_and_right_hand_freshness():
    decoder=RightControllerDecoder('quest')
    T=np.eye(4);T[:3,3]=[.1,.2,.3]
    v={'right':T.flatten(order='F').tolist(),'rightState':{'squeeze':False,'triggerValue':.3}}
    p=decoder.update(v,10**12);frame=decode(p,10**12)
    assert np.allclose(frame.position,[.1,.2,.3]) and frame.fresh(10**12,.15)
    assert decoder.update({'left':v['right']},10**12+1) is None
    p=decoder.update({'rightState':{'squeeze':True,'triggerValue':1}},10**12+200_000_000)
    frame=decode(p,10**12+200_000_000)
    assert frame.pose_ns==10**12 and not frame.fresh(10**12+200_000_000,.15)
    assert frame.clutch_epoch==1
    p=decoder.update({'right':[0]*16},10**12+210_000_000)
    assert not decode(p,10**12+210_000_000).valid


def test_rotation_wrap_and_sign_flip(arm):
    op=Operator(arm)
    qa=pin.Quaternion(pin.exp3(np.array([0.,0.,np.pi-.01]))).coeffs()
    qb=pin.Quaternion(pin.exp3(np.array([0.,0.,-np.pi+.01]))).coeffs()
    op.send(quat=qa);op.send(held=True,quat=qa)
    R0=arm.raw_target[:3,:3].copy()
    op.send(held=True,quat=-qb)
    assert arm.state=='TELEOP'
    assert np.linalg.norm(pin.log3(arm.raw_target[:3,:3]@R0.T))==pytest.approx(.02)
