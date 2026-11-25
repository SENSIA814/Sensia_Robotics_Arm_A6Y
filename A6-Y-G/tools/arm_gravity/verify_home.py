#!/usr/bin/env python3
"""Offline only: actual MIT driver plus an eight-joint rigid-body PTY plant."""
import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time
from unittest.mock import patch

import numpy as np
import pinocchio as pin

from gravity_config import JOINTS, ROLE_ORDER, base_parameters
from gravity_session import GravitySession
from verify_gravity import MotorEmulator, wait


POSES={
    'small':[.05,.15,.2,.04,.04,.08,-.01],
    'recorded':[-.0051,.9466,1.1404,.0643,.1741,.1661,0.],
    'wrist':[.10,0.,.005,.46,.008,-.025,0.],
}


def solve_coupled_friction(mass, drive, velocity, stiction, coulomb, velocity_threshold=.005):
    """Frozen-step rigid-body Coulomb friction with constrained sticking axes.

    Returns acceleration, projected velocity, friction torque, and sticking mask.
    A sticking joint needs a reaction torque, not zero net torque: its reaction
    must also cancel acceleration induced by the other joints through M. Low
    velocity capture is an explicit test-model approximation; its mass-metric
    projection removes energy and preserves free-axis generalized momentum.
    """
    mass=np.asarray(mass,dtype=float)
    drive=np.asarray(drive,dtype=float);velocity=np.asarray(velocity,dtype=float)
    stiction=np.asarray(stiction,dtype=float);coulomb=np.asarray(coulomb,dtype=float)
    n=velocity.size
    if (velocity.ndim!=1 or n==0 or mass.shape!=(n,n) or
        any(x.shape!=(n,) for x in (drive,stiction,coulomb)) or
        not all(np.isfinite(x).all() for x in (mass,drive,velocity,stiction,coulomb)) or
        not np.isfinite(velocity_threshold) or velocity_threshold<0 or
        (stiction<0).any() or (coulomb<0).any() or (coulomb>stiction).any() or
        not np.allclose(mass,mass.T,atol=1e-12,rtol=1e-10)):
        raise ValueError('invalid rigid-body friction inputs')
    np.linalg.cholesky(mass)  # Reject a non-positive-definite inertia matrix.
    sticking=np.abs(velocity)<velocity_threshold
    release_sign=np.zeros(n)
    for _ in range(n+1):  # A candidate is released at most once in this step.
        free=np.flatnonzero(~sticking);held=np.flatnonzero(sticking)
        projected=velocity.copy();projected[held]=0.
        acceleration=np.zeros(n);friction=np.zeros(n)
        if len(free):
            block=mass[np.ix_(free,free)]
            if len(held):
                projected[free]+=np.linalg.solve(block,mass[np.ix_(free,held)]@velocity[held])
            direction=np.sign(projected[free])
            # A just-released axis at rest takes the direction inferred from
            # the failed static reaction; otherwise friction opposes velocity.
            friction[free]=np.where(direction!=0.,-coulomb[free]*direction,
                                    coulomb[free]*release_sign[free])
            acceleration[free]=np.linalg.solve(block,drive[free]+friction[free])
        if len(held):
            friction[held]=mass[np.ix_(held,free)]@acceleration[free]-drive[held]
            violated=held[np.abs(friction[held])>stiction[held]+1e-12]
            if len(violated):
                release_sign[violated]=np.sign(friction[violated])
                sticking[violated]=False
                continue
        return acceleration,projected,friction,sticking
    raise AssertionError('friction active set did not terminate')


class Plant(MotorEmulator):
    def __init__(self,role,pose='small',gravity_scale=1.,friction_scale=1.,wrist_stiction=None,wrist_coulomb=None,command_delay=0.):
        self.velocity=np.zeros(7)
        self.supported=True
        self.disable_positions=[]
        super().__init__(role)
        self.q=np.array(POSES[pose]);self.initial_q=self.q.copy()
        self.friction_scale=friction_scale
        self.stiction=np.full(6,.025*friction_scale);self.coulomb=np.full(6,.02*friction_scale)
        if wrist_stiction is not None:self.stiction[3]=wrist_stiction
        if wrist_coulomb is not None:self.coulomb[3]=wrist_coulomb
        if (self.coulomb>self.stiction).any():
            super().close()
            raise ValueError('plant sliding friction must not exceed static friction')
        self.command_delay=command_delay;self.command_history=deque();self.peak_speed=np.zeros(6)
        # Independent full model (not the controller's reduced mimic model).
        self.full=pin.buildModelFromXML(self.model['description'],False)
        # Deliberate plant-only gravity mismatch; this does not model a payload
        # at a particular centre of mass or change the controller's parameters.
        self.full.gravity.linear=gravity_scale*self.full.gravity.linear
        self.data=self.full.createData()
        self.integrator=threading.Thread(target=self.integrate,daemon=True);self.integrator.start()

    def feedback(self,i):
        if self.drop_feedback:return
        if self.frames and self.frames[-1][1]==b'\xff'*7+b'\xfd':
            self.disable_positions.append((len(self.frames)-1,self.q.copy()))
        e=self.model['joints'][JOINTS[i-1]];limits=self.ranges[i]
        def raw(v,m,b):return int(round(np.clip((v+m)/(2*m),0,1)*(2**b-1)))
        p=raw(e['sign']*(self.q[i-1]-e['offset'])/e['position_scale'],limits['PMAX'],16)
        v=raw(e['sign']*self.velocity[i-1]/e['position_scale'],limits['VMAX'],12)
        t=raw(self.last_torque.get(i,0.) if i in self.enabled else 0.,limits['TMAX'],12)
        self.reply(i,bytes([(16 if i in self.enabled else 0)|i,p>>8,p&255,v>>4,((v&15)<<4)|(t>>8),t&255,25,25]))

    def integrate(self):
        last=time.monotonic()
        while not self.shutdown.wait(.004):
            now=time.monotonic();dt=min(.02,now-last);last=now
            if self.supported or self.enabled!=set(range(1,7)):
                self.velocity[:]=0
                self.command_history.clear()
                continue  # supported when disabled; no simulated free fall
            try:
                tau=np.array([self.last_torque.get(i+1,0.)/(self.model['joints'][n]['sign']*
                    self.model['joints'][n]['position_scale']) for i,n in enumerate(JOINTS[:6])])
                self.command_history.append((now,tau))
                while len(self.command_history)>1 and self.command_history[1][0]<=now-self.command_delay:
                    self.command_history.popleft()
                tau=self.command_history[0][1]
                steps=int(np.ceil(dt/.001));h=dt/steps
                for _ in range(steps):
                    q=np.r_[self.q,-self.q[-1]];v=np.r_[self.velocity,-self.velocity[-1]]
                    mass=pin.crba(self.full,self.data,q)[:6,:6].copy()
                    bias=pin.nonLinearEffects(self.full,self.data,q,v)[:6].copy()
                    net=tau-bias-.15*self.friction_scale*self.velocity[:6]
                    a,projected,_,_=solve_coupled_friction(
                        mass,net,self.velocity[:6],self.stiction,self.coulomb)
                    self.velocity[:6]=projected
                    self.velocity[:6]+=h*a;self.q[:6]+=h*self.velocity[:6]
                    self.peak_speed=np.maximum(self.peak_speed,abs(self.velocity[:6]))
                if not np.isfinite(self.q).all():raise AssertionError('plant nonfinite')
            except Exception as exc:
                self.failure=exc;return

    def close(self):
        self.shutdown.set();self.integrator.join(2);super().close()


def events(session):
    return [json.loads(l) for l in Path(session.snapshot()['directory'],'events.jsonl').read_text().splitlines()]


def verify_plan_event(session):
    executing=[e for e in events(session) if e['kind']=='home_executing']
    assert len(executing)==1,'repeated close unexpectedly restarted home'
    event=executing[0];name=event['plan_file']
    assert Path(name).name==name and name.startswith('home_plan_') and name.endswith('.json')
    directory=Path(session.snapshot()['directory'])
    saved=json.loads((directory/name).read_text())
    assert saved==json.loads((directory/'home_plan.json').read_text())
    assert np.max(abs(np.asarray(saved['positions'])[-1]))<=1e-4
    assert event['duration']>=saved['times'][-1]
    return event


def verify_aborted(session,em,first):
    state=session.snapshot()
    assert state['running'] and state['active'] and state['close_failed'],state
    assert em.enabled==set(range(1,7)),'home failure must leave gravity compensation enabled'
    frames=em.frames[first:]
    assert not any(d in (b'\xff'*7+b'\xfd',b'\xff'*7+b'\xfc') for _,d in frames)
    assert not any(cid==7 for cid,_ in frames),'gripper must stay read-only'
    executing=verify_plan_event(session)
    failures=[e for e in events(session) if e['kind']=='home_aborted']
    assert len(failures)==1,failures
    failure=failures[0]
    assert failure['revision']==executing['revision'] and failure['plan_file']==executing['plan_file']
    message=failure['message']
    assert 'Return aborted: ' in message and 'start mismatch, tracking, timing or heartbeat' not in message,message
    assert all(field in message for field in ('revision=','value=','limit=','elapsed=')),message
    assert len(failure['positions'])>=6 and len(failure['velocity'])>=6
    assert 'released' not in [e['kind'] for e in events(session)]
    return failure


def verify(role, scenario='normal', profile='unit', pose='small', gravity_scale=1.,friction_scale=1.,expect='success',wrist_stiction=None,wrist_coulomb=None,command_delay=0.):
    if not np.isfinite(gravity_scale) or gravity_scale<=0:raise ValueError('gravity_scale must be positive and finite')
    if not np.isfinite(friction_scale) or friction_scale<0:raise ValueError('friction_scale must be nonnegative and finite')
    if scenario!='model_mismatch' and expect!='success':raise ValueError('--expect applies only to model_mismatch')
    for value in (wrist_stiction,wrist_coulomb,command_delay):
        if value is not None and (not np.isfinite(value) or value<0):raise ValueError('friction/delay must be finite and nonnegative')
    em=Plant(role,pose,gravity_scale,friction_scale,wrist_stiction,wrist_coulomb,command_delay);s=GravitySession(em.roles)
    params=base_parameters()
    if profile=='unit':
        params['torque_scales']=[1.]*6+[0.];params['friction_compensation_enabled']=False
    outcome='pending';failure=None
    try:
        s.start(role,mock=False,with_rviz=False,parameters=params)
        wait(s,lambda x:x['ready'])
        assert not em.enabled and not em.commands
        s.enable();wait(s,lambda x:x['active']);time.sleep(.1)
        first=len(em.frames)
        if scenario=='plan_failure':
            with patch('gravity_home.GravityHome.plan',side_effect=ValueError('injected collision')):
                s.stop();wait(s,lambda x:x['close_failed'])
            assert s.snapshot()['running'] and s.snapshot()['active']
            assert em.enabled==set(range(1,7))
            assert not any(d==b'\xff'*7+b'\xfd' for _,d in em.frames[first:])
            outcome='planning_aborted'
        else:
            s.stop();s.stop()  # repeated close must not restart the sequence
            wait(s,lambda x:x['home_stage']=='executing' or x['close_failed'],timeout=40)
            assert not s.snapshot()['close_failed'],s.snapshot()['notice']
            em.supported=scenario=='tracking_failure'
            if scenario in ('cancel','feedback_loss'):
                wait(s,lambda x:max(abs(em.q[:6]-em.initial_q[:6]))>.02,timeout=15)
            if scenario=='cancel':
                wait(s,lambda x:x['home_stage']=='executing',timeout=30)
                s.release();wait(s,lambda x:x['release_confirmed'] and not x['busy'])
                assert s.snapshot()['running'] and s.snapshot()['close_failed']
                outcome='cancelled_and_disabled'
            elif scenario=='tracking_failure':
                wait(s,lambda x:x['close_failed'],timeout=25)
                failure=verify_aborted(s,em,first)
                assert max(abs(em.q[:6]))>.15
                outcome='controller_aborted_gravity_retained'
            elif scenario=='feedback_loss':
                wait(s,lambda x:x['home_stage']=='executing',timeout=30)
                em.drop_feedback=True
                wait(s,lambda x:not x['running'],timeout=20,allow_error=True)
                assert s.snapshot()['release_confirmed'] is False and not em.enabled
                outcome='feedback_lost_disable_unconfirmed'
            else:
                wait(s,lambda x:not x['running'] or x['close_failed'],timeout=100)
                if s.snapshot()['close_failed']:
                    assert scenario=='model_mismatch' and expect in ('abort','either'),s.snapshot()['notice']
                    failure=verify_aborted(s,em,first)
                    outcome='controller_aborted_gravity_retained'
                else:
                    assert expect in ('success','either'),'expected controller abort, but home completed'
                    assert not s.snapshot()['running'] and s.snapshot()['release_confirmed'] is True
                    assert max(abs(em.q[:6]))<.005
                    kinds=[e['kind'] for e in events(s)]
                    verify_plan_event(s)
                    assert kinds.index('home_reached')<kinds.index('released')<kinds.index('closed')
                    # No release/re-enable between acquiring output and reaching zero.
                    frames=em.frames[first:];first_fd=next(i for i,(_,d) in enumerate(frames) if d==b'\xff'*7+b'\xfd')
                    assert not any(d==b'\xff'*7+b'\xfc' for _,d in frames)
                    assert not any(cid==7 for cid,_ in frames)
                    positions=[q for index,q in em.disable_positions if index>=first]
                    assert positions and all(max(abs(q[:6]))<.005 for q in positions),'disabled before reaching zero'
                    assert first_fd>0
                    outcome='home_completed_and_disabled'
        result=dict(result='PASS',role=role,scenario=scenario,physical_devices=0,transport=em.port,
                    outcome=outcome,expected_outcome=expect if scenario=='model_mismatch' else scenario,
                    profile=profile,parameters=params,pose=pose,initial_position=em.initial_q.tolist(),
                    independent_rigid_body_plant=True,position=em.q.tolist(),report=s.snapshot()['directory'],
                    peak_speed=em.peak_speed.tolist(),
                    controller_failure=failure,
                    plant_assumptions=dict(gravity_scale=gravity_scale,gravity_mismatch='uniform plant gravity only; not a payload model',
                        viscous_friction=.15*friction_scale,coulomb_friction=.02*friction_scale,
                        stiction=.025*friction_scale,stiction_velocity_threshold=.005,
                        wrist_stiction=em.stiction[3],wrist_coulomb=em.coulomb[3],command_delay=command_delay,
                        static_friction_model='coupled acceleration constraints with bounded static reactions and kinetic release',
                        low_velocity_capture='mass-metric projection below 0.005 rad/s; not identified from hardware',
                        delay_model='latest command sampled by non-real-time plant thread; requested delay plus sampling/scheduling error',
                        supported_until_execution=True,supported_while_disabled=True,
                        held_during_execution=scenario=='tracking_failure'))
        Path(s.snapshot()['directory'],'home_verification.json').write_text(json.dumps(result,indent=2))
        return result
    finally:
        s.release()
        if s.worker and s.worker.is_alive():
            try:wait(s,lambda x:not x['running'] or (x['release_confirmed'] and not x['busy']),10,allow_error=True)
            except AssertionError:pass
        s.stop()
        if s.worker:s.worker.join(20)
        em.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--role',choices=ROLE_ORDER)
    p.add_argument('--scenario',choices=['normal','plan_failure','cancel','feedback_loss','tracking_failure','model_mismatch'],default='normal')
    p.add_argument('--profile',choices=['unit','reference'],default='unit',help='reference preserves base_parameters, including friction compensation')
    p.add_argument('--pose',choices=POSES,default='small',help='recorded reproduces the large real-log starting pose')
    p.add_argument('--gravity-scale',type=float,default=1.,help='plant-only gravity multiplier; controller model is unchanged')
    p.add_argument('--friction-scale',type=float,default=1.,help='multiplier of the documented simulated viscous/Coulomb/stiction terms')
    p.add_argument('--expect',choices=['success','abort','either'],default='success',help='model_mismatch only; either records actual outcome and checks safety, not successful homing')
    p.add_argument('--wrist-stiction',type=float,help='plant-only J4 static friction in Nm')
    p.add_argument('--wrist-coulomb',type=float,help='plant-only J4 sliding friction in Nm')
    p.add_argument('--command-delay',type=float,default=0.,help='plant-only command delay in seconds')
    args=p.parse_args()
    for role in ([args.role] if args.role else ROLE_ORDER):
        print(json.dumps(verify(role,args.scenario,args.profile,args.pose,args.gravity_scale,args.friction_scale,args.expect,args.wrist_stiction,args.wrist_coulomb,args.command_delay),ensure_ascii=False),flush=True)
