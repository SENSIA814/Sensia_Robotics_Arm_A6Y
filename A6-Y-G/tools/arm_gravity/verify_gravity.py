#!/usr/bin/env python3
"""Actual C++ plugin + PTY motor emulator; refuses all physical-device paths."""
import argparse
import errno
import json
import os
from pathlib import Path
import pty
import select
import struct
import threading
import time
from copy import deepcopy

import numpy as np

from gravity_config import JOINTS, ROLE_ORDER, load_model, load_roles, load_profile, protocol_ranges, GravityModel
from gravity_session import GravitySession


def wait(session, predicate, timeout=25, allow_error=False):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        s=session.snapshot()
        if predicate(s): return s
        if s['error'] and not allow_error: raise AssertionError(s['error']+' '+str(s['directory']))
        time.sleep(.02)
    raise AssertionError('timeout '+json.dumps(session.snapshot(),ensure_ascii=False))


class MotorEmulator:
    def __init__(self,role,position_mode=False,native_port=False):
        master,slave=pty.openpty()
        self.master=master; self.port=os.ttyname(slave); os.close(slave)
        assert self.port.startswith('/dev/pts/')
        self.roles=load_roles(); self.roles[role]['port']=self.port
        self.model=load_model(role,self.roles); self.ranges=protocol_ranges(self.model)
        self.q=np.array([.2,1.2,1.2,.1,.1,.2,-.01])
        self.enabled=set(); self.frames=[]; self.commands=[]; self.last_torque={}
        self.position_mode=position_mode;self.last_position={};self.mode_writes=[]
        self.native_port=native_port;self.modes={};self.preloaded=set()
        self.freeze_motion=False;self.reject_mode=False;self.max_speed={}
        self.shutdown=threading.Event(); self.drop_feedback=False; self.failure=None
        self.worker=threading.Thread(target=self.run,daemon=True); self.worker.start()

    @staticmethod
    def decode(raw,limit,bits): return raw/(2**bits-1)*2*limit-limit

    def feedback(self,i):
        if self.drop_feedback:return
        e=self.model['joints'][JOINTS[i-1]]; pmax=self.ranges[i]['PMAX']
        motor=e['sign']*(self.q[i-1]-e['offset'])/e['position_scale']
        raw=round((motor+pmax)/(2*pmax)*65535)
        tau=self.last_torque.get(i,0.) if i in self.enabled else 0.
        tm=self.ranges[i]['TMAX']; tround=round((tau+tm)/(2*tm)*4095)
        data=bytes([(16 if i in self.enabled else 0)|i,raw>>8,raw&255,0x80,0x00|(tround>>8),tround&255,25,25])
        self.reply(i,data)

    def reply(self,i,data):
        try: os.write(self.master,f't{i+16:03X}8{data.hex()}\r'.encode())
        except OSError:pass

    def run(self):
        buffer=b''
        try:
            while not self.shutdown.is_set():
                if not select.select([self.master],[],[],.01)[0]:continue
                try: buffer+=os.read(self.master,16384)
                except OSError as exc:
                    if exc.errno == errno.EIO:self.shutdown.wait(.01);continue
                    raise
                while b'\r' in buffer:
                    line,buffer=buffer.split(b'\r',1)
                    if len(line)!=21 or not line.startswith(b't'):continue
                    cid,data=int(line[1:4],16),bytes.fromhex(line[5:].decode())
                    self.frames.append((cid,data))
                    if cid==0x7ff and data[2]==0xcc:self.feedback(data[0])
                    elif cid==0x7ff and data[2:4]==b'\x55\x0a':
                        mode=int.from_bytes(data[4:8],'little')
                        if self.native_port:
                            assert not self.enabled,"mode write while motors enabled"
                            assert data[0]!=7 and mode in (1,2)
                            if self.reject_mode:continue
                            self.modes[data[0]]=mode;self.preloaded.discard(data[0])
                        else:assert mode==(2 if self.position_mode else 1),(self.position_mode,mode)
                        self.mode_writes.append((data[0],mode));self.reply(data[0],data)
                    elif cid in range(1,8) and data==b'\xff'*7+b'\xfc':
                        if self.native_port:assert cid in self.preloaded,"enable without command preload"
                        self.enabled.add(cid); self.feedback(cid)
                    elif cid in range(1,8) and data==b'\xff'*7+b'\xfd':
                        self.enabled.discard(cid); self.feedback(cid)
                    elif cid in range(1,8):
                        assert self.modes.get(cid)==1 if self.native_port else not self.position_mode,'MIT command in wrong mode'
                        self.preloaded.add(cid)
                        assert cid!=7,'gripper must stay read-only'
                        kp=(data[3]&15)<<8|data[4]; kd=data[5]<<4|data[6]>>4
                        velocity=data[2]<<4|data[3]>>4
                        assert kp==kd==0,'MIT stiffness/damping unexpectedly nonzero'
                        assert velocity in (2047,2048),'MIT target velocity not zero'
                        traw=(data[6]&15)<<8|data[7]
                        tau=self.decode(traw,self.ranges[cid]['TMAX'],12)
                        self.last_torque[cid]=tau; self.commands.append((cid,tau))
                        self.feedback(cid)
                    elif 0x101<=cid<=0x107:
                        assert self.modes.get(cid-0x100)==2 if self.native_port else self.position_mode,'POS_VEL command in wrong mode'
                        i=cid-0x100;assert i!=7,'gripper must stay read-only'
                        position,speed=struct.unpack('<ff',data)
                        limit=([7.5]*3+[5.]*3)[i-1] if self.native_port else .1
                        assert np.isfinite([position,speed]).all() and 0<speed<=limit+.00001,(position,speed)
                        self.preloaded.add(i);self.max_speed[i]=max(self.max_speed.get(i,0),speed)
                        self.last_position[i]=position;self.commands.append((i,position))
                        if i in self.enabled and not self.freeze_motion:
                            entry=self.model['joints'][JOINTS[i-1]]
                            self.q[i-1]=entry['sign']*position*entry['position_scale']+entry['offset']
                        self.feedback(i)
                    else: raise AssertionError('unexpected frame '+line.decode())
        except Exception as exc: self.failure=exc

    def close(self):
        self.shutdown.set();self.worker.join(2);os.close(self.master)
        if self.failure:raise self.failure


def verify(role, fault=False, readonly_close=False, cancel_enable=False, live=False, cancel_tuning=False, preenabled=False):
    em=MotorEmulator(role); s=GravitySession(em.roles)
    if preenabled: em.enabled.update(range(1,7))
    # Verify the profile the GUI will actually load, including per-arm tuning.
    params=load_profile(role,em.roles)
    result={}
    try:
        s.start(role,mock=False,with_rviz=False)
        initial=wait(s,lambda x:x['ready'])
        assert em.frames and all(cid==0x7ff and d[2]==0xcc for cid,d in em.frames)
        assert not em.commands
        if preenabled:
            # Reproduce a rebooted PC with six motors still reporting state 1.
            generation=s.stack.control_generation
            assert s._fresh() and not s._fresh(disabled=True)
            assert '仍报告已使能' in s.enable_block_reason()
            s.enable()
            wait(s,lambda x:not x['busy'] and x['status']=='尚未开启补偿，反馈保持连接')
            assert s.snapshot()['ready'] and s.snapshot()['running'] and not s.snapshot()['error']
            assert s.stack.control_generation==generation and not s.output_started
            assert em.enabled==set(range(1,7)) and not em.commands
            assert all(cid==0x7ff and d[2]==0xcc for cid,d in em.frames)
            s.stop()
            wait(s,lambda x:x['close_failed'])
            assert s.snapshot()['running'] and em.enabled==set(range(1,7))
            s.release()
            wait(s,lambda x:x['release_confirmed'] and not x['busy'])
            assert not em.enabled and not em.commands
            assert not any(d==b'\xff'*7+b'\xfc' for _,d in em.frames)
            assert not any(cid==7 for cid,d in em.frames)
            wait(s,lambda x:not s.enable_block_reason())
        assert not em.enabled
        if live:
            params=deepcopy(params);params['damping_gains'][0]+=.005
            s.tune(params)
            wait(s,lambda x:not x['busy'] and x['tuning_revision']==1)
            assert not em.enabled and not em.commands
        if cancel_enable:
            s.enable()
            wait(s,lambda x:s.output_started)
            s.release()
            wait(s,lambda x:x['release_confirmed'] and not x['busy'])
            assert not em.enabled
        if not readonly_close:
            s.enable()
            active=wait(s,lambda x:x['active'] and x.get('sequence',0)>5 and x.get('commanded') is not None)
            wait(s,lambda x:len(em.commands)>60)
            state=s.snapshot(); q=np.array(state['hardware']['q']); dq=np.array(state['hardware']['velocity'])
            g=GravityModel(em.model).gravity(q)
            # The emulator's quantized zero velocity must remain in the friction
            # deadband, so this check isolates gravity scale/sign and damping.
            assert np.all(abs(dq) <= np.array(params['friction_velocity_deadbands']))
            expected=np.clip(np.array(params['effort_signs'])*params['torque_scales']*g-np.array(params['damping_gains'])*dq,
                             -np.array(params['torque_limits']),params['torque_limits'])
            np.testing.assert_allclose(state['commanded'],expected,atol=1e-8)
            for i in range(1,7):
                e=em.model['joints'][JOINTS[i-1]]
                np.testing.assert_allclose(em.last_torque[i],expected[i-1]*e['sign']*e['position_scale'],atol=2*em.ranges[i]['TMAX']/4095+.0001)
            assert em.enabled==set(range(1,7))
            # Force calculation and a complete MIT batch precede the first enable.
            first=next(k for k,(_,d) in enumerate(em.frames) if d==b'\xff'*7+b'\xfc')
            assert set(cid for cid,d in em.frames[:first] if cid in range(1,7) and d[:7]!=b'\xff'*7)==set(range(1,7))
            if cancel_tuning:
                changed=deepcopy(params);changed['torque_scales'][2]+=.05
                s.tune(changed)
                wait(s,lambda x:x.get('tuning_status')=='控制器已接受，正在平滑应用')
                s.release()
                wait(s,lambda x:x['release_confirmed'] and not x['busy'])
                assert not em.enabled and not s.snapshot()['error']
                assert s.snapshot()['applied_parameters']==changed
                s.enable();wait(s,lambda x:x['active'])
                expected[2]=np.clip(changed['effort_signs'][2]*changed['torque_scales'][2]*g[2]-changed['damping_gains'][2]*dq[2],
                                    -changed['torque_limits'][2],changed['torque_limits'][2])
                wait(s,lambda x:np.allclose(x['commanded'],expected,atol=1e-8))
            if live:
                generation=s.stack.control_generation
                enable_count=sum(d==b'\xff'*7+b'\xfc' for _,d in em.frames)
                changed=deepcopy(params)
                for index in (1,2,3): changed['torque_scales'][index]+=.1
                changed['damping_gains'][1]=.025
                changed['friction_compensation_enabled']=False
                s.tune(changed)
                wait(s,lambda x:not x['busy'] and x['tuning_revision']==1)
                assert not s.snapshot()['tuning_error']
                assert s.snapshot()['applied_parameters']==changed
                expected=np.clip(np.array(changed['effort_signs'])*changed['torque_scales']*g-np.array(changed['damping_gains'])*dq,
                                 -np.array(changed['torque_limits']),changed['torque_limits'])
                wait(s,lambda x:np.allclose(x['commanded'],expected,atol=1e-8))
                assert s.stack.control_generation==generation,'tuning must not reconnect'
                assert sum(d==b'\xff'*7+b'\xfc' for _,d in em.frames)==enable_count,'tuning must not reenable motors'
                for i in range(1,7):
                    e=em.model['joints'][JOINTS[i-1]]
                    np.testing.assert_allclose(em.last_torque[i],expected[i-1]*e['sign'],atol=2*em.ranges[i]['TMAX']/4095+.0001)
            if fault:
                em.drop_feedback=True
                wait(s,lambda x:bool(x['error']),timeout=8,allow_error=True)
                s.worker.join(20)
                assert not em.enabled,'driver must disable after feedback loss'
                assert s.snapshot()['release_confirmed'] is False,'missing feedback cannot count as confirmed release'
            else:
                s.release(); wait(s,lambda x:x['release_confirmed'])
                assert not em.enabled
                count=len(em.commands);time.sleep(.15)
                assert len(em.commands)==count,'MIT commands continued after release'
                if live:
                    changed['torque_scales'][2]+=.05
                    s.tune(changed);wait(s,lambda x:not x['busy'] and x['tuning_revision']==2)
                    expected[2]=np.clip(changed['effort_signs'][2]*changed['torque_scales'][2]*g[2]-changed['damping_gains'][2]*dq[2],
                                        -changed['torque_limits'][2],changed['torque_limits'][2])
                s.enable();wait(s,lambda x:x['active'])
                assert em.enabled==set(range(1,7))
                if live: wait(s,lambda x:np.allclose(x['commanded'],expected,atol=1e-8))
        if s.snapshot()['active']:
            s.release();wait(s,lambda x:x['release_confirmed'] and not x['busy'])
        s.stop();s.worker.join(20)
        assert not s.worker.is_alive() and not em.enabled
        assert all(p.poll() is not None for _,p in s.stack.children)
        if readonly_close: assert all(cid==0x7ff and d[2]==0xcc for cid,d in em.frames)
        if not fault:assert not s.snapshot()['error'],s.snapshot()['error']
        assert not any(d==b'\xff'*7+b'\xfe' for _,d in em.frames),'zero-setting frame forbidden'
        assert not any(cid==0x7ff and d[2] not in (0xcc,0x55) for cid,d in em.frames),'unexpected register/flash command'
        result=dict(result='PASS',role=role,physical_devices=0,transport=em.port,
                    torque_scales=params['torque_scales'],
                    friction_compensation_enabled=params['friction_compensation_enabled'],
                    live_tuning=live,
                    cancel_tuning=cancel_tuning,
                    preenabled_start_rejected_without_output=preenabled,
                    frames=len(em.frames),mit_commands=len(em.commands),readonly_close=readonly_close,
                    feedback_loss=fault,cancel_enable=cancel_enable,all_children_exited=True,release_confirmed=s.snapshot()['release_confirmed'],
                    report=s.snapshot()['directory'])
        Path(s.snapshot()['directory'],'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        return result
    finally:
        s.release()
        if s.worker and s.worker.is_alive():
            try:wait(s,lambda x:not x['running'] or (x['release_confirmed'] and not x['busy']),timeout=8,allow_error=True)
            except AssertionError:pass
        s.stop()
        if s.worker:s.worker.join(20)
        em.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role',choices=ROLE_ORDER)
    p.add_argument('--fault',action='store_true');p.add_argument('--readonly-close',action='store_true');p.add_argument('--cancel-enable',action='store_true')
    p.add_argument('--live',action='store_true',help='verify live torque changes and inactive edits without reconnecting')
    p.add_argument('--cancel-tuning',action='store_true')
    p.add_argument('--preenabled',action='store_true',help='simulate motors left enabled across a PC restart')
    args=p.parse_args()
    results=[]
    for role in ([args.role] if args.role else ROLE_ORDER):
        result=verify(role,args.fault,args.readonly_close,args.cancel_enable,args.live,args.cancel_tuning,args.preenabled);results.append(result)
        print(json.dumps(result,ensure_ascii=False),flush=True)
    return results

if __name__ == '__main__':main()
