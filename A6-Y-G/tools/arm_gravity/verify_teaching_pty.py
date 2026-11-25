#!/usr/bin/env python3
"""Four-arm native transport: MIT teachers, POS_VEL followers, no physical ports.

Follower encoders accept targets instantaneously. This verifies protocol,
routing, watchdogs and teardown, not real motor dynamics or tracking accuracy.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from teaching import TeachingCoordinator, DEFAULT_PAIRS
from gravity_config import load_roles
from verify_multi import ProcessMotorEmulator
from verify_teaching import wait


def verify(fault=False):
    emulators={};c=None
    try:
        for pair in DEFAULT_PAIRS:
            for role in pair:
                emulators[role]=ProcessMotorEmulator(role,position_mode=role.startswith('motion_'))
                emulators[role].snapshot(('set_q',[0.]*7))
        roles=load_roles()
        for role,em in emulators.items():roles[role]=dict(roles[role],port=em.port)
        assert all(a['port'].startswith('/dev/pts/') for a in roles.values())
        c=TeachingCoordinator(roles=roles);c.start(mock=False,with_rviz=False)
        wait(c,lambda s:s['phase']=='following')
        if fault:
            emulators['teaching_left'].snapshot('drop_feedback')
            deadline=time.monotonic()+15
            while time.monotonic()<deadline:
                state=c.snapshot()
                if state['close_failed']:break
                time.sleep(.02)
            assert state['close_failed'] and state['error'],state
            time.sleep(.4)
            for follower in ('motion_left','motion_right'):
                arm=c.sessions[follower].snapshot()
                assert arm['follow_phase'] in (2,3) or not arm['active'],arm
            result=dict(result='PASS',physical_devices=0,feedback_loss=True,whole_batch_stopped=True,
                        failed_exit_retains_sessions=c.alive(),report=state['directory'])
            Path(state['directory'],'verification_fault.json').write_text(json.dumps(result,indent=2))
            return result
        before={r:em.snapshot()['last_position'] for r,em in emulators.items() if r.startswith('motion_')}
        for k in range(60):
            emulators['teaching_left'].snapshot(('set_q',[.025*k/60]+[0.]*6))
            emulators['teaching_right'].snapshot(('set_q',[-.015*k/60]+[0.]*6))
            wait(c,lambda s:s['phase']=='following',timeout=1);time.sleep(.02)
        time.sleep(.5);state=wait(c,lambda s:s['packets']>=60)
        # Opposite signed model targets cause opposite J1 motor positions;
        # J1's model-to-motor sign is -1 on both motion arms.
        changes={r:emulators[r].snapshot()['last_position'][1]-before[r][1] for r in ('motion_left','motion_right')}
        assert changes['motion_left']<-.01 and changes['motion_right']>.005,changes
        # Return the manually held teaching encoders to zero before normal exit.
        for k in range(60):
            emulators['teaching_left'].snapshot(('set_q',[.025*(59-k)/60]+[0.]*6))
            emulators['teaching_right'].snapshot(('set_q',[-.015*(59-k)/60]+[0.]*6))
            time.sleep(.02)
        time.sleep(.4);c.stop();c.worker.join(150)
        final=c.snapshot();assert not c.worker.is_alive() and final['phase']=='stopped' and not final['error'],final
        for role,em in emulators.items():
            audit=em.snapshot()
            assert not audit['enabled'] and not audit['failure'],(role,audit)
            assert audit['gripper_commands']==0 and audit['zero_commands']==0
            assert c.sessions[role].snapshot()['release_confirmed'] is True
        result=dict(result='PASS',physical_devices=0,ports={r:e.port for r,e in emulators.items()},
            instantaneous_follower_plant=True,j1_motor_position_changes=changes,packets=state['packets'],
            actual_hz=state['actual_hz'],gripper_commands=0,zero_or_flash_writes=0,all_released=True,
            report=final['directory'])
        Path(final['directory'],'verification_pty.json').write_text(json.dumps(result,indent=2))
        return result
    finally:
        if c and c.alive():
            c.release()
            if c.worker:c.worker.join(10)
            for s in c.sessions.values():
                deadline=time.monotonic()+12
                while s.snapshot()['running'] and (s.snapshot()['active'] or s.snapshot()['busy']) and time.monotonic()<deadline:time.sleep(.02)
                for _ in range(3):
                    s.stop()
                    if s.worker:s.worker.join(5)
                    if not s.snapshot()['running']:break
                if s.snapshot()['running']:
                    # Test-only forced teardown after release; never used by UI.
                    s.stop_event.set()
                    if s.worker:s.worker.join(10)
        for em in emulators.values():em.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--fault',action='store_true');args=parser.parse_args()
    print(json.dumps(verify(args.fault),ensure_ascii=False,indent=2))
