#!/usr/bin/env python3
"""Real ROS controllers and native driver on PTYs; no physical ports or dynamics."""
import argparse
import csv
import json
from pathlib import Path
import time
import numpy as np

from gravity_config import ROOT, CONTROLLER, JOINTS, load_roles
from teaching import DEFAULT_PAIRS
from teaching_port import TeachingCoordinator
from teaching_port_config import STARTUP_BLEND_SECONDS
from verify_multi import ProcessMotorEmulator, cleanup


def wait(coordinator, predicate, timeout=220., allow_error=False):
    deadline=time.monotonic()+timeout;last=None
    while time.monotonic()<deadline:
        state=coordinator.snapshot()
        phase=(state['phase'],state['status'])
        if phase!=last:print(phase,flush=True);last=phase
        if state['error'] and not allow_error:raise AssertionError(state['error'])
        if predicate(state):return state
        time.sleep(.03)
    raise AssertionError('Timeout: '+json.dumps(state,ensure_ascii=False))


# First feedback from teaching_fdcc2ed1ee; includes J2 just outside the URDF lower bound.
RECORDED_START = {
    'teaching_left': [-.0070573,.000572213,.002479591,-.100900282,.157740139,-.097467002,-.000756146],
    'motion_left': [.059700923,-.000190738,.002861067,-.253490501,-.006294347,-.155832761,-.003853358],
    'teaching_right': [-.077630274,-.002479591,.00171664,.001335164,.001335164,.060463874,-.000242883],
    'motion_right': [-.091363394,-.011635004,.00171664,-.353437095,.061226825,.031090257,-.000114568],
}


def verify(single=False, fault=False, exit_failure=False, mode_failure=False, cancel_home=False, step=False,
           recorded_start=False):
    pairs=DEFAULT_PAIRS[:1] if single else DEFAULT_PAIRS
    selected=[r for pair in pairs for r in pair]
    emulators={};c=None;initial={}
    try:
        roles=load_roles()
        for index,role in enumerate(selected):
            emulators[role]=ProcessMotorEmulator(role,native_port=True)
            q=[.26,.18,.17,.15,-.14,.16,-.01] if role.startswith('teaching_') else [-.12,.04,.05,-.05,.04,-.06,-.01]
            if recorded_start:q=RECORDED_START[role]
            initial[role]=list(q)
            emulators[role].snapshot(('set_q',q))
            roles[role]=dict(roles[role],port=emulators[role].port)
        assert all(roles[r]['port'].startswith('/dev/pts/') for r in selected)
        c=TeachingCoordinator(roles=roles,report_dir=ROOT.parent/'diagnostics/arm_teaching_test_reports')
        c.start(pairs,mock=False,with_rviz=False)
        state=wait(c,lambda s:s['phase']=='following',timeout=30)
        time.sleep(STARTUP_BLEND_SECONDS+.3)
        for role,em in emulators.items():
            data=em.snapshot();modes=[m for i,m in data['mode_writes'] if i==1]
            assert modes==([1] if role.startswith('teaching') else [2]),(role,modes)
            assert data['enabled']==list(range(1,7)),(role,data)
            arm=c.sessions[role]
            assert not arm.snapshot()['startup_homed']
            events=Path(arm.snapshot()['directory'],'events.jsonl').read_text()
            assert 'native_home_started' not in events
            if role.startswith('motion_'):
                assert arm.math is None and not arm.snapshot()['gravity_enabled']
                assert arm.snapshot()['gravity']==[None]*7
                assert not list(Path(arm.snapshot()['directory']).glob('spawner_'+CONTROLLER+'*'))
                assert not data['last_torque']
        startup_audit={r:em.snapshot() for r,em in emulators.items()}
        for leader,follower in pairs:
            np.testing.assert_allclose(emulators[leader].snapshot()['q'][:6],initial[leader][:6],atol=.001)
            np.testing.assert_allclose(emulators[follower].snapshot()['q'][:6],emulators[leader].snapshot()['q'][:6],atol=.003)
            # Includes the pre-enable preload and every native POS_VEL command:
            # first hold the measured follower pose, then move only toward the teacher.
            commands=startup_audit[follower]['position_commands']
            assert set(commands)==set(range(1,7)),commands
            for i,values in commands.items():
                begin,target=initial[follower][i-1],initial[leader][i-1]
                assert abs(values['first']-begin)<.001,(follower,i,values)
                assert values['min']>=min(begin,target)-.001,(follower,i,values)
                assert values['max']<=max(begin,target)+.001,(follower,i,values)
        print('PASS: no startup home, nonzero teacher pose preserved, follower converges directly; no follower gravity',flush=True)
        if cancel_home:
            c.stop()
            wait(c,lambda s:any(a.get('home_stage')=='executing' for a in s['arms'].values()),timeout=15)
            started=time.monotonic();c.release()
            state=wait(c,lambda s:s['phase']=='released',allow_error=True,timeout=20)
            release_time=time.monotonic()-started
            assert all(not em.snapshot()['enabled'] for em in emulators.values())
            c.stop()
            wait(c,lambda s:s['phase']=='stopped',allow_error=True,timeout=25)
            for arm in c.sessions.values():
                events=Path(arm.snapshot()['directory'],'events.jsonl').read_text()
                assert 'native_home_reached' not in events
            return dict(result='PASS',case='release_during_exit_home',release_time_s=release_time,physical_devices=0,report=state['directory'])
        if mode_failure:
            emulators[selected[0]].snapshot('reject_mode')
            c.stop()
            state=wait(c,lambda s:bool(s['close_failed']),allow_error=True,timeout=30)
            assert not emulators[selected[0]].snapshot()['enabled']
            assert state['phase']=='error'
            return dict(result='PASS',case='hot_mode_ack_failure_no_reenable',physical_devices=0,report=state['directory'])
        if fault:
            dropped=time.monotonic()
            emulators[selected[0]].snapshot('drop_feedback')
            deadline=dropped+3.
            while time.monotonic()<deadline:
                if all(c.sessions[f].snapshot()['follow_phase']==2 or not c.sessions[f].snapshot()['active'] for _,f in pairs):break
                time.sleep(.01)
            else:raise AssertionError('Peer follower did not hold within 3 seconds of feedback loss')
            hold_latency=time.monotonic()-dropped
            if len(pairs)>1:
                leader,follower=pairs[1]
                baseline=np.array(emulators[follower].snapshot()['q'])
                emulators[leader].snapshot(('set_q',[.2,.1,.1,0.,0.,0.,-.01]))
                time.sleep(.3)
                np.testing.assert_allclose(emulators[follower].snapshot()['q'],baseline,atol=.001)
            state=wait(c,lambda s:s['phase']=='error',allow_error=True,timeout=20)
            for _,follower in pairs:
                arm=c.sessions[follower].snapshot()
                assert arm['follow_phase']==2 or not arm['active'],arm
            assert state['error'] and state['close_failed']
            return dict(result='PASS',case='feedback_loss_group_hold',physical_devices=0,pairs=pairs,
                        follower_hold_latency_s=hold_latency,report=state['directory'])
        rate_start={r:s.feedback.snapshot()['rows']['joint1'] for r,s in c.sessions.items()}
        if step:time.sleep(2.)
        for k in ([50] if step else range(51)):
            for index,(leader,_) in enumerate(pairs):
                emulators[leader].snapshot(('set_q',[(.18 if index==0 else -.12)*k/50,.06,.08,0.,0.,0.,-.01]))
            time.sleep(.02)
        time.sleep(4. if step else 1.)
        state=wait(c,lambda s:s['packets']>60)
        moved={f:emulators[f].snapshot()['q'][0] for _,f in pairs}
        assert abs(moved[pairs[0][1]]-.18)<.01,moved
        if len(pairs)==2:assert abs(moved[pairs[1][1]]+.12)<.01,moved
        print('PASS: upstream mapper follows distinct targets beyond old 0.15-rad abort threshold',flush=True)
        rates={}
        for role,arm in c.sessions.items():
            rows=arm.feedback.snapshot()['rows'];before=rate_start[role];after=rows['joint1']
            dt=after['control_time']-before['control_time']
            rates[role]={k:(after[k]-before[k])/dt for k in ('control_cycle_count','command_count','feedback_count')}
            assert all(225 < hz < 275 for hz in rates[role].values()),(role,rates[role])
            assert rows[JOINTS[-1]]['command_count']==0
        with Path(state['directory'],'frequency.csv').open() as stream:
            timing=list(csv.DictReader(stream))
        assert set(row['role'] for row in timing)==set(selected)
        assert all(any(row['role']==r and row['command_hz_j1'] for row in timing) for r in selected)
        assert state['frequency']==100 and 85 < state['actual_hz'] < 115,state['actual_hz']
        print('PASS: native loop, command and feedback counters near 250 Hz; mapper near 100 Hz; frequency.csv present',flush=True)
        if exit_failure:
            emulators[selected[0]].snapshot('freeze_motion')
            c.stop()
            state=wait(c,lambda s:s['phase']=='error',allow_error=True,timeout=50)
            assert state['close_failed'] and c.alive() and '原生位置回零失败' in state['close_error'],state
            assert emulators[selected[0]].snapshot()['enabled']
            held=np.array(emulators[selected[0]].snapshot()['q'])
            assert not c.sessions[pairs[0][1]].snapshot()['running']
            emulators[selected[0]].snapshot('restore_motion')
            time.sleep(.3)
            np.testing.assert_allclose(emulators[selected[0]].snapshot()['q'],held,atol=.001)
            print('PASS: failed exit retains hold/session; retry uses native position home',flush=True)
        exit_started=time.monotonic();c.stop()
        state=wait(c,lambda s:s['phase']=='stopped',timeout=60.)
        exit_elapsed=time.monotonic()-exit_started
        assert exit_elapsed<40.,exit_elapsed
        c.worker.join(10)
        assert not c.worker.is_alive() and not c.alive()
        audits={r:em.snapshot() for r,em in emulators.items()}
        for role,audit in audits.items():
            assert not audit['enabled'] and not audit['failure']
            if role.startswith('motion_'):assert not audit['last_torque']
            assert audit['gripper_commands']==audit['zero_commands']==0
            modes=[m for i,m in audit['mode_writes'] if i==1]
            assert modes==([1,2] if role.startswith('teaching') else [2]),(role,modes)
            assert max(abs(q) for q in audit['q'][:6])<.03
            assert audit['max_speed']=={1:7.5,2:7.5,3:7.5,4:5.,5:5.,6:5.},audit
            events=[json.loads(line) for line in Path(c.sessions[role].snapshot()['directory'],'events.jsonl').read_text().splitlines()]
            assert sum(e['kind']=='native_home_reached' for e in events)==1
            assert c.sessions[role].snapshot()['release_confirmed'] is True
        result=dict(result='PASS',physical_devices=0,case='protected_exit_retry' if exit_failure else 'normal',
            pairs=pairs,report=state['directory'],exit_elapsed_s=exit_elapsed,movement=moved,packets=state['packets'],actual_hz=state['actual_hz'],
            audit=audits,plant='instantaneous encoder response; no physical dynamics verification')
        result['native_rates_hz']=rates
        result.update(recorded_start=recorded_start,initial_positions=initial,startup_audit=startup_audit)
        if step:
            folder=Path(state['directory'])
            quality=json.loads((folder/'step_quality.json').read_text())
            assert quality['complete'] and not any(quality['queue_dropped'].values()),quality
            coverage={r:quality['written']['arm:'+r]/(v['last']-v['first']+1)
                      for r,v in quality['cycles'].items()}
            assert all(.95 < value <= 1.01 for value in coverage.values()),coverage
            with (folder/'step_arms.csv').open() as stream:
                for row in csv.DictReader(stream):
                    for j in range(1,7):
                        if row['role'].startswith('motion_'):
                            assert row[f'sent_mode_j{j}']=='2.000000000',row
                            assert row[f'send_complete_time_j{j}'] and row[f'feedback_receive_time_j{j}']
                            assert abs(float(row[f'error_j{j}'])-(float(row[f'sent_position_j{j}'])-float(row[f'position_j{j}'])))<2e-8
                        else:
                            assert not row[f'sent_position_j{j}'] and not row[f'error_j{j}']
                    assert not row['sent_position_j7'] and not row['error_j7']
            summary=json.loads((folder/'step_summary.json').read_text())
            assert len(summary['results'])==7*len(pairs)
            for row in summary['results']:
                assert row['status']==('read_only_gripper' if row['joint']==7 else 'measured'),row
                if row['joint']<7:
                    assert 'virtual_motor_no_physical_dynamics' in row['flags']
                    assert row['teacher_to_actual_t10_s'] is not None
            result.update(step_logging=True,step_sample_coverage=coverage,step_quality=quality)
        Path(state['directory'],'verification_port.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        return result
    finally:
        if c:
            if c.alive():c.release()
            if c.worker:c.worker.join(15)
            cleanup(list(c.sessions.values()))
        for em in emulators.values():em.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--single',action='store_true')
    parser.add_argument('--fault',action='store_true')
    parser.add_argument('--exit-failure',action='store_true')
    parser.add_argument('--mode-failure',action='store_true')
    parser.add_argument('--cancel-home',action='store_true')
    parser.add_argument('--step',action='store_true')
    parser.add_argument('--recorded-start',action='store_true')
    args=parser.parse_args()
    print(json.dumps(verify(args.single,args.fault,args.exit_failure,args.mode_failure,args.cancel_home,args.step,args.recorded_start),ensure_ascii=False,indent=2))
