#!/usr/bin/env python3
"""Real ROS controllers and native driver on PTYs; no physical ports or dynamics."""
import argparse
import csv
import json
from pathlib import Path
import time
import numpy as np

from gravity_config import ROOT, CONTROLLER, JOINTS, load_roles, load_model, protocol_ranges
from teaching import DEFAULT_PAIRS
from teaching_port import TeachingCoordinator
from teaching_port_config import STARTUP_BLEND_SECONDS
from teaching_gripper import gripper_mapping, gripper_profile
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


def assert_gripper_slew(trace, speed=.100):
    """Check the complete receipt-time envelope, with bounded PTY scheduling jitter.

    The emulator may be descheduled and then parse several already-sent frames
    in one batch. Its receipt times are not the mapper's command times. Allow
    30 ms (three 100 Hz target periods) in this receipt-time-only audit.
    The actual mapper's per-tick 100 mm/s bound (1.0 mm per tick) is
    checked independently in test_teaching_gripper, and the wire speed field
    must remain exactly 100 mm/s in motor units. Startup first-hold is audited separately.
    """
    assert trace, 'No motion gripper POS_VEL commands received'
    origin=trace[0][0];lower=float('-inf');upper=float('inf');tolerance=speed*.03
    for stamp,position,_,_ in trace:
        t=stamp-origin
        if not lower-speed*t-tolerance <= position <= upper+speed*t+tolerance:
            evidence=ROOT.parent/'diagnostics/arm_teaching_test_reports'/f'gripper_slew_failure_{time.time_ns()}.json'
            evidence.parent.mkdir(parents=True,exist_ok=True)
            evidence.write_text(json.dumps(dict(speed=speed,tolerance=tolerance,trace=trace,failed_at=t,
                position=position,lower=lower,upper=upper),indent=2))
            raise AssertionError((t,position,lower,upper,str(evidence)))
        lower=max(lower,position+speed*t)
        upper=min(upper,position-speed*t)


def wait_grippers(coordinator, emulators, pairs, targets, timeout=12.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        state=coordinator.snapshot()
        assert not state['error'],state['error']
        actual={f:emulators[f].snapshot()['q'][6] for _,f in pairs}
        if all(abs(actual[f]-targets[f])<.00005 for _,f in pairs):return actual
        time.sleep(.05)
    raise AssertionError(('Gripper target not reached',actual,targets))


def mapped_gripper(coordinator, leader, follower, q):
    mapping=gripper_mapping(coordinator.sessions[leader].model,coordinator.sessions[follower].model)
    return float(np.clip(q*mapping['scale']+mapping['offset'],
                         mapping['follower']['open_position_m'],mapping['follower']['closed_position_m']))


def gripper_home_quantization_budget(model, hold_duration):
    """Bound stock JTC's initial cubic bump caused by encoded zero velocity.

    The 12-bit symmetric motor codec cannot represent zero: codes 2047/2048
    decode to +/-VMAX/4095. The driver scales that measured velocity into m/s.
    move_to_group_state.py supplies the same held position at T=2 s with zero
    final velocity. JTC's cubic Hermite interpolation is then
    q(t)-q(0)=v0*T*u*(1-u)^2, whose maximum is abs(v0)*T*4/27.
    Two position LSBs cover the independently sampled start/target positions.
    This is a PTY quantization budget, not a relaxed real-hardware hold limit.
    """
    entry=model['joints'][JOINTS[-1]]
    limits=protocol_ranges(model)[entry['motor_id']]
    scale=abs(float(entry['position_scale']))
    position_lsb=2.*limits['PMAX']/65535.*scale
    zero_velocity=limits['VMAX']/4095.*scale
    cubic_excursion=zero_velocity*float(hold_duration)*4./27.
    return {'position_lsb_m':position_lsb,'encoded_zero_velocity_m_s':zero_velocity,
            'hold_duration_s':float(hold_duration),'cubic_excursion_m':cubic_excursion,
            'max_hold_deviation_m':cubic_excursion+2.*position_lsb}


def verify(single=False, fault=False, exit_failure=False, mode_failure=False, cancel_home=False, step=False,
           recorded_start=False, gripper_outside_calibration=False):
    pairs=DEFAULT_PAIRS[:1] if single else DEFAULT_PAIRS
    selected=[r for pair in pairs for r in pair]
    emulators={};c=None;initial={}
    try:
        roles=load_roles()
        for index,role in enumerate(selected):
            emulators[role]=ProcessMotorEmulator(role,native_port=True)
            # Start the left follower at its measured open end (> old 35 mm),
            # and the right at its measured closed end, outside inset goals.
            profile=gripper_profile(load_model(role))
            edge=profile['measured_open_position_m'] if role=='motion_left' else profile['measured_closed_position_m']
            q=[.26,.18,.17,.15,-.14,.16,-.01] if role.startswith('teaching_') else [-.12,.04,.05,-.05,.04,-.06,edge]
            if recorded_start:q=RECORDED_START[role]
            if gripper_outside_calibration:
                # Recorded leader failure and both edges on the other roles.
                q=list(q)
                q[6]=(.00010540215444589034 if role=='teaching_left' else
                      profile['measured_closed_position_m']+.001 if role=='motion_left' else
                      profile['measured_open_position_m']-.001)
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
            assert data['enabled']==list(range(1,8)),(role,data['enabled'])
            gripper_modes=[m for i,m in data['mode_writes'] if i==7]
            assert gripper_modes==([1] if role.startswith('teaching_') else [2]),(role,gripper_modes)
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
            assert set(commands)==set(range(1,8)),commands
            for i,values in commands.items():
                begin,target=initial[follower][i-1],initial[leader][i-1]
                if i==7:target=mapped_gripper(c,leader,follower,target)
                assert abs(values['first']-begin)<(.00005 if i==7 else .001),(follower,i,values)
                assert values['min']>=min(begin,target)-.001,(follower,i,values)
                assert values['max']<=max(begin,target)+.001,(follower,i,values)
            assert_gripper_slew(startup_audit[follower]['gripper_position_history'])
            assert startup_audit[leader]['gripper_torque_history']
            assert max(abs(v) for _,v in startup_audit[leader]['gripper_torque_history'])<.0025
        initial_gripper_targets={f:mapped_gripper(c,l,f,initial[l][6]) for l,f in pairs}
        wait_grippers(c,emulators,pairs,initial_gripper_targets)
        print('PASS: no startup home, nonzero teacher pose preserved, follower converges directly; no follower gravity',flush=True)
        print('PASS: motion J7 holds then slews; teacher J7 MIT has zero stiffness and zero free-space torque',flush=True)
        if gripper_outside_calibration:
            print('PASS: beyond-calibration feedback accepted; startup holds actual pose, then slews to inset 0/100% goals',flush=True)
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
                emulators[leader].snapshot(('set_q',[(.18 if index==0 else -.12)*k/50,.06,.08,0.,0.,0.,-.014-.002*index]))
            time.sleep(.02)
        time.sleep(4. if step else 1.)
        state=wait(c,lambda s:s['packets']>60)
        moved={f:emulators[f].snapshot()['q'][0] for _,f in pairs}
        assert abs(moved[pairs[0][1]]-.18)<.01,moved
        if len(pairs)==2:assert abs(moved[pairs[1][1]]+.12)<.01,moved
        opened=wait_grippers(c,emulators,pairs,{f:mapped_gripper(c,l,f,-.014-.002*i) for i,(l,f) in enumerate(pairs)})
        for index,(leader,_) in enumerate(pairs):
            q=emulators[leader].snapshot()['q'];q[6]=-.010-.002*index
            emulators[leader].snapshot(('set_q',q))
        closed=wait_grippers(c,emulators,pairs,{f:mapped_gripper(c,l,f,-.010-.002*i) for i,(l,f) in enumerate(pairs)})
        assert all(closed[f]-opened[f]>.0035 for _,f in pairs),(opened,closed)
        percentage_checks=[]
        for fraction in (0., .25, .5, .75, 1., .5):
            targets={}
            for leader,follower in pairs:
                mapping=gripper_mapping(c.sessions[leader].model,c.sessions[follower].model)
                lp,fp=mapping['leader'],mapping['follower']
                q=emulators[leader].snapshot()['q']
                q[6]=(lp['measured_closed_position_m'] if fraction==0. else
                      lp['measured_open_position_m'] if fraction==1. else
                      lp['closed_position_m']+fraction*(lp['open_position_m']-lp['closed_position_m']))
                emulators[leader].snapshot(('set_q',q))
                targets[follower]=fp['closed_position_m']+fraction*(fp['open_position_m']-fp['closed_position_m'])
            actual=wait_grippers(c,emulators,pairs,targets)
            percentage_checks.append(dict(percent=100*fraction,targets=targets,actual=actual))
        print('PASS: measured manual endpoints saturate to inset 0/100%; 25/50/75% map to each follower travel',flush=True)
        for leader,follower in pairs:
            assert_gripper_slew(emulators[follower].snapshot()['gripper_position_history'])
            assert max(abs(v) for _,v in emulators[leader].snapshot()['gripper_torque_history'])<.0025
        print('PASS: both motion grippers open and close with distinct mapped targets; complete POS_VEL history obeys gripper speed',flush=True)
        print('PASS: upstream mapper follows distinct targets beyond old 0.15-rad abort threshold',flush=True)
        rates={}
        for role,arm in c.sessions.items():
            rows=arm.feedback.snapshot()['rows'];before=rate_start[role];after=rows['joint1']
            dt=after['control_time']-before['control_time']
            rates[role]={k:(after[k]-before[k])/dt for k in ('control_cycle_count','command_count','feedback_count')}
            assert all(225 < hz < 275 for hz in rates[role].values()),(role,rates[role])
            if role.startswith('teaching_'):assert rows[JOINTS[-1]]['command_count']>0
            else:
                assert rows[JOINTS[-1]]['command_count']>0
                expected=.100/arm.model['joints'][JOINTS[-1]]['position_scale']
                assert abs(rows[JOINTS[-1]]['sent_velocity_limit']-expected)<1e-6
        with Path(state['directory'],'frequency.csv').open() as stream:
            timing=list(csv.DictReader(stream))
        assert set(row['role'] for row in timing)==set(selected)
        assert all(any(row['role']==r and row['command_hz_j1'] for row in timing) for r in selected)
        for leader,_ in pairs:
            # Check real native status query cadence, not repeated joint_states.
            hz=[float(row['feedback_hz_j7']) for row in timing
                if row['role']==leader and row['phase']=='following' and row['feedback_hz_j7']]
            measured=float(np.median(hz))
            assert 225 < measured < 275,(leader,'MIT J7 feedback Hz',measured)
            rates[leader]['gripper_feedback_hz']=measured
        print('PASS: teacher J7 MIT feedback near 250 Hz with zero free-space torque',flush=True)
        assert state['frequency']==100 and 85 < state['actual_hz'] < 115,state['actual_hz']
        print('PASS: native loop, command and feedback counters near 250 Hz; mapper near 100 Hz; frequency.csv present',flush=True)
        # Capture before the first stop, including a failed-exit retry where
        # unaffected motion arms may have already finished their HOME stage.
        home_baselines={r:emulators[r].snapshot() for r in selected}
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
        home_hold_audits={}
        for role,audit in audits.items():
            events=[json.loads(line) for line in Path(c.sessions[role].snapshot()['directory'],'events.jsonl').read_text().splitlines()]
            assert not audit['enabled'] and not audit['failure']
            if role.startswith('motion_'):assert not audit['last_torque']
            assert audit['zero_commands']==0
            if role.startswith('teaching_'):
                assert max(abs(v) for _,v in audit['gripper_torque_history'])<.0025
            # All four grippers hold their measured opening during HOME.
            assert audit['gripper_commands']>0
            before=home_baselines[role]
            home_trace=audit['gripper_position_history'][len(before['gripper_position_history']):]
            assert home_trace,'No J7 hold commands during HOME'
            home_event=next(e for e in events if e['kind']=='native_home_started')
            budget=gripper_home_quantization_budget(c.sessions[role].model,home_event['hold_duration'])
            deviation=max(abs(q-before['q'][6]) for _,q,_,_ in home_trace)
            details=dict(budget,observed_max_deviation_m=deviation,held_position_m=before['q'][6],
                         samples=len(home_trace),first=home_trace[0],last=home_trace[-1])
            home_hold_audits[role]=details
            assert deviation<=budget['max_hold_deviation_m'],(role,details)
            assert abs(audit['q'][6]-before['q'][6])<.00005,(role,audit['q'][6],before['q'][6])
            assert abs(audit['q'][6])>.003,'HOME must preserve an open gripper, never zero it'
            assert_gripper_slew(audit['gripper_position_history'])
            modes=[m for i,m in audit['mode_writes'] if i==1]
            assert modes==([1,2] if role.startswith('teaching') else [2]),(role,modes)
            assert max(abs(q) for q in audit['q'][:6])<.03
            expected_speeds={1:7.5,2:7.5,3:7.5,4:5.,5:5.,6:5.}
            expected_speeds[7]=gripper_profile(c.sessions[role].model)['max_velocity_m_s']/c.sessions[role].model['joints'][JOINTS[-1]]['position_scale']
            assert set(audit['max_speed'])==set(expected_speeds),(role,audit['max_speed'])
            for i,speed in expected_speeds.items():assert abs(audit['max_speed'][i]-speed)<1e-6,(role,i,audit['max_speed'])
            assert sum(e['kind']=='native_home_reached' for e in events)==1
            assert c.sessions[role].snapshot()['release_confirmed'] is True
        result=dict(result='PASS',physical_devices=0,case='protected_exit_retry' if exit_failure else 'normal',
            pairs=pairs,report=state['directory'],exit_elapsed_s=exit_elapsed,movement=moved,packets=state['packets'],actual_hz=state['actual_hz'],
            audit=audits,plant='instantaneous encoder response; no physical dynamics verification')
        result['native_rates_hz']=rates
        result['gripper_follow']={'opened_m':opened,'closed_m':closed,'home_held_m':{f:b['q'][6] for f,b in home_baselines.items()},
                                 'home_hold_audit':home_hold_audits,'max_speed_m_s':.100}
        result.update(recorded_start=recorded_start,gripper_outside_calibration=gripper_outside_calibration,
                      initial_positions=initial,startup_audit=startup_audit,
                      gripper_percentage_checks=percentage_checks)
        if step:
            folder=Path(state['directory'])
            quality=json.loads((folder/'step_quality.json').read_text())
            assert quality['complete'] and not any(quality['queue_dropped'].values()),quality
            coverage={r:quality['written']['arm:'+r]/(v['last']-v['first']+1)
                      for r,v in quality['cycles'].items()}
            assert all(.95 < value <= 1.01 for value in coverage.values()),coverage
            with (folder/'step_arms.csv').open() as stream:
                for row in csv.DictReader(stream):
                    for j in range(1,8):
                        if row['role'].startswith('motion_'):
                            assert row[f'sent_mode_j{j}']=='2.000000000',row
                            assert row[f'send_complete_time_j{j}'] and row[f'feedback_receive_time_j{j}']
                            assert abs(float(row[f'error_j{j}'])-(float(row[f'sent_position_j{j}'])-float(row[f'position_j{j}'])))<2e-8
                        else:
                            assert not row[f'sent_position_j{j}'] and not row[f'error_j{j}']
            summary=json.loads((folder/'step_summary.json').read_text())
            assert len(summary['results'])==7*len(pairs)
            for row in summary['results']:
                assert row['status']==('gripper_recorded_not_analyzed' if row['joint']==7 else 'measured'),row
                if row['joint']<7:
                    assert 'virtual_motor_no_physical_dynamics' in row['flags']
                    assert row['teacher_to_actual_t10_s'] is not None
            result.update(step_logging=True,step_sample_coverage=coverage,step_quality=quality)
        trace_path=Path(state['directory'],'verification_gripper_commands.csv')
        with trace_path.open('w',newline='') as stream:
            writer=csv.writer(stream)
            writer.writerow(('role','elapsed_s','single_finger_position_m','motor_speed_limit_rad_s','enabled'))
            for role,audit in audits.items():
                trace=audit['gripper_position_history']
                if trace:
                    origin=trace[0][0]
                    writer.writerows((role,t-origin,q,v,enabled) for t,q,v,enabled in trace)
        # Keep stdout and the summary JSON small; complete wire-level J7
        # evidence lives beside the report in a named, unit-labelled CSV.
        for audit in list(audits.values())+list(startup_audit.values()):
            trace=audit.pop('gripper_position_history')
            torques=audit.pop('gripper_torque_history')
            audit['gripper_torque_summary']={'samples':len(torques),'max_abs_Nm':max((abs(v) for _,v in torques),default=0.)}
            audit['gripper_trace_summary']={'samples':len(trace),'first':trace[0] if trace else None,'last':trace[-1] if trace else None}
        result['gripper_command_trace']=str(trace_path)
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
    parser.add_argument('--gripper-outside-calibration',action='store_true')
    args=parser.parse_args()
    print(json.dumps(verify(args.single,args.fault,args.exit_failure,args.mode_failure,args.cancel_home,args.step,args.recorded_start,args.gripper_outside_calibration),ensure_ascii=False,indent=2))
