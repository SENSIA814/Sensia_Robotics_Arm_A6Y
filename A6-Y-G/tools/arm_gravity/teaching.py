"""1:1 / 2:2 joint teaching orchestration and pure incremental mapping."""
from copy import deepcopy
import csv
import json
import math
import os
from pathlib import Path
import threading
import time
import traceback
import uuid

import numpy as np

from gravity_config import ROOT, JOINTS, load_roles, load_profile, load_model, motor_limit_report
from teaching_session import TeachingArmSession
from teaching_logs import SessionLog, software_manifest, write_json

DEFAULT_PAIRS=(('teaching_left','motion_left'),('teaching_right','motion_right'))


def validate_pairs(pairs, roles):
    pairs=tuple(tuple(p) for p in pairs)
    if len(pairs) not in (1,2) or any(len(p)!=2 for p in pairs):
        raise ValueError('请选择一组或两组示教臂 → 运动臂')
    leaders=[p[0] for p in pairs];followers=[p[1] for p in pairs]
    if any(r not in ('teaching_left','teaching_right') for r in leaders) or any(r not in ('motion_left','motion_right') for r in followers):
        raise ValueError('配对方向必须为示教臂 → 运动臂')
    if len(set(leaders))!=len(leaders) or len(set(followers))!=len(followers):
        raise ValueError('每条示教臂和运动臂只能出现在一组配对中')
    selected=leaders+followers
    serials=[roles[r]['usb_serial'] for r in selected]
    ports=[str(Path(roles[r]['port']).resolve()) for r in selected]
    if len(set(serials))!=len(selected) or len(set(ports))!=len(selected):
        raise ValueError('所选机械臂的 USB 身份或实际端口重复')
    return pairs


class IncrementalTarget:
    def __init__(self, leader, follower, lower, upper):
        self.previous=self.vector(leader)
        self.target=self.vector(follower)
        self.lower=self.vector(lower);self.upper=self.vector(upper)
        if np.any(self.lower>=self.upper): raise ValueError('运动臂关节范围无效')
        # Encoder residuals are accepted at home; clamp only the command copy.
        if np.any(self.target<self.lower-.005) or np.any(self.target>self.upper+.005):
            raise ValueError('运动臂起点超出关节范围')
        self.target=np.clip(self.target,self.lower,self.upper)
        self.accumulated=self.target.copy()

    @staticmethod
    def vector(value):
        value=np.asarray(value,dtype=float)
        if value.shape!=(6,) or not np.isfinite(value).all(): raise ValueError('六轴位置必须为有限数值')
        return value.copy()

    def step(self, leader):
        current=self.vector(leader);delta=current-self.previous
        if np.max(np.abs(delta))>.10: raise ValueError('示教臂单次增量超过 0.10 rad，跟随已停止')
        target=self.accumulated+delta
        if np.any(target<self.lower-.005) or np.any(target>self.upper+.005):
            raise ValueError('累计目标超出运动臂关节范围，跟随已停止')
        self.previous=current;self.accumulated=target;self.target=np.clip(target,self.lower,self.upper)
        return delta,self.target.copy()


class TeachingCoordinator:
    def __init__(self, roles=None, session_factory=TeachingArmSession, report_dir=None):
        self.roles=roles or load_roles();self.session_factory=session_factory
        self.report_dir=Path(report_dir or ROOT.parent/'diagnostics/arm_teaching_reports')
        self.sessions={};self.worker=None;self.lock=threading.Lock();self.log=None;self.sample_meta={}
        self.limit_records={}
        self.stop_requested=threading.Event();self.emergency=threading.Event()
        self.close_after_release=threading.Event()
        self.state=dict(running=False,phase='idle',status='选择配对后启动',error='',directory=None,
                        packets=0,missed_periods=0,actual_hz=0.,close_failed=False,close_error='',log_error='')

    def update(self,**values):
        with self.lock:
            changes={k:v for k,v in values.items() if k in ('phase','status','error','close_error','close_failed') and self.state.get(k)!=v}
            self.state.update(values)
        if self.log and changes:
            self._event('state',level='ERROR' if changes.get('error') or changes.get('close_error') else 'INFO',**changes)
            self._save()

    def _event(self,kind,**values):
        if self.log:
            try:self.log.event(kind,**values)
            except Exception as exc:self._log_failure(exc)

    def _log_failure(self,exc):
        # A full/unavailable log disk must never prevent stop or release.
        with self.lock:self.state['log_error']='日志写入失败：'+str(exc)

    def _save(self):
        if self.log:
            try:self.log.snapshot(self.snapshot())
            except Exception as exc:self._log_failure(exc)

    def snapshot(self):
        with self.lock:state=deepcopy(self.state)
        state['arms']={r:s.snapshot() for r,s in list(self.sessions.items())}
        return state

    def alive(self):
        return any(s.snapshot()['running'] or (s.worker and s.worker.is_alive()) for s in self.sessions.values())

    def start(self,pairs=DEFAULT_PAIRS,frequency=50.,mock=True,with_rviz=False):
        if self.alive() or (self.worker and self.worker.is_alive()): raise ValueError('请先安全结束当前示教会话')
        pairs=validate_pairs(pairs,self.roles)
        if not math.isfinite(frequency) or not 10<=frequency<=100: raise ValueError('发送频率范围为 10～100 Hz')
        # Validate every profile before starting even the first session.
        selected=list(dict.fromkeys(r for pair in pairs for r in pair))
        parameters={r:self._parameters_for_role(r) for r in selected}
        self.limit_records={r:motor_limit_report(load_model(r,self.roles)) for r in selected}
        self.log=None
        self.sample_meta={}
        self.sessions={r:self.session_factory(roles=self.roles,report_dir=self.report_dir/'arms') for r in selected}
        self.stop_requested.clear();self.emergency.clear();self.close_after_release.clear()
        self.update(running=True,phase='connecting',status='建立所选机械臂反馈',error='',close_error='',log_error='',close_failed=False,directory=None,
                    packets=0,missed_periods=0,actual_hz=0.,pairs=pairs,frequency=frequency,mock=mock)
        self.worker=threading.Thread(target=self._run,args=(pairs,frequency,mock,with_rviz,parameters),daemon=False)
        self.worker.start()

    def _parameters_for_role(self, role):return load_profile(role,self.roles)

    def stop(self):
        if self.emergency.is_set():self.close_after_release.set()
        self.stop_requested.set()
        if not (self.worker and self.worker.is_alive()) and self.alive():
            self.update(running=True,phase='closing',close_error='',close_failed=False)
            self.worker=threading.Thread(target=self._retry_close,daemon=False);self.worker.start()
        self._event('stop_requested')

    def release(self):
        self.emergency.set();self.stop_requested.set()
        for s in list(self.sessions.values()):s.release()
        if not (self.worker and self.worker.is_alive()):
            self.worker=threading.Thread(target=self._finish_release,daemon=False);self.worker.start()
        self._event('immediate_release_requested',level='WARNING')

    def _check(self,cancel=True):
        if cancel and self.stop_requested.is_set(): raise InterruptedError('已请求停止示教')
        for role,s in self.sessions.items():
            state=s.control_state() if hasattr(s,'control_state') else s.snapshot()
            if state.get('error'):raise RuntimeError(self.roles[role]['name']+'：'+state['error'])
            if state.get('role') and not state['running']:
                raise RuntimeError(self.roles[role]['name']+'：会话已结束，停止整组示教')
            if state.get('follow_phase')==3:
                raise RuntimeError(self.roles[role]['name']+'：'+state['follow_message'])

    def _await(self,predicate,timeout=60.,cancel=True):
        deadline=time.monotonic()+timeout
        while not predicate():
            self._check(cancel)
            if time.monotonic()>deadline:raise TimeoutError('等待机械臂操作超时')
            time.sleep(.01)
        self._check(cancel)

    def _operation(self,session,operation,timeout=150.):
        role=session.snapshot().get('role')
        self._event('operation_started',role=role,operation=operation)
        future=session.submit(operation)
        self._await(future.done,timeout)
        try:future.result()
        except Exception as exc:raise RuntimeError(self.roles[role]['name']+' / '+operation+'：'+str(exc)) from exc
        self._event('operation_completed',role=role,operation=operation)
        self._save()

    def _sample(self,session):
        state=session.control_state(feedback=True) if hasattr(session,'control_state') else session.snapshot()
        hw=state.get('hardware',{})
        now=time.monotonic()
        # Source timestamps, not UI refresh timestamps, authorize targets.
        stamp=hw.get('sample_source_time');diagnostic=hw.get('diagnostic_source_time')
        healthy=session._fresh(enabled=True,sample=hw)
        # Position samples authorize the high-rate reference. The separate
        # diagnostic stream can trail by one bus cycle; retain its established
        # 0.5 s watchdog, including the motor age already reported in that frame.
        position_fresh=stamp is not None and 0<=now-stamp<.15
        diagnostic_fresh=diagnostic is not None and 0<=now-diagnostic<.5
        if diagnostic_fresh:
            diagnostic_fresh=all(row.get('feedback_age',0)+now-diagnostic<.5 for row in hw.get('rows',{}).values())
        if not state['active'] or not healthy or not position_fresh or not diagnostic_fresh:
            detail=dict(role=state['role'],active=state['active'],driver_healthy=healthy,
                position_age=None if stamp is None else now-stamp,
                diagnostic_age=None if diagnostic is None else now-diagnostic,hardware=hw)
            folder=self.snapshot().get('directory')
            if folder:Path(folder,'feedback_fault.json').write_text(json.dumps(detail,ensure_ascii=False,indent=2))
            raise RuntimeError(self.roles[state['role']]['name']+'：反馈过期或电机状态异常，停止整组跟随'
                +f"（位置年龄 {detail['position_age']} s，状态年龄 {detail['diagnostic_age']} s，有效状态 {healthy}）")
        q=IncrementalTarget.vector(hw['q'][:6]);v=IncrementalTarget.vector(hw['velocity'][:6])
        limits=np.asarray(self.limit_records[state['role']]['feedback_velocity_limits'])
        if np.any(np.abs(v)>limits):
            joint=int(np.argmax(np.abs(v)/limits))
            raise RuntimeError(f"{state['role']} / J{joint+1} 反馈速度 {v[joint]:.6f} rad/s 超过电机 VMAX 换算范围 {limits[joint]:.6f} rad/s")
        self.sample_meta[state['role']]=dict(velocity=v.tolist(),position_age=now-stamp,diagnostic_age=now-diagnostic)
        return q

    def _run(self,pairs,frequency,mock,with_rviz,parameters):
        folder=None;failed=False
        try:
            folder=self.report_dir/('teaching_'+uuid.uuid4().hex[:10]);folder.mkdir(parents=True)
            self.update(directory=str(folder))
            self.log=SessionLog(folder)
            self._event('session_started',pairs=pairs,frequency=frequency,mock=mock,
                        leader_backend='mit_gravity',follower_backend='pos_vel')
            write_json(folder/'software.json',software_manifest(ROOT))
            write_json(folder/'motor_limits.json',self.limit_records)
            for role,record in self.limit_records.items():
                self._event('motor_feedback_limits',role=role,usb_serial=record['usb_serial'],
                    limits_rad_s=record['feedback_velocity_limits'],source=record['source'],
                    observed_at=record['observed_at'],policy=record['policy'])
            (folder/'session.json').write_text(json.dumps(dict(pairs=pairs,frequency=frequency,mock=mock,
                roles={r:self.roles[r] for r in self.sessions},leader_backend='mit_gravity',
                follower_backend='pos_vel',application_log=os.environ.get('A6Y_TEACHING_APP_LOG')),ensure_ascii=False,indent=2))
            for role,s in self.sessions.items():
                if self.stop_requested.is_set():raise InterruptedError()
                s.start(role,mock,with_rviz,parameters[role])
            self._await(lambda:all(s.snapshot()['ready'] for s in self.sessions.values()),45.)
            self._save()
            # Refuse the complete batch before acquiring output on any arm.
            for s in self.sessions.values():
                reason=s.enable_block_reason()
                if reason:raise RuntimeError(reason)
            self.update(phase='homing')
            for role,s in self.sessions.items():
                self.update(status=self.roles[role]['name']+'：启动回零；已回零机械臂保持位置')
                self._operation(s,'home')
            for s in self.sessions.values():
                if np.max(np.abs(self._sample(s)))>.005:raise RuntimeError('所有机械臂必须在零位 ±0.005 rad 内才可开始示教')
            trackers={}
            for leader,follower in pairs:
                source=self._sample(self.sessions[leader]);target=self._sample(self.sessions[follower])
                model=self.sessions[follower].model
                trackers[follower]=IncrementalTarget(source,target,
                    [model['joints'][n]['lower'] for n in JOINTS[:6]],
                    [model['joints'][n]['upper'] for n in JOINTS[:6]])
            # Release the leaders to gravity only AFTER the all-arm home barrier.
            for leader,_ in pairs:self._operation(self.sessions[leader],'gravity')
            # Start services concurrently, while publishing an anchored target to
            # each acknowledged follower so the first cannot time out on the second.
            futures={f:self.sessions[f].submit('follow') for _,f in pairs}
            sequence=0;deadline=time.monotonic()+3.
            while True:
                self._check();sequence+=1
                for follower,future in futures.items():
                    if future.done():
                        future.result()
                        self.sessions[follower].publish_reference(trackers[follower].target,sequence,time.time_ns())
                if all(f.done() for f in futures.values()):break
                if time.monotonic()>deadline:raise TimeoutError('运动臂跟随启动超时')
                self.stop_requested.wait(.02)
            self.update(phase='following',status='动力示教运行中：拖动示教臂，运动臂增量跟随')
            self._stream(pairs,trackers,frequency,sequence,folder)
        except InterruptedError:
            self._event('cancelled')
        except Exception as exc:
            failed=True
            self._event('first_failure',level='ERROR',message=str(exc),traceback=traceback.format_exc(),phase=self.snapshot()['phase'])
            self.update(error=str(exc),status='操作失败，停止整组跟随；保留反馈等待处理')
            if folder:
                try:write_json(folder/'first_failure.json',dict(message=str(exc),traceback=traceback.format_exc(),state=self.snapshot()))
                except Exception as log_exc:self._log_failure(log_exc)
        finally:
            if self.emergency.is_set():
                self._finish_release()
            elif failed:
                # A failed startup home is not permission to start another home.
                # Keep support/hold and the first fault; Stop/close explicitly
                # invokes the existing protected return when the user is ready.
                self._hold_sessions()
                for s in self.sessions.values():
                    if s.snapshot()['running'] and not s.snapshot()['active'] and not s.snapshot()['busy']:s.stop()
                unresolved=any(s.snapshot()['active'] or (s.snapshot().get('role') and s.snapshot().get('release_confirmed') is not True)
                               for s in self.sessions.values())
                self.update(running=False,phase='error',close_failed=bool(unresolved),
                    status='整组已停止；未自动再次回零。可查看/导出日志，处理后点击回零退出或托住后立即失能')
            else:self._close_sessions()
            self._save()

    def _stream(self,pairs,trackers,frequency,sequence,folder):
        period=1./frequency;deadline=started=time.monotonic();sent=missed=0
        with (folder/'references.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['wall_time','sequence','leader','follower']+
                [f'{kind}_j{i}' for kind in ('delta','target','leader_q','follower_q','tracking_error','leader_v','follower_v') for i in range(1,7)]+
                ['leader_position_age_s','leader_diagnostic_age_s','follower_position_age_s','follower_diagnostic_age_s','loop_lateness_s'])
            while not self.stop_requested.is_set():
                delay=deadline-time.monotonic()
                if delay>0 and self.stop_requested.wait(delay):break
                self._check();now=time.monotonic()
                if now-deadline>.15:raise RuntimeError('目标发送循环延迟超过 150 ms，跟随已停止')
                sequence+=1;stamp=time.time_ns();batch=[]
                for leader,follower in pairs:
                    measured=self._sample(self.sessions[follower])
                    source=self._sample(self.sessions[leader]);delta,target=trackers[follower].step(source)
                    if np.max(np.abs(target-measured))>.15:
                        joint=int(np.argmax(np.abs(target-measured)))
                        raise RuntimeError(f'{follower} / J{joint+1} 累计目标与反馈偏差 {target[joint]-measured[joint]:.6f} rad 超过 0.15 rad')
                    batch.append((leader,follower,delta,target,source,measured))
                # Validate both pairs before publishing either pair's new target.
                for leader,follower,delta,target,source,measured in batch:
                    self.sessions[follower].publish_reference(target,sequence,stamp)
                    source_meta=self.sample_meta[leader];target_meta=self.sample_meta[follower]
                    writer.writerow([stamp/1e9,sequence,leader,follower,*delta,*target,*source,*measured,*(target-measured),
                        *source_meta['velocity'],*target_meta['velocity'],source_meta['position_age'],source_meta['diagnostic_age'],
                        target_meta['position_age'],target_meta['diagnostic_age'],now-deadline])
                sent+=1;deadline+=period
                if deadline<time.monotonic():
                    skip=int((time.monotonic()-deadline)/period)+1;missed+=skip;deadline+=skip*period
                if sent%max(1,int(frequency/5))==0:
                    f.flush();self.update(packets=sent,missed_periods=missed,actual_hz=(sent-1)/max(period,now-started))

    def _hold_sessions(self):
        # Freeze every follower before the first potentially long home operation.
        for s in self.sessions.values():
            state=s.snapshot()
            if state['active'] and not state['busy'] and state.get('follow_phase',0):
                try:
                    future=s.submit('hold');deadline=time.monotonic()+2
                    while not future.done() and time.monotonic()<deadline:time.sleep(.01)
                    if future.done():future.result()
                    else:raise TimeoutError('保持命令确认超时')
                    self._event('hold_confirmed',role=state.get('role'))
                except Exception as exc:
                    self._event('hold_unverified',level='ERROR',role=state.get('role'),message=str(exc))
                    # Native 250 ms watchdog also latches hold.

    def _close_sessions(self):
        self.update(phase='closing',status='停止发送目标；各臂依次回零、核验到位并失能',close_failed=False,close_error='')
        self._hold_sessions()
        for role,s in self.sessions.items():
            if self.emergency.is_set():
                self._finish_release();return
            state=s.snapshot()
            if not state['running']:
                if state.get('release_confirmed') is False:
                    self.update(running=False,phase='error',close_failed=True,status='失能未确认，请保留窗口处理',
                                close_error=state.get('error') or state.get('notice','失能未确认'));return
                continue
            s.stop();deadline=time.monotonic()+180.
            while s.snapshot()['running']:
                state=s.snapshot()
                if self.emergency.is_set():
                    self._finish_release();return
                if state.get('close_failed') or time.monotonic()>deadline:
                    self.update(running=False,phase='error',close_failed=True,status='回零或失能失败，退出已取消',
                        close_error=self.roles[role]['name']+'：'+(state.get('notice') or state.get('error') or '退出等待超时'));return
                time.sleep(.02)
            if s.worker:s.worker.join(2.)
            if s.snapshot().get('release_confirmed') is not True:
                self.update(running=False,phase='error',close_failed=True,status='失能未确认，退出已取消',close_error=self.roles[role]['name']+'：失能未确认');return
            self._event('arm_closed',role=role,release_confirmed=True)
        self.update(running=False,phase='stopped',status='示教会话已安全结束，已连接机械臂均确认失能')

    def _retry_close(self):
        self.emergency.clear()
        self._close_sessions()
        self._save()

    def _finish_release(self):
        self.update(running=True,phase='releasing',status='正在核验全部机械臂失能')
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            states=[s.snapshot() for s in self.sessions.values()]
            if all(s.get('release_confirmed') is True and not s.get('active') and not s.get('busy') for s in states):break
            time.sleep(.02)
        unverified=[role for role,s in self.sessions.items() if s.snapshot().get('release_confirmed') is not True or s.snapshot().get('active')]
        self.update(running=False,phase='error' if unverified else 'released',close_failed=bool(unverified),
                    close_error='失能未确认：'+', '.join(unverified) if unverified else '',
                    status='失能核验未完成，请保留窗口处理' if unverified else '全部失能已确认；反馈与日志仍可查看')
        self._event('release_result',unverified=unverified)
        self._save()
        if not unverified and self.close_after_release.is_set():
            self.emergency.clear();self._close_sessions();self._save()
