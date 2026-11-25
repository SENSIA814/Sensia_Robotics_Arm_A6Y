#!/usr/bin/env python3
"""Exercise actual Qt buttons and ROS dry-run controller; never open hardware."""
import argparse
import json
import os
from pathlib import Path
import time
import tempfile
from unittest.mock import patch
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QFont
from gravity_view import GravityWindow
from gravity_config import ROLE_ORDER, ROOT, LIVE_KEYS, save_profile, load_profile, sag_trial_parameters


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--rviz',action='store_true')
    parser.add_argument('--output',default=str(ROOT.parent/'diagnostics/arm_gravity_ui'))
    args=parser.parse_args()
    os.environ.setdefault('CYCLONEDDS_URI','file://'+str((ROOT/'scripts/gravity_dds.xml').resolve()))
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    app=QApplication([]);app.setFont(QFont('Noto Sans CJK SC',10))
    w=GravityWindow();w.show();w.rviz.setChecked(args.rviz)
    def wait(predicate,timeout=60,check_errors=True):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            app.processEvents()
            w.refresh()
            states={role:session.snapshot() for role,session in w.sessions.items()}
            if check_errors:
                errors={r:s.get('error') or s.get('notice') for r,s in states.items()
                        if s.get('error') or s.get('close_failed')}
                if errors:raise AssertionError(json.dumps(errors,ensure_ascii=False))
            if predicate(states):return states
            time.sleep(.02)
        raise AssertionError('UI timeout '+str({r:s.snapshot() for r,s in w.sessions.items()}))
    try:
        for role in ROLE_ORDER:
            w.buttons[role].click();app.processEvents();assert w.role==role
            w.batch_checks[role].setChecked(True)
        assert w.mode.currentData() is True
        print('Four-arm UI: establish read-only dry-run feedback',flush=True)
        w.batch_start.click()
        wait(lambda states:len(states)==4 and all(s['ready'] for s in states.values()))
        assert all(not s.snapshot()['active'] for s in w.sessions.values())
        assert all(s.snapshot()['mock'] and s.stack.mock for s in w.sessions.values())
        assert len({s.namespace for s in w.sessions.values()})==4
        assert all(b.isEnabled() for b in w.buttons.values())
        assert w.enable_button.isEnabled()
        print('Four-arm UI: explicit batch enable',flush=True)
        w.batch_enable.click()
        initial=wait(lambda states:all(s['active'] and s.get('sequence',0)>20 for s in states.values()))
        target='teaching_right';other='teaching_left'
        w.buttons[target].click();app.processEvents()
        w.grab().save(str(out/'gravity-live.png'))
        if args.rviz:app.primaryScreen().grabWindow(0).save(str(out/'gravity-rviz.png'))
        w.tabs.setCurrentIndex(1);app.processEvents()
        w.trial.click();app.processEvents()
        trial=sag_trial_parameters()
        assert all(w.parameters()[key]==trial[key] for key in LIVE_KEYS)
        # Ensure this remains a real pending edit even if a saved profile already
        # happens to use the trial values.
        value=w.editors['torque_scales'][1].value()
        if w.parameters()==initial[target]['applied_parameters']:
            w.editors['torque_scales'][1].setValue(value+.05)
        target_draft=w.parameters()
        w.buttons[other].click();app.processEvents()
        e=w.editors['damping_gains'][1];e.setValue(e.value()+.001 if e.value()<4.999 else e.value()-.001)
        other_draft=w.parameters()
        w.buttons[target].click();app.processEvents()
        assert w.parameters()==target_draft
        assert w.plot.samples is w.plot_histories[target]
        assert all(w.plot_histories[r] for r in ROLE_ORDER)
        assert len({id(w.plot_histories[r]) for r in ROLE_ORDER})==4
        assert w.apply.isEnabled() and not w.save.isEnabled()
        assert not w.editors['effort_signs'][0].isEnabled()
        assert not w.editors['torque_limits'][0].isEnabled()
        generation=w.session.stack.control_generation
        w.apply.click()
        states=wait(lambda states:not states[target]['busy'] and states[target]['tuning_revision']==1)
        assert w.session.snapshot()['applied_parameters']==w.parameters()
        assert w.save.isEnabled() and w.session.stack.control_generation==generation
        assert all(states[r]['applied_parameters']==initial[r]['applied_parameters'] for r in ROLE_ORDER if r!=target)
        w.buttons[other].click();app.processEvents()
        assert w.parameters()==other_draft
        assert w.plot.samples is w.plot_histories[other]
        w.buttons[target].click();app.processEvents()
        # Saving during live operation must persist only confirmed parameters;
        # never overwrite a physical arm's profile from a UI test.
        with tempfile.TemporaryDirectory() as folder:
            with patch('gravity_view.save_profile',side_effect=lambda r,p,roles:save_profile(r,p,roles,directory=folder)):
                w.save.click();app.processEvents()
            assert load_profile(w.role,directory=folder)==w.parameters()
            with patch('gravity_view.load_profile',side_effect=lambda r,roles:load_profile(r,roles,directory=folder)),\
                 patch('gravity_view.PROFILE_DIR',Path(folder)):
                reopened=GravityWindow();reopened.select_role(w.role)
                assert reopened.parameters()==w.parameters()
                reopened.close()
        w.grab().save(str(out/'gravity-parameters.png'))
        w.release_button.click()
        states=wait(lambda states:states[target]['release_confirmed'] and not states[target]['busy'])
        assert all(states[r]['active'] for r in ROLE_ORDER if r!=target)
        w.enable_button.click();wait(lambda states:all(s['active'] and not s['busy'] for s in states.values()))
        print('Four-arm UI: sequential planned return, disable, and close',flush=True)
        w.close()
        close_order=[]
        def all_closed(states):
            returning=[r for r,s in states.items() if s['running'] and s.get('closing_requested')]
            assert len(returning)<=1,returning
            if returning and (not close_order or close_order[-1]!=returning[0]):close_order.append(returning[0])
            return all(not s['running'] for s in states.values()) and all(not w.session_alive(s) for s in w.sessions.values())
        states=wait(all_closed,timeout=240)
        for session in w.sessions.values():session.worker.join(10)
        app.processEvents();w.refresh();app.processEvents()
        assert not w.isVisible() and all(not s.worker.is_alive() for s in w.sessions.values())
        assert close_order==list(ROLE_ORDER),close_order
        assert all(s['release_confirmed'] is True for s in states.values())
        previous_closed=0.
        reports={}
        for role in ROLE_ORDER:
            reports[role]=states[role]['directory']
            events=[json.loads(line) for line in (Path(reports[role])/'events.jsonl').read_text().splitlines()]
            planning=next(e for e in events if e['kind']=='home_planning')
            reached=next(e for e in events if e['kind']=='home_reached')
            released=next(e for e in events if e['kind']=='released' and e['time']>=reached['time'])
            closed=next(e for e in events if e['kind']=='closed')
            assert previous_closed<=planning['time']<=reached['time']<=released['time']<=closed['time']
            assert closed['release_confirmed'] is True
            previous_closed=closed['time']
        result=dict(result='PASS',physical_devices=0,qt_role_buttons=4,rviz=args.rviz,
                    parallel_dry_run_arms=4,batch_feedback_and_explicit_enable=True,
                    live_tuning_and_save_reload=True,per_arm_draft_and_plot_isolation=True,
                    mock_enable_release_reenable_close=True,sequential_return_order=close_order,
                    all_four_disable_confirmed=True,cyclonedds_config=os.environ['CYCLONEDDS_URI'],
                    report=reports[target],reports=reports)
        (out/'verification.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result),flush=True)
    finally:
        # Release and join every mock session even if the selected detail changed
        # or an assertion failed midway through the batch.
        w.release_every_arm()
        try:wait(lambda states:all(s['release_confirmed'] or not s['running'] for s in states.values()),timeout=15,check_errors=False)
        except AssertionError:pass
        for session in w.sessions.values():session.stop()
        try:wait(lambda states:all(not s['running'] for s in states.values()),timeout=40,check_errors=False)
        except AssertionError:
            for session in w.sessions.values():
                if session.snapshot()['running']:session.stop_event.set()
        for session in w.sessions.values():
            if session.worker:session.worker.join(20)
        w.close();app.processEvents()

if __name__ == '__main__':main()
