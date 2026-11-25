#!/usr/bin/env python3
"""Exercise Qt log refresh and live ZIP export against four native PTYs only."""
import json
import os
from pathlib import Path
import time
from unittest.mock import patch
import zipfile

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication

from gravity_config import ROOT, load_roles
from teaching import DEFAULT_PAIRS
from teaching_port import TeachingCoordinator
from teaching_view import TeachingWindow
from verify_multi import ProcessMotorEmulator,cleanup


def verify():
    app=QApplication.instance() or QApplication([]);app.setFont(QFont('Noto Sans CJK SC',10))
    emulators={};c=None;window=None
    def wait(predicate,timeout=220):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            app.processEvents()
            state=c.snapshot()
            if state['error'] or state.get('close_error'):raise AssertionError(state)
            if predicate():return
            time.sleep(.01)
        raise AssertionError('UI wait timeout: '+str(c.snapshot()))
    def pause(seconds):
        deadline=time.monotonic()+seconds;wait(lambda:time.monotonic()>=deadline,seconds+2)
    try:
        roles=load_roles()
        for pair in DEFAULT_PAIRS:
            for role in pair:
                em=ProcessMotorEmulator(role,native_port=True);emulators[role]=em
                em.snapshot(('set_q',[.1,.1,.1,0.,0.,0.,-.01] if role.startswith('teaching_') else [0.,.03,.03,0.,0.,0.,-.01]));roles[role]=dict(roles[role],port=em.port)
        assert all(a['port'].startswith('/dev/pts/') for a in roles.values())
        c=TeachingCoordinator(roles=roles,report_dir=ROOT.parent/'diagnostics/arm_teaching_test_reports');window=TeachingWindow(c);window.show();app.processEvents()
        assert window.frequency.value()==window.frequency.minimum()==window.frequency.maximum()==100
        window.mode.setCurrentIndex(window.mode.findData(False))
        window.start_button.click();wait(lambda:c.snapshot()['phase']=='following')
        for k in range(60):
            emulators['teaching_left'].snapshot(('set_q',[.024*k/60]+[0.]*6))
            emulators['teaching_right'].snapshot(('set_q',[-.018*k/60]+[0.]*6));pause(.02)
        directory=Path(c.snapshot()['directory']);bundle=directory.parent/(directory.name+'_live.zip')
        initial_packets=c.snapshot()['packets']
        with patch('teaching_view.QFileDialog.getSaveFileName',return_value=(str(bundle),'')):
            window.export_logs()
        wait(lambda:window.export_worker is not None and not window.export_worker.is_alive(),10)
        wait(lambda:c.snapshot()['packets']>initial_packets+25)
        wait(lambda:window.log_notice.text()!='正在打包日志…',3)
        assert '诊断包已保存' in window.log_notice.text(),window.log_notice.text()
        with zipfile.ZipFile(bundle) as archive:
            assert archive.testzip() is None
            assert json.loads(archive.read('bundle.json'))['live']
            for role in emulators:
                assert 'arms/'+role+'/events.jsonl' in archive.namelist()
                assert 'arms/'+role+'/controller_manager.log' in archive.namelist()
            assert 'references.csv' in archive.namelist()
            assert 'frequency.csv' in archive.namelist()
            assert 'step_arms.csv' in archive.namelist()
            assert 'step_targets.csv' in archive.namelist()
            assert 'step_quality.json' in archive.namelist()
            assert json.loads(archive.read('session.json'))['configured_rates']['control_hz']==250
        screenshot=directory/'ui_live.png';window.grab().save(str(screenshot))
        exit_started=time.monotonic();window.close()
        wait(lambda:c.snapshot()['phase']=='closing',10)
        pause(2.)
        assert window.isVisible() and window.release_button.isEnabled() and window.export_button.isEnabled()
        assert '回零' in window.status.text()
        assert window.close_pending
        wait(lambda:not window.isVisible(),60)
        exit_elapsed=time.monotonic()-exit_started
        assert exit_elapsed<40.,exit_elapsed
        state=c.snapshot();assert state['phase']=='stopped' and not c.alive(),state
        assert json.loads((directory/'step_quality.json').read_text())['complete']
        assert (directory/'step_summary.json').is_file()
        assert all(s.snapshot()['release_confirmed'] for s in c.sessions.values())
        result=dict(result='PASS',physical_devices=0,four_arm_ui_refresh=True,live_export=True,
                    packets_before_export=initial_packets,packets=state['packets'],actual_hz=state['actual_hz'],
                    protected_window_close=True,exit_ui_responsive=True,exit_elapsed_s=exit_elapsed,bundle=str(bundle),screenshot=str(screenshot),report=str(directory))
        (directory/'verification_ui.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        return result
    finally:
        if window:window.timer.stop()
        if c and c.alive():
            c.release()
            if c.worker:c.worker.join(15)
            cleanup(list(c.sessions.values()))
        for em in emulators.values():em.close()
        if window:window.deleteLater();app.processEvents()


if __name__=='__main__':print(json.dumps(verify(),ensure_ascii=False,indent=2))
