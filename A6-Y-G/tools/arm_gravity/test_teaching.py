from concurrent.futures import Future
from copy import deepcopy
import json
import os
import tempfile
import time
import unittest
import zipfile
from unittest.mock import patch

import numpy as np

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5.QtWidgets import QApplication
from gravity_config import JOINTS, load_roles, load_model, motor_limit_report
from teaching import DEFAULT_PAIRS, IncrementalTarget, TeachingCoordinator, validate_pairs
from teaching_view import TeachingWindow


class MappingTests(unittest.TestCase):
    def test_single_default_and_cross_pairs(self):
        roles=load_roles()
        for pairs in (DEFAULT_PAIRS,DEFAULT_PAIRS[:1],(('teaching_right','motion_left'),),
                      (('teaching_left','motion_right'),('teaching_right','motion_left'))):
            self.assertEqual(validate_pairs(pairs,roles),tuple(pairs))

    def test_duplicates_invalid_roles_and_duplicate_usb_rejected(self):
        roles=load_roles()
        for pairs in ((),(('motion_left','teaching_left'),),
                      (('teaching_left','motion_left'),('teaching_right','motion_left')),
                      (('teaching_left','motion_left'),('teaching_left','motion_right'))):
            with self.assertRaises(ValueError):validate_pairs(pairs,roles)
        roles['motion_left']['usb_serial']=roles['teaching_left']['usb_serial']
        with self.assertRaises(ValueError):validate_pairs(DEFAULT_PAIRS,roles)

    def test_increment_accumulation_stationary_and_reversal(self):
        mapping=IncrementalTarget([.003]*6,[-.002]*6,[-1]*6,[1]*6)
        delta,target=mapping.step([.043]*6)
        np.testing.assert_allclose(delta,.04);np.testing.assert_allclose(target,.038)
        for _ in range(100):np.testing.assert_allclose(mapping.step([.043]*6)[1],.038)
        np.testing.assert_allclose(mapping.step([.013]*6)[1],.008)

    def test_bad_samples_never_mutate_target(self):
        mapping=IncrementalTarget([0]*6,[0]*6,[-.05]*6,[.05]*6)
        for bad in ([float('nan')]*6,[float('inf')]*6,[0]*5,[.2]*6,[.08]*6):
            with self.assertRaises(ValueError):mapping.step(bad)
            np.testing.assert_array_equal(mapping.target,[0]*6)
            np.testing.assert_array_equal(mapping.previous,[0]*6)

    def test_boundary_residual_does_not_discard_cumulative_motion(self):
        mapping=IncrementalTarget([0]*6,[0]*6,[0]*6,[1]*6)
        mapping.step([-.002]*6);mapping.step([-.004]*6)
        with self.assertRaises(ValueError):mapping.step([-.006]*6)
        np.testing.assert_allclose(mapping.step([.002]*6)[1],.002)


class FakeArm:
    events=[];failed_home=None;failed_close=None;blocked=None
    def __init__(self,**_):
        self.worker=None;self.q=np.zeros(7)
        self.model={'joints':{n:{'lower':-2.,'upper':2.} for n in JOINTS}}
        self.state=dict(running=False,ready=False,active=False,busy=False,status='未连接',error='',
            notice='',release_confirmed=None,follow_phase=0,follow_message='',close_failed=False)
    def snapshot(self):
        state=deepcopy(self.state)
        state['hardware']=dict(q=self.q.tolist(),velocity=[0]*7,sample_source_time=time.monotonic(),diagnostic_source_time=time.monotonic())
        return state
    def start(self,role,*_):self.role=role;self.state.update(role=role,running=True,ready=True)
    def _fresh(self,**_):return True
    def enable_block_reason(self):return '残留使能' if self.role==self.blocked else ''
    def submit(self,operation):
        self.events.append((self.role,operation));future=Future()
        if operation=='home' and self.role==self.failed_home:future.set_exception(RuntimeError('模拟回零失败'));return future
        self.state.update(active=True,follow_phase={'home':2,'hold':2,'follow':1,'gravity':0}[operation])
        future.set_result(True);return future
    def publish_reference(self,target,sequence,stamp):self.events.append((self.role,'packet',sequence,list(target)))
    def stop(self):
        self.events.append((self.role,'stop'))
        if self.role==self.failed_close:self.state.update(close_failed=True,notice='模拟退出回零失败');return
        self.state.update(running=False,ready=False,active=False,release_confirmed=True)
    def release(self):
        self.events.append((getattr(self,'role','unknown'),'release'));self.state.update(active=False,release_confirmed=True)


class CoordinatorTests(unittest.TestCase):
    def setUp(self):
        FakeArm.events=[];FakeArm.failed_home=FakeArm.failed_close=FakeArm.blocked=None
        self.temp=tempfile.TemporaryDirectory();self.c=TeachingCoordinator(session_factory=FakeArm,report_dir=self.temp.name)
        self.c.limit_records={r:motor_limit_report(load_model(r)) for r in load_roles()}
    def tearDown(self):
        FakeArm.failed_close=None;self.c.stop()
        if self.c.worker:self.c.worker.join(3)
        self.temp.cleanup()
    def wait(self,predicate):
        until=time.monotonic()+3
        while not predicate():
            if time.monotonic()>until:self.fail(str(self.c.snapshot()))
            time.sleep(.01)
    def test_all_homed_before_any_teacher_gravity_or_follower_packet(self):
        self.c.start();self.wait(lambda:self.c.snapshot()['packets']>=10)
        events=FakeArm.events;homes=[i for i,e in enumerate(events) if e[1]=='home']
        self.assertEqual(len(homes),4)
        self.assertTrue(all(i>max(homes) for i,e in enumerate(events) if e[1] in ('gravity','follow','packet')))
        self.c.stop();self.c.worker.join(3)
        self.assertEqual(self.c.snapshot()['phase'],'stopped')
        self.assertTrue(all(s.state['release_confirmed'] for s in self.c.sessions.values()))
    def test_one_to_one_only_connects_selected_arms(self):
        pairs=(('teaching_right','motion_left'),)
        self.c.start(pairs);self.wait(lambda:self.c.snapshot()['phase']=='following')
        self.assertEqual(set(self.c.sessions),{'teaching_right','motion_left'})
    def test_any_home_failure_prevents_follow(self):
        FakeArm.failed_home='teaching_right';self.c.start();self.wait(lambda:not self.c.snapshot()['running'])
        self.assertFalse(any(e[1] in ('gravity','follow','packet') for e in FakeArm.events))
        self.assertIn('回零失败',self.c.snapshot()['error'])

    def test_failed_startup_does_not_automatically_home_active_arms_again(self):
        FakeArm.failed_home='teaching_right';self.c.start();self.wait(lambda:not self.c.snapshot()['running'])
        self.c.worker.join(3)
        self.assertNotIn(('teaching_left','stop'),FakeArm.events)
        self.assertNotIn(('motion_left','stop'),FakeArm.events)
        self.assertTrue(self.c.alive())
        folder=self.c.snapshot()['directory']
        with open(folder+'/first_failure.json') as f:failure=json.load(f)
        self.assertIn('模拟回零失败',failure['message']);self.assertIn('RuntimeError',failure['traceback'])
        FakeArm.failed_close='teaching_left';self.c.stop();self.c.worker.join(3)
        state=self.c.snapshot();self.assertIn('模拟回零失败',state['error'])
        self.assertIn('退出回零失败',state['close_error'])
        FakeArm.failed_close=None;self.c.stop();self.c.worker.join(3)
        with open(folder+'/result.json') as f:saved=json.load(f)
        self.assertEqual(saved['phase'],'stopped');self.assertEqual(saved['error'],state['error'])
        self.assertFalse(saved['close_failed'])

    def test_immediate_release_preserves_first_failure_and_updates_result(self):
        FakeArm.failed_home='teaching_right';self.c.start();self.wait(lambda:not self.c.snapshot()['running'])
        self.c.worker.join(3);self.c.release();self.c.worker.join(3)
        state=self.c.snapshot();self.assertEqual(state['phase'],'released');self.assertFalse(state['close_failed'])
        self.assertIn('模拟回零失败',state['error'])
        with open(state['directory']+'/result.json') as f:self.assertEqual(json.load(f)['phase'],'released')

    def test_close_requested_during_release_finishes_protected_exit(self):
        self.c.start();self.wait(lambda:self.c.snapshot()['phase']=='following')
        self.c.emergency.set();self.c.close_after_release.set()
        for s in self.c.sessions.values():s.release()
        self.c.stop_requested.set();self.c.worker.join(3)
        self.assertEqual(self.c.snapshot()['phase'],'stopped');self.assertFalse(self.c.alive())

    def test_log_io_failure_cannot_block_stop_or_release(self):
        self.c.start();self.wait(lambda:self.c.snapshot()['phase']=='following')
        with patch.object(self.c.log,'event',side_effect=OSError('disk full')),patch.object(self.c.log,'snapshot',side_effect=OSError('disk full')):
            self.c.release();self.c.worker.join(3)
            self.assertTrue(all(s.state['release_confirmed'] for s in self.c.sessions.values()))
            self.c.stop();self.c.worker.join(3)
            self.assertEqual(self.c.snapshot()['phase'],'stopped');self.assertFalse(self.c.alive())
            self.assertIn('disk full',self.c.snapshot()['log_error'])
    def test_batch_precheck_prevents_partial_enable(self):
        FakeArm.blocked='motion_right';self.c.start();self.wait(lambda:not self.c.snapshot()['running'])
        self.assertFalse(any(e[1]=='home' for e in FakeArm.events))
    def test_return_failure_retains_session_and_retry(self):
        self.c.start();self.wait(lambda:self.c.snapshot()['phase']=='following')
        FakeArm.failed_close='teaching_left';self.c.stop();self.c.worker.join(3)
        self.assertTrue(self.c.snapshot()['close_failed']);self.assertTrue(self.c.alive())
        FakeArm.failed_close=None;self.c.stop();self.c.worker.join(3)
        self.assertFalse(self.c.alive());self.assertEqual(self.c.snapshot()['phase'],'stopped')
    def test_stale_feedback_aborts_whole_batch(self):
        self.c.start();self.wait(lambda:self.c.snapshot()['phase']=='following')
        self.c.sessions['teaching_left']._fresh=lambda **_:False
        self.wait(lambda:not self.c.snapshot()['running'])
        self.assertIn('反馈过期',self.c.snapshot()['error'])

    def test_position_and_diagnostic_streams_use_their_own_freshness_limits(self):
        arm=FakeArm();arm.start('teaching_left');arm.state['active']=True
        state=arm.snapshot();now=time.monotonic()
        state['hardware']['sample_source_time']=now-.01
        state['hardware']['diagnostic_source_time']=now-.20
        with patch.object(arm,'snapshot',return_value=state):
            self.c._sample(arm) # A one-cycle-old diagnostic is still valid.
            state['hardware']['sample_source_time']=now-.16
            with self.assertRaises(RuntimeError):self.c._sample(arm)
            state['hardware']['sample_source_time']=now-.01
            state['hardware']['rows']={'joint1':{'feedback_age':.31}}
            with self.assertRaises(RuntimeError):self.c._sample(arm) # Combined age exceeds .5.

    def test_recorded_start_and_exit_speeds_use_each_motor_range(self):
        for role,limit in (('teaching_left',50.),('motion_left',50.),('teaching_right',30.),('motion_right',30.)):
            arm=FakeArm();arm.start(role);arm.state['active']=True
            state=arm.snapshot()
            with patch.object(arm,'snapshot',return_value=state):
                for speed in (.525031,-.549451,limit,-limit):
                    state['hardware']['velocity'][3]=speed
                    self.c._sample(arm)
                state['hardware']['velocity'][3]=limit+.001
                with self.assertRaisesRegex(RuntimeError,f'J4.*{limit:.6f}'):
                    self.c._sample(arm)
                state['hardware']['velocity'][3]=0.
                state['hardware']['velocity'][0]=10.001
                with self.assertRaisesRegex(RuntimeError,'J1.*10.000000'):
                    self.c._sample(arm)
    def test_ui_defaults_single_pair_and_failed_close_keeps_window(self):
        app=QApplication.instance() or QApplication([])
        w=TeachingWindow(self.c);w.timer.stop();w.show()
        self.assertEqual(w.pairs(),DEFAULT_PAIRS)
        w.pair_checks[1].setChecked(False);self.assertEqual(w.pairs(),DEFAULT_PAIRS[:1])
        w.start();self.wait(lambda:self.c.snapshot()['phase']=='following');w.refresh()
        self.assertFalse(w.leaders[0].isEnabled())
        FakeArm.failed_close='teaching_left';w.close();self.c.worker.join(3);w.refresh();app.processEvents()
        self.assertTrue(w.isVisible());self.assertFalse(w.close_pending)
        FakeArm.failed_close=None;w.close();self.c.worker.join(3);w.refresh();app.processEvents()
        self.assertFalse(w.isVisible())

    def test_gravity_parent_close_uses_teaching_exit_gate(self):
        from gravity_view import GravityWindow
        app=QApplication.instance() or QApplication([])
        with patch('gravity_view.inventory',return_value=[]):
            parent=GravityWindow();parent.timer.stop();parent.show()
        child=TeachingWindow(self.c);child.timer.stop();parent.teaching_window=child;child.show()
        self.c.start();self.wait(lambda:self.c.snapshot()['phase']=='following')
        FakeArm.failed_close='teaching_left';parent.close();self.c.worker.join(3)
        child.refresh();parent.finish_teaching_close();app.processEvents()
        self.assertTrue(parent.isVisible());self.assertTrue(child.isVisible())
        self.assertFalse(parent._teaching_close_pending)
        FakeArm.failed_close=None;parent.close();self.c.worker.join(3)
        child.refresh();parent.finish_teaching_close();app.processEvents()
        self.assertFalse(child.isVisible());self.assertFalse(parent.isVisible())

    def test_log_export_button_and_clipboard_after_session(self):
        app=QApplication.instance() or QApplication([])
        self.c.start();self.wait(lambda:self.c.snapshot()['packets']>=10);self.c.stop();self.c.worker.join(3)
        w=TeachingWindow(self.c);w.timer.stop();w.refresh();w.copy_summary()
        self.assertIn('A6Y 动力示教诊断',app.clipboard().text())
        path=self.temp.name+'/from_ui.zip'
        with patch('teaching_view.QFileDialog.getSaveFileName',return_value=(path,'')):
            w.export_logs();w.export_worker.join(3)
        w.refresh();self.assertIn('诊断包已保存',w.log_notice.text())
        with zipfile.ZipFile(path) as z:
            self.assertIn('references.csv',z.namelist());self.assertIn('events.jsonl',z.namelist())
            self.assertEqual(json.loads(z.read('result.json'))['phase'],'stopped')
        w.close()


if __name__=='__main__':unittest.main()
