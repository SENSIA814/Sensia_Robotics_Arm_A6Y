"""Multi-arm Qt orchestration with in-memory sessions; no ROS or USB commands."""
from copy import deepcopy
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import unittest
from unittest.mock import patch

from PyQt5.QtWidgets import QApplication
from gravity_config import ROLE_ORDER, base_parameters
from gravity_view import GravityWindow


class FakeSession:
    def __init__(self):
        self.worker=None
        self.calls=[]
        self.block=''
        self.state=dict(running=False,ready=False,active=False,busy=False,mock=True,
                        status='未连接',error='',notice='',directory=None,
                        closing_requested=False,close_failed=False,release_confirmed=None)

    def snapshot(self): return deepcopy(self.state)
    def enable_block_reason(self): return self.block

    def start(self,role,mock,with_rviz,parameters):
        self.calls.append(('start',role))
        self.state.update(running=True,ready=True,active=False,mock=mock,role=role,
                          applied_parameters=deepcopy(parameters),status='反馈就绪')

    def enable(self):
        self.calls.append(('enable',))
        self.state.update(active=True,status='补偿运行')

    def tune(self,p):
        self.calls.append(('tune',))
        self.state['applied_parameters']=deepcopy(p)

    def release(self):
        self.calls.append(('release',))
        self.state.update(active=False,release_confirmed=True,closing_requested=False)

    def stop(self):
        self.calls.append(('stop',))
        self.state.update(closing_requested=True,close_failed=False,busy=True)

    def finish(self):
        self.state.update(running=False,ready=False,active=False,busy=False,
                          release_confirmed=True,closing_requested=False)


class MultiArmWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QApplication.instance() or QApplication([])

    def setUp(self):
        self.patches=[patch('gravity_view.inventory',return_value=[]),
                      patch('gravity_view.load_profile',side_effect=lambda *_:base_parameters())]
        for p in self.patches:p.start()
        self.window=GravityWindow(session_factory=FakeSession)
        self.window.timer.stop()
        self.roles=['teaching_left','teaching_right']

    def tearDown(self):
        for session in self.window.sessions.values():session.finish()
        self.window.close();self.app.processEvents()
        for p in reversed(self.patches):p.stop()

    def connect_pair(self):
        for role in self.roles:self.window.batch_checks[role].setChecked(True)
        self.window.batch_start.click()
        self.assertEqual(self.window.role,self.roles[0])
        return [self.window.sessions[role] for role in self.roles]

    def test_batch_feedback_is_separate_from_explicit_enable(self):
        left,right=self.connect_pair()
        self.assertTrue(left.state['ready'] and right.state['ready'])
        self.assertFalse(left.state['active'] or right.state['active'])
        self.window.batch_enable.click()
        self.assertTrue(left.state['active'] and right.state['active'])
        self.assertTrue(all(button.isEnabled() for button in self.window.buttons.values()))
        self.window.buttons[self.roles[1]].click()
        self.assertIs(self.window.session,right)
        self.assertTrue(left.state['active'])
        self.window.release_button.click()
        self.assertFalse(right.state['active'])
        self.assertTrue(left.state['active'])

    def test_drafts_and_live_tuning_are_per_arm(self):
        self.window.select_role(self.roles[0])
        self.window.editors['torque_scales'][1].setValue(1.55)
        self.window.select_role(self.roles[1])
        self.window.editors['torque_scales'][1].setValue(1.25)
        self.window.select_role(self.roles[0])
        self.assertEqual(self.window.parameters()['torque_scales'][1],1.55)
        left,right=self.connect_pair()
        self.assertEqual(left.state['applied_parameters']['torque_scales'][1],1.55)
        self.assertEqual(right.state['applied_parameters']['torque_scales'][1],1.25)
        self.window.editors['torque_scales'][1].setValue(1.65)
        self.window.refresh();self.window.apply.click()
        self.assertEqual(left.state['applied_parameters']['torque_scales'][1],1.65)
        self.assertEqual(right.state['applied_parameters']['torque_scales'][1],1.25)

    def test_unselected_arm_plot_keeps_its_own_samples(self):
        left,right=self.connect_pair()
        for i,session in enumerate((left,right),1):
            session.state.update(sequence=1,gravity=[i]*7,commanded=[i+.1]*7,effort=[i+.2]*7)
        self.window.refresh();self.window.refresh()
        self.assertEqual(len(self.window.plot.samples),1)
        self.assertEqual(self.window.plot.samples[0][1],[1]*7)
        self.window.select_role(self.roles[1])
        self.assertEqual(len(self.window.plot.samples),1)
        self.assertEqual(self.window.plot.samples[0][1],[2]*7)

    def test_batch_precheck_does_not_enable_other_arm_when_one_is_blocked(self):
        left,right=self.connect_pair()
        right.block='电机已在使能状态'
        self.window.batch_enable.click()
        self.assertFalse(left.state['active'] or right.state['active'])
        self.assertIn(right.block,self.window.error.text())

    def test_all_four_arms_run_and_idle_detail_switch_does_not_hide_global_controls(self):
        left,right=self.connect_pair();self.window.batch_enable.click()
        self.window.select_role('motion_left')
        self.assertTrue(self.window.start_button.isEnabled())
        self.assertTrue(self.window.release_all.isEnabled())
        self.assertFalse(self.window.mode.isEnabled())
        for role in ROLE_ORDER:self.window.batch_checks[role].setChecked(True)
        self.window.batch_start.click();self.window.batch_enable.click()
        self.assertEqual(len(self.window.sessions),4)
        self.assertTrue(all(s.state['active'] for s in self.window.sessions.values()))
        self.assertEqual(left.calls.count(('start',self.roles[0])),1)
        self.assertEqual(right.calls.count(('start',self.roles[1])),1)

    def test_window_close_returns_one_arm_at_a_time(self):
        left,right=self.connect_pair();self.window.batch_enable.click()
        self.window.show();self.window.close()
        self.assertEqual(left.calls.count(('stop',)),1)
        self.assertEqual(right.calls.count(('stop',)),0)
        self.assertTrue(right.state['active'])
        self.window.refresh()
        self.assertEqual(right.calls.count(('stop',)),0)
        left.finish();self.window.refresh()
        self.assertEqual(right.calls.count(('stop',)),1)
        right.finish();self.window.refresh()
        self.assertFalse(self.window.isVisible())

    def test_failed_return_keeps_window_and_does_not_start_remaining_arm(self):
        left,right=self.connect_pair();self.window.batch_enable.click()
        self.window.show();self.window.close()
        left.state.update(close_failed=True,closing_requested=False,busy=False,notice='J4 反馈速度限制')
        self.window.refresh()
        self.assertFalse(self.window._closing)
        self.assertTrue(self.window.isVisible())
        self.assertTrue(right.state['active'])
        self.assertEqual(right.calls.count(('stop',)),0)
        self.assertIn('J4',self.window.error.text())
        self.window.close()
        self.assertTrue(self.window._closing)
        self.assertEqual(left.calls.count(('stop',)),2)
        self.assertEqual(right.calls.count(('stop',)),0)

    def test_all_release_targets_every_running_arm_and_cancels_close_queue(self):
        left,right=self.connect_pair();self.window.batch_enable.click()
        self.window.show();self.window.close()
        self.window.release_all.click()
        self.assertFalse(self.window._closing)
        self.assertFalse(left.state['active'] or right.state['active'])
        self.assertEqual(left.calls.count(('release',)),1)
        self.assertEqual(right.calls.count(('release',)),1)
        self.assertEqual(right.calls.count(('stop',)),0)

    def test_new_fault_on_waiting_arm_cancels_remaining_close_queue(self):
        left,right=self.connect_pair();self.window.batch_enable.click()
        self.window.show();self.window.close()
        right.state.update(error='模拟反馈中断',close_failed=True)
        self.window.refresh()
        self.assertFalse(self.window._closing)
        self.assertTrue(self.window.isVisible())
        self.assertEqual(right.calls.count(('stop',)),0)
        self.assertEqual(left.calls.count(('stop',)),1)
        self.assertTrue(left.state['closing_requested'])
        self.assertIn('停止后续回零',self.window.error.text())

    def test_existing_individual_return_finishes_before_window_close_next_arm(self):
        left,right=self.connect_pair();self.window.batch_enable.click()
        self.window.select_role(self.roles[1]);self.window.disconnect_button.click()
        self.window.select_role(self.roles[0]);self.window.refresh()
        self.assertFalse(self.window.disconnect_button.isEnabled())
        self.window.show();self.window.close()
        self.assertEqual(left.calls.count(('stop',)),0)
        right.finish();self.window.refresh()
        self.assertEqual(left.calls.count(('stop',)),1)


if __name__=='__main__':unittest.main()
