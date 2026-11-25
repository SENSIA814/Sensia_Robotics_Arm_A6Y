"""Native window lifecycle checks using Qt offscreen; no physical devices."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import can

from PyQt5.QtWidgets import QApplication
from PyQt5.QtTest import QTest

import desktop
import scanner
import session as session_module


class DesktopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='a6y-desktop-test-')
        roles = scanner.load_roles()
        for arm in roles.values():
            arm['port'] = '/nonexistent-desktop-test/' + arm['usb_serial']
        self.session = session_module.ScanSession(roles, Path(self.temp.name))
        self.window = desktop.ScanWindow(self.session)
        self.window.show()
        self.app.processEvents()

    def tearDown(self):
        self.session.cancel.set()
        if self.session.worker:
            self.session.worker.join(timeout=3)
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.temp.cleanup()

    def wait_for(self, predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.app.processEvents()
            if predicate():
                return
            QTest.qWait(20)
        self.fail('Qt action did not finish')

    def test_open_requires_explicit_selection_and_does_not_open_hardware(self):
        self.assertEqual(self.window.selected_roles(), [])
        self.assertFalse(self.window.start_button.isEnabled())
        self.assertIsNone(self.session.worker)
        with patch.object(scanner.StatusOnlyControl, 'open', side_effect=AssertionError('hardware opened')):
            self.window.refresh_button.click()
            self.window.select_all.click()
            self.assertEqual(len(self.window.selected_roles()), 4)
            self.assertTrue(self.window.start_button.isEnabled())
            self.window.select_all.click()
            self.assertEqual(self.window.selected_roles(), [])
            self.assertIsNone(self.session.worker)

    def test_two_selected_arms_get_fourteen_rows_and_selection_resets(self):
        for role in ('motion_left', 'teaching_right'):
            self.window.checkboxes[role].setChecked(True)
        self.window.start_button.click()
        self.wait_for(lambda: self.window._report is not None)
        self.assertEqual(self.window.table.rowCount(), 14)
        self.assertEqual(self.window._report['selected_roles'], ['motion_left', 'teaching_right'])
        self.assertEqual(self.window.selected_roles(), [])
        self.assertFalse(self.window.start_button.isEnabled())
        self.assertTrue(self.window.save_button.isEnabled())
        self.assertTrue(self.window.folder_button.isEnabled())
        self.assertIn('未检测到 USB', self.window.error_label.text())
        self.assertEqual(len(list(Path(self.temp.name).glob('*.json'))), 1)
        self.assertTrue(all(not a['tx'] for a in self.window._report['arms']))
        self.assertEqual(self.window.table.item(0, 3).text(), '未扫描')
        self.assertEqual(self.window.table.item(0, 4).text(), '—')

    def test_position_columns_show_latest_sample_and_missing_range(self):
        state = self.session.snapshot()
        state.update(selected=['motion_left'], passes=3)
        motor = scanner.empty_motor(7)
        scale = scanner.load_position_scales()['304C33733036'][7]
        frame = can.Message(arbitration_id=23, is_extended_id=False, data=bytes.fromhex('07ffff000000191a'))
        motor.update(last=scanner.decode_status(frame, 7, scale), position_scale=scale,
                     attempts=3, hits=3, result='ok')
        state['motors']['motion_left:7'] = motor
        self.window._render(state)
        self.assertEqual(self.window.table.item(6, 4).text(), '179.91')
        self.assertEqual(self.window.table.item(6, 5).text(), '3.1400')
        tooltip = self.window.table.item(6, 4).toolTip()
        self.assertIn('65535', tooltip)
        self.assertIn(motor['last']['received_at'], tooltip)
        self.assertIn(scale['source'], tooltip)
        motor.update(last=scanner.decode_status(frame, 7), position_scale=None)
        self.window._render(state)
        self.assertEqual(self.window.table.item(6, 4).text(), '未配置量程')
        self.assertEqual(self.window.table.item(6, 5).text(), '—')

    def slow_scan(self, selected, roles, passes, cancel, emit):
        # Hold an in-memory job until the actual Stop/Close UI sets cancellation.
        cancel.wait(3)
        return scanner.scan_selected(selected, roles, passes=passes, cancel=cancel, emit=emit)

    def test_stop_cancels_and_keeps_window_and_partial_report(self):
        with patch.object(session_module, 'scan_selected', side_effect=self.slow_scan):
            self.window.checkboxes['motion_left'].setChecked(True)
            self.window.start_button.click()
            self.assertFalse(self.window.checkboxes['motion_left'].isEnabled())
            with self.assertRaises(RuntimeError):
                self.session.start(['motion_right'], 3)
            self.window.stop_button.click()
            self.wait_for(lambda: self.window._report is not None)
        self.assertTrue(self.window._report['cancelled'])
        self.assertFalse(self.window._report['ok'])
        self.assertTrue(self.window.isVisible())
        self.assertEqual(self.window.selected_roles(), [])

    def test_close_waits_for_cancelled_report_before_exiting(self):
        with patch.object(session_module, 'scan_selected', side_effect=self.slow_scan):
            self.window.checkboxes['teaching_left'].setChecked(True)
            self.window.start_button.click()
            self.window.close()
            self.wait_for(lambda: not self.window.isVisible())
        self.assertFalse(self.session.worker.is_alive())
        self.assertTrue(self.session.snapshot()['report']['cancelled'])
        self.assertEqual(len(list(Path(self.temp.name).glob('*.json'))), 1)

    def test_zero_mode_selects_one_arm_and_switching_mode_does_not_start(self):
        self.window.select_all.click()
        self.window.mode.setCurrentIndex(1)
        self.assertEqual(self.window.selected_roles(), [])
        self.assertFalse(self.window.zero_button.isEnabled())
        self.assertFalse(self.window.select_all.isVisible())
        self.window.checkboxes['motion_left'].setChecked(True)
        self.window.checkboxes['teaching_right'].setChecked(True)
        self.assertEqual(self.window.selected_roles(), ['teaching_right'])
        self.assertIn('示教端右臂', self.window.zero_button.text())
        self.assertTrue(self.window.zero_button.isEnabled())
        self.assertIsNone(self.session.worker)
        self.window.mode.setCurrentIndex(0)
        self.assertEqual(self.window.selected_roles(), [])
        self.assertIsNone(self.session.worker)

    def test_zero_mode_missing_adapter_saves_failure_and_requires_reselection(self):
        self.window.mode.setCurrentIndex(1)
        self.window.checkboxes['motion_left'].setChecked(True)
        self.window.zero_button.click()
        self.wait_for(lambda: self.window._report is not None)
        report = self.window._report
        self.assertEqual(report['operation'], 'zero')
        self.assertFalse(report['ok'])
        self.assertEqual(report['arms'][0]['written_ids'], [])
        self.assertEqual(self.window.table.item(0, 3).text(), '未写入')
        self.assertEqual(len(list(Path(self.temp.name).glob('arm_zero_*.json'))), 1)
        self.assertEqual(self.window.selected_roles(), [])
        self.assertFalse(self.window.zero_button.isEnabled())

    def test_scan_and_zero_share_worker_and_close_saves_zero_report(self):
        original = session_module.calibrate_zero
        def slow_zero(role, roles, cancel, emit):
            cancel.wait(3)
            return original(role, roles, cancel=cancel, emit=emit)
        with patch.object(session_module, 'calibrate_zero', side_effect=slow_zero):
            self.window.mode.setCurrentIndex(1)
            self.window.checkboxes['motion_left'].setChecked(True)
            self.window.zero_button.click()
            self.assertFalse(self.window.mode.isEnabled())
            self.assertFalse(self.window.zero_button.isEnabled())
            with self.assertRaises(RuntimeError):
                self.session.start(['motion_right'], 3)
            with self.assertRaises(RuntimeError):
                self.session.start_zero('teaching_left')
            self.window.close()
            self.wait_for(lambda: not self.window.isVisible())
        self.assertTrue(self.session.snapshot()['report']['cancelled'])
        self.assertEqual(len(list(Path(self.temp.name).glob('arm_zero_*.json'))), 1)

    def test_running_scan_rejects_zero_and_invalid_zero_target(self):
        for bad in (None, ['motion_left'], 'all', 'unknown'):
            with self.assertRaises(ValueError):
                self.session.start_zero(bad)
        with patch.object(session_module, 'scan_selected', side_effect=self.slow_scan):
            self.window.checkboxes['motion_left'].setChecked(True)
            self.window.start_button.click()
            with self.assertRaises(RuntimeError):
                self.session.start_zero('motion_right')
            self.window.stop_button.click()
            self.wait_for(lambda: self.window._report is not None)


if __name__ == '__main__':
    unittest.main()
