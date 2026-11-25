"""Read-only telemetry conversion, persistence, loss handling, and Qt history."""
import csv
import json
import math
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import pinocchio as pin

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtWidgets import QApplication
from motion_model import JOINTS, MOCK_Q
from native_model import native_model
from native_stack import NativeFeedback
from native_session import PlanningSession
from telemetry import FeedbackSampler, SessionRecorder
from monitor_view import LogViewer, MonitorView


class TelemetryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = native_model('motion_left')

    def hardware(self):
        return dict(q=list(MOCK_Q), velocity=[.1, -.2, .3, -.4, .5, -.6, .002],
                    sample_time=10., diagnostic_time=10., mock=False,
                    rows={name: {'has_feedback': 1., 'feedback_age': .01, 'fault': 0.} for name in JOINTS})

    def test_fk_degrees_motor_mapping_and_missing_velocity(self):
        h = self.hardware()
        h.update(gripper_target_percent=75., gripper_target_mm=-26.25)
        h['velocity'][1] = None
        sampler = FeedbackSampler(self.model)
        sample = sampler.sample(h, 10.1)
        self.assertEqual(sample['valid'], [True]*7)
        self.assertAlmostEqual(sample['joint_deg'][1], math.degrees(MOCK_Q[1]))
        self.assertIsNone(sample['joint_deg_s'][1])
        self.assertAlmostEqual(sample['joint_deg_s'][2], math.degrees(.3))
        e = self.model['joints'][JOINTS[6]]
        self.assertAlmostEqual(sample['motor_deg'][e['motor_id']-1], math.degrees((MOCK_Q[6]-e['offset'])*e['sign']/e['position_scale']))
        self.assertAlmostEqual(sample['motor_deg_s'][e['motor_id']-1], math.degrees(.002*e['sign']/e['position_scale']))
        self.assertEqual(sample['gripper_mm'], [-10.])
        self.assertEqual(sample['gripper_mm_s'], [2.])
        self.assertAlmostEqual(sample['gripper_feedback_percent'], 100*.0093/.0336)
        self.assertAlmostEqual(sample['gripper_model_percent'], 100*.01/.035)
        self.assertEqual(sample['gripper_open_percent'], [sample['gripper_feedback_percent'], 75.])
        self.assertEqual(sample['gripper_motion'], 'closing')
        expected = sampler.kin.fk(MOCK_Q)
        self.assertEqual(sample['tcp_xyz_mm'], (expected[:3, 3]*1000).tolist())

    def test_home_reference_zero_and_relative_translation_rotation_preserve_absolute_logs(self):
        sampler = FeedbackSampler(self.model)
        h = self.hardware()
        h['q'] = [0.]*6 + [-.01]
        h['plot_relative'] = True
        missing = sampler.sample(h, 10.1)
        self.assertEqual(missing['tcp_xyz_mm'], [None]*3)
        self.assertTrue(all(v is not None for v in missing['tcp_absolute_xyz_mm']))
        reference = sampler.kin.fk(h['q'])
        h.update(plot_zero_pose=reference.tolist(), plot_zero_at=123.)
        zero = sampler.sample(h, 10.1)
        np.testing.assert_allclose(zero['tcp_xyz_mm']+zero['tcp_rpy_deg'], [0.]*6, atol=1e-10)
        h['q'] = [.2, .1, .2, .3, -.1, .1, -.01]
        moved = sampler.sample(h, 10.1)
        pose = sampler.kin.fk(h['q'])
        np.testing.assert_allclose(moved['tcp_xyz_mm'], (pose[:3, 3]-reference[:3, 3])*1000)
        np.testing.assert_allclose(moved['tcp_rpy_deg'], np.rad2deg(pin.rpy.matrixToRpy(reference[:3, :3].T@pose[:3, :3])))
        np.testing.assert_allclose(moved['tcp_absolute_xyz_mm'], pose[:3, 3]*1000)
        self.assertGreater(np.linalg.norm(moved['tcp_xyz_mm']), 1.)
        self.assertGreater(np.linalg.norm(moved['tcp_rpy_deg']), 1.)
        self.assertEqual(moved['raw_position'], h['q'])

    def test_stale_diagnostics_or_joint_data_create_gaps_keep_raw(self):
        sampler = FeedbackSampler(self.model)
        h = self.hardware()
        h['diagnostic_time'] = 9.
        sample = sampler.sample(h, 10.1)
        self.assertEqual(sample['valid'], [False]*7)
        self.assertEqual(sample['tcp_xyz_mm'], [None]*3)
        self.assertEqual(sample['motor_deg_s'], [None]*7)
        self.assertIsNone(sample['gripper_feedback_percent'])
        self.assertEqual(sample['gripper_motion'], 'unknown')
        self.assertEqual(sample['raw_position'], MOCK_Q)
        h['mock'] = True
        self.assertTrue(all(sampler.sample(h, 10.1)['valid']))
        self.assertFalse(any(sampler.sample(h, 10.7)['valid']))
        h = self.hardware()
        h['rows'][JOINTS[6]]['fault'] = 1.
        self.assertIsNone(sampler.sample(h, 10.1)['gripper_mm'][0])
        self.assertTrue(all(v is not None for v in sampler.sample(h, 10.1)['tcp_xyz_mm']))

    def test_joint_state_names_are_ordered_and_empty_velocity_not_zero(self):
        runtime = NativeFeedback(self.model, True)
        names = list(reversed(JOINTS))
        msg = SimpleNamespace(name=names, position=list(reversed(MOCK_Q)), velocity=[],
                              header=SimpleNamespace(stamp=SimpleNamespace(sec=4, nanosec=500000000)))
        runtime.joint_states(msg)
        self.assertEqual(runtime.snapshot()['q'], MOCK_Q)
        self.assertEqual(runtime.snapshot()['velocity'], [None]*7)
        self.assertEqual(runtime.snapshot()['ros_stamp'], 4.5)
        msg.velocity = list(range(7))
        runtime.joint_states(msg)
        self.assertEqual(runtime.snapshot()['velocity'], list(reversed(range(7))))

    def test_csv_records_without_gui_and_flushes_events_on_close(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = NativeFeedback(self.model, True)
            runtime.update(q=MOCK_Q, velocity=[0.]*7, sample_time=time.monotonic())
            runtime.update(gripper_target_percent=100., gripper_target_mm=-34.3)
            recorder = SessionRecorder(directory, self.model, runtime)
            recorder.event('plan_result', '碰撞 link1 ↔ link4', moveit_error_code=-2)
            recorder.event('plan_saved', 'saved', trajectory_file='plan_test.json', trajectory={'points': [1, 2]})
            recorder.event('gripper_target', '夹爪张开目标 100%（未执行）', group='gripper', target_percent=100.)
            deadline = time.monotonic()+3
            while len(recorder.snapshot()['samples']) < 3 and time.monotonic() < deadline:
                time.sleep(.02)
            recorder.close()
            self.assertFalse(recorder.thread.is_alive())
            self.assertFalse(recorder.error)
            with Path(directory, 'telemetry.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertGreaterEqual(len(rows), 3)
            self.assertAlmostEqual(float(rows[0]['joint_deg_j2']), math.degrees(1.2))
            self.assertTrue(rows[0]['time_iso'].endswith('+00:00'))
            self.assertEqual(float(rows[0]['gripper_target_percent']), 100.)
            self.assertAlmostEqual(float(rows[0]['gripper_open_percent_feedback']), 100*.0093/.0336)
            self.assertAlmostEqual(float(rows[0]['gripper_model_percent']), 100*.01/.035)
            self.assertIn('夹爪张开目标', Path(directory, 'gripper.log').read_text())
            self.assertNotIn('gripper_execution_complete', Path(directory, 'gripper.log').read_text())
            events = [json.loads(line) for line in Path(directory, 'events.jsonl').read_text().splitlines()]
            self.assertEqual(events[0]['moveit_error_code'], -2)
            self.assertIn('碰撞', Path(directory, 'application.log').read_text())
            self.assertEqual(json.loads(Path(directory, 'plan_test.json').read_text())['points'], [1, 2])

    def test_disk_failure_reported_without_changing_control_state(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = NativeFeedback(self.model, True)
            before = runtime.snapshot()
            with patch('pathlib.Path.write_text', side_effect=OSError('simulated disk full')):
                recorder = SessionRecorder(directory, self.model, runtime)
                recorder.thread.join(3)
                recorder.close()
            self.assertIn('simulated disk full', recorder.error)
            self.assertEqual(before, runtime.snapshot())

    def test_session_collision_and_target_are_in_structured_log(self):
        with tempfile.TemporaryDirectory() as directory:
            s = PlanningSession()
            s.runtime = NativeFeedback(self.model, True)
            s.update(directory=directory)
            s.recorder = SessionRecorder(directory, self.model, s.runtime)
            Path(directory, 'move_group.log').write_text("Found a contact between 'link1' (type 'Robot link') and 'link4' (type 'Robot link')\n")
            target = [[1., 0., 0., .2], [0., 1., 0., 0.], [0., 0., 1., .3], [0., 0., 0., 1.]]
            s.update(target=target)
            s.update(error=s._planning_error(-2))
            s.recorder.close()
            events = [json.loads(line) for line in Path(directory, 'events.jsonl').read_text().splitlines()]
            collision = next(e for e in events if e['kind'] == 'collision')
            self.assertEqual(collision['pairs'], [['link1', 'link4']])
            self.assertEqual(collision['moveit_error_code'], -2)
            self.assertTrue(any(e.get('changes', {}).get('target') == target for e in events))
            self.assertTrue(any('error' in e.get('changes', {}) for e in events))


class MonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)

    def test_tail_filter_partial_utf8_and_file_truncation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory, 'move_group.log')
            path.write_bytes(b'INFO ready\n'+ '碰撞'.encode()[:2])
            view = LogViewer()
            view.set_folder(directory)
            self.assertIn('INFO ready', view.text.toPlainText())
            with path.open('ab') as stream:
                stream.write('碰撞'.encode()[2:] + b' link1 link4\n')
            view.poll()
            self.assertIn('碰撞 link1 link4', view.text.toPlainText())
            view.search.setText('碰撞')
            self.assertNotIn('INFO ready', view.text.toPlainText())
            self.assertIn('碰撞', view.text.toPlainText())
            path.write_text('碰撞 new\n')
            view.poll()
            self.assertIn('碰撞 new', view.text.toPlainText())
            view.deleteLater()

    def test_pause_still_queries_recorder_resume_and_bounded_history(self):
        model = native_model('motion_left')
        sampler = FeedbackSampler(model)
        packet = sampler.sample(dict(q=MOCK_Q, velocity=[0.]*7, sample_time=10., mock=True), 10.)
        packet['gripper_open_percent'][1] = 80.
        packet['sequence'] = 1
        class Session:
            def telemetry_snapshot(self, after=0):
                return dict(error='', dropped_events=0, samples=[packet] if after < 1 else [])
        view = MonitorView()
        state = dict(directory=None, running=True, mock=True, role='motion_left')
        view.pause.setChecked(True)
        view.refresh(Session(), state)
        self.assertEqual(view.sequence, 0)
        view.pause.setChecked(False)
        view.refresh(Session(), state)
        self.assertEqual(view.sequence, 1)
        self.assertAlmostEqual(view.charts['gripper_open_percent'].samples[-1][1][0], 100*.0093/.0336)
        self.assertEqual(view.charts['gripper_open_percent'].samples[-1][1][1], 80.)
        self.assertIn('80.0%', view.gripper_readout.text())
        chart = view.charts['joint_deg']
        for i in range(3000):
            chart.append(i*.05, [0.]*6)
        self.assertEqual(len(chart.samples), 2400)
        state['running'] = False
        view.refresh(Session(), state)
        self.assertIn('已结束', view.health.text())
        view.resize(950, 850)
        view.show()
        view.tabs.setCurrentWidget(view.gripper_page)
        self.app.processEvents()
        self.assertFalse(view.grab().isNull())
        view.close()
        view.deleteLater()


if __name__ == '__main__':
    unittest.main()
