"""No physical hardware: fake frames, PTY CANable2, and offscreen Qt."""
from contextlib import contextmanager
from copy import deepcopy
import errno
import json
import os
from pathlib import Path
import pty
import select
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import can
import yaml

from model import JOINTS, MAPPING, ROLE_ORDER, convert_position, load_model, load_roles
from feedback import complete_positions, read_cycle
from scanner import StatusOnlyControl, status_payload
from viewer_session import QueryAudit, ViewSession
from operation_lock import exclusive_operation
from ros_display import rviz_config


class FakeDisplay:
    calls = []

    def __init__(self, *args):
        self.calls = []
        FakeDisplay.calls.append(self)
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.closed = True

    def alive(self):
        return True

    def publish(self, positions, status, warning=False):
        self.calls.append((positions, status, warning))


class ReplyBus:
    def __init__(self, missing=None, fault=None):
        self.pending = None
        self.sent = []
        self.missing, self.fault = missing, fault

    def send(self, msg, timeout):
        self.sent.append(msg)
        i = msg.data[0]
        if i != self.missing:
            state = 8 if i == self.fault else 0
            self.pending = can.Message(arbitration_id=i + 16, is_extended_id=False,
                                       data=bytes([state << 4 | i, 0x80, 0, 0, 0, 0, 24, 25]))

    def recv(self, timeout):
        msg, self.pending = self.pending, None
        return msg


@contextmanager
def simulated_adapter(role='motion_left'):
    """Real python-can SLCAN path, with explicit PTY replacement of the USB path."""
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    os.close(slave)
    stop = threading.Event()
    commands = []
    roles = load_roles()
    roles[role]['port'] = device
    def emulate():
        buffer = b''
        while not stop.is_set():
            if not select.select([master], [], [], .02)[0]:
                continue
            try:
                buffer += os.read(master, 4096)
            except OSError as exc:
                if exc.errno == errno.EIO:
                    stop.wait(.01)
                    continue
                return
            while b'\r' in buffer:
                command, buffer = buffer.split(b'\r', 1)
                if not command:
                    continue
                commands.append(command)
                if command.startswith(b't7FF8') and len(command) == 21:
                    payload = bytes.fromhex(command[5:].decode())
                    i = payload[0]
                    if i in range(1, 8) and payload == status_payload(i):
                        feedback = bytes([i, 0x80, i, 0, 0, 0, 24, 25])
                        os.write(master, f't{i+16:03X}8{feedback.hex()}\r'.encode())
    thread = threading.Thread(target=emulate, daemon=True)
    thread.start()
    try:
        yield roles, commands
    finally:
        stop.set()
        thread.join(1)
        os.close(master)


def wait_for(predicate, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('condition did not become true')


class ModelTests(unittest.TestCase):
    def test_all_roles_and_measured_pmax_and_meshes(self):
        for role in ROLE_ORDER:
            model = load_model(role)
            self.assertFalse(model['mapping_verified'])
            self.assertEqual([model['joints'][n]['motor_id'] for n in JOINTS], list(range(1, 8)))
            scale = model['joints'][JOINTS[-1]]['position_scale_evidence']['pmax_rad']
            self.assertAlmostEqual(scale, 3.14 if role == 'motion_left' else 12.566, places=4)
            tree = ET.fromstring(model['description'])
            self.assertTrue(all(m.attrib['filename'].startswith('file:///') for m in tree.iter('mesh')))
            self.assertEqual(tree.find("joint[@name='gripper_finger_joint2']/mimic").attrib['multiplier'], '-1')

    def test_mapping_math_never_zeroes_or_clamps(self):
        model = load_model('motion_left')
        self.assertEqual(convert_position(4., model['joints']['joint1']), -4.)
        self.assertEqual(convert_position(-.8, model['joints']['gripper_finger_joint1']), -.8 * .0239)
        self.assertEqual(convert_position(.5, {'sign': -1., 'position_scale': 2., 'offset': .1}), -.9)

    def test_invalid_binding_id_and_nan_refused_before_device_open(self):
        original = yaml.safe_load(MAPPING.read_text())
        for change in ('serial', 'duplicate_id', 'nan', 'negative_scale'):
            data = deepcopy(original)
            if change == 'serial':
                data['roles']['motion_left']['usb_serial'] = 'wrong'
            elif change == 'duplicate_id':
                data['defaults']['joint2']['motor_id'] = 1
            elif change == 'nan':
                data['defaults']['joint1']['offset'] = float('nan')
            else:
                data['defaults']['joint1']['position_scale'] = -1.
            with tempfile.TemporaryDirectory() as temp:
                path = Path(temp) / 'mapping.yaml'
                path.write_text(yaml.safe_dump(data))
                with self.assertRaises(ValueError):
                    load_model('motion_left', mapping_path=path)

    def test_role_override_does_not_modify_other_arms(self):
        data = yaml.safe_load(MAPPING.read_text())
        data['roles']['motion_left']['overrides'] = {'joint1': {'sign': 1, 'offset': .2}}
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'mapping.yaml'
            path.write_text(yaml.safe_dump(data))
            left = load_model('motion_left', mapping_path=path)
            right = load_model('motion_right', mapping_path=path)
        self.assertEqual(left['joints']['joint1']['offset'], .2)
        self.assertEqual(right['joints']['joint1']['offset'], 0.)

    def test_rviz_scoped_description_and_markers(self):
        config = rviz_config('/test/role/session')
        displays = config['Visualization Manager']['Displays']
        self.assertEqual(displays[1]['Description Topic']['Value'], '/test/role/session/robot_description')
        self.assertEqual(displays[-1]['Topic']['Value'], '/test/role/session/markers')


class FeedbackTests(unittest.TestCase):
    def test_live_positions_missing_not_replaced_by_zero_or_old_value(self):
        model = load_model('motion_left')
        bus = ReplyBus()
        control = StatusOnlyControl(bus=bus)
        rows = read_cycle(control, model, threading.Event(), .002)
        self.assertIsNotNone(complete_positions(rows))
        bus.missing = 3
        rows = read_cycle(control, model, threading.Event(), .002)
        self.assertIsNone(rows['joint3']['q'])
        self.assertIsNone(complete_positions(rows))
        self.assertEqual(len(bus.sent), 14)

    def test_stale_or_cancelled_cycle_not_published(self):
        model = load_model('motion_left')
        control = StatusOnlyControl(bus=ReplyBus())
        rows = read_cycle(control, model, threading.Event(), .002)
        self.assertIsNone(complete_positions(rows, now=time.monotonic() + 1))
        cancel = threading.Event()
        cancel.set()
        self.assertEqual(read_cycle(control, model, cancel), {})

    def test_fault_positions_remain_visible_and_state_preserved(self):
        rows = read_cycle(StatusOnlyControl(bus=ReplyBus(fault=4)), load_model('motion_left'), threading.Event(), .002)
        self.assertIsNotNone(complete_positions(rows))
        self.assertEqual(rows['joint4']['feedback']['state'], 8)

    def test_control_commands_and_zeroing_rejected_and_audit_bounded(self):
        bus = ReplyBus()
        audit = QueryAudit()
        control = StatusOnlyControl(bus=bus, tx=audit)
        for data in [b'\xff' * 7 + bytes([i]) for i in (0xfc, 0xfd, 0xfe)]:
            with self.assertRaises(RuntimeError):
                control.send_std(1, data)
        with self.assertRaises(RuntimeError):
            control.send_std(0x7ff, bytes.fromhex('0100550800000000'))
        self.assertEqual(bus.sent, [])
        for _ in range(100):
            control.send_std(0x7ff, status_payload(1))
        self.assertEqual(audit.count, 100)
        self.assertEqual(len(audit), 70)


class SessionTests(unittest.TestCase):
    def test_preview_never_opens_hardware_and_stops(self):
        with tempfile.TemporaryDirectory() as temp, patch('viewer_session.open_arm_control', side_effect=AssertionError('opened device')):
            session = ViewSession(report_dir=temp, display_factory=FakeDisplay)
            session.start('teaching_right', preview=True)
            try:
                wait_for(lambda: session.snapshot().get('publish_count', 0) > 0)
                with self.assertRaises(RuntimeError):
                    session.start('motion_left')
            finally:
                session.stop()
                session.worker.join(3)
            state = session.snapshot()
            self.assertFalse(state['running'])
            self.assertIsNone(state['error'])
            self.assertEqual(state['tx_count'], 0)
            self.assertFalse(json.loads(Path(state['report']).read_text())['calibration_verified'])

    def test_pty_end_to_end_only_queries_all_ids_and_releases_lock(self):
        with tempfile.TemporaryDirectory() as temp, simulated_adapter() as (roles, commands):
            session = ViewSession(roles, temp, FakeDisplay)
            session.start('motion_left')
            try:
                wait_for(lambda: session.snapshot().get('publish_count', 0) >= 2)
                with self.assertRaises(RuntimeError):
                    with exclusive_operation():
                        pass
            finally:
                session.stop()
                session.worker.join(5)
            self.assertFalse(session.worker.is_alive())
            state = session.snapshot()
            self.assertIsNone(state['error'])
            frames = [cmd for cmd in commands if cmd.startswith(b't')]
            self.assertGreaterEqual(len(frames), 14)
            self.assertTrue(all(cmd.startswith(b't7FF8') and bytes.fromhex(cmd[5:].decode()) == status_payload(bytes.fromhex(cmd[5:].decode())[0]) for cmd in frames))
            self.assertEqual({bytes.fromhex(cmd[5:].decode())[0] for cmd in frames}, set(range(1, 8)))
            self.assertEqual(commands.count(b'S8'), 1)
            self.assertEqual(commands[-1], b'C')
            self.assertTrue(FakeDisplay.calls[-1].closed)
            with exclusive_operation():
                pass

    def test_busy_calibration_blocks_visualization_before_open(self):
        with tempfile.TemporaryDirectory() as temp, exclusive_operation(), patch('viewer_session.open_arm_control', side_effect=AssertionError('opened device')):
            session = ViewSession(report_dir=temp, display_factory=FakeDisplay)
            session.start('motion_right')
            session.worker.join(3)
            self.assertIn('另一个窗口', session.snapshot()['error'])
            self.assertEqual(session.snapshot()['tx_count'], 0)

    def test_disconnect_reports_error_and_closes_display(self):
        with tempfile.TemporaryDirectory() as temp, simulated_adapter() as (roles, _), patch('viewer_session.read_cycle', side_effect=OSError('USB disconnected')):
            session = ViewSession(roles, temp, FakeDisplay)
            session.start('motion_left')
            session.worker.join(6)
            self.assertIn('USB disconnected', session.snapshot()['error'])
            self.assertTrue(FakeDisplay.calls[-1].closed)


os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtWidgets import QApplication
from PyQt5.QtTest import QTest
from arm_view import ViewWindow


class WindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)

    def test_selection_preview_stop_and_close(self):
        with tempfile.TemporaryDirectory() as temp, patch('viewer_session.open_arm_control', side_effect=AssertionError('opened device')):
            session = ViewSession(report_dir=temp, display_factory=FakeDisplay)
            window = ViewWindow(session)
            try:
                window.show()
                self.assertIsNone(window.selected_role())
                self.assertFalse(window.start_button.isEnabled())
                self.assertIsNone(session.worker)
                window.buttons['teaching_left'].click()
                self.assertEqual(window.selected_role(), 'teaching_left')
                window.buttons['motion_right'].click()
                self.assertEqual(window.selected_role(), 'motion_right')
                window.mode.setCurrentIndex(1)
                window.start_button.click()
                self.assertFalse(window.buttons['motion_left'].isEnabled())
                wait_for(lambda: session.snapshot().get('publish_count', 0) > 0)
                window.tick()
                self.assertEqual(window.table.item(0, 1).text(), '—')
                self.assertIn('预览', window.table.item(0, 6).text())
                window.stop_button.click()
                session.worker.join(3)
                window.tick()
                self.assertIsNone(window.selected_role())
                self.assertFalse(window.start_button.isEnabled())
                window.buttons['motion_left'].click()
                window.start_button.click()
                wait_for(lambda: session.snapshot().get('publish_count', 0) > 0)
                window.close()
                session.worker.join(3)
                window.tick()
                self.assertFalse(window.isVisible())
            finally:
                session.stop()
                if session.worker:
                    session.worker.join(3)
                window.close()
                window.deleteLater()
                self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
