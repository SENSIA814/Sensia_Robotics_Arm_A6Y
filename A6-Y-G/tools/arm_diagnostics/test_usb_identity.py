"""Verify USB identity checks with fake metadata, never opening a USB device."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import scanner
import usb_identity as usb


def device(serial, tty=None, mode='serial', path='1-3.1'):
    return dict(serial=serial, mode=mode, usb_path=path, ttys=[tty] if tty else [],
                idVendor='0483' if mode == 'dfu' else '16d0',
                idProduct='df11' if mode == 'dfu' else '117e',
                product='DFU in FS Mode' if mode == 'dfu' else 'CANable2')


class UsbIdentityTests(unittest.TestCase):
    def setUp(self):
        self.arm = dict(role='motion_left', name='运动端左臂', usb_serial='SERIAL_A',
                        port='/dev/serial/by-id/usb-SERIAL_A-if00')

    def inspect(self, records, exists=True):
        with patch.object(Path, 'exists', return_value=exists), patch.object(Path, 'resolve', return_value=Path('/dev/ttyACM1')):
            return usb.classify_connection(self.arm, records)

    def test_correct_serial_with_changed_tty_number_is_accepted(self):
        state = self.inspect([device('SERIAL_A', '/dev/ttyACM1')])
        self.assertTrue(state['present'])
        self.assertEqual(state['device'], '/dev/ttyACM1')

    def test_stale_alias_to_another_serial_is_rejected(self):
        state = self.inspect([device('SERIAL_B', '/dev/ttyACM1')])
        self.assertFalse(state['present'])
        self.assertEqual(state['connection_state'], 'identity_mismatch')
        self.assertIn('SERIAL_B', state['connection_message'])

    def test_dfu_is_detected_even_with_a_stale_alias(self):
        state = self.inspect([device('SERIAL_A', mode='dfu'), device('SERIAL_B', '/dev/ttyACM1')])
        self.assertTrue(state['usb_detected'])
        self.assertFalse(state['present'])
        self.assertEqual(state['connection_state'], 'dfu')
        self.assertIsNone(state['device'])

    def test_duplicate_usb_serial_is_rejected(self):
        state = self.inspect([device('SERIAL_A', '/dev/ttyACM1'), device('SERIAL_A', '/dev/ttyACM2', path='1-3.2')])
        self.assertFalse(state['present'])
        self.assertEqual(state['connection_state'], 'duplicate_serial')

    def test_kernel_visible_but_inaccessible_device_is_not_missing_usb(self):
        state = self.inspect([device('SERIAL_A', '/dev/ttyACM1')], exists=False)
        self.assertTrue(state['usb_detected'])
        self.assertFalse(state['present'])
        self.assertEqual(state['connection_state'], 'unavailable')

    def test_two_role_aliases_to_one_tty_are_marked_conflicting(self):
        second = {**self.arm, 'role': 'motion_right', 'name': '运动端右臂', 'usb_serial': 'SERIAL_B', 'port': '/dev/serial/by-id/usb-SERIAL_B-if00'}
        with patch.object(usb, 'usb_snapshot', return_value=[device('SERIAL_A', '/dev/ttyACM1')]), \
                patch.object(Path, 'exists', return_value=True), patch.object(Path, 'resolve', return_value=Path('/dev/ttyACM1')):
            states = usb.connection_inventory({'motion_left': self.arm, 'motion_right': second})
        self.assertTrue(all(not s['present'] and s['connection_state'] == 'alias_conflict' for s in states))

    def test_identity_failure_prevents_driver_open(self):
        with patch.object(scanner, 'verify_usb_identity', side_effect=RuntimeError('identity mismatch')), \
                patch.object(scanner.StatusOnlyControl, 'open', side_effect=AssertionError('serial opened')) as opened:
            report = scanner.scan_selected(['motion_left'], {'motion_left': self.arm}, passes=1)
        opened.assert_not_called()
        self.assertEqual(report['arms'][0]['tx'], [])
        self.assertIn('identity mismatch', report['arms'][0]['error'])

    def test_sysfs_metadata_resolves_tty_parent_and_dfu(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            bus = root / 'usb'
            tty = root / 'tty'
            bus.mkdir()
            tty.mkdir()
            for name, serial, vid, pid in [('1-1', 'SERIAL_A', '16d0', '117e'), ('1-2', 'SERIAL_B', '0483', 'df11')]:
                path = bus / name
                path.mkdir()
                for key, value in [('serial', serial), ('idVendor', vid), ('idProduct', pid)]:
                    (path / key).write_text(value)
            target = bus / '1-1/1-1:1.0/tty/ttyACM7'
            target.mkdir(parents=True)
            (tty / 'ttyACM7').symlink_to(target)
            result = usb.usb_snapshot({'SERIAL_A', 'SERIAL_B'}, bus, tty)
            self.assertEqual(result[0]['ttys'], ['/dev/ttyACM7'])
            self.assertEqual(result[0]['mode'], 'serial')
            self.assertEqual(result[1]['mode'], 'dfu')

    def nested_usb_fixture(self, root):
        bus, tty = root / 'bus', root / 'tty'
        bus.mkdir()
        tty.mkdir()
        nodes = [('usb1', 'usb1', '0000:00:14.0', '1d6b', '0002'),
                 ('1-3', 'usb1/1-3', 'HUB_SERIAL', '05e3', '0610'),
                 ('1-3.1', 'usb1/1-3/1-3.1', 'SERIAL_A', '16d0', '117e'),
                 ('1-3.2', 'usb1/1-3/1-3.2', 'SERIAL_B', '16d0', '117e')]
        for name, relative, serial, vid, pid in nodes:
            path = root / 'devices' / relative
            path.mkdir(parents=True, exist_ok=True)
            for field, value in [('serial', serial), ('idVendor', vid), ('idProduct', pid)]:
                (path / field).write_text(value)
            (bus / name).symlink_to(path)
        for name, number in [('1-3.1', 1), ('1-3.2', 2)]:
            target = (bus / name).resolve() / f'{name}:1.0/tty/ttyACM{number}'
            target.mkdir(parents=True)
            (tty / f'ttyACM{number}').symlink_to(target)
        return bus, tty

    def test_upstream_and_root_hub_serials_do_not_own_adapter_ttys(self):
        with tempfile.TemporaryDirectory() as temp:
            bus, tty = self.nested_usb_fixture(Path(temp))
            records = usb.usb_snapshot({'0000:00:14.0', 'HUB_SERIAL', 'SERIAL_A', 'SERIAL_B'}, bus, tty)
            by_serial = {d['serial']: d for d in records}
            self.assertEqual(by_serial['0000:00:14.0']['ttys'], [])
            self.assertEqual(by_serial['HUB_SERIAL']['ttys'], [])
            self.assertEqual(by_serial['SERIAL_A']['ttys'], ['/dev/ttyACM1'])
            self.assertEqual(by_serial['SERIAL_B']['ttys'], ['/dev/ttyACM2'])
            # Reproduce scan-time verification with ALL USB serials included.
            self.assertTrue(self.inspect(records)['present'])
            with patch.object(Path, 'resolve', return_value=Path('/dev/ttyACM1')), \
                    patch.object(Path, 'exists', return_value=True), \
                    patch.object(usb, 'usb_snapshot', return_value=records):
                self.assertTrue(usb.verify_usb_identity(self.arm)['present'])

    def test_filtered_or_unidentified_child_never_falls_back_to_hub(self):
        with tempfile.TemporaryDirectory() as temp:
            bus, tty = self.nested_usb_fixture(Path(temp))
            for remove_serial in (False, True):
                if remove_serial:
                    (bus / '1-3.1/serial').unlink()
                records = usb.usb_snapshot({'0000:00:14.0', 'HUB_SERIAL'}, bus, tty)
                self.assertTrue(all(not d['ttys'] for d in records))
                state = self.inspect(records)
                self.assertFalse(state['present'])
                self.assertEqual(state['connection_state'], 'identity_mismatch')


if __name__ == '__main__':
    unittest.main()
