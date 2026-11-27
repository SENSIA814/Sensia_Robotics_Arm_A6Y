"""Position decoding checks, without opening serial ports."""
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import can
import yaml

import scanner
from position_feedback import DEFAULT_SCALES, load_position_scales


class PositionTests(unittest.TestCase):
    def test_checked_in_ranges_match_all_twenty_eight_measured_parameters(self):
        scales = load_position_scales()
        evidence = json.loads((scanner.ROOT.parent / 'diagnostics/all_arms_20261124_1737.json').read_text())
        roles = yaml.safe_load(scanner.DEFAULT_ROLES.read_text())['roles']
        checked_serials = set()
        self.assertEqual(sum(len(motors) for motors in scales.values()), 28)
        for adapter in evidence['adapters']:
            serial = adapter['port'].rsplit('_', 1)[1].removesuffix('-if00')
            # A replaced USB adapter keeps the same physical motors and PMAX.
            # Resolve the original measurement through the recorded binding history.
            current = [entry['usb_serial'] for entry in roles.values()
                       if serial == entry['usb_serial'] or any(
                           serial == item['previous_binding']['usb_serial']
                           for item in entry.get('adapter_history', []))]
            self.assertEqual(len(current), 1, serial)
            serial = current[0]
            self.assertNotIn(serial, checked_serials)
            checked_serials.add(serial)
            for motor in adapter['motors']:
                scale = scales[serial][motor['requested_id']]
                self.assertEqual(scale['pmax_rad'], motor['parameters']['PMAX'])
                self.assertEqual(scale['measured_at'], evidence['time'])
        self.assertEqual(checked_serials, set(scales))

    def test_signed_endpoints_and_gripper_specific_ranges(self):
        scales = load_position_scales()
        for raw, sign in ((0, -1), (65535, 1)):
            msg = can.Message(arbitration_id=23, is_extended_id=False,
                              data=bytes([7, raw >> 8, raw & 255, 0, 0, 0, 25, 26]))
            left = scanner.decode_status(msg, 7, scales['304C33733036'][7])
            right = scanner.decode_status(msg, 7, scales['209B3971534B'][7])
            self.assertAlmostEqual(left['position_rad'], sign * 3.140000104904175)
            self.assertAlmostEqual(right['position_rad'], sign * 12.565999984741211)
            self.assertAlmostEqual(left['position_deg'], sign * 179.90875368181415)
            self.assertEqual(left['position_raw'], raw)
            unknown = scanner.decode_status(msg, 7)
            self.assertIsNone(unknown['position_rad'])
            self.assertIsNone(unknown['position_deg'])
            self.assertEqual(unknown['position_raw'], raw)

    def test_invalid_ranges_are_rejected(self):
        data = yaml.safe_load(DEFAULT_SCALES.read_text())
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'ranges.yaml'
            for bad in (0, -12.5, math.inf, math.nan, True, '12.5'):
                with self.subTest(bad=bad):
                    data['adapters']['304C33733036'][1] = bad
                    path.write_text(yaml.safe_dump(data))
                    with self.assertRaises(ValueError):
                        load_position_scales(path)

    def test_range_failure_is_reported_without_fabricating_angles(self):
        cancelled = threading.Event()
        cancelled.set()
        with patch.object(scanner, 'load_position_scales', side_effect=ValueError('invalid PMAX')), \
                patch.object(scanner.StatusOnlyControl, 'open', side_effect=AssertionError('hardware opened')):
            report = scanner.scan_selected(['motion_left'], scanner.load_roles(), cancel=cancelled)
        self.assertIn('invalid PMAX', report['position_scale_error'])
        self.assertTrue(all(m['position_scale'] is None and m['last'] is None for m in report['arms'][0]['motors']))


if __name__ == '__main__':
    unittest.main()
