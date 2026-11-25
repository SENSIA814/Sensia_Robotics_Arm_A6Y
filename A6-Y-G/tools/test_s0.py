#!/usr/bin/env python3
"""Regression checks for geometry interpretation and the field-calibration gate."""
from copy import deepcopy
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from derive_a5_tcp import DESCRIPTION, URDF, check_tcp, derive, finger_geometry, read_stl
from verify_s0 import CONFIG, field_issues, load
from calibration import field_summary


class GeometryTest(unittest.TestCase):
    def test_ascii_and_binary_including_solid_binary_header(self):
        expected = np.array([[[0, 0, 0], [1, 0, 0], [0, 2, 0]]], dtype=float)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'test.stl'
            path.write_text('solid test\n facet normal 0 0 1\n outer loop\n vertex 0 0 0\n vertex 1 0 0\n vertex 0 2 0\n endloop\n endfacet\nendsolid test\n')
            np.testing.assert_array_equal(read_stl(path), expected)
            path.write_bytes(b'solid binary'.ljust(80, b' ') + struct.pack('<I', 1)
                             + struct.pack('<12fH', 0, 0, 1, *expected.ravel(), 0))
            np.testing.assert_array_equal(read_stl(path), expected)
            path.write_bytes(path.read_bytes()[:-2])
            with self.assertRaises((ValueError, UnicodeDecodeError)):
                read_stl(path)

    def test_visual_transform_and_opening_invariance(self):
        robot = ET.parse(URDF).getroot()
        original, _, _, _ = finger_geometry(robot, DESCRIPTION, 0)
        # A known common visual translation must shift the result once, in metres.
        for name in ('gripper_finger_link1', 'gripper_finger_link2'):
            robot.find(f"link[@name='{name}']/visual/origin").set('xyz', '0.01 0.02 -0.03')
        for q in (0, -0.015, -0.03):
            shifted, _, _, _ = finger_geometry(robot, DESCRIPTION, q)
            np.testing.assert_allclose(shifted-original, [.01, .02, -.03], atol=1e-12)

    def test_stale_tcp_is_rejected(self):
        robot = ET.parse(URDF).getroot()
        robot.find("joint[@name='tool_tcp_joint']/origin").set('xyz', '0 0 0')
        with self.assertRaisesRegex(ValueError, 'differs'):
            check_tcp(robot, derive())


class CalibrationGateTest(unittest.TestCase):
    def setUp(self):
        self.record = load(CONFIG / 'a5_calibration.yaml')
        self.motors = load(CONFIG / 'a5_motors.yaml')['joints']
        self.names = list(self.motors)
        self.model = pin.buildModelFromUrdf(URDF, True)

    def issues(self, record):
        return field_issues(record, self.motors, self.model, self.names)

    def synthetic_record(self):
        # Generated test observations exercise the checker, NOT physical calibration.
        record = deepcopy(self.record)
        record['base_installation'].update(status='verified', description='synthetic fixture',
            measured_gravity_in_base_m_s2=[0, 0, -9.81], max_gravity_error_m_s2=.01)
        for name, motor in self.motors.items():
            positions = np.linspace(motor['min_position'], motor['max_position'], 4)[1:3]
            samples = [{'motor_position': float((q-motor['offset'])/(motor['sign']*motor.get('position_scale', 1))),
                        'reference_model_position': float(q)} for q in positions]
            record['joint_mapping'][name].update(status='verified', motor_id_verified=True,
                direction_verified=True, max_error=1e-6, samples=samples)
        record['gripper_gap'].update(status='verified', closed_gap_m=.003, max_error_m=1e-6,
            samples=[{'motor_position_rad': q/.0239, 'measured_gap_m': .003-2*q}
                     for q in (0, -.015, -.03)])
        samples = []
        for fraction in (.3, .5, .7):
            q = np.array([m['min_position'] + fraction*(m['max_position']-m['min_position'])
                          for m in self.motors.values()])
            data = self.model.createData(); pin.framesForwardKinematics(self.model, data, q)
            tcp = data.oMf[self.model.getFrameId('tool_tcp')]
            samples.append({'q': q.tolist(), 'measured_tcp_xyz_m': tcp.translation.tolist(),
                            'measured_tcp_rpy_rad': pin.rpy.matrixToRpy(tcp.rotation).tolist()})
        record['fk_spot_checks'].update(status='verified', max_position_error_m=1e-6,
            max_orientation_error_rad=1e-6, samples=samples)
        return record

    def test_status_flags_cannot_replace_observations(self):
        record = deepcopy(self.record)
        record['base_installation']['status'] = 'verified'
        for entry in record['joint_mapping'].values():
            entry.update(status='verified', motor_id_verified=True, direction_verified=True)
        record['gripper_gap']['status'] = record['fk_spot_checks']['status'] = 'verified'
        self.assertEqual(len(self.issues(record)), 10)

    def test_user_decisions_resolve_only_base_and_joint_mapping(self):
        issues = self.issues(self.record)
        self.assertEqual(len(issues), 2)
        self.assertTrue(any(issue.startswith('gripper_gap:') for issue in issues))
        self.assertTrue(any(issue.startswith('fk_spot_checks:') for issue in issues))
        summary = field_summary(self.record)
        self.assertEqual(len(summary['operator_confirmed']), 8)
        self.assertEqual(len(summary['skipped_by_user']), 7)
        self.assertIsNone(self.record['base_installation']['measured_gravity_in_base_m_s2'])
        for record in self.record['joint_mapping'].values():
            self.assertEqual(record['samples'], [])
            self.assertIsNone(record['max_error'])

    def test_user_status_requires_recorded_confirmation(self):
        for bad in (None, {}, {'date': 'invalid', 'source': 'user_message', 'statement': 'test'},
                    {'date': '2026-10-08', 'source': 'assumed', 'statement': 'test'},
                    {'date': '2026-10-08', 'source': 'user_message', 'statement': ' '}):
            with self.subTest(bad=bad):
                record = deepcopy(self.record); record['operator_confirmation'] = bad
                self.assertEqual(len(self.issues(record)), 10)

    def test_operator_orientation_must_agree_with_model_gravity(self):
        for direction in ([0, 0, 1], [0, 1, 0], [0, 0, 0], [0, 0, float('nan')], None):
            with self.subTest(direction=direction):
                record = deepcopy(self.record)
                record['base_installation']['gravity_direction_in_base'] = direction
                self.assertTrue(any(issue.startswith('base_installation:') for issue in self.issues(record)))

    def test_mapping_confirmation_does_not_implicitly_skip_zero_scale(self):
        for key, value in (('motor_id_verified', False), ('direction_verified', False),
                           ('zero_scale_test', None), ('zero_scale_test', 'verified')):
            with self.subTest(key=key):
                record = deepcopy(self.record); record['joint_mapping']['joint3'][key] = value
                self.assertTrue(any(issue.startswith('joint3:') for issue in self.issues(record)))

    def test_consistent_measurements_pass_and_wrong_offset_is_rejected(self):
        record = self.synthetic_record()
        self.assertEqual(self.issues(record), [])
        record['joint_mapping']['joint3']['samples'][0]['reference_model_position'] += .1
        self.assertTrue(any('joint3:' in issue for issue in self.issues(record)))

    def test_nonfinite_gap_and_wrong_tcp_are_rejected(self):
        record = self.synthetic_record()
        record['gripper_gap']['samples'][1]['measured_gap_m'] = float('nan')
        record['fk_spot_checks']['samples'][0]['measured_tcp_xyz_m'][0] += .01
        issues = self.issues(record)
        self.assertTrue(any('gripper_gap:' in issue for issue in issues))
        self.assertTrue(any('fk_spot_checks:' in issue for issue in issues))


if __name__ == '__main__':
    unittest.main()
