"""Configuration/provenance checks for the transplanted stack, no ROS/device I/O."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

from motion_model import JOINTS, ROLE_ORDER
from native_model import CONFIG, UPSTREAM, native_model, control_description, write_controller_config


class NativeConfigTests(unittest.TestCase):
    def test_bare_policy_only_excludes_documented_pairs_for_selected_role(self):
        for role in ROLE_ORDER:
            full = native_model(role, collision_profile='full')
            bare = native_model(role, collision_profile='bare')
            self.assertEqual(full['description'], bare['description'])
            self.assertEqual(full['urdf_sha256'], bare['urdf_sha256'])
            self.assertEqual(full['joints'], bare['joints'])
            def pairs(model):
                return {frozenset((e.get('link1'),e.get('link2'))) for e in ET.fromstring(model['semantic']).findall('disable_collisions')}
            expected = {frozenset(('link1','link4')),frozenset(('link1','link5'))}
            if role == 'motion_right':
                expected.add(frozenset(('link2','link4')))
                self.assertIn('link2↔link4', bare['collision_policy']['description'])
            else:
                self.assertNotIn('link2↔link4', bare['collision_policy']['description'])
            self.assertEqual(pairs(bare)-pairs(full), expected)
            self.assertNotIn(frozenset(('link2', 'link4')), pairs(full))
            self.assertFalse(pairs(full)-pairs(bare))
            for tag in ('group','end_effector','virtual_joint','group_state'):
                self.assertEqual([ET.tostring(e) for e in ET.fromstring(full['semantic']).findall(tag)],
                                 [ET.tostring(e) for e in ET.fromstring(bare['semantic']).findall(tag)])

    def test_unknown_collision_policy_is_rejected(self):
        with self.assertRaisesRegex(ValueError, '未知碰撞配置'):
            native_model('motion_left', collision_profile='all_collisions_off')

    def test_reference_files_preserved_except_documented_hardware_range_adapter(self):
        manifest = json.loads((UPSTREAM.parent / 'upstream_manifest.json').read_text())
        changed = [name for name, digest in manifest['files'].items()
                   if hashlib.sha256((UPSTREAM / name).read_bytes()).hexdigest() != digest]
        self.assertEqual(changed, ['a6_y_ros2_control/src/a6_y_damiao_system.cpp'])

    def test_all_four_calibrations_and_measured_pmax_reach_original_driver(self):
        for role in ROLE_ORDER:
            model = native_model(role)
            system = ET.fromstring(control_description(model, mock=False, enabled=True)).find('ros2_control')
            params = {p.get('name'): p.text for p in system.find('hardware').findall('param')}
            self.assertEqual(params['can_channel'], model['arm']['port'])
            self.assertEqual(params['dry_run'], 'false')
            self.assertEqual(params['auto_enable'], 'true')
            for joint in system.findall('joint'):
                name = joint.get('name')
                values = {p.get('name'): p.text for p in joint.findall('param')}
                expected = model['joints'][name]
                for key in ('sign', 'position_scale', 'offset'):
                    self.assertEqual(float(values[key]), expected[key])
                self.assertEqual(float(values['protocol_pmax']), expected['position_scale_evidence']['pmax_rad'])
                self.assertEqual(int(values['slave_id']), expected['motor_id'])

    def test_readonly_model_cannot_enable_set_modes_write_or_disable_motors(self):
        tree = ET.fromstring(control_description(native_model('motion_left'), mock=False, enabled=False))
        system = tree.find('ros2_control')
        hardware = {p.get('name'): p.text for p in system.find('hardware').findall('param')}
        self.assertEqual(hardware['auto_enable'], 'false')
        self.assertEqual(hardware['set_control_mode_on_configure'], 'false')
        self.assertEqual([j.find("param[@name='read_only']").text for j in system.findall('joint')], ['true'] * 7)

    def test_reference_100_hz_and_both_position_trajectory_controllers(self):
        with tempfile.TemporaryDirectory() as folder:
            path = write_controller_config(native_model('motion_left'), folder, '/test', True, True)
            config = yaml.safe_load(path.read_text())
            original = yaml.safe_load((CONFIG / 'ros2_controllers.yaml').read_text())
            self.assertEqual(config['/test/controller_manager']['ros__parameters']['update_rate'], 100)
            for name in ('arm_controller', 'gripper_controller'):
                self.assertEqual(config['/test/'+name], original[name])
                self.assertEqual(config['/test/'+name]['ros__parameters']['command_interfaces'], ['position'])

    def test_reference_srdf_adapts_valid_link_names_and_current_zero_gripper(self):
        model = native_model('motion_left')
        robot, semantic = ET.fromstring(model['description']), ET.fromstring(model['semantic'])
        links = {e.get('name') for e in robot.findall('link')}
        self.assertEqual(robot.get('name'), semantic.get('name'))
        for element in semantic.iter():
            for name in ('base_link', 'tip_link', 'link1', 'link2', 'parent_link', 'child_link'):
                if name in element.attrib:
                    self.assertIn(element.get(name), links)
        home = semantic.find("group_state[@name='home']")
        self.assertEqual([float(j.get('value')) for j in home.findall('joint')], [0.] * 6)
        self.assertEqual(float(semantic.find("group_state[@name='open']/joint").get('value')), model['gripper_limits']['open_position_m'])
        self.assertEqual(float(semantic.find("group_state[@name='close']/joint").get('value')), model['gripper_limits']['closed_position_m'])

    def test_launcher_uses_native_stack_and_not_old_python_execution(self):
        folder = Path(__file__).parent
        text = (folder / 'native_session.py').read_text()
        self.assertNotIn('from hardware import', text)
        self.assertNotIn('from trajectory import', text)
        self.assertNotIn('.kin.ik(', text)
        self.assertNotIn('bounded_start', text)
        self.assertIn('from native_session import PlanningSession', (folder/'moveit_view.py').read_text())


if __name__ == '__main__':
    unittest.main()
