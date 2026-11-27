"""Offline regressions for measured J7 travel, percentage mapping and margins."""
from copy import deepcopy
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
import yaml

import teaching_gripper as gripper


def calibration():
    return yaml.safe_load(gripper.CALIBRATION.read_text())


def model(role='motion_left', lower=-.035, upper=0., pmax=None):
    row=calibration()['roles'].get(role,calibration()['roles']['motion_left'])
    return {'role':role,'arm':{'usb_serial':row['usb_serial']},'joints':{
        gripper.GRIPPER_JOINT:{'lower':lower,'upper':upper,'type':'prismatic','motor_id':7,
            'position_scale':.0239,'offset':0.,'sign':1.,
            'position_scale_evidence':{'pmax_rad':row['pmax_rad'] if pmax is None else pmax}}}}


class GripperPolicyTests(unittest.TestCase):
    def test_all_controllers_claim_seven_for_haptics_and_measured_exit_hold(self):
        for role in ('teaching_left','teaching_right'):
            self.assertEqual(gripper.controlled_joints(model(role)),gripper.ARM_JOINTS+(gripper.GRIPPER_JOINT,))
        for role in ('motion_left','motion_right'):
            self.assertEqual(gripper.controlled_joints(model(role)),gripper.ARM_JOINTS+(gripper.GRIPPER_JOINT,))
        with self.assertRaises(ValueError):gripper.controlled_joints(model('unknown'))

    def test_all_four_measured_endpoints_have_inward_motor_margin(self):
        for role,row in calibration()['roles'].items():
            p=gripper.gripper_profile(model(role))
            self.assertAlmostEqual(p['open_motor_rad']-row['open_motor_rad'],.0007/.0239)
            self.assertAlmostEqual(row['closed_motor_rad']-p['closed_motor_rad'],.0007/.0239)
            self.assertAlmostEqual(p['endpoint_margin_model_m'],.0007)
            self.assertLess(p['min_position_m'],p['measured_open_position_m'])
            self.assertLess(p['measured_open_position_m'],p['open_position_m'])
            self.assertLess(p['open_position_m'],p['closed_position_m'])
            self.assertLess(p['closed_position_m'],p['measured_closed_position_m'])
            self.assertLess(p['measured_closed_position_m'],p['max_position_m'])
            self.assertTrue(p['endpoint_calibration_verified'])
            self.assertFalse(p['physical_jaw_gap_measured'])
            self.assertFalse(p['measured_endpoint_guard_enabled'])
            self.assertEqual(p['feedback_position_policy'],'protocol_range_only')
            self.assertAlmostEqual(p['hardware_min_position_m'],-row['pmax_rad']*.0239)
            self.assertAlmostEqual(p['hardware_max_position_m'],row['pmax_rad']*.0239)
            self.assertEqual(p['max_velocity_m_s'],.100)
            self.assertEqual(len(p['endpoint_source_sha256']),64)

    def test_old_urdf_and_pmax_are_not_used_as_gripper_travel(self):
        p=gripper.gripper_profile(model())
        self.assertLess(p['open_position_m'],-.035)
        altered=gripper.gripper_profile(model(lower=-.025,upper=.01))
        for key in ('open_position_m','closed_position_m','min_position_m','max_position_m'):
            self.assertEqual(p[key],altered[key])
        with self.assertRaisesRegex(ValueError,'PMAX'):
            gripper.gripper_profile(model(pmax=12.566))

    def test_identity_mapping_and_malformed_calibration_fail_closed(self):
        for key,value in (('position_scale',0.),('sign',-1.),('offset',float('nan')),('lower',1.),('upper',float('inf')),('motor_id',6)):
            bad=deepcopy(model());bad['joints'][gripper.GRIPPER_JOINT][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):gripper.gripper_profile(bad)
        bad=model();bad['arm']['usb_serial']='different-arm'
        with self.assertRaisesRegex(ValueError,'USB'):gripper.gripper_profile(bad)
        for speed in (0.,-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):gripper.gripper_profile(model(),speed)
        for key,value in (('endpoint_margin_motor_rad',0.),('endpoint_margin_motor_rad',2.),
                          ('feedback_tolerance_motor_rad',1.),('feedback_tolerance_motor_rad',float('nan'))):
            data=calibration();data[key]=value
            with tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'limits.yaml';path.write_text(yaml.safe_dump(data))
                with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                    gripper.gripper_profile(model(),calibration_path=path)

    def test_every_default_and_cross_pair_maps_percentages(self):
        for leader in ('teaching_left','teaching_right'):
            for follower in ('motion_left','motion_right'):
                m=gripper.gripper_mapping(model(leader),model(follower));source,target=m['leader'],m['follower']
                for fraction in (0.,.25,.5,.75,1.):
                    q=source['closed_position_m']+fraction*(source['open_position_m']-source['closed_position_m'])
                    expected=target['closed_position_m']+fraction*(target['open_position_m']-target['closed_position_m'])
                    self.assertAlmostEqual(q*m['scale']+m['offset'],expected)
                    self.assertAlmostEqual(gripper.opening_percent(source,q),100*fraction)
                self.assertEqual(gripper.opening_percent(source,source['measured_open_position_m']),100.)
                self.assertEqual(gripper.opening_percent(source,source['measured_closed_position_m']),0.)
                self.assertEqual(gripper.opening_percent(source,source['measured_open_position_m']-.001),100.)
                self.assertEqual(gripper.opening_percent(source,source['measured_closed_position_m']+.001),0.)
                self.assertNotEqual(m['scale'],1.)
        with self.assertRaises(ValueError):gripper.opening_percent(source,float('nan'))
        with self.assertRaises(ValueError):gripper.gripper_mapping(model('motion_left'),model('teaching_right'))

    def test_native_driver_urdf_mimic_and_mapper_have_correct_distinct_bounds(self):
        from gravity_config import load_model
        from teaching_port_config import write_port_control,port_parameters,write_mapper_config
        from a6_y_teleop.leader_follower_teleop import load_config
        with tempfile.TemporaryDirectory() as folder:
            for role in calibration()['roles']:
                m=load_model(role);p=gripper.gripper_profile(m)
                path=write_port_control(m,port_parameters(role),folder,'/test','cm',False,True)
                data=yaml.safe_load(path.read_text())
                robot=ET.fromstring(data['/test/cm']['ros__parameters']['robot_description'])
                period=robot.find("ros2_control/hardware/param[@name='status_request_period']")
                self.assertEqual(float(period.text),.01 if role.startswith('teaching_') else .05)
                joint=robot.find("ros2_control/joint[@name='gripper_finger_joint1']")
                self.assertEqual(joint.find("param[@name='read_only']").text, 'false')
                if role.startswith('teaching_'):
                    gravity=data['/test/a6_y_gravity_comp_controller']['ros__parameters']
                    self.assertEqual(gravity['haptic_effort_limits'][:6],[0.]*6)
                    from gravity_config import protocol_ranges
                    self.assertAlmostEqual(gravity['torque_limits'][6]*.0239+2*protocol_ranges(m)[7]['TMAX']/4095,2.0)
                    self.assertEqual(joint.find("param[@name='mit_kp']").text,'0.0')
                    self.assertEqual(joint.find("param[@name='mit_kd']").text,'0.0')
                self.assertEqual(float(joint.find("param[@name='min_position']").text),p['hardware_min_position_m'])
                self.assertEqual(float(joint.find("param[@name='max_position']").text),p['hardware_max_position_m'])
                bounds=robot.find("joint[@name='gripper_finger_joint1']/limit")
                mimic=robot.find("joint[@name='gripper_finger_joint2']/limit")
                self.assertEqual(float(bounds.get('lower')),p['hardware_min_position_m'])
                self.assertEqual(float(bounds.get('upper')),p['hardware_max_position_m'])
                self.assertEqual(float(mimic.get('lower')),-p['hardware_max_position_m'])
                self.assertEqual(float(mimic.get('upper')),-p['hardware_min_position_m'])
                self.assertEqual(m['joints'][gripper.GRIPPER_JOINT]['lower'],-.035)
            path,_=write_mapper_config(folder,SimpleNamespace(namespace='/l',model=load_model('teaching_left')),
                                      SimpleNamespace(namespace='/f',model=load_model('motion_left')))
            config=load_config(str(path));p=gripper.gripper_profile(load_model('motion_left'))
            self.assertEqual(config.min_position[-1],p['open_position_m'])
            self.assertEqual(config.max_position[-1],p['closed_position_m'])

    def test_actual_mapper_clamps_margins_and_slews_from_measured_extremes(self):
        from gravity_config import load_model
        from teaching_port_config import write_mapper_config
        from a6_y_teleop.leader_follower_teleop import LeaderFollowerTeleop,load_config
        with tempfile.TemporaryDirectory() as folder:
            for leader in ('teaching_left','teaching_right'):
                for follower in ('motion_left','motion_right'):
                    lm,fm=load_model(leader),load_model(follower)
                    path,_=write_mapper_config(folder,SimpleNamespace(namespace='/l',model=lm),SimpleNamespace(namespace='/f',model=fm))
                    lp,fp=gripper.gripper_profile(lm),gripper.gripper_profile(fm)
                    for edge,opposite in (('open','closed'),('closed','open')):
                        mapper=LeaderFollowerTeleop.__new__(LeaderFollowerTeleop);mapper.config=load_config(str(path))
                        mapper.last_safe=mapper.last_command_time=None
                        start=[0.]*6+[fp['measured_'+opposite+'_position_m']]
                        mapper.latest_leader=SimpleNamespace(name=mapper.config.leader_joint_names,position=[0.]*6+[lp['measured_'+edge+'_position_m']])
                        raw=mapper.map_leader_to_follower();previous=start
                        for tick in range(400):
                            now=tick*.01;safe=mapper.apply_safety(raw,previous,now)
                            if tick==0:self.assertEqual(safe,start)
                            self.assertLessEqual(abs(safe[-1]-previous[-1]),.100*.01+1e-12)
                            self.assertGreaterEqual(safe[-1],fp['min_position_m'])
                            self.assertLessEqual(safe[-1],fp['max_position_m'])
                            self.assertEqual(safe[:6],[0.]*6)
                            mapper.last_safe=safe;mapper.last_command_time=now;previous=safe
                        self.assertAlmostEqual(safe[-1],fp[edge+'_position_m'])


if __name__=='__main__':unittest.main()
