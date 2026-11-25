import json
from pathlib import Path
import tempfile
import unittest
import zipfile
import xml.etree.ElementTree as ET

import yaml

from gravity_config import load_model,load_profile,write_config
from teaching_logs import LogTail,SessionLog,export_report,report_arms


class TeachingReportTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.folder=self.root/'teaching_test';self.folder.mkdir()
        self.arm=self.root/'arms'/'teaching_left_test';self.arm.mkdir(parents=True)
        self.state=dict(phase='error',status='失败',error='第一次故障',close_error='退出故障',directory=str(self.folder),
            arms={'teaching_left':dict(directory=str(self.arm),running=True,release_confirmed=False)})
        self.log=SessionLog(self.folder);self.log.snapshot(self.state)
        self.log.event('first_failure',level='ERROR',message='第一次故障')
        (self.arm/'events.jsonl').write_text('{"time":1,"kind":"home_aborted","message":"关节4"}\n')
        (self.arm/'controller_manager.log').write_text('controller diagnostic\n')

    def tearDown(self):self.temp.cleanup()

    def test_bundle_includes_exact_arms_and_is_relocatable(self):
        unrelated=self.root/'arms'/'motion_unused';unrelated.mkdir();(unrelated/'secret.log').write_text('unrelated')
        (self.arm/'skip_symlink').symlink_to(unrelated/'secret.log')
        dest=export_report(self.folder,self.root/'report.zip',self.state)
        with zipfile.ZipFile(dest) as z:
            self.assertIn('arms/teaching_left/events.jsonl',z.namelist())
            self.assertIn('arms/teaching_left/controller_manager.log',z.namelist())
            self.assertFalse(any('secret' in n or 'symlink' in n for n in z.namelist()))
            self.assertIn('第一次故障',z.read('summary.txt').decode())
            self.assertTrue(json.loads(z.read('bundle.json'))['live'])
            unpacked=self.root/'unpacked';z.extractall(unpacked)
        self.assertEqual(report_arms(unpacked)['teaching_left'],unpacked/'arms/teaching_left')

    def test_export_rejects_external_arm_path(self):
        self.state['arms']['teaching_left']['directory']=str(self.root)
        with self.assertRaises(ValueError):export_report(self.folder,self.root/'bad.zip',self.state)
        self.assertFalse((self.root/'bad.zip').exists())

    def test_tail_reads_new_complete_utf8_lines_and_filters_role(self):
        tail=LogTail();initial=tail.read(self.folder,state=self.state)
        self.assertTrue(any('第一次故障' in s for s in initial));self.assertTrue(any('关节4' in s for s in initial))
        self.assertEqual(tail.read(self.folder,state=self.state),[])
        path=self.arm/'controller_manager.log'
        with path.open('ab') as f:f.write('未完成'.encode())
        self.assertEqual(tail.read(self.folder,state=self.state),[])
        with path.open('ab') as f:f.write('的一行\n'.encode())
        self.assertIn('未完成的一行',tail.read(self.folder,state=self.state)[0])
        filtered=LogTail().read(self.folder,role='coordinator',state=self.state)
        self.assertFalse(any('关节4' in s for s in filtered))
        important=LogTail().read(self.folder,state=self.state,detailed=False)
        self.assertFalse(any('controller diagnostic' in s for s in important))
        self.assertTrue(any('第一次故障' in s for s in important))

    def test_position_output_config_keeps_teacher_mit_and_gripper_readonly(self):
        for role,position in [('teaching_left',False),('motion_left',True)]:
            model=load_model(role);p=load_profile(role);p.update(teaching_follow_enabled=True,teaching_position_output=position)
            path=write_config(model,p,self.root,'/test','manager',False,True)
            config=yaml.safe_load(path.read_text());robot=ET.fromstring(config['/test/manager']['ros__parameters']['robot_description'])
            hw=robot.find('ros2_control/hardware')
            self.assertEqual(hw.find("param[@name='control_mode']").text,'pos_vel' if position else 'mit')
            joints=robot.findall('ros2_control/joint')
            self.assertEqual(joints[-1].find("param[@name='read_only']").text,'true')
            if position:
                for joint in joints[:6]:self.assertEqual(float(joint.find("param[@name='max_velocity']").text),.1)


if __name__=='__main__':unittest.main()
