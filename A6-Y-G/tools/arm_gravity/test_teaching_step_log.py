import csv
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

from analyze_teaching_step import asof, response_metrics
from teaching_step_log import StepRecorder
from teaching_step_observer import StepObserverProcess
from gravity_config import JOINTS


class StepAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.t = np.arange(100.,108.,.004)
        self.u = np.where(self.t >= 102.,.5,0.)
        self.y = .5*(1.-np.exp(-np.maximum(0.,self.t-102.1)/.4))

    def test_known_positive_and_negative_step(self):
        for sign in (1.,-1.):
            r=response_metrics(self.t,sign*self.u,sign*self.u,sign*self.y)
            self.assertEqual(r['status'],'measured')
            self.assertEqual(r['input_shape'],'step_like')
            self.assertAlmostEqual(r['actual_rise_10_90_s'],.4*np.log(9),delta=.009)
            self.assertAlmostEqual(r['sent_to_actual_t10_s'],.1-.4*np.log(.9),delta=.009)
            self.assertAlmostEqual(r['settling_from_teacher_onset_s'],.1-.4*np.log(.02),delta=.009)
            self.assertTrue(r['parameter_fit_allowed'])

    def test_ramp_is_not_an_ideal_motor_step(self):
        ramp=.5*np.clip((self.t-102.)/1.,0,1)
        r=response_metrics(self.t,ramp,ramp,np.interp(self.t-.2,self.t,ramp))
        self.assertEqual(r['status'],'measured')
        self.assertEqual(r['input_shape'],'ramp_or_limited')
        self.assertFalse(r['parameter_fit_allowed'])

    def test_unmoved_axis_and_gripper_do_not_get_zero_latency(self):
        r=response_metrics(self.t,np.zeros(len(self.t)),self.u,self.y)
        self.assertEqual(r['status'],'no_excitation')
        self.assertNotIn('sent_to_actual_t10_s',r)

    def test_unsettled_error_is_not_success_or_zero_time(self):
        r=response_metrics(self.t,self.u,self.u,self.y*.7)
        self.assertEqual(r['status'],'not_settled')
        self.assertIsNone(r['actual_rise_10_90_s'])
        self.assertIsNone(r['settling_from_teacher_onset_s'])
        self.assertAlmostEqual(r['steady_error_rad'],.15,delta=.001)

    def test_return_and_reversal_are_rejected(self):
        returning=np.where((self.t>=102)&(self.t<104),.5,0)
        self.assertEqual(response_metrics(self.t,returning,returning,returning)['status'],
                         'returned_or_multiple_movements')
        reverse=np.where(self.t>=102,.7,0.)
        reverse[self.t>=104]=.5
        self.assertEqual(response_metrics(self.t,reverse,reverse,reverse)['status'],
                         'reversal_or_multiple_movements')

    def test_two_separated_moves_are_flagged(self):
        u=np.where(self.t>=102,.25,0.)
        u[self.t>=104]=.5
        r=response_metrics(self.t,u,u,u)
        self.assertIn('intermediate_pause_or_multiple_moves',r['flags'])
        self.assertFalse(r['parameter_fit_allowed'])

    def test_sample_gap_does_not_silently_allow_fit(self):
        keep=(self.t<102.2)|(self.t>102.4)
        r=response_metrics(self.t[keep],self.u[keep],self.u[keep],self.y[keep])
        self.assertIn('sample_gaps_or_invalid_feedback',r['flags'])
        self.assertFalse(r['parameter_fit_allowed'])

    def test_asof_never_uses_future_or_stale_values(self):
        q,age=asof(np.array([1.,1.05,1.2,2.]),np.array([1.02,1.02,2.]),np.array([3.,4.,5.]))
        self.assertTrue(np.isnan(q[0]) and np.isnan(q[2]))
        self.assertEqual(q[1],4.)
        self.assertAlmostEqual(age[1],.03)
        self.assertEqual(q[3],5.)

    def test_instant_virtual_response_is_resolution_limited(self):
        r=response_metrics(self.t,self.u,self.u,self.u)
        self.assertIn('response_below_observation_resolution',r['flags'])
        self.assertFalse(r['parameter_fit_allowed'])


def message(cycle=1,mode=2):
    fields=dict(control_time=100.+cycle*.004,control_cycle_count=float(cycle),dry_run=0.,
                position=.2,sent_position=.5,sent_mode=float(mode),command_count=1.,position_valid=1.)
    return NS(header=NS(stamp=NS(sec=1000,nanosec=0)),joint_names=['joint1'],
              interface_values=[NS(interface_names=list(fields),values=list(fields.values()))])


class StepRecorderTests(unittest.TestCase):
    def test_controlled_gripper_logs_and_analysis_complete_with_meter_units(self):
        with tempfile.TemporaryDirectory() as temp:
            errors=[]
            recorder=StepRecorder(temp,[('teaching_left','motion_left')],errors.append)
            msg=message()
            msg.joint_names=list(JOINTS)
            msg.interface_values=[msg.interface_values[0]]*7
            recorder.put('arm','motion_left',msg)
            # Reversed message order must still put the gripper in q_j7.
            action=NS(header=msg.header,name=list(reversed(JOINTS)),position=[-.012]+[.2]*6)
            recorder.put('safe','motion_left',action,leader='teaching_left')
            recorder.close()
            self.assertTrue(recorder.done.wait(3.))
            self.assertFalse(errors)
            quality=json.loads(Path(temp,'step_quality.json').read_text())
            self.assertTrue(quality['analysis_complete'])
            report=json.loads(Path(temp,'step_summary.json').read_text())
            gripper=report['results'][-1]
            self.assertEqual(gripper['status'],'gripper_recorded_not_analyzed')
            self.assertEqual(gripper['position_unit'],'m_single_finger')
            self.assertNotIn('steady_error_rad',gripper)
            with Path(temp,'step_targets.csv').open() as stream:
                row=next(csv.DictReader(stream))
            self.assertAlmostEqual(float(row['q_j7']),-.012)

    def test_full_lifecycle_pipe_does_not_block_or_raise_into_shutdown(self):
        observer=StepObserverProcess.__new__(StepObserverProcess)
        errors=[]
        observer.on_error=errors.append;observer.reported=False
        observer.child=NS(stdin=NS(fileno=lambda:99))
        with patch('os.write',side_effect=BlockingIOError('pipe full')):
            observer.command('pause');observer.command('stop')
        self.assertEqual(len(errors),1)
    def test_actual_errors_native_gaps_and_no_mit_position_target(self):
        with tempfile.TemporaryDirectory() as temp:
            errors=[]
            recorder=StepRecorder(temp,[('teaching_left','motion_left')],errors.append,analyze=False)
            recorder.put('arm','motion_left',message(1))
            recorder.put('arm','motion_left',message(3))
            recorder.put('arm','teaching_left',message(1,mode=1))
            recorder.close()
            self.assertTrue(recorder.done.wait(2.))
            self.assertFalse(errors)
            with Path(temp,'step_arms.csv').open() as stream:rows=list(csv.DictReader(stream))
            self.assertAlmostEqual(float(rows[0]['error_j1']),.3)
            self.assertEqual(rows[2]['error_j1'],'')
            self.assertEqual(rows[0]['error_j7'],'')
            quality=json.loads(Path(temp,'step_quality.json').read_text())
            self.assertTrue(quality['complete'])
            self.assertEqual(quality['cycles']['motion_left']['skipped'],1)

    def test_queue_full_drops_are_counted_without_blocking(self):
        with tempfile.TemporaryDirectory() as temp,patch('threading.Thread.start'):
            recorder=StepRecorder(temp,[],lambda e:None,capacity=1,analyze=False)
            recorder.put('arm','motion_left',message())
            recorder.put('arm','motion_left',message())
            self.assertEqual(recorder.dropped,{'arm:motion_left':1})
            recorder.close()

    def test_disk_failure_is_reported_without_blocking_close(self):
        with tempfile.TemporaryDirectory() as temp:
            errors=[]
            with patch.object(StepRecorder,'_arm_row',side_effect=OSError('full disk')):
                recorder=StepRecorder(temp,[],errors.append,analyze=False)
                recorder.put('arm','motion_left',message())
                recorder.close()
                self.assertTrue(recorder.done.wait(2.))
            self.assertEqual(len(errors),1)
            self.assertTrue(recorder.failed)
            self.assertFalse(json.loads(Path(temp,'step_quality.json').read_text())['complete'])

    def test_close_does_not_wait_for_stalled_writer(self):
        entered,release=threading.Event(),threading.Event()
        original=StepRecorder._arm_row
        def stalled(recorder,*args):
            entered.set();release.wait(2.)
            return original(recorder,*args)
        with tempfile.TemporaryDirectory() as temp,patch.object(StepRecorder,'_arm_row',stalled):
            recorder=StepRecorder(temp,[],lambda e:None,analyze=False)
            recorder.put('arm','motion_left',message())
            self.assertTrue(entered.wait(1.))
            started=time.monotonic();recorder.close()
            elapsed=time.monotonic()-started
            release.set();self.assertTrue(recorder.done.wait(2.))
            self.assertLess(elapsed,.05)


if __name__=='__main__':unittest.main()
