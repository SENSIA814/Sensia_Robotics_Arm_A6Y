#!/usr/bin/env python3
"""Replay logged joint positions with MoveIt and mock motors; never opens hardware."""
import json, sys
from pathlib import Path
from unittest.mock import patch
from planning_session import PlanningSession
from verify_planning import wait
raw=[.006294,-.000190738,.00171664,-.101663,.008202,-.018502,.00038952970121311915]
s=PlanningSession()
with patch('hardware.MOCK_Q',raw), patch('hardware.MotionControl.open',side_effect=AssertionError('physical hardware forbidden')):
 s.start('teaching_left',mock=True,with_rviz=False)
 try:
  wait(s,lambda s:s['ready'],35)
  s.send('gripper_target',10)
  s.send('gripper_plan',{'velocity_scaling':.4,'acceleration_scaling':.2})
  state=wait(s,lambda s:s['plan_ready'],25)
  assert '边界残差' in state['warning']
  assert s.plan.q[0,0]==raw[6]
  print('BOUNDARY_PLAN',s.plan.duration,state['warning'],flush=True)
  s.send('execute_gripper');wait(s,lambda s:s['hardware']['completed']==1,20)
  assert abs(s.runtime.measured()[6]+.0035)<.00005
  assert all(abs(s.runtime.measured()[i]-raw[i])<1e-9 for i in range(6))
  s.send('gripper_target',0);s.send('gripper_plan',{'velocity_scaling':.4,'acceleration_scaling':.2})
  wait(s,lambda s:s['plan_ready'],25)
  s.send('execute_gripper');wait(s,lambda s:s['hardware']['completed']==2,20)
  assert abs(s.runtime.measured()[6])<.00005
 finally:
  s.abort();s.worker.join(10)
 assert not s.snapshot()['error'],s.snapshot()['error']
 assert s.snapshot()['hardware']['tx_count']==0
 print('BOUNDARY_PASS',s.snapshot()['report'],flush=True)
