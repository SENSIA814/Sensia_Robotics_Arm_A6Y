#!/usr/bin/env python3
"""All four arms: startup-home and gripper limits using C++ dry_run only."""
import csv
import json
from pathlib import Path
import time
import numpy as np

from motion_model import ROOT, ROLE_ORDER
from native_session import PlanningSession
from verify_native import wait


def main():
    reports = []
    for role in ROLE_ORDER:
        s = PlanningSession()
        s.start(role, mock=True, with_rviz=False, home_on_start=True)
        try:
            home = wait(s, lambda v: v.get('startup_home') == 'complete', timeout=100)
            folder = Path(home['directory'])
            assert home['hardware']['enabled'] and not home['hardware']['executing']
            assert max(abs(v) for v in home['hardware']['q'][:6]) <= .005
            assert home['hardware']['completed'] == 1
            assert home['hardware']['plot_zero_pose'] is not None
            deadline = time.monotonic()+3
            sample = None
            while time.monotonic() < deadline:
                samples = s.telemetry_snapshot()['samples']
                if samples and samples[-1]['plot_zero_at'] is not None and all(samples[-1]['valid']):
                    sample = samples[-1]
                    break
                time.sleep(.03)
            assert sample is not None
            np.testing.assert_allclose(sample['tcp_xyz_mm']+sample['tcp_rpy_deg'], [0.]*6, atol=1e-6)
            assert np.linalg.norm(sample['tcp_absolute_xyz_mm']) > 1.
            q = s.runtime.measured()
            time.sleep(.5)
            np.testing.assert_allclose(s.runtime.measured(), q, atol=1e-6)
            print(role+': startup home, hold, zero-relative curves PASS', flush=True)
            grip_results = []
            if role == 'motion_left':
                for percent, completed in ((0., 2), (100., 3)):
                    s.send('gripper_target', percent)
                    s.send('gripper_plan')
                    wait(s, lambda v: v['plan_ready'] and v['plan_group'] == 'gripper')
                    s.send('execute_gripper')
                    wait(s, lambda v: v['hardware']['completed'] == completed)
                    measured = s.runtime.measured()[6]
                    assert abs(measured - s.gripper_target) < 1e-5
                    assert -.035 < measured < 0.
                    grip_results.append(dict(percent=percent, target_m=s.gripper_target, feedback_m=measured))
                    print(f'  gripper {percent}%: {measured*1000:.4f} mm PASS', flush=True)
            s.stop()
            wait(s, lambda v: v['close_allowed'], timeout=60, allow_error=True)
            s.worker.join(15)
            assert not s.worker.is_alive()
            assert not s.snapshot()['hardware']['enabled']
            assert not s.recorder.error
            logs = (folder/'controller_manager.log').read_text()
            assert 'Opened SLCAN bus' not in logs
            assert all(p.poll() is not None for _, p in s.display.children)
            rows = list(csv.DictReader((folder/'telemetry.csv').open()))
            assert any(r['plot_zero_at'] for r in rows)
            reports.append(dict(role=role, result='PASS', report=s.snapshot()['report'],
                                zero_xyz_mm=sample['tcp_xyz_mm'], zero_rpy_deg=sample['tcp_rpy_deg'],
                                absolute_xyz_mm=sample['tcp_absolute_xyz_mm'], gripper=grip_results,
                                telemetry_rows=len(rows), shutdown_confirmed=True))
            print(role+': release and exit PASS', flush=True)
        finally:
            s.abort()
            s.worker.join(15)
    result = dict(result='PASS', backend='MoveIt + original 100 Hz JTC + C++ dry_run, no real device', roles=reports)
    output = ROOT.parent/'diagnostics/arm_moveit_startup_home_validation_20261125.json'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(str(output), flush=True)


if __name__ == '__main__':
    main()
