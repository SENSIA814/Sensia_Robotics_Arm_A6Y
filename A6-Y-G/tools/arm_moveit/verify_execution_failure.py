#!/usr/bin/env python3
"""Real MoveIt/RViz lifecycle with injected execution faults; no physical bus."""
import argparse
import json
from pathlib import Path
import threading
import time
from unittest.mock import patch

import numpy as np

from planning_session import PlanningSession
from trajectory import Trajectory


def wait(session, predicate, timeout=35):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = session.snapshot()
        if predicate(state):
            return state
        if not state['running']:
            raise AssertionError('Unexpected session exit: ' + str(state))
        time.sleep(.03)
    raise AssertionError('Timed out: ' + str(session.snapshot()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--screenshot', help='Save the virtual display after the injected failure')
    args = parser.parse_args()
    session = PlanningSession()
    inject = threading.Event()
    original = Trajectory.sample

    def sample(trajectory, elapsed):
        desired, velocity = original(trajectory, elapsed)
        if threading.current_thread() is session.runtime.worker and inject.is_set():
            inject.clear()
            desired = desired.copy()
            desired[0] += .2  # Fault injection only, never reaches a physical bus.
        return desired, velocity

    with patch('hardware.MotionControl.open', side_effect=AssertionError('physical bus forbidden')), patch.object(Trajectory, 'sample', sample):
        session.start('motion_left', mock=True, with_rviz=args.rviz)
        try:
            wait(session, lambda s: s['ready'] and s['target_valid'])
            children = dict(session.display.children)
            q = session.runtime.measured()
            q[0] += .04
            session.send('target', session.kin.fk(q))
            session.send('plan')
            wait(session, lambda s: s['plan_ready'])
            inject.set()
            session.send('execute')
            failed = wait(session, lambda s: s['handled_motion_failures'] == 1)
            assert failed['ready'] and not failed['plan_ready']
            assert not failed['hardware']['executing'] and failed['hardware']['enabled']
            assert not failed['hardware']['error'] and failed['hardware']['completed'] == 0
            assert session.runtime.worker.is_alive() and all(c.poll() is None for c in children.values())
            before = failed['hardware']['sample_time']
            wait(session, lambda s: s['hardware']['sample_time'] > before + .3)
            folder = Path(failed['directory'])
            (folder / 'tracking_failure_alive.json').write_text(json.dumps(session.snapshot(), ensure_ascii=False, indent=2))
            if args.screenshot:
                from PyQt5.QtWidgets import QApplication
                app = QApplication.instance() or QApplication([])
                time.sleep(.3)
                app.primaryScreen().grabWindow(0).save(args.screenshot)
            # A new explicit target/plan/execute works without restarting ROS.
            session.send('reset')
            wait(session, lambda s: s['target_valid'])
            q = session.runtime.measured()
            q[0] += .025
            session.send('target', session.kin.fk(q))
            session.send('plan')
            wait(session, lambda s: s['plan_ready'])
            session.send('execute')
            wait(session, lambda s: s['hardware']['completed'] == 1)
            # The same fault during exit must cancel exit, not kill RViz.
            inject.set()
            session.stop()
            failed_home = wait(session, lambda s: s['handled_motion_failures'] == 2 and s['close_failed'])
            assert not failed_home['home_active'] and not failed_home['close_allowed']
            assert all(c.poll() is None for c in children.values())
            (folder / 'home_failure_alive.json').write_text(json.dumps(failed_home, ensure_ascii=False, indent=2))
            # A stopped communication worker also cannot tear down the display.
            session.runtime.cancel_motion(shutdown=True)
            session.runtime.worker.join(5)
            assert not session.runtime.worker.is_alive()
            session.runtime.update(error='TEST ONLY: simulated communication failure', q=None)
            fault = wait(session, lambda s: s['runtime_fault'] is not None)
            assert not fault['ready'] and fault['running']
            assert all(c.poll() is None for c in children.values())
            session.stop()  # Already released: explicit close is now allowed.
            session.worker.join(10)
            assert not session.worker.is_alive()
            assert session.snapshot()['close_allowed']
            assert session.runtime.snapshot()['tx_count'] == 0
            events = [json.loads(line) for line in (folder / 'events.jsonl').read_text().splitlines()]
            assert len([e for e in events if e.get('execution_failure', {}).get('sequence') in (1, 2)]) >= 2
            print(json.dumps({'result': 'PASS', 'rviz': args.rviz, 'physical_can_frames': 0,
                              'report': session.snapshot()['report']}, ensure_ascii=False), flush=True)
        finally:
            session.abort()
            session.worker.join(10)


if __name__ == '__main__':
    main()
