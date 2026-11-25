#!/usr/bin/env python3
"""ROS/dry-run integration. Physical USB is never opened by this verifier."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from teaching import TeachingCoordinator, DEFAULT_PAIRS


class SimulatedHand(TeachingCoordinator):
    offset=0.
    def _sample(self,session):
        q=super()._sample(session)
        if session.snapshot()['role']=='teaching_left':q[0]+=self.offset
        if session.snapshot()['role']=='teaching_right':q[0]-=self.offset/2
        return q


def wait(c,predicate,timeout=180):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        state=c.snapshot()
        if state['error']:raise AssertionError(state['error'])
        if predicate(state):return state
        time.sleep(.02)
    raise AssertionError('timeout: '+json.dumps(c.snapshot(),ensure_ascii=False,default=str))


def verify(two=True):
    c=SimulatedHand();pairs=DEFAULT_PAIRS if two else DEFAULT_PAIRS[:1]
    try:
        c.start(pairs,mock=True,with_rviz=False)
        wait(c,lambda s:s['phase']=='following')
        baseline={f:c.sessions[f].feedback.measured().copy() for _,f in pairs}
        for k in range(61):
            c.offset=.03*k/60
            wait(c,lambda s:s['phase']=='following',timeout=1)
            time.sleep(.02)
        time.sleep(1.5)
        state=wait(c,lambda s:s['packets']>70)
        moved={f:(c.sessions[f].feedback.measured()-baseline[f]).tolist() for _,f in pairs}
        assert .02<moved['motion_left'][0]<.035,moved
        if two:assert -.019<moved['motion_right'][0]<-.009,moved
        assert all(np.max(np.abs(v[1:6]))<.003 for v in moved.values()),moved
        assert all(s.snapshot()['active'] for s in c.sessions.values())
        c.stop();c.worker.join(180)
        final=c.snapshot();assert not c.worker.is_alive() and not c.alive(),final
        assert final['phase']=='stopped' and not final['error'],final
        for s in c.sessions.values():
            assert s.snapshot()['release_confirmed'] is True
            events=[json.loads(line) for line in Path(s.snapshot()['directory'],'events.jsonl').read_text().splitlines()]
            reached=max(i for i,e in enumerate(events) if e['kind']=='home_reached')
            released=max(i for i,e in enumerate(events) if e['kind']=='released')
            closed=max(i for i,e in enumerate(events) if e['kind']=='closed')
            assert reached<released<closed
        result=dict(result='PASS',pairs=pairs,physical_devices=0,backend='actual C++ controller + driver dry_run',
            hand_input='synthetic offset injected before incremental mapping',movement=moved,
            packets=state['packets'],actual_hz=state['actual_hz'],missed_periods=state['missed_periods'],
            all_homed_released_closed=True,report=final['directory'])
        Path(final['directory'],'verification.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
        return result
    finally:
        if c.alive():
            c.release()
            if c.worker:c.worker.join(10)
            for s in c.sessions.values():
                until=time.monotonic()+12
                while s.snapshot()['running'] and (s.snapshot()['busy'] or s.snapshot()['active']) and time.monotonic()<until:time.sleep(.02)
                s.stop()
                if s.worker:s.worker.join(20)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--single',action='store_true');args=parser.parse_args()
    print(json.dumps(verify(not args.single),ensure_ascii=False,indent=2))
