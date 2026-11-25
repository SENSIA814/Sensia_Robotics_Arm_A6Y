#!/usr/bin/env python3
"""Kill ONLY a test-owned mock backend and verify its ROS/RViz children exit."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def alive(pid):
    p=Path('/proc')/str(pid)/'status'
    if not p.exists(): return False
    try: return '\nState:\tZ' not in p.read_text()
    except FileNotFoundError: return False


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--owner')
    args=parser.parse_args()
    if args.owner:
        from native_session import PlanningSession
        from verify_native import wait
        s=PlanningSession()
        s.start('motion_left', mock=True, with_rviz=True)
        try:
            state=wait(s,lambda v:v['ready'] and v['target_valid'])
            guards=[p.pid for _,p in s.display.children if p.poll() is None]
            native=[]
            for pid in guards:
                native.extend(int(n) for n in Path(f'/proc/{pid}/task/{pid}/children').read_text().split())
            Path(args.owner).write_text(json.dumps(dict(directory=state['directory'], guards=guards, native=native, mock=True)))
            while True: time.sleep(.1)
        finally:
            s.abort()
            s.worker.join(15)
    else:
        with tempfile.TemporaryDirectory() as folder:
            statefile=Path(folder)/'owner.json'
            logfile=open('/tmp/a6y-shutdown-owner.log','w')
            owner=subprocess.Popen([sys.executable,__file__,'--owner',str(statefile)],stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
            state=None
            try:
                deadline=time.monotonic()+45
                while not statefile.exists() and time.monotonic()<deadline and owner.poll() is None: time.sleep(.05)
                assert statefile.exists(), Path('/tmp/a6y-shutdown-owner.log').read_text()[-3000:]
                state=json.loads(statefile.read_text())
                assert state['mock'] and len(state['native'])>=5
                owner.kill()  # Simulated backend crash; never a user's process.
                owner.wait(timeout=3)
                start=time.monotonic()
                while any(alive(pid) for pid in state['guards']+state['native']) and time.monotonic()-start<9: time.sleep(.05)
                remaining=[pid for pid in state['guards']+state['native'] if alive(pid)]
                assert not remaining, remaining
                control_log=Path(state['directory'],'controller_manager.log').read_text()
                assert 'Opened SLCAN bus' not in control_log
                assert "Successful 'deactivate'" in control_log
                result=dict(result='PASS', mock_only=True, physical_devices=0, backend_crash_simulated=True,
                            all_guard_and_native_processes_exited=True, seconds=round(time.monotonic()-start,2),
                            directory=state['directory'])
                Path(state['directory'],'shutdown_verification.json').write_text(json.dumps(result,indent=2))
                print(json.dumps(result,indent=2),flush=True)
            finally:
                if owner.poll() is None:
                    owner.terminate()
                    try: owner.wait(timeout=8)
                    except subprocess.TimeoutExpired: owner.kill();owner.wait(timeout=2)
                if state:
                    for pid in state['guards']:
                        if alive(pid):
                            try: os.killpg(pid,signal.SIGKILL)
                            except ProcessLookupError: pass
                logfile.close()


if __name__=='__main__':
    main()
