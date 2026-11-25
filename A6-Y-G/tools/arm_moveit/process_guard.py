#!/usr/bin/env python3
"""Keep a native child bounded by the UI process lifetime; never issue motion.

A dedicated process avoids unsafe preexec_fn hooks in the multithreaded Qt/ROS
parent. Native ROS shutdown still gets a grace period before TERM/KILL.
"""
import os
import signal
import subprocess
import sys
import time


def main():
    parent = int(sys.argv[1])
    stopping = False
    def stop(_signal, _frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    if os.getppid() != parent:
        return 125
    child = subprocess.Popen(sys.argv[2:])
    try:
        while child.poll() is None and not stopping and os.getppid() == parent:
            time.sleep(.05)
    finally:
        if child.poll() is None:
            for sig, seconds in ((signal.SIGINT, 3.), (signal.SIGTERM, 2.), (signal.SIGKILL, 1.)):
                try:
                    child.send_signal(sig)
                    child.wait(timeout=seconds)
                    break
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
    code = child.poll()
    return 124 if code is None else (128-code if code < 0 else code)


if __name__ == '__main__':
    raise SystemExit(main())
