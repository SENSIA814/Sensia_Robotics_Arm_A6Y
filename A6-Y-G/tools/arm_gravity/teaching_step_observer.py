#!/usr/bin/env python3
"""Read-only telemetry in a separate interpreter, outside the UI/control GIL."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time


class StepObserverProcess:
    def __init__(self, folder, config, on_error):
        self.folder, self.on_error = Path(folder), on_error
        self.closing = self.reported = False
        path = self.folder/'step_observer.json'
        path.write_text(json.dumps(config,ensure_ascii=False,indent=2))
        with (self.folder/'step_observer.log').open('w') as output:
            self.child = subprocess.Popen([sys.executable,str(Path(__file__).resolve()),str(path)],
                stdin=subprocess.PIPE,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        # At most a handful of tiny lifecycle messages; a stalled collector can
        # never hold up disable/home while writing to its pipe.
        os.set_blocking(self.child.stdin.fileno(),False)

    def command(self, action):
        try:
            payload=(json.dumps(dict(action=action,monotonic=time.monotonic()))+'\n').encode()
            if os.write(self.child.stdin.fileno(),payload) != len(payload):
                raise OSError('incomplete telemetry lifecycle message')
        except (OSError,ValueError) as exc:
            if not self.reported:
                self.on_error(RuntimeError('阶跃采集进程通信失败：'+str(exc)))
                self.reported = True

    def enable(self, enabled):
        self.command('start' if enabled else 'pause')

    def check(self):
        code = self.child.poll()
        if code is not None and not self.closing and not self.reported:
            self.on_error(RuntimeError(f'阶跃采集进程提前退出（{code}），见 step_observer.log'))
            self.reported = True

    def close(self):
        if not self.closing:
            self.closing = True
            self.command('stop')
            self.child.stdin.close()
            # Reap asynchronously. Collection/analysis owns no motor resources.
            threading.Thread(target=self.child.wait,daemon=True,name='step-log-reaper').start()


def collect(path):
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from control_msgs.msg import DynamicJointState
    from sensor_msgs.msg import JointState
    from teaching_step_log import StepRecorder,stamp

    config=json.loads(path.read_text())
    folder=path.parent
    errors=[]
    recorder=StepRecorder(folder,config['pairs'],lambda exc:errors.append(str(exc)),devices=config['devices'])
    state=dict(enabled=False,start=0.)
    stop=threading.Event()

    def commands():
        try:
            for line in sys.stdin:
                command=json.loads(line)
                if command['action']=='start':
                    state['start']=command['monotonic']
                    recorder.metadata['following_start_monotonic']=state['start']
                    state['enabled']=True
                else:
                    was_enabled=state['enabled']
                    state['enabled']=False
                    if was_enabled:
                        recorder.metadata['following_stop_monotonic']=command['monotonic']
                    if command['action']=='stop':break
        finally:
            state['enabled']=False
            stop.set()

    def observed(kind,role,msg,leader=''):
        if state['enabled']:
            # Do not label DDS messages queued before activation as following.
            source=stamp(msg)+time.monotonic()-time.time()
            if source >= state['start']:
                recorder.put(kind,role,msg,leader)

    rclpy.init()
    node=rclpy.create_node('teaching_step_observer_'+str(os.getpid()))
    qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.BEST_EFFORT)
    try:
        for role,namespace in config['namespaces'].items():
            node.create_subscription(DynamicJointState,namespace+'/dynamic_joint_states',
                lambda msg,r=role:observed('arm',r,msg),qos)
        for leader,follower in config['pairs']:
            for kind in ('raw','safe'):
                node.create_subscription(JointState,config['topics'][follower][kind],
                    lambda msg,k=kind,l=leader,r=follower:observed(k,r,msg,l),qos)
        (folder/'step_observer.ready').write_text(str(os.getpid()))
        threading.Thread(target=commands,daemon=True).start()
        while not stop.is_set():
            rclpy.spin_once(node,timeout_sec=.02)
            if recorder.failed:break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        recorder.close()
        # This is a separate read-only process: motor shutdown never joins it.
        recorder.done.wait()
    if errors:raise RuntimeError('; '.join(errors))


if __name__=='__main__':collect(Path(sys.argv[1]))
