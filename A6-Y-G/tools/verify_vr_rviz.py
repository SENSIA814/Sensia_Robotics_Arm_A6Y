#!/usr/bin/env python3
"""Launch an isolated virtual graph and validate 60s of synthetic teleoperation."""
import json,os,signal,socket,subprocess,sys,time,uuid
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from diagnostic_msgs.msg import DiagnosticArray
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/a6_y_vr_teleop'))
from a6_y_vr_teleop.input_protocol import make_packet
from vr_replay import demo_packets


def stats(x):
    a=np.asarray(x,dtype=float)
    return dict(n=len(a),mean=float(a.mean()),p99=float(np.percentile(a,99)),max=float(a.max())) if len(a) else None


def main():
    out=ROOT/'validation/vr_rviz'
    cfg=yaml.safe_load((ROOT/'src/a6_y_vr_teleop/config/a5_vr.yaml').read_text())
    cfg['domain_id']=85;cfg['udp_port']=15152
    config=out/'integration_config.yaml';config.write_text(yaml.safe_dump(cfg))
    os.environ['ROS_DOMAIN_ID']='85';os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init();node=rclpy.create_node('vr_virtual_integration_probe')
    q=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    js=[];diag=[];frames=set()
    node.create_subscription(JointState,'/arm6_vr/joint_states',lambda m:js.append((time.monotonic(),m)),q)
    node.create_subscription(DiagnosticArray,'/arm6_vr/status',lambda m:diag.append(m),q)
    def tf_cb(m):
        for t in m.transforms: frames.update([t.header.frame_id,t.child_frame_id])
    node.create_subscription(TFMessage,'/tf',tf_cb,q)
    node.create_subscription(TFMessage,'/tf_static',tf_cb,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    log=(out/'integration_launch.log').open('w')
    record=out/('integration_'+uuid.uuid4().hex[:8]+'.jsonl')
    proc=subprocess.Popen(['ros2','launch','a6_y_vr_teleop','a5_vr_rviz.launch.py',
        'use_rviz:=false','config:='+str(config),'record_file:='+str(record)],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:
        deadline=time.monotonic()+15
        while not js and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if proc.poll() is not None: raise RuntimeError('launch exited; inspect integration_launch.log')
        assert js,'no virtual JointState'
        js.clear();diag.clear();origin=time.monotonic();next_print=origin+15
        packets=iter(demo_packets(60,90));offset,packet=next(packets)
        exhausted=False
        while time.monotonic()-origin<60:
            now=time.monotonic()
            if not exhausted and now-origin>=offset:
                v=json.loads(packet);ns=time.monotonic_ns()
                for key in ['event_received_monotonic_ns','pose_received_monotonic_ns','buttons_received_monotonic_ns']:v[key]=ns
                sock.sendto(json.dumps(v).encode(),('127.0.0.1',15152))
                try: offset,packet=next(packets)
                except StopIteration: exhausted=True
            rclpy.spin_once(node,timeout_sec=.001)
            if now>=next_print:
                print('samples',len(js),'elapsed',round(now-origin),flush=True);next_print+=15
        sample_js=js.copy()
        # No more input: check dropout freeze and state transition.
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.005)
        tail=[m for t,m in js if t>deadline-.2]
        assert tail and all(np.array_equal(m.position,tail[0].position) for m in tail),'timeout did not freeze'
        statuses=[{v.key:v.value for v in s.values} for msg in diag for s in msg.status]
        assert statuses[-1]['state']=='PAUSED','timeout state missing'
        stamps=np.array([m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for _,m in sample_js])
        positions=np.array([m.position for _,m in sample_js])
        assert len(sample_js)>7000 and np.max(np.ptp(positions[:,:6],axis=0))>.02
        assert np.ptp(positions[:,6])>.005,'gripper did not move'
        assert all(m.name==[f'joint{i}' for i in range(1,7)]+['gripper_finger_joint1'] for _,m in sample_js)
        assert 'vr_a5/tool_tcp' in frames and len(frames)>=10 and all(f.startswith('vr_a5/') for f in frames)
        names=node.get_node_names_and_namespaces()
        assert not any('controller_manager' in n or 'damiao' in n for n,ns in names)
        fd_targets=[];children=[]
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():continue
            try:
                if os.getpgid(int(p.name))!=proc.pid:continue
                children.append(int(p.name))
                for fd in (p/'fd').iterdir():
                    try: fd_targets.append(os.readlink(fd))
                    except OSError:pass
            except (PermissionError,ProcessLookupError,FileNotFoundError):pass
        assert not any('/dev/ttyACM' in f or '/dev/ttyUSB' in f for f in fd_targets)
        dt=np.diff(stamps)
        report=dict(passed=True,duration_s=60,domain_id=85,configured_hz=125,
            measured_state_hz=float(1/np.mean(dt)),stamp_period_ms=stats(dt*1000),
            received_messages=len(sample_js),joint_spans=np.ptp(positions,axis=0).tolist(),
            state_modes=sorted(set(s['state'] for s in statuses)),
            compute_ms=stats([float(s['compute_ms']) for s in statuses]),
            callback_max_ms=max(float(s['callback_max_ms']) for s in statuses),
            handled_overruns=max(int(s['overruns']) for s in statuses),
            reasons=sorted(set(s['reason'] for s in statuses)),
            tf_frames=sorted(frames),nodes=names,processes_inspected=children,
            hardware_device_opened=False,actual_quest_tested=False,record_file=str(record))
        (out/'integration_check.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)
    finally:
        sock.close()
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
