#!/usr/bin/env python3
"""Send synthetic/recorded Quest packets to the isolated virtual arm only."""
import argparse
import json
from pathlib import Path
import math
import socket
import sys
import time
import uuid
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/a6_y_vr_teleop'))
from a6_y_vr_teleop.input_protocol import make_packet


def demo_packets(duration,rate=90):
    session='demo-'+uuid.uuid4().hex
    last=False;epoch=0
    for seq in range(int(duration*rate)):
        t=seq/rate;phase=t%8
        held=1<=phase<6
        if held!=last: epoch+=1
        last=held
        u=max(0,min(5,phase-1))
        # Smooth, small motion in all translation axes and one rotation axis.
        a=.12*math.sin(u*.8)
        p=[.035*math.sin(u),.025*math.sin(u*.7),-.06*math.sin(u*.6)]
        quat=[0,math.sin(a/2),0,math.cos(a/2)]
        packet=make_packet(session,seq,time.monotonic_ns(),p,quat,held,.5+.45*math.sin(u),epoch)
        yield t,packet


def main():
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--demo',action='store_true')
    source.add_argument('--replay',type=Path)
    p.add_argument('--duration',type=float,default=16)
    p.add_argument('--port',type=int,default=15150)
    args=p.parse_args()
    if args.port == 15153: p.error('real robot port 15153 rejects demo/replay')
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    origin=time.monotonic()
    try:
        if args.demo:
            if not math.isfinite(args.duration) or args.duration<=0: p.error('duration must be positive')
            for offset,packet in demo_packets(args.duration):
                time.sleep(max(0,origin+offset-time.monotonic()))
                # Generator runs before the sleep: stamp at actual send time.
                v=json.loads(packet);now=time.monotonic_ns()
                for k in ['event_received_monotonic_ns','pose_received_monotonic_ns','buttons_received_monotonic_ns']: v[k]=now
                sock.sendto(json.dumps(v).encode(),('127.0.0.1',args.port))
        else:
            rows=[]
            with args.replay.open() as f:
                for line in f:
                    r=json.loads(line)
                    if r.get('kind')=='input': rows.append(r)
            if not rows: p.error('no input records')
            base=rows[0]['receive_ns'];newbase=time.monotonic_ns()
            # File loading may take time: scheduling and remapped timestamps
            # must use the same origin or later packets appear future-dated.
            origin=newbase/1e9
            sessions={}
            for row in rows:
                offset=(row['receive_ns']-base)/1e9
                time.sleep(max(0,origin+offset-time.monotonic()))
                v=json.loads(row['packet']);oldsession=v['session_id']
                v.update(schema_version=1)
                v.pop('purpose',None); v.pop('source_kind',None)
                v['session_id']=sessions.setdefault(oldsession,'replay-'+uuid.uuid4().hex)
                for key in ['event_received_monotonic_ns','pose_received_monotonic_ns','buttons_received_monotonic_ns']:
                    if v[key]: v[key]+=newbase-base
                sock.sendto(json.dumps(v,allow_nan=False).encode(),('127.0.0.1',args.port))
    finally:
        sock.close()  # Consumer enters PAUSED on timeout; never controls hardware.


if __name__=='__main__': main()
