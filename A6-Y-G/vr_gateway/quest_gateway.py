#!/usr/bin/env python3
"""Quest browser -> loopback virtual input. No hardware communication."""
import argparse
import asyncio
import base64
import json
from pathlib import Path
import socket
import sys
import time
from urllib.parse import urlencode

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/a6_y_vr_teleop'))
from a6_y_vr_teleop.quest_input import RightControllerDecoder


def encode_record(event_value, packet, now):
    row = {'kind':'input','receive_ns':now,'raw_event':event_value,'packet':packet.decode()}
    def encode_binary(value):
        if isinstance(value, (bytes, bytearray, memoryview)):
            return {'bytes_base64': base64.b64encode(value).decode('ascii')}
        raise TypeError('unsupported raw event value: '+type(value).__name__)
    try:
        return json.dumps(row, allow_nan=False, default=encode_binary)+'\n'
    except (TypeError, ValueError) as error:
        # Preserve the already-decoded control packet even if an optional raw
        # browser field cannot be logged. Never change or replay its input data.
        row.update(raw_event=None, raw_recording_error=str(error))
        return json.dumps(row, allow_nan=False)+'\n'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cert',default=str(ROOT/'validation/vr_rviz/tls/cert.pem'))
    parser.add_argument('--key',default=str(ROOT/'validation/vr_rviz/tls/key.pem'))
    parser.add_argument('--host',default='0.0.0.0')
    parser.add_argument('--port',type=int,default=8012)
    parser.add_argument('--profile', choices=('virtual','real'), default='virtual')
    parser.add_argument('--udp-port',type=int)
    parser.add_argument('--record',type=Path,help='New JSONL file for raw events and canonical packets')
    args=parser.parse_args()
    if args.udp_port is None: args.udp_port = 15153 if args.profile == 'real' else 15150
    if args.profile == 'virtual' and args.udp_port == 15153:
        parser.error('15153 is reserved for real-profile live input')
    if not Path(args.cert).is_file() or not Path(args.key).is_file():
        parser.error('TLS files missing. Run scripts/create_vr_certificate.bash <LAN IP> first.')
    from aiohttp import web
    from vuer import Vuer
    from vuer.schemas import DefaultScene, MotionControllers
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    record=args.record.open('x',buffering=1) if args.record else None
    app=Vuer(host=args.host,port=args.port,cert=args.cert,key=args.key,queue_len=8,
             free_port=False,static_root=str(ROOT/'vr_gateway/public'))
    decoders={}
    event_counts={}

    @web.middleware
    async def explicit_websocket(request,handler):
        if request.method=='GET' and request.path in ('/','/index.html') and request.headers.get('Upgrade','').lower()!='websocket' and 'ws' not in request.query:
            query=dict(request.query);query['ws']='wss://'+request.host
            raise web.HTTPFound(request.path+'?'+urlencode(query))
        return await handler(request)
    app.app.middlewares.append(explicit_websocket)

    def send(packet):
        if args.profile == 'real':
            payload=json.loads(packet)
            payload.update(schema_version=2,purpose='real',source_kind='live')
            packet=json.dumps(payload,allow_nan=False).encode()
        sock.sendto(packet,('127.0.0.1',args.udp_port))
        return packet

    @app.add_handler('CONTROLLER_MOVE')
    async def controller(event,session):
        sid=str(session.CURRENT_WS_ID)
        decoder=decoders.get(sid)
        if decoder is None: return
        now=time.monotonic_ns()
        packet=decoder.update(event.value,now)
        if packet is None: return
        packet=send(packet)
        event_counts[sid]+=1
        if record:
            record.write(encode_record(event.value,packet,now))

    @app.spawn(start=False)
    async def scene(session):
        sid=str(session.CURRENT_WS_ID)
        # A browser session id is not reused across reconnections by this server.
        decoders[sid]=RightControllerDecoder(sid);event_counts[sid]=0
        session.set @ DefaultScene(frameloop='always')
        session.upsert @ MotionControllers(stream=True,fps=90,key='vr-right-controller',left=False,right=True)
        print('Quest connected:',sid,'; release grip before following.',flush=True)
        try:
            while session.CURRENT_WS_ID in app.ws and not session.socket.closed:
                await asyncio.sleep(2)
                decoder=decoders[sid]
                age=(time.monotonic_ns()-decoder.pose_ns)/1e9 if decoder.pose_ns else -1
                print('right events=%d pose_age=%.3fs grip=%s' % (event_counts[sid],age,decoder.squeeze),flush=True)
        finally:
            decoder=decoders.pop(sid,None)
            event_counts.pop(sid,None)
            if decoder: send(decoder.disconnect(time.monotonic_ns()))
            print('Quest disconnected:',sid,flush=True)

    print(args.profile+' A5 gateway: https://<LAN IP>:%d  -> 127.0.0.1:%d' % (args.port,args.udp_port),flush=True)
    try:
        app.start(free_port=False)
    finally:
        sock.close()
        if record: record.close()


if __name__=='__main__': main()
