#!/usr/bin/env python3
"""Test a local gateway instance with a synthetic WebXR websocket client."""
import asyncio,json,socket,ssl,time
from pathlib import Path
import aiohttp,msgpack,numpy as np


async def run():
    context=ssl.create_default_context();context.check_hostname=False;context.verify_mode=ssl.CERT_NONE
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',15151));sock.setblocking(False)
    async with aiohttp.ClientSession() as client:
        async with client.get('https://127.0.0.1:8013/',ssl=context,allow_redirects=False) as r:
            assert r.status==302 and 'ws=' in r.headers['Location']
        async with client.get('https://127.0.0.1:8013/?ws=wss://127.0.0.1:8013',ssl=context) as r:
            assert r.status==200 and '<html' in (await r.text()).lower()
        async with client.get('https://127.0.0.1:8013/static/validation/vr_rviz/tls/key.pem',ssl=context) as r:
            assert r.status==404
        async with client.ws_connect('https://127.0.0.1:8013/',ssl=context) as ws:
            # Receiving the scene verifies that the server initialized the session.
            for _ in range(2): await asyncio.wait_for(ws.receive(),5)
            T=np.eye(4);T[:3,3]=[.1,.2,.3]
            packets=[]
            for squeeze in [False,True,False,True]:
                await ws.send_bytes(msgpack.packb(dict(etype='CONTROLLER_MOVE',value={
                    'right':T.flatten(order='F').tolist(),
                    'rightState':{'squeeze':squeeze,'triggerValue':.7}})))
                data=await asyncio.wait_for(asyncio.get_running_loop().sock_recv(sock,2048),2)
                packets.append(json.loads(data))
            assert [p['clutch_epoch'] for p in packets]==[0,1,2,3]
            assert packets[-1]['position_m']==[.1,.2,.3]
            assert packets[-1]['trigger']==.7 and packets[-1]['pose_valid']
            assert all(a['pose_received_monotonic_ns']<b['pose_received_monotonic_ns'] for a,b in zip(packets,packets[1:]))
    sock.close()
    report={'passed':True,'tls_http':True,'websocket_events':4,'clutch_epochs':[0,1,2,3],
            'private_file_not_served':True,'hardware_accessed':False,'actual_quest_tested':False}
    path=Path(__file__).resolve().parents[1]/'validation/vr_rviz/gateway_check.json'
    path.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__': asyncio.run(run())
