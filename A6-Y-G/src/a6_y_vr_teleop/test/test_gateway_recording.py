import base64
import importlib.util
import json
from pathlib import Path

path = Path(__file__).resolve().parents[3]/'vr_gateway/quest_gateway.py'
spec = importlib.util.spec_from_file_location('a5_quest_gateway_recording_test', path)
gateway = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gateway)


def test_binary_controller_event_is_recorded_without_changing_canonical_packet():
    packet = b'{"valid":true,"sequence":42}'
    raw = {'right': {'buffer': b'\x00\xff\x01'}, 'buttons': [False]}
    row = json.loads(gateway.encode_record(raw, packet, 123))
    assert row['packet'] == packet.decode()
    assert base64.b64decode(row['raw_event']['right']['buffer']['bytes_base64']) == b'\x00\xff\x01'
    assert raw['right']['buffer'] == b'\x00\xff\x01'


def test_unserializable_raw_event_does_not_discard_canonical_packet():
    packet = b'{"valid":false,"sequence":43}'
    row = json.loads(gateway.encode_record({'bad': float('nan')}, packet, 123))
    assert row['packet'] == packet.decode()
    assert row['raw_event'] is None and row['raw_recording_error']
