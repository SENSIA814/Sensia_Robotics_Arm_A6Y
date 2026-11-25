"""A slow recording load must not create future-dated live replay packets."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from a6_y_vr_teleop.input_protocol import decode, make_packet


def test_realtime_replay_timestamps_share_scheduling_origin(tmp_path, monkeypatch):
    script = Path(__file__).resolve().parents[3] / 'tools/vr_replay.py'
    spec = importlib.util.spec_from_file_location('vr_replay_under_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    base = 10**12
    rows = []
    for seq in range(3):
        stamp = base + seq * 10_000_000
        packet = make_packet('recorded', seq, stamp, [0, 0, 0], [0, 0, 0, 1], False, .5, 0)
        rows.append(json.dumps(dict(kind='input', receive_ns=stamp, packet=packet.decode())))
    recording = tmp_path / 'input.jsonl'
    recording.write_text('\n'.join(rows) + '\n')
    clock = dict(now=2000., first=True)

    def monotonic():
        if clock['first']:
            clock['first'] = False
            # Simulate a two-second file load after main's initial timestamp.
            return clock['now'] - 2.
        return clock['now']

    def sleep(seconds):
        clock['now'] += seconds

    frames = []

    def send(packet, destination):
        assert destination == ('127.0.0.1', 15150)
        now = round(clock['now'] * 1e9)
        frame = decode(packet, now)
        assert frame.fresh(now, .15)
        frames.append(frame)

    socket = SimpleNamespace(sendto=send, close=lambda: None)
    monkeypatch.setattr(module, 'socket', SimpleNamespace(AF_INET=2, SOCK_DGRAM=2, socket=lambda *_: socket))
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=monotonic,
        monotonic_ns=lambda: round(clock['now'] * 1e9), sleep=sleep))
    monkeypatch.setattr(module.sys, 'argv', ['vr_replay.py', '--replay', str(recording)])
    module.main()
    assert len(frames) == 3
    assert len({frame.session for frame in frames}) == 1
    assert frames[0].session != 'recorded'
