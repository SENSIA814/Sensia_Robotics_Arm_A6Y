"""Versioned loopback protocol; all times use the same host monotonic clock."""
from dataclasses import dataclass
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation

MAX_PACKET = 2048


@dataclass(frozen=True)
class Frame:
    session: str
    sequence: int
    pose_sequence: int
    event_ns: int
    pose_ns: int
    buttons_ns: int
    clutch_epoch: int
    position: np.ndarray
    rotation: np.ndarray
    squeeze: bool
    trigger: float
    button_a: bool
    valid: bool
    connected: bool

    def fresh(self, now_ns, timeout):
        return self.valid and self.connected and all(
            0 <= now_ns - t <= timeout * 1e9
            for t in (self.event_ns, self.pose_ns, self.buttons_ns))


def decode(packet, now_ns, expected_schema=1):
    if len(packet) > MAX_PACKET:
        raise ValueError('oversize packet')
    v = json.loads(packet)
    if not isinstance(v, dict) or type(v.get('schema_version')) is not int or v.get('schema_version') != expected_schema:
        raise ValueError('unknown schema')
    if expected_schema == 2 and (v.get('purpose') != 'real' or v.get('source_kind') not in ('live', 'test')):
        raise ValueError('invalid real input purpose/source')
    if v.get('source_frame') != 'webxr_world' or v.get('hand') != 'right':
        raise ValueError('unexpected frame or hand')
    session = v['session_id']
    if not isinstance(session, str) or not 1 <= len(session) <= 80:
        raise ValueError('invalid session')
    ints = ['sequence', 'pose_sequence', 'event_received_monotonic_ns',
            'pose_received_monotonic_ns', 'buttons_received_monotonic_ns', 'clutch_epoch']
    for key in ints:
        if type(v[key]) is not int or not 0 <= v[key] < 2**63:
            raise ValueError('invalid integer: ' + key)
    for key in ints[2:5]:
        if v[key] > now_ns:
            raise ValueError('future timestamp')
    for key in ['squeeze', 'button_a', 'pose_valid', 'buttons_valid', 'connected']:
        if type(v[key]) is not bool:
            raise ValueError('invalid boolean: ' + key)
    p = np.asarray(v['position_m'], dtype=float)
    q = np.asarray(v['quaternion_xyzw'], dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('invalid pose')
    norm = np.linalg.norm(q)
    if not 0.9 <= norm <= 1.1:
        raise ValueError('quaternion not normalized')
    trigger = float(v['trigger'])
    if not math.isfinite(trigger) or not 0 <= trigger <= 1:
        raise ValueError('invalid trigger')
    return Frame(session, v['sequence'], v['pose_sequence'],
                 v['event_received_monotonic_ns'], v['pose_received_monotonic_ns'],
                 v['buttons_received_monotonic_ns'], v['clutch_epoch'], p,
                 Rotation.from_quat(q / norm).as_matrix(), v['squeeze'], trigger,
                 v['button_a'], v['pose_valid'] and v['buttons_valid'], v['connected'])


def make_packet(session, sequence, now_ns, position, quaternion=(0, 0, 0, 1),
                squeeze=False, trigger=0.5, clutch_epoch=0, **overrides):
    """Synthetic/replay adapter uses exactly the same wire schema as Quest."""
    v = dict(schema_version=1, session_id=session, sequence=sequence,
             pose_sequence=sequence, event_received_monotonic_ns=now_ns,
             pose_received_monotonic_ns=now_ns, buttons_received_monotonic_ns=now_ns,
             source_frame='webxr_world', hand='right', position_m=list(position),
             quaternion_xyzw=list(quaternion), squeeze=squeeze, trigger=trigger,
             button_a=False, clutch_epoch=clutch_epoch, pose_valid=True,
             buttons_valid=True, connected=True)
    v.update(overrides)
    return json.dumps(v, allow_nan=False).encode()
