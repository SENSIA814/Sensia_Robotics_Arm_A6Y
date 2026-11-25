"""Full position control supplies goals to the controller's trajectory limiter."""
from dataclasses import replace
import numpy as np

from a6_y_vr_teleop.position_profile import position_profile
from a6_y_vr_teleop.real_bridge import RealBridge
from test_pitch_tracking import pitch_bridge


def test_full_bridge_provides_braking_distance_without_moving_feedback():
    fixture, now, _ = pitch_bridge('down')
    _, cfg = position_profile('follow', {}, fixture.cfg, 'full')
    arm = RealBridge(fixture.kin, None, cfg, 'execute', clock=22)
    for key in ('producer', 'frame', 'anchor_frame', 'anchor_tcp', 'target',
                'live', 'latched', 'sequence', 'first_sequence', 'takeover', 'state'):
        setattr(arm, key, getattr(fixture, key))
    # All six measured axes equal the output reference: no physical lag is
    # needed to reproduce the old 8 ms goal / braking-distance mismatch.
    q = np.r_[arm.state.reference_positions, arm.state.gripper_reference_position]
    arm.state.actual_positions = q.tolist()
    for _ in range(25):
        now += 8_000_000
        arm.state.monotonic_ns = now
        arm.frame = replace(arm.frame, sequence=arm.frame.sequence+1,
            pose_sequence=arm.frame.pose_sequence+1, pose_ns=now,
            buttons_ns=now, event_ns=now)
        ref, _ = arm.tick(now, .008, enforce_budget=False)
        assert ref['valid'], arm.reason
    goal = np.asarray(ref['positions'])
    # At 1 rad/s, 3 rad/s² and 125 Hz, the local trajectory limiter needs
    # 1*.008 + 1**2/(2*3) = .1747 rad to accelerate to the requested cap.
    assert np.max(np.abs(goal-q[:6])) > .175
    assert np.all(goal >= arm.kin.lower[:6]) and np.all(goal <= arm.kin.upper[:6])
    assert np.max(np.abs(goal-q[:6])) <= .25+1e-12
    np.testing.assert_array_equal(arm.state.reference_positions, q[:6])

