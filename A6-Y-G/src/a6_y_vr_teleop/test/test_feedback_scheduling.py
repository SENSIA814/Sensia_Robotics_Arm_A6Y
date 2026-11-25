"""A slow solve must not make the next solve reuse queued, older feedback."""
from types import SimpleNamespace

from a6_y_vr_teleop.real_node import RealBridgeNode


def test_accepted_feedback_drives_one_update_and_duplicates_do_not():
    # No ROS graph or hardware. Exercise the node's actual dispatch boundary.
    node = RealBridgeNode.__new__(RealBridgeNode)
    old = SimpleNamespace(state=3, reason=4, monotonic_ns=100)
    new = SimpleNamespace(state=3, reason=4, monotonic_ns=200)
    node.arm = SimpleNamespace(state=old)
    def observe(state, now):
        if state.monotonic_ns > node.arm.state.monotonic_ns:
            node.arm.state = state
    node.arm.observe = observe
    node.record = None
    seen = []
    node.update = lambda: seen.append(node.arm.state.monotonic_ns)
    node.observe(new)
    node.observe(old)
    assert seen == [200], 'compute did not run on newly received feedback'


def test_watchdog_never_computes_on_fresh_but_queued_feedback():
    node = RealBridgeNode.__new__(RealBridgeNode)
    node.arm = SimpleNamespace(feedback_ok=lambda now: True)
    seen = []
    node.update = lambda: seen.append(True)
    node.watchdog()
    assert seen == []
    node.arm.feedback_ok = lambda now: False
    node.watchdog()
    assert seen == [True], 'missing feedback did not refresh the stopped diagnostic'
