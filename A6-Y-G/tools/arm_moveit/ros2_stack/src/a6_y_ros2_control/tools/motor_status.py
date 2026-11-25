#!/usr/bin/env python3
"""Print A6_Y motor diagnostics from /dynamic_joint_states."""

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node


DEFAULT_TOPIC = "/dynamic_joint_states"


@dataclass
class StatusResult:
    received: bool = False
    stale: List[str] = field(default_factory=list)
    missing_feedback: List[str] = field(default_factory=list)
    faulted: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.received and not self.stale and not self.missing_feedback and not self.faulted


class MotorStatusNode(Node):
    def __init__(self, topic: str):
        super().__init__("a6_y_motor_status")
        self.last_msg: Optional[DynamicJointState] = None
        self.create_subscription(DynamicJointState, topic, self._on_msg, 10)

    def _on_msg(self, msg: DynamicJointState) -> None:
        self.last_msg = msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print A6_Y motor diagnostics from /dynamic_joint_states.")
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--timeout", type=float, default=3.0, help="Seconds to wait for the first message.")
    parser.add_argument("--watch", action="store_true", help="Keep printing status until Ctrl+C.")
    parser.add_argument("--interval", type=float, default=1.0, help="Print interval in watch mode.")
    parser.add_argument("--max-feedback-age", type=float, default=None, help="Fail if feedback_age is above this value.")
    parser.add_argument("--require-feedback", action="store_true", help="Fail if a joint has no feedback.")
    parser.add_argument("--fail-on-fault", action="store_true", help="Fail if motor_error or fault is nonzero.")
    parser.add_argument("--joints", nargs="*", default=None, help="Optional joint name filter.")
    return parser.parse_args()


def interface_map(msg: DynamicJointState) -> Dict[str, Dict[str, float]]:
    by_joint: Dict[str, Dict[str, float]] = {}
    for joint_name, values in zip(msg.joint_names, msg.interface_values):
        interfaces: Dict[str, float] = {}
        for interface_name, value in zip(values.interface_names, values.values):
            interfaces[interface_name] = float(value)
        by_joint[joint_name] = interfaces
    return by_joint


def format_value(value: Optional[float], width: int = 8, precision: int = 3) -> str:
    if value is None or not math.isfinite(value):
        return "-".rjust(width)
    return f"{value:{width}.{precision}f}"


def selected_items(
    by_joint: Dict[str, Dict[str, float]], joint_filter: Optional[Iterable[str]]
) -> List[tuple[str, Dict[str, float]]]:
    if not joint_filter:
        return sorted(by_joint.items())
    return [(name, by_joint.get(name, {})) for name in joint_filter]


def print_status(msg: DynamicJointState, joint_filter: Optional[Iterable[str]]) -> None:
    by_joint = interface_map(msg)
    rows = selected_items(by_joint, joint_filter)

    print(
        "joint".ljust(32)
        + "pos".rjust(9)
        + "vel".rjust(9)
        + "eff".rjust(9)
        + "fb".rjust(5)
        + "age".rjust(8)
        + "err".rjust(7)
        + "fault".rjust(7)
        + "ena".rjust(6)
    )
    print("-" * 90)
    for joint_name, values in rows:
        has_feedback = values.get("has_feedback")
        motor_error = values.get("motor_error")
        fault = values.get("fault")
        enabled = values.get("enabled")
        print(
            joint_name.ljust(32)
            + format_value(values.get("position"), 9)
            + format_value(values.get("velocity"), 9)
            + format_value(values.get("effort"), 9)
            + flag_text(has_feedback).rjust(5)
            + format_value(values.get("feedback_age"), 8)
            + error_text(motor_error).rjust(7)
            + flag_text(fault).rjust(7)
            + flag_text(enabled).rjust(6)
        )


def flag_text(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return "-"
    return "1" if value >= 0.5 else "0"


def error_text(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return "-"
    return f"0x{int(round(value)):X}"


def evaluate(
    msg: Optional[DynamicJointState],
    joint_filter: Optional[Iterable[str]],
    require_feedback: bool,
    max_feedback_age: Optional[float],
    fail_on_fault: bool,
) -> StatusResult:
    result = StatusResult(received=msg is not None)
    if msg is None:
        return result

    by_joint = interface_map(msg)
    for joint_name, values in selected_items(by_joint, joint_filter):
        has_feedback = values.get("has_feedback")
        feedback_age = values.get("feedback_age")
        motor_error = values.get("motor_error", 0.0)
        fault = values.get("fault", 0.0)

        if require_feedback and (has_feedback is None or has_feedback < 0.5):
            result.missing_feedback.append(joint_name)
        if (
            max_feedback_age is not None
            and feedback_age is not None
            and math.isfinite(feedback_age)
            and feedback_age > max_feedback_age
        ):
            result.stale.append(joint_name)
        raw_code = int(round(motor_error)) if motor_error is not None else 0
        if fail_on_fault and ((fault is not None and fault >= 0.5) or 0x08 <= raw_code <= 0x0E):
            result.faulted.append(joint_name)

    return result


def print_result(result: StatusResult) -> None:
    if not result.received:
        print("ERROR: no /dynamic_joint_states message received", file=sys.stderr)
        return
    if result.missing_feedback:
        print("ERROR: missing feedback: " + ", ".join(result.missing_feedback), file=sys.stderr)
    if result.stale:
        print("ERROR: stale feedback: " + ", ".join(result.stale), file=sys.stderr)
    if result.faulted:
        print("ERROR: faulted motors: " + ", ".join(result.faulted), file=sys.stderr)


def wait_for_message(node: MotorStatusNode, timeout: float) -> Optional[DynamicJointState]:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and node.last_msg is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.last_msg


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = MotorStatusNode(args.topic)
    try:
        msg = wait_for_message(node, args.timeout)
        if msg is None:
            result = evaluate(None, args.joints, args.require_feedback, args.max_feedback_age, args.fail_on_fault)
            print_result(result)
            return 2

        if not args.watch:
            print_status(msg, args.joints)
            result = evaluate(msg, args.joints, args.require_feedback, args.max_feedback_age, args.fail_on_fault)
            print_result(result)
            return 0 if result.ok else 1

        last_print = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if node.last_msg is not None and now - last_print >= args.interval:
                print_status(node.last_msg, args.joints)
                print()
                last_print = now
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
