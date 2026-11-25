#!/usr/bin/env python3
"""Capture A6_Y joint positions and propose sign/offset updates for zero calibration."""

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional

import yaml

try:
    import rclpy
    from control_msgs.msg import DynamicJointState
    from rclpy.node import Node
except ImportError:  # pragma: no cover - allows --position use without ROS sourced.
    rclpy = None
    DynamicJointState = None
    Node = object


DEFAULT_TOPIC = "/dynamic_joint_states"


EXPECTED_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


@dataclass
class JointObservation:
    position: float
    has_feedback: Optional[float] = None
    feedback_age: Optional[float] = None
    motor_error: Optional[float] = None
    fault: Optional[float] = None


class SnapshotNode(Node):
    def __init__(self, topic: str):
        super().__init__("a6_y_capture_zero_offsets")
        self.last_msg = None
        self.create_subscription(DynamicJointState, topic, self._on_msg, 10)

    def _on_msg(self, msg) -> None:
        self.last_msg = msg


def default_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("A6_Y_Moveitconfig")) / "config" / "a6_y_damiao_motors.yaml"
    except Exception:
        return Path(__file__).resolve().parents[2] / "A6_Y_Moveitconfig" / "config" / "a6_y_damiao_motors.yaml"


def parse_joint_value(text: str) -> tuple[str, float]:
    if "=" not in text:
        raise argparse.ArgumentTypeError("expected JOINT=VALUE")
    joint, value = text.split("=", 1)
    joint = joint.strip()
    if not joint:
        raise argparse.ArgumentTypeError("joint name must not be empty")
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid numeric value in {text!r}") from exc
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError(f"value in {text!r} must be finite")
    return joint, parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--target", type=float, default=0.0, help="Default desired joint position at calibration pose.")
    parser.add_argument(
        "--target-joint",
        action="append",
        type=parse_joint_value,
        default=[],
        metavar="JOINT=VALUE",
        help="Override desired joint position for one joint.",
    )
    parser.add_argument(
        "--sign",
        action="append",
        type=parse_joint_value,
        default=[],
        metavar="JOINT=+1|-1",
        help="Override sign while computing the proposed offset.",
    )
    parser.add_argument(
        "--position",
        action="append",
        type=parse_joint_value,
        default=[],
        metavar="JOINT=VALUE",
        help="Use an offline observed joint position instead of /dynamic_joint_states.",
    )
    parser.add_argument("--max-feedback-age", type=float, default=None)
    parser.add_argument("--allow-missing", action="store_true")
    parser.add_argument("--allow-fault", action="store_true")
    parser.add_argument("--output", type=Path, default=None, help="Optional YAML patch output path.")
    parser.add_argument("--decimals", type=int, default=6)
    return parser.parse_args()


def load_motor_config(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return data["a6_y_damiao_system"]["ros__parameters"]["joints"]


def dynamic_joint_state_to_observations(msg) -> Dict[str, JointObservation]:
    observations: Dict[str, JointObservation] = {}
    for joint_name, values in zip(msg.joint_names, msg.interface_values):
        interface_values = {
            interface_name: float(value)
            for interface_name, value in zip(values.interface_names, values.values)
        }
        if "position" not in interface_values:
            continue
        observations[joint_name] = JointObservation(
            position=interface_values["position"],
            has_feedback=interface_values.get("has_feedback"),
            feedback_age=interface_values.get("feedback_age"),
            motor_error=interface_values.get("motor_error"),
            fault=interface_values.get("fault"),
        )
    return observations


def wait_for_observations(topic: str, timeout: float) -> Dict[str, JointObservation]:
    if rclpy is None:
        raise RuntimeError("ROS Python modules are not available; use --position for offline input")

    rclpy.init()
    node = SnapshotNode(topic)
    try:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and node.last_msg is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.last_msg is None:
            raise TimeoutError(f"no {topic} message received within {timeout:.1f} s")
        return dynamic_joint_state_to_observations(node.last_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_offline_observations(values: Iterable[tuple[str, float]]) -> Dict[str, JointObservation]:
    return {joint: JointObservation(position=value, has_feedback=1.0, feedback_age=0.0, motor_error=0.0, fault=0.0)
            for joint, value in values}


def rounded(value: float, decimals: int) -> float:
    return round(value, decimals)


def validate_inputs(
    joints: Dict,
    observations: Dict[str, JointObservation],
    max_feedback_age: Optional[float],
    allow_missing: bool,
    allow_fault: bool,
) -> list[str]:
    errors: list[str] = []
    for joint in EXPECTED_JOINTS:
        if joint not in joints:
            errors.append(f"{joint}: missing from motor config")
            continue
        observation = observations.get(joint)
        if observation is None:
            if not allow_missing:
                errors.append(f"{joint}: missing observed position")
            continue
        if observation.has_feedback is not None and observation.has_feedback < 0.5 and not allow_missing:
            errors.append(f"{joint}: has_feedback is 0")
        if (
            max_feedback_age is not None
            and observation.feedback_age is not None
            and observation.feedback_age > max_feedback_age
        ):
            errors.append(f"{joint}: feedback_age {observation.feedback_age:.3f} exceeds {max_feedback_age:.3f}")
        if not allow_fault:
            if observation.motor_error is not None and int(round(observation.motor_error)) != 0:
                errors.append(f"{joint}: motor_error is 0x{int(round(observation.motor_error)):X}")
            if observation.fault is not None and observation.fault >= 0.5:
                errors.append(f"{joint}: fault is 1")
    return errors


def propose_patch(
    joints: Dict,
    observations: Dict[str, JointObservation],
    target_overrides: Dict[str, float],
    sign_overrides: Dict[str, float],
    default_target: float,
    decimals: int,
) -> Dict:
    patch = {
        "a6_y_zero_calibration": {
            "formula": "new_offset = target_position - new_sign * old_sign * (observed_position - old_offset)",
            "joints": {},
        }
    }
    output_joints = patch["a6_y_zero_calibration"]["joints"]
    for joint in EXPECTED_JOINTS:
        if joint not in joints or joint not in observations:
            continue
        entry = joints[joint]
        old_sign = float(entry.get("sign", 1.0))
        old_offset = float(entry.get("offset", 0.0))
        new_sign = float(sign_overrides.get(joint, old_sign))
        target = float(target_overrides.get(joint, default_target))
        observed = observations[joint].position
        new_offset = target - new_sign * old_sign * (observed - old_offset)
        output_joints[joint] = {
            "sign": rounded(new_sign, decimals),
            "offset": rounded(new_offset, decimals),
            "target_position": rounded(target, decimals),
            "observed_position": rounded(observed, decimals),
            "previous_sign": rounded(old_sign, decimals),
            "previous_offset": rounded(old_offset, decimals),
        }
    return patch


def print_table(patch: Dict) -> None:
    rows = patch["a6_y_zero_calibration"]["joints"]
    print(
        "joint".ljust(32)
        + "obs".rjust(10)
        + "target".rjust(10)
        + "old_s".rjust(8)
        + "new_s".rjust(8)
        + "old_off".rjust(12)
        + "new_off".rjust(12)
    )
    print("-" * 92)
    for joint, values in rows.items():
        print(
            joint.ljust(32)
            + f"{values['observed_position']:10.6f}"
            + f"{values['target_position']:10.6f}"
            + f"{values['previous_sign']:8.1f}"
            + f"{values['sign']:8.1f}"
            + f"{values['previous_offset']:12.6f}"
            + f"{values['offset']:12.6f}"
        )


def main() -> int:
    args = parse_args()
    try:
        joints = load_motor_config(args.config)
    except Exception as exc:
        print(f"ERROR: failed to load {args.config}: {exc}", file=sys.stderr)
        return 2

    target_overrides = dict(args.target_joint)
    sign_overrides = dict(args.sign)
    for joint, sign in sign_overrides.items():
        if sign not in (-1.0, 1.0):
            print(f"ERROR: {joint}: --sign must be +1 or -1", file=sys.stderr)
            return 2

    try:
        observations = (
            build_offline_observations(args.position)
            if args.position
            else wait_for_observations(args.topic, args.timeout)
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    errors = validate_inputs(joints, observations, args.max_feedback_age, args.allow_missing, args.allow_fault)
    if errors:
        for error in errors:
            print("ERROR: " + error, file=sys.stderr)
        return 1

    patch = propose_patch(joints, observations, target_overrides, sign_overrides, args.target, args.decimals)
    print_table(patch)

    patch_text = yaml.safe_dump(patch, sort_keys=False, allow_unicode=True)
    if args.output is not None:
        args.output.write_text(patch_text, encoding="utf-8")
        print(f"\nwrote calibration patch: {args.output}")
    else:
        print("\nYAML patch:")
        print(patch_text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
