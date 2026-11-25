#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml

EXPECTED_JOINTS = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
    'gripper_finger_joint1',
]
MOTOR_LIMITS = {
    "DM4310": {"max_velocity": 30.0},
    "DM3507": {"max_velocity": 50.0},
    "DM4340": {"max_velocity": 10.0},
    "DM4340P": {"max_velocity": 10.0},
}
REQUIRED_DEFAULTS = [
    "dry_run", "control_mode", "auto_enable", "set_control_mode_on_configure",
    "require_feedback_on_configure", "allow_partial_feedback", "disable_on_deactivate",
    "command_timeout", "max_start_delta", "receive_timeout", "feedback_timeout",
    "status_request_period", "initial_feedback_attempts", "mit_kp", "mit_kd",
]


def package_path(package: str, fallback: Path) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package))
    except Exception:
        return fallback


def default_config_path() -> Path:
    return package_path("A6_Y_Moveitconfig", Path(__file__).resolve().parents[2] / "A6_Y_Moveitconfig") / "config" / "a6_y_damiao_motors.yaml"


def default_xacro_file() -> Path:
    return package_path("A6_Y_Moveitconfig", Path(__file__).resolve().parents[2] / "A6_Y_Moveitconfig") / "config" / "y1_6.urdf.xacro"


def default_urdf_file() -> Path:
    return package_path("y1_6", Path(__file__).resolve().parents[2] / "A6_Y_description") / "urdf" / "y1.6.urdf"


def parse_int(value: Any) -> int:
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def parse_float(value: Any) -> float:
    return float(value)


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def urdf_limits(path: Path) -> dict[str, tuple[float, float]]:
    root = ET.parse(path).getroot()
    limits = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        limit = joint.find("limit")
        if name and limit is not None and "lower" in limit.attrib and "upper" in limit.attrib:
            limits[name] = (float(limit.attrib["lower"]), float(limit.attrib["upper"]))
    return limits


def validate_config(config_path: Path, urdf_path: Path) -> list[str]:
    errors: list[str] = []
    data = load_yaml(config_path)
    params = data.get("a6_y_damiao_system", {}).get("ros__parameters", {})
    can = params.get("can", {})
    defaults = params.get("defaults", {})
    joints = params.get("joints", {})
    if not isinstance(can, dict) or "channel" not in can:
        errors.append("can.channel is required")
    for field in REQUIRED_DEFAULTS:
        if field not in defaults:
            errors.append(f"defaults missing {field}")
    if defaults.get("control_mode") not in ("pos_vel", "pos_vel_csp", "mit"):
        errors.append("defaults.control_mode must be pos_vel, pos_vel_csp, or mit")
    if list(joints.keys()) != EXPECTED_JOINTS:
        errors.append(f"joint order must be {EXPECTED_JOINTS}, got {list(joints.keys())}")
    seen_ids: set[int] = set()
    limits = urdf_limits(urdf_path)
    for index, joint in enumerate(EXPECTED_JOINTS):
        entry = joints.get(joint)
        if not isinstance(entry, dict):
            errors.append(f"{joint} missing motor config")
            continue
        for field in ("motor_type", "slave_id", "master_id", "sign", "offset", "min_position", "max_position", "max_velocity"):
            if field not in entry:
                errors.append(f"{joint} missing {field}")
        if joint == "gripper_finger_joint1" and "position_scale" not in entry:
            errors.append(f"{joint} missing position_scale")
        if entry.get("motor_type") not in MOTOR_LIMITS:
            errors.append(f"{joint} motor_type is unsupported: {entry.get('motor_type')}")
        try:
            slave_id = parse_int(entry["slave_id"])
            if slave_id in seen_ids:
                errors.append(f"duplicate slave_id 0x{slave_id:02X}")
            seen_ids.add(slave_id)
            sign = parse_float(entry["sign"])
            if sign not in (-1.0, 1.0):
                errors.append(f"{joint} sign must be +1.0 or -1.0")
            soft_min = parse_float(entry["min_position"])
            soft_max = parse_float(entry["max_position"])
            max_velocity = parse_float(entry["max_velocity"])
            position_scale = parse_float(entry.get("position_scale", 1.0))
        except Exception as exc:
            errors.append(f"{joint} invalid numeric field: {exc}")
            continue
        if position_scale <= 0.0:
            errors.append(f"{joint} position_scale must be positive")
        if soft_min >= soft_max:
            errors.append(f"{joint} min_position must be smaller than max_position")
        if joint in limits:
            urdf_min, urdf_max = limits[joint]
            if soft_min < urdf_min - 1e-9 or soft_max > urdf_max + 1e-9:
                errors.append(f"{joint} soft limits [{soft_min}, {soft_max}] exceed URDF [{urdf_min}, {urdf_max}]")
        motor_type = str(entry.get("motor_type"))
        motor_velocity = max_velocity / position_scale
        if motor_type in MOTOR_LIMITS and motor_velocity > MOTOR_LIMITS[motor_type]["max_velocity"]:
            errors.append(f"{joint} max_velocity exceeds {motor_type} protocol limit")
    return errors


def validate_xacro(xacro_file: Path, config_path: Path) -> list[str]:
    initial_positions_file = xacro_file.parent / "initial_positions.yaml"
    command = [
        "xacro",
        str(xacro_file),
        f"initial_positions_file:={initial_positions_file.resolve()}",
        f"motor_config_file:={config_path.resolve()}",
        "hardware_plugin:=a6_y_ros2_control/A6YDamiaoSystem",
    ]
    try:
        result = subprocess.run(command, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except Exception as exc:
        return [f"xacro expansion failed: {exc}"]
    root = ET.fromstring(result.stdout)
    ros2_control = root.find("ros2_control")
    if ros2_control is None:
        return ["expanded xacro has no ros2_control tag"]
    plugin = ros2_control.find("hardware/plugin")
    errors = []
    if plugin is None or (plugin.text or "").strip() != "a6_y_ros2_control/A6YDamiaoSystem":
        errors.append("hardware plugin mismatch")
    joints = [joint.attrib.get("name") for joint in ros2_control.findall("joint")]
    if joints != EXPECTED_JOINTS:
        errors.append(f"expanded ros2_control joints mismatch: {joints}")
    required_commands = {"position", "velocity", "effort"}
    required_states = {"position", "velocity", "effort", "motor_error", "fault", "has_feedback", "feedback_age", "enabled"}
    for joint in ros2_control.findall("joint"):
        name = joint.attrib.get("name", "")
        commands = {item.attrib.get("name") for item in joint.findall("command_interface")}
        states = {item.attrib.get("name") for item in joint.findall("state_interface")}
        if not required_commands.issubset(commands):
            errors.append(f"{name} missing command interfaces")
        if not required_states.issubset(states):
            errors.append(f"{name} missing state interfaces")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--xacro-file", type=Path, default=default_xacro_file())
    parser.add_argument("--urdf-file", type=Path, default=default_urdf_file())
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    errors = []
    errors.extend(validate_config(args.config, args.urdf_file))
    errors.extend(validate_xacro(args.xacro_file, args.config))
    if errors:
        for error in errors:
            print("ERROR: " + error, file=sys.stderr)
        return 1
    if not args.quiet:
        print(
            "validation passed: A6_Y launch defaults, single CAN settings, 6 arm joints plus "
            "gripper_finger_joint1, supported motor types, valid soft limits, xacro hardware/state interfaces present"
        )
    else:
        print("validation passed: A6_Y config and xacro")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
