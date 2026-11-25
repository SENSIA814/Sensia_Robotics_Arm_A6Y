#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


DEFAULT_JOINTS = ["joint1", "joint2", "joint3"]
DEFAULT_PARAMS = ["KP_ASR", "KI_ASR", "KP_APR", "KI_APR"]


@dataclass
class ParamTarget:
    name: str
    motor_type: str
    slave_id: int
    master_id: int


def parse_int(value: Any) -> int:
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def parse_ids(text: str) -> list[int]:
    result: list[int] = []
    for part in text.split(","):
        token = part.strip()
        if not token:
            continue
        if "-" in token:
            start_text, end_text = token.split("-", 1)
            start = parse_int(start_text.strip())
            end = parse_int(end_text.strip())
            step = 1 if start <= end else -1
            result.extend(range(start, end + step, step))
        else:
            result.append(parse_int(token))
    return result


def find_driver_root() -> Path:
    source_root = Path(__file__).resolve().parents[1] / "Motor Drive"
    if source_root.exists():
        return source_root
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("a6_y_ros2_control")) / "Motor Drive"


def default_config_path(leader: bool) -> Path:
    filename = "a6_y_leader_damiao_motors.yaml" if leader else "a6_y_follower_damiao_motors.yaml"
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("A6_Y_Moveitconfig")) / "config" / filename
    except Exception:
        return Path(__file__).resolve().parents[2] / "A6_Y_Moveitconfig" / "config" / filename


def load_config_targets(config_path: Path, joints: list[str] | None) -> list[ParamTarget]:
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    configured_joints = data["a6_y_damiao_system"]["ros__parameters"]["joints"]

    if joints:
        missing = [name for name in joints if name not in configured_joints]
        if missing:
            raise KeyError(f"joint not found in {config_path}: {', '.join(missing)}")
        joint_names = joints
    else:
        joint_names = list(configured_joints.keys())

    targets: list[ParamTarget] = []
    for name in joint_names:
        entry = configured_joints[name]
        targets.append(
            ParamTarget(
                name=str(name),
                motor_type=str(entry["motor_type"]),
                slave_id=parse_int(entry["slave_id"]),
                master_id=parse_int(entry.get("master_id", 0)),
            )
        )
    return targets


def build_id_targets(ids: list[int], motor_type: str) -> list[ParamTarget]:
    return [ParamTarget(f"motor_0x{motor_id:02X}", motor_type, motor_id, 0) for motor_id in ids]


def parse_param_names(text: str, dm_variable) -> list[int]:
    params: list[int] = []
    for part in text.split(","):
        token = part.strip()
        if not token:
            continue
        if token in dm_variable.__members__:
            params.append(int(dm_variable[token]))
        else:
            params.append(parse_int(token))
    if not params:
        raise ValueError("no params selected")
    return params


def param_name(rid: int, dm_variable) -> str:
    try:
        return dm_variable(int(rid)).name
    except ValueError:
        return f"RID_{int(rid)}"


def format_value(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read Damiao internal motor parameters over CANable2/SLCAN."
    )
    parser.add_argument("--channel", "--port", dest="channel", default="/dev/ttyACM0")
    parser.add_argument("--can-bitrate", type=int, default=1000000)
    parser.add_argument("--tty-baudrate", type=int, default=115200)
    parser.add_argument("--receive-timeout", type=float, default=0.05)
    parser.add_argument("--leader", action="store_true", help="Use the default leader motor config.")
    parser.add_argument("--config", type=Path, help="Motor yaml to read targets from.")
    parser.add_argument(
        "--joints",
        nargs="*",
        default=DEFAULT_JOINTS,
        help="Joint names from the yaml. Default: joint1 joint2 joint3.",
    )
    parser.add_argument("--all", action="store_true", help="Read all joints from the yaml.")
    parser.add_argument("--ids", help="Comma-separated IDs or ranges, for example 0x01-0x03.")
    parser.add_argument("--motor-type", default="DM4340P", help="Motor type used with --ids.")
    parser.add_argument(
        "--params",
        default=",".join(DEFAULT_PARAMS),
        help="Comma-separated DM_variable names or register IDs.",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    config_path = args.config if args.config else default_config_path(args.leader)
    joints = None if args.all else list(args.joints)

    try:
        targets = (
            build_id_targets(parse_ids(args.ids), args.motor_type)
            if args.ids
            else load_config_targets(config_path, joints)
        )
    except Exception as exc:
        print(f"failed to load targets: {exc}", file=sys.stderr)
        return 2

    if not targets:
        print("no targets", file=sys.stderr)
        return 2

    sys.path.insert(0, str(find_driver_root()))
    from dm_canable2 import DM_Motor_Type, DM_variable, Motor, MotorControl

    try:
        param_rids = parse_param_names(args.params, DM_variable)
    except Exception as exc:
        print(f"failed to parse params: {exc}", file=sys.stderr)
        return 2

    print("read targets:")
    print("joint,motor_type,slave_id,master_id")
    for target in targets:
        master = f"0x{target.master_id:02X}" if target.master_id else "-"
        print(f"{target.name},{target.motor_type},0x{target.slave_id:02X},{master}")
    print("\nparams:")
    print(",".join(param_name(rid, DM_variable) for rid in param_rids))

    if args.dry_run:
        return 0

    try:
        motors = [
            (target, Motor(DM_Motor_Type[target.motor_type], target.slave_id, target.master_id))
            for target in targets
        ]
    except KeyError as exc:
        print(f"unknown motor type: {exc}", file=sys.stderr)
        return 2

    rows: list[tuple[ParamTarget, dict[int, Any]]] = []
    with MotorControl(
        channel=args.channel,
        can_bitrate=args.can_bitrate,
        tty_baudrate=args.tty_baudrate,
        receive_timeout=args.receive_timeout,
    ) as control:
        for _, motor in motors:
            control.addMotor(motor)
        for target, motor in motors:
            values: dict[int, Any] = {}
            for rid in param_rids:
                values[rid] = control.read_motor_param(motor, rid)
            rows.append((target, values))

    header = ["joint", "motor_type", "slave_id"] + [param_name(rid, DM_variable) for rid in param_rids]
    print("\nread result:")
    print(",".join(header))
    missing = False
    for target, values in rows:
        formatted = [format_value(values.get(rid)) for rid in param_rids]
        missing = missing or any(values.get(rid) is None for rid in param_rids)
        print(f"{target.name},{target.motor_type},0x{target.slave_id:02X}," + ",".join(formatted))

    if missing:
        print("\nsome parameters were not read; check channel, permission, bus occupancy, and motor power.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
