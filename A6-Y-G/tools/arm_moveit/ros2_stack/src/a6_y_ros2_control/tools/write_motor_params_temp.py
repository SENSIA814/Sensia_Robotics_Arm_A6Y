#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

from read_motor_params import (
    build_id_targets,
    default_config_path,
    find_driver_root,
    format_value,
    load_config_targets,
    param_name,
    parse_ids,
    parse_int,
    parse_param_names,
)


UINT_PARAM_RIDS = set(range(7, 11)) | set(range(13, 17)) | {35, 36}
DEFAULT_ALLOWED_PARAMS = {
    "ACC",
    "DEC",
    "MAX_SPD",
    "KP_ASR",
    "KI_ASR",
    "KP_APR",
    "KI_APR",
}


def parse_assignment(text: str, dm_variable) -> tuple[int, Any]:
    if "=" not in text:
        raise ValueError(f"--set expects NAME=VALUE, got {text!r}")
    name_text, value_text = text.split("=", 1)
    name_text = name_text.strip()
    value_text = value_text.strip()
    if not name_text or not value_text:
        raise ValueError(f"--set expects NAME=VALUE, got {text!r}")

    rids = parse_param_names(name_text, dm_variable)
    if len(rids) != 1:
        raise ValueError(f"--set expects one parameter name, got {text!r}")
    rid = rids[0]
    value = parse_int(value_text) if rid in UINT_PARAM_RIDS else float(value_text)
    return rid, value


def allowed_param_rids(dm_variable) -> set[int]:
    return {int(dm_variable[name]) for name in DEFAULT_ALLOWED_PARAMS}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Temporarily write Damiao motor parameters over CANable2/SLCAN. This tool never saves to flash."
    )
    parser.add_argument("--channel", "--port", dest="channel", default="/dev/ttyACM0")
    parser.add_argument("--can-bitrate", type=int, default=1000000)
    parser.add_argument("--tty-baudrate", type=int, default=115200)
    parser.add_argument("--receive-timeout", type=float, default=0.05)
    parser.add_argument("--leader", action="store_true", help="Use the default leader motor config.")
    parser.add_argument("--config", type=Path, help="Motor yaml to read targets from.")
    parser.add_argument("--joints", nargs="+", help="Joint names from the yaml, for example joint1 joint2 joint3.")
    parser.add_argument("--all", action="store_true", help="Write all joints from the yaml.")
    parser.add_argument("--ids", help="Comma-separated IDs or ranges, for example 0x01-0x03.")
    parser.add_argument("--motor-type", default="DM4340P", help="Motor type used with --ids.")
    parser.add_argument(
        "--set",
        dest="sets",
        action="append",
        required=True,
        help="Temporary parameter assignment, for example --set KP_APR=75. Can be repeated.",
    )
    parser.add_argument("--allow-any-param", action="store_true", help="Allow writing registers outside the safe tuning set.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Actually send write frames.")
    args = parser.parse_args()

    if sum(bool(value) for value in (args.all, args.ids, args.joints)) != 1:
        print("choose exactly one target selector: --joints, --all, or --ids", file=sys.stderr)
        return 2

    sys.path.insert(0, str(find_driver_root()))
    from dm_canable2 import DM_Motor_Type, DM_variable, Motor, MotorControl

    try:
        writes = [parse_assignment(item, DM_variable) for item in args.sets]
    except Exception as exc:
        print(f"failed to parse --set: {exc}", file=sys.stderr)
        return 2

    if not args.allow_any_param:
        allowed = allowed_param_rids(DM_variable)
        blocked = [param_name(rid, DM_variable) for rid, _ in writes if rid not in allowed]
        if blocked:
            print(
                "refusing to write non-tuning parameter(s): "
                + ", ".join(blocked)
                + ". Use --allow-any-param only if you know the register is safe.",
                file=sys.stderr,
            )
            return 2

    config_path = args.config if args.config else default_config_path(args.leader)
    try:
        targets = (
            build_id_targets(parse_ids(args.ids), args.motor_type)
            if args.ids
            else load_config_targets(config_path, None if args.all else list(args.joints))
        )
    except Exception as exc:
        print(f"failed to load targets: {exc}", file=sys.stderr)
        return 2

    if not targets:
        print("no targets", file=sys.stderr)
        return 2

    print("temporary write targets:")
    print("joint,motor_type,slave_id,master_id")
    for target in targets:
        master = f"0x{target.master_id:02X}" if target.master_id else "-"
        print(f"{target.name},{target.motor_type},0x{target.slave_id:02X},{master}")

    print("\nwrites:")
    for rid, value in writes:
        print(f"{param_name(rid, DM_variable)}={format_value(value)}")
    print("\nflash_save=false")

    if args.dry_run:
        return 0
    if not args.yes:
        print("\nadd --yes to apply these temporary writes", file=sys.stderr)
        return 2

    try:
        motors = [
            (target, Motor(DM_Motor_Type[target.motor_type], target.slave_id, target.master_id))
            for target in targets
        ]
    except KeyError as exc:
        print(f"unknown motor type: {exc}", file=sys.stderr)
        return 2

    rows: list[tuple[str, str, int, str, Any, Any, bool]] = []
    with MotorControl(
        channel=args.channel,
        can_bitrate=args.can_bitrate,
        tty_baudrate=args.tty_baudrate,
        receive_timeout=args.receive_timeout,
    ) as control:
        for _, motor in motors:
            control.addMotor(motor)
        for target, motor in motors:
            for rid, value in writes:
                before = control.read_motor_param(motor, rid)
                ok = control.change_motor_param(motor, rid, value)
                after = control.read_motor_param(motor, rid)
                rows.append((target.name, target.motor_type, target.slave_id, param_name(rid, DM_variable), before, after, ok))

    print("\nwrite result:")
    print("joint,motor_type,slave_id,param,before,after,ack")
    failed = False
    for joint_name, motor_type, slave_id, name, before, after, ok in rows:
        failed = failed or not ok or after is None
        ack = "OK" if ok else "MISS"
        print(
            f"{joint_name},{motor_type},0x{slave_id:02X},{name},"
            f"{format_value(before)},{format_value(after)},{ack}"
        )

    if failed:
        print("\nsome writes were not acknowledged; check channel, permission, bus occupancy, and motor power.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
