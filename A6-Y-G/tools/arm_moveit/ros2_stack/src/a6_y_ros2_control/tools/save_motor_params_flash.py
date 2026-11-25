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


DEFAULT_VERIFY_PARAMS = ["KP_ASR", "KI_ASR", "KP_APR", "KI_APR", "ACC", "DEC"]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Save current Damiao motor RAM parameters to flash. This makes temporary writes persistent."
    )
    parser.add_argument("--channel", "--port", dest="channel", default="/dev/ttyACM0")
    parser.add_argument("--can-bitrate", type=int, default=1000000)
    parser.add_argument("--tty-baudrate", type=int, default=115200)
    parser.add_argument("--receive-timeout", type=float, default=0.05)
    parser.add_argument("--leader", action="store_true", help="Use the default leader motor config.")
    parser.add_argument("--config", type=Path, help="Motor yaml to read targets from.")
    parser.add_argument("--joints", nargs="+", help="Joint names from the yaml, for example joint1 joint2 joint3.")
    parser.add_argument("--all", action="store_true", help="Save all joints from the yaml.")
    parser.add_argument("--ids", help="Comma-separated IDs or ranges, for example 0x01-0x03.")
    parser.add_argument("--motor-type", default="DM4340P", help="Motor type used with --ids.")
    parser.add_argument(
        "--params",
        default=",".join(DEFAULT_VERIFY_PARAMS),
        help="Parameters to print before/after saving. This does not choose what gets saved; flash save persists all current motor parameters.",
    )
    parser.add_argument("--save-code", type=lambda value: parse_int(value), default=0x00)
    parser.add_argument("--skip-readback", action="store_true", help="Do not read parameters before/after save.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Actually send flash-save frames.")
    args = parser.parse_args()

    if sum(bool(value) for value in (args.all, args.ids, args.joints)) != 1:
        print("choose exactly one target selector: --joints, --all, or --ids", file=sys.stderr)
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

    sys.path.insert(0, str(find_driver_root()))
    from dm_canable2 import DM_Motor_Type, DM_variable, Motor, MotorControl

    try:
        verify_rids = [] if args.skip_readback else parse_param_names(args.params, DM_variable)
    except Exception as exc:
        print(f"failed to parse --params: {exc}", file=sys.stderr)
        return 2

    print("flash-save targets:")
    print("joint,motor_type,slave_id,master_id")
    for target in targets:
        master = f"0x{target.master_id:02X}" if target.master_id else "-"
        print(f"{target.name},{target.motor_type},0x{target.slave_id:02X},{master}")

    print("\nwarning:")
    print("flash_save=true")
    print("selected motors will be disabled by the save command")
    print("do not power off while saving")
    if verify_rids:
        print("\nreadback params:")
        print(",".join(param_name(rid, DM_variable) for rid in verify_rids))

    if args.dry_run:
        return 0
    if not args.yes:
        print("\nadd --yes to save current motor RAM parameters to flash", file=sys.stderr)
        return 2

    try:
        motors = [
            (target, Motor(DM_Motor_Type[target.motor_type], target.slave_id, target.master_id))
            for target in targets
        ]
    except KeyError as exc:
        print(f"unknown motor type: {exc}", file=sys.stderr)
        return 2

    rows: list[tuple[str, str, int, dict[int, Any], dict[int, Any]]] = []
    with MotorControl(
        channel=args.channel,
        can_bitrate=args.can_bitrate,
        tty_baudrate=args.tty_baudrate,
        receive_timeout=args.receive_timeout,
    ) as control:
        for _, motor in motors:
            control.addMotor(motor)
        for target, motor in motors:
            before = {rid: control.read_motor_param(motor, rid) for rid in verify_rids}
            control.save_motor_param(motor, args.save_code)
            after = {rid: control.read_motor_param(motor, rid) for rid in verify_rids}
            rows.append((target.name, target.motor_type, target.slave_id, before, after))

    print("\nsave result:")
    if verify_rids:
        header = ["joint", "motor_type", "slave_id"]
        header.extend(f"before_{param_name(rid, DM_variable)}" for rid in verify_rids)
        header.extend(f"after_{param_name(rid, DM_variable)}" for rid in verify_rids)
        print(",".join(header))
        failed_readback = False
        for joint_name, motor_type, slave_id, before, after in rows:
            failed_readback = failed_readback or any(before.get(rid) is None or after.get(rid) is None for rid in verify_rids)
            values = [format_value(before.get(rid)) for rid in verify_rids]
            values.extend(format_value(after.get(rid)) for rid in verify_rids)
            print(f"{joint_name},{motor_type},0x{slave_id:02X}," + ",".join(values))
        if failed_readback:
            print("\nsave frame was sent, but some readbacks failed; verify after power cycle before trusting flash.", file=sys.stderr)
            return 1
    else:
        print("joint,motor_type,slave_id,status")
        for joint_name, motor_type, slave_id, _, _ in rows:
            print(f"{joint_name},{motor_type},0x{slave_id:02X},SAVE_SENT")

    print("\nflash save frames sent. Power-cycle and read parameters again to verify persistence.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
