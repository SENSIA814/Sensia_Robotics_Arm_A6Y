#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass
class ScanTarget:
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


def default_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("A6_Y_Moveitconfig")) / "config" / "a6_y_damiao_motors.yaml"
    except Exception:
        return Path(__file__).resolve().parents[2] / "A6_Y_Moveitconfig" / "config" / "a6_y_damiao_motors.yaml"


def load_targets(config_path: Path) -> list[ScanTarget]:
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    joints = data["a6_y_damiao_system"]["ros__parameters"]["joints"]
    return [
        ScanTarget(str(name), str(entry["motor_type"]), parse_int(entry["slave_id"]), parse_int(entry.get("master_id", 0)))
        for name, entry in joints.items()
    ]


def build_id_targets(ids: list[int], motor_type: str) -> list[ScanTarget]:
    return [ScanTarget(f"motor_0x{motor_id:02X}", motor_type, motor_id, 0) for motor_id in ids]


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only Damiao CANable2/SLCAN bus scan for A6_Y.")
    parser.add_argument("--channel", "--port", dest="channel", default="/dev/ttyACM0")
    parser.add_argument("--can-bitrate", type=int, default=1000000)
    parser.add_argument("--tty-baudrate", type=int, default=115200)
    parser.add_argument("--receive-timeout", type=float, default=0.05)
    parser.add_argument("--passes", type=int, default=3)
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--ids", help="Comma-separated IDs or ranges, for example 0x01-0x06.")
    parser.add_argument("--motor-type", default="DM4340", help="Motor type used with --ids.")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    targets = build_id_targets(parse_ids(args.ids), args.motor_type) if args.ids else load_targets(args.config)
    if not targets:
        print("no scan targets", file=sys.stderr)
        return 2
    print("scan targets:")
    print("joint,motor_type,slave_id,master_id")
    for target in targets:
        master = f"0x{target.master_id:02X}" if target.master_id else "-"
        print(f"{target.name},{target.motor_type},0x{target.slave_id:02X},{master}")
    if args.dry_run:
        return 0

    sys.path.insert(0, str(find_driver_root()))
    from dm_canable2 import DM_Motor_Type, Motor, MotorControl
    motors = [(target, Motor(DM_Motor_Type[target.motor_type], target.slave_id, target.master_id)) for target in targets]
    with MotorControl(channel=args.channel, can_bitrate=args.can_bitrate, tty_baudrate=args.tty_baudrate, receive_timeout=args.receive_timeout) as control:
        for _, motor in motors:
            control.addMotor(motor)
        seen_ids: set[int] = set()
        hit_counts = {target.slave_id: 0 for target, _ in motors}
        for pass_index in range(max(args.passes, 1)):
            for _, motor in motors:
                before = motor.getTimeInterval()
                control.refresh_motor_status(motor)
                control.drain_rx(timeout=0.0, limit=16)
                if motor.getTimeInterval() != before:
                    seen_ids.add(motor.SlaveID)
                    hit_counts[motor.SlaveID] = hit_counts.get(motor.SlaveID, 0) + 1
            if pass_index + 1 < max(args.passes, 1):
                time.sleep(0.02)
    print("\nscan result:")
    print("status,joint,motor_type,slave_id,hits,passes,q,dq,tau,err")
    passes = max(args.passes, 1)
    for target, motor in motors:
        found = target.slave_id in seen_ids
        status = "FOUND" if found else "MISS"
        print(f"{status},{target.name},{target.motor_type},0x{target.slave_id:02X},{hit_counts.get(target.slave_id, 0)},{passes},{motor.getPosition():.6f},{motor.getVelocity():.6f},{motor.getTorque():.6f},0x{motor.getError():X}")
    missing = [target for target in targets if target.slave_id not in seen_ids]
    if missing:
        print("\nmissing ids: " + ", ".join(f"0x{target.slave_id:02X}" for target in missing), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
