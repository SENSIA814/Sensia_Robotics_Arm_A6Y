#!/usr/bin/env python3
"""Conservative single-motor check tool for the bundled Damiao CANable2 driver."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path


def parse_int(value: str) -> int:
    return int(value, 0)


def find_driver_root() -> Path:
    source_root = Path(__file__).resolve().parents[1] / "Motor Drive"
    if source_root.exists():
        return source_root

    try:
        from ament_index_python.packages import get_package_share_directory

        share_root = Path(get_package_share_directory("a6_y_ros2_control")) / "Motor Drive"
        if share_root.exists():
            return share_root
    except Exception:
        pass

    raise RuntimeError("Cannot locate bundled 'Motor Drive' directory for dm_canable2.")


def main() -> int:
    sys.path.insert(0, str(find_driver_root()))

    from dm_canable2 import Control_Type, DM_Motor_Type, Motor, MotorControl

    try:
        import can  # noqa: F401
    except ModuleNotFoundError:
        print("ERROR: missing python-can module required by single_motor_check.py.", file=sys.stderr)
        print("Install it with: sudo apt install python3-can", file=sys.stderr)
        print("Or, if apt is unavailable: python3 -m pip install --user python-can", file=sys.stderr)
        return 2

    parser = argparse.ArgumentParser(description="Check one Damiao motor through CANable2/SLCAN.")
    parser.add_argument("--channel", "--port", dest="channel", default="/dev/ttyACM0")
    parser.add_argument("--can-bitrate", type=int, default=1000000)
    parser.add_argument("--tty-baudrate", type=int, default=115200)
    parser.add_argument("--motor-type", choices=list(DM_Motor_Type.__members__.keys()), default="DM4340P")
    parser.add_argument("--motor-id", type=parse_int, required=True)
    parser.add_argument("--master-id", type=parse_int, default=0x11)
    parser.add_argument("--mode", choices=["status", "disable", "pos_vel", "pos_vel_csp", "vel", "mit"], default="status")
    parser.add_argument("--enable", action="store_true", help="Actually enable the motor before command tests.")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--amplitude", type=float, default=0.15)
    parser.add_argument(
        "--center-position",
        type=float,
        default=None,
        help="Center position for pos_vel/mit sine tests. Defaults to current motor feedback.",
    )
    parser.add_argument("--velocity", type=float, default=0.2)
    parser.add_argument("--kp", type=float, default=10.0)
    parser.add_argument("--kd", type=float, default=0.3)
    args = parser.parse_args()

    motor = Motor(DM_Motor_Type[args.motor_type], args.motor_id, args.master_id)
    mode_map = {
        "status": None,
        "pos_vel": Control_Type.POS_VEL,
        "pos_vel_csp": Control_Type.POS_VEL_CSP,
        "vel": Control_Type.VEL,
        "mit": Control_Type.MIT,
    }

    with MotorControl(
        channel=args.channel,
        can_bitrate=args.can_bitrate,
        tty_baudrate=args.tty_baudrate,
    ) as control:
        control.addMotor(motor)

        if args.mode == "status":
            for _ in range(10):
                control.refresh_motor_status(motor)
                print(
                    f"q={motor.getPosition(): .4f} dq={motor.getVelocity(): .4f} "
                    f"tau={motor.getTorque(): .4f} err=0x{motor.getError():X}"
                )
                time.sleep(0.05)
            return 0

        if args.mode == "disable":
            control.disable(motor)
            time.sleep(0.05)
            control.refresh_motor_status(motor)
            print(
                f"disable sent: q={motor.getPosition(): .4f} dq={motor.getVelocity(): .4f} "
                f"tau={motor.getTorque(): .4f} err=0x{motor.getError():X}"
            )
            return 0

        if not args.enable:
            print("Refusing to command motion without --enable. Use mode=status for read-only checks.")
            return 2

        center_position = args.center_position
        if center_position is None and args.mode in ("pos_vel", "pos_vel_csp", "mit"):
            for _ in range(3):
                control.refresh_motor_status(motor)
                time.sleep(0.02)
            center_position = motor.getPosition()
            print(f"using current feedback as motion center: {center_position:.4f} rad")
        elif center_position is not None and args.mode in ("pos_vel", "pos_vel_csp", "mit"):
            print(f"using requested motion center: {center_position:.4f} rad")

        control_mode = mode_map[args.mode]
        if control.switchControlMode(motor, control_mode):
            print(f"control mode set to {control_mode.name}")
        else:
            print("control mode write was not confirmed")

        control.enable(motor)
        print("motor enabled")
        start = time.time()
        try:
            while time.time() - start < args.duration:
                elapsed = time.time() - start
                if args.mode == "pos_vel":
                    q = center_position + args.amplitude * math.sin(
                        2.0 * math.pi * elapsed / max(args.duration, 0.1)
                    )
                    control.control_Pos_Vel(motor, q, abs(args.velocity))
                elif args.mode == "pos_vel_csp":
                    q = center_position + args.amplitude * math.sin(
                        2.0 * math.pi * elapsed / max(args.duration, 0.1)
                    )
                    control.control_Pos_Vel_CSP(motor, q, abs(args.velocity))
                elif args.mode == "vel":
                    control.control_Vel(motor, args.velocity)
                elif args.mode == "mit":
                    q = center_position + args.amplitude * math.sin(
                        2.0 * math.pi * elapsed / max(args.duration, 0.1)
                    )
                    control.controlMIT(motor, args.kp, args.kd, q, 0.0, 0.0)

                print(
                    f"[{elapsed:5.2f}s] q={motor.getPosition(): .4f} dq={motor.getVelocity(): .4f} "
                    f"tau={motor.getTorque(): .4f} err=0x{motor.getError():X}"
                )
                time.sleep(0.02)
        finally:
            control.disable(motor)
            print("motor disabled")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
