#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import time

from dm_canable2 import Control_Type, DM_Motor_Type, DM_variable, Motor, MotorControl


def parse_int(value: str) -> int:
    return int(value, 0)


def main() -> int:
    parser = argparse.ArgumentParser(description="DM3507 demo using the dm_canable2 package")
    parser.add_argument("--channel", "--port", dest="channel", default="/dev/ttyACM0")
    parser.add_argument("--motor-id", type=parse_int, default=0x01)
    parser.add_argument("--master-id", type=parse_int, default=0x11)
    parser.add_argument("--can-bitrate", type=int, default=1000000)
    parser.add_argument("--tty-baudrate", type=int, default=115200)
    parser.add_argument("--mode", choices=["mit", "pos_vel", "vel"], default="vel")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--velocity", type=float, default=5.0)
    parser.add_argument("--kp", type=float, default=30.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--tmax", type=float, default=None)
    args = parser.parse_args()

    motor = Motor(DM_Motor_Type.DM3507, args.motor_id, args.master_id)

    mode_map = {
        "mit": Control_Type.MIT,
        "pos_vel": Control_Type.POS_VEL,
        "vel": Control_Type.VEL,
    }
    control_mode = mode_map[args.mode]

    with MotorControl(
        channel=args.channel,
        can_bitrate=args.can_bitrate,
        tty_baudrate=args.tty_baudrate,
    ) as control:
        control.addMotor(motor)

        if args.tmax is not None:
            if control.change_motor_param(motor, DM_variable.TMAX, args.tmax):
                print(f"TMAX set to {args.tmax}")
            else:
                print("TMAX write was not confirmed")

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
                if args.mode == "mit":
                    q = 0.8 * math.sin(2.0 * math.pi * elapsed / 3.0)
                    control.controlMIT(motor, args.kp, args.kd, q, 0.0, 0.0)
                elif args.mode == "pos_vel":
                    q = 0.8 * math.sin(2.0 * math.pi * elapsed / 3.0)
                    control.control_Pos_Vel(motor, q, abs(args.velocity))
                else:
                    control.control_Vel(motor, args.velocity)

                print(
                    f"[{elapsed:5.2f}s] q={motor.getPosition():7.3f} "
                    f"dq={motor.getVelocity():7.3f} tau={motor.getTorque():7.3f} "
                    f"err=0x{motor.getError():X}"
                )
                time.sleep(0.02)
        finally:
            control.disable(motor)
            print("motor disabled")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
