#!/usr/bin/env python3
"""Role-selected seven-motor diagnostics using the existing CANable2 driver.

The only permitted motor command is CAN 0x7FF, <ID> 00 CC 00 00 00 00 00.
This module never enables/disables a motor, reads parameters, or sends motion.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack, contextmanager
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import threading
import time
from zoneinfo import ZoneInfo

import yaml
from usb_identity import connection_inventory, verify_usb_identity
from position_feedback import load_position_scales, position_fields, position_text
from operation_lock import exclusive_operation

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ROLES = ROOT.parent / "diagnostics/arm_roles.yaml"
DEFAULT_REPORTS = ROOT.parent / "diagnostics/arm_scan_reports"
ROLE_ORDER = ("motion_left", "motion_right", "teaching_left", "teaching_right")
IDS = tuple(range(1, 8))
sys.path.insert(0, str(ROOT / "src/a6_y_ros2_control/tools"))
from damiao_bus_scan import find_driver_root
sys.path.insert(0, str(find_driver_root()))
from dm_canable2 import MotorControl


def timestamp():
    return datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(timespec="milliseconds")


def load_roles(path=DEFAULT_ROLES):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("roles"), dict):
        raise ValueError("角色配置缺少 roles 映射")
    roles = {}
    for role in ROLE_ORDER:
        entry = data["roles"][role]
        serial = str(entry["usb_serial"])
        port = Path(entry["by_id"])
        # Never silently fall back to last_device (/dev/ttyACM numbers change).
        if not port.is_absolute() or not port.name.endswith(serial + "-if00"):
            raise ValueError(f"{role}: USB 序列号与 by_id 路径不一致")
        roles[role] = {"role": role, "name": str(entry["display_name"]),
                       "usb_serial": serial, "port": str(port)}
    if len({r["usb_serial"] for r in roles.values()}) != len(ROLE_ORDER):
        raise ValueError("多个机械臂不能绑定同一 USB 序列号")
    return roles


def validate_selection(selected, roles):
    if (not isinstance(selected, list) or not 1 <= len(selected) <= 4
            or any(not isinstance(r, str) or r not in roles for r in selected)
            or len(set(selected)) != len(selected)):
        raise ValueError("请明确选择 1～4 台不同的机械臂")
    return list(selected)


def validate_options(passes, timeout):
    if type(passes) is not int or not 1 <= passes <= 30:
        raise ValueError("扫描轮数必须为 1～30 的整数")
    if isinstance(timeout, bool) or not isinstance(timeout, (float, int)) or not math.isfinite(timeout) or not .02 <= timeout <= .5:
        raise ValueError("单次响应等待必须在 0.02～0.5 秒之间")


def inventory(roles):
    return connection_inventory(roles)


def status_payload(motor_id):
    if type(motor_id) is not int or motor_id not in IDS:
        raise ValueError("Only motor IDs 1–7 are allowed")
    return bytes([motor_id, 0, 0xCC, 0, 0, 0, 0, 0])


class StatusOnlyControl(MotorControl):
    """Allowlist at the last driver entry point before a CAN transmission."""
    def __init__(self, *args, tx=None, **kwargs):
        self.tx = tx if tx is not None else []
        super().__init__(*args, **kwargs)

    def send_std(self, arbitration_id, payload, *, pad_to_8=True):
        data = bytes(payload)
        if arbitration_id != 0x7FF or data not in {status_payload(i) for i in IDS}:
            raise RuntimeError("已拦截非状态查询 CAN 指令")
        super().send_std(arbitration_id, data, pad_to_8=pad_to_8)
        self.tx.append({"can_id": arbitration_id, "data": data.hex(), "at": timestamp()})


def decode_status(msg, motor_id, position_scale=None):
    data = bytes(msg.data)
    if (msg.arbitration_id != motor_id + 16 or len(data) != 8
            or getattr(msg, "dlc", len(data)) != 8
            or any(getattr(msg, flag, False) for flag in
                   ("is_extended_id", "is_remote_frame", "is_error_frame", "is_fd"))
            or (data[0] & 15) != motor_id):
        return None
    # Decode the fresh frame with this adapter/motor's measured PMAX. Never use
    # another arm's range or a model default when the scale is unavailable.
    position_raw = data[1] << 8 | data[2]
    return {"state": data[0] >> 4, "mos_temperature_c": data[6],
            "rotor_temperature_c": data[7], "position_raw": position_raw,
            **position_fields(position_raw, position_scale),
            "velocity_raw": data[3] << 4 | data[4] >> 4,
            "torque_raw": (data[4] & 15) << 8 | data[5], "raw_hex": data.hex(),
            "feedback_can_id": msg.arbitration_id, "received_at": timestamp()}


def empty_motor(i):
    return {"id": i, "joint": f"J{i}" if i < 7 else "夹爪", "feedback_can_id": i + 16,
            "attempts": 0, "hits": 0, "frames": 0, "states": [], "last": None,
            "result": "pending"}


def summarize(arm, passes):
    for motor in arm["motors"]:
        if not motor["hits"]:
            motor["result"] = "missing" if motor["attempts"] else "not_scanned"
        elif any(state != 0 for state in motor["states"]):
            motor["result"] = "state_warning"
        elif motor["hits"] != passes or motor["attempts"] != passes:
            motor["result"] = "intermittent"
        else:
            motor["result"] = "ok"
    arm["responding"] = sum(m["hits"] > 0 for m in arm["motors"])
    arm["missing_ids"] = [m["id"] for m in arm["motors"] if not m["hits"]]
    arm["communication_ok"] = all(m["hits"] == passes and m["attempts"] == passes for m in arm["motors"])
    arm["ok"] = (all(m["result"] == "ok" for m in arm["motors"])
                 and not arm["error"] and not arm["cancelled"] and not arm["unexpected_frames"])


@contextmanager
def open_arm_control(arm, control_type):
    """Keep identity verification, ownership checks and the device lock shared."""
    port = Path(arm["port"])
    verify_usb_identity(arm)
    if not port.exists():
        raise FileNotFoundError(f"未检测到 USB 适配器 {arm['usb_serial']}，请检查 USB 连接")
    if not stat.S_ISCHR(port.stat().st_mode):
        raise ValueError("配置路径不是串口字符设备")
    fuser = shutil.which("fuser")
    if not fuser:
        raise RuntimeError("缺少 fuser，无法检查串口占用（需要 psmisc）")
    busy = subprocess.run([fuser, str(port)], capture_output=True, timeout=3)
    if busy.returncode == 0:
        raise RuntimeError("串口被占用，请先停止使用该机械臂的 launch 或扫描程序")
    if busy.returncode != 1:
        raise RuntimeError("串口占用检查失败")
    with ExitStack() as stack:
        # A GUI process may have no controlling terminal. Do not accidentally
        # acquire the serial device as one (disconnect could deliver SIGHUP).
        fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
        device = stack.enter_context(os.fdopen(fd, "rb", buffering=0))
        try:
            fcntl.flock(device.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("串口被锁定，请先停止该机械臂的其他程序") from None
        control = control_type(channel=str(port), auto_open=False, tx=arm["tx"])
        stack.callback(control.close)
        verify_usb_identity(arm)
        control.open()
        arm["device"] = str(port.resolve())
        yield control


def scan_arm(arm, passes, timeout, cancel, emit):
    try:
        if cancel.is_set():
            arm["cancelled"] = True
            return
        with open_arm_control(arm, StatusOnlyControl) as control:
            for round_index in range(passes):
                for motor in arm["motors"]:
                    if cancel.is_set():
                        arm["cancelled"] = True
                        return
                    # Discard buffered frames; only count replies after this request.
                    for _ in range(512):
                        if control.bus.recv(timeout=0) is None:
                            break
                    else:
                        raise RuntimeError("接收队列持续繁忙，请停止其他总线通信")
                    control.send_std(0x7FF, status_payload(motor["id"]))
                    motor["attempts"] += 1
                    deadline = time.monotonic() + timeout
                    hit = False
                    while time.monotonic() < deadline and not cancel.is_set():
                        msg = control.bus.recv(timeout=min(.01, max(0, deadline - time.monotonic())))
                        if msg is None:
                            continue
                        feedback = decode_status(msg, motor["id"], motor.get("position_scale"))
                        if feedback is None:
                            arm["unexpected_frames"] += 1
                            continue
                        motor["frames"] += 1
                        motor["last"] = feedback
                        motor["states"] = sorted(set(motor["states"]) | {feedback["state"]})
                        if not hit:
                            motor["hits"] += 1
                            hit = True
                    if cancel.is_set():
                        arm["cancelled"] = True
                    summarize(arm, passes)
                    emit({"event": "progress", "role": arm["role"], "round": round_index + 1,
                          "motor_id": motor["id"], "motor": dict(motor)})
                if cancel.wait(.02):
                    arm["cancelled"] = True
                    return
    except Exception as exc:
        arm["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        summarize(arm, passes)
        emit({"event": "arm_done", "arm": arm})


def scan_selected(selected, roles, passes=10, timeout=.06, cancel=None, emit=None):
    with exclusive_operation():
        return _scan_selected(selected, roles, passes, timeout, cancel, emit)


def _scan_selected(selected, roles, passes, timeout, cancel, emit):
    selected = validate_selection(selected, roles)
    validate_options(passes, timeout)
    cancel = cancel if cancel is not None else threading.Event()
    emit = emit or (lambda event: None)
    scale_error = None
    try:
        scales = load_position_scales()
    except (OSError, ValueError, yaml.YAMLError) as exc:
        # Status/raw feedback is still useful when a range configuration is
        # unavailable. Do not fabricate an angle or change the query protocol.
        scales = {}
        scale_error = f"位置量程不可用，仅保留原始计数：{exc}"
    report = {"schema_version": 1, "operation": "scan", "started_at": timestamp(), "status_requests_only": True,
              "selected_roles": selected, "passes": passes, "timeout_s": timeout,
              "can_bitrate": 1000000, "tty_baudrate": 115200,
              "position_scale_error": scale_error,
              "position_note": "电机轴角度，使用之前读取的 PMAX；每台电机独立采样，结束后保留最后一次反馈。未作关节零位/方向校准，夹爪角度不是开口宽度。",
              "note": "仅确认本轮通信；不能证明所有接线、供电或机械条件完好，也不能排除重复 ID。",
              "arms": [{**roles[role], "motors": [empty_motor(i) for i in IDS], "tx": [],
                        "error": None, "cancelled": False, "unexpected_frames": 0,
                        "device": None} for role in selected]}
    for arm in report["arms"]:
        for motor in arm["motors"]:
            motor["position_scale"] = scales.get(arm["usb_serial"], {}).get(motor["id"])
        emit({"event": "arm_start", "role": arm["role"], "name": arm["name"]})
        scan_arm(arm, passes, timeout, cancel, emit)
    report.update(finished_at=timestamp(), cancelled=cancel.is_set(),
                  ok=all(arm["ok"] for arm in report["arms"]) and not cancel.is_set())
    return report


def save_report(report, directory=DEFAULT_REPORTS):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
    prefix = "arm_zero" if report.get("operation") == "zero" else "arm_scan"
    path = directory / f"{prefix}_{stamp}.json"
    with path.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return path


def main():
    parser = argparse.ArgumentParser(description="按机械臂角色扫描七电机，仅发送状态查询")
    parser.add_argument("--arms", nargs="+", required=True, choices=[*ROLE_ORDER, "all"])
    parser.add_argument("--roles", type=Path, default=DEFAULT_ROLES)
    parser.add_argument("--passes", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=.06)
    parser.add_argument("--report-dir", type=Path, default=DEFAULT_REPORTS)
    parser.add_argument("--dry-run", action="store_true", help="只列出目标，不打开串口")
    args = parser.parse_args()
    try:
        roles = load_roles(args.roles)
        selected = list(ROLE_ORDER) if args.arms == ["all"] else args.arms
        validate_selection(selected, roles)
        validate_options(args.passes, args.timeout)
    except (ValueError, KeyError, OSError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    for role in selected:
        print(f"{roles[role]['name']} [{role}]  USB={roles[role]['usb_serial']}")
    if args.dry_run:
        print("DRY RUN：没有打开串口；本次目标为每臂 ID 1～7。")
        return 0
    cancel = threading.Event()
    import signal
    signal.signal(signal.SIGINT, lambda *_: cancel.set())
    signal.signal(signal.SIGTERM, lambda *_: cancel.set())
    try:
        report = scan_selected(selected, roles, args.passes, args.timeout, cancel,
                               lambda e: print(f"正在扫描：{e['name']}", flush=True) if e["event"] == "arm_start" else None)
    except (RuntimeError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(f"\n位置说明：{report['position_note']}")
    if report["position_scale_error"]:
        print(report["position_scale_error"])
    for arm in report["arms"]:
        print(f"\n{arm['name']}：{arm['responding']}/7 有响应；" + ("本轮通信检查通过" if arm["ok"] else "需检查"))
        if arm["error"]:
            print(arm["error"])
        for motor in arm["motors"]:
            print(f"  ID {motor['id']} {motor['joint']:<3} {motor['result']:<14} "
                  f"响应 {motor['hits']}/{args.passes} 状态 {motor['states']} "
                  f"位置 {position_text(motor['last'])}"
                  + (f" 采样 {motor['last']['received_at']}" if motor['last'] else ""))
    try:
        print(f"\n报告：{save_report(report, args.report_dir)}")
    except OSError as exc:
        print(f"报告保存失败：{exc}", file=sys.stderr)
        return 2
    return 130 if report["cancelled"] else (0 if report["ok"] else 1)


if __name__ == "__main__":
    raise SystemExit(main())
