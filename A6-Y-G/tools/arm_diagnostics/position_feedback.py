"""Decode fresh status positions using previously measured, serial-bound PMAX.

Loading this file or its configuration never connects to hardware. Only range
parameters are cached; positions must always come from the current scan.
"""
import math
from pathlib import Path

import yaml


DEFAULT_SCALES = Path(__file__).with_name("motor_position_limits.yaml")


def load_position_scales(path=DEFAULT_SCALES):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("位置量程配置格式无效")
    for key in ("source", "measured_at"):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f"位置量程配置缺少 {key}")
    if not isinstance(data.get("adapters"), dict):
        raise ValueError("位置量程配置缺少 adapters 映射")
    scales = {}
    for serial, motors in data["adapters"].items():
        if not isinstance(serial, str) or not isinstance(motors, dict):
            raise ValueError("位置量程必须按 USB 序列号和电机 ID 配置")
        scales[serial] = {}
        for motor_id, pmax in motors.items():
            if type(motor_id) is not int or motor_id not in range(1, 8):
                raise ValueError("位置量程的电机 ID 必须为 1～7")
            if type(pmax) not in (int, float) or not math.isfinite(pmax) or pmax <= 0:
                raise ValueError(f"{serial} ID {motor_id}: PMAX 必须为有限正数")
            scales[serial][motor_id] = {"pmax_rad": float(pmax),
                                       "measured_at": data["measured_at"],
                                       "source": data["source"]}
    return scales


def position_fields(raw, scale):
    if scale is None:
        return {"position_rad": None, "position_deg": None, "position_pmax_rad": None}
    pmax = scale["pmax_rad"]
    radians = raw / 65535.0 * (2.0 * pmax) - pmax
    return {"position_rad": radians, "position_deg": math.degrees(radians),
            "position_pmax_rad": pmax}


def position_text(last):
    if not last:
        return "—"
    if last.get("position_rad") is None:
        return f"原始计数 {last['position_raw']}（未配置量程）"
    return f"{last['position_deg']:.2f}° / {last['position_rad']:.4f} rad（原始计数 {last['position_raw']}）"
