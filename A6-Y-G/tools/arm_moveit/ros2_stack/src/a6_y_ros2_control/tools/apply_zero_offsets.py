#!/usr/bin/env python3
"""Apply a capture_zero_offsets.py patch to a6_y_damiao_motors.yaml."""

import argparse
import difflib
import math
import re
import shutil
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import yaml


EXPECTED_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def default_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("A6_Y_Moveitconfig")) / "config" / "a6_y_damiao_motors.yaml"
    except Exception:
        return Path(__file__).resolve().parents[2] / "A6_Y_Moveitconfig" / "config" / "a6_y_damiao_motors.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("patch", type=Path, help="YAML patch produced by capture_zero_offsets.py --output.")
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--write", action="store_true", help="Write changes. Without this, only print a diff.")
    parser.add_argument("--no-backup", action="store_true", help="Do not create CONFIG.bak when using --write.")
    parser.add_argument(
        "--ignore-previous",
        action="store_true",
        help="Do not reject if patch previous_sign/previous_offset differ from current config.",
    )
    parser.add_argument("--decimals", type=int, default=6)
    return parser.parse_args()


def finite_float(value) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(value)
    return result


def format_number(value: float, decimals: int) -> str:
    if abs(value) == 1.0:
        return "1.0" if value > 0.0 else "-1.0"
    text = f"{value:.{decimals}f}".rstrip("0").rstrip(".")
    if text == "-0":
        text = "0"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text


def load_motor_joints(config_path: Path) -> Dict:
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return data["a6_y_damiao_system"]["ros__parameters"]["joints"]


def load_patch(patch_path: Path) -> Dict[str, Dict[str, float]]:
    with patch_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    joints = data.get("a6_y_zero_calibration", {}).get("joints", {})
    if not isinstance(joints, dict) or not joints:
        raise ValueError("patch must contain a6_y_zero_calibration.joints")
    return joints


def validate_patch(joints_config: Dict, patch_joints: Dict, ignore_previous: bool) -> List[str]:
    errors: List[str] = []
    for joint, values in patch_joints.items():
        if joint not in EXPECTED_JOINTS:
            errors.append(f"{joint}: unexpected joint")
            continue
        if joint not in joints_config:
            errors.append(f"{joint}: missing from motor config")
            continue
        if not isinstance(values, dict):
            errors.append(f"{joint}: patch entry must be a mapping")
            continue
        try:
            sign = finite_float(values["sign"])
            offset = finite_float(values["offset"])
        except (KeyError, ValueError) as exc:
            errors.append(f"{joint}: invalid sign/offset in patch: {exc}")
            continue
        if sign not in (-1.0, 1.0):
            errors.append(f"{joint}: sign must be +1.0 or -1.0, got {sign}")
        if abs(offset) > 20.0:
            errors.append(f"{joint}: offset {offset} looks too large")

        if ignore_previous:
            continue
        current = joints_config[joint]
        if "previous_sign" in values:
            previous_sign = finite_float(values["previous_sign"])
            current_sign = finite_float(current.get("sign", 1.0))
            if abs(previous_sign - current_sign) > 1e-9:
                errors.append(f"{joint}: previous_sign {previous_sign} does not match current sign {current_sign}")
        if "previous_offset" in values:
            previous_offset = finite_float(values["previous_offset"])
            current_offset = finite_float(current.get("offset", 0.0))
            if abs(previous_offset - current_offset) > 1e-9:
                errors.append(
                    f"{joint}: previous_offset {previous_offset} does not match current offset {current_offset}"
                )
    return errors


def replace_field(line: str, field: str, value: str) -> Tuple[str, bool]:
    pattern = re.compile(rf"(\b{re.escape(field)}:\s*)([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)")
    new_line, count = pattern.subn(rf"\g<1>{value}", line, count=1)
    return new_line, count == 1


def update_config_text(config_text: str, patch_joints: Dict, decimals: int) -> str:
    lines = config_text.splitlines(keepends=True)
    updated_joints = set()
    output: List[str] = []

    for line in lines:
        new_line = line
        for joint, values in patch_joints.items():
            if not re.match(rf"^\s*{re.escape(joint)}:\s*", line):
                continue
            sign = format_number(finite_float(values["sign"]), decimals)
            offset = format_number(finite_float(values["offset"]), decimals)
            new_line, sign_replaced = replace_field(new_line, "sign", sign)
            new_line, offset_replaced = replace_field(new_line, "offset", offset)
            if not sign_replaced or not offset_replaced:
                raise ValueError(f"{joint}: could not replace sign/offset in config line")
            updated_joints.add(joint)
            break
        output.append(new_line)

    missing = [joint for joint in patch_joints if joint not in updated_joints]
    if missing:
        raise ValueError("could not find config lines for: " + ", ".join(missing))
    return "".join(output)


def print_diff(original: str, updated: str, config_path: Path) -> None:
    diff = difflib.unified_diff(
        original.splitlines(keepends=True),
        updated.splitlines(keepends=True),
        fromfile=str(config_path),
        tofile=str(config_path) + " (updated)",
    )
    print("".join(diff), end="")


def main() -> int:
    args = parse_args()
    try:
        patch_joints = load_patch(args.patch)
        joints_config = load_motor_joints(args.config)
        errors = validate_patch(joints_config, patch_joints, args.ignore_previous)
        if errors:
            for error in errors:
                print("ERROR: " + error, file=sys.stderr)
            return 1

        original = args.config.read_text(encoding="utf-8")
        updated = update_config_text(original, patch_joints, args.decimals)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if original == updated:
        print("no changes")
        return 0

    print_diff(original, updated, args.config)
    if not args.write:
        print("\ndry-run only; rerun with --write to update the config")
        return 0

    if not args.no_backup:
        shutil.copy2(args.config, args.config.with_suffix(args.config.suffix + ".bak"))
    args.config.write_text(updated, encoding="utf-8")
    print(f"\nupdated {args.config}")
    if not args.no_backup:
        print(f"backup: {args.config.with_suffix(args.config.suffix + '.bak')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
