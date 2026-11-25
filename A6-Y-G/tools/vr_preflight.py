#!/usr/bin/env python3
"""Report physical commissioning gaps and exact profile hashes; no ROS/hardware I/O."""
import argparse
import json
from pathlib import Path
import sys
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/a6_y_vr_teleop'))
from a6_y_vr_teleop.a5_kinematics import A5Kinematics
from a6_y_vr_teleop.commissioning import profile_paths, commissioning_issues, commissioning_stage_issues, sha256
from a6_y_vr_teleop.field_calibration import field_summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    paths, manifest_path = profile_paths()
    cfg = yaml.safe_load(paths['bridge'].read_text())
    kin = A5Kinematics(paths['urdf'], paths['limits'], cfg)
    manifest = yaml.safe_load(manifest_path.read_text())
    issues = commissioning_issues(manifest, paths, kin, cfg)
    stages = {}
    for stage in ('hold', 'follow'):
        gaps = commissioning_stage_issues(stage, manifest, paths, kin, cfg)
        stages[stage] = dict(ready=not gaps, issues=gaps)
    report = dict(ready=not issues, hardware_accessed=False, j3_confirmed_cap_nm=20.,
                  calibration_evidence=field_summary(yaml.safe_load(paths['calibration'].read_text())),
                  issues=issues, commissioning_stages=stages, sha256={k: sha256(p) for k, p in paths.items()})
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 2 if issues else 0


if __name__ == '__main__': raise SystemExit(main())
