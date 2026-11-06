#!/usr/bin/env python3
"""Install the pinned MID360 robot assets into a Lidar_Hiking checkout."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from validate_assets import validate


ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = ROOT / "assets"
RELATIVE_TARGET = Path("simulation/source/lidar_hiking_sim/lidar_hiking_sim/assets")
PROFILES = ("petal_scan", "rotary_scan")
ASSET_FORMS = ("standalone", "g1_4010", "g1_5010_mode_13", "g1_5010_mode_15")
PUBLISHER_SOURCE = ROOT / "scripts/publish_mid360_ros2_isaacsim62.py"
PUBLISHER_TARGET = Path("simulation/scripts/publish_lidar_hiking_mid360_ros2_isaacsim62.py")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project_root", type=Path)
    parser.add_argument("--profile", choices=PROFILES, default="petal_scan")
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()

    validate()
    target_root = args.project_root.resolve() / RELATIVE_TARGET
    if not target_root.parent.is_dir():
        raise FileNotFoundError(f"not a Lidar_Hiking checkout: {args.project_root}")
    install_root = target_root / "mid360"
    for directory in ASSET_FORMS:
        source = ASSET_ROOT / args.profile / directory
        target = install_root / args.profile / directory
        if not args.verify_only:
            shutil.copytree(source, target, dirs_exist_ok=True)
        print(f"{'verified' if args.verify_only else 'installed'}: {target}")

    common_target = install_root / "common"
    if not args.verify_only:
        shutil.copytree(ASSET_ROOT / "common", common_target, dirs_exist_ok=True)
    print(f"{'verified' if args.verify_only else 'installed'}: {common_target}")

    publisher_target = args.project_root.resolve() / PUBLISHER_TARGET
    if publisher_target.parent.is_dir():
        if not args.verify_only:
            shutil.copy2(PUBLISHER_SOURCE, publisher_target)
            shutil.copy2(ROOT / 'scripts/mid360_runtime_isaacsim62.py', publisher_target.parent / 'mid360_runtime_isaacsim62.py')
        print(f"{'verified' if args.verify_only else 'installed'}: {publisher_target}")


if __name__ == "__main__":
    main()
