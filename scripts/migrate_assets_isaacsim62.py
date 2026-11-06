#!/usr/bin/env python3
"""Rebuild the eight existing assets for the Isaac Sim 6.2 RTX API."""
from __future__ import annotations

import json
import os
from pathlib import Path

from pxr import Sdf, Usd, Vt

from apply_mid360_petal_profile import ROOT, apply_profile, configure_core, load_trajectory
from update_asset_metadata import ASSETS, update


def migrate_rotary(path: Path) -> None:
    stage = Usd.Stage.Open(str(path))
    lidar = next(p for p in stage.Traverse() if p.GetTypeName() == "OmniLidar")
    prefix = "omni:sensor:Core:emitterState:s001:"
    elevation = list(lidar.GetAttribute(prefix + "elevationDeg").Get())
    configure_core(lidar, "rotary")
    values = {
        "azimuthDeg": (Sdf.ValueTypeNames.FloatArray, Vt.FloatArray([0.] * 40)),
        "elevationDeg": (Sdf.ValueTypeNames.FloatArray, Vt.FloatArray(elevation)),
        "fireTimeNs": (Sdf.ValueTypeNames.UIntArray, Vt.UIntArray([i * 5000 for i in range(40)])),
        "channelId": (Sdf.ValueTypeNames.UIntArray, Vt.UIntArray(list(range(1, 41)))),
        "bank": (Sdf.ValueTypeNames.UIntArray, Vt.UIntArray([0] * 40)),
    }
    for name, (dtype, value) in values.items():
        lidar.CreateAttribute(prefix + name, dtype, custom=False).Set(value)
    stage.GetDefaultPrim().SetAssetInfoByKey("version", "Isaac Sim 6.2")
    temporary = path.with_name(path.stem + ".migration-tmp.usd")
    stage.GetRootLayer().Export(str(temporary))
    os.replace(temporary, path)


def main() -> None:
    trajectory = load_trajectory()
    for spec in ASSETS:
        path = ROOT / spec["path"]
        if spec["profile"] == "petal_scan":
            apply_profile(path, trajectory)
        else:
            migrate_rotary(path)
        print(f'MIGRATED={spec["id"]}', flush=True)
    update()
    print(json.dumps({"isaac_sim": "6.2", "assets": len(ASSETS)}, indent=2))


if __name__ == "__main__":
    main()
