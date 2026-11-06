#!/usr/bin/env python3
"""Build one standalone MID-360 USD from a robot asset and the common CAD USD.

The source OmniLidar scanType selects the native 6.2 Petal or Rotary profile.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path


def _bootstrap_pxr():
    try:
        from pxr import Gf, Kind, Sdf, Usd, UsdGeom

        return None, Gf, Kind, Sdf, Usd, UsdGeom
    except ModuleNotFoundError:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True})
        from pxr import Gf, Kind, Sdf, Usd, UsdGeom

        return app, Gf, Kind, Sdf, Usd, UsdGeom


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("cad_usd", type=Path)
    parser.add_argument("output_usd", type=Path)
    parser.add_argument(
        "--sensor-source",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd",
    )
    args = parser.parse_args()

    app, Gf, Kind, Sdf, Usd, UsdGeom = _bootstrap_pxr()
    try:
        source = Usd.Stage.Open(str(args.sensor_source.resolve()))
        if source is None:
            raise RuntimeError(f"Could not open sensor source {args.sensor_source}")
        args.output_usd.parent.mkdir(parents=True, exist_ok=True)
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        root = UsdGeom.Xform.Define(stage, "/MID360").GetPrim()
        stage.SetDefaultPrim(root)
        source_mount = source.GetDefaultPrim().GetPath().AppendPath("torso_link/mid360_link")
        source_lidar = source.GetPrimAtPath(source_mount.AppendChild("mid360_native_approx"))
        profile_name = "Petal Scan" if source_lidar.GetAttribute('omni:sensor:Core:scanType').Get() == 'SOLID_STATE' else "Rotary Scan"
        root.SetAssetInfoByKey("name", f"Livox MID-360 {profile_name} RTX LiDAR")
        root.SetAssetInfoByKey("version", "Isaac Sim 6.2")
        Usd.ModelAPI(root).SetKind(Kind.Tokens.component)
        UsdGeom.Xform.Define(stage, "/MID360/torso_link")
        UsdGeom.Xform.Define(stage, "/MID360/torso_link/mid360_link")

        source_root = source.GetRootLayer()
        target_root = stage.GetRootLayer()
        source_root_path = source.GetDefaultPrim().GetPath()
        copies = (
            (
                str(source_root_path.AppendPath("torso_link/mid360_link/mid360_native_approx")),
                "/MID360/torso_link/mid360_link/mid360_native_approx",
            ),
            (
                str(source_root_path.AppendPath("torso_link/mid360_imu")),
                "/MID360/torso_link/mid360_imu",
            ),
        )
        for source_path, target_path in copies:
            if source.GetPrimAtPath(source_path).IsValid():
                if not Sdf.CopySpec(source_root, source_path, target_root, target_path):
                    raise RuntimeError(f"Could not copy {source_path}")

        cad = UsdGeom.Xform.Define(stage, "/MID360/torso_link/mid360_link/CAD")
        relative_cad = os.path.relpath(args.cad_usd.resolve(), args.output_usd.resolve().parent)
        cad.GetPrim().GetReferences().AddReference(relative_cad)
        # HOOPS preserves STEP coordinates in millimetres.  The wrapper is in metres.
        cad.AddScaleOp().Set(Gf.Vec3f(0.001, 0.001, 0.001))

        temporary = args.output_usd.with_name(args.output_usd.stem + '.build-tmp.usd')
        stage.GetRootLayer().Export(str(temporary))
        os.replace(temporary, args.output_usd)
        from add_mid360_imu_isaacsim62 import add_imu
        add_imu(args.output_usd.resolve())
        print(f"PROFILE={profile_name.upper().replace(' ', '_')}", flush=True)
        print(f"STANDALONE_USD={args.output_usd.resolve()}", flush=True)
        return 0
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    raise SystemExit(main())
