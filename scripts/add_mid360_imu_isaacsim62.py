#!/usr/bin/env python3
"""Add a colocated 200 Hz Isaac IMU to every published MID-360 asset."""

from __future__ import annotations

import json
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "assets/catalog.json"
LIDAR_SUFFIX = "/torso_link/mid360_link/mid360_native_approx"


def add_imu(path: Path) -> str:
    from pxr import Sdf, Usd, UsdPhysics

    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"could not open {path}")
    lidars = [
        prim
        for prim in stage.TraverseAll()
        if prim.GetTypeName() == "OmniLidar" and str(prim.GetPath()).endswith(LIDAR_SUFFIX)
    ]
    if len(lidars) != 1:
        raise RuntimeError(f"expected one MID-360 in {path}, found {len(lidars)}")
    lidar = lidars[0]
    root_path = str(lidar.GetPath())[: -len(LIDAR_SUFFIX)]
    mount = lidar.GetParent()
    imu_path = root_path + "/torso_link/mid360_imu"
    stage.RemovePrim(imu_path)
    imu = stage.DefinePrim(imu_path, "IsaacImuSensor")

    for name in ("xformOp:translate", "xformOp:orient", "xformOp:scale", "xformOpOrder"):
        source = mount.GetAttribute(name)
        if source.IsValid() and source.HasAuthoredValueOpinion():
            imu.CreateAttribute(name, source.GetTypeName(), custom=False).Set(source.Get())

    # Isaac physics IMUs require a rigid-body ancestor.  The robot torso already
    # has one; a standalone component needs a fixed kinematic rigid body so it can
    # be imported at an arbitrary pose without falling under gravity.
    if root_path == "/MID360":
        torso = stage.GetPrimAtPath("/MID360/torso_link")
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(torso)
        rigid_body.CreateRigidBodyEnabledAttr(True)
        rigid_body.CreateKinematicEnabledAttr(True)

    attributes = (
        ("enabled", Sdf.ValueTypeNames.Bool, True),
        ("sensorPeriod", Sdf.ValueTypeNames.Float, 0.005),
        ("linearAccelerationFilterWidth", Sdf.ValueTypeNames.UInt, 1),
        ("angularVelocityFilterWidth", Sdf.ValueTypeNames.UInt, 1),
        ("orientationFilterWidth", Sdf.ValueTypeNames.UInt, 1),
    )
    for name, value_type, value in attributes:
        imu.CreateAttribute(name, value_type, custom=False).Set(value)

    temporary = path.with_name(path.stem + ".imu-tmp.usd")
    if not stage.GetRootLayer().Export(str(temporary)):
        raise RuntimeError(f"could not export {temporary}")
    os.replace(temporary, path)
    return imu_path


def main() -> None:
    catalog = json.loads(CATALOG.read_text(encoding="utf-8"))
    for asset_id, entry in catalog["assets"].items():
        path = ROOT / entry["path"]
        imu_path = add_imu(path)
        print(f"IMU_ADDED={asset_id} PRIM={imu_path}", flush=True)


if __name__ == "__main__":
    simulation_app = None
    try:
        try:
            import pxr  # noqa: F401
        except ModuleNotFoundError:
            from isaacsim import SimulationApp

            simulation_app = SimulationApp({"headless": True})
        main()
    finally:
        if simulation_app is not None:
            simulation_app.close()
