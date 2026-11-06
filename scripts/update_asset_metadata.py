#!/usr/bin/env python3
"""Synchronize the eight published MID-360 manifests and asset catalog."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "assets/catalog.json"
CAD = ROOT / "assets/common/Livox_MID360_CAD.usd"
TRAJECTORY = ROOT / "assets/common/mid360_official_pattern/mid360.csv"
TRAJECTORY_SHA256 = "aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a"

ASSETS = (
    {
        "id": "mid360-petal-standalone",
        "path": "assets/petal_scan/standalone/Livox_MID360_Petal_Scan.usd",
        "profile": "petal_scan",
        "form": "standalone",
    },
    {
        "id": "g1-4010-mid360-petal",
        "path": "assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd",
        "profile": "petal_scan",
        "form": "robot",
        "wrist_motor": "4010",
    },
    {
        "id": "g1-5010-mode13-mid360-petal",
        "path": "assets/petal_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Petal_Scan.usd",
        "profile": "petal_scan",
        "form": "robot",
        "wrist_motor": "5010",
        "mode_machine": 13,
    },
    {
        "id": "g1-5010-mode15-mid360-petal",
        "path": "assets/petal_scan/g1_5010_mode_15/Unitree_G1_5010_Mode15_MID360_Petal_Scan.usd",
        "profile": "petal_scan",
        "form": "robot",
        "wrist_motor": "5010",
        "mode_machine": 15,
    },
    {
        "id": "mid360-rotary-standalone",
        "path": "assets/rotary_scan/standalone/Livox_MID360_Rotary_Scan.usd",
        "profile": "rotary_scan",
        "form": "standalone",
    },
    {
        "id": "g1-4010-mid360-rotary",
        "path": "assets/rotary_scan/g1_4010/Unitree_G1_4010_MID360_Rotary_Scan.usd",
        "profile": "rotary_scan",
        "form": "robot",
        "wrist_motor": "4010",
    },
    {
        "id": "g1-5010-mode13-mid360-rotary",
        "path": "assets/rotary_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Rotary_Scan.usd",
        "profile": "rotary_scan",
        "form": "robot",
        "wrist_motor": "5010",
        "mode_machine": 13,
    },
    {
        "id": "g1-5010-mode15-mid360-rotary",
        "path": "assets/rotary_scan/g1_5010_mode_15/Unitree_G1_5010_Mode15_MID360_Rotary_Scan.usd",
        "profile": "rotary_scan",
        "form": "robot",
        "wrist_motor": "5010",
        "mode_machine": 15,
    },
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _petal_contract() -> dict[str, object]:
    return {
        "profile": "petal_scan",
        "model": "MID-360 official non-repetitive petal scan replay",
        "scan_type": "SOLID_STATE",
        "scan_rate_hz": 10,
        "report_rate_hz": 10,
        "emitters_per_state": 20_000,
        "rtx_emitter_states": 40,
        "runtime_array_swapping": False,
        "pattern_firing_rate_hz": 10,
        "trajectory_states": 40,
        "channels": 20_000,
        "points_per_second": 200_000,
        "trajectory_duration_s": 4.0,
        "trajectory_points": 800_000,
        "trajectory_source": str(TRAJECTORY.relative_to(ROOT)),
        "trajectory_sha256": TRAJECTORY_SHA256,
        "nominal_elevation_deg": [-7.0, 52.0],
        "trajectory_elevation_deg": [-7.2123, 52.164],
        "robot_frame_nominal_elevation_deg": [-52.0, 7.0],
        "range_m": [0.1, 70.0],
        "noise_model": "ideal; no calibrated range or angle noise",
    }


def _rotary_contract() -> dict[str, object]:
    return {
        "profile": "rotary_scan",
        "model": "MID-360 40-line rotary approximation",
        "scan_type": "ROTARY",
        "scan_rate_hz": 10,
        "report_rate_per_emitter_hz": 5_000,
        "pattern_firing_rate_hz": 5_000,
        "emitters": 40,
        "points_per_second": 200_000,
        "elevation_deg": [-7.0, 52.0],
        "robot_frame_elevation_deg": [-52.0, 7.0],
        "range_m": [0.1, 70.0],
    }


def _update_robot_manifest(spec: dict[str, str], path: Path) -> None:
    manifest_path = path.parent / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    digest = sha256(path)
    previous = manifest.get("mid360", {})
    contract = _petal_contract() if spec["profile"] == "petal_scan" else _rotary_contract()
    for key in ("prim", "type", "mount_translation_m", "mount_roll_deg", "low_reflectivity_range_m"):
        if key in previous:
            contract[key] = previous[key]
    contract.setdefault("type", "OmniLidar")
    contract.setdefault("mount_roll_deg", 180.0)
    manifest["asset"] = path.name
    manifest["target_isaac_sim"] = "6.2"
    manifest["ros2_pointcloud_publisher"] = "scripts/publish_mid360_ros2_isaacsim62.py"
    manifest["profile"] = spec["profile"]
    manifest["mid360"] = contract
    robot_root = str(contract["prim"])[: -len("/torso_link/mid360_link/mid360_native_approx")]
    manifest["imu"] = {
        "prim": robot_root + "/torso_link/mid360_imu",
        "type": "IsaacImuSensor",
        "rate_hz": 200,
        "sensor_period_s": 0.005,
        "colocated_and_aligned_with_lidar": True,
        "ros2_topic": "/mid360/imu",
        "ros2_frame": "mid360_link",
    }
    manifest["files"] = {path.name: {"bytes": path.stat().st_size, "sha256": digest}}
    if isinstance(manifest.get("source"), dict):
        manifest["source"]["mid360_source"] = (
            "assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd"
            if spec["profile"] == "petal_scan"
            else "assets/rotary_scan/g1_4010/Unitree_G1_4010_MID360_Rotary_Scan.usd"
        )
        source_path = ROOT / manifest["source"]["mid360_source"]
        manifest["source"]["mid360_source_sha256"] = sha256(source_path)
        manifest["source"]["support_root"] = "assets/common/g1_5010_support"
        manifest["source"]["training_collision_urdf"] = (
            "assets/common/g1_5010_support/instinctlab/"
            "g1_29dof_torsoBase_popsicle_with_shoe.urdf"
        )
        mode = spec.get("mode_machine", 13)
        manifest["derived_urdf"] = (
            "../../common/g1_5010_support/Unitree_G1_5010_Mode15_Training_Collision.urdf"
            if mode == 15
            else "../../common/g1_5010_support/Unitree_G1_5010_Training_Collision.urdf"
        )
    manifest["builder"] = "scripts/build_g1_5010_mid360_asset.py" if spec["wrist_motor"] == "5010" else None
    if manifest["builder"] is None:
        manifest.pop("builder", None)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_standalone_manifest(spec: dict[str, str], path: Path) -> None:
    digest = sha256(path)
    contract = _petal_contract() if spec["profile"] == "petal_scan" else _rotary_contract()
    contract.update(
        {
            "prim": "/MID360/torso_link/mid360_link/mid360_native_approx",
            "type": "OmniLidar",
            "mount_translation_m": [0.0, 0.0, 0.0],
            "mount_roll_deg": 0.0,
        }
    )
    manifest = {
        "schema_version": 1,
        "asset": path.name,
        "target_isaac_sim": "6.2",
        "ros2_pointcloud_publisher": "scripts/publish_mid360_ros2_isaacsim62.py",
        "profile": spec["profile"],
        "form": "standalone",
        "default_prim": "/MID360",
        "coordinate_system": {"meters_per_unit": 1, "up_axis": "Z"},
        "composition": {
            "self_contained_sensor": True,
            "cad_reference": "../../common/Livox_MID360_CAD.usd",
        },
        "mid360": contract,
        "imu": {
            "prim": "/MID360/torso_link/mid360_imu",
            "type": "IsaacImuSensor",
            "rate_hz": 200,
            "sensor_period_s": 0.005,
            "colocated_and_aligned_with_lidar": True,
            "ros2_topic": "/mid360/imu",
            "ros2_frame": "mid360_link",
        },
        "files": {path.name: {"bytes": path.stat().st_size, "sha256": digest}},
    }
    (path.parent / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def update() -> dict[str, object]:
    if sha256(TRAJECTORY) != TRAJECTORY_SHA256:
        raise ValueError("official MID-360 trajectory hash changed")
    catalog_assets: dict[str, object] = {}
    for spec in ASSETS:
        path = ROOT / spec["path"]
        if not path.is_file():
            raise FileNotFoundError(path)
        if spec["form"] == "robot":
            _update_robot_manifest(spec, path)
        else:
            _write_standalone_manifest(spec, path)
        entry = {key: value for key, value in spec.items() if key != "id"}
        entry["sha256"] = sha256(path)
        entry["includes_imu"] = True
        catalog_assets[spec["id"]] = entry
    catalog = {
        "schema_version": 2,
        "project": "Livox_MID360_IsaacSim",
        "target_isaac_sim": "6.2",
        "legacy_branch": "isaac-sim-5.1",
        "common_files": {
            "cad": {"path": str(CAD.relative_to(ROOT)), "sha256": sha256(CAD)},
            "official_pattern": {
                "path": str(TRAJECTORY.relative_to(ROOT)),
                "sha256": sha256(TRAJECTORY),
            },
        },
        "assets": catalog_assets,
    }
    CATALOG.write_text(json.dumps(catalog, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return catalog


if __name__ == "__main__":
    print(json.dumps(update(), indent=2, sort_keys=True))
