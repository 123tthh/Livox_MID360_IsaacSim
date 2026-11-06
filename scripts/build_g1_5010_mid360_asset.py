#!/usr/bin/env python3
"""Build an explicitly labelled Unitree G1 mode-13/mode-15 (5010) + MID360 USD.

Unitree's mode-13 URDF differs from the previous ``g1_29dof_rev_1_0``
mainly at the two wrist pitch/yaw actuators and the six 5010 wrist meshes.
The locomotion checkpoint, however, was trained with InstinctLab's
torso-rooted popsicle/shoe collision model.  This builder therefore keeps the
training URDF's root and collision contract, imports the official mode-13
wrist geometry/inertias/joint limits, and finally copies the existing MID360
sensor prim below ``torso_link``.

The resulting USD is flattened so it has no external USD references.  The
vendored STL/URDF sources remain next to it for provenance and regeneration.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import sys
import tempfile
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SUPPORT_DIR = REPOSITORY_ROOT / "assets/common/g1_5010_support"
OFFICIAL_DESCRIPTION_DIR = SUPPORT_DIR / "unitree_ros/robots/g1_description"
OFFICIAL_URDF = OFFICIAL_DESCRIPTION_DIR / "g1_29dof_mode_13.urdf"
INSTINCTLAB_SOURCE_DIR = SUPPORT_DIR / "instinctlab"
TRAINING_URDF = INSTINCTLAB_SOURCE_DIR / "g1_29dof_torsoBase_popsicle_with_shoe.urdf"
CURRENT_LIDAR_USD = (
    REPOSITORY_ROOT / "assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd"
)
DERIVED_URDF = SUPPORT_DIR / "Unitree_G1_5010_Training_Collision.urdf"
OUTPUT_USD = REPOSITORY_ROOT / "assets/petal_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Petal_Scan.usd"
MANIFEST_PATH = OUTPUT_USD.parent / "manifest.json"
UNITREE_REPOSITORY = "https://github.com/unitreerobotics/unitree_ros"
UNITREE_COMMIT = "f3772ce54c56ef2d34c6aee8100bc768896c7d19"
INSTINCTLAB_REPOSITORY = "https://github.com/project-instinct/InstinctLab"
INSTINCTLAB_COMMIT = "ba28d3d2655b15a19b729476a630937a19610a3b"

WRIST_LINK_NAMES = tuple(
    f"{side}_wrist_{axis}_link"
    for side in ("left", "right")
    for axis in ("roll", "pitch", "yaw")
)
WRIST_JOINT_NAMES = tuple(
    f"{side}_wrist_{axis}_joint"
    for side in ("left", "right")
    for axis in ("roll", "pitch", "yaw")
)


def _named_elements(root: ET.Element, tag: str) -> dict[str, ET.Element]:
    elements = {element.get("name"): element for element in root.findall(tag)}
    if None in elements:
        raise ValueError(f"unnamed <{tag}> element")
    return elements


def _replace_child(target: ET.Element, source: ET.Element, tag: str) -> None:
    old_child = target.find(tag)
    new_child = source.find(tag)
    if old_child is None or new_child is None:
        raise ValueError(f"cannot replace missing <{tag}> in {target.get('name')!r}")
    index = list(target).index(old_child)
    target.remove(old_child)
    target.insert(index, copy.deepcopy(new_child))


def derive_mode13_training_urdf(
    training_urdf: str | Path,
    official_mode13_urdf: str | Path,
    output_urdf: str | Path,
    mesh_prefix: str = "unitree_ros/robots/g1_description/meshes",
) -> dict[str, object]:
    """Apply Unitree's complete 4010-to-5010 wrist delta to the training URDF."""

    training_urdf = Path(training_urdf).resolve()
    official_mode13_urdf = Path(official_mode13_urdf).resolve()
    output_urdf = Path(output_urdf).resolve()
    training_root = ET.parse(training_urdf).getroot()
    official_root = ET.parse(official_mode13_urdf).getroot()

    official_variant = official_root.get("name")
    if official_variant not in {"g1_29dof_mode_13", "g1_29dof_mode_15"}:
        raise ValueError(f"not Unitree 5010 mode 13/15: {official_mode13_urdf}")
    mode_machine = int(official_variant.rsplit("_", 1)[1])
    training_root.set("name", f"{official_variant}_5010_mid360")

    training_links = _named_elements(training_root, "link")
    official_links = _named_elements(official_root, "link")
    for name in WRIST_LINK_NAMES:
        if name not in training_links or name not in official_links:
            raise ValueError(f"missing wrist link {name!r}")
        # Keep the policy's simplified collision geometry, but take the full
        # 5010 mass/inertia and visuals from Unitree mode 13.
        _replace_child(training_links[name], official_links[name], "inertial")
        _replace_child(training_links[name], official_links[name], "visual")

    training_joints = _named_elements(training_root, "joint")
    official_joints = _named_elements(official_root, "joint")
    for name in WRIST_JOINT_NAMES:
        if name not in training_joints or name not in official_joints:
            raise ValueError(f"missing wrist joint {name!r}")
        for tag in ("origin", "axis", "limit"):
            _replace_child(training_joints[name], official_joints[name], tag)

    mesh_dir = official_mode13_urdf.parent / "meshes"
    mesh_files = []
    for mesh in training_root.findall(".//mesh"):
        basename = Path(mesh.get("filename", "")).name
        source_mesh = mesh_dir / basename
        if not source_mesh.is_file():
            raise FileNotFoundError(source_mesh)
        mesh.set("filename", f"{mesh_prefix.rstrip('/')}/{basename}")
        mesh_files.append(basename)

    revolute_joints = [joint for joint in training_root.findall("joint") if joint.get("type") == "revolute"]
    if len(revolute_joints) != 29:
        raise ValueError(f"expected 29 revolute joints, found {len(revolute_joints)}")

    for side in ("left", "right"):
        pitch_link = training_links[f"{side}_wrist_pitch_link"]
        pitch_mass = float(pitch_link.find("inertial/mass").get("value"))
        if not math.isclose(pitch_mass, 0.684, abs_tol=1.0e-12):
            raise ValueError(f"unexpected {side} 5010 wrist pitch mass: {pitch_mass}")
        for axis in ("pitch", "yaw"):
            limit = training_joints[f"{side}_wrist_{axis}_joint"].find("limit")
            if not math.isclose(float(limit.get("effort")), 13.4, abs_tol=1.0e-12):
                raise ValueError(f"unexpected {side} wrist {axis} effort")
            if not math.isclose(float(limit.get("velocity")), 27.0, abs_tol=1.0e-12):
                raise ValueError(f"unexpected {side} wrist {axis} velocity")

    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(training_root, space="  ")
    ET.ElementTree(training_root).write(output_urdf, encoding="utf-8", xml_declaration=True)
    return {
        "official_variant": official_variant,
        "mode_machine": mode_machine,
        "wrist_motor": "5010",
        "revolute_joint_count": len(revolute_joints),
        "wrist_links_replaced": len(WRIST_LINK_NAMES),
        "wrist_joints_replaced": len(WRIST_JOINT_NAMES),
        "mesh_count": len(set(mesh_files)),
        "output_urdf": str(output_urdf),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _tree_sha256(root: Path, paths: list[Path]) -> str:
    digest = hashlib.sha256()
    for path in sorted(paths):
        digest.update(path.relative_to(root).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(bytes.fromhex(_sha256(path)))
    return digest.hexdigest()


def write_manifest(
    derived_summary: dict[str, object],
    composed_summary: dict[str, object],
    derived_urdf: Path,
    output_usd: Path,
    official_urdf: Path,
    lidar_usd: Path,
    manifest_path: Path = MANIFEST_PATH,
) -> dict[str, object]:
    """Write deterministic provenance and runtime-contract metadata."""

    mesh_dir = official_urdf.parent / "meshes"
    mesh_paths = sorted(mesh_dir.glob("*.STL")) + sorted(mesh_dir.glob("*.stl"))
    manifest = {
        "schema_version": 2,
        "asset": output_usd.name,
        "derived_urdf": derived_urdf.name,
        "builder": "scripts/build_g1_5010_mid360_asset.py",
        "source": {
            "unitree_repository": UNITREE_REPOSITORY,
            "unitree_commit": UNITREE_COMMIT,
            "unitree_variant": derived_summary["official_variant"],
            "unitree_urdf_sha256": _sha256(official_urdf),
            "unitree_mesh_count": len(mesh_paths),
            "unitree_mesh_tree_sha256": _tree_sha256(official_urdf.parent, mesh_paths),
            "instinctlab_repository": INSTINCTLAB_REPOSITORY,
            "instinctlab_commit": INSTINCTLAB_COMMIT,
            "training_collision_urdf": str(TRAINING_URDF.relative_to(REPOSITORY_ROOT)),
            "training_collision_urdf_sha256": _sha256(TRAINING_URDF),
            "mid360_source": str(lidar_usd.relative_to(REPOSITORY_ROOT)),
            "mid360_source_sha256": _sha256(lidar_usd),
        },
        "composition": {
            "self_contained_usd": True,
            "external_usd_references": [],
            "embedded_action_graph": False,
            "training_shoe_integrated": True,
            "fixed_joints_merged": True,
            "root_and_collision_contract": "InstinctLab torsoBase popsicle with shoe",
        },
        "coordinate_system": {"meters_per_unit": 1, "up_axis": "Z"},
        "default_prim": composed_summary["default_prim"],
        "articulation_root": composed_summary["articulation_root"],
        "revolute_joint_count": composed_summary["revolute_joint_count"],
        "robot": {
            "mode_machine": derived_summary["mode_machine"],
            "wrist_motor": "5010",
            "hip_pitch_roll_gear_ratio": (
                [14.3, 22.5] if derived_summary["mode_machine"] == 13 else [22.5, 22.5]
            ),
            "waist_locked": False,
            "wrist_pitch_mass_kg": 0.684,
            "wrist_pitch_yaw_effort_limit_nm": 13.4,
            "wrist_pitch_yaw_velocity_limit_rad_s": 27.0,
        },
        "mid360": {
            "prim": composed_summary["mid360_prim"],
            "type": "OmniLidar",
            "model": composed_summary["mid360_model"],
            "scan_type": composed_summary["mid360_scan_type"],
            "scan_rate_hz": composed_summary["mid360_scan_rate_hz"],
            "report_rate_hz": composed_summary["mid360_report_rate_hz"],
            "emitters": composed_summary["mid360_emitters"],
            "points_per_second": 200000,
            "nominal_elevation_deg": [-7.0, 52.0],
            "robot_frame_nominal_elevation_deg": [-52.0, 7.0],
            "mount_translation_m": [0.0002835, 0.00003, 0.41618],
            "mount_roll_deg": 180.0,
            "range_m": [0.1, 70.0],
        },
        "imu": {
            "prim": composed_summary["articulation_root"] + "/mid360_imu",
            "type": "IsaacImuSensor",
            "rate_hz": 200,
            "sensor_period_s": 0.005,
            "colocated_and_aligned_with_lidar": True,
            "ros2_topic": "/mid360/imu",
            "ros2_frame": "mid360_link",
        },
        "files": {
            output_usd.name: {"bytes": output_usd.stat().st_size, "sha256": _sha256(output_usd)},
            derived_urdf.name: {"bytes": derived_urdf.stat().st_size, "sha256": _sha256(derived_urdf)},
        },
    }
    if composed_summary["mid360_scan_type"] == "SOLID_STATE":
        manifest["mid360"].update(
            {
                "profile": "petal_scan",
                "emitters_per_state": composed_summary["mid360_emitters"],
                "rtx_emitter_states": 40,
                "trajectory_states": 40,
                "channels": 20_000,
                "trajectory_elevation_deg": [-7.2123, 52.164],
                "trajectory_duration_s": 4.0,
                "trajectory_points": 800000,
                "trajectory_source": "assets/common/mid360_official_pattern/mid360.csv",
                "trajectory_sha256": "aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a",
                "runtime_pattern_driver": None,
            }
        )
    else:
        manifest["mid360"].update(
            {
                "profile": "rotary_scan",
                "report_rate_per_emitter_hz": composed_summary["mid360_report_rate_hz"],
            }
        )
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def _convert_and_compose(derived_urdf: Path, lidar_usd: Path, output_usd: Path) -> dict[str, object]:
    """Run the Isaac Lab URDF converter, attach MID360, and flatten the USD."""

    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
    from pxr import Sdf, Usd, UsdPhysics

    robot_name = ET.parse(derived_urdf).getroot().get("name")
    official_variant = robot_name.removesuffix("_5010_mid360")
    with tempfile.TemporaryDirectory(prefix="livox_mid360_g1_5010_") as temp_dir:
        converter_cfg = UrdfConverterCfg(
            asset_path=str(derived_urdf),
            usd_dir=temp_dir,
            usd_file_name=f"{robot_name}.usd",
            make_instanceable=False,
            replace_cylinders_with_capsules=True,
            merge_fixed_joints=True,
            fix_base=False,
            self_collision=True,
            force_usd_conversion=True,
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                drive_type="force",
                target_type="position",
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0),
            ),
        )
        converted = UrdfConverter(converter_cfg)
        print(f"[5010 builder] converted={converted.usd_path}", flush=True)
        target_stage = Usd.Stage.Open(converted.usd_path)
        source_stage = Usd.Stage.Open(str(lidar_usd))
        if target_stage is None or source_stage is None:
            raise RuntimeError("failed to open converted robot or source MID360 USD")

        target_root = target_stage.GetDefaultPrim()
        source_root = source_stage.GetDefaultPrim()
        if not target_root.IsValid() or not source_root.IsValid():
            raise RuntimeError("robot or MID360 source USD has no default prim")
        target_root.CreateAttribute(
            "lidarHiking:trainingShoeIntegrated",
            Sdf.ValueTypeNames.Bool,
            custom=True,
        ).Set(True)
        target_root.CreateAttribute(
            "lidarHiking:trainingShoeSoleOffsetM",
            Sdf.ValueTypeNames.Double,
            custom=True,
        ).Set(0.023)
        target_root.CreateAttribute(
            "lidarHiking:unitreeVariant",
            Sdf.ValueTypeNames.String,
            custom=True,
        ).Set(official_variant)
        target_root.CreateAttribute(
            "lidarHiking:wristMotor",
            Sdf.ValueTypeNames.String,
            custom=True,
        ).Set("5010")
        source_sensor_path = source_root.GetPath().AppendPath("torso_link/mid360_link")
        target_sensor_path = target_root.GetPath().AppendPath("torso_link/mid360_link")
        source_imu_path = source_root.GetPath().AppendPath("torso_link/mid360_imu")
        target_imu_path = target_root.GetPath().AppendPath("torso_link/mid360_imu")
        print(
            f"[5010 builder] copy {source_sensor_path} -> {target_sensor_path}",
            flush=True,
        )
        if not source_stage.GetPrimAtPath(source_sensor_path).IsValid():
            raise RuntimeError(f"source MID360 prim is missing: {source_sensor_path}")
        # The converter's root layer contains composition arcs while the body
        # specs live in its configuration layers.  Author a root-layer over
        # for the composed torso before copying a new child into that layer.
        Sdf.CreatePrimInLayer(target_stage.GetRootLayer(), target_sensor_path.GetParentPath())
        copy_result = Sdf.CopySpec(
            source_stage.GetRootLayer(),
            source_sensor_path,
            target_stage.GetRootLayer(),
            target_sensor_path,
        )
        print(f"[5010 builder] Sdf.CopySpec={copy_result!r}", flush=True)
        if copy_result is False:
            raise RuntimeError("failed to copy MID360 prim into the target G1 5010 stage")
        if not source_stage.GetPrimAtPath(source_imu_path).IsValid():
            raise RuntimeError(f"source MID360 IMU prim is missing: {source_imu_path}")
        imu_copy_result = Sdf.CopySpec(
            source_stage.GetRootLayer(),
            source_imu_path,
            target_stage.GetRootLayer(),
            target_imu_path,
        )
        print(f"[5010 builder] IMU Sdf.CopySpec={imu_copy_result!r}", flush=True)
        if imu_copy_result is False:
            raise RuntimeError("failed to copy MID360 IMU into the target stage")
        target_stage.GetRootLayer().Save()

        output_usd.parent.mkdir(parents=True, exist_ok=True)
        flattened = target_stage.Flatten(addSourceFileComment=False)
        if not flattened.Export(str(output_usd)):
            raise RuntimeError(f"failed to export flattened asset: {output_usd}")

    stage = Usd.Stage.Open(str(output_usd))
    default_prim = stage.GetDefaultPrim()
    revolute_joints = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.RevoluteJoint)]
    sensor_path = default_prim.GetPath().AppendPath("torso_link/mid360_link/mid360_native_approx")
    sensor = stage.GetPrimAtPath(sensor_path)
    if len(revolute_joints) != 29:
        raise RuntimeError(f"flattened asset has {len(revolute_joints)} revolute joints, expected 29")
    if not sensor.IsValid() or sensor.GetTypeName() != "OmniLidar":
        raise RuntimeError(f"flattened asset has no OmniLidar at {sensor_path}")
    emitters = int(sensor.GetAttribute("omni:sensor:Core:numberOfEmitters").Get())
    scan_type = str(sensor.GetAttribute("omni:sensor:Core:scanType").Get())
    report_rate = int(sensor.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get())
    if (scan_type, emitters, report_rate) not in {
        ("SOLID_STATE", 20_000, 10),
        ("ROTARY", 40, 5_000),
    }:
        raise RuntimeError(
            f"MID360 profile changed during composition: {scan_type}, {emitters}, {report_rate}"
        )
    if sensor.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() != 10:
        raise RuntimeError("MID360 scan-rate contract changed during composition")

    return {
        "output_usd": str(output_usd),
        "default_prim": str(default_prim.GetPath()),
        "articulation_root": str(default_prim.GetPath().AppendPath("torso_link")),
        "revolute_joint_count": len(revolute_joints),
        "mid360_prim": str(sensor_path),
        "mid360_emitters": emitters,
        "mid360_scan_rate_hz": 10,
        "mid360_scan_type": scan_type,
        "mid360_report_rate_hz": report_rate,
        "mid360_model": (
            "MID-360 official non-repetitive pattern replay"
            if scan_type == "SOLID_STATE"
            else "MID-360 40-line rotary approximation"
        ),
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-urdf", type=Path, default=TRAINING_URDF)
    parser.add_argument("--official-urdf", type=Path, default=OFFICIAL_URDF)
    parser.add_argument("--lidar-usd", type=Path, default=CURRENT_LIDAR_USD)
    parser.add_argument("--derived-urdf", type=Path, default=DERIVED_URDF)
    parser.add_argument("--output-usd", type=Path, default=OUTPUT_USD)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--derive-only", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    derived = derive_mode13_training_urdf(
        args.training_urdf,
        args.official_urdf,
        args.derived_urdf,
    )
    if args.derive_only:
        print(json.dumps({"derived": derived}, indent=2, sort_keys=True), flush=True)
        return

    # Isaac Sim extensions (including pxr and the URDF importer) only become
    # importable after AppLauncher has initialized Kit.
    from isaaclab.app import AppLauncher

    simulation_app = AppLauncher(headless=True).app
    try:
        composed = _convert_and_compose(
            args.derived_urdf.resolve(),
            args.lidar_usd.resolve(),
            args.output_usd.resolve(),
        )
        manifest_path = args.manifest.resolve() if args.manifest else args.output_usd.resolve().parent / "manifest.json"
        manifest = write_manifest(
            derived,
            composed,
            args.derived_urdf.resolve(),
            args.output_usd.resolve(),
            args.official_urdf.resolve(),
            args.lidar_usd.resolve(),
            manifest_path,
        )
        print(
            json.dumps(
                {
                    "derived": derived,
                    "composed": composed,
                    "manifest": str(manifest_path),
                    "asset_sha256": manifest["files"][args.output_usd.name]["sha256"],
                },
                indent=2,
                sort_keys=True,
            ),
            flush=True,
        )
    except BaseException:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        raise
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
