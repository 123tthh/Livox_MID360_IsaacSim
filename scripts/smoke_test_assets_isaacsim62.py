#!/usr/bin/env python3
"""Open and validate all published assets inside Isaac Sim 6.2."""

from __future__ import annotations

import json
import traceback
from importlib.metadata import version
from pathlib import Path
import numpy as np

from isaacsim import SimulationApp


APP = SimulationApp({"headless": True, "multi_gpu": False,
                     "extra_args": ["--/renderer/raytracingMotion/enabled=true",
                                    "--/renderer/raytracingMotion/enabledForHydraEngines=0,1,2,3"]})

import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdUtils  # noqa: E402
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
from apply_mid360_petal_profile import load_trajectory


ROOT = Path(__file__).resolve().parents[1]
CATALOG = json.loads((ROOT / "assets/catalog.json").read_text(encoding="utf-8"))
SCENE_CATALOG = json.loads((ROOT / "tests/scenes/catalog.json").read_text(encoding="utf-8"))
OBJECT_FIELD_SCENES = {
    "petal": "tests/scenes/object_field/MID360_G1_5010_Petal_Object_Field.usda",
    "rotary": "tests/scenes/object_field/MID360_G1_5010_Rotary_Object_Field.usda",
}
LIDAR_SUFFIX = "/torso_link/mid360_link/mid360_native_approx"
TRAJECTORY = load_trajectory()
RESULTS = {"status": "RUNNING", "isaac_sim": version("isaacsim"),
           "assets": {}, "stair_scenes": [], "object_field_scenes": [], "render_products": []}
REPORT = ROOT / "docs/validation/isaacsim62_structure.json"


def _one_lidar(stage: Usd.Stage):
    lidars = [
        prim
        for prim in stage.TraverseAll()
        if prim.GetTypeName() == "OmniLidar" and str(prim.GetPath()).endswith(LIDAR_SUFFIX)
    ]
    if len(lidars) != 1:
        raise RuntimeError(f"expected one MID-360 OmniLidar, found {len(lidars)}")
    return lidars[0]


def _same_transform(left, right) -> bool:
    return max(
        abs(float(left[row][column] - right[row][column]))
        for row in range(4)
        for column in range(4)
    ) <= 1.0e-6


def validate_asset(asset_id: str, entry: dict[str, object]) -> None:
    path = ROOT / str(entry["path"])
    stage = Usd.Stage.Open(str(path))
    if stage is None or not stage.GetDefaultPrim().IsValid():
        raise RuntimeError(f"could not open {path}")
    lidar = _one_lidar(stage)
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.
    assert UsdGeom.GetStageUpAxis(stage) == 'Z'
    assert not UsdUtils.ComputeAllDependencies(str(path))[2], f'unresolved dependencies: {asset_id}'
    core = "omni:sensor:Core:"
    scan_type = str(lidar.GetAttribute(core + "scanType").Get())
    emitters = int(lidar.GetAttribute(core + "numberOfEmitters").Get())
    scan_rate = int(lidar.GetAttribute(core + "scanRateBaseHz").Get())
    pattern = lidar.GetParent().GetChild("mid360_nonrepetitive_pattern")
    expected_profile = str(entry["profile"])
    if expected_profile == "petal_scan":
        if (scan_type, emitters, scan_rate) != ("SOLID_STATE", 20_000, 10):
            raise RuntimeError(f"bad Petal profile in {asset_id}")
        schemas = list(lidar.GetMetadata('apiSchemas').explicitItems)
        assert sum('EmitterStateAPI:' in name for name in schemas) == 40
        assert not pattern.IsValid(), '6.2 must use native states, without a swapped-array Scope'
        for i, (az, el) in enumerate(TRAJECTORY.states(), start=1):
            prefix = core + f'emitterState:s{i:03d}:'
            expected_az = np.asarray([-(v if v < 180. else v-360.) for v in az], dtype=np.float32)
            assert np.array_equal(lidar.GetAttribute(prefix+'azimuthDeg').Get(), expected_az)
            assert np.array_equal(lidar.GetAttribute(prefix+'elevationDeg').Get(), np.asarray(el, dtype=np.float32))
            assert np.array_equal(lidar.GetAttribute(prefix+'fireTimeNs').Get(), np.arange(20000)*5000)
    else:
        report_rate = int(lidar.GetAttribute(core + "patternFiringRateHz").Get())
        if (scan_type, emitters, scan_rate, report_rate) != ("ROTARY", 40, 10, 5_000):
            raise RuntimeError(f"bad Rotary profile in {asset_id}")
        if pattern.IsValid():
            raise RuntimeError(f"Rotary asset unexpectedly embeds Petal trajectory: {asset_id}")

    joint_count = sum(1 for prim in stage.Traverse() if prim.IsA(UsdPhysics.RevoluteJoint))
    if entry["form"] == "robot":
        if joint_count != 29:
            raise RuntimeError(f"{asset_id} has {joint_count} revolute joints")
        mount = lidar.GetParent()
        robot_root_path = str(lidar.GetPath())[: -len(LIDAR_SUFFIX)]
        imu = stage.GetPrimAtPath(robot_root_path + "/torso_link/mid360_imu")
        if not imu.IsValid():
            imu_candidates = [
                str(prim.GetPath())
                for prim in stage.TraverseAll()
                if prim.GetTypeName() == "IsaacImuSensor" or "mid360_imu" in str(prim.GetPath())
            ]
            raise RuntimeError(f"missing matching IMU in {asset_id}; candidates={imu_candidates}")
        mount_matrix = UsdGeom.Xformable(mount).GetLocalTransformation()
        imu_matrix = UsdGeom.Xformable(imu).GetLocalTransformation()
        if not _same_transform(mount_matrix, imu_matrix):
            raise RuntimeError(f"LiDAR/IMU transform mismatch in {asset_id}")
        sensor_up = mount_matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)).GetNormalized()
        if sensor_up[2] > -0.999:
            raise RuntimeError(f"robot MID-360 is not mounted with 180-degree roll: {asset_id}")
        if "mode_machine" in entry:
            expected_variant = f"g1_29dof_mode_{entry['mode_machine']}"
            actual_variant = stage.GetDefaultPrim().GetAttribute(
                "lidarHiking:unitreeVariant"
            ).Get()
            if actual_variant != expected_variant:
                raise RuntimeError(
                    f"{asset_id} is labelled {actual_variant!r}, expected {expected_variant!r}"
                )
    else:
        if joint_count != 0:
            raise RuntimeError(f"standalone asset contains robot joints: {asset_id}")
        torso = stage.GetPrimAtPath("/MID360/torso_link")
        rigid_body = UsdPhysics.RigidBodyAPI(torso)
        if not rigid_body or not rigid_body.GetRigidBodyEnabledAttr().Get():
            raise RuntimeError(f"standalone IMU parent is not a rigid body: {asset_id}")
        if not rigid_body.GetKinematicEnabledAttr().Get():
            raise RuntimeError(f"standalone MID-360 rigid body is not fixed: {asset_id}")

    RESULTS['assets'][asset_id] = {'profile': expected_profile, 'joint_count': joint_count,
                                  'unresolved_dependencies': [], 'official_rays_verified': 800000 if expected_profile == 'petal_scan' else 0}
    print(
        f"ASSET_OK={asset_id} PROFILE={expected_profile} FORM={entry['form']} "
        f"SCAN_TYPE={scan_type} EMITTERS={emitters} JOINTS={joint_count}",
        flush=True,
    )


def render_smoke(asset_id: str, entry: dict[str, object]) -> None:
    context = omni.usd.get_context()
    context.open_stage(str(ROOT / str(entry["path"])))
    for _ in range(8):
        APP.update()
    stage = context.get_stage()
    lidar = _one_lidar(stage)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        sensor = LidarSensor(Lidar(str(lidar.GetPath()), reset_xform_op_properties=False), annotators=['generic-model-output'])
    for _ in range(12):
        APP.update()
    sensor.detach_annotators(['generic-model-output'])
    for _ in range(3):
        APP.update()
    print(f"RENDER_OK={asset_id}", flush=True)
    RESULTS['render_products'].append(asset_id)


def validate_scene(scene_id: str, entry: dict[str, object]) -> None:
    path = ROOT / str(entry["stage"]["path"])
    stage = Usd.Stage.Open(str(path))
    if stage is None or str(stage.GetDefaultPrim().GetPath()) != "/World":
        raise RuntimeError(f"could not open test scene {path}")
    environment = stage.GetPrimAtPath("/World/Environment")
    mesh = stage.GetPrimAtPath("/World/Environment/mesh")
    if not environment.IsValid() or not mesh.IsValid():
        raise RuntimeError(f"missing composed stair environment in {scene_id}")
    step_height = float(environment.GetAttribute("stairs:stepHeight").Get())
    if abs(step_height - float(entry["step_height_m"])) > 1.0e-9:
        raise RuntimeError(f"wrong step height in {scene_id}: {step_height}")
    collision = mesh.GetAttribute("physics:collisionEnabled").Get()
    if collision is not True:
        raise RuntimeError(f"collision is not enabled in {scene_id}")
    lidar = _one_lidar(stage)
    imu = stage.GetPrimAtPath("/World/MID360/torso_link/mid360_imu")
    if not imu.IsValid():
        raise RuntimeError(f"missing standalone MID-360 IMU in {scene_id}")
    print(
        f"SCENE_OK={scene_id} STEP_HEIGHT_M={step_height:g} LIDAR={lidar.GetPath()}",
        flush=True,
    )
    RESULTS['stair_scenes'].append(scene_id)


def render_scene_smoke(scene_id: str, entry: dict[str, object]) -> None:
    context = omni.usd.get_context()
    context.open_stage(str(ROOT / str(entry["stage"]["path"])))
    for _ in range(8):
        APP.update()
    stage = context.get_stage()
    lidar = _one_lidar(stage)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        sensor = LidarSensor(Lidar(str(lidar.GetPath()), reset_xform_op_properties=False), annotators=['generic-model-output'])
    for _ in range(12):
        APP.update()
    sensor.detach_annotators(['generic-model-output'])
    for _ in range(3):
        APP.update()
    print(f"SCENE_RENDER_OK={scene_id}", flush=True)
    RESULTS['render_products'].append(scene_id)


def validate_object_field_scene(profile: str, relative_path: str) -> None:
    path = ROOT / relative_path
    stage = Usd.Stage.Open(str(path))
    if stage is None or str(stage.GetDefaultPrim().GetPath()) != "/World":
        raise RuntimeError(f"could not open object-field scene {path}")
    world = stage.GetDefaultPrim()
    if world.GetAttribute("mid360Validation:profile").Get() != profile:
        raise RuntimeError(f"wrong object-field profile in {path}")
    if world.GetAttribute("mid360Validation:lockedJointCount").Get() != 29:
        raise RuntimeError(f"wrong locked-joint count in {path}")

    lidar = _one_lidar(stage)
    expected_scan_type = "SOLID_STATE" if profile == "petal" else "ROTARY"
    actual_scan_type = str(lidar.GetAttribute("omni:sensor:Core:scanType").Get())
    if actual_scan_type != expected_scan_type:
        raise RuntimeError(f"wrong LiDAR profile in {path}: {actual_scan_type}")

    joints = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.RevoluteJoint)]
    if len(joints) != 29:
        raise RuntimeError(f"object-field scene has {len(joints)} revolute joints")
    for joint in joints:
        lower = float(joint.GetAttribute("physics:lowerLimit").Get())
        upper = float(joint.GetAttribute("physics:upperLimit").Get())
        if lower != 0.0 or upper != 0.0:
            raise RuntimeError(f"joint is not locked in {profile}: {joint.GetPath()}")

    anchor = stage.GetPrimAtPath("/World/G1WorldAnchor")
    if not anchor.IsA(UsdPhysics.FixedJoint):
        raise RuntimeError(f"missing G1 world anchor in {profile}")
    objects = stage.GetPrimAtPath("/World/ObjectField/Objects").GetChildren()
    if len(objects) != 10:
        raise RuntimeError(f"expected 10 object groups in {profile}, found {len(objects)}")
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    for object_prim in objects:
        box = bbox_cache.ComputeWorldBound(object_prim).ComputeAlignedBox()
        minimum = box.GetMin()
        maximum = box.GetMax()
        center_x = 0.5 * (minimum[0] + maximum[0])
        center_y = 0.5 * (minimum[1] + maximum[1])
        if abs(float(minimum[2])) > 1.0e-5:
            raise RuntimeError(f"object does not touch ground: {object_prim.GetPath()} z={minimum[2]}")
        if center_x * center_x + center_y * center_y > 25.0:
            raise RuntimeError(f"object center is outside 5 m: {object_prim.GetPath()}")

    projection_wall = stage.GetPrimAtPath("/World/ObjectField/Objects/ProjectionWall")
    if projection_wall.GetAttribute("mid360Validation:angularProjectionTarget").Get() is not True:
        raise RuntimeError(f"missing angular projection target in {profile}")

    robot_box = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath("/World/G1")).ComputeAlignedBox()
    robot_min_z = float(robot_box.GetMin()[2])
    if abs(robot_min_z) > 5.0e-4:
        raise RuntimeError(f"G1 feet are not on the ground in {profile}: z={robot_min_z}")
    print(
        f"OBJECT_SCENE_OK={profile} OBJECTS={len(objects)} JOINTS={len(joints)} "
        f"ROBOT_MIN_Z={robot_min_z:.6f} SCAN_TYPE={actual_scan_type}",
        flush=True,
    )
    RESULTS['object_field_scenes'].append(profile)


def main() -> None:
    try:
        for asset_id, entry in CATALOG["assets"].items():
            validate_asset(asset_id, entry)
        for asset_id in ("mid360-petal-standalone", "mid360-rotary-standalone"):
            render_smoke(asset_id, CATALOG["assets"][asset_id])
        for scene_id, entry in SCENE_CATALOG["scenes"].items():
            validate_scene(scene_id, entry)
        render_scene_smoke("h10cm", SCENE_CATALOG["scenes"]["h10cm"])
        for profile, relative_path in OBJECT_FIELD_SCENES.items():
            validate_object_field_scene(profile, relative_path)
            render_scene_smoke(
                f"object_{profile}", {"stage": {"path": relative_path}}
            )
        print(f"ISAACSIM_ASSET_COUNT={len(CATALOG['assets'])}", flush=True)
        print(f"ISAACSIM_SCENE_COUNT={len(SCENE_CATALOG['scenes'])}", flush=True)
        RESULTS['status'] = 'PASS'
    except BaseException:
        RESULTS['status'] = 'FAIL'
        traceback.print_exc()
        raise
    finally:
        REPORT.write_text(json.dumps(RESULTS, indent=2)+'\n')
        APP.close()


main()
