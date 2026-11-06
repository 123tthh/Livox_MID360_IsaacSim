#!/usr/bin/env python3
"""Embed the Livox MID-360 non-repetitive firing pattern in bundled USDs.

Run with Isaac Sim 6.2's Python environment (or another Python environment
that provides ``pxr``).  The source trajectory is the 800,000-ray MID-360
table published by Livox-SDK/livox_laser_simulation.  Forty emitter states
represent consecutive 0.1 s hardware frames, so every state contains 20,000
rays and the full table represents four seconds of non-repeating motion.
"""

from __future__ import annotations

import argparse
import array
import csv
import hashlib
import os
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY = ROOT / "assets/common/mid360_official_pattern/mid360.csv"
ASSETS = (
    ROOT / "assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd",
    ROOT / "assets/petal_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Petal_Scan.usd",
    ROOT / "assets/petal_scan/g1_5010_mode_15/Unitree_G1_5010_Mode15_MID360_Petal_Scan.usd",
)
POINT_RATE_HZ = 200_000
SCAN_RATE_HZ = 10
POINTS_PER_STATE = POINT_RATE_HZ // SCAN_RATE_HZ
STATE_COUNT = 40
EMBEDDED_EMITTER_STATE_COUNT = STATE_COUNT
POINT_INTERVAL_NS = 1_000_000_000 // POINT_RATE_HZ
CHANNEL_COUNT = POINTS_PER_STATE
EXPECTED_POINTS = POINTS_PER_STATE * STATE_COUNT
EXPECTED_TRAJECTORY_SHA256 = "aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a"
LIDAR_SUFFIX = "/torso_link/mid360_link/mid360_native_approx"
PATTERN_DATA_NAME = "mid360_nonrepetitive_pattern"
PATTERN_ENCODING = "zlib+u16le:azimuth_centideg,elevation_centideg_plus_1000"


@dataclass(frozen=True)
class Trajectory:
    azimuth_deg: tuple[float, ...]
    elevation_deg: tuple[float, ...]

    def states(self) -> Iterable[tuple[tuple[float, ...], tuple[float, ...]]]:
        for start in range(0, len(self.azimuth_deg), POINTS_PER_STATE):
            stop = start + POINTS_PER_STATE
            yield self.azimuth_deg[start:stop], self.elevation_deg[start:stop]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_trajectory(path: Path = TRAJECTORY) -> Trajectory:
    """Load and validate Livox's azimuth/zenith table.

    The first column is a monotonically increasing sample index despite its
    upstream ``Time/s`` heading.  Timing is reconstructed from the official
    200,000 points/s specification instead of interpreting that heading.
    """

    actual_sha = sha256(path)
    if actual_sha != EXPECTED_TRAJECTORY_SHA256:
        raise ValueError(f"unexpected MID-360 trajectory sha256: {actual_sha}")

    azimuth: list[float] = []
    elevation: list[float] = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = csv.reader(stream)
        header = next(rows, None)
        if header != ["Time/s", "Azimuth/deg", "Zenith/deg"]:
            raise ValueError(f"unexpected MID-360 trajectory header: {header!r}")
        for expected_index, row in enumerate(rows, start=1):
            if len(row) != 3 or int(row[0]) != expected_index:
                raise ValueError(f"invalid trajectory row {expected_index}: {row!r}")
            azimuth_deg = float(row[1]) % 360.0
            elevation_deg = 90.0 - float(row[2])
            if not 0.0 <= azimuth_deg < 360.0:
                raise ValueError(f"azimuth outside 360-degree FOV at row {expected_index}")
            # Livox's reference trace has sub-degree edge excursions around
            # the nominal [-7, 52] degree FOV.  Reject anything larger.
            if not -7.25 <= elevation_deg <= 52.25:
                raise ValueError(f"elevation outside MID-360 FOV tolerance at row {expected_index}")
            azimuth.append(azimuth_deg)
            elevation.append(elevation_deg)

    if len(azimuth) != EXPECTED_POINTS:
        raise ValueError(f"expected {EXPECTED_POINTS} trajectory points, found {len(azimuth)}")
    return Trajectory(tuple(azimuth), tuple(elevation))


def encode_trajectory(trajectory: Trajectory) -> bytes:
    """Quantize directions to Livox's 0.01-degree wire resolution."""

    values = array.array("H")
    for azimuth_deg, elevation_deg in zip(trajectory.azimuth_deg, trajectory.elevation_deg):
        values.append(round(azimuth_deg * 100.0) % 36_000)
        values.append(round((elevation_deg + 10.0) * 100.0))
    if sys.byteorder != "little":
        values.byteswap()
    return zlib.compress(values.tobytes(), level=9)


def _find_lidar(stage):
    candidates = [
        prim
        for prim in stage.TraverseAll()
        if prim.GetTypeName() == "OmniLidar" and str(prim.GetPath()).endswith(LIDAR_SUFFIX)
    ]
    if len(candidates) != 1:
        raise RuntimeError(f"expected one bundled MID-360 OmniLidar, found {len(candidates)}")
    return candidates[0]


def apply_profile(asset_path: Path, trajectory: Trajectory) -> dict[str, object]:
    """Author all forty native states; no runtime array swapping is required."""

    from pxr import Sdf, Usd, Vt

    stage = Usd.Stage.Open(str(asset_path))
    if stage is None:
        raise RuntimeError(f"failed to open {asset_path}")
    lidar = _find_lidar(stage)

    configure_core(lidar, "petal")
    fire_times = Vt.UIntArray([index * POINT_INTERVAL_NS for index in range(POINTS_PER_STATE)])
    channel_ids = Vt.UIntArray(range(1, POINTS_PER_STATE + 1))
    banks = Vt.UIntArray([0] * POINTS_PER_STATE)
    for state, (azimuth, elevation) in enumerate(trajectory.states(), start=1):
        # RTX Core uses clockwise-positive azimuth; Livox XYZ is CCW-positive.
        signed_azimuth = [-(value if value < 180.0 else value - 360.0) for value in azimuth]
        prefix = f"omni:sensor:Core:emitterState:s{state:03d}:"
        values = (
            ("azimuthDeg", Sdf.ValueTypeNames.FloatArray, Vt.FloatArray(signed_azimuth)),
            ("elevationDeg", Sdf.ValueTypeNames.FloatArray, Vt.FloatArray(elevation)),
            ("fireTimeNs", Sdf.ValueTypeNames.UIntArray, fire_times),
            ("channelId", Sdf.ValueTypeNames.UIntArray, channel_ids),
            ("bank", Sdf.ValueTypeNames.UIntArray, banks),
        )
        for field, value_type, value in values:
            lidar.CreateAttribute(prefix + field, value_type, custom=False).Set(value)
    pattern_path = lidar.GetParent().GetPath().AppendChild(PATTERN_DATA_NAME)
    stage.RemovePrim(pattern_path)
    lidar.CreateAttribute("mid360:trajectorySha256", Sdf.ValueTypeNames.String, custom=True).Set(EXPECTED_TRAJECTORY_SHA256)
    lidar.CreateAttribute("mid360:trajectoryStates", Sdf.ValueTypeNames.UInt, custom=True).Set(STATE_COUNT)
    lidar.CreateAttribute("mid360:trajectoryPeriodSeconds", Sdf.ValueTypeNames.Double, custom=True).Set(4.)
    stage.GetDefaultPrim().SetAssetInfoByKey("version", "Isaac Sim 6.2")

    # Sdf crate Save() appends changed data and grows on every regeneration.
    # Export to a fresh crate and atomically replace the asset so this command
    # stays size-stable and deterministic.
    temporary_asset = asset_path.with_name(asset_path.stem + ".profile-tmp.usd")
    if not stage.GetRootLayer().Export(str(temporary_asset)):
        raise RuntimeError(f"failed to export compact profile asset {temporary_asset}")
    os.replace(temporary_asset, asset_path)
    return {
        "asset": str(asset_path.relative_to(ROOT)),
        "lidar_prim": str(lidar.GetPath()),
        "scan_type": "SOLID_STATE",
        "scan_rate_hz": SCAN_RATE_HZ,
        "rtx_emitter_states": EMBEDDED_EMITTER_STATE_COUNT,
        "trajectory_states": STATE_COUNT,
        "emitters_per_state": POINTS_PER_STATE,
        "points_per_second": POINT_RATE_HZ,
        "trajectory_duration_s": EXPECTED_POINTS / POINT_RATE_HZ,
        "runtime_array_swapping": False,
    }


def configure_core(lidar, profile: str) -> None:
    """Set the 6.2 LidarCore schema and its ideal single-return configuration."""
    from pxr import Sdf, Vt

    for name in tuple(lidar.GetPropertyNames()):
        if name.startswith("omni:sensor:"):
            lidar.RemoveProperty(name)
    count = STATE_COUNT if profile == "petal" else 1
    emitters = POINTS_PER_STATE if profile == "petal" else 40
    firing_rate = SCAN_RATE_HZ if profile == "petal" else 5000
    lidar.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(
        ["OmniSensorGenericLidarCoreAPI"] +
        [f"OmniSensorGenericLidarCoreEmitterStateAPI:s{i:03d}" for i in range(1, count + 1)]))

    def set_value(name, dtype, value):
        lidar.CreateAttribute(name, dtype, custom=False).Set(value)

    for name, value in {"marketName": "Livox MID-360", "modelName": "LidarCore", "modelVendor": "NVIDIA", "modelVersion": "0.0.0"}.items():
        set_value("omni:sensor:" + name, Sdf.ValueTypeNames.String, value)
    set_value("omni:sensor:tickRate", Sdf.ValueTypeNames.Float, 10.)
    core = "omni:sensor:Core:"
    for name, value in {"scanType": "SOLID_STATE" if profile == "petal" else "ROTARY", "rayType": "IDEALIZED",
                        "elementsCoordsType": "CARTESIAN", "outputFrameOfReference": "SENSOR",
                        "outputMotionCompensationState": "NONCOMPENSATED", "rotationDirection": "CW"}.items():
        set_value(core + name, Sdf.ValueTypeNames.Token, value)
    for name, value in {"scanRateBaseHz": 10, "patternFiringRateHz": firing_rate, "numberOfEmitters": emitters,
                        "numberOfChannels": emitters, "numLines": 1, "maxReturns": 1, "stateResolutionStep": 1}.items():
        set_value(core + name, Sdf.ValueTypeNames.UInt, value)
    for name, value in {"nearRangeM": .1, "farRangeM": 70., "minDistBetweenEchosM": 0., "rangeOffsetM": 0.,
                        "rangeAccuracyM": 0., "rangeResolutionM": .001, "azimuthErrorMean": 0., "azimuthErrorStd": 0.,
                        "elevationErrorMean": 0., "elevationErrorStd": 0., "minReflectance": .01,
                        "minReflectionRangeM": 70., "divergenceHorDeg": 0., "divergenceVerDeg": 0.,
                        "validStartAzimuthDeg": -180. if profile == "petal" else 0.,
                        "validEndAzimuthDeg": 180. if profile == "petal" else 360.}.items():
        set_value(core + name, Sdf.ValueTypeNames.Float, value)
    for name, value in {"accumulateOutputs": True, "instantLidar": False, "skipDroppingInvalidPoints": False}.items():
        set_value(core + name, Sdf.ValueTypeNames.Bool, value)
    for name in ["originErrorMean", "originErrorStd"]:
        set_value(core + name, Sdf.ValueTypeNames.FloatArray, Vt.FloatArray([0., 0., 0.]))
    set_value(core + "numRaysPerLine", Sdf.ValueTypeNames.UIntArray, Vt.UIntArray([emitters]))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, default=TRAJECTORY)
    parser.add_argument("assets", type=Path, nargs="*", default=list(ASSETS))
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    trajectory = load_trajectory(args.trajectory.resolve())
    simulation_app = None
    try:
        # Isaac Sim 5.1 exposes pxr after Kit has initialized, even when this
        # script is launched through python.sh.
        try:
            import pxr  # noqa: F401
        except ModuleNotFoundError:
            from isaacsim import SimulationApp

            simulation_app = SimulationApp({"headless": True})
        results = [apply_profile(asset.resolve(), trajectory) for asset in args.assets]
        print(json.dumps({"updated": results}, indent=2, sort_keys=True))
    finally:
        if simulation_app is not None:
            simulation_app.close()


if __name__ == "__main__":
    main()
