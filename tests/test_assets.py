"""Repository-level contracts for the published MID360 assets."""

from __future__ import annotations

import hashlib
import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from validate_assets import validate  # noqa: E402
from apply_mid360_petal_profile import (  # noqa: E402
    CHANNEL_COUNT,
    EXPECTED_POINTS,
    EXPECTED_TRAJECTORY_SHA256,
    POINT_INTERVAL_NS,
    POINTS_PER_STATE,
    SCAN_RATE_HZ,
    STATE_COUNT,
    TRAJECTORY,
    load_trajectory,
)


class AssetTests(unittest.TestCase):
    def test_catalog_hashes(self) -> None:
        self.assertEqual(len(validate()), 8)

        catalog = json.loads((ROOT / "assets/catalog.json").read_text())
        self.assertEqual(catalog["schema_version"], 2)
        self.assertEqual(catalog["project"], "Livox_MID360_IsaacSim")
        self.assertEqual(catalog["target_isaac_sim"], "6.2")
        self.assertEqual(catalog["legacy_branch"], "isaac-sim-5.1")
        profiles = [entry["profile"] for entry in catalog["assets"].values()]
        forms = [entry["form"] for entry in catalog["assets"].values()]
        self.assertEqual(profiles.count("petal_scan"), 4)
        self.assertEqual(profiles.count("rotary_scan"), 4)
        self.assertEqual(forms.count("standalone"), 2)
        self.assertEqual(forms.count("robot"), 6)
        for entry in catalog["assets"].values():
            path = Path(entry["path"])
            self.assertIn(entry["profile"], path.parts)
            self.assertNotIn("issacsim", str(path).lower())

    def test_shared_mid360_contract(self) -> None:
        catalog = json.loads((ROOT / "assets/catalog.json").read_text())
        manifests = []
        for entry in catalog["assets"].values():
            asset_path = ROOT / entry["path"]
            manifest = json.loads((asset_path.parent / "manifest.json").read_text())
            self.assertEqual(manifest["asset"], asset_path.name)
            self.assertEqual(manifest["profile"], entry["profile"])
            self.assertEqual(manifest["target_isaac_sim"], "6.2")
            self.assertTrue((ROOT / manifest["ros2_pointcloud_publisher"]).is_file())
            self.assertTrue(entry["includes_imu"])
            self.assertEqual(manifest["imu"]["type"], "IsaacImuSensor")
            self.assertEqual(manifest["imu"]["rate_hz"], 200)
            self.assertTrue(manifest["imu"]["colocated_and_aligned_with_lidar"])
            manifests.append((entry, manifest))

        for entry, manifest in manifests:
            sensor = manifest["mid360"]
            self.assertEqual(sensor["scan_rate_hz"], SCAN_RATE_HZ)
            self.assertEqual(sensor["points_per_second"], 200000)
            if entry["profile"] == "petal_scan":
                self.assertEqual(sensor["scan_type"], "SOLID_STATE")
                self.assertEqual(sensor["emitters_per_state"], POINTS_PER_STATE)
                self.assertEqual(sensor["rtx_emitter_states"], STATE_COUNT)
                self.assertEqual(sensor["trajectory_states"], STATE_COUNT)
                self.assertEqual(sensor["channels"], CHANNEL_COUNT)
                self.assertEqual(sensor["report_rate_hz"], SCAN_RATE_HZ)
                self.assertEqual(sensor["trajectory_points"], EXPECTED_POINTS)
                self.assertEqual(sensor["trajectory_duration_s"], 4.0)
                self.assertEqual(sensor["trajectory_sha256"], EXPECTED_TRAJECTORY_SHA256)
            else:
                self.assertEqual(sensor["scan_type"], "ROTARY")
                self.assertEqual(sensor["emitters"], 40)
                self.assertEqual(sensor["report_rate_per_emitter_hz"], 5000)

        robot_motors = sorted(
            manifest["robot"]["wrist_motor"]
            for entry, manifest in manifests
            if entry["form"] == "robot"
        )
        self.assertEqual(robot_motors, ["4010", "4010", "5010", "5010", "5010", "5010"])

        mode_assets = {
            entry["mode_machine"]
            for entry in catalog["assets"].values()
            if entry.get("wrist_motor") == "5010"
        }
        self.assertEqual(mode_assets, {13, 15})

    def test_official_nonrepetitive_trajectory(self) -> None:
        trajectory = load_trajectory(TRAJECTORY)
        self.assertEqual(len(trajectory.azimuth_deg), EXPECTED_POINTS)
        self.assertAlmostEqual(min(trajectory.elevation_deg), -7.2123, places=4)
        self.assertAlmostEqual(max(trajectory.elevation_deg), 52.164, places=4)

        # Each 0.1 s state must differ. A repeated state would recreate the
        # old mechanically repeating scan instead of accumulating coverage.
        state_hashes = []
        for azimuth, elevation in trajectory.states():
            digest = hashlib.sha256()
            digest.update(repr(azimuth).encode("ascii"))
            digest.update(repr(elevation).encode("ascii"))
            state_hashes.append(digest.digest())
        self.assertEqual(len(state_hashes), STATE_COUNT)
        self.assertEqual(len(set(state_hashes)), STATE_COUNT)
        self.assertEqual((POINTS_PER_STATE - 1) * POINT_INTERVAL_NS, 99_995_000)


if __name__ == "__main__":
    unittest.main()
