#!/usr/bin/env python3
"""Measure native scans, per-point timing, IMU and optional ROS 2 in a closed room.

Run with the 6.2 Python interpreter. --all runs a fresh process per asset so
RTX/PhysX state cannot leak across sensor instances. Reports survive Kit exit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
import traceback
from datetime import datetime
from zoneinfo import ZoneInfo
from importlib.metadata import version
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CATALOG = json.loads((ROOT / 'assets/catalog.json').read_text())


def args_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset-id', choices=tuple(CATALOG['assets']), default='mid360-petal-standalone')
    parser.add_argument('--all', action='store_true')
    parser.add_argument('--ros2', action='store_true')
    parser.add_argument('--callbacks', action='store_true', help='test Script Editor physics/render callbacks (requires --ros2)')
    parser.add_argument('--resume', action='store_true', help='reuse PASS reports only when asset hash and run mode match')
    parser.add_argument('--frames', type=int)
    parser.add_argument('--output', type=Path)
    return parser.parse_args()


def validate_all(args):
    reports = {}
    output = args.output or ROOT / 'docs/validation/isaacsim62_runtime.json'
    for asset_id in CATALOG['assets']:
        path = output.parent / 'runtime' / (asset_id + '.json')
        if args.resume and path.is_file():
            cached = json.loads(path.read_text())
            asset = ROOT / CATALOG['assets'][asset_id]['path']
            if (cached.get('status') == 'PASS' and cached.get('asset_sha256') == hashlib.sha256(asset.read_bytes()).hexdigest()
                    and cached.get('callback_mode') == args.callbacks and ('ros2' in cached) == args.ros2
                    and (args.frames is None or cached.get('checks', {}).get('frames') == args.frames)):
                reports[asset_id] = cached
                print(f'RUNTIME_CACHED_PASS={asset_id}', flush=True)
                continue
        command = [sys.executable, str(Path(__file__).resolve()), '--asset-id', asset_id, '--output', str(path)]
        if args.ros2:
            command.append('--ros2')
        if args.callbacks:
            command.append('--callbacks')
        if args.frames:
            command += ['--frames', str(args.frames)]
        log = Path('/tmp') / ('mid360_62_' + asset_id + '.log')
        with log.open('w') as stream:
            process = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
        report = json.loads(path.read_text()) if path.is_file() else {'status': 'FAIL', 'error': 'no report'}
        reports[asset_id] = report
        print(f'RUNTIME_{report["status"]}={asset_id} log={log}', flush=True)
        if process.returncode or report['status'] != 'PASS':
            print(log.read_text()[-6000:], flush=True)
            break
    result = {'status': 'PASS' if len(reports) == 8 and all(r['status'] == 'PASS' for r in reports.values()) else 'FAIL',
              'verified_date': datetime.now(ZoneInfo('Asia/Shanghai')).date().isoformat(), 'isaac_sim': version('isaacsim'),
              'ros2_tested': args.ros2, 'callback_mode': args.callbacks, 'assets': reports,
              'scope': 'Native sensor/runtime and ROS transport in a static closed room; robot joints anchored and locked for the fixture. Not locomotion or SLAM performance.'}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + '\n')
    return 0 if result['status'] == 'PASS' else 1


def validate_one(args):
    from isaacsim import SimulationApp
    output = args.output or ROOT / 'docs/validation/runtime' / (args.asset_id + '.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {'status': 'RUNNING', 'asset_id': args.asset_id, 'isaac_sim': version('isaacsim')}
    output.write_text(json.dumps(report) + '\n')
    app = SimulationApp({'headless': True, 'multi_gpu': False,
                         'extra_args': ['--/renderer/raytracingMotion/enabled=true',
                                        '--/renderer/raytracingMotion/enabledForHydraEngines=0,1,2,3']})
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdLux
    import omni.usd
    from isaacsim.core.api import SimulationContext
    from mid360_runtime_isaacsim62 import MID360Runtime
    from apply_mid360_petal_profile import load_trajectory

    sim = runtime = bridge = None
    try:
        assert version('isaacsim').startswith('6.2.'), 'Use Isaac Sim 6.2 for main'
        if args.ros2:
            from isaacsim.core.utils.extensions import enable_extension
            enable_extension('isaacsim.ros2.bridge')
            for _ in range(6):
                app.update()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdGeom.SetStageUpAxis(stage, 'Z')
        world = UsdGeom.Xform.Define(stage, '/World')
        stage.SetDefaultPrim(world.GetPrim())
        root = UsdGeom.Xform.Define(stage, '/World/SensorRig')
        spec = CATALOG['assets'][args.asset_id]
        asset = ROOT / spec['path']
        root.GetPrim().GetReferences().AddReference(str(asset))
        translation = root.GetPrim().GetAttribute('xformOp:translate')
        if translation.IsValid():
            translation.Set(Gf.Vec3d(0, 0, 1.))
        else:
            root.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.))
        if spec['form'] == 'robot':
            for prim in Usd.PrimRange(root.GetPrim()):
                if prim.IsA(UsdPhysics.RevoluteJoint):
                    joint = UsdPhysics.RevoluteJoint(prim)
                    joint.CreateLowerLimitAttr(0.)
                    joint.CreateUpperLimitAttr(0.)
            torso = stage.GetPrimAtPath('/World/SensorRig/torso_link')
            pose = UsdGeom.Xformable(torso).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            anchor = UsdPhysics.FixedJoint.Define(stage, '/World/FixtureAnchor')
            anchor.CreateBody1Rel().SetTargets([torso.GetPath()])
            anchor.CreateLocalPos0Attr(Gf.Vec3f(pose.ExtractTranslation()))
            anchor.CreateLocalRot0Attr(Gf.Quatf(pose.ExtractRotationQuat()))
        # Inner bounds: X/Y +/-4 m, Z 0..4 m.
        boxes = [((0, 0, -.05), (8.2, 8.2, .1)), ((0, 0, 4.05), (8.2, 8.2, .1)),
                 ((4.05, 0, 2), (.1, 8.2, 4.2)), ((-4.05, 0, 2), (.1, 8.2, 4.2)),
                 ((0, 4.05, 2), (8.2, .1, 4.2)), ((0, -4.05, 2), (8.2, .1, 4.2))]
        for i, (center, size) in enumerate(boxes):
            cube = UsdGeom.Cube.Define(stage, f'/World/Room/Box{i}')
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*center))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1200)
        sim = SimulationContext(physics_dt=.005, rendering_dt=.005, stage_units_in_meters=1.)
        if args.ros2:
            from publish_mid360_ros2_isaacsim62 import MID360ROS2
            bridge = MID360ROS2(stage, '/World/SensorRig', self_check=True)
            runtime = bridge.runtime
        else:
            runtime = MID360Runtime(stage, '/World/SensorRig')
        sim.initialize_physics()
        sim.play()
        if args.callbacks:
            assert bridge is not None
            bridge.start()
        frames, imus = [], []
        last_cloud = last_imu = None
        discard = 2
        target = args.frames or (45 if spec['profile'] == 'petal_scan' else 12)
        for _ in range((target + 10) * 25):
            sim.step(render=True)
            if bridge:
                if args.callbacks:
                    cloud, imu = bridge.latest_cloud, bridge.latest_imu
                else:
                    cloud, imu = bridge.tick(sim.current_time)
            else:
                cloud, imu = runtime.read_cloud(), runtime.read_imu(sim.current_time)
            if imu is not None and imu['timestamp_ns'] != last_imu:
                last_imu = imu['timestamp_ns']
                imus.append(imu)
            if cloud is not None and cloud['timestamp_ns'] != last_cloud and len(cloud['xyz']):
                last_cloud = cloud['timestamp_ns']
                if discard:
                    discard -= 1
                else:
                    frames.append(cloud)
            if len(frames) == target:
                break
        assert len(frames) == target, f'Only {len(frames)} usable scans'
        periods = np.diff([f['timestamp_ns'] for f in frames])
        assert np.all(periods == 100_000_000), np.unique(periods)
        assert min(len(f['xyz']) for f in frames) > 10000, [(len(f['xyz']), f['raw_count']) for f in frames]
        checks = {'frames': len(frames), 'cloud_period_ns': 100_000_000,
                  'valid_points_min_max': [min(len(f['xyz']) for f in frames), max(len(f['xyz']) for f in frames)],
                  'raw_points_min_max': [min(f['raw_count'] for f in frames), max(f['raw_count'] for f in frames)]}
        report['checks'] = checks
        if spec['profile'] == 'petal_scan':
            trajectory = load_trajectory()
            az = np.radians(np.asarray(trajectory.azimuth_deg).reshape(40, 20000))
            el = np.radians(np.asarray(trajectory.elevation_deg).reshape(40, 20000))
            official = np.stack([np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)], axis=-1)
            errors, states = [], []
            for frame in frames:
                xyz, ids = frame['xyz'], frame['emitter_id']
                observed = xyz / np.linalg.norm(xyz, axis=1)[:, None]
                # Search every state independently rather than trusting runtime labels.
                sample = np.arange(0, len(xyz), 100)
                match = np.mean(np.linalg.norm(observed[None, sample] - official[:, ids[sample]], axis=2), axis=1)
                state = int(match.argmin())
                assert np.all(frame['state_id'] == state)
                errors.append(float(np.max(np.linalg.norm(observed - official[state, ids], axis=1))))
                assert np.array_equal(frame['time_offset_ns'], ids * 5000)
                states.append(state)
            assert max(errors) < 1e-5, max(errors)
            assert all(b == (a+1) % 40 for a, b in zip(states, states[1:]))
            if target >= 40:
                assert set(states) == set(range(40))
            checks.update(observed_states=states, all_40_states_verified_from_xyz=target >= 40,
                          max_unit_direction_error=max(errors), per_point_interval_ns=5000,
                          raw_tick_states=frames[0]['raw_tick_states'])
        else:
            checks['unique_channels'] = sorted(set(int(v) for f in frames for v in f['emitter_id']))
            # Robot geometry can occlude entire downward-looking channels.
            # Require all 40 only for the unobstructed standalone fixture.
            if spec['form'] == 'standalone':
                assert checks['unique_channels'] == list(range(40)), checks['unique_channels']
        # IMU specific force follows its own (possibly inverted) sensor axes.
        samples = [f for f in imus if f['timestamp_ns'] > 500_000_000]
        assert len(samples) > 30
        imu_periods = np.diff([f['timestamp_ns'] for f in samples])
        assert np.max(np.abs(imu_periods - 5_000_000)) <= 2
        acceleration = np.stack([f['linear_acceleration'] for f in samples])
        gyro = np.stack([f['angular_velocity'] for f in samples])
        pose = UsdGeom.Xformable(stage.GetPrimAtPath(runtime.imu_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        expected = np.asarray(pose.GetInverse().TransformDir(Gf.Vec3d(0, 0, 9.81)))
        tolerance = .05 if spec['form'] == 'robot' else .001
        assert np.max(np.abs(acceleration-expected)) < tolerance, acceleration.mean(axis=0)
        assert np.max(np.abs(gyro)) < tolerance
        checks['imu'] = {'samples': len(samples), 'period_ns': float(np.median(imu_periods)),
                         'mean_acceleration_m_s2': acceleration.mean(axis=0).tolist(),
                         'expected_acceleration_m_s2': expected.tolist(), 'max_gyro_rad_s': float(np.abs(gyro).max())}
        if bridge:
            sim.pause()
            deadline = time.monotonic() + 2.
            while time.monotonic() < deadline:
                bridge.spin()
                if all(bridge.counts[k+'_received'] == bridge.counts[k+'_sent'] for k in ('cloud', 'imu', 'tf', 'clock')):
                    break
                time.sleep(.01)
            ros = bridge.report()
            assert all(ros[k+'_received'] == ros[k+'_sent'] and ros[k+'_sent'] > 0 for k in ('cloud', 'imu', 'tf', 'clock')), ros
            assert all(ros[k] for k in ('layout_ok', 'imu_units_ok', 'tf_ok', 'clock_ok'))
            report['ros2'] = ros
        report.update(status='PASS', checks=checks, asset=spec['path'],
                      asset_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(), callback_mode=args.callbacks)
        print('RUNTIME_PASS=' + args.asset_id, flush=True)
    except BaseException as error:
        report.update(status='FAIL', error=repr(error))
        traceback.print_exc()
    finally:
        output.write_text(json.dumps(report, indent=2) + '\n')
        if bridge:
            bridge.close()
        elif runtime:
            runtime.close()
        if sim:
            sim.stop()
        app.close()
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    args = args_parser()
    raise SystemExit(validate_all(args) if args.all else validate_one(args))
