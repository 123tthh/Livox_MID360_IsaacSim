#!/usr/bin/env python3
"""Open the fixed G1 object field and run native 6.2 MID-360 sensors."""
from __future__ import annotations

import argparse
import json
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENES = {
    'petal': ROOT / 'tests/scenes/object_field/MID360_G1_5010_Petal_Object_Field.usda',
    'rotary': ROOT / 'tests/scenes/object_field/MID360_G1_5010_Rotary_Object_Field.usda',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=tuple(SCENES), default='petal')
    parser.add_argument('--duration', type=float, default=0., help='wall seconds; 0 runs until the window closes')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--no-ros', action='store_true')
    parser.add_argument('--report', type=Path, default=Path('/tmp/mid360_object_field_62_report.json'))
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless, 'multi_gpu': False, 'width': 1440, 'height': 900,
                         'extra_args': ['--/renderer/raytracingMotion/enabled=true',
                                        '--/renderer/raytracingMotion/enabledForHydraEngines=0,1,2,3']})
    import omni.usd
    from isaacsim.core.api import SimulationContext
    from mid360_runtime_isaacsim62 import MID360Runtime

    sim = runtime = bridge = None
    report = {'status': 'RUNNING', 'profile': args.profile, 'points_topic': '/mid360/points'}
    try:
        if not args.no_ros:
            from isaacsim.core.utils.extensions import enable_extension
            enable_extension('isaacsim.ros2.bridge')
            for _ in range(6):
                app.update()
        context = omni.usd.get_context()
        context.open_stage(str(SCENES[args.profile]))
        for _ in range(12):
            app.update()
        stage = context.get_stage()
        assert stage.GetPrimAtPath('/World/G1').IsValid()
        if not args.headless:
            from omni.kit.viewport.utility import get_active_viewport
            viewport = get_active_viewport()
            if viewport:
                viewport.set_active_camera('/World/ValidationCamera')
        sim = SimulationContext(physics_dt=.005, rendering_dt=.005, stage_units_in_meters=1.)
        if args.no_ros:
            runtime = MID360Runtime(stage, '/World/G1')
        else:
            from publish_mid360_ros2_isaacsim62 import MID360ROS2
            bridge = MID360ROS2(stage, '/World/G1')
            runtime = bridge.runtime
        sim.initialize_physics()
        sim.play()
        print(f'VALIDATION_READY profile={args.profile} root=/World/G1 frame=mid360_link joints_locked=29', flush=True)
        start = time.monotonic()
        clouds = points = 0
        while app.is_running():
            sim.step(render=True)
            if sim.is_playing():
                if bridge:
                    cloud, _ = bridge.tick(sim.current_time)
                else:
                    cloud = runtime.read_cloud()
                if cloud is not None:
                    clouds += 1
                    points += len(cloud['xyz'])
            if args.duration > 0. and time.monotonic() - start >= args.duration:
                break
        if args.duration > 0.:
            assert clouds > 0 and points > 0, 'No live RTX points produced'
        report.update(status='PASS', clouds=clouds, points=points, simulation_time_s=sim.current_time)
        if bridge:
            report['ros2'] = bridge.report()
        print(f'OBJECT_FIELD_PASS={args.profile} clouds={clouds} points={points}', flush=True)
    except BaseException as error:
        report.update(status='FAIL', error=repr(error))
        traceback.print_exc()
    finally:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
        if bridge:
            bridge.close()
        elif runtime:
            runtime.close()
        if sim:
            sim.stop()
        app.close()
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
