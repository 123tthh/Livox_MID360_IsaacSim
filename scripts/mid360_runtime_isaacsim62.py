"""Read the bundled native RTX sensors after creating SimulationApp (6.2)."""
from __future__ import annotations

import numpy as np
from pxr import Usd
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data

LIDAR_SUFFIX = "/torso_link/mid360_link/mid360_native_approx"
CORE = "omni:sensor:Core:"


def find_lidar(stage, root_path=None):
    candidates = [p for p in stage.Traverse() if p.GetTypeName() == "OmniLidar"
                  and str(p.GetPath()).endswith(LIDAR_SUFFIX)
                  and (root_path is None or str(p.GetPath()) == str(root_path).rstrip('/') + LIDAR_SUFFIX)]
    if len(candidates) != 1:
        raise RuntimeError(f"Expected one MID-360, found {len(candidates)}; supply root_path explicitly")
    return candidates[0]


def as_numpy(value):
    return value.numpy() if hasattr(value, 'numpy') else np.asarray(value)


class MID360Runtime:
    def __init__(self, stage, root_path=None, *, with_imu=True):
        self.stage = stage
        prim = find_lidar(stage, root_path)
        self.lidar_path = str(prim.GetPath())
        self.root_path = self.lidar_path[:-len(LIDAR_SUFFIX)]
        self.mount_path = self.root_path + '/torso_link/mid360_link'
        self.imu_path = self.root_path + '/torso_link/mid360_imu'
        self.profile = str(prim.GetAttribute(CORE + 'scanType').Get())
        self.near = float(prim.GetAttribute(CORE + 'nearRangeM').Get())
        self.far = float(prim.GetAttribute(CORE + 'farRangeM').Get())
        self.emitter_count = int(prim.GetAttribute(CORE + 'numberOfEmitters').Get())
        self.last_cloud_ns = self.last_imu_ns = None
        self.pattern_epoch_ns = None
        self.pattern_initial_state = 0
        self.directions = None
        if self.profile == 'SOLID_STATE':
            states = []
            for i in range(1, 41):
                prefix = CORE + f'emitterState:s{i:03d}:'
                az = -np.radians(np.asarray(prim.GetAttribute(prefix + 'azimuthDeg').Get(), dtype=np.float32))
                el = np.radians(np.asarray(prim.GetAttribute(prefix + 'elevationDeg').Get(), dtype=np.float32))
                if len(az) != 20000 or len(el) != 20000:
                    raise RuntimeError('Use the native 40-state 6.2 assets from main; 5.1 assets are on isaac-sim-5.1')
                states.append(np.stack([np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)], axis=-1))
            self.directions = np.stack(states)
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            # A component reference does not supply its parent stage's time
            # range. Kit 6.2 otherwise loops an empty timeline at one frame.
            if stage.GetEndTimeCode() <= stage.GetStartTimeCode():
                stage.SetStartTimeCode(0.)
                stage.SetEndTimeCode(stage.GetTimeCodesPerSecond() * 3600.)
            self.lidar = Lidar(self.lidar_path, accumulate_outputs=None, aux_output_level='FULL', reset_xform_op_properties=False)
            self.sensor = LidarSensor(self.lidar, annotators=['generic-model-output'])
            self.imu = None
            if with_imu:
                from isaacsim.sensors.experimental.physics import IMU, IMUSensor
                self.imu = IMUSensor(IMU(self.imu_path, reset_xform_op_properties=False))

    def read_cloud(self):
        data, _ = self.sensor.get_data('generic-model-output')
        if data is None:
            return None
        raw = as_numpy(data).copy()
        if raw.nbytes < 28 or int(np.frombuffer(raw.tobytes()[:4], dtype='<u4')[0]) != 0x4E474D4F:
            return None
        g = parse_generic_model_output_data(raw)
        ns = int(g.timestampNs)
        if ns == self.last_cloud_ns:
            return None
        if self.last_cloud_ns is not None and ns < self.last_cloud_ns:
            self.pattern_epoch_ns = None
        self.last_cloud_ns = ns
        xyz = np.stack([g.x, g.y, g.z], axis=1).astype(np.float32, copy=True)
        ids = np.asarray(g.emitterId, dtype=np.int64).copy()
        offsets = np.asarray(g.timeOffsetNs, dtype=np.int64).copy()
        if len(ids) != len(xyz) or np.any((ids < 0) | (ids >= self.emitter_count)):
            raise RuntimeError('Invalid RTX emitter IDs')
        ranges = np.linalg.norm(xyz, axis=1)
        valid = (np.isfinite(xyz).all(axis=1) & ((np.asarray(g.flags) & 64) != 0)
                 & (ranges >= self.near-1e-5) & (ranges <= self.far+1e-5))
        state = 0
        # In local 6.2 the raw tickStates label is always zero while native
        # directions advance. Establish phase from XYZ once, then use the
        # acquisition timestamps; never infer phase from render callback count.
        if self.directions is not None:
            if self.pattern_epoch_ns is None and np.any(valid):
                sample = np.flatnonzero(valid)[::max(1, int(valid.sum()) // 256)]
                observed = xyz[sample] / ranges[sample, None]
                error = np.mean(np.linalg.norm(observed[None] - self.directions[:, ids[sample]], axis=2), axis=1)
                self.pattern_initial_state = int(error.argmin())
                self.pattern_epoch_ns = ns
            if self.pattern_epoch_ns is not None:
                state = (self.pattern_initial_state + round((ns-self.pattern_epoch_ns)/100_000_000)) % 40
        select = np.flatnonzero(valid)[np.argsort(offsets[valid], kind='stable')]
        return {'timestamp_ns': ns, 'xyz': xyz[select],
                'intensity': np.asarray(g.scalar, dtype=np.float32)[select].copy(),
                'emitter_id': ids[select], 'time_offset_ns': offsets[select],
                'state_id': np.full(len(select), state, dtype=np.int64),
                'raw_count': len(xyz), 'raw_tick_states': np.unique(g.tickStates).tolist()}

    def read_imu(self, simulation_time_s):
        if self.imu is None:
            return None
        reading = self.imu.get_sensor_reading(read_gravity=True)
        if not reading.is_valid:
            return None
        frame = self.imu.get_data(read_gravity=True)
        ns = round(float(simulation_time_s) * 1e9)
        if ns == self.last_imu_ns:
            return None
        self.last_imu_ns = ns
        return {'timestamp_ns': ns,
                'linear_acceleration': np.array(frame['linear_acceleration'], copy=True),
                'angular_velocity': np.array(frame['angular_velocity'], copy=True)}

    def close(self):
        self.sensor.detach_annotators(['generic-model-output'])
