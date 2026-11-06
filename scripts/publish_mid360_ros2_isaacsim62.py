"""Publish native 6.2 MID-360 clouds, IMU, TF and simulation clock.

Import after SimulationApp and enable isaacsim.ros2.bridge first. Standalone
and G1 assets share this implementation. No Action Graph is saved into USD.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import PointCloud2, PointField, Imu
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from pxr import Sdf, Usd, UsdGeom

# A Script Editor execution may not supply __file__. Find the checkout from
# the composed asset layers in that case, including referenced robot assets.
if '__file__' in globals() and (Path(__file__).resolve().parent / 'mid360_runtime_isaacsim62.py').is_file():
    sys.path.insert(0, str(Path(__file__).resolve().parent))
else:
    import omni.usd
    for layer in omni.usd.get_context().get_stage().GetUsedLayers():
        for candidate in Path(layer.realPath).parents if layer.realPath else []:
            if (candidate / 'scripts/mid360_runtime_isaacsim62.py').is_file():
                sys.path.insert(0, str(candidate / 'scripts'))
                break

from mid360_runtime_isaacsim62 import MID360Runtime

POINT_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
                       'formats': ['<f4'] * 4 + ['u1', 'u1', '<f8'],
                       'offsets': [0, 4, 8, 12, 16, 17, 18], 'itemsize': 26})


def stamp(ns):
    return Time(sec=int(ns) // 1_000_000_000, nanosec=int(ns) % 1_000_000_000)


def pointcloud_message(frame, frame_id, petal):
    points = np.empty(len(frame['xyz']), dtype=POINT_DTYPE)
    for i, name in enumerate('xyz'):
        points[name] = frame['xyz'][:, i]
    points['intensity'] = np.clip(np.rint(frame['intensity']), 0, 255)
    points['tag'] = 0
    points['line'] = frame['emitter_id'] % 4 if petal else frame['emitter_id']
    # Matches livox_ros_driver2's packed PointCloud2 timestamp convention.
    points['timestamp'] = frame['timestamp_ns'] + frame['time_offset_ns'].astype(np.float64)
    message = PointCloud2()
    message.header.frame_id = frame_id
    message.header.stamp = stamp(frame['timestamp_ns'])
    message.height = 1
    message.width = len(points)
    message.is_bigendian = False
    message.is_dense = True
    message.point_step = 26
    message.row_step = 26 * len(points)
    message.fields = [PointField(name=name, offset=POINT_DTYPE.fields[name][1],
        datatype=PointField.UINT8 if name in ('tag', 'line') else PointField.FLOAT64 if name == 'timestamp' else PointField.FLOAT32,
        count=1) for name in POINT_DTYPE.names]
    message.data = points.tobytes()
    return message


class MID360ROS2:
    def __init__(self, stage, root_path=None, *, self_check=False, frame_id='mid360_link',
                 points_topic='/mid360/points', imu_topic='/mid360/imu', publish_clock=True):
        self.stage = stage
        self.runtime = MID360Runtime(stage, root_path)
        self.frame_id = frame_id
        self.root_frame = Sdf.Path(self.runtime.root_path).name
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('mid360_sim_sensor', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.cloud_publisher = self.node.create_publisher(PointCloud2, points_topic, 100)
        self.imu_publisher = self.node.create_publisher(Imu, imu_topic, 2000)
        self.tf_publisher = self.node.create_publisher(TFMessage, '/tf', 100)
        self.clock_publisher = self.node.create_publisher(Clock, '/clock', 100) if publish_clock else None
        self.counts = {'cloud_sent': 0, 'cloud_received': 0, 'imu_sent': 0, 'imu_received': 0,
                       'tf_sent': 0, 'tf_received': 0, 'clock_sent': 0, 'clock_received': 0}
        self.layout_ok = self.imu_units_ok = self.tf_ok = self.clock_ok = True
        self.cloud_stamps = []
        self.imu_stamps = []
        self._subscription = None
        self._physics_callback = None
        self._closed = False
        self._last_clock_ns = None
        self.latest_cloud = self.latest_imu = None
        if self_check:
            self.node.create_subscription(PointCloud2, points_topic, self._check_cloud, 100)
            self.node.create_subscription(Imu, imu_topic, self._check_imu, 2000)
            self.node.create_subscription(TFMessage, '/tf', self._check_tf, 100)
            if publish_clock:
                self.node.create_subscription(Clock, '/clock', self._check_clock, 100)

    def _check_cloud(self, message):
        self.counts['cloud_received'] += 1
        points = np.frombuffer(bytes(message.data), dtype=POINT_DTYPE)
        ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        expected = [(name, POINT_DTYPE.fields[name][1], PointField.UINT8 if name in ('tag', 'line')
                     else PointField.FLOAT64 if name == 'timestamp' else PointField.FLOAT32) for name in POINT_DTYPE.names]
        self.layout_ok &= (message.header.frame_id == self.frame_id and message.point_step == 26
                           and len(message.data) == message.width * 26
                           and [(f.name, f.offset, f.datatype) for f in message.fields] == expected
                           and bool(np.all((points['timestamp'] >= ns) & (points['timestamp'] < ns + 100_000_000))))
        self.cloud_stamps.append(ns)

    def _check_imu(self, message):
        self.counts['imu_received'] += 1
        self.imu_units_ok &= message.header.frame_id == self.frame_id and message.orientation_covariance[0] == -1.
        self.imu_stamps.append(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)

    def _check_tf(self, message):
        self.counts['tf_received'] += 1
        self.tf_ok &= any(t.header.frame_id == self.root_frame and t.child_frame_id == self.frame_id for t in message.transforms)

    def _check_clock(self, message):
        self.counts['clock_received'] += 1
        self.clock_ok &= message.clock.sec >= 0 and 0 <= message.clock.nanosec < 1_000_000_000

    def publish_imu_and_clock(self, simulation_time_s):
        frame = self.runtime.read_imu(simulation_time_s)
        if frame is not None:
            self.latest_imu = frame
            message = Imu()
            message.header.frame_id = self.frame_id
            message.header.stamp = stamp(frame['timestamp_ns'])
            message.orientation_covariance[0] = -1.
            a, w = frame['linear_acceleration'], frame['angular_velocity']
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = map(float, a)
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = map(float, w)
            self.imu_publisher.publish(message)
            self.counts['imu_sent'] += 1
        ns = round(float(simulation_time_s) * 1e9)
        if self.clock_publisher is not None and ns != self._last_clock_ns:
            self.clock_publisher.publish(Clock(clock=stamp(ns)))
            self.counts['clock_sent'] += 1
            self._last_clock_ns = ns
        return frame

    def publish_cloud_and_tf(self, simulation_time_s):
        frame = self.runtime.read_cloud()
        if frame is not None:
            self.latest_cloud = frame
            self.cloud_publisher.publish(pointcloud_message(frame, self.frame_id, self.runtime.profile == 'SOLID_STATE'))
            self.counts['cloud_sent'] += 1
            parent = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.runtime.root_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            child = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.runtime.mount_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            matrix = child * parent.GetInverse()
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = self.root_frame, self.frame_id
            t.header.stamp = stamp(round(float(simulation_time_s) * 1e9))
            position = matrix.ExtractTranslation()
            quat = matrix.ExtractRotationQuat()
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, position)
            t.transform.rotation.w = float(quat.GetReal())
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = map(float, quat.GetImaginary())
            self.tf_publisher.publish(TFMessage(transforms=[t]))
            self.counts['tf_sent'] += 1
        return frame

    def spin(self):
        for _ in range(4):
            self.executor.spin_once(timeout_sec=0)

    def tick(self, simulation_time_s):
        imu = self.publish_imu_and_clock(simulation_time_s)
        cloud = self.publish_cloud_and_tf(simulation_time_s)
        self.spin()
        return cloud, imu

    def start(self):
        """Attach GUI callbacks: IMU/clock after physics; clouds after rendering."""
        import omni.kit.app
        import omni.timeline
        from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent

        def physics(_dt, _context):
            if self.stage.GetPrimAtPath(self.runtime.imu_path).IsValid():
                self.publish_imu_and_clock(SimulationManager.get_simulation_time())

        def render(_event):
            import omni.usd
            if omni.usd.get_context().get_stage() != self.stage:
                self.close()
                return
            if omni.timeline.get_timeline_interface().is_playing():
                self.publish_cloud_and_tf(SimulationManager.get_simulation_time())
                self.spin()

        self._physics_callback = SimulationManager.register_callback(physics, event=SimulationEvent.PHYSICS_POST_STEP, order=100)
        self._subscription = omni.kit.app.get_app().get_post_update_event_stream().create_subscription_to_pop(render, name='MID360 ROS2 6.2')

    def report(self):
        return {**self.counts, 'layout_ok': bool(self.layout_ok), 'imu_units_ok': bool(self.imu_units_ok),
                'tf_ok': bool(self.tf_ok), 'clock_ok': bool(self.clock_ok), 'point_step': 26,
                'point_timestamp_unit': 'absolute nanoseconds, float64', 'imu_units': 'm/s^2, rad/s',
                'cloud_period_ns_median': float(np.median(np.diff(self.cloud_stamps))) if len(self.cloud_stamps) > 1 else None,
                'imu_period_ns_median': float(np.median(np.diff(self.imu_stamps))) if len(self.imu_stamps) > 1 else None}

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._subscription = None
        if self._physics_callback is not None:
            from isaacsim.core.simulation_manager import SimulationManager
            SimulationManager.deregister_callback(self._physics_callback)
            self._physics_callback = None
        self.runtime.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


def start_mid360_ros2(stage=None, root_path=None):
    import __main__
    import omni.usd
    old = getattr(__main__, '_MID360_ROS2_RUNTIME', None)
    if old is not None:
        old.close()
    stage = stage or omni.usd.get_context().get_stage()
    root_path = root_path or getattr(__main__, 'MID360_ROBOT_ROOT_PATH', None)
    # Avoid duplicating an optional existing scene clock publisher.
    publish_clock = not stage.GetPrimAtPath('/World/Graphs/ROS_RobotState/PublishClock').IsValid()
    runtime = MID360ROS2(stage, root_path, publish_clock=publish_clock)
    runtime.start()
    __main__._MID360_ROS2_RUNTIME = runtime
    __main__.cleanup_mid360_ros2 = runtime.close
    print(f'[MID360 6.2] /mid360/points, /mid360/imu, /tf; root={runtime.runtime.root_path}; frame=mid360_link')
    return runtime


if __name__ in ('__main__', 'mid360_object_field_publisher'):
    start_mid360_ros2()
