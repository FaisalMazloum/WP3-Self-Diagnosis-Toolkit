#!/usr/bin/env python3
"""
GNSS-based virtual range sensor ("LIDAR from GNSS"), robots only.

  1. Subscribes to  <ns>/gnss  (sensor_msgs/NavSatFix, RTK)
     and broadcasts  /echi_status  (remora_interfaces/EchiStatus, shared by all robots).
  2. Subscribes to  /echi_status  and keeps the latest fix of every other robot
     in a dict keyed by robot id (entries older than `stale_timeout` are ignored).
  3. Subscribes to  <ns>/imu  (sensor_msgs/Imu) for this robot's own heading (yaw).
  4. At `scan_rate` Hz builds a scan in the same format as Unity's RemoraLIDAR
     (communication_interfaces/RangeData, +inf = no hit) and publishes it on
     <ns>/<output_topic>.

Each neighbour is modelled as a disc of radius `robot_radius`, and every scan ray
is intersected with it, so a neighbour spans several bins like with a real LIDAR
(ros2swarm's identify_robots() relies on that via min_width).

Angle convention (use_yaw = true, default): angles are relative to the robot's
heading, like a real LIDAR. 0 = straight ahead, and
  angle_direction = 'cw'  -> increasing clockwise (same as Unity's RemoraLIDAR, 90 deg = right)
  angle_direction = 'ccw' -> increasing counter-clockwise (ROS REP-103, 90 deg = left)

Heading: yaw is taken from the IMU orientation and converted to ENU
(0 = East, CCW) with   yaw_enu = yaw_imu + imu_yaw_offset_deg.
  Unity IMU (UnitySensors, To<FLU>): yaw 0 = North (Unity +Z)  -> offset 90
  REP-103 ENU IMU (typical real robot):  yaw 0 = East           -> offset 0

use_yaw = false: no IMU needed, angles are in the WORLD (ENU) frame
(0 = East, CCW) and do not follow the robot's heading.

No shared map origin is needed: offsets to neighbours are computed with a local
tangent-plane approximation around this robot's own fix (sub-millimetre error at
tens of metres).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from communication_interfaces.msg import RangeData
from remora_interfaces.msg import EchiStatus

# WGS84
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def enu_offset(lat0, lon0, lat, lon):
    """East/North offset (m) of (lat, lon) relative to (lat0, lon0), all in degrees."""
    phi0 = math.radians(lat0)
    s = math.sin(phi0)
    w = 1.0 - WGS84_E2 * s * s
    r_prime_vertical = WGS84_A / math.sqrt(w)
    r_meridian = WGS84_A * (1.0 - WGS84_E2) / (w * math.sqrt(w))
    d_east = math.radians(lon - lon0) * r_prime_vertical * math.cos(phi0)
    d_north = math.radians(lat - lat0) * r_meridian
    return d_east, d_north


def quaternion_yaw(q):
    """Yaw (rad) about +z of a geometry_msgs/Quaternion, robust to roll/pitch."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GnssRangeSensor(Node):

    def __init__(self):
        super().__init__('gnss_range_sensor')

        # Defaults mirror RemoraLIDAR.cs (0..360 deg, 1 deg step, 10 m, 5 Hz)
        self.declare_parameter('robot_id', '')              # '' -> use the node namespace
        self.declare_parameter('status_topic', '/echi_status')
        self.declare_parameter('output_topic', 'gnss_range_data')
        self.declare_parameter('angle_min_deg', 0.0)
        self.declare_parameter('angle_max_deg', 259.0)
        self.declare_parameter('angle_increment_deg', 1.0)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('robot_radius', 0.225)       # m, radius of the disc each neighbour occupies (Remora: 0.45 m diameter)
        self.declare_parameter('scan_rate', 5.0)            # Hz
        self.declare_parameter('status_rate', 5.0)          # Hz
        self.declare_parameter('stale_timeout', 1.0)        # s, drop neighbours / own fix / imu older than this
        self.declare_parameter('use_yaw', True)             # False -> world-frame (ENU) angles, no IMU needed
        self.declare_parameter('angle_direction', 'cw')     # 'cw' = RemoraLIDAR convention, 'ccw' = REP-103
        self.declare_parameter('imu_yaw_offset_deg', 0.0)   # yaw_enu = yaw_imu + offset (Unity IMU: 90)

        gp = self.get_parameter
        self.robot_id = gp('robot_id').value or self.get_namespace().strip('/')
        if not self.robot_id:
            raise RuntimeError('No robot id: run the node in a namespace or set the robot_id parameter')

        self.range_max = float(gp('range_max').value)
        self.robot_radius = float(gp('robot_radius').value)
        self.stale_timeout = float(gp('stale_timeout').value)
        self.use_yaw = bool(gp('use_yaw').value)
        direction = str(gp('angle_direction').value).lower()
        if direction not in ('cw', 'ccw'):
            raise RuntimeError(f"angle_direction must be 'cw' or 'ccw', got '{direction}'")
        self.clockwise = direction == 'cw'
        self.imu_yaw_offset = math.radians(float(gp('imu_yaw_offset_deg').value))

        # Precomputed scan geometry (same bin count as RemoraLIDAR: ceil(span / inc) + 1)
        a_min = float(gp('angle_min_deg').value)
        a_max = float(gp('angle_max_deg').value)
        a_inc = float(gp('angle_increment_deg').value)
        n_rays = int(math.ceil((a_max - a_min) / a_inc)) + 1
        self.angles = np.radians(a_min + a_inc * np.arange(n_rays))   # float64, also RangeData.angles
        self.angles_list = self.angles.tolist()
        self.frame_id = f'{self.robot_id}/gnss_scan' if self.use_yaw else f'{self.robot_id}/enu'

        # State
        self.own_fix = None          # (lat, lon, alt, stamp_msg, receive_time)
        self.own_yaw = None          # (yaw_enu_rad, receive_time)
        self.neighbours = {}         # id -> (lat, lon, alt, receive_time)

        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,                # room for a burst from ~30 robots between spins
        )
        status_topic = gp('status_topic').value

        self.create_subscription(NavSatFix, 'gnss', self._on_gnss, qos_profile_sensor_data)
        if self.use_yaw:
            self.create_subscription(Imu, 'imu', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(EchiStatus, status_topic, self._on_status, status_qos)
        self.status_pub = self.create_publisher(EchiStatus, status_topic, status_qos)
        self.scan_pub = self.create_publisher(RangeData, gp('output_topic').value, 10)

        self.create_timer(1.0 / float(gp('status_rate').value), self._publish_status)
        self.create_timer(1.0 / float(gp('scan_rate').value), self._publish_scan)

        self.get_logger().info(
            f'[{self.robot_id}] gnss -> {status_topic} -> '
            f'{self.resolve_topic_name(gp("output_topic").value)} '
            f'({n_rays} rays, range_max={self.range_max} m, robot_radius={self.robot_radius} m, '
            + (f'body frame {direction.upper()}, imu_yaw_offset={math.degrees(self.imu_yaw_offset):g} deg)'
               if self.use_yaw else 'world ENU frame, no yaw)')
        )

    # ------------------------------------------------------------------ inputs

    def _on_gnss(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        self.own_fix = (msg.latitude, msg.longitude, msg.altitude, msg.header.stamp, self._now())

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return   # orientation not provided
        self.own_yaw = (quaternion_yaw(q) + self.imu_yaw_offset, self._now())

    def _on_status(self, msg: EchiStatus):
        if msg.id == self.robot_id or not msg.id:
            return
        self.neighbours[msg.id] = (msg.latitude, msg.longitude, msg.altitude, self._now())

    # ----------------------------------------------------------------- outputs

    def _publish_status(self):
        if not self._own_fix_fresh():
            return
        lat, lon, alt, stamp, _ = self.own_fix
        msg = EchiStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = self.robot_id
        msg.id = self.robot_id
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        self.status_pub.publish(msg)

    def _publish_scan(self):
        if not self._own_fix_fresh():
            self.get_logger().warn('No fresh own GNSS fix, not publishing scan',
                                   throttle_duration_sec=5.0)
            return

        now = self._now()
        if self.use_yaw:
            if self.own_yaw is None or now - self.own_yaw[1] > self.stale_timeout:
                self.get_logger().warn('No fresh IMU yaw, not publishing scan',
                                       throttle_duration_sec=5.0)
                return
            yaw = self.own_yaw[0]

        lat0, lon0 = self.own_fix[0], self.own_fix[1]
        r = self.robot_radius
        ranges = np.full(self.angles.shape, np.inf)

        for rid, (lat, lon, _alt, t_rx) in list(self.neighbours.items()):
            if now - t_rx > self.stale_timeout:
                del self.neighbours[rid]
                continue

            dx, dy = enu_offset(lat0, lon0, lat, lon)
            d = math.hypot(dx, dy)
            if d - r > self.range_max:
                continue
            if d <= r:
                self.get_logger().warn(f'{rid} is within robot_radius ({d:.2f} m), skipped',
                                       throttle_duration_sec=2.0)
                continue

            # Direction of the neighbour in the scan's angle convention
            bearing = math.atan2(dy, dx)               # ENU: 0 = East, CCW
            if self.use_yaw:
                bearing = (yaw - bearing) if self.clockwise else (bearing - yaw)

            # Rays hitting the disc: |angle - bearing| <= asin(r / d)
            delta = (self.angles - bearing + math.pi) % (2.0 * math.pi) - math.pi
            mask = np.abs(delta) <= math.asin(r / d)
            if not mask.any():
                continue   # disc thinner than one angular bin at this distance

            # Distance along each ray to the near edge of the disc
            dm = delta[mask]
            perp = d * np.sin(dm)
            hit = d * np.cos(dm) - np.sqrt(np.maximum(r * r - perp * perp, 0.0))
            hit[hit > self.range_max] = np.inf
            ranges[mask] = np.minimum(ranges[mask], hit)   # closer robots occlude farther ones

        msg = RangeData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.ranges = ranges.astype(np.float32).tolist()
        msg.angles = self.angles_list
        self.scan_pub.publish(msg)

    # ----------------------------------------------------------------- helpers

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _own_fix_fresh(self):
        return self.own_fix is not None and self._now() - self.own_fix[4] <= self.stale_timeout


def main(args=None):
    rclpy.init(args=args)
    node = GnssRangeSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
