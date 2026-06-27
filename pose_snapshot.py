#!/usr/bin/env python3
"""Snapshot every pose source in the aisle stack at one instant for cross-checking
transforms. Subscribes to all pose-bearing topics, reads /mavros/state (PASSIVE -
never commands arming or mode), waits until each source has produced a message (or
times out), then prints them together with the TF chain and consistency hints.

Run with both workspaces sourced (pos_cmd is quadrotor_msgs from ego_ws):
  source /opt/ros/humble/setup.bash && source ~/jetvio_ws/install/setup.bash && source ~/ego_ws/install/setup.bash
  python3 ~/jetvio_ws/pose_snapshot.py
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import PositionTarget, State
from tf2_ros import Buffer, TransformListener

try:
    from quadrotor_msgs.msg import PositionCommand
    HAVE_QUAD = True
except Exception:
    HAVE_QUAD = False


def yaw(q):
    return math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))


def fmt_xyzq(p, q):
    return f"pos[{p.x:+.3f} {p.y:+.3f} {p.z:+.3f}]  yaw {yaw(q):+7.1f}deg"


class PoseSnapshot(Node):
    def __init__(self):
        super().__init__('pose_snapshot')
        self.msgs = {}
        self.cuvslam_z = None      # for the z-datum comparison
        self.local_z = None
        self.state = None          # /mavros/state, read-only

        subs = [
            ('cuVSLAM odom            (odom->camera via cuVSLAM)', Odometry, '/visual_slam/tracking/odometry'),
            ('EV relay out            (/mavros/vision_pose)',     PoseStamped, '/mavros/vision_pose/pose'),
            ('EGO camera_pose         (/ego/camera_pose)',        PoseStamped, '/ego/camera_pose'),
            ('MAVROS local_position   (PX4 EKF2 estimate)',       PoseStamped, '/mavros/local_position/pose'),
            ('vo_pose_covariance      (cuVSLAM cov)',             PoseWithCovarianceStamped, '/visual_slam/tracking/vo_pose_covariance'),
        ]
        for label, typ, topic in subs:
            self.create_subscription(typ, topic,
                                     lambda m, l=label, t=typ: self._cb(l, m, t),
                                     qos_profile_sensor_data)

        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local',
                                 self._cb_target, qos_profile_sensor_data)
        # PASSIVE read of vehicle state - subscribe only, no command publishers anywhere.
        self.create_subscription(State, '/mavros/state', self._cb_state, qos_profile_sensor_data)
        if HAVE_QUAD:
            self.create_subscription(PositionCommand, '/ego/pos_cmd',
                                     self._cb_poscmd, qos_profile_sensor_data)

        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

        self.expected = set(l for l, _, _ in subs)
        self.expected.add('setpoint_raw/local      (bridge -> PX4)')
        if HAVE_QUAD:
            self.expected.add('EGO pos_cmd             (planner setpoint, odom)')

        self.t0 = self.get_clock().now()
        self.create_timer(0.2, self._check)

    def _cb(self, label, m, typ):
        if typ is Odometry:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            self.cuvslam_z = p.z
            self.msgs[label] = f"{fmt_xyzq(p, q)}   frame={m.header.frame_id} child={m.child_frame_id}"
        elif typ is PoseWithCovarianceStamped:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            c = m.pose.covariance
            self.msgs[label] = (f"{fmt_xyzq(p, q)}   frame={m.header.frame_id}   "
                                f"orient_cov[{c[21]:.4f} {c[28]:.4f} {c[35]:.4f}]")
        else:
            p, q = m.pose.position, m.pose.orientation
            if 'local_position' in label:
                self.local_z = p.z
            self.msgs[label] = f"{fmt_xyzq(p, q)}   frame={m.header.frame_id}"

    def _cb_target(self, m):
        label = 'setpoint_raw/local      (bridge -> PX4)'
        frames = {1: 'LOCAL_NED', 7: 'LOCAL_OFFSET_NED', 8: 'BODY_NED', 9: 'BODY_OFFSET_NED'}
        self.msgs[label] = (f"pos[{m.position.x:+.3f} {m.position.y:+.3f} {m.position.z:+.3f}]  "
                            f"yaw {math.degrees(m.yaw):+7.1f}deg  "
                            f"vel[{m.velocity.x:+.2f} {m.velocity.y:+.2f} {m.velocity.z:+.2f}]  "
                            f"coord={frames.get(m.coordinate_frame, m.coordinate_frame)} mask={m.type_mask}")

    def _cb_state(self, m):
        self.state = m

    def _cb_poscmd(self, m):
        label = 'EGO pos_cmd             (planner setpoint, odom)'
        self.msgs[label] = (f"pos[{m.position.x:+.3f} {m.position.y:+.3f} {m.position.z:+.3f}]  "
                            f"yaw {math.degrees(m.yaw):+7.1f}deg  "
                            f"vel[{m.velocity.x:+.2f} {m.velocity.y:+.2f} {m.velocity.z:+.2f}]")

    def _tf(self, parent, child):
        try:
            t = self.buf.lookup_transform(parent, child, Time()).transform
            return (f"  {parent} -> {child}: "
                    f"pos[{t.translation.x:+.3f} {t.translation.y:+.3f} {t.translation.z:+.3f}]  "
                    f"yaw {yaw(t.rotation):+7.1f}deg")
        except Exception as e:
            return f"  {parent} -> {child}: (unavailable: {str(e).splitlines()[0][:60]})"

    def _check(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if set(self.msgs.keys()) >= self.expected or elapsed > 6.0:
            self._report(elapsed)
            rclpy.shutdown()

    def _report(self, elapsed):
        print("\n" + "=" * 78)
        print(f"POSE SNAPSHOT  (collected over {elapsed:.1f}s)")
        print("=" * 78)

        print("\n-- VEHICLE STATE (read-only, no commands issued) --")
        if self.state is not None:
            print(f"  armed={self.state.armed}  mode={self.state.mode}  "
                  f"connected={self.state.connected}")
        else:
            print("  (no /mavros/state - MAVROS not up?)")

        print("\n-- TOPICS --")
        for label in sorted(self.expected):
            print(f"{label}\n    {self.msgs.get(label, '(no message received)')}")

        print("\n-- TF CHAIN --")
        for parent, child in [('odom', 'base_link'),
                              ('odom', 'camera_link'),
                              ('camera_link', 'base_link'),
                              ('base_link', 'camera_depth_optical_frame'),
                              ('odom', 'camera_depth_optical_frame')]:
            print(self._tf(parent, child))

        print("\n-- Z DATUM --")
        if self.cuvslam_z is not None and self.local_z is not None:
            off = self.local_z - self.cuvslam_z
            print(f"  cuVSLAM odom z = {self.cuvslam_z:+.3f}   PX4 local z = {self.local_z:+.3f}   "
                  f"offset = {off:+.3f} m")
            if abs(off) > 5.0:
                print(f"  -> large constant offset (~origin altitude). At rest on ground after EV")
                print(f"     fusion, PX4 local z should re-datum near 0. If it stays ~{off:+.0f},")
                print(f"     the bridge frame-offset (measured at FOLLOW-engage) must cancel it,")
                print(f"     AND the z bounds box [z_min,z_max] must be set in that datum, or every")
                print(f"     setpoint will be rejected. Confirm armed + fusing before flight.")
            else:
                print(f"  -> small offset; PX4 local z is datumed near ground. Bounds box OK as-is.")
        else:
            print("  (need both cuVSLAM odom and local_position to compare)")

        print("\n-- CONSISTENCY HINTS --")
        print("  * cuVSLAM odom and EV relay out should be IDENTICAL (relay is a pass-through).")
        print("  * EGO camera_pose position == cuVSLAM odom; its yaw is ~-90 deg (optical frame).")
        print("  * MAVROS local_position x/y = cuVSLAM shifted by EKF2_EV_POS (0.220/0/0.050) = base_link.")
        print("  * odom->base_link == odom->camera_link composed with camera_link->base_link.")
        print("=" * 78 + "\n")


def main():
    rclpy.init()
    node = PoseSnapshot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
