"""VIO health monitor: watch cuVSLAM odometry + covariance, publish HEALTHY/DEGRADED.

Detection (piece 1 of the VIO-dropout failsafe). Observes only; does NOT touch the
EV path. cuVSLAM's vo_state is unreliable (stays 1 when tracking is lost) and on VO
failure cuVSLAM rides its IMU integrator for ~0.5-1s emitting a drifting/snapping
pose rather than going silent. Verdict from four signals:

  - staleness  : no odom for > stale_timeout                 -> DEGRADED (latched)
  - jump       : implied |dp|/dt > v_jump, or yaw-rate spike, for jump_strikes
                 consecutive frames                            -> DEGRADED
  - covariance : orientation variance (cov[21/28/35]) > cov_orient_max. Healthy
                 cuVSLAM ~0.003 rad^2; on loss it collapses to ~identity (~1.0).
                 Earliest signal - flips at the moment of loss, before the pose
                 drifts far enough to register as a jump.                -> DEGRADED
  - vo_state   : != 1 from /visual_slam/status (bonus, never relied on) -> DEGRADED

Recovery latched by default (re-injecting a recovered/jumped EV pose mid-flight
would step the EKF); clear with ~/reset. auto_recover (default False) re-enables
after recover_secs of continuous clean frames, for bench use.

Publishes std_msgs/Bool on /vio/health (True = healthy).
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class VioHealthMonitor(Node):
    def __init__(self):
        super().__init__('vio_health_monitor')
        self.rate = self.declare_parameter('rate_hz', 10.0).value
        self.odom_topic = self.declare_parameter('odom_topic', '/visual_slam/tracking/odometry').value
        self.cov_topic = self.declare_parameter('cov_topic', '/visual_slam/tracking/vo_pose_covariance').value
        self.status_topic = self.declare_parameter('status_topic', '/visual_slam/status').value
        self.health_topic = self.declare_parameter('health_topic', '/vio/health').value
        self.stale_timeout = self.declare_parameter('stale_timeout', 0.3).value
        self.v_jump = self.declare_parameter('v_jump', 3.0).value
        self.w_jump = self.declare_parameter('w_jump', 3.0).value
        self.jump_strikes = self.declare_parameter('jump_strikes', 2).value
        self.use_cov = self.declare_parameter('use_cov', True).value
        self.cov_orient_max = self.declare_parameter('cov_orient_max', 0.1).value
        self.use_vo_state = self.declare_parameter('use_vo_state', True).value
        self.auto_recover = self.declare_parameter('auto_recover', False).value
        self.recover_secs = self.declare_parameter('recover_secs', 2.0).value

        self.last_odom_t = None
        self.last_stamp = None
        self.last_pos = None
        self.last_yaw = None
        self.strikes = 0
        self.cov_bad = False
        self.cov_val = 0.0
        self.cov_seen_good = False
        self.vo_state = 1
        self.degraded = False
        self.reason = ''
        self.clean_since = None

        self.pub = self.create_publisher(Bool, self.health_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos_profile_sensor_data)
        if self.use_cov:
            self.create_subscription(PoseWithCovarianceStamped, self.cov_topic,
                                     self.cov_cb, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset', self.reset_cb)
        self._sub_status()
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'vio_health_monitor: odom={self.odom_topic} -> {self.health_topic} @ {self.rate:.0f}Hz; '
            f'stale>{self.stale_timeout}s, v_jump>{self.v_jump} w_jump>{self.w_jump} x{self.jump_strikes}, '
            f'cov_orient>{self.cov_orient_max if self.use_cov else "off"}, '
            f'vo_state={"on" if self.use_vo_state else "off"}, auto_recover={self.auto_recover}')

    def _sub_status(self):
        if not self.use_vo_state:
            return
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            self.create_subscription(VisualSlamStatus, self.status_topic,
                                     self._status_cb, qos_profile_sensor_data)
        except Exception as e:
            self.get_logger().warn(f'vo_state disabled (status msg unavailable): {e}')
            self.use_vo_state = False

    def _status_cb(self, m):
        self.vo_state = m.vo_state

    def cov_cb(self, m):
        c = m.pose.covariance               # row-major 6x6; orientation diag = 21,28,35
        self.cov_val = max(c[21], c[28], c[35])
        good = self.cov_val <= self.cov_orient_max
        if good:
            self.cov_seen_good = True       # cuVSLAM has converged at least once
        # Ignore the boot placeholder (~1.0 before first lock); only flag a
        # collapse AFTER we've seen a good estimate.
        self.cov_bad = (not good) and self.cov_seen_good

    def reset_cb(self, req, resp):
        self.degraded = False
        self.reason = ''
        self.strikes = 0
        self.clean_since = None
        resp.success = True
        resp.message = 'vio health reset -> HEALTHY'
        self.get_logger().warn(resp.message)
        return resp

    def _flag(self, reason):
        if not self.degraded:
            self.degraded = True
            self.reason = reason
            self.clean_since = None
            self.get_logger().error(f'VIO DEGRADED: {reason}')

    def odom_cb(self, m):
        self.last_odom_t = self.get_clock().now()
        p = m.pose.pose.position
        yaw = yaw_from_quat(m.pose.pose.orientation)
        stamp = stamp_s(m.header.stamp)
        if self.last_pos is not None and self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if dt > 1e-3:
                dist = math.sqrt((p.x - self.last_pos[0])**2 +
                                 (p.y - self.last_pos[1])**2 +
                                 (p.z - self.last_pos[2])**2)
                v = dist / dt
                w = abs(ang_diff(yaw, self.last_yaw)) / dt
                if v > self.v_jump or w > self.w_jump:
                    self.strikes += 1
                    if self.strikes >= self.jump_strikes:
                        self._flag(f'jump v={v:.1f}m/s w={w:.1f}rad/s (x{self.strikes})')
                else:
                    self.strikes = 0
        self.last_pos = (p.x, p.y, p.z)
        self.last_yaw = yaw
        self.last_stamp = stamp

    def tick(self):
        now = self.get_clock().now()
        # Startup: wait quietly until the first odom arrives. 'no data yet' is not
        # a degraded verdict - publish healthy=False without latching/logging, so a
        # slow cuVSLAM convergence at boot does not show a spurious DEGRADED.
        if self.last_odom_t is None:
            self.pub.publish(Bool(data=False))
            return
        age = (now - self.last_odom_t).nanoseconds * 1e-9
        stale = age > self.stale_timeout
        if stale:
            self._flag(f'odom stale {age:.2f}s>{self.stale_timeout}s')
        if self.use_cov and self.cov_bad:
            self._flag(f'cov orient var {self.cov_val:.2f}>{self.cov_orient_max}')
        if self.use_vo_state and self.vo_state != 1:
            self._flag(f'vo_state={self.vo_state}')

        clean = ((not stale) and self.strikes == 0
                 and (not self.use_cov or not self.cov_bad)
                 and (not self.use_vo_state or self.vo_state == 1))
        if self.degraded and self.auto_recover and clean:
            if self.clean_since is None:
                self.clean_since = now
            elif (now - self.clean_since).nanoseconds * 1e-9 >= self.recover_secs:
                self.get_logger().warn('VIO recovered -> HEALTHY (auto_recover)')
                self.degraded = False
                self.reason = ''
        elif not clean:
            self.clean_since = None

        self.pub.publish(Bool(data=not self.degraded))


def main():
    rclpy.init()
    node = VioHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
