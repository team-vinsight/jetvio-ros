"""Relay cuVSLAM odometry to MAVROS external-vision, gated on VIO health.

/visual_slam/tracking/odometry -> /mavros/vision_pose/pose, but ONLY while VIO is
healthy. Subscribes /vio/health (std_msgs/Bool, from vio_health_monitor): on a
DEGRADED verdict the relay STOPS publishing, EKF2 times out EV aiding and falls to
optical flow, and the drone holds on flow (piece 2 of the VIO-dropout failsafe).

Fail-safe default: NO health message within health_timeout is treated as DEGRADED,
identical to an explicit false. A dead/absent monitor therefore cuts EV rather than
silently leaving stale vision flowing into PX4. Health starts assumed-bad until the
monitor's first 'healthy' arrives.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class VioMavrosRelay(Node):
    def __init__(self):
        super().__init__('vio_mavros_relay')
        self.health_topic = self.declare_parameter('health_topic', '/vio/health').value
        self.health_timeout = self.declare_parameter('health_timeout', 0.5).value
        self.require_health = self.declare_parameter('require_health', True).value

        self.healthy = False           # assumed-bad until first healthy verdict
        self.last_health_t = None
        self.publishing = False        # for edge-logging only

        self.pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.cb, 10)
        self.create_subscription(Bool, self.health_topic, self.health_cb, qos_profile_sensor_data)
        self.count = 0
        if self.require_health:
            self.get_logger().info(
                f'relay up: odom -> /mavros/vision_pose/pose, GATED on {self.health_topic} '
                f'(fail-safe: no verdict >{self.health_timeout}s = cut). Idle until first healthy.')
        else:
            self.healthy = True
            self.get_logger().warn('relay up: health gating DISABLED (require_health=false) - EV always published')

    def health_cb(self, msg):
        self.last_health_t = self.get_clock().now()
        self.healthy = msg.data

    def _gate_open(self):
        if not self.require_health:
            return True
        if not self.healthy:
            return False
        if self.last_health_t is None:
            return False
        age = (self.get_clock().now() - self.last_health_t).nanoseconds * 1e-9
        return age <= self.health_timeout

    def cb(self, msg):
        if not self._gate_open():
            if self.publishing:
                self.publishing = False
                self.get_logger().error('VIO unhealthy/stale -> EV CUT (stopping vision_pose; EKF2 falls to flow)')
            return
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.pub.publish(ps)
        self.count += 1
        if not self.publishing:
            self.publishing = True
            self.get_logger().warn('VIO healthy -> EV publishing to /mavros/vision_pose/pose')
        p, q = ps.pose.position, ps.pose.orientation
        yaw = math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))
        self.get_logger().info(
            f'#{self.count:>6}  pos[{p.x:+.3f} {p.y:+.3f} {p.z:+.3f}] m  yaw {yaw:+6.1f} deg',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = VioMavrosRelay()
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
