"""Stage 9 setpoint_bridge: EGO PositionCommand -> MAVROS setpoint, guarded.

Sole node permitted to command the vehicle. Converts /ego/pos_cmd (planning-point
trajectory in odom) into /mavros/setpoint_raw/local (body setpoint in PX4's local
frame). Safety layers, in precedence order each tick:

  1. EGO-loss failsafe : pos_cmd absent > fail_timeout -> STOP publishing (latched),
     PX4 offboard-loss action (Position mode) takes over.
  2. VIO-health gate    : /vio/health DEGRADED (or stale, fail-safe) -> freeze to a
     held position (zero vel), DISABLE follow. EKF2 has fallen to optical flow, so
     PX4 holds; we hold station and hand nothing to EGO. Recovery does NOT auto-
     resume follow - re-enable is required (a recovered VIO pose may have jumped).
  3. Hover-hold         : not enabled / not armed+offboard -> stream current pose.
  4. FOLLOW             : transformed EGO setpoint with pos+vel+yaw feedforward,
     bounds + speed guarded, hold-last on brief pos_cmd staleness.

Never arms or changes mode. Run with BOTH ~/aisle_ws and ~/ego_ws sourced
(quadrotor_msgs lives in ego_ws).
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from quadrotor_msgs.msg import PositionCommand


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def finite(*v):
    return all(math.isfinite(x) for x in v)


class SetpointBridge(Node):
    def __init__(self):
        super().__init__('setpoint_bridge')
        self.rate = self.declare_parameter('rate_hz', 50.0).value
        self.world = self.declare_parameter('world_frame', 'odom').value
        self.body = self.declare_parameter('base_frame', 'base_link').value
        self.plan = self.declare_parameter('planning_frame', 'camera_depth_optical_frame').value
        self.x_max = self.declare_parameter('x_max', 2.0).value
        self.y_max = self.declare_parameter('y_max', 2.0).value
        self.z_min = self.declare_parameter('z_min', 0.3).value
        self.z_max = self.declare_parameter('z_max', 1.5).value
        self.v_max = self.declare_parameter('v_max', 2.0).value
        self.t_hold = self.declare_parameter('hold_timeout', 0.5).value
        self.t_fail = self.declare_parameter('fail_timeout', 2.0).value
        self.bypass = self.declare_parameter('bypass_gate', False).value
        self.require_vio = self.declare_parameter('require_vio', True).value
        self.vio_topic = self.declare_parameter('vio_health_topic', '/vio/health').value
        self.vio_timeout = self.declare_parameter('vio_timeout', 0.5).value

        self.offset_b = None
        self.frame_offset = None
        self.frame_yaw = 0.0
        self.local = None
        self.state = None
        self.enabled = False
        self.following = False
        self.failsafe = False
        self.last_cmd = None
        self.last_t = None
        self.vio_healthy = False        # assumed-bad until first healthy verdict
        self.last_vio_t = None
        self.vio_lost = False
        self.vio_hold = None            # latched (x,y,z,yaw) held during VIO loss

        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.create_subscription(PositionCommand, '/ego/pos_cmd', self.cmd_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, self.vio_topic, self.vio_cb, qos_profile_sensor_data)
        self.create_service(SetBool, '~/follow_enable', self.enable_cb)
        self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(
            f'setpoint_bridge: {self.rate:.0f}Hz, box |x|<={self.x_max} |y|<={self.y_max} '
            f'z[{self.z_min},{self.z_max}] v<={self.v_max}, hold={self.t_hold}s fail={self.t_fail}s, '
            f'vio_gate={"on" if self.require_vio else "off"} ({self.vio_topic}, '
            f'fail-safe >{self.vio_timeout}s). Disabled; hover-hold until follow_enable.')
        if self.bypass:
            self.get_logger().warn('BYPASS_GATE ON - armed/offboard check skipped. BENCH VALIDATION ONLY.')

    def state_cb(self, m): self.state = m
    def pose_cb(self, m): self.local = m

    def vio_cb(self, m):
        self.last_vio_t = self.get_clock().now()
        self.vio_healthy = m.data

    def _vio_ok(self):
        if not self.require_vio:
            return True
        if not self.vio_healthy or self.last_vio_t is None:
            return False
        age = (self.get_clock().now() - self.last_vio_t).nanoseconds * 1e-9
        return age <= self.vio_timeout

    def enable_cb(self, req, resp):
        self.enabled = req.data
        self.failsafe = False
        self.following = False
        self.last_cmd = None
        self.last_t = None
        resp.success = True
        resp.message = f'follow {"ENABLED" if req.data else "disabled"}'
        self.get_logger().warn(resp.message)
        return resp

    def offset(self):
        if self.offset_b is None:
            try:
                t = self.buf.lookup_transform(self.body, self.plan, Time()).transform.translation
                self.offset_b = (t.x, t.y, t.z)
                self.get_logger().info(f'planning-point offset in {self.body}: [{t.x:.3f},{t.y:.3f},{t.z:.3f}]')
            except Exception:
                return None
        return self.offset_b

    def cmd_cb(self, m):
        off = self.offset()
        if off is None:
            self.get_logger().warn('no planning-point TF yet; dropping pos_cmd', throttle_duration_sec=2.0)
            return
        yaw, p, v = m.yaw, m.position, m.velocity
        c, s = math.cos(yaw), math.sin(yaw)
        bx = p.x - (c * off[0] - s * off[1])
        by = p.y - (s * off[0] + c * off[1])
        bz = p.z - off[2]
        if not finite(bx, by, bz, v.x, v.y, v.z, yaw):
            self.get_logger().error('non-finite pos_cmd; rejected', throttle_duration_sec=1.0)
            return
        if abs(bx) > self.x_max or abs(by) > self.y_max or bz < self.z_min or bz > self.z_max:
            self.get_logger().error(f'pos_cmd out of box [{bx:.2f},{by:.2f},{bz:.2f}]; rejected',
                                    throttle_duration_sec=1.0)
            return
        if math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) > self.v_max:
            self.get_logger().error('pos_cmd speed over cap; rejected', throttle_duration_sec=1.0)
            return
        self.last_cmd = (bx, by, bz, v.x, v.y, v.z, yaw)
        self.last_t = self.get_clock().now()

    def measure_offset(self):
        if self.local is None:
            return
        try:
            tf = self.buf.lookup_transform(self.world, self.body, Time())
        except Exception:
            return
        lp, ob = self.local.pose.position, tf.transform.translation
        self.frame_offset = (lp.x - ob.x, lp.y - ob.y, lp.z - ob.z)
        self.frame_yaw = yaw_from_quat(self.local.pose.orientation) - yaw_from_quat(tf.transform.rotation)

    def target(self, x, y, z, vx, vy, vz, yaw, hold):
        t = PositionTarget()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        t.type_mask = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                       PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)
        t.position.x, t.position.y, t.position.z = x, y, z
        t.velocity.x, t.velocity.y, t.velocity.z = (0.0, 0.0, 0.0) if hold else (vx, vy, vz)
        t.yaw = yaw
        return t

    def hover(self):
        if self.local is None:
            return
        p = self.local.pose.position
        self.pub.publish(self.target(p.x, p.y, p.z, 0, 0, 0,
                                     yaw_from_quat(self.local.pose.orientation), True))

    def tick(self):
        if self.failsafe:
            return

        # VIO-health gate: degraded/stale -> freeze to held position, disable follow.
        # Until the first verdict arrives (boot), wait quietly in hover-hold rather
        # than logging a loss - 'no verdict yet' is not the same as 'was healthy, lost'.
        if self.require_vio and self.last_vio_t is None:
            self.measure_offset()
            self.following = False
            self.hover()
            return
        if not self._vio_ok():
            if not self.vio_lost:
                self.vio_lost = True
                self.enabled = False
                self.following = False
                if self.local is not None:
                    p = self.local.pose.position
                    self.vio_hold = (p.x, p.y, p.z, yaw_from_quat(self.local.pose.orientation))
                self.get_logger().error('VIO unhealthy -> HOLD (freezing position; follow disabled, flow holds)')
            if self.vio_hold is not None:
                x, y, z, yaw = self.vio_hold
                self.pub.publish(self.target(x, y, z, 0.0, 0.0, 0.0, yaw, True))
            return
        if self.vio_lost:
            self.vio_lost = False
            self.vio_hold = None
            self.get_logger().warn('VIO healthy again -> follow stays disabled; re-enable to resume')

        st = self.state
        gate = self.bypass or (st is not None and st.armed and st.mode == 'OFFBOARD')
        if not (self.enabled and gate):
            self.measure_offset()
            self.following = False
            self.hover()
            return
        have = self.last_cmd is not None and self.last_t is not None
        dt = (self.get_clock().now() - self.last_t).nanoseconds * 1e-9 if have else 1e9
        if not self.following:
            if dt > self.t_hold:
                self.measure_offset()
                self.hover()
                return
            if self.frame_offset is None:
                self.measure_offset()
            self.following = True
            fo = self.frame_offset or (0.0, 0.0, 0.0)
            self.get_logger().warn(f'FOLLOW engaged. odom->local offset=[{fo[0]:.3f},{fo[1]:.3f},{fo[2]:.3f}] '
                                   f'yawdiff={math.degrees(self.frame_yaw):.1f}deg')
            if abs(self.frame_yaw) > math.radians(5):
                self.get_logger().warn('odom/local yaw differ >5deg; setpoints assume aligned')
        if dt > self.t_fail:
            self.get_logger().error(f'pos_cmd lost {dt:.1f}s>{self.t_fail}s: stopping setpoints -> PX4 failsafe')
            self.failsafe = True
            self.following = False
            return
        fo = self.frame_offset or (0.0, 0.0, 0.0)
        bx, by, bz, vx, vy, vz, yaw = self.last_cmd
        hold = dt > self.t_hold
        if hold:
            self.get_logger().warn(f'pos_cmd stale {dt:.2f}s: holding position', throttle_duration_sec=1.0)
        self.pub.publish(self.target(bx + fo[0], by + fo[1], bz + fo[2], vx, vy, vz, yaw, hold))


def main():
    rclpy.init()
    node = SetpointBridge()
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
