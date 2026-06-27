"""Wait for the FCU link, optionally reboot for a fresh EKF2, then set the EKF origin.

Replaces restart_fc.sh. Runs once and exits. Parameters:
  reboot (bool, default True), latitude/longitude/altitude (origin).
"""
import time
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from geographic_msgs.msg import GeoPointStamped

REBOOT_AUTOPILOT = 246  # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN

class EkfOriginSetter(Node):
    def __init__(self):
        super().__init__('ekf_origin_setter')
        self.reboot = self.declare_parameter('reboot', True).value
        self.lat = self.declare_parameter('latitude', 47.3977).value
        self.lon = self.declare_parameter('longitude', 8.5456).value
        self.alt = self.declare_parameter('altitude', 488.0).value
        self._connected = False
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self._origin_pub = self.create_publisher(
            GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)

    def _state_cb(self, msg):
        self._connected = msg.connected

    def _wait_link(self, what):
        self.get_logger().info(f'waiting for FCU link ({what})...')
        while rclpy.ok() and not self._connected:
            rclpy.spin_once(self, timeout_sec=0.5)

    def run(self):
        self._wait_link('initial')
        if self.reboot:
            self.get_logger().info('rebooting autopilot for a fresh EKF2...')
            cli = self.create_client(CommandLong, '/mavros/cmd/command')
            cli.wait_for_service()
            req = CommandLong.Request()
            req.command = REBOOT_AUTOPILOT
            req.param1 = 1.0
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            self._connected = False
            time.sleep(4.0)
            self._wait_link('after reboot')
            time.sleep(3.0)  # let the global_position plugin settle
        self.get_logger().info('setting EKF origin...')
        msg = GeoPointStamped()
        msg.header.frame_id = 'map'
        msg.position.latitude = self.lat
        msg.position.longitude = self.lon
        msg.position.altitude = self.alt
        for _ in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._origin_pub.publish(msg)
            time.sleep(0.5)
        self.get_logger().info('EKF origin set. done.')

def main():
    rclpy.init()
    node = EkfOriginSetter()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
