#!/usr/bin/env python3
"""Publish the depth camera's pose in the world frame for EGO-Planner's grid_map.

EGO grid_map (pose_type 1) fuses depth using a PoseStamped of the depth optical
frame in the world frame, paired by timestamp with each depth image. cuVSLAM's TF
lags the depth by ~one frame, so we look up the LATEST available transform (no
extrapolation) and stamp it to the depth frame for EGO's sync. The ~30 ms staleness
is negligible for a slow platform.
Params: world_frame [odom], depth_topic, pose_topic.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

class CamPoseRelay(Node):
    def __init__(self):
        super().__init__('cam_pose_relay')
        self.world = self.declare_parameter('world_frame', 'odom').value
        depth_topic = self.declare_parameter('depth_topic', '/realsense_splitter_node/output/depth').value
        pose_topic = self.declare_parameter('pose_topic', '/ego/camera_pose').value
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.create_subscription(Image, depth_topic, self.cb, qos)
        self.n = 0
        self.miss = 0
        self.get_logger().info(f'{self.world} -> [depth frame] as PoseStamped on {pose_topic}, gated by {depth_topic}')

    def cb(self, msg):
        cam = msg.header.frame_id
        try:
            tf = self.buf.lookup_transform(self.world, cam, Time())  # latest available, no extrapolation
        except Exception as e:
            self.miss += 1
            if self.miss % 30 == 1:
                self.get_logger().warn(f'TF {self.world}->{cam} not ready ({self.miss}): {e}')
            return
        p = PoseStamped()
        p.header.stamp = msg.header.stamp   # pair with this depth frame for EGO's sync
        p.header.frame_id = self.world
        t = tf.transform.translation
        p.pose.position.x, p.pose.position.y, p.pose.position.z = t.x, t.y, t.z
        p.pose.orientation = tf.transform.rotation
        self.pub.publish(p)
        self.n += 1
        if self.n % 60 == 1:
            self.get_logger().info(f'pose #{self.n}: [{t.x:.2f},{t.y:.2f},{t.z:.2f}] miss={self.miss}')

def main():
    rclpy.init()
    node = CamPoseRelay()
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
