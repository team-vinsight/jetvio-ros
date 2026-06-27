"""Foxglove bridge for ground-station visualization (run alongside bringup, not part
of it). Whitelists the aisle topics worth watching: EGO viz + occupancy, camera pose,
VIO odometry + health, and TF. Connect Foxglove Studio to ws://<jetson-ip>:8765,
fixed frame 'odom'.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        Node(
            package='foxglove_bridge', executable='foxglove_bridge',
            name='foxglove_bridge', output='screen',
            parameters=[{
                'port': port,
                'topic_whitelist': [
                    '/ego/grid_map/occupancy_inflate',
                    '/ego_vis/.*',
                    '/ego/camera_pose',
                    '/ego/pos_cmd',
                    '/visual_slam/tracking/odometry',
                    '/vio/health',
                    '/mavros/local_position/pose',
                    '/mavros/setpoint_raw/local',
                    '/tf', '/tf_static',
                ],
            }])
    ])
