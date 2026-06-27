"""Static robot frames: FCU body (base_link) relative to the camera.

cuVSLAM owns odom->camera_link and PX4 already compensates the camera offset via
EKF2_EV_POS, so we keep cuVSLAM emitting the camera pose and add base_link as a
CHILD of camera_link (the inverse of the physical mount). Mount: camera is
0.220 m forward and 0.050 m below the FCU, so from the camera the FCU (base_link)
is 0.220 m back and 0.050 m up. Assumes a level, front-facing camera (no pitch).
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_to_base_link',
        arguments=['--frame-id', 'camera_link', '--child-frame-id', 'base_link',
                   '--x', '-0.220', '--y', '0.0', '--z', '0.050'],
        output='screen')
    return LaunchDescription([camera_to_base])
