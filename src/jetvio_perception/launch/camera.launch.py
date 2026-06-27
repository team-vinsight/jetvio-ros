import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    realsense = ComposableNode(
        package='realsense2_camera', plugin='realsense2_camera::RealSenseNodeFactory',
        name='camera',
        parameters=[{
            'enable_infra1': True, 'enable_infra2': True,
            'enable_color': False, 'enable_depth': False,
            'depth_module.emitter_enabled': 0,
            'depth_module.profile': '848x480x30',
            'enable_gyro': True, 'enable_accel': True,
            'gyro_fps': 200, 'accel_fps': 200, 'unite_imu_method': 2,
        }])
    return launch.LaunchDescription([ComposableNodeContainer(
        name='perception_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[realsense], output='screen')])
