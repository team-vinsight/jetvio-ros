import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
def generate_launch_description():
    realsense = ComposableNode(
        package='realsense2_camera', plugin='realsense2_camera::RealSenseNodeFactory',
        name='camera',
        parameters=[{
            'enable_infra1': True, 'enable_infra2': True,
            'enable_color': False, 'enable_depth': True,
            'depth_module.emitter_enabled': 1,
            'depth_module.emitter_on_off': True,
            'depth_module.emitter_always_on': False,
            'depth_module.profile': '848x480x30',
            'enable_gyro': True, 'enable_accel': True,
            'gyro_fps': 200, 'accel_fps': 200, 'unite_imu_method': 2,
        }])
    splitter = ComposableNode(
        package='realsense_splitter', plugin='nvblox::RealsenseSplitterNode',
        name='realsense_splitter_node',
        parameters=[{'input_qos': 'SENSOR_DATA', 'output_qos': 'DEFAULT'}],
        remappings=[
            ('input/infra_1', 'camera/camera/infra1/image_rect_raw'),
            ('input/infra_1_metadata', 'camera/camera/infra1/metadata'),
            ('input/infra_2', 'camera/camera/infra2/image_rect_raw'),
            ('input/infra_2_metadata', 'camera/camera/infra2/metadata'),
            ('input/depth', 'camera/camera/depth/image_rect_raw'),
            ('input/depth_metadata', 'camera/camera/depth/metadata'),
        ])
    return launch.LaunchDescription([ComposableNodeContainer(
        name='perception_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[realsense, splitter], output='screen')])
