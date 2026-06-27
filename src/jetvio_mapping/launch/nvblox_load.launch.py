import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    share = get_package_share_directory('nvblox_examples_bringup')
    base_cfg = os.path.join(share, 'config/nvblox/nvblox_base.yaml')
    rs_cfg   = os.path.join(share, 'config/nvblox/specializations/nvblox_realsense.yaml')
    nvblox = ComposableNode(
        name='nvblox_node', package='nvblox_ros', plugin='nvblox::NvbloxNode',
        parameters=[base_cfg, rs_cfg, {
            'use_sim_time': False,          # wall clock: no bag, so timers must tick on their own
            'global_frame': 'map',
            'num_cameras': 1, 'use_lidar': False,
            'map_clearing_frame_id': 'camera_link',
            'esdf_slice_bounds_visualization_attachment_frame_id': 'camera_link',
        }],
        remappings=[
            ('camera_0/depth/image', '/realsense_splitter_node/output/depth'),
            ('camera_0/depth/camera_info', '/camera/camera/depth/camera_info'),
        ])
    return LaunchDescription([ComposableNodeContainer(
        name='nvblox_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[nvblox], output='screen')])
