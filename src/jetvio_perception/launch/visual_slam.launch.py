import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cam = os.path.join(get_package_share_directory('aisle_perception'),
                       'launch', 'camera.launch.py')
    visual_slam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'enable_image_denoising': False, 'rectified_images': True,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244, 'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862, 'accel_random_walk': 0.003,
            'calibration_frequency': 200.0, 'image_jitter_threshold_ms': 34.0,
            'base_frame': 'camera_link', 'imu_frame': 'camera_gyro_optical_frame',
            'camera_optical_frames': ['camera_infra1_optical_frame',
                                      'camera_infra2_optical_frame'],
        }],
        remappings=[
            ('visual_slam/image_0', 'camera/camera/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', 'camera/camera/infra1/camera_info'),
            ('visual_slam/image_1', 'camera/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', 'camera/camera/infra2/camera_info'),
            ('visual_slam/imu', 'camera/camera/imu'),
        ])
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cam)),
        LoadComposableNodes(target_container='perception_container',
                            composable_node_descriptions=[visual_slam]),
    ])
