"""Perception: camera + splitter, emitter heal, THEN cuVSLAM (load-ordered).

cuVSLAM is loaded into the container only AFTER the emitter heal process exits, so
it never converges concurrently with the emitter toggle (the toggle's stream
stop/start would otherwise disrupt an in-progress lock - the 3.6s frame-delta).
Sequence: container+camera+splitter up -> check_emitter probes/toggles on stable
frames -> on its exit, cuVSLAM loads and converges once against clean frames.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cam = os.path.join(get_package_share_directory('aisle_perception'),
                       'launch', 'camera_depth.launch.py')

    visual_slam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'enable_image_denoising': False, 'rectified_images': True,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244, 'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862, 'accel_random_walk': 0.003,
            'calibration_frequency': 200.0, 'image_jitter_threshold_ms': 70.0,
            'base_frame': 'camera_link', 'imu_frame': 'camera_gyro_optical_frame',
            'camera_optical_frames': ['camera_infra1_optical_frame',
                                      'camera_infra2_optical_frame'],
        }],
        remappings=[
            ('visual_slam/image_0', 'realsense_splitter_node/output/infra_1'),
            ('visual_slam/camera_info_0', 'camera/camera/infra1/camera_info'),
            ('visual_slam/image_1', 'realsense_splitter_node/output/infra_2'),
            ('visual_slam/camera_info_1', 'camera/camera/infra2/camera_info'),
            ('visual_slam/imu', 'camera/camera/imu'),
        ])

    emitter_heal = Node(
        package='aisle_perception', executable='check_emitter.py',
        name='emitter_probe', output='screen')

    load_cuvslam = LoadComposableNodes(
        target_container='perception_container',
        composable_node_descriptions=[visual_slam])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cam)),
        TimerAction(period=3.0, actions=[emitter_heal]),
        RegisterEventHandler(OnProcessExit(
            target_action=emitter_heal,
            on_exit=[LogInfo(msg='emitter heal done -> loading cuVSLAM'), load_cuvslam])),
    ])
