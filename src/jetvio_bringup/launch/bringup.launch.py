"""Full runtime bring-up: perception (self-heals + load-orders cuVSLAM) + description
TF + MAVROS + EKF origin + VIO health monitor + gated VIO relay.

Perception runs the emitter heal internally and loads cuVSLAM only after it
completes, so there is no concurrent emitter-toggle / cuVSLAM-convergence race.
The relay is health-GATED (idle until /vio/health true; cuts EV on DEGRADED).
Stop with Ctrl-C. setpoint_bridge is launched separately at flight time.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    reboot_fc = LaunchConfiguration('reboot_fc')

    perception = os.path.join(
        get_package_share_directory('aisle_perception'),
        'launch', 'visual_slam_depth.launch.py')
    description = os.path.join(
        get_package_share_directory('aisle_description'),
        'launch', 'description.launch.py')
    mavros = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'px4.launch')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyTHS1:921600'),
        DeclareLaunchArgument('reboot_fc', default_value='true'),

        # 1) perception: camera + splitter -> emitter heal -> cuVSLAM (load-ordered internally)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(perception)),

        # 2) static TF: camera_link <-> base_link (planning-point offset for the bridge)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description)),

        # 3) MAVROS (PX4)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros),
            launch_arguments={'fcu_url': fcu_url}.items()),

        # 4) EKF origin (+ optional FCU reboot) - one-shot, self-waits for /mavros/state
        TimerAction(period=5.0, actions=[
            Node(package='aisle_flight', executable='set_ekf_origin',
                 name='ekf_origin_setter', output='screen',
                 parameters=[{'reboot': reboot_fc}])]),

        # 5) VIO health monitor - delayed so cuVSLAM has loaded+converged before it judges
        TimerAction(period=15.0, actions=[
            Node(package='aisle_flight', executable='vio_health_monitor',
                 name='vio_health_monitor', output='screen')]),

        # 6) VIO -> /mavros/vision_pose relay, GATED on /vio/health (idle until healthy)
        Node(package='aisle_flight', executable='vio_mavros_relay',
             name='vio_mavros_relay', output='screen'),
    ])
