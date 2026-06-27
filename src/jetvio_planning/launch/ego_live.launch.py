"""EGO-Planner wired to the live aisle perception stack (planning only, no actuation).

Brings up cam_pose_relay + ego_planner_node + traj_server. Requires perception
(camera+splitter+cuVSLAM) already running and the ego_planner overlay sourced
(source ~/ego_ws/install/setup.bash). Output PositionCommand: /ego/pos_cmd.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

PARAMS = {
    'fsm/flight_type': 1, 'fsm/thresh_replan_time': 1.0, 'fsm/thresh_no_replan_meter': 1.0,
    'fsm/planning_horizon': 7.5, 'fsm/planning_horizen_time': 3.0, 'fsm/emergency_time': 1.0,
    'fsm/realworld_experiment': True, 'fsm/fail_safe': True,
    'fsm/waypoint_num': 1,
    'fsm/waypoint0_x': 2.0, 'fsm/waypoint0_y': 0.0, 'fsm/waypoint0_z': 0.0,
    'fsm/waypoint1_x': 0.0, 'fsm/waypoint1_y': 0.0, 'fsm/waypoint1_z': 0.0,
    'fsm/waypoint2_x': 0.0, 'fsm/waypoint2_y': 0.0, 'fsm/waypoint2_z': 0.0,
    'fsm/waypoint3_x': 0.0, 'fsm/waypoint3_y': 0.0, 'fsm/waypoint3_z': 0.0,
    'fsm/waypoint4_x': 0.0, 'fsm/waypoint4_y': 0.0, 'fsm/waypoint4_z': 0.0,
    'grid_map/resolution': 0.1, 'grid_map/map_size_x': 20.0, 'grid_map/map_size_y': 20.0,
    'grid_map/map_size_z': 5.0, 'grid_map/local_update_range_x': 5.5,
    'grid_map/local_update_range_y': 5.5, 'grid_map/local_update_range_z': 4.5,
    'grid_map/obstacles_inflation': 0.2,  # HARD FLOOR 0.65 (cam->prop-tip); never lower for flight
    'grid_map/local_map_margin': 10,
    'grid_map/ground_height': -0.01,
    'grid_map/cx': 426.90252686, 'grid_map/cy': 237.11561584,
    'grid_map/fx': 429.84915161, 'grid_map/fy': 429.84915161,
    'grid_map/use_depth_filter': True, 'grid_map/depth_filter_tolerance': 0.15,
    'grid_map/depth_filter_maxdist': 5.0, 'grid_map/depth_filter_mindist': 0.2,
    'grid_map/depth_filter_margin': 2, 'grid_map/k_depth_scaling_factor': 1000.0,
    'grid_map/skip_pixel': 2, 'grid_map/p_hit': 0.65, 'grid_map/p_miss': 0.35,
    'grid_map/p_min': 0.12, 'grid_map/p_max': 0.90, 'grid_map/p_occ': 0.80,
    'grid_map/min_ray_length': 0.1, 'grid_map/max_ray_length': 4.5,
    'grid_map/virtual_ceil_height': 2.9, 'grid_map/visualization_truncate_height': 1.8,
    'grid_map/show_occ_time': False, 'grid_map/pose_type': 1, 'grid_map/frame_id': 'odom',
    'manager/max_vel': 1.5, 'manager/max_acc': 3.0, 'manager/max_jerk': 4.0,
    'manager/control_points_distance': 0.4, 'manager/feasibility_tolerance': 0.05,
    'manager/planning_horizon': 7.5, 'manager/use_distinctive_trajs': True, 'manager/drone_id': 0,
    'optimization/lambda_smooth': 1.0, 'optimization/lambda_collision': 0.5,
    'optimization/lambda_feasibility': 0.1, 'optimization/lambda_fitness': 1.0,
    'optimization/dist0': 0.5, 'optimization/swarm_clearance': 0.5,
    'optimization/max_vel': 1.5, 'optimization/max_acc': 3.0,
    'bspline/limit_vel': 1.5, 'bspline/limit_acc': 3.0, 'bspline/limit_ratio': 1.1,
    'prediction/obj_num': 10, 'prediction/lambda': 1.0, 'prediction/predict_rate': 1.0,
}

def generate_launch_description():
    relay = Node(
        package='aisle_planning', executable='cam_pose_relay.py',
        name='cam_pose_relay', output='screen',
        parameters=[{'world_frame': 'odom',
                     'depth_topic': '/realsense_splitter_node/output/depth',
                     'pose_topic': '/ego/camera_pose'}])
    ego = Node(
        package='ego_planner', executable='ego_planner_node',
        name='drone_0_ego_planner_node', output='screen',
        remappings=[
            ('odom_world', '/visual_slam/tracking/odometry'),
            ('grid_map/odom', '/visual_slam/tracking/odometry'),
            ('grid_map/depth', '/realsense_splitter_node/output/depth'),
            ('grid_map/pose', '/ego/camera_pose'),
            ('grid_map/cloud', '/ego/unused_cloud'),
            ('grid_map/occupancy_inflate', '/ego/grid_map/occupancy_inflate'),
            ('planning/bspline', '/ego/planning/bspline'),
            ('planning/data_display', '/ego/planning/data_display'),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            ('goal_point', '/ego_vis/goal_point'),
            ('global_list', '/ego_vis/global_list'),
            ('init_list', '/ego_vis/init_list'),
            ('optimal_list', '/ego_vis/optimal_list'),
            ('a_star_list', '/ego_vis/a_star_list'),
        ],
        parameters=[PARAMS])
    traj = Node(
        package='ego_planner', executable='traj_server',
        name='drone_0_traj_server', output='screen',
        remappings=[
            ('planning/bspline', '/ego/planning/bspline'),
            ('position_cmd', '/ego/pos_cmd'),
        ],
        parameters=[{'traj_server/time_forward': 1.0}])
    world_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='odom_to_world',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'world'],
        output='screen')
    return LaunchDescription([world_tf, relay, ego, traj])
