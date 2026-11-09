import math
from .core import require_calibration


def parameters(c,bt, require_verified=True):
    require_calibration(c, require_verified=require_verified)
    radius=c['wheelbase_m']/(2*math.tan(c['max_steer_rad']))
    speed=c['measured_speed_mps']
    footprint='[[0.1524,0.1524],[0.1524,-0.1524],[-0.1524,-0.1524],[-0.1524,0.1524]]'
    def costmap(global_map):
        p=dict(global_frame='map' if global_map else 'odom',robot_base_frame='base_link',
            update_frequency=3. if global_map else 8.,publish_frequency=2.,resolution=.05,
            robot_radius=.216,footprint=footprint,footprint_padding=0.,transform_tolerance=.3,
            rolling_window=not global_map,track_unknown_space=global_map,
            plugins=(['static_layer'] if global_map else [])+['obstacle_layer','inflation_layer'])
        if not global_map:p.update(width=5,height=5)
        p['static_layer']=dict(plugin='nav2_costmap_2d::StaticLayer',map_subscribe_transient_local=True,map_topic='/navigation/map')
        p['obstacle_layer']=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=True,
            observation_sources='scan',scan=dict(topic='scan',data_type='LaserScan',clearing=True,
                marking=True,max_obstacle_height=1.5,obstacle_max_range=6.,raytrace_max_range=8.,
                observation_persistence=.5,expected_update_rate=.2))
        p['inflation_layer']=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.45,cost_scaling_factor=3.)
        return {'ros__parameters':p}
    ekf=dict(frequency=20.,sensor_timeout=.3,two_d_mode=True,publish_tf=True,
        map_frame='map',odom_frame='odom',base_link_frame='base_link',world_frame='odom',
        transform_time_offset=0.,transform_timeout=.1,
        odom0='wheel/odometry',odom0_config=[False]*6+[True,True,False]+[False]*6,
        odom0_queue_size=10,odom0_differential=False,
        imu0='imu/data',imu0_config=[False,False,False,False,False,True]+[False]*9,
        imu0_queue_size=10,imu0_differential=False,imu0_relative=True,
        imu0_remove_gravitational_acceleration=False,
        odom0_twist_rejection_threshold=5.,imu0_pose_rejection_threshold=5.)
    # Independent body-frame metric velocities; never feed SLAM map poses back
    # into the odom EKF that supplies SLAM's own motion prior.
    for index,source in enumerate(('lidar','front','rear'),1):
        ekf.update({f'odom{index}':source+'/odometry',
                    f'odom{index}_config':[False]*6+[True,True,False,False,False,True]+[False]*3,
                    f'odom{index}_queue_size':3,f'odom{index}_differential':False,
                    f'odom{index}_twist_rejection_threshold':3.})
    slam=dict(use_sim_time=False,mode='mapping',map_frame='map',odom_frame='odom',base_frame='base_link',
        scan_topic='/scan',resolution=.05,map_update_interval=2.,transform_publish_period=.05,
        transform_timeout=.3,tf_buffer_duration=30.,minimum_time_interval=.1,
        minimum_travel_distance=.08,minimum_travel_heading=.08,
        max_laser_range=8.,min_laser_range=.10,scan_queue_size=1,throttle_scans=1,
        use_scan_matching=True,use_scan_barycenter=True,do_loop_closing=True,
        solver_plugin='solver_plugins::CeresSolver',ceres_linear_solver='SPARSE_NORMAL_CHOLESKY',
        ceres_preconditioner='SCHUR_JACOBI',ceres_trust_strategy='LEVENBERG_MARQUARDT',
        ceres_dogleg_type='TRADITIONAL_DOGLEG',ceres_loss_function='None',stack_size_to_use=40000000)
    nav={
        'planner_server':{'ros__parameters':dict(expected_planner_frequency=.5,planner_plugins=['GridBased'],
            GridBased=dict(plugin='nav2_smac_planner::SmacPlannerHybrid',tolerance=.10,downsample_costmap=False,
                allow_unknown=False,max_iterations=500000,max_planning_time=3.,motion_model_for_search='REEDS_SHEPP',
                angle_quantization_bins=72,minimum_turning_radius=radius,
                analytic_expansion_max_length=max(3.,5*radius),cost_penalty=3.,smooth_path=True))},
        'controller_server':{'ros__parameters':dict(controller_frequency=10.,odom_topic='odometry/filtered',
            enable_stamped_cmd_vel=True,failure_tolerance=.5,progress_checker_plugins=['progress_checker'],
            goal_checker_plugins=['goal_checker'],controller_plugins=['FollowPath'],
            progress_checker=dict(plugin='nav2_controller::SimpleProgressChecker',required_movement_radius=.10,movement_time_allowance=8.),
            goal_checker=dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=.08,yaw_goal_tolerance=.5,stateful=True),
            FollowPath=dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                max_linear_vel=speed,lookahead_dist=max(.35,radius*.5),min_lookahead_dist=.25,max_lookahead_dist=.8,
                use_velocity_scaled_lookahead_dist=False,use_rotate_to_heading=False,allow_reversing=True,
                use_collision_detection=True,max_allowed_time_to_collision_up_to_carrot=1.5,
                use_regulated_linear_velocity_scaling=True,regulated_linear_scaling_min_radius=radius,
                regulated_linear_scaling_min_speed=speed,min_approach_linear_velocity=speed,
                approach_velocity_scaling_dist=.4,transform_tolerance=.3))},
        'bt_navigator':{'ros__parameters':dict(global_frame='map',robot_base_frame='base_link',odom_topic='odometry/filtered',
            bt_loop_duration=20,default_server_timeout=100,wait_for_service_timeout=2000,
            navigators=['navigate_to_pose'],navigate_to_pose=dict(plugin='nav2_bt_navigator::NavigateToPoseNavigator'),
            default_nav_to_pose_bt_xml=bt)},
        'behavior_server':{'ros__parameters':dict(local_costmap_topic='local_costmap/costmap_raw',
            global_costmap_topic='global_costmap/costmap_raw',local_footprint_topic='local_costmap/published_footprint',
            global_footprint_topic='global_costmap/published_footprint',global_frame='map',local_frame='odom',
            robot_base_frame='base_link',transform_tolerance=.3,enable_stamped_cmd_vel=True,
            behavior_plugins=['wait'],wait=dict(plugin='nav2_behaviors::Wait'))},
        'global_costmap':{'global_costmap':costmap(True)},
        'local_costmap':{'local_costmap':costmap(False)}}
    return {'ekf_filter_node':{'ros__parameters':ekf}}, {'slam_toolbox':{'ros__parameters':slam}},nav
