#!/usr/bin/env python

import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def controller_spawning(context, *args, robots_file=None, waypoints_file=None, poses_file=None, **kwargs):
    controllers = []

    n_robots = LaunchConfiguration('n_robots').perform(context)
    n_robots = int(n_robots)
    use_sim_time = TextSubstitution(text='False')
    param_file = os.path.join(get_package_share_directory('ccr'), 'params', LaunchConfiguration('params').perform(context))
    with open(param_file, 'r') as stream:
         params = yaml.safe_load(stream)
         
    grid_params = params['grid_params']
    grid_params['graph_file'] = os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps', 'icra2024.yaml')
    local_planner_params = params['local_planner_params']
    global_planner_params = params['global_planner_params']
    ccr_version = LaunchConfiguration('ccr_version').perform(context)
    
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)

    controllers.append(Node(
           package='ccr',
           executable='ccr_goal_provider',
           parameters=[{
              'use_sim_time': use_sim_time,
              'waypoints': waypoints_file,
              'robot_names': robots[:n_robots],
           }, grid_params, local_planner_params, global_planner_params],
           output='both',
    ))
    controllers.append(Node(
           package='ccr',
           executable='ccr_fake_execution',
           parameters=[{
              'use_sim_time': use_sim_time,
              'poses': poses_file,
              'robot_names': robots[:n_robots],
           }, grid_params, local_planner_params, global_planner_params],
           output='both',
    ))
    
    for robot in robots[:n_robots]:
        controllers.append(Node(
           package='ccr',
           executable=f'ccr_{ccr_version}',
           namespace=robot,
           parameters=[{
              'use_sim_time': use_sim_time,
              'robot_names': robots[:n_robots],
              'priorities': LaunchConfiguration('priorities').perform(context),
           }, grid_params, local_planner_params, global_planner_params
              ],
           output='both',
        ))
    
    return controllers


def generate_launch_description():
    args = {
         'behaviour': 'false',      
         'world': 'icra2024.world',
         'map': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps' ,'icra2024.yaml'),
         'robot_names_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'robot_names_sim.yaml'),
         'waypoints_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'icra2024_waypoints.yaml'),
         'poses_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'icra2024_poses.yaml'),
         'rosbag_topics_file': os.path.join(get_package_share_directory('trajectory_follower'), 'params', 'rosbag_topics.yaml'),
         'qos_override_file': os.path.join(get_package_share_directory('experiment_measurement'), 'params', 'qos_override.yaml')
    }
    rosbag_recording = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('experiment_measurement'),
                         'launch', 'rosbag_recording.launch.py')),
        launch_arguments=args.items()
    )

    ld = LaunchDescription()
    ld.add_action(rosbag_recording)
    ld.add_action(DeclareLaunchArgument('ccr_version', default_value=EnvironmentVariable('CCR_VERSION', default_value='global_planner')))
    ld.add_action(DeclareLaunchArgument('priorities', default_value=EnvironmentVariable('CCR_PRIORITIES', default_value='same')))
    ld.add_action(DeclareLaunchArgument('params', default_value='ccr_params.yaml'))
    ld.add_action(OpaqueFunction(function=controller_spawning, kwargs={
        'robots_file': args['robot_names_file'],
        'waypoints_file': args['waypoints_file'],
        'poses_file': args['poses_file']}))
    return ld