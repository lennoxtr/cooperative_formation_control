#!/usr/bin/env python3

import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def gen_robot_list(number_of_robots):

    robots = []

    for i in range(number_of_robots):
        robot_name = "turtlebot"+str(i)
        x_pos = float(i) + 1
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01})

    return robots 

def generate_launch_description():
    
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    assert os.path.exists(urdf_path), "Thebox_bot.urdf doesnt exist in "+str(urdf_path)

    #change
    pkg_box_bot_description = get_package_share_directory('turtlebot3_gazebo')
    

    # Names and poses of the robots
    robots = gen_robot_list(5)

    # We create the list of spawn robots commands
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_box_bot_description, 'launch',
                                                           'spawn_turtlebot_with_node.py')),
                launch_arguments={
                                  'robot_urdf': urdf_path,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_namespace': robot['name']
                                  }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()
    
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld