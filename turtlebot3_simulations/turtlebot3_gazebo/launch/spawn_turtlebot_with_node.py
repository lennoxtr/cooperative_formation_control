from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='turtlebot3_gazebo', #change
            executable='spawn_turtlebot_launch.py', #change
            output='screen',
            arguments=[
                '--robot_urdf', launch.substitutions.LaunchConfiguration('robot_urdf'),
                '--robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
                '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-z', launch.substitutions.LaunchConfiguration('z')]),
    ])