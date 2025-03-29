import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_controller_dir = get_package_share_directory('robot_controller')
    rviz_path = os.path.join(robot_controller_dir, 'rviz', 'turtlebot2.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            remappings=[
                ('/tf', '/turtlebot2/tf'),
                ('/tf_static', '/turtlebot2/tf_static')
            ]
        )
    ])
