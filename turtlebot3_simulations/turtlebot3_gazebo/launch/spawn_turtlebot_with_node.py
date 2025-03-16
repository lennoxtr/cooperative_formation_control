from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('turtlebot3_gazebo')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, 'launch', 'spawn_turtlebot_launch.py')
            ),
            launch_arguments={
                'robot_urdf': LaunchConfiguration('robot_urdf'),
                'robot_name': LaunchConfiguration('robot_name'),
                'robot_namespace': LaunchConfiguration('robot_namespace'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
            }.items(),
        )
    ])
