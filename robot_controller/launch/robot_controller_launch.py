

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression
import os

def generate_launch_description():

    # Namespace and Robot ID
    namespace = LaunchConfiguration('namespace')
    robot_id_default = PythonExpression(["'", namespace, "'[-1]"])

    # Paths
    robot_controller_dir = get_package_share_directory('robot_controller')
    yaml_map_file = os.path.join(robot_controller_dir, 'maps', 'my_map.yaml')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('namespace', description='Namespace for the robot'),
        DeclareLaunchArgument('robot_id', default_value=robot_id_default, description='Unique ID of the robot'),

        # Apply namespace to all nodes
        

        # Robot Controller Node
        Node(
            package='robot_controller',
            executable='robot_controller',
            name='robot_controller',
            parameters=[{'robot_id': LaunchConfiguration('robot_id')}], 
            output='screen',
        ),
    
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': yaml_map_file}],
            output='screen',
        ),

        # AMCL for Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[{
                'use_sim_time': False,
                'base_frame_id': '/base_footprint',
                'global_frame_id': '/map',
                'scan_topic': '/scan',
                'tf_broadcast': True,
            }],
            output='screen',
        ),

        # TF Static Transform Publisher (Ensures correct transforms)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            parameters=[],
            remappings=[('/tf_static', PythonExpression(["'", LaunchConfiguration('namespace'), "' + '/tf_static'"]))],
            arguments=["0", "0", "0", "0", "0", "0", "/base_link", "/base_scan"],
        ),
    ])
