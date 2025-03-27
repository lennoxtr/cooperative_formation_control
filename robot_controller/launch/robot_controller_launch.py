

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    # Namespace and Robot ID
    namespace = LaunchConfiguration('namespace', default='turtlebot0')
    robot_id = LaunchConfiguration('robot_id', default='0')

    # Paths
    robot_controller_dir = get_package_share_directory('robot_controller')
    yaml_map_file = os.path.join(robot_controller_dir, 'maps', 'my_map.yaml')

    # LiDAR
    if LDS_MODEL == 'LDS-01':
        lidar_launch_file = os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch', 'hlds_laser.launch.py')
    elif LDS_MODEL == 'LDS-02':
        lidar_launch_file = os.path.join(get_package_share_directory('ld08_driver'), 'launch', 'ld08.launch.py')
    else:
        lidar_launch_file = os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch', 'hlds_laser.launch.py')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('namespace', default_value='turtlebot0', description='Namespace for the robot'),
        DeclareLaunchArgument('robot_id', default_value='0', description='Unique ID of the robot'),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0', description='OpenCR USB port'),

        # Apply namespace to all nodes

        # Start LiDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': [namespace, '/base_scan'],
                              }.items(),
        ),

        # TurtleBot3 Bringup (Core Nodes)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')
            ),
            launch_arguments={'use_sim_time': 'False',
                              'namespace': namespace
                              }.items(),
        ),

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
                'base_frame_id': [namespace, '/base_footprint'],
                'global_frame_id': [namespace, '/map'],
                'scan_topic': [namespace, '/scan'],
                'tf_broadcast': True,
            }],
            output='screen',
        ),

        # TF Static Transform Publisher (Ensures correct transforms)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0",
                       [namespace, "/base_link"],
                       [namespace, "/base_scan"]],
        ),
    ])
