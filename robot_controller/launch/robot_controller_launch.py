import launch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    # Namespace and Robot ID
    namespace = LaunchConfiguration('namespace', default='turtlebot0')
    robot_id = LaunchConfiguration('robot_id', default='0')

    # USB Port for OpenCR
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    # Paths
    tb3_param_dir = os.path.join(get_package_share_directory('robot_controller'), 'param', f'{TURTLEBOT3_MODEL}.yaml')
    pkg_nav2_bringup = '/opt/ros/humble/share/nav2_bringup/'  # Update this with the correct path
    rviz_file = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')  # Ensure this path is correct

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
        PushRosNamespace(namespace),

        # Start LiDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        # TurtleBot3 Bringup (Core Nodes)
        Node(
            package='turtlebot3_bringup',
            executable='turtlebot3_robot',
            name='turtlebot3_robot',
            output='screen',
            parameters=[tb3_param_dir, {'use_sim_time': False}],
            remappings=[
                ('/cmd_vel', [LaunchConfiguration('namespace'), '/cmd_vel']),
                ('/scan', [LaunchConfiguration('namespace'), '/scan']),
                ('/odom', [LaunchConfiguration('namespace'), '/odom']),
                ('/tf', [LaunchConfiguration('namespace'), '/tf']),
                ('/tf_static', [LaunchConfiguration('namespace'), '/tf_static']),
                ('/joint_states', [LaunchConfiguration('namespace'), '/joint_states']),
                ('/imu', [LaunchConfiguration('namespace'), '/imu'])
            ],
        ),

        # Robot Controller Node
        Node(
            package='robot_controller',
            executable='robot_controller',
            name='robot_controller',
            parameters=[{'robot_id': LaunchConfiguration('robot_id')}], 
            output='screen',
        ),

        # Localization (AMCL)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[
                ('/scan', [LaunchConfiguration('namespace'), '/scan']),
                ('/map', [LaunchConfiguration('namespace'), '/map']),
                ('/amcl_pose', [LaunchConfiguration('namespace'), '/amcl_pose']),
                ('/tf', [LaunchConfiguration('namespace'), '/tf']),
                ('/tf_static', [LaunchConfiguration('namespace'), '/tf_static'])
            ],
        ),

        # Navigation Stack (Nav2)
        Node(
            package='nav2_bringup',
            executable='nav2_bringup',
            name='nav2_bringup',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[
                ('/goal_pose', [LaunchConfiguration('namespace'), '/goal_pose']),
                ('/cmd_vel', [LaunchConfiguration('namespace'), '/cmd_vel']),
                ('/tf', [LaunchConfiguration('namespace'), '/tf']),
                ('/tf_static', [LaunchConfiguration('namespace'), '/tf_static']),
                ('/map', [LaunchConfiguration('namespace'), '/map'])
            ],
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],  # RViz configuration file
            parameters=[{'use_sim_time': False}],
            remappings=[('/goal_pose', '/turtlebot0/goal_pose')],
            output='screen'
        ),
    ])
