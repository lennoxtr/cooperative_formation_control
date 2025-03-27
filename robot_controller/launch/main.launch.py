from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart
import os

def generate_launch_description():
    # Declare namespace
    namespace = LaunchConfiguration('namespace', default= os.environ['ROBOT_NAMESPACE'])

    robot_controller_dir = get_package_share_directory('robot_controller')
    yaml_map_file = os.path.join(robot_controller_dir, 'maps', 'my_map.yaml')

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        parameters=[{'yaml_filename': yaml_map_file}],
        output='screen',
    )

    configure_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', [namespace, '/map_server'], 'configure'],
        output='screen'
    )

    # Activate lifecycle transition for map_server (with namespace)
    activate_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', [namespace, '/map_server'], 'activate'],
        output='screen'
    )

    return LaunchDescription([
        # Namespace declaration
        DeclareLaunchArgument('namespace', default_value=namespace, description='Namespace for the robot'),

        # Launch TurtleBot3 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')
            ),
            launch_arguments={'namespace': namespace}.items()  # Ensure namespace consistency
        ),

        # Launch additional nodes (LiDAR, AMCL, Map Server, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_controller'), 'launch', 'robot_controller_launch.py')
            ),
            launch_arguments={'namespace': namespace}.items()
        ),

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=map_server,
                on_start=[configure_map_server],
            )
        ),

        # Activate map_server after it's configured
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=configure_map_server,
                on_start=[activate_map_server],
            )
        ),
    ])
