

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

import os

def generate_launch_description():

    # Namespace and Robot ID
    namespace = LaunchConfiguration('namespace')
    robot_id_default = PythonExpression(["'", namespace, "'[-1]"])
    frame_id = PythonExpression(["'", namespace, "/map'"])
    base_frame_id = PythonExpression(["'", namespace, "/base_footprint'"])
    odom_frame_id = PythonExpression(["'", namespace, "/odom'"])

    # Paths
    robot_controller_dir = get_package_share_directory('robot_controller')
    yaml_map_file = os.path.join(robot_controller_dir, 'maps', 'my_map.yaml')
    


    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('namespace', description='Namespace for the robot'),
        DeclareLaunchArgument('robot_id', default_value=robot_id_default, description='Unique ID of the robot'),
        DeclareLaunchArgument('frame_id', default_value=frame_id, description='Frame ID for the map'),
        

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
            parameters=[{'yaml_filename': yaml_map_file,
                         'frame_id': frame_id,
                         'use_sim_time': False
                        }],
            output='screen',
        ),

        # AMCL for Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[{
                "use_sim_time": False,
                "alpha1": 0.1,
                "alpha2": 0.1,
                "alpha3": 0.05,
                "alpha4": 0.1,
                "alpha5": 0.1,
                "base_frame_id": base_frame_id,
                "beam_skip_distance": 0.5,
                "beam_skip_error_threshold": 0.9,
                "beam_skip_threshold": 0.3,
                "do_beamskip": False,
                "global_frame_id": frame_id,
                "lambda_short": 0.1,
                "laser_likelihood_max_dist": 2.0,
                "laser_max_range": 10.0,
                "laser_min_range": 0.1,
                "laser_model_type": "likelihood_field",
                "max_beams": 90,
                "max_particles": 2000,
                "min_particles": 500,
                "odom_frame_id": odom_frame_id,
                "pf_err": 0.05,
                "pf_z": 0.99,
                "recovery_alpha_fast": 0.0,
                "recovery_alpha_slow": 0.0,
                "resample_interval": 1,
                "robot_model_type": "nav2_amcl::DifferentialMotionModel",
                "save_pose_rate": 0.5,
                "sigma_hit": 0.2,
                "tf_broadcast": True,
                "transform_tolerance": 0.2,
                "update_min_a": 0.1,
                "update_min_d": 0.1,
                "z_hit": 0.8,
                "z_max": 0.05,
                "z_rand": 0.2   ,
                "z_short": 0.1,
                "scan_topic": "scan",
            }],
            remappings=[
                ("/amcl_pose", [namespace, "/amcl_pose"]),
                ("/particlecloud", [namespace, "/particlecloud"]),
                ("/scan", [namespace, "/scan"]),
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
            output='screen',
        ),
    ])
