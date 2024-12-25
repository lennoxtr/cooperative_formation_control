from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    num_of_robot = 4
    robot_controller_node_list = []

    i = 0
    while i < num_of_robot:
        node = Node(
            package='robot_controller',
            executable='robot_controller',
            name=f'robot_controller{i}',  # Unique name for each instance
            parameters=[{'robot_id': i}],
            output='screen',
            emulate_tty=True
        )
        robot_controller_node_list.append(node)
        i += 1
    
    return LaunchDescription(robot_controller_node_list)