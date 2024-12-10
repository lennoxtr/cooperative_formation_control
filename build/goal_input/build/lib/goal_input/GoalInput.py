import rclpy
from rclpy.node import Node

from robot_goal.msg import Goal

initial_msg = """
---------------------------------------------------------------------
Input Goal for Robot as x,y. Numbers are rounded to 3 decimal places.
---------------------------------------------------------------------
"""

def get_goal():
    goal_input = input("New goal is: ")
    split_input = goal_input.split(",")
    # Input 0 is x
    goal_x = float("{:.3f}".format(float(split_input[0].replace(" ", ""))))
    # Input 1 is y
    goal_y = float("{:.3f}".format(float(split_input[1].replace(" ", "")))) 
    return goal_x, goal_y

def main(args=None):
    rclpy.init()
    node = rclpy.create_node('get_input')
    pub = node.create_publisher(Goal, '/robot_goal', 10)

    try:
        print(initial_msg)
        while (True):
            (goal_x, goal_y) = get_goal()
            msg = Goal()
            msg.goal_x = goal_x
            msg.goal_y = goal_y
            print(f"Publishing {goal_x}, {goal_y}")
            pub.publish(msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
