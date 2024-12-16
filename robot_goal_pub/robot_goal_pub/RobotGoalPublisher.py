import rclpy
import time
from rclpy.node import Node


from gazebo_msgs.msg import ModelStates
from robot_goal.msg import Goal
from velocity_msg.msg import Velocity
from heading_msg.msg import Heading

from robot_goal_pub.RobotController import RobotController
from robot_goal_pub.ControlProtocol import ControlProtocol


class RobotGoalPublisher(Node):
    def __init__(self):
        super().__init__('robot_goal_publisher')
        self.num_of_robot  = 5
        self.received_goal = False
        self.computed_current_yaw = False
        self.received_position_updated = False
        self.leader_namespace = 'turtlebot0'
        self.control_protocol = ControlProtocol(self.num_of_robot)

        self.robot_controller_map = {}
        for robot_id in range(self.num_of_robot):
            namespace = 'turtlebot' + str(robot_id)
            self.get_logger().info(namespace + " initialized")
            if namespace == self.leader_namespace:
                robot_controller = RobotController(robot_id, is_leader=True)
                rclpy.spin_once(robot_controller)
                self.robot_controller_map[namespace] = robot_controller
            else:
                robot_controller = RobotController(robot_id)
                rclpy.spin_once(robot_controller)
                self.robot_controller_map[namespace] = robot_controller

        #Goal
        self.goal_x = 0
        self.goal_y = 0

        #Leader
        self.leader_x = 0
        self.leader_y = 0

        #Velocity mapping
        self.velocity_mapping = []
        for i in range(self.num_of_robot):
            self.velocity_mapping.append(0.0)

        #Heading mapping
        self.heading_mapping = []
        for i in range(self.num_of_robot):
            self.heading_mapping.append(0.0)

        #Subscription
        self.goal_subscription = self.create_subscription(
            Goal,
            '/robot_goal',
            self.goal_listener_callback,
            10)

        self.position_subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.position_listener_callback,
            15)
        
        self.velocity_subscription = self.create_subscription(
            Velocity,
            '/robot_linear_vel',
            self.velocity_listener_callback,
            15)

        self.heading_subscription = self.create_subscription(
            Heading,
            '/robot_heading',
            self.heading_listener_callback,
            15)

    def goal_listener_callback(self, msg):
        self.goal_x = float("{:.3f}".format(msg.goal_x))
        self.goal_y = float("{:.3f}".format(msg.goal_y))
        self.received_goal = True
        self.robot_controller_map[self.leader_namespace].update_goal(self.goal_x, self.goal_y)
        self.get_logger().info(f"Goal set to {self.goal_x}, {self.goal_y}")

    def position_listener_callback(self, msg):
        for i, namespace in enumerate(msg.name):
            if (namespace == "ground_plane"):
                continue
            position = msg.pose[i].position
            current_x = float("{:.3f}".format(position.x))
            current_y = float("{:.3f}".format(position.y))
            self.robot_controller_map[namespace].update_position(current_x, current_y)
            if (namespace == self.leader_namespace):
                self.leader_x = current_x
                self.leader_y = current_y
            else:
                self.robot_controller_map[namespace].update_goal(self.leader_x, self.leader_y)
        #print("Leader position is ", self.leader_x, " ", self.leader_y)
        self.received_position_updated = True
    
    def velocity_listener_callback(self, msg):
        index = msg.robot_id
        self.velocity_mapping[index] = msg.linear_x
        return
    
    
    def heading_listener_callback(self, msg):
        index = msg.robot_id
        self.heading_mapping[index] = msg.heading
        return
        
    def execute(self):
        rclpy.spin_once(self)
        # Have not received goal
        while not self.received_goal:
            return
        
        # Robot formation moving to goal
        leader_robot = self.robot_controller_map[self.leader_namespace]
        while not leader_robot.arrived_at_goal():
            while not self.received_position_updated:
                rclpy.spin_once(self)

            for robot_controller_name in self.robot_controller_map:
                robot_controller = self.robot_controller_map[robot_controller_name]
                rclpy.spin_once(robot_controller)
                linear_x_change, angular_z_change = self.control_protocol.execute_control(robot_controller)

                # Move to goal
                robot_controller.move_bot(linear_x_change, angular_z_change)
                rclpy.spin_once(robot_controller)

            self.received_position_updated = False
            rclpy.spin_once(self)
        
        for robot_controller in self.robot_controller_map:
            self.robot_controller_map[robot_controller].stop_bot()

def main(args=None):
    rclpy.init(args=args)
    robot_goal_publisher = RobotGoalPublisher()
    rclpy.spin_once(robot_goal_publisher)
    time.sleep(1)

    while True:
        robot_goal_publisher.execute()
    
    robot_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
