import rclpy
import time
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelStates
from robot_goal.msg import Goal
from velocity_msg.msg import Velocity
from heading_msg.msg import Heading

class RobotGoalPublisher(Node):
    def __init__(self):
        super().__init__('robot_goal_publisher')
        self.received_goal = False
        self.received_position_updated = False
        self.leader_namespace = 'turtlebot0'
        self.arrived_at_goal = False
        self.robot_controller_map = {}
    
        # Position mapping
        self.position_mapping = []

        # Velocity mapping
        self.velocity_mapping = []

        # Heading mapping
        self.heading_mapping = []

        # Subscription
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

        self.heartbeat_subscription = self.create_subscription(
            String,
            '/heartbeat',
            self.heartbeat_callback,
            15)

        self.arrived_at_goal_subscription = self.create_subscription(
            Bool,
            '/arrived_at_goal',
            self.arrived_at_goal_callback,
            15)

        # Publisher
        self.goal_publisher = self.create_publisher(
            Goal,
            f'/{self.leader_namespace}/goal',
            10)

        self.position_mapping_publisher = self.create_publisher(
            Float32MultiArray(),
            '/position_mapping',
            10)

        self.heading_mapping_publisher = self.create_publisher(
            Float32MultiArray(),
            '/heading_mapping',
            10)

        self.velocity_mapping_publisher = self.create_publisher(
            Float32MultiArray(),
            '/velocity_mapping',
            10)

        self.leader_namespace_publisher = self.create_publisher(
            String,
            '/leader',
            10)


        # Need to implement
        self.is_started_publisher = self.create_publisher(
            Bool,
            '/start',
            100)

    def goal_listener_callback(self, msg):
        self.goal_x = float("{:.3f}".format(msg.goal_x))
        self.goal_y = float("{:.3f}".format(msg.goal_y))
        self.received_goal = True
        self.get_logger().info(f"Goal set to {self.goal_x}, {self.goal_y}")
        # Publish goal to leader
        self.goal_publisher.publish(msg)
        time.sleep(1)

        # Publish start signal to all robots
        msg = True
        self.is_started_publisher.publish(msg)

    def position_listener_callback(self, msg):
        if len(enumerate(msg.name)) != len(self.robot_controller_map):
            return

        for i, namespace in enumerate(msg.name):
            if (namespace == "ground_plane"):
                continue
            index = int(namespace[-1])
            position = msg.pose[i].position
            current_x = float("{:.3f}".format(position.x))
            current_y = float("{:.3f}".format(position.y))

            # TODO: Need to publish robot current position
            self.position_mapping[index] = (current_x, current_y)
            # Check this

            self.robot_controller_map[namespace].update_position(current_x, current_y)

            if (namespace == self.leader_namespace):
                self.leader_x = current_x
                self.leader_y = current_y
            else:
                self.robot_controller_map[namespace].update_goal(self.leader_x, self.leader_y)
        
        # Publish position update to all robots
        msg = Float32MultiArray()
        msg.data = self.position_mapping
        self.position_mapping_publisher.publish(msg)
        self.received_position_updated = True
    
    def velocity_listener_callback(self, msg):
        index = msg.robot_id
        self.velocity_mapping[index] = msg.linear_x
        # Publish position update to all robots
        msg = Float32MultiArray()
        msg.data = self.velocity_mapping
        self.velocity_mapping_publisher.publish(msg)
    
    def heading_listener_callback(self, msg):
        index = msg.robot_id
        self.heading_mapping[index] = msg.heading
        # Publish position update to all robots
        msg = Float32MultiArray()
        msg.data = self.heading_mapping
        self.heading_mapping_publisher.publish(msg)

    def heartbeat_callback(self, msg):
        robot_namespace = msg
        if robot_namespace not in self.robot_controller_map:
            self.robot_controller_map[robot_namespace] = 1
            self.heading_mapping.append(0.0)
            self.velocity_mapping.append(0.0)

    def arrived_at_goal_callback(self, msg):
        self.arrived_at_goal = msg.data
        
    def execute(self):
        rclpy.spin_once(self)

        # Have not received goal
        while not self.received_goal:
            self.leader_namespace_publisher.publish(self.leader_namespace)
            return

        # Executing while leader robot has not reached goal
        while not self.arrived_at_goal:
            while not self.received_position_updated:
                rclpy.spin_once(self)
            rclpy.spin_once(self)
            self.received_position_updated = False
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    robot_goal_publisher = RobotGoalPublisher()
    rclpy.spin_once(robot_goal_publisher)
    time.sleep(1)

    executor = MultiThreadedExecutor()
    executor.add_node(robot_goal_publisher)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    while True:
        try:
            rclpy.spin_once(robot_goal_publisher)
            robot_goal_publisher.execute()
        except KeyboardInterrupt:
            break
    
    robot_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
