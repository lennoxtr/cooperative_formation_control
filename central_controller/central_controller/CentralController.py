import rclpy
import time
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point
from position_mapping_msg.msg import PositionMapping

qos_profile_amcl = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class CentralController(Node):
    def __init__(self):
        super().__init__('central_controller')
        self.num_of_robot = 3
    
        # Position mapping
        self.position_mapping = [(0.0, 0.0)] * self.num_of_robot

        self.arrived_at_goal = False

        # Mutex lock
        self.lock = threading.Lock()  

        # Subscription
        self.create_subscription(PoseWithCovarianceStamped, '/turtlebot0/amcl_pose', lambda msg: self.update_position(0, msg), qos_profile_amcl)
        self.create_subscription(PoseWithCovarianceStamped, '/turtlebot1/amcl_pose', lambda msg: self.update_position(1, msg), qos_profile_amcl)
        self.create_subscription(PoseWithCovarianceStamped, '/turtlebot2/amcl_pose', lambda msg: self.update_position(2, msg), qos_profile_amcl)

        # Publisher
        self.position_mapping_publisher = self.create_publisher(
            PositionMapping,
            '/position_mapping',
            10)

    def update_position(self, index, msg):
        with self.lock:
            self.position_mapping[index] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.get_logger().info(f"Current Position Mapping: {self.position_mapping}")
        
        # Publish position update to all robots
        self.publish_position_mapping()


    def publish_position_mapping(self):
        msg = PositionMapping()
        msg.data = [Point(x=t[0], y=t[1], z=0.0) for t in self.position_mapping]
        self.position_mapping_publisher.publish(msg)

    def execute(self):
        # Have not received goal
        rclpy.spin_once(self)

        # Executing while leader robot has not reached goal
        while not self.arrived_at_goal:
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    central_controller = CentralController()
    rclpy.spin_once(central_controller)
    time.sleep(1)

    executor = MultiThreadedExecutor()
    executor.add_node(central_controller)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    while True:
        try:
            central_controller.execute()
        except KeyboardInterrupt:
            break
    
    central_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()