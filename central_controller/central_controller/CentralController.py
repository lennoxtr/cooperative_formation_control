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
        self.create_subscription(PoseWithCovarianceStamped, '/turtlebot0/amcl_pose', self.amcl_callback_0, qos_profile_amcl)
        self.create_subscription(PoseWithCovarianceStamped, '/turtlebot1/amcl_pose', self.amcl_callback_1, qos_profile_amcl)
        self.create_subscription(PoseWithCovarianceStamped, '/turtlebot2/amcl_pose', self.amcl_callback_2, qos_profile_amcl)

        # Publisher
        self.position_mapping_publisher = self.create_publisher(
            PositionMapping,
            '/position_mapping',
            10)
        
        self.create_timer(0.075, self.publish_position_mapping)

    def amcl_callback_0(self, msg):
        with self.lock:
            self.position_mapping[0] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def amcl_callback_1(self, msg):
        with self.lock:
            self.position_mapping[1] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    def amcl_callback_2(self, msg):
        with self.lock:
            self.position_mapping[2] = (msg.pose.pose.position.x, msg.pose.pose.position.y)


    def publish_position_mapping(self):
        msg = PositionMapping()
        msg.data = [Point(x=t[0], y=t[1], z=0.0) for t in self.position_mapping]
        self.position_mapping_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    central_controller = CentralController()

    executor = MultiThreadedExecutor()
    executor.add_node(central_controller)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # Keep the main thread alive — logic now handled by timer + executor
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        central_controller.get_logger().info("Shutting down central controller...")
    
    central_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()