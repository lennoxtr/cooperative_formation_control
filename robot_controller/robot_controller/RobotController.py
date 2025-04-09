import rclpy
import time
import threading
import numpy as np

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from robot_goal.msg import Goal
from position_mapping_msg.msg import PositionMapping

from robot_controller.PidController import PidController
from robot_controller.ControlProtocol import ControlProtocol
from robot_controller.GoalProcessor import arrived_at_goal, quaternion_to_euler, get_all_postion_in_formation, normalize_yaw_error


MAX_LINEAR_VEL = 0.08
MAX_ANGLE_VEL = 1.5 #1.5

MAX_LIDAR_RANGE = 3.5

# Define your QoS profile
qos_profile_lidar = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

qos_profile_amcl = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

class RobotController(Node):
    def __init__(self):         
        super().__init__('RobotController')
        self.declare_parameter('robot_id', 0)
        robot_id = self.get_parameter('robot_id').value
        namespace = "turtlebot" + str(robot_id)

        self.initialized_imu = False
        self.imu_offset = 0.0

        # Identification
        self.robot_id = robot_id
        self.namespace = namespace

        if self.namespace == 'turtlebot0':
            self.is_leader = True
        else:
            self.is_leader = False

        # Start execution flag
        self.is_started = False
        self.received_goal = False

        # Control Protocol
        self.rendezvous_distance = 0.4
        self.control_protocol = ControlProtocol(self.rendezvous_distance)

        # Collision avoidance threshold
        self.max_lidar_range = MAX_LIDAR_RANGE
        self.dangerous_radius = 0.6

        # Lidar data for collision avoidance
        self.lidar_data = np.zeros(360)

        # Position variables
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        # Kinematic variables
        # Yaw is +- pi from north
        self.current_imu_heading = 0
        
        # Kinematic PID Controller (may add more for different control policies)
        self.PID_position = PidController(Kp=1.0, Ki=0.0, Kd=0.0)
        self.PID_heading = PidController(Kp=2.5, Ki=0.0, Kd=0.14)

        # Pure pursuit settings
        self.lookahead_dist = 0.8
        self.curvature_thres = 7.0

        '''
        if self.is_leader:
            self.desired_linear_vel = MAX_LINEAR_VEL
        else:
            self.desired_linear_vel = 0.06
        '''

        # For leader
        self.follower_robot_id_list = [1, 2]
        self.position_mapping = np.array([(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)])

        # Subscriptions
        self.imu_subscription = self.create_subscription(
            Imu,
            f'/{self.namespace}/imu',
            self.imu_callback,
            10)
        
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            f'/{self.namespace}/scan',
            self.lidar_callback, 
            qos_profile_lidar)
        
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/turtlebot0/goal_pose',
            self.goal_listener_callback,
            10)
        
        self.is_started_subscription = self.create_subscription(
            Bool,
            '/start',
            self.is_started_callback,
            10)
        
        self.tracking_position_subscription = self.create_subscription(
            Goal,
            f'/{self.namespace}/tracking_position',
            self.tracking_position_callback,
            10)
        
        self.position_mapping_subscription = self.create_subscription(
            PositionMapping,
            '/position_mapping',
            self.position_mapping_callback,
            10)
        
        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.namespace}/amcl_pose',
            self.amcl_pose_callback,
            qos_profile_amcl)

        # Publishers
        self.is_started_publisher = self.create_publisher(
            Bool,
            '/start',
            10)
        
        if self.is_leader:
            self.tracking_publishers = {}
            for robot_id in self.follower_robot_id_list:
                namespace = 'turtlebot' + str(robot_id)
                topic = f'/{namespace}/tracking_position'
                self.tracking_publishers[robot_id] = self.create_publisher(Goal, topic, 10)


        self.self_twist_publisher = self.create_publisher(
            Twist,
            f'/{self.namespace}/cmd_vel',
            10)
        
        self.control_timer = self.create_timer(0.06, self.execute)

    def amcl_pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        #self.get_logger().info(f"Current position from AMCL: ({self.current_x}, {self.current_y})")
        _, _, yaw = quaternion_to_euler([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        #self.current_imu_heading = float("{:.3f}".format(yaw))
        #self.get_logger().info(f"Current yaw from AMCL: {yaw}")

        
    def imu_callback(self, msg):
        orientation_q = msg.orientation
        quaternion = [orientation_q.x,
                      orientation_q.y,
                      orientation_q.z,
                      orientation_q.w]
        
        euler = quaternion_to_euler(quaternion)
        yaw = euler[2]  # radians
        if not self.initialized_imu:
            self.imu_offset = normalize_yaw_error(-np.pi + 0.01 - yaw)
            self.initialized_imu = True
            return

        #self.get_logger().info(f"Normal IMU Heading: {yaw}")
        #self.get_logger().info(f"IMU Offset: {self.imu_offset}")
        yaw = yaw + self.imu_offset
        self.current_imu_heading = normalize_yaw_error(yaw)
        #self.get_logger().info(f"Map IMU Heading: {self.current_imu_heading}")

    def lidar_callback(self, msg):
        self.lidar_data = np.array(msg.ranges)
        #self.lidar_data[self.lidar_data==0.0] = np.nan
        self.lidar_data[self.lidar_data==np.inf] = MAX_LIDAR_RANGE

    def goal_listener_callback(self, msg):
        self.goal_x = round(msg.pose.position.x, 3)
        self.goal_y = round(msg.pose.position.y, 3)
        self.get_logger().info(f"Received Goal at ({self.goal_x}, {self.goal_y})")
        if self.is_leader:
            msg = Bool()
            msg.data = True
            self.is_started_publisher.publish(msg)
            self.is_started = True
        self.received_goal = True
    
    def is_started_callback(self, msg):
        self.is_started = msg
        self.get_logger().info("Received start signal. Executing")
    
    def tracking_position_callback(self, msg):
        self.goal_x = float("{:.3f}".format(msg.goal_x))
        self.goal_y = float("{:.3f}".format(msg.goal_y))
        #self.get_logger().info(f"Tracking position: ({self.goal_x}, {self.goal_y})")

    def position_mapping_callback(self, msg):
        position_list = msg.data
        self.position_mapping = np.array([(position.x, position.y) for position in position_list])
        #self.get_logger().info(f"Current Position Mapping: {self.position_mapping}")
    
    def move_bot(self, linear_x_change, angular_z_change):
        if abs(angular_z_change) > MAX_ANGLE_VEL:
            target_angular_velocity = angular_z_change / abs(angular_z_change) * MAX_ANGLE_VEL
        else:
            target_angular_velocity = angular_z_change

        if abs(linear_x_change) > MAX_LINEAR_VEL:
            target_linear_velocity = linear_x_change / abs(linear_x_change) * MAX_LINEAR_VEL
        else:
            target_linear_velocity = linear_x_change
        
        #target_linear_velocity = 0.0

        #self.get_logger().info(f"Linear Vel: {target_linear_velocity}")
        #self.get_logger().info(f"Angular Vel: {target_angular_velocity}")

        twist = Twist()
        twist.linear.x = target_linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = target_angular_velocity

        self.self_twist_publisher.publish(twist)
    
    def stop_bot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.self_twist_publisher.publish(twist)
        self.get_logger().info(self.namespace + " stopped")
    
    def is_arrived(self):
        return arrived_at_goal(self.current_x,
                               self.current_y,
                               self.goal_x,
                               self.goal_y)
    
    def execute(self):
        if not self.is_started:
            return

        if self.is_leader:
            robot_formation_position_list = get_all_postion_in_formation(self.current_x,
                                                                        self.current_y,
                                                                        self.current_imu_heading,
                                                                        self.follower_robot_id_list,
                                                                        adjacent_distance = 0.35)
            for item in robot_formation_position_list:
                # Namespace of follower robot
                robot_id = item[0]

                # Tracking position for follower robot
                position_tuple = item[1]
                position_x = position_tuple[0]
                position_y = position_tuple[1]

                # Preparing tracking position message
                msg = Goal()
                msg.goal_x = position_x
                msg.goal_y = position_y

                self.tracking_publishers[robot_id].publish(msg)

        # Control Protocol output linear and angular speed change
        linear_x_change, angular_z_change = self.control_protocol.execute_control(self,
                                                                                self.position_mapping)
        
        self.move_bot(linear_x_change, angular_z_change)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    robot_controller.get_logger().info(f"Am I Leader: {robot_controller.is_leader}")

    # Allow simultaneous processing of callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(robot_controller)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    robot_controller.get_logger().info(robot_controller.namespace + " initialized")

    try:
        # Keep the main thread alive — logic now handled by timer + executor
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Shutting down robot controller...")
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
