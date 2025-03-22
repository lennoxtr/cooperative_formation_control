import rclpy
import time
import threading
import numpy as np

import pandas as pd

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from heading_msg.msg import Heading
from velocity_msg.msg import Velocity
from robot_goal.msg import Goal
from position_mapping_msg.msg import PositionMapping

from robot_controller.PidController import PidController
from robot_controller.ControlProtocol import ControlProtocol
from robot_controller.GoalProcessor import arrived_at_goal, quaternion_to_euler, get_all_postion_in_formation, get_rendezvous_pos


MAX_LINEAR_VEL = 0.2
MAX_ANGLE_VEL = 1.5 #1.5

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

MAX_LIDAR_RANGE = 3.5

class RobotController(Node):
    def __init__(self, is_leader=False):
        
        # TODO: implement formation projection for leader robot
         

        super().__init__('RobotController')
        self.declare_parameter('robot_id', 0)
        robot_id = self.get_parameter('robot_id').value
        namespace = "turtlebot" + str(robot_id)

        # Identification
        self.robot_id = robot_id
        self.namespace = namespace
        self.is_leader = is_leader

        # Start execution flag
        self.is_started = False
        self.received_goal = False

        # Control Protocol
        self.rendezvous_distance = 2
        self.control_protocol = ControlProtocol(self.rendezvous_distance)
        
        # Mappings for control
        self.position_mapping = np.array([])
        self.heading_mapping = np.array([])
        self.velocity_mapping = np.array([])
        
        # Rendezvous flag
        self.is_rendezvoused = False

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
        self.leader_heading = 0.0

        # Kinematic variables
        # Yaw is +- pi from north
        self.current_imu_heading = 0
        self.linear_x_velocity = 0
        self.angular_z_velocity = 0
        
        # Kinematic PID Controller (may add more for different control policies)
        self.PID_position = PidController(Kp=2.3, Ki=0.0, Kd=0.0)
        self.PID_heading = PidController(Kp=5.0, Ki=0.0, Kd=0.1)

        # Formation
        self.is_in_formation = False

        # Pure pursuit settings
        self.lookahead_dist = 0.8
        self.curvature_thres = 7.0
        self.desired_linear_vel = MAX_LINEAR_VEL

        # TODO: Remove this as it is hard-coded
        self.follower_robot_id_list = [1, 2]

        # Subscriptions
        self.imu_subscription = self.create_subscription(
            Imu,
            f'/{self.namespace}/imu',
            self.imu_callback,
            10)
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            f'/{self.namespace}/odom',
            self.odom_callback,
            10)
        
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            f'/{self.namespace}/scan',
            self.lidar_callback,
            10)
        
        self.goal_subscription = self.create_subscription(
            Goal,
            f'/{self.namespace}/goal',
            self.goal_listener_callback,
            10)
        
        self.is_leader_subscription = self.create_subscription(
            String,
            '/leader',
            self.is_leader_callback,
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
        
        self.leader_heading_subscription = self.create_subscription(
            Heading,
            '/leader_heading',
            self.leader_heading_callback,
            10)
        
        self.current_position_subscription = self.create_subscription(
            Goal,
            f'/{self.namespace}/robot_position',
            self.current_position_callback,
            10)
        
        self.position_mapping_subscription = self.create_subscription(
            PositionMapping,
            '/position_mapping',
            self.position_mapping_callback,
            10)
        
        self.velocity_mapping_subscription = self.create_subscription(
            Float64MultiArray,
            '/velocity_mapping',
            self.velocity_mapping_callback,
            10)
        
        self.heading_mapping_subscription = self.create_subscription(
            Float64MultiArray,
            '/heading_mapping',
            self.heading_mapping_callback,
            10)
        
        self.arrived_at_goal_subscription = self.create_subscription(
            Bool,
            '/arrived_at_goal',
            self.arrived_at_goal_callback,
            15)

        # Publishers
        self.heartbeat_publisher = self.create_publisher(
            String,
            '/heartbeat',
            10)
        
        self.heartbeat_timer = self.create_timer(1, self.heartbeat_timer_callback)

        self.arrive_at_goal_publisher = self.create_publisher(
            Bool,
            '/arrived_at_goal',
            10)

        self.self_twist_publisher = self.create_publisher(
            Twist,
            f'/{self.namespace}/cmd_vel',
            10)
        
        self.controller_velocity_publisher = self.create_publisher(
            Velocity,
            '/robot_linear_vel',
            10)
        
        self.heading_publisher = self.create_publisher(
            Heading,
            '/robot_heading',
            10)
        
        self.leader_heading_publisher = self.create_publisher(
            Heading,
            '/leader_heading',
            10)
        
    def imu_callback(self, msg):
        orientation_q = msg.orientation
        quaternion = [orientation_q.x,
                      orientation_q.y,
                      orientation_q.z,
                      orientation_q.w]
        
        euler = quaternion_to_euler(quaternion)
        roll = euler[0]  # radians
        pitch = euler[1]  # radians
        yaw = euler[2]  # radians
        self.current_imu_heading = float("{:.3f}".format(yaw))

        # Publish yaw to central controller node
        msg = Heading()
        msg.robot_id = self.robot_id
        msg.heading = self.current_imu_heading
        self.heading_publisher.publish(msg)
        if self.is_leader:
            self.leader_heading = self.current_imu_heading
            self.leader_heading_publisher.publish(msg)
    
    def odom_callback(self, msg):
        linear_velocity = msg.twist.twist.linear
        self.linear_x = linear_velocity.x
        self.linear_y = linear_velocity.y

        # Publish linear velocity to central controller node
        msg = Velocity()
        msg.robot_id = self.robot_id
        msg.linear_x = self.linear_x
        msg.linear_y = self.linear_y
        self.controller_velocity_publisher.publish(msg)

    def lidar_callback(self, msg):
        self.lidar_data = np.array(msg.ranges)
        #self.lidar_data[self.lidar_data==0.0] = np.nan
        self.lidar_data[self.lidar_data==np.inf] = MAX_LIDAR_RANGE

    def goal_listener_callback(self, msg):
        self.goal_x = float("{:.3f}".format(msg.goal_x))
        self.goal_y = float("{:.3f}".format(msg.goal_y))
        print("Received Goal at ", self.goal_x, " ", self.goal_y)
        self.received_goal = True

    def is_leader_callback(self, msg):
        leader_namespace = msg.data
        if self.namespace == leader_namespace:
            self.is_leader = True
            self.desired_linear_vel = 0.12
            print(self.namespace, " is leader")
    
    def is_started_callback(self, msg):
        self.is_started = msg
        self.get_logger().info("Received start signal. Executing")
    
    def tracking_position_callback(self, msg):
        self.goal_x = float("{:.3f}".format(msg.goal_x))
        self.goal_y = float("{:.3f}".format(msg.goal_y))
    
    def leader_heading_callback(self, msg):
        self.leader_heading = msg.heading
    
    def current_position_callback(self, msg):
        self.current_x = float("{:.3f}".format(msg.goal_x))
        self.current_y = float("{:.3f}".format(msg.goal_y))

    def position_mapping_callback(self, msg):
        position_list = msg.data
        self.position_mapping = np.array([(position.x, position.y) for position in position_list])
    
    def velocity_mapping_callback(self, msg):
        self.velocity_mapping = np.array(msg.data)
    
    def heading_mapping_callback(self, msg):
        self.heading_mapping = np.array(msg.data)
    
    def heartbeat_timer_callback(self):
        if not self.is_started:
            msg = String()
            msg.data = self.namespace
            self.heartbeat_publisher.publish(msg)
    
    def arrived_at_goal_callback(self, msg):
        self.arrived_at_goal = msg.data
        if self.arrived_at_goal:
            self.stop_bot()
            print(self.namespace, " arrived")
    
    def move_bot(self, linear_x_change, angular_z_change):
        if abs(angular_z_change) > MAX_ANGLE_VEL:
            target_angular_velocity = angular_z_change / abs(angular_z_change) * MAX_ANGLE_VEL
        else:
            target_angular_velocity = angular_z_change

        if abs(linear_x_change) > MAX_LINEAR_VEL:
            target_linear_velocity = linear_x_change / abs(linear_x_change) * MAX_LINEAR_VEL
        else:
            target_linear_velocity = linear_x_change
        
        self.linear_x_velocity = target_linear_velocity
        self.angular_z_velocity = target_angular_velocity

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
        while not self.is_started:
            return
        
        '''
        print("Robot id: ", self.robot_id)
        print("Current x: ", self.current_x)
        print("Current y: ", self.current_y)
        print("Tracking pos x: ", self.goal_x)
        print("Tracking pos y: ", self.goal_y)
        '''

        if self.is_leader:
            print("Position mapping: ", self.position_mapping)
            rendezvous_pos = get_rendezvous_pos(self.position_mapping)
            print("Rendezvous pos: ", rendezvous_pos)
            robot_formation_position_list = get_all_postion_in_formation(self.current_x,
                                                                        self.current_y,
                                                                        self.current_imu_heading,
                                                                        self.follower_robot_id_list,
                                                                        adjacent_distance = 0.45)
            for item in robot_formation_position_list:
                # Namespace of follower robot
                robot_id = item[0]
                namespace = 'turtlebot' + str(robot_id)

                # Tracking position for follower robot
                position_tuple = item[1]
                position_x = position_tuple[0]
                position_y = position_tuple[1]

                # Preparing tracking position message
                msg = Goal()
                msg.goal_x = position_x
                msg.goal_y = position_y

                # Dynamic publisher
                dynamic_topic = f'/{namespace}/tracking_position'
                tracking_position_publisher = self.create_publisher(
                Goal,
                dynamic_topic,
                10)

                tracking_position_publisher.publish(msg)

            if self.is_arrived() and self.is_rendezvoused:
                arrived_msg = Bool()
                arrived_msg.data = True
                self.arrive_at_goal_publisher.publish(arrived_msg)
        
        else:
            if self.is_arrived():
                self.is_in_formation = True
            else:
                self.is_in_formation = False
            

        # Control Protocol output linear and angular speed change
        linear_x_change, angular_z_change = self.control_protocol.execute_control(self,
                                                                                self.position_mapping,
                                                                                self.velocity_mapping,
                                                                                self.heading_mapping)
        # Move to goal
        # Uncomment to test collision avoidance
        '''
        if self.namespace == 'turtlebot0':
            self.move_bot(linear_x_change, angular_z_change)
        '''
        
        self.move_bot(linear_x_change, angular_z_change)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin_once(robot_controller)
    time.sleep(1)

    # Allow simultaneous processing of callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(robot_controller)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    robot_controller.get_logger().info(robot_controller.namespace + " initialized")

    while True:
        try:
            rclpy.spin_once(robot_controller)
            robot_controller.execute()
        except KeyboardInterrupt:
            '''
            df = pd.DataFrame(data={"Flocking_gain": robot_controller.control_protocol.flocking_gain_list,
                                    "Total_Yaw_Error": robot_controller.control_protocol.total_yaw_error_list,
                                    "Collision_Avoidance": robot_controller.control_protocol.collision_avoidance_list,
                                    "Goal_seeking_Error": robot_controller.control_protocol.flocking_goal_seeking_error,
                                     "Time": robot_controller.control_protocol.recorded_time})
            df.to_csv(f'./{robot_controller.namespace}.csv', sep=',',index=False)
            '''
    
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
