import rclpy
import time
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from heading_msg.msg import Heading
from velocity_msg.msg import Velocity

from robot_goal_pub.GoalProcessor import arrived_at_goal

MAX_LINEAR_VEL = 0.2
MAX_ANGLE_VEL = 1.5

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

class RobotController(Node):
    def __init__(self, robot_id, is_leader=False):
        super().__init__('RobotController_' + namespace)
        self.robot_id = robot_id
        self.namespace = "turtlebot" + str(robot_id)
        self.is_leader = is_leader

        # change
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        # Yaw is +- pi from north
        self.current_yaw = 0
        self.computed_current_yaw = False

        self.linear_x_velocity = 0
        self.angular_z_velocity = 0

        #TODO: create subscription for goal for leader bot

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

        self.velocity_publisher = self.create_publisher(
            Twist,
            f'/{self.namespace}/cmd_vel',
            10)
        
        self.velocity_publisher = self.create_publisher(
            Velocity,
            '/robot_linear_vel',
            10)
        
        self.heading_publisher = self.create_publisher(
            Heading,
            '/robot_heading',
            10)
    
    def update_position(self, current_x, current_y):
        self.current_x = current_x
        self.current_y = current_y
    
    def quaternion_to_euler(self, q):
        """Convert a quaternion into euler angles (roll, pitch, yaw)."""
        # roll (x-axis rotation)
        sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2])
        cosr_cosp = 1 - 2 * (q[0]**2 + q[1]**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (q[3] * q[1] - q[2] * q[0])
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1 - 2 * (q[1]**2 + q[2]**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
    
    def imu_callback(self, msg):
        orientation_q = msg.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        euler = self.quaternion_to_euler(quaternion)
        roll = euler[0]  # radians
        pitch = euler[1]  # radians
        yaw = euler[2]  # radians
        self.current_yaw = float("{:.3f}".format(yaw))
        self.computed_current_yaw = True
        # Publish yaw to central controller node
        msg = Heading()
        msg.robot_id = self.robot_id
        msg.heading = self.current_yaw
        self.heading_publisher.publish(msg)
        
    
    def odom_callback(self, msg):
        linear_velocity = msg.twist.twist.linear
        self.linear_x = linear_velocity.x
        self.linear_y = linear_velocity.y
        # Publish linear velocity to central controller node
        msg = Velocity()
        msg.robot_id = self.robot_id
        msg.linear_x = self.linear_x
        msg.linear_y = self.linear_y
        self.velocity_publisher.publish(msg)
    
    def calculate_target_yaw(self):
        delta_x = self.goal_x - self.current_x
        delta_y = self.goal_y - self.current_y
        target_heading = np.arctan2(delta_y, delta_x)
        target_yaw = float("{:.3f}".format(target_heading))
        return target_yaw
    
    def move_bot(self, linear_x_change, angular_z_change):
        ## change
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

        self.velocity_publisher.publish(twist)
        
        return
    
    def stop_bot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.velocity_publisher.publish(twist)
        self.get_logger().info(self.namespace + " stopped")
    
    def arrived_at_goal(self):
        return arrived_at_goal(self.current_x, self.current_y, self.goal_x, self.goal_y)
