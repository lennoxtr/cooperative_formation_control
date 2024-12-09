import os
import select
import math
import numpy as np

import rclpy
import time
from rclpy.node import Node

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu
from robot_goal.msg import Goal

from GoalProcessor import is_equal
from GoalProcessor import arrive_at_goal
from GoalProcessor import get_distance

from PidController import PidController

MAX_LINEAR_VEL = 0.2
MAX_ANGLE_VEL = 1.5

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

POSITION_TOLERANCE = 1e-2
HEADING_TOLERANCE = 1e-2

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output

class RobotGoalPublisher(Node):
    def __init__(self):
        super().__init__('robot_goal_publisher')
        self.current_x = 0.0
        self.current_y = 0.0

        self.received_goal = False
        self.computed_current_yaw = False
        self.received_position_updated = False

        self.goal_x = 0
        self.goal_y = 0

        # Yaw is +- pi from north

        self.current_yaw = 0

        self.linear_x_velocity = 0
        self.angular_z_velocity = 0

        self.goal_subscription = self.create_subscription(
            Goal,
            '/robot_goal',
            self.goal_listener_callback,
            10)

        self.position_subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.position_listener_callback,
            10)

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)

        self.publisher_ = self.create_publisher(
            Twist,
            'turtlebot0/cmd_vel',
            10)

        self.goal_subscription
        self.position_subscription
        self.imu_subscription
        self.publisher_

    def goal_listener_callback(self, msg):
        self.goal_x = float("{:.3f}".format(msg.goal_x))
        self.goal_y = float("{:.3f}".format(msg.goal_y))
        self.received_goal = True
        # TODO: set PID for each robot
        self.PID_position = PidController(Kp=,Ki=,Kd=)
        self.PID_heading = PidController(Kp=,Ki=,Kd=)
        #self.get_logger().info(f"I heard {self.goal_x}, {self.goal_y}")

    def position_listener_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "turtlebot0" and self.received_goal:
                position = msg.pose[i].position
                self.current_x = float("{:.3f}".format(position.x))
                self.current_y = float("{:.3f}".format(position.y))

                self.received_position_updated = True
                #self.get_logger().info(f"The robot position is x= '{position.x:.3f}', y='{position.y:.3f}'")

                # target heading is at the momement relative to the x axis
                #self.get_logger().info(f"Robot currently at x= {self.current_x}, y= {self.current_y}. Goal is x= {self.goal_x}, y= {self.goal_y}. Target heading is at {self.target_heading}")
    
    def imu_callback(self, msg):
        orientation_q = msg.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        euler = self.quaternion_to_euler(quaternion)
        roll = euler[0]  # radians
        pitch = euler[1]  # radians
        yaw = euler[2]  # radians
        self.current_yaw = float("{:.3f}".format(yaw))
        self.computed_current_yaw = True
        #self.get_logger().info(f"Current yaw is {yaw}")
        #self.get_logger().info(f"Current yaw is {self.current_yaw}. Goal yaw is {self.goal_yaw}")
    
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
    
    def calculate_target_yaw(self):
        delta_x = self.goal_x - self.current_x
        delta_y = self.goal_y - self.current_y
        target_heading = np.arctan2(delta_y, delta_x)
        target_yaw = float("{:.3f}".format(target_heading))
        return target_yaw

    def move_to_goal(self, linear_x_change, angular_z_change):
        
        if (linear_x_change > LIN_VEL_STEP_SIZE):
            target_linear_velocity = self.linear_x_velocity + LIN_VEL_STEP_SIZE
        else:
            target_linear_velocity = self.linear_x_velocity + linear_x_change

        if (abs(angular_z_change) > ANG_VEL_STEP_SIZE and angular_z_change > 0):
            target_angular_velocity = self.angular_z_velocity + ANG_VEL_STEP_SIZE

        elif (abs(angular_z_change) > ANG_VEL_STEP_SIZE and angular_z_change < 0):
            target_angular_velocity = self.angular_z_velocity - ANG_VEL_STEP_SIZE

        else:
            target_angular_velocity = self.angular_z_velocity + angular_z_change

        self.linear_x_velocity = target_linear_velocity
        self.angular_z_velocity = target_angular_velocity

        twist = Twist()
        twist.linear.x = target_linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = target_angular_velocity

        self.publisher_.publish(twist)
        #rclpy.spin_once(self)

        return
        
    def execute(self):
        rclpy.spin_once(self)
        time.sleep(1)
        
        # Have not received goal
        while not self.received_goal:
            return
        
        # Moving to goal
        while not arrive_at_goal(self.current_x, self.current_y, self.goal_x, self.goal_y):
            while not (self.computed_current_yaw and self.received_position_updated):
                self.get_logger().info("Waiting for position and heading update")

            position_error = get_distance(self.current_x, self.current_y, self.goal_x, self.goal_y)

            target_yaw = self.calculate_target_yaw()
            yaw_error = target_yaw - self.current_yaw

            ## get time
            current_time = 1
            linear_x_change = self.PID_position.compute(position_error, current_time)
            angular_z_change = self.PID_heading.compute(yaw_error, current_time)

            # Move to goal
            self.move_to_goal(linear_x_change, angular_z_change)

            self.computed_current_yaw = False
            self.received_position_updated = False
            rclpy.spin_once(self)
        

            
def main(args=None):
    rclpy.init(args=args)
    robot_goal_publisher = RobotGoalPublisher()

    while True:
        robot_goal_publisher.execute()
    
    robot_goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
