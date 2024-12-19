import time
import math
import numpy as np
import rclpy

from robot_goal_pub.GoalProcessor import get_position_error
from robot_goal_pub.GoalProcessor import get_yaw_error

class ControlProtocol():
    def __init__(self, num_of_robot, rendezvous_distance):
        self.position_gain = 0.2
        self.velocity_gain = 0
        self.heading_gain = 0
        self.collision_prevention_gain = 0.8
        self.leader_follower_gain = 0.8
        self.num_of_robot = num_of_robot
        self.rendezvous_distance = rendezvous_distance

        #TODO: implement adjacency matrix for imperfect information between robots
        #self.adjacency_matrix = adjacency_matrix

    def position_matching(self, robot_controller, position_mapping):
        # Position matching may have higher weight for leader
        # to ensure rendezvous before mmoving to goal
        a_ij_val = 1

        avg_position_x = 0
        avg_position_y = 0

        for coord in position_mapping:
            avg_position_x += coord[0]
            avg_position_y += coord[1]

        avg_position_x -= robot_controller.current_x
        avg_position_y -= robot_controller.current_y
        avg_position_x = avg_position_x / (self.num_of_robot - 1)
        avg_position_y = avg_position_y / (self.num_of_robot - 1)
        print(robot_controller.namespace, " tracking position (", avg_position_x, ", ", avg_position_y, ")")

        position_error = get_position_error(robot_controller.current_x, 
                                        robot_controller.current_y,
                                        avg_position_x,
                                        avg_position_y)
        
        yaw_error = get_yaw_error(robot_controller.current_x, 
                                        robot_controller.current_y,
                                        avg_position_x,
                                        avg_position_y,
                                        robot_controller.current_imu_heading)
        return position_error, yaw_error
    
    def velocity_matching(self, robot_controller, velocity_mapping):
        a_ij_val = 1
        velocity_error = self.num_of_robot * a_ij_val * robot_controller.linear_x_velocity - \
                            a_ij_val * sum(velocity_mapping)
        return velocity_error
    
    def get_sensitivity_bubble_gain(self, angle_in_degree):
        ''' Map [-pi, +pi] to minimum and maximum gain for sensitivity bubble'''
        min_gain = 2
        max_gain = 4

        angle_in_rad = angle_in_degree * math.pi / 180

        # Normalize to [-pi, +pi]
        if angle_in_rad > math.pi:
            angle_in_rad -= 2 * math.pi
        A = (max_gain + min_gain) / 2
        B = (max_gain - min_gain) / 2

        sensitivity_bubble_gain = A + B * math.cos(angle_in_rad)
        return sensitivity_bubble_gain
    
    def collision_prevention(self, robot_controller):
        # Needs to be implemented in all control algo
        # Using bubble rebound algo
        rclpy.spin_once(robot_controller)
        lidar_data = robot_controller.lidar_data
        current_vel = robot_controller.linear_x_velocity
        delta_t = 1 # may tune to get actual delta t

        # Calculating Sensitivity Bubble
        sensitivity_bubble = np.array([self.get_sensitivity_bubble_gain(i) for i in range(360)]) * \
                                current_vel * delta_t

        possible_collision_angle = np.where((lidar_data != np.isnan) & (lidar_data < sensitivity_bubble))[0]

        if possible_collision_angle.size == 0:
            yaw_error = 0
            return yaw_error
        # Calculate rebound angle
        weighted_sum_of_distance = 0
        sum_of_distance = 0
        for angle in possible_collision_angle:
            distance_measured = lidar_data[angle]
            sum_of_distance += distance_measured
            weighted_sum_of_distance += angle * distance_measured

        # bug when angle = 0
        # TODO: fix this bug
        
        rebound_angle = weighted_sum_of_distance / sum_of_distance
        print("Rebound angle for ", robot_controller.namespace, " is: ", rebound_angle)
        # TODO: Normalize rebound_angle to +- pi


        yaw_error = rebound_angle - robot_controller.current_imu_heading
        return yaw_error
    
    def heading_matching(self, robot_controller, heading_mapping):
        a_ij_val = 1
        heading_error = self.num_of_robot * a_ij_val * robot_controller.current_imu_heading - \
                            a_ij_val * sum(heading_mapping)
        return heading_error
    
    def leader_follower(self, robot_controller):
        position_error = get_position_error(robot_controller.current_x,
                                        robot_controller.current_y,
                                        robot_controller.goal_x,
                                        robot_controller.goal_y)
        
        yaw_error = get_yaw_error(robot_controller.current_x,
                                        robot_controller.current_y,
                                        robot_controller.goal_x,
                                        robot_controller.goal_y,
                                        robot_controller.current_imu_heading)
        return position_error, yaw_error

    def get_flocking_goalseeking_gain(self, robot_controller):
        # Implement as logistic function


        return
        
    def execute_control(self, robot_controller, position_mapping, velocity_mapping, heading_mapping):
        ### Sum of all control policies

        # Method 1: have 1 PID (currently only P) for all
        # Method 2: have individual PID for each control policy

        # Position Matching 
        pc_position_error, pc_yaw_error = self.position_matching(robot_controller,
                                                                position_mapping)
        
        # Velocity Matching
        velocity_control_output = self.velocity_matching(robot_controller, velocity_mapping)

        # Heading Matching
        heading_control_output = self.heading_matching(robot_controller, heading_mapping)

        # Leader Follower
        lf_position_error, lf_yaw_error = self.leader_follower(robot_controller)

        # Collision Prevention
        ca_yaw_error = self.collision_prevention(robot_controller)

        # Calculate total error with weightage of flocking and goal seeking
        fl_gs_position_error = self.leader_follower_gain * lf_position_error + \
                                self.position_gain * pc_position_error

        fl_gs_yaw_error = self.leader_follower_gain * lf_yaw_error + \
                            self.position_gain * pc_yaw_error
        
        total_position_error = fl_gs_position_error
        total_yaw_error = fl_gs_yaw_error * (1 - self.collision_prevention_gain) + \
                            ca_yaw_error * self.collision_prevention_gain
        #Implementing method 1
        current_time = time.time()
        linear_x_change = robot_controller.PID_position.compute(total_position_error, current_time)
        angular_z_change = robot_controller.PID_heading.compute(total_yaw_error, current_time)

        '''
        control_input = self.position_gain * position_control_output + \
                        self.velocity_gain * velocity_control_output + \
                        self.heading_gain * heading_control_output + \
                        self.leader_follower_gain * leader_follower_output
        '''
        return linear_x_change, angular_z_change
    