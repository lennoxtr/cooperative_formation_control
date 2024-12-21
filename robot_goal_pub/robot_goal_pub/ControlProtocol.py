import time
import math
import numpy as np
import rclpy

from robot_goal_pub.GoalProcessor import get_position_error
from robot_goal_pub.GoalProcessor import get_yaw_error

class ControlProtocol():
    def __init__(self, num_of_robot, rendezvous_distance):
        self.velocity_gain = 0
        self.heading_gain = 0
        self.collision_prevention_gain = 0.9
        self.num_of_robot = num_of_robot
        self.rendezvous_distance = rendezvous_distance
        self.avg_position_x = 0
        self.avg_position_y = 0
        self.rendezvoused = False

        #TODO: implement adjacency matrix for imperfect information between robots
        #self.adjacency_matrix = adjacency_matrix

    def position_matching(self, robot_controller, position_mapping):
        # Position matching may have higher weight for leader
        # to ensure rendezvous before moving to goal

        sum_position_x = 0
        sum_position_y = 0

        for (x_coord, y_coord) in position_mapping:
            sum_position_x += x_coord
            sum_position_y += y_coord

        self.avg_position_x = sum_position_x / (self.num_of_robot)
        self.avg_position_y = sum_position_y / (self.num_of_robot)
        if robot_controller.namespace == "turtlebot4":
            print(robot_controller.namespace, " tracking position (", self.avg_position_x, ", ", self.avg_position_y, ")")

        position_error = get_position_error(robot_controller.current_x, 
                                        robot_controller.current_y,
                                        self.avg_position_x,
                                        self.avg_position_y)
        
        yaw_error = get_yaw_error(robot_controller.current_x, 
                                        robot_controller.current_y,
                                        self.avg_position_x,
                                        self.avg_position_y,
                                        robot_controller.current_imu_heading)
        if position_error < self.rendezvous_distance:
            position_error = 0
        #if robot_controller.namespace == "turtlebot0":
        #    print("Yaw error position matching: ", yaw_error)
        return position_error, yaw_error
    
    def velocity_matching(self, robot_controller, velocity_mapping):
        a_ij_val = 1
        velocity_error = self.num_of_robot * a_ij_val * robot_controller.linear_x_velocity - \
                            a_ij_val * sum(velocity_mapping)
        return velocity_error
    
    def get_sensitivity_bubble_gain(self, angle_in_degree):
        ''' Map [-pi, +pi] to minimum and maximum gain for sensitivity bubble'''
        min_gain = 3
        max_gain = 5

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
        delta_t = 3 # may tune to get actual delta t

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
        for angle_in_degree in possible_collision_angle:
            angle_in_rad =  angle_in_degree / 180 * np.pi
            if angle_in_rad > np.pi:
                angle_in_rad -= 2 * np.pi

            distance_measured = lidar_data[angle_in_degree]
            sum_of_distance += distance_measured
            weighted_sum_of_distance += angle_in_rad * distance_measured
        
        rebound_angle = weighted_sum_of_distance / sum_of_distance

        if rebound_angle == 0:
            yaw_error = np.pi/2

        #print("Rebound angle for ", robot_controller.namespace, " is: ", rebound_angle)

        yaw_error = rebound_angle - robot_controller.current_imu_heading

        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        elif yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        # TODO: implement slow down

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

        #if robot_controller.namespace == "turtlebot0":
        #    print("Yaw error goal seeking: ", yaw_error)
        return position_error, yaw_error

    def get_flocking_gain(self, robot_controller, position_mapping):
        # Must be called after position matching

        sum_distance_to_formation_center = 0
        for (x_coord, y_coord) in position_mapping:
            sum_distance_to_formation_center += get_position_error(x_coord,
                                                               y_coord,
                                                               self.avg_position_x,
                                                               self.avg_position_y)
        
        avg_distance = sum_distance_to_formation_center / self.num_of_robot
        if avg_distance < self.rendezvous_distance:
            self.rendezvoused = True
            print("RENDEZVOUS DETECTED")
        
        # Implement as logistic function
        if robot_controller.is_leader:
            k = 1
        else:
            k = 1.5 # k is the flocking function steepness

        # TODO: check whether -self.rendezvous_distance is needed
        flocking_gain = 1 / (1 + math.e ** (-k * (avg_distance - self.rendezvous_distance)))
        return flocking_gain
        
    def execute_control(self, robot_controller, position_mapping, velocity_mapping, heading_mapping):
        ### Sum of all control policies

        # Method 1: have 1 PID (currently only P) for all
        # Method 2: have individual PID for each control policy

        # Position Matching 
        pm_position_error, pm_yaw_error = self.position_matching(robot_controller,
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
        #flocking_gain = self.get_flocking_gain(robot_controller, position_mapping)
        flocking_gain = 0
        #if robot_controller.namespace == "turtlebot0": 
        #    print("Flocking gain is: ", flocking_gain)
        fl_gs_position_error = (1 - flocking_gain) * lf_position_error + \
                                flocking_gain * pm_position_error
  
        fl_gs_yaw_error = (1 - flocking_gain) * lf_yaw_error + \
                            flocking_gain * pm_yaw_error
        


        total_position_error = fl_gs_position_error
        #total_yaw_error = fl_gs_yaw_error * (1 - self.collision_prevention_gain) + \
        #                    ca_yaw_error * self.collision_prevention_gain

        if ca_yaw_error > 0:
            total_yaw_error = ca_yaw_error
            print("Collision avoidance")
        else:
            total_yaw_error = fl_gs_yaw_error

        if robot_controller.namespace == "turtlebot3":
            print("Total position error of ", robot_controller.namespace, " : ", total_position_error)                    
            print("Total yaw error of ", robot_controller.namespace, " : ", total_yaw_error)
        #Implementing method 1
        current_time = time.time()
        linear_x_change = robot_controller.PID_position.compute(total_position_error, current_time)
        angular_z_change = robot_controller.PID_heading.compute(total_yaw_error, current_time)

        return linear_x_change, angular_z_change
    