import time
import math
import numpy as np
import rclpy

from robot_goal_pub.GoalProcessor import get_position_error
from robot_goal_pub.GoalProcessor import get_yaw_error

ANG_TOL = 0.2
POSITION_TOL = 0.05

class ControlProtocol():
    def __init__(self, rendezvous_distance, num_of_robot=4):
        #TODO: need to find out the +- quadrant of angles
        #TODO: Implement slow down for collision avoidance
        self.velocity_gain = 0
        self.heading_gain = 0
        self.num_of_robot = num_of_robot
        self.rendezvous_distance = rendezvous_distance

        # To determine rendezvous position
        self.avg_position_x = 0
        self.avg_position_y = 0

        # All robots in formation flag
        self.all_rendezvoused = False

        # Sensitivity bubble for individual robots
        angles_in_rad = np.arange(360) * np.pi / 180
        self.normalized_angle_in_rad = (angles_in_rad + np.pi) % (2 * np.pi) - np.pi
        bare_sensitivity_bubble = np.array([self.get_sensitivity_bubble_gain(i) for i in range(360)])

        # Assume current_vel = max_vel = 0.2 to reduce calculation
        current_vel = 0.2
        delta_t = 4
        self.sensitivity_bubble = bare_sensitivity_bubble * current_vel * delta_t


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

        #if robot_controller.namespace == "turtlebot0":
        #    print(robot_controller.namespace, " tracking position (", self.avg_position_x, ", ", self.avg_position_y, ")")

        position_error = get_position_error(robot_controller.current_x, 
                                        robot_controller.current_y,
                                        self.avg_position_x,
                                        self.avg_position_y)
        
        yaw_error = get_yaw_error(robot_controller.current_x, 
                                        robot_controller.current_y,
                                        self.avg_position_x,
                                        self.avg_position_y,
                                        robot_controller.current_imu_heading)
        #if robot_controller.namespace == "turtlebot0":
        #    print("Yaw error position matching: ", yaw_error)

        if position_error < self.rendezvous_distance:
            robot_controller.is_rendezvoused = True
            position_error = 0.0
            yaw_error = 0.0
            print(robot_controller.namespace, " rendezvoused")
        else:
            robot_controller.is_rendezvoused = False

        return position_error, yaw_error
    
    def velocity_matching(self, robot_controller, velocity_mapping):
        a_ij_val = 1
        velocity_error = self.num_of_robot * a_ij_val * robot_controller.linear_x_velocity - \
                            a_ij_val * sum(velocity_mapping)
        return velocity_error
    
    def get_sensitivity_bubble_gain(self, angle_in_degree):
        ''' Map [-pi, +pi] to minimum and maximum gain for sensitivity bubble'''
        min_gain = 0.2 #2
        max_gain = 1.1 #2

        angle_in_rad = self.normalized_angle_in_rad[angle_in_degree]

        A = (max_gain + min_gain) / 2
        B = (max_gain - min_gain) / 2

        sensitivity_bubble_gain = A + B * math.cos(angle_in_rad) 
        # max = 1.5 at the front
        # min = 0.5 at the back
        # value = 1 at +- 90 degrees

        return sensitivity_bubble_gain
    
    def collision_prevention(self, robot_controller):
        # Test using only frontal 180

        # Needs to be implemented in all control algo
        # Using bubble rebound algo
        rclpy.spin_once(robot_controller)
        lidar_data = robot_controller.lidar_data
        
        possible_collision_angle = np.where((lidar_data > 0.0) & (lidar_data < self.sensitivity_bubble))[0]

        valid_angles = (possible_collision_angle >= 270) | (possible_collision_angle <= 180)
        possible_collision_angle = possible_collision_angle[valid_angles]
        
        '''
        if (robot_controller.namespace == "turtlebot0"):
            print(sensitivity_bubble)
            print(lidar_data)
            print(" ")
            print(lidar_data[possible_collision_angle])
            print(" ")
            print("Collision angle:")
            print(possible_collision_angle)
            print(" ")
        '''
        
        
        if possible_collision_angle.size == 0:
            position_error = 0.0
            yaw_error = 0.0
            return position_error, yaw_error

        valid_distances = lidar_data[possible_collision_angle]
        valid_collision_angles_in_rad = self.normalized_angle_in_rad[possible_collision_angle]
        distance_weights =  valid_distances - robot_controller.dangerous_radius

        # Calculate position error for slowing down to prevent collision
        # TODO: tune velocity gain for collision avoidance
        velocity_gain = 1

        min_distance = np.min(valid_distances)
        #position_error =  velocity_gain * (robot_controller.dangerous_radius - min_distance) / robot_controller.dangerous_radius * 0.2
        position_error = 0.2

        # Calculate rebound angle
        weighted_sum_of_distance = np.sum(valid_collision_angles_in_rad * distance_weights)
        sum_of_distance = robot_controller.dangerous_radius * possible_collision_angle.size - np.sum(valid_distances)

        # There is a chance collision angle all > 90 and < 270 causing sum_of_distance to be 0
        rebound_angle = weighted_sum_of_distance / sum_of_distance

        '''
        if (robot_controller.namespace == "turtlebot0"):
            print(sensitivity_bubble)
            print("Valid collision angles in rad: ", valid_collision_angles_in_rad)
            print("Weighted sum: ", weighted_sum_of_distance)
            print("Sum of distances: ", sum_of_distance)
            print(" ")
            print("Rebound angle: ", rebound_angle)
            print("--------------")
        '''
        
        
        left_lidar_data = lidar_data[45:100]
        mean_distance_left_side = np.mean(left_lidar_data)

        '''
        if (robot_controller.namespace == "turtlebot0"):
            print("Left mean: ", mean_distance_left_side)
        '''

        if -ANG_TOL < rebound_angle < ANG_TOL:
            sign_of_rebound_angle = rebound_angle / abs(rebound_angle)
            if mean_distance_left_side < robot_controller.max_lidar_range:
                # Left and right side has obstacles turn 180
                rebound_angle = sign_of_rebound_angle * np.pi - sign_of_rebound_angle * ANG_TOL
            else: 
                rebound_angle = sign_of_rebound_angle * np.pi/2

        #print("Rebound angle for ", robot_controller.namespace, " is: ", rebound_angle)

        yaw_error = rebound_angle

        return position_error, yaw_error
    
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

    def get_flocking_gain(self, robot_controller, position_mapping):
        # Must be called after position matching
        # At the moment recalculated for all robots => wasteful

        if self.all_rendezvoused:
            flocking_gain = 0.0
            return flocking_gain

        sum_distance_to_formation_center = 0
        for (x_coord, y_coord) in position_mapping:
            sum_distance_to_formation_center += get_position_error(x_coord,
                                                               y_coord,
                                                               self.avg_position_x,
                                                               self.avg_position_y)
        
        avg_distance = sum_distance_to_formation_center / self.num_of_robot
        if avg_distance < self.rendezvous_distance:
            self.all_rendezvoused = True
            print("RENDEZVOUS DETECTED")
        else:
            self.rendezvoused = False
        
        # Implement as logistic function
        # TODO: Need to tune
        if robot_controller.is_leader:
            k = 1.5
        else:
            k = 0.5 # k is the flocking function steepness

        # TODO: check whether -self.rendezvous_distance is needed
        flocking_gain = 1 / (1 + math.e ** (-k * (avg_distance - self.rendezvous_distance)))
        return flocking_gain
        
    def calculate_control(self, robot_controller, position_mapping, velocity_mapping, heading_mapping):
        ### Sum of all control policies

        # Method 1: have 1 PID (currently only P) for all
        # Method 2: have individual PID for each control policy

        # Collision Prevention
        ca_position_error, ca_yaw_error = self.collision_prevention(robot_controller)

        if ca_yaw_error != 0.0:
            total_yaw_error = ca_yaw_error
            total_position_error = ca_position_error
            return total_position_error, total_yaw_error

        # Position Matching 
        pm_position_error, pm_yaw_error = self.position_matching(robot_controller,
                                                                position_mapping)

        # Velocity Matching
        #velocity_control_output = self.velocity_matching(robot_controller, velocity_mapping)

        # Heading Matching
        #heading_control_output = self.heading_matching(robot_controller, heading_mapping)

        # Leader Follower
        lf_position_error, lf_yaw_error = self.leader_follower(robot_controller)

        if robot_controller.is_rendezvoused and not self.all_rendezvoused:
            total_position_error = 0.0
            total_yaw_error = 0.0
            return total_position_error, total_yaw_error


        # If robot is at rendezvous position, but still waiting for other

        # Calculate total error with weightage of flocking and goal seeking
        flocking_gain = self.get_flocking_gain(robot_controller, position_mapping)

        # Set to 0 to test PID
        #flocking_gain = 0 

        '''
        if robot_controller.namespace == "turtlebot0": 
            print("Flocking gain is: ", flocking_gain)
        '''

        fl_gs_position_error = (1 - flocking_gain) * lf_position_error + \
                                flocking_gain * pm_position_error
  
        fl_gs_yaw_error = (1 - flocking_gain) * lf_yaw_error + \
                            flocking_gain * pm_yaw_error



        total_yaw_error = fl_gs_yaw_error
        total_position_error = fl_gs_position_error

        '''
        if robot_controller.namespace == "turtlebot0":
            print("Flocking gain: ", flocking_gain)
            print("LF_position_error: ", lf_position_error)
            print("PM_position_error: ", pm_position_error)
            print("Total position error of ", robot_controller.namespace, " : ", total_position_error)                    
            print("Total yaw error of ", robot_controller.namespace, " : ", total_yaw_error)
        '''
        
        return total_position_error, total_yaw_error

    def execute_control(self, robot_controller, position_mapping, velocity_mapping, heading_mapping):
        total_position_error, total_yaw_error = self.calculate_control(robot_controller,
                                                                    position_mapping,
                                                                    velocity_mapping,
                                                                    heading_mapping)

        # Implementing Method 1: 1 set of PID for all policies
        current_time = time.time()
        '''
        if (robot_controller.is_leader):
            linear_x_change = robot_controller.PID_position.compute(total_position_error, current_time)
            angular_z_change = robot_controller.PID_heading.compute(total_yaw_error, current_time)
        else:
            linear_x_change = 0.0
            angular_z_change = 0.0
        '''

        linear_x_change = robot_controller.PID_position.compute(total_position_error, current_time)
        angular_z_change = robot_controller.PID_heading.compute(total_yaw_error, current_time)

        return linear_x_change, angular_z_change
    