import time
import math
import numpy as np
import rclpy

from robot_controller.GoalProcessor import get_position_error
from robot_controller.GoalProcessor import get_yaw_error
from robot_controller.GoalProcessor import normalize_yaw_error
from robot_controller.GoalProcessor import generate_straight_path

ANG_TOL = 0.2
POSITION_TOL = 0.05

class ControlProtocol():
    def __init__(self, rendezvous_distance, num_of_robot=3):
        #TODO: consider changing collision threshold when rendezvous
        self.velocity_gain = 0
        self.heading_gain = 0
        self.num_of_robot = num_of_robot
        self.rendezvous_distance = rendezvous_distance

        # Gains for control output
        self.velocity_matching_gain = 1

        # To determine rendezvous position
        self.avg_position_x = 0
        self.avg_position_y = 0

        # All robots in formation flag
        self.avg_flocking_gain = 1.0
        self.all_rendezvoused = False

        # Sensitivity bubble for individual robots
        angles_in_rad = np.arange(360) * np.pi / 180
        self.normalized_angle_in_rad = (angles_in_rad + np.pi) % (2 * np.pi) - np.pi
        bare_sensitivity_bubble = np.array([self.get_sensitivity_bubble_gain(i) for i in range(360)])

        # Assume current_vel = max_vel = 0.2 to reduce calculation
        current_vel = 0.2
        delta_t = 3
        self.sensitivity_bubble = bare_sensitivity_bubble * current_vel * delta_t

        # For reporting flocking gain
        self.start_time = time.time()
        self.last_time = time.time()
        self.flocking_gain_list = []
        self.collision_avoidance_list = []
        self.total_yaw_error_list = []
        self.flocking_goal_seeking_error = []
        self.recorded_time = []

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
            #print(robot_controller.namespace, " rendezvoused")
        else:
            robot_controller.is_rendezvoused = False

        return position_error, yaw_error
    
    def velocity_matching(self, robot_controller, velocity_mapping):
        a_ij_val = 1
        velocity_error = self.num_of_robot * a_ij_val * robot_controller.linear_x_velocity - \
                            a_ij_val * sum(velocity_mapping)
        return velocity_error
    
    def heading_matching(self, robot_controller, heading_mapping):
        heading_error =  robot_controller.leader_heading - robot_controller.current_imu_heading
        heading_error = normalize_yaw_error(heading_error)
        return heading_error
    
    def goal_seeking(self, robot_controller):
        path = generate_straight_path(robot_controller.current_x,
                                      robot_controller.current_y,
                                      robot_controller.goal_x,
                                      robot_controller.goal_y)
        look_ahead_coord = (0, 0)
        if robot_controller.is_leader:
            lookahead_dist = 2.0
        else:
            lookahead_dist = 0.05
        found_dist = 0.0
        for coord in path:
            x = coord[0]
            y = coord[1]
            dist = get_position_error(robot_controller.current_x,
                                      robot_controller.current_y,
                                      x, y)
            if dist > lookahead_dist:
                look_ahead_coord = coord
                found_dist = dist
                break
        
        lookahead_x = look_ahead_coord[0]
        lookahead_y = look_ahead_coord[1]
        delta_x = lookahead_x - robot_controller.current_x
        delta_y = lookahead_y - robot_controller.current_y

        # check yaw angle
        phi_r = robot_controller.current_imu_heading

        x_dash = delta_x * np.cos(phi_r) + delta_y * np.sin(phi_r)
        y_dash = delta_y * np.cos(phi_r) - delta_x * np.sin(phi_r)

        denom = ((x_dash * x_dash) + (y_dash * y_dash)) + 1e-6
        curvature = (2 * y_dash) / denom

        linear_vel  = robot_controller.desired_linear_vel
        # Curvature heuristic
        if abs(curvature) > robot_controller.curvature_thres:
            linear_vel  *= robot_controller.curvature_thres / abs(curvature)

        angular_vel = robot_controller.desired_linear_vel * curvature

        print("Robot id: ", robot_controller.robot_id)
        print("Angular vel: ", angular_vel)
        print("Linear vel: ", linear_vel)
        print("Curvature: ", curvature)
        print("Distance to look ahead: ", found_dist)


        return linear_vel, angular_vel


    def get_sensitivity_bubble_gain(self, angle_in_degree):
        ''' Map [-pi, +pi] to minimum and maximum gain for sensitivity bubble'''

        min_gain = 0.2 
        max_gain = 1


        angle_in_rad = self.normalized_angle_in_rad[angle_in_degree]

        A = (max_gain + min_gain) / 2
        B = (max_gain - min_gain) / 2

        sensitivity_bubble_gain = A + B * math.cos(angle_in_rad) 
        # max = 1 at the front
        # min = 0.2 at the back
        # value = 1 at +- 90 degrees

        return sensitivity_bubble_gain
    
    def collision_prevention(self, robot_controller):
        # Test using only frontal 180

        # Needs to be implemented in all control algo
        # Using bubble rebound algo
        lidar_data = robot_controller.lidar_data
        
        possible_collision_angle = np.where((lidar_data > 0.0) & (lidar_data < self.sensitivity_bubble))[0]

        valid_angles = (possible_collision_angle >= 270) | (possible_collision_angle <= 180)
        possible_collision_angle = possible_collision_angle[valid_angles]
        
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
        
        left_lidar_data = lidar_data[45:100]
        mean_distance_left_side = np.mean(left_lidar_data)

        if rebound_angle == 0.0:
            rebound_angle += ANG_TOL

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

        #if self.all_rendezvoused:
        #    flocking_gain = 0.0
        #    return flocking_gain

        sum_distance_to_formation_center = 0
        for (x_coord, y_coord) in position_mapping:
            sum_distance_to_formation_center += get_position_error(x_coord,
                                                               y_coord,
                                                               self.avg_position_x,
                                                               self.avg_position_y)
        
        avg_distance = sum_distance_to_formation_center / self.num_of_robot

        dist_to_rendezvous = get_position_error(robot_controller.current_x,
                                                robot_controller.current_y,
                                                self.avg_position_x,
                                                self.avg_position_y)
        
        #Check
        '''
        if avg_distance < self.rendezvous_distance:
            self.all_rendezvoused = True
        else:
            self.all_rendezvoused = False
        '''

        # Implement as logistic function
        # TODO: Need to tune
        if robot_controller.is_leader:
            k = 7
        else:
            k = 0 # k is the flocking function steepness

        # TODO: check whether -self.rendezvous_distance is needed
        flocking_gain = (1 - math.e ** (-k * (dist_to_rendezvous - self.rendezvous_distance))) / (1 + math.e ** (-k * (dist_to_rendezvous - self.rendezvous_distance)))
        self.avg_flocking_gain = (1 - math.e ** (-k * (avg_distance - self.rendezvous_distance))) / (1 + math.e ** (-k * (avg_distance - self.rendezvous_distance)))
        
        if flocking_gain < 0.0:
            flocking_gain = 0.0
        
        if self.avg_flocking_gain < 0.0:
            self.avg_flocking_gain = 0.0
        
        return flocking_gain
        
    def calculate_control(self, robot_controller, position_mapping, velocity_mapping, heading_mapping):
        ### Sum of all control policies

        # Collision Avoidance
        ca_position_error, ca_yaw_error = self.collision_prevention(robot_controller)

        if ca_yaw_error != 0.0 and not self.all_rendezvoused:
            total_yaw_error = ca_yaw_error
            total_position_error = ca_position_error
            return total_position_error, total_yaw_error

        # Position Matching 
        pm_position_error, pm_yaw_error = self.position_matching(robot_controller,
                                                                position_mapping)

        # Velocity Matching
        #velocity_error = self.velocity_matching(robot_controller, velocity_mapping)

        # Heading Matching
        heading_error = self.heading_matching(robot_controller, heading_mapping)

        # If robot is at rendezvous position, but still waiting for other

        # Calculate total error with weightage of flocking and goal seeking
        flocking_gain = self.get_flocking_gain(robot_controller, position_mapping)

        # Leader Follower
        lf_position_error, lf_yaw_error = self.leader_follower(robot_controller)

        if robot_controller.is_leader:
            if robot_controller.is_rendezvoused:
                total_position_error = 0.0
                total_yaw_error = 0.0
                return total_position_error, total_yaw_error
        else:
            if robot_controller.is_in_formation:
                total_position_error = 0.0
                total_yaw_error = 0.0
                return total_position_error, total_yaw_error

        '''
        if robot_controller.is_rendezvoused and self.avg_flocking_gain > 0.1:
            total_position_error = 0.0
            total_yaw_error = 0.0
            return total_position_error, total_yaw_error
        '''

        # Set to 0 to test PID
        # Set to 1 to test forming formation
        #flocking_gain = 0

        

        fl_gs_position_error = (1 - flocking_gain) * lf_position_error + \
                                flocking_gain * pm_position_error
  
        fl_gs_yaw_error = (1 - flocking_gain) * lf_yaw_error + \
                            flocking_gain * pm_yaw_error

        if not self.all_rendezvoused:
            total_yaw_error = fl_gs_yaw_error
            total_position_error = fl_gs_position_error

        else: # all rendezvoused
            total_position_error = fl_gs_position_error
            
            if robot_controller.is_leader:
                total_yaw_error = fl_gs_yaw_error
            else:
                total_yaw_error = heading_error
        
        '''
        current_time = time.time()

        if current_time - self.last_time > 0.3:
            #print("Flocking gain: ", flocking_gain)
            self.flocking_gain_list.append(flocking_gain)
            self.collision_avoidance_list.append(ca_yaw_error)
            self.total_yaw_error_list.append(total_yaw_error)
            self.flocking_goal_seeking_error.append(fl_gs_yaw_error)
            self.recorded_time.append(time.time() - self.start_time)

            self.last_time = current_time
        '''
        
        return total_position_error, total_yaw_error

    def execute_control(self, robot_controller, position_mapping, velocity_mapping, heading_mapping):
        '''
        total_position_error, total_yaw_error = self.calculate_control(robot_controller,
                                                                    position_mapping,
                                                                    velocity_mapping,
                                                                    heading_mapping)
        '''

        linear_vel, angular_vel = self.goal_seeking(robot_controller)

        '''
        # Implementing Method 1: 1 set of PID for all policies
        current_time = time.time()

        # Uncomment to test PID
        
        if (robot_controller.is_leader):
            linear_x_change = robot_controller.PID_position.compute(total_position_error, current_time)
            angular_z_change = robot_controller.PID_heading.compute(total_yaw_error, current_time)
        else:
            linear_x_change = 0.0
            angular_z_change = 0.0
        

        linear_x_change = robot_controller.PID_position.compute(total_position_error, current_time)
        angular_z_change = robot_controller.PID_heading.compute(total_yaw_error, current_time)

        return linear_x_change, angular_z_change
        '''

        return linear_vel, angular_vel
    