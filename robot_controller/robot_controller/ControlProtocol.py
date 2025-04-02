import time
import math
import itertools
import numpy as np

from robot_controller.GoalProcessor import get_position_error
from robot_controller.GoalProcessor import get_yaw_error
from robot_controller.GoalProcessor import generate_straight_path
from robot_controller.GoalProcessor import get_rendezvous_pos

ANG_TOL = 0.2
POSITION_TOL = 0.05

class ControlProtocol():
    def __init__(self, rendezvous_distance, num_of_robot=3):
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
        delta_t = 2.0
        self.sensitivity_bubble = bare_sensitivity_bubble * current_vel * delta_t

    def position_matching(self, robot_controller, position_mapping):
        # Position matching may have higher weight for leader
        # to ensure rendezvous before moving to goal

        rendezvous_pos = get_rendezvous_pos(position_mapping)

        self.avg_position_x = rendezvous_pos[0]
        self.avg_position_y = rendezvous_pos[1]

        self.get_logger().info(f"Rendezvous position: ({self.avg_position_x}, {self.avg_position_y})")


        position_error = get_position_error(robot_controller.current_x, 
                                        robot_controller.current_y,
                                        self.avg_position_x,
                                        self.avg_position_y)
        
        yaw_error = get_yaw_error(robot_controller.current_x, 
                                        robot_controller.current_y,
                                        self.avg_position_x,
                                        self.avg_position_y,
                                        robot_controller.current_imu_heading)

        return position_error, yaw_error
    
    def goal_seeking(self, robot_controller):
        path = generate_straight_path(robot_controller.current_x,
                                      robot_controller.current_y,
                                      robot_controller.goal_x,
                                      robot_controller.goal_y)
        look_ahead_coord = (0, 0)
        if robot_controller.is_leader:
            lookahead_dist = 1.2
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
        min_distance = np.min(valid_distances)
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
        if self.all_rendezvoused:
            flocking_gain = 0.0
            return flocking_gain

        total_distance = 0
        pair_count = 0
        for (x1, y1), (x2, y2) in itertools.combinations(position_mapping, 2):
            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            total_distance += distance
            pair_count += 1

        # Calculate the average distance
        avg_distance = total_distance / pair_count if pair_count > 0 else 0

        dist_to_rendezvous = get_position_error(robot_controller.current_x,
                                                robot_controller.current_y,
                                                self.avg_position_x,
                                                self.avg_position_y)
        #Check
        if avg_distance < self.rendezvous_distance:
            self.all_rendezvoused = True
        else:
            self.all_rendezvoused = False
        
        # Implement as logistic function
        # k is the flocking function steepness
        # For follower, flocking gain = 0

        if robot_controller.is_leader:
            k = 4.0
        else:
            k = 0.0

        flocking_gain = (1 - math.e ** (-k * (dist_to_rendezvous - self.rendezvous_distance))) / (1 + math.e ** (-k * (dist_to_rendezvous - self.rendezvous_distance)))
        
        if flocking_gain < 0.0:
            flocking_gain = 0.0
        
        return flocking_gain
        
    def execute_control(self, robot_controller, position_mapping):
        ### Sum of all control policies

        # Collision Avoidance
        #ca_position_error, ca_yaw_error = self.collision_prevention(robot_controller)
        ca_position_error, ca_yaw_error = 0.0, 0.0

        if ca_yaw_error != 0.0 and not robot_controller.is_leader:
            total_yaw_error = ca_yaw_error
            total_position_error = ca_position_error
            linear_vel, angular_vel = self.calculate_vel(robot_controller, total_position_error, total_yaw_error)
            
            linear_vel = 0.05
            
            return linear_vel, angular_vel

        # Position Matching (Rendezvous)
        pm_position_error, pm_yaw_error = self.position_matching(robot_controller,
                                                                position_mapping)

        # Calculate total error with weightage of flocking and goal seeking
        flocking_gain = self.get_flocking_gain(robot_controller, position_mapping)
        if robot_controller.is_leader:
            print("flocking_gain is: ", flocking_gain)

        # Leader Follower (followers tracking formation, leader tracking goal)
        lf_position_error, lf_yaw_error = self.leader_follower(robot_controller)

        # Set to 0 to test PID
        # Set to 1 to test forming formation
        #flocking_gain = 0

        if robot_controller.is_leader:
            if 0.8 > flocking_gain > 0.2 and abs(pm_yaw_error) > 1/4 * math.pi:
                pm_yaw_error = 0.0

        fl_gs_position_error = (1 - flocking_gain) * lf_position_error + \
                                flocking_gain * pm_position_error
  
        fl_gs_yaw_error = (1 - flocking_gain) * lf_yaw_error + \
                            flocking_gain * pm_yaw_error

        total_yaw_error = fl_gs_yaw_error
        total_position_error = fl_gs_position_error
        linear_vel, angular_vel = self.calculate_vel(robot_controller, total_position_error, total_yaw_error)

        if robot_controller.is_leader and flocking_gain < 0.5 and not self.all_rendezvoused:
            linear_vel = 0.1

        if flocking_gain == 0.0:
            if robot_controller.is_leader:
                return self.goal_seeking(robot_controller)
            else:
                return linear_vel, angular_vel
        
        return linear_vel, angular_vel

    def calculate_vel(self, robot_controller, total_position_error, total_yaw_error):
        '''
        current_time = time.time()
        if robot_controller.is_leader:
            linear_vel, angular_vel = self.goal_seeking(robot_controller)
        else:
            total_position_error, total_yaw_error = self.leader_follower(robot_controller)
            linear_vel = robot_controller.PID_position.compute(total_position_error, current_time)
            angular_vel = robot_controller.PID_heading.compute(total_yaw_error, current_time)

        '''
        # Implementing Method 1: 1 set of PID for all policies
        current_time = time.time()

        linear_x_change = robot_controller.PID_position.compute(total_position_error, current_time)
        angular_z_change = robot_controller.PID_heading.compute(total_yaw_error, current_time)

        return linear_x_change, angular_z_change
    