import time
import math
import numpy as np

from robot_controller.GoalProcessor import get_position_error
from robot_controller.GoalProcessor import get_yaw_error

ANG_TOL = 0.2

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
        current_vel = 0.1
        delta_t = 2.0
        self.sensitivity_bubble = bare_sensitivity_bubble * current_vel * delta_t

    def position_matching(self, robot_controller):
        # Position matching may have higher weight for leader
        # to ensure rendezvous before moving to goal

        #robot_controller.get_logger().info(f"Rendezvous position: ({self.avg_position_x}, {self.avg_position_y})")


        position_error = get_position_error(robot_controller.current_x, 
                                        robot_controller.current_y,
                                        robot_controller.meeting_x,
                                        robot_controller.meeting_y)
        
        yaw_error = get_yaw_error(robot_controller.current_x, 
                                        robot_controller.current_y,
                                        robot_controller.meeting_x,
                                        robot_controller.meeting_y,
                                        robot_controller.current_imu_heading)

        return position_error, yaw_error
    
    def goal_seeking(self, robot_controller):
        if robot_controller.is_leader:
            lookahead_dist = 1.2
        else:
            lookahead_dist = 0.05

        dx = robot_controller.goal_x - robot_controller.current_x
        dy = robot_controller.goal_y - robot_controller.current_y
        dist = math.hypot(dx, dy)



        if dist > lookahead_dist:
            look_ahead_coord = (
                robot_controller.current_x + lookahead_dist * dx / dist,
                robot_controller.current_y + lookahead_dist * dy / dist
            )
        else:
            look_ahead_coord = (robot_controller.goal_x, robot_controller.goal_y)

        
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

        mask = (lidar_data > 0.0) & (lidar_data < self.sensitivity_bubble)
        collision_indices = np.flatnonzero(mask)

        if collision_indices.size == 0:
            return 0.0, 0.0
        
        front_mask = (collision_indices <= 180) | (collision_indices >= 270)
        collision_indices = collision_indices[front_mask]

        if collision_indices.size == 0:
            return 0.0, 0.0

        distances = lidar_data[collision_indices]
        angles = self.normalized_angle_in_rad[collision_indices]

        weights = distances - robot_controller.dangerous_radius
        weighted_sum = np.sum(angles * weights)

        sum_weight = robot_controller.dangerous_radius * len(collision_indices) - np.sum(distances)
        rebound_angle = weighted_sum / (sum_weight + 1e-6)
        

        if abs(rebound_angle) < ANG_TOL:
            mean_left = np.mean(lidar_data[45:100])
            sign = np.sign(rebound_angle) or 1
            if mean_left < robot_controller.max_lidar_range:
                rebound_angle = sign * (np.pi - ANG_TOL)
            else:
                rebound_angle = sign * (np.pi / 2)

        return 0.2, rebound_angle
    
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

    def get_flocking_gain(self, robot_controller):
        # Must be called after position matching

        if not robot_controller.is_leader:
            return 0.0

        if self.all_rendezvoused:
            return 0.0

        dist_to_rendezvous = get_position_error(robot_controller.current_x,
                                                robot_controller.current_y,
                                                robot_controller.meeting_x,
                                                robot_controller.meeting_y)
        #Check
        self.all_rendezvoused = robot_controller.t1_rendezvoused and robot_controller.t2_rendezvoused

        k = 4.0 if robot_controller.is_leader else 0.0
        x = dist_to_rendezvous - self.rendezvous_distance
        flocking_gain = (1 - math.exp(-k * x)) / (1 + math.exp(-k * x)) if k != 0 else 0.0
        
        if flocking_gain < 0.0:
            flocking_gain = 0.0
        
        return flocking_gain
        
    def execute_control(self, robot_controller):

        # Collision Avoidance
        ca_position_error, ca_yaw_error = self.collision_prevention(robot_controller)
        #ca_position_error, ca_yaw_error = 0.0, 0.0

        if ca_yaw_error != 0.0 and not robot_controller.is_leader:
            total_yaw_error = ca_yaw_error
            total_position_error = ca_position_error
            linear_vel, angular_vel = self.calculate_vel(robot_controller, total_position_error, total_yaw_error)
            
            linear_vel = 0.04
            
            return linear_vel, angular_vel

        # Position Matching (Rendezvous)
        if robot_controller.is_leader:
            pm_position_error, pm_yaw_error = self.position_matching(robot_controller)
        else:
            pm_position_error, pm_yaw_error = 0.0, 0.0

        #robot_controller.get_logger().info(f"PM yaw error: {pm_yaw_error}")
        # Calculate total error with weightage of flocking and goal seeking
        flocking_gain = self.get_flocking_gain(robot_controller)

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
        #robot_controller.get_logger().info(f"Total yaw error: {total_yaw_error}")
        #robot_controller.get_logger().info(f"Flocking Gain: {flocking_gain}")
        total_position_error = fl_gs_position_error
        linear_vel, angular_vel = self.calculate_vel(robot_controller, total_position_error, total_yaw_error)

        if robot_controller.is_leader and flocking_gain < 0.5 and not self.all_rendezvoused:
            linear_vel = 0.02

        if flocking_gain == 0.0:
            if robot_controller.is_leader:
                return self.goal_seeking(robot_controller)
            else:
                return linear_vel, angular_vel
        robot_controller.get_logger().info(f"Flocking gain: {flocking_gain}")
        #robot_controller.get_logger().info(f"All rendezvoused: {self.all_rendezvoused}")
        #robot_controller.get_logger().info(f"Linear Vel: {linear_vel}")
        #robot_controller.get_logger().info(f"Angular Vel: {angular_vel}")
        return linear_vel, angular_vel

    def calculate_vel(self, robot_controller, total_position_error, total_yaw_error):
        # Implementing Method 1: 1 set of PID for all policies
        current_time = time.time()

        linear_x_change = robot_controller.PID_position.compute(total_position_error, current_time)
        angular_z_change = robot_controller.PID_heading.compute(total_yaw_error, current_time)

        return linear_x_change, angular_z_change
    