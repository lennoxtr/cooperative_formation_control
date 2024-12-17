import time

from robot_goal_pub.GoalProcessor import get_position_error
from robot_goal_pub.GoalProcessor import get_yaw_error

class ControlProtocol():
    def __init__(self, num_of_robot):
        self.position_gain = 0.2
        self.velocity_gain = 0
        self.heading_gain = 0
        self.leader_follower_gain = 0.8
        self.num_of_robot = num_of_robot

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

        avg_position_x = avg_position_x / self.num_of_robot
        avg_position_y = avg_position_y / self.num_of_robot

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
    
    def collision_prevention(self, robot_controller, position_mapping):
        # Needs to be implemented in all control algo
        return
    
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

        # Calculate total error with weightage
        total_position_error = self.leader_follower_gain * lf_position_error + \
                                self.position_gain * pc_position_error

        total_yaw_error = self.leader_follower_gain * lf_yaw_error + \
                            self.position_gain * pc_yaw_error

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