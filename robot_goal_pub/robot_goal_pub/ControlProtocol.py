import time

from robot_goal_pub.GoalProcessor import get_position_error
from robot_goal_pub.GoalProcessor import get_yaw_error

class ControlProtocol():
    def __init__(self, num_of_robot):
        self.position_gain = 0
        self.velocity_gain = 0
        self.heading_gain = 0
        self.leader_follower_gain = 1
        self.num_of_robot = num_of_robot

        #TODO: implement adjacency matrix for imperfect information between robots
        #self.adjacency_matrix = adjacency_matrix

    def position_matching(self, robot_controller, position_mapping):
        # position matching is mainly for leader as 
        a_ij_val = 1
        #position_error = 4 * a_ij_val * current_robot_pos - 
        return 1
    
    def velocity_matching(self, robot_controller, velocity_mapping):
        a_ij_val = 1
        velocity_error = self.num_of_robot * a_ij_val * robot_controller.linear_x_velocity - \
                            a_ij_val * sum(velocity_mapping)
        return velocity_error
    
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

        current_time = time.time()
        linear_x_change = robot_controller.PID_position.compute(position_error, current_time)
        angular_z_change = robot_controller.PID_heading.compute(yaw_error, current_time)

        return linear_x_change, angular_z_change
        
    def execute_control(self, robot_controller, position_mapping, velocity_mapping, heading_mapping):
        ### Sum of all control policies
        position_control_output = self.position_matching(robot_controller, position_mapping)
        velocity_control_output = self.velocity_matching(robot_controller, velocity_mapping)
        lf_linear_x_output, lf_angular_z_output = self.leader_follower(robot_controller)
        heading_control_output = self.heading_matching(robot_controller, heading_mapping)

        control_linear_x_input = self.leader_follower_gain * lf_linear_x_output
        control_angular_z_input = self.leader_follower_gain * lf_angular_z_output

        '''
        control_input = self.position_gain * position_control_output + \
                        self.velocity_gain * velocity_control_output + \
                        self.heading_gain * heading_control_output + \
                        self.leader_follower_gain * leader_follower_output
        '''

        return control_linear_x_input, control_angular_z_input