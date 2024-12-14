class ControlProtocol():
    def __init__(self, num_of_robot):
        self.position_gain = 1
        self.velocity_gain = 1
        self.heading_gain = 1
        self. leader_follower_gain = 1
        self.num_of_robot = num_of_robot

        #TODO: implement adjacency matrix for imperfect information between robots
        #self.adjacency_matrix = adjacency_matrix

    def position_matching(self, current_robot_pos, all_robot_pos_list):
        a_ij_val = 1
        #position_error = 4 * a_ij_val * current_robot_pos - 
        return
    def velocity_matching(self, current_robot_vel, all_robot_vel_list):
        a_ij_val = 1
        velocity_error = self.num_of_robot * a_ij_val * current_robot_vel - \
                            a_ij_val * sum(all_robot_vel_list)
        return velocity_error
    def heading_matching(self, current_robot_heading, all_robot_heading_list):
        a_ij_val = 1
        heading_error = self.num_of_robot * a_ij_val * current_robot_heading - \
                            a_ij_val * sum(all_robot_heading_list)
        return heading_error
    def leader_follower(self):
        return
    def execute_control(self):
        ### Sum of all control policies
        position_control_output = self.position_matching()
        velocity_control_output = self.velocity_matching()
        leader_follower_output = self.leader_follower()
        heading_control_output = self.heading_matching()

        control_input = self.position_gain * position_control_output + \
                        self.velocity_gain * velocity_control_output + \
                        self.heading_gain * heading_control_output + \
                        self.leader_follower_gain * leader_follower_output

        return control_input