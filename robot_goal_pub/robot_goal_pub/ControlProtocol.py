class ControlProtocol():
    def __init__(self):
        self.position_gain = 1
        self.velocity_gain = 1
        self. leader_follower_gain = 1
        
        #TODO: implement adjacency matrix for imperfect information between robots
        #self.adjacency_matrix = adjacency_matrix

    def position_matching(self):
        #position_error = 
        return
    def velocity_matching(self):
        #velocity_error =
        return
    def leader_follower(self):
        return
    def execute_control(self):
        ### Sum of all control policies
        position_control_output = self.position_matching()
        velocity_control_output = self.velocity_matching()
        leader_follower_output = self.leader_follower()

        control_input = self.position_gain * position_control_output + \
                        self.velocity_gain * velocity_control_output + \
                        self.leader_follower_gain * leader_follower_output

        return control_input