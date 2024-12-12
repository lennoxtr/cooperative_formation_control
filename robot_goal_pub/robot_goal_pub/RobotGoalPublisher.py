import rclpy
import time
from rclpy.node import Node


from gazebo_msgs.msg import ModelStates
from robot_goal.msg import Goal

from robot_goal_pub.GoalProcessor import get_distance
from robot_goal_pub.PidController import PidController

from robot_goal_pub.RobotController import RobotController

class RobotGoalPublisher(Node):
    def __init__(self):
        super().__init__('robot_goal_publisher')
        self.received_goal = False
        self.computed_current_yaw = False
        self.received_position_updated = False
        self.leader_namespace = 'turtlebot0'

        self.goal_x = 0
        self.goal_y = 0

        self.goal_subscription = self.create_subscription(
            Goal,
            '/robot_goal',
            self.goal_listener_callback,
            10)

        self.position_subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.position_listener_callback,
            10)

        self.robot_controller_map = {}
        for i in range(5):
            namespace = 'turtlebot' + str(i)
            if namespace == self.leader_namespace:
                robot_controller = RobotController(namespace, is_leader=True)
                rclpy.spin_once(robot_controller)
                self.robot_controller_map[namespace] = robot_controller
            else:
                robot_controller = RobotController(namespace)
                rclpy.spin_once(robot_controller)
                self.robot_controller_map[namespace] = robot_controller

    def goal_listener_callback(self, msg):
        self.goal_x = float("{:.3f}".format(msg.goal_x))
        self.goal_y = float("{:.3f}".format(msg.goal_y))
        self.received_goal = True
        # TODO: set PID for each robot
        self.PID_position = PidController(Kp=0.5,Ki=0.05,Kd=0.06)
        self.PID_heading = PidController(Kp=0.6,Ki=0.05,Kd=0.06)
        self.get_logger().info(f"Goal set to {self.goal_x}, {self.goal_y}")

    def position_listener_callback(self, msg):
        for i, namespace in enumerate(msg.name):
            position = msg.pose[i].position
            current_x = float("{:.3f}".format(position.x))
            current_y = float("{:.3f}".format(position.y))
            self.robot_controller_map[namespace].update_position(current_x, current_y)
        self.received_position_updated = True
        
    def execute(self):
        rclpy.spin_once(self)
        # Have not received goal
        while not self.received_goal:
            return
        
        # Robot formation moving to goal
        leader_robot = self.robot_controller_map[self.leader_namespace]
        while not leader_robot.arrived_at_goal():
            while not (self.computed_current_yaw and self.received_position_updated):
                rclpy.spin_once(self)

            position_error = get_distance(self.current_x, self.current_y, self.goal_x, self.goal_y)

            target_yaw = self.calculate_target_yaw()
            yaw_error = target_yaw - self.current_yaw

            #print(position_error)

            ## get time
            current_time = time.time()
            linear_x_change = self.PID_position.compute(position_error, current_time)
            angular_z_change = self.PID_heading.compute(yaw_error, current_time)
            #print(angular_z_change)

            # Move to goal
            self.move_to_goal(linear_x_change, angular_z_change)

            self.computed_current_yaw = False
            self.received_position_updated = False
            rclpy.spin_once(self)
        
        for robot_controller in self.robot_controller_map:
            self.robot_controller_map[robot_controller].stop_bot()

def main(args=None):
    rclpy.init(args=args)
    robot_goal_publisher = RobotGoalPublisher()

    while True:
        robot_goal_publisher.execute()
    
    robot_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
