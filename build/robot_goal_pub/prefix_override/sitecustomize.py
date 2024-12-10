import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lennoxtr/turtlebot3_ws/src/install/robot_goal_pub'
