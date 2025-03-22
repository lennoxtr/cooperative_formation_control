import numpy as np
import math
from scipy.optimize import minimize 
from scipy.optimize import linear_sum_assignment

def is_equal(a, b, tol=1e-2):
    return abs(a - b) <= tol

def quaternion_to_euler(q):
    """Convert a quaternion into euler angles (roll, pitch, yaw)."""
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2])
    cosr_cosp = 1 - 2 * (q[0]**2 + q[1]**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q[3] * q[1] - q[2] * q[0])
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
    cosy_cosp = 1 - 2 * (q[1]**2 + q[2]**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def get_position_error(current_x, current_y, goal_x, goal_y):
    delta_x = goal_x - current_x
    delta_y = goal_y - current_y
    distance_squared = delta_x**2 + delta_y**2
    distance = distance_squared**0.5
    return distance

def get_target_yaw(current_x, current_y, goal_x, goal_y):
    # Target yaw is relative to true north
    delta_x = goal_x - current_x
    delta_y = goal_y - current_y
    target_heading = np.arctan2(delta_y, delta_x)
    target_yaw = float("{:.3f}".format(target_heading))
    return target_yaw

def normalize_yaw_error(yaw_error):
    # Normalize yaw_error to [-π, π]
    if yaw_error > np.pi:
        yaw_error -= 2 * np.pi
    elif yaw_error < -np.pi:
        yaw_error += 2 * np.pi
    
    return yaw_error

def get_yaw_error(current_x, current_y, goal_x, goal_y, current_imu_heading):
    target_yaw = get_target_yaw(current_x, current_y, goal_x, goal_y)
    yaw_error = target_yaw - current_imu_heading

    # Normalize yaw_error to [-π, π]
    yaw_error = normalize_yaw_error(yaw_error)
    return yaw_error

def arrived_at_goal(current_x, current_y, goal_x, goal_y):
    dist_tol = 1e-1
    distance = get_position_error(current_x, current_y, goal_x, goal_y)
    return is_equal(distance, 0, dist_tol)

def generate_straight_path(current_x, current_y, goal_x, goal_y, step_size = 0.1):
    distance = np.hypot(goal_x - current_x, goal_y - current_y)
    num_steps = int(distance / step_size)

    x_points = np.linspace(current_x, goal_x, num_steps)
    y_points = np.linspace(current_y, goal_y, num_steps)
    
    path = list(zip(x_points, y_points))
    return path

# this is essentially derived from the shape function
def get_angle_increment(num_of_robots):
    total_interior_angle = (num_of_robots - 2) * np.pi
    interior_angle = total_interior_angle / num_of_robots

    remaining_robots = num_of_robots - 2
    angle_increment = interior_angle / remaining_robots
    return angle_increment

# Control the shape of formation
# Right now, formation is regular polygon
def get_single_position_in_formation(leader_position_x, leader_position_y, leader_heading, follower_robot_id, num_of_robots, adjacent_distance):
    # Normalize leader heading to 0 to 2 pi
    normalized_leader_heading = (leader_heading + 2 * np.pi) % (2 * np.pi)

    angle_increment = get_angle_increment(num_of_robots)

    remaining_robots = num_of_robots - 2
    interior_angle = remaining_robots * angle_increment
    initial_angle = (np.pi - interior_angle) / 2

    # This is because index 0 is leader
    # TODO: make this more generalized
    relative_angle_from_leader = angle_increment * (follower_robot_id - 1) + initial_angle

    global_frame_angle = normalized_leader_heading - (np.pi / 2 + relative_angle_from_leader)

    robot_step = follower_robot_id
    actual_distance = adjacent_distance * math.sin(robot_step * np.pi / num_of_robots) / math.sin(np.pi / num_of_robots)

    x_coord_formation = leader_position_x + actual_distance * math.cos(global_frame_angle)
    y_coord_formation = leader_position_y + actual_distance * math.sin(global_frame_angle)
    return (x_coord_formation, y_coord_formation)


# Called only by leader robot to project formation
def get_all_postion_in_formation(leader_position_x, leader_position_y, leader_heading, follower_robot_id_list, adjacent_distance):
    position_in_formation_list = []
    num_of_robots = len(follower_robot_id_list) + 1
    for follower_robot_id in follower_robot_id_list:
        position_in_formation = get_single_position_in_formation(leader_position_x,
                                                                leader_position_y,
                                                                leader_heading,
                                                                follower_robot_id,
                                                                num_of_robots,
                                                                adjacent_distance)
        position_in_formation_list.append((follower_robot_id, position_in_formation))
    
    return position_in_formation_list

# Helper function to calculate rendezvous
def arrival_time_error(p, positions):
    x, y, T = p  # Unpack optimization variables
    velocity = 0.2
    errors = [
        (np.sqrt((x - x_i) ** 2 + (y - y_i) ** 2) - velocity * T) ** 2
        for (x_i, y_i) in positions
    ]
    return sum(errors)



def get_rendezvous_pos(position_mapping):
    x0, y0 = np.mean(position_mapping, axis=0)
    avg_distance = np.mean([np.linalg.norm(np.array(pos) - np.array((x0, y0))) for pos in position_mapping])
    T0 = avg_distance / 0.2
    initial_guess = (x0, y0, T0)
    result = minimize(arrival_time_error, initial_guess, args=(position_mapping), method='Nelder-Mead')
    meeting_point = tuple(map(float, result.x[:2]))

    return meeting_point