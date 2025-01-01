import numpy as np

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