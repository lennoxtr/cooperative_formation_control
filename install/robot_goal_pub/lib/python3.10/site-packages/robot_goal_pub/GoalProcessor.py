def is_equal(a, b, tol=1e-2):
    return abs(a - b) <= tol

def get_distance(current_x, current_y, goal_x, goal_y):
    delta_x = goal_x - current_x
    delta_y = goal_y - current_y
    distance_squared = delta_x**2 + delta_y**2
    distance = distance_squared**0.5
    return distance

def arrive_at_goal(current_x, current_y, goal_x, goal_y):
    dist_tol = 1e-1
    distance = get_distance(current_x, current_y, goal_x, goal_y)
    return is_equal(distance, 0, dist_tol)