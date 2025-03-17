#include "robot_controller_cpp/FormationUtils.hpp"
#include <cmath>
#include <tuple>

namespace FormationUtils {

bool is_equal(double a, double b, double tol) {
    return std::abs(a - b) <= tol;
}

std::tuple<double, double, double> quaternion_to_euler(const std::vector<double>& q) {
    double sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2]);
    double cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q[3] * q[1] - q[2] * q[0]);
    double pitch = (std::abs(sinp) >= 1) ? std::copysign(M_PI / 2, sinp) : std::asin(sinp);

    double siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1]);
    double cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

double get_position_error(double current_x, double current_y, double goal_x, double goal_y) {
    double delta_x = goal_x - current_x;
    double delta_y = goal_y - current_y;
    return std::hypot(delta_x, delta_y);
}

double get_target_yaw(double current_x, double current_y, double goal_x, double goal_y) {
    return std::atan2(goal_y - current_y, goal_x - current_x);
}

double normalize_yaw_error(double yaw_error) {
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    return yaw_error;
}

double get_yaw_error(double current_x, double current_y, double goal_x, double goal_y, double current_imu_heading) {
    double target_yaw = get_target_yaw(current_x, current_y, goal_x, goal_y);
    return normalize_yaw_error(target_yaw - current_imu_heading);
}

bool arrived_at_goal(double current_x, double current_y, double goal_x, double goal_y) {
    double distance = get_position_error(current_x, current_y, goal_x, goal_y);
    return is_equal(distance, 0.0, 1e-1);
}

double get_angle_increment(int num_of_robots) {
    double total_interior_angle = (num_of_robots - 2) * M_PI;
    double interior_angle = total_interior_angle / num_of_robots;
    int remaining_robots = num_of_robots - 2;
    return interior_angle / remaining_robots;
}

std::pair<double, double> get_single_position_in_formation(
    double leader_position_x, double leader_position_y,
    double leader_heading, int follower_robot_id,
    int num_of_robots, double adjacent_distance
) {
    double normalized_leader_heading = std::fmod(leader_heading + 2 * M_PI, 2 * M_PI);
    double angle_increment = get_angle_increment(num_of_robots);
    int remaining_robots = num_of_robots - 2;
    double interior_angle = remaining_robots * angle_increment;
    double initial_angle = (M_PI - interior_angle) / 2.0;
    double relative_angle = angle_increment * (follower_robot_id - 1) + initial_angle;
    double global_frame_angle = normalized_leader_heading - (M_PI / 2 + relative_angle);

    int robot_step = follower_robot_id;
    double actual_distance = adjacent_distance * std::sin(robot_step * M_PI / num_of_robots) / std::sin(M_PI / num_of_robots);

    double x_coord = leader_position_x + actual_distance * std::cos(global_frame_angle);
    double y_coord = leader_position_y + actual_distance * std::sin(global_frame_angle);

    return {x_coord, y_coord};
}

std::vector<std::pair<int, std::pair<double, double>>> get_all_position_in_formation(
    double leader_position_x, double leader_position_y,
    double leader_heading, const std::vector<int>& follower_robot_id_list,
    double adjacent_distance
) {
    std::vector<std::pair<int, std::pair<double, double>>> formation_positions;
    int num_of_robots = follower_robot_id_list.size() + 1;
    for (int id : follower_robot_id_list) {
        auto pos = get_single_position_in_formation(
            leader_position_x, leader_position_y, leader_heading,
            id, num_of_robots, adjacent_distance
        );
        formation_positions.emplace_back(id, pos);
    }
    return formation_positions;
}

} // namespace FormationUtils
