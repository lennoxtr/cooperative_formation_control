#ifndef FORMATION_UTILS_HPP
#define FORMATION_UTILS_HPP

#include <cmath>
#include <vector>
#include <tuple>

namespace FormationUtils {

bool is_equal(double a, double b, double tol = 1e-2);

std::tuple<double, double, double> quaternion_to_euler(const std::vector<double>& q);

double get_position_error(double current_x, double current_y, double goal_x, double goal_y);

double get_target_yaw(double current_x, double current_y, double goal_x, double goal_y);

double normalize_yaw_error(double yaw_error);

double get_yaw_error(double current_x, double current_y, double goal_x, double goal_y, double current_imu_heading);

bool arrived_at_goal(double current_x, double current_y, double goal_x, double goal_y);

double get_angle_increment(int num_of_robots);

std::pair<double, double> get_single_position_in_formation(
    double leader_position_x, double leader_position_y,
    double leader_heading, int follower_robot_id,
    int num_of_robots, double adjacent_distance
);

std::vector<std::pair<int, std::pair<double, double>>> get_all_position_in_formation(
    double leader_position_x, double leader_position_y,
    double leader_heading, const std::vector<int>& follower_robot_id_list,
    double adjacent_distance
);

}

#endif
