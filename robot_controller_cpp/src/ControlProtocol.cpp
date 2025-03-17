#include "robot_controller_cpp/ControlProtocol.hpp"
#include "robot_controller_cpp/FormationUtils.hpp"
#include "robot_controller_cpp/RobotController.hpp"

ControlProtocol::ControlProtocol(double rendezvous_distance, int num_of_robot)
    : rendezvous_distance_(rendezvous_distance), num_of_robot_(num_of_robot), avg_position_x_(0), avg_position_y_(0), all_rendezvoused_(false), avg_flocking_gain_(1.0) {
    normalized_angle_in_rad_.resize(360);
    sensitivity_bubble_.resize(360);
    double current_vel = 0.2, delta_t = 3.0;
    for (int i = 0; i < 360; ++i) {
        double angle = i * M_PI / 180.0;
        normalized_angle_in_rad_[i] = std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
        sensitivity_bubble_[i] = get_sensitivity_bubble_gain(i) * current_vel * delta_t;
    }
}

double ControlProtocol::get_sensitivity_bubble_gain(int angle_in_degree) {
    double min_gain = 0.2, max_gain = 1.0;
    double angle_in_rad = normalized_angle_in_rad_[angle_in_degree];
    double A = (max_gain + min_gain) / 2;
    double B = (max_gain - min_gain) / 2;
    return A + B * cos(angle_in_rad);
}

std::pair<double, double> ControlProtocol::position_matching(RobotController& rc, const std::vector<std::pair<double, double>>& pos_map) {
    avg_position_x_ = avg_position_y_ = 0.0;
    for (const auto& pos : pos_map) {
        avg_position_x_ += pos.first;
        avg_position_y_ += pos.second;
    }
    avg_position_x_ /= num_of_robot_;
    avg_position_y_ /= num_of_robot_;

    double pos_error = FormationUtils::get_position_error(rc.current_x, rc.current_y, avg_position_x_, avg_position_y_);
    double yaw_error = FormationUtils::get_yaw_error(rc.current_x, rc.current_y, avg_position_x_, avg_position_y_, rc.current_imu_heading);
    rc.is_rendezvoused = (pos_error < rendezvous_distance_);
    return {pos_error, yaw_error};
}

double ControlProtocol::velocity_matching(RobotController& rc, const std::vector<double>& vel_map) {
    double sum_vel = std::accumulate(vel_map.begin(), vel_map.end(), 0.0);
    return num_of_robot_ * rc.linear_x_velocity - sum_vel;
}

double ControlProtocol::heading_matching(RobotController& rc, const std::vector<double>& head_map) {
    double error = rc.leader_heading - rc.current_imu_heading;
    return FormationUtils::normalize_yaw_error(error);
}

std::pair<double, double> ControlProtocol::collision_prevention(RobotController& rc) {
    auto& lidar = rc.lidar_data;
    std::vector<int> coll_angles;

    for (int i = 0; i < lidar.size(); i++) {
        if (lidar[i] > 0.0 && lidar[i] < sensitivity_bubble_[i] && (i >= 270 || i <= 180))
            coll_angles.push_back(i);
    }

    if (coll_angles.empty()) return {0.0, 0.0};

    std::vector<double> distances, angles;
    for (auto angle : coll_angles) {
        distances.push_back(lidar[angle]);
        angles.push_back(normalized_angle_in_rad_[angle]);
    }

    double min_distance = *std::min_element(distances.begin(), distances.end());
    double pos_error = 0.2;
    double rebound_angle = ANG_TOL; // simplified for now

    return {pos_error, rebound_angle};
}

std::pair<double, double> ControlProtocol::leader_follower(RobotController& rc) {
    return {
        FormationUtils::get_position_error(rc.current_x, rc.current_y, rc.goal_x, rc.goal_y),
        FormationUtils::get_yaw_error(rc.current_x, rc.current_y, rc.goal_x, rc.goal_y, rc.current_imu_heading)
    };
}

double ControlProtocol::get_flocking_gain(RobotController& rc, const std::vector<std::pair<double, double>>& pos_map) {
    double sum_dist = 0.0;
    for (const auto& pos : pos_map)
        sum_dist += FormationUtils::get_position_error(pos.first, pos.second, avg_position_x_, avg_position_y_);

    double avg_dist = sum_dist / num_of_robot_;
    double dist_to_rendezvous = FormationUtils::get_position_error(rc.current_x, rc.current_y, avg_position_x_, avg_position_y_);
    double k = rc.is_leader ? 7 : 0;
    double fg = (1 - exp(-k * (dist_to_rendezvous - rendezvous_distance_))) / (1 + exp(-k * (dist_to_rendezvous - rendezvous_distance_)));
    return std::max(fg, 0.0);
}

std::pair<double, double> ControlProtocol::calculate_control(RobotController& rc, const std::vector<std::pair<double, double>>& pos_map, const std::vector<double>& vel_map, const std::vector<double>& head_map) {
    auto [ca_pe, ca_ye] = collision_prevention(rc);
    if (ca_ye != 0.0 && !all_rendezvoused_) return {ca_pe, ca_ye};

    auto [pm_pe, pm_ye] = position_matching(rc, pos_map);
    double flock_gain = get_flocking_gain(rc, pos_map);
    auto [lf_pe, lf_ye] = leader_follower(rc);
    double heading_err = heading_matching(rc, head_map);

    double total_pe = (1 - flock_gain) * lf_pe + flock_gain * pm_pe;
    double total_ye = rc.is_leader ? ((1 - flock_gain) * lf_ye + flock_gain * pm_ye) : heading_err;

    return {total_pe, total_ye};
}

std::pair<double, double> ControlProtocol::execute_control(RobotController& rc, const std::vector<std::pair<double, double>>& pos_map, const std::vector<double>& vel_map, const std::vector<double>& head_map) {
    auto [total_pe, total_ye] = calculate_control(rc, pos_map, vel_map, head_map);
    auto now = std::chrono::steady_clock::now();
    double time_now = std::chrono::duration<double>(now.time_since_epoch()).count();

    double linear = rc.PID_position.compute(total_pe, time_now);
    double angular = rc.PID_heading.compute(total_ye, time_now);

    return {linear, angular};
}
