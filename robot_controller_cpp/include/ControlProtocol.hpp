#ifndef CONTROL_PROTOCOL_HPP
#define CONTROL_PROTOCOL_HPP

#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <chrono>

#include "FormationUtils.hpp"
#include "RobotController.hpp"

#define ANG_TOL 0.2
#define POSITION_TOL 0.05

class ControlProtocol {

public:
    ControlProtocol(double rendezvous_distance, int num_of_robot = 4);

    std::pair<double, double> position_matching(RobotController& robot_controller, const std::vector<std::pair<double, double>>& position_mapping);
    double velocity_matching(RobotController& robot_controller, const std::vector<double>& velocity_mapping);
    double heading_matching(RobotController& robot_controller, const std::vector<double>& heading_mapping);
    std::pair<double, double> collision_prevention(RobotController& robot_controller);
    std::pair<double, double> leader_follower(RobotController& robot_controller);
    double get_flocking_gain(RobotController& robot_controller, const std::vector<std::pair<double, double>>& position_mapping);

    std::pair<double, double> calculate_control(RobotController& robot_controller, const std::vector<std::pair<double, double>>& position_mapping, const std::vector<double>& velocity_mapping, const std::vector<double>& heading_mapping);
    std::pair<double, double> execute_control(RobotController& robot_controller, const std::vector<std::pair<double, double>>& position_mapping, const std::vector<double>& velocity_mapping, const std::vector<double>& heading_mapping);

private:
    double get_sensitivity_bubble_gain(int angle_in_degree);

    int num_of_robot_;
    double rendezvous_distance_;
    double avg_position_x_, avg_position_y_;
    bool all_rendezvoused_;
    double avg_flocking_gain_;
    std::vector<double> normalized_angle_in_rad_;
    std::vector<double> sensitivity_bubble_;
};

#endif // CONTROL_PROTOCOL_HPP
