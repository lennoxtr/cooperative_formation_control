#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <heading_msg/msg/heading.hpp>
#include <velocity_msg/msg/velocity.hpp>
#include <robot_goal/msg/goal.hpp>
#include <position_mapping_msg/msg/position_mapping.hpp>

#include <thread>
#include <vector>
#include <array>
#include <cmath>

#include "PidController.hpp"
#include "ControlProtocol.hpp"
#include "FormationUtils.hpp"

#define MAX_LINEAR_VEL 0.2
#define MAX_ANGLE_VEL 1.5
#define LIN_VEL_STEP_SIZE 0.01
#define ANG_VEL_STEP_SIZE 0.1
#define MAX_LIDAR_RANGE 3.5

class RobotController : public rclcpp::Node
{
public: 
    explicit RobotController(bool is_leader = false);

private:
    // Identification
    int robot_id_;
    std::string namespace_;
    bool is_leader_;

    // Start execution flag
    bool is_started_;
    bool received_goal_;

    // Control Protocol
    float rendezvous_distance_;
    ControlProtocol control_protocol_;

    // Mappings for control
    std::vector<std::array<float, 2>> position_mapping_;
    std::vector<float> velocity_mapping_;
    std::vector<float> heading_mapping_;

    // Rendezvous Flag
    bool is_rendezvoused_;

    // Collision avoidance threshold
    double max_lidar_range_;
    float dangerous_radius_;

    // Lidar data for collision avoidance
    std::array<float, 360> lidar_data;

    // Position variables
    double goal_x_, goal_y_;
    double current_x_, current_y_;
    double leader_heading_;

    // Kinematic variables
    // Yaw is +- pi from north
    double current_imu_heading_;
    double linear_x_velocity_;
    double angular_z_velocity_;

    // Kinematic PID Controller
    PidController PID_position_;
    PidController PID_heading_;

    // Formation
    bool is_in_formation_;
    bool arrived_at_goal_;

    // TODO: Remove this as it is hard-coded
    std::array<int, 3> follower_robot_id_list_ = {1, 2, 3};

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<robot_goal::msg::Goal>::SharedPtr goal_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr is_leader_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_started_subscription_;
    rclcpp::Subscription<robot_goal::msg::Goal>::SharedPtr tracking_position_subscription_;
    rclcpp::Subscription<heading_msg::msg::Heading>::SharedPtr leader_heading_subscription_;
    rclcpp::Subscription<robot_goal::msg::Goal>::SharedPtr current_position_subscription_;
    rclcpp::Subscription<position_mapping_msg::msg::PositionMapping>::SharedPtr position_mapping_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_mapping_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr heading_mapping_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_at_goal_subscription_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrive_at_goal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr self_twist_publisher_;
    rclcpp::Publisher<velocity_msg::msg::Velocity>::SharedPtr controller_velocity_publisher_;
    rclcpp::Publisher<heading_msg::msg::Heading>::SharedPtr heading_publisher_;
    rclcpp::Publisher<heading_msg::msg::Heading>::SharedPtr leader_heading_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // Callback functions
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void goal_listener_callback(const robot_goal::msg::Goal::SharedPtr msg);
    void is_leader_callback(const std_msgs::msg::String::SharedPtr msg);
    void is_started_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void tracking_position_callback(const robot_goal::msg::Goal::SharedPtr msg);
    void leader_heading_callback(const heading_msg::msg::Heading::SharedPtr msg);
    void current_position_callback(const robot_goal::msg::Goal::SharedPtr msg);
    void position_mapping_callback(const position_mapping_msg::msg::PositionMapping::SharedPtr msg);
    void velocity_mapping_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void heading_mapping_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void arrived_at_goal_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void heartbeat_timer_callback();

    // Control methods
    void move_bot(float linear_x_change, float angular_z_change);
    void stop_bot();
    bool is_arrived();
    void execute();

};

#endif // ROBOT_CONTROLLER_HPP