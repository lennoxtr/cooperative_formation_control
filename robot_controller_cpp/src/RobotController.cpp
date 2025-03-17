#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <sstream>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "heading_msg/msg/heading.hpp"
#include "velocity_msg/msg/velocity.hpp"
#include "robot_goal/msg/goal.hpp"
#include "position_mapping_msg/msg/position_mapping.hpp"

// PID Controller & other utilities
#include "robot_controller_cpp/RobotController.hpp"

RobotController::RobotController(bool is_leader)
    : Node("RobotController"),
    is_leader_(is_leader),
    is_started_(false),
    received_goal_(false),
    rendezvous_distance_(2.0),
    control_protocol_(rendezvous_distance_),
    is_rendezvoused_(false),
    max_lidar_range_(3.5),
    dangerous_radius_(0.6),
    linear_x_velocity_(0.0),
    angular_z_velocity_(0.0),
    PID_position_(1.0, 0.0, 0.0),
    PID_heading_(5.0, 0.0, 0.1),
    is_in_formation_(false)
    {
        this->declare_parameter<int>("robot_id", 0);
        robot_id_ = this->get_parameter("robot_id").as_int();
        namespace_ = "turtlebot" + std::to_string(robot_id_);

        // Subscribers
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/" + namespace_ + "/imu", rclcpp::SensorDataQoS(), std::bind(&RobotController::imu_callback, this, std::placeholders::_1));
        
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + namespace_ + "/odom", 10, std::bind(&RobotController::odom_callback, this, std::placeholders::_1));
        
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/" + namespace_ + "/scan", rclcpp::SensorDataQoS(), std::bind(&RobotController::lidar_callback, this, std::placeholders::_1));
        
        goal_subscription_ = this->create_subscription<robot_goal::msg::Goal>(
            "/" + namespace_ + "/goal", 10, std::bind(&RobotController::goal_listener_callback, this, std::placeholders::_1));

        is_leader_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/leader", 10, std::bind(&RobotController::is_leader_callback, this, std::placeholders::_1));
    
        is_started_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start", 10, std::bind(&RobotController::is_started_callback, this, std::placeholders::_1));

        tracking_position_subscription_ = this->create_subscription<robot_goal::msg::Goal>(
            "/" + namespace_ + "/tracking_position", 10, std::bind(&RobotController::tracking_position_callback, this, std::placeholders::_1));
        
        leader_heading_subscription_ = this->create_subscription<heading_msg::msg::Heading>(
            "/leader_heading", 10, std::bind(&RobotController::leader_heading_callback, this, std::placeholders::_1));

        current_position_subscription_ = this->create_subscription<robot_goal::msg::Goal>(
            "/" + namespace_ + "/robot_position", 10, std::bind(&RobotController::current_position_callback, this, std::placeholders::_1));
        
        position_mapping_subscription_ = this->create_subscription<position_mapping_msg::msg::PositionMapping>(
            "/position_mapping", 10, std::bind(&RobotController::position_mapping_callback, this, std::placeholders::_1));

        velocity_mapping_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/velocity_mapping", 10, std::bind(&RobotController::velocity_mapping_callback, this, std::placeholders::_1));
        
        heading_mapping_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/heading_mapping", 10, std::bind(&RobotController::heading_mapping_callback, this, std::placeholders::_1));
        
        arrived_at_goal_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/arrived_at_goal", 10, std::bind(&RobotController::arrived_at_goal_callback, this, std::placeholders::_1));

        // Timer
        heartbeat_timer_ = this->create_wall_timer(1s, std::bind(&RobotController::heartbeat_timer_callback, this));

        // Publishers
        heartbeat_publisher_ = this->create_publisher<std_msgs::msg::String>("/heartbeat", 10);
        arrive_at_goal_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/arrived_at_goal", 10);
        self_twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + namespace_ + "/cmd_vel", 10);
        controller_velocity_publisher_ = this->create_publisher<velocity_msg::msg::Velocity>("/robot_linear_vel", 10);
        heading_publisher_ = this->create_publisher<heading_msg::msg::Heading>("/robot_heading", 10);
        leader_heading_publisher_ = this->create_publisher<heading_msg::msg::Heading>("/leader_heading", 10);
    }  
        // Callbacks
        void RobotController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
            auto orientation_q = msg->orientation;
            std::vector<double> quaternion = {orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w};

            std::vector<double> euler = FormationUtils::quaternion_to_euler(quaternion);
            double yaw = euler[2];

            current_imu_heading_ = round(yaw * 1000.0) / 1000.0;
            heading_msg::msg::Heading msg_out;
            msg_out.robot_id = robot_id_;
            msg_out.heading = current_imu_heading_;
            heading_publisher_->publish(msg_out);

            if (is_leader_) {
                leader_heading_ = current_imu_heading;
                leader_heading_publisher_->publish(msg_out);
            }
        }

        void RobotController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            auto linear_velocity = msg->twist.twist.linear;
            auto linear_x_ = linear_velocity.x;
            auto linear_y_ = linear_velocity.y;

            velocity_msg::msg::Velocity velocity_msg;
            velocity_msg.robot_id = robot_id_;
            velocity_msg.linear_x = linear_x_;
            velocity_msg.linear_y = linear_y_;
            controller_velocity_publisher_->publish(velocity_msg);
        }

        void RobotController::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            lidar_data_ = msg->ranges;
            for (auto &range : lidar_data_) {
                if (range == std::numeric_limits<float>::infinity()) range = max_lidar_range_;
            }
        }

        void RobotController::goal_listener_callback(const robot_goal::msg::Goal::SharedPtr msg) {
            goal_x_ = msg->goal_x;
            goal_y_ = msg->goal_y;
            received_goal_ = true;
        }

        void RobotController::is_leader_callback(const std_msgs::msg::String::SharedPtr msg) {
            auto leader_namespace = msg->data;
            if (namespace_ == leader_namespace) {
                is_leader_ = true;
            }
        }

        void RobotController::is_started_callback(const std_msgs::msg::Bool::SharedPtr msg) {
            is_started_ = msg->data;
        }

        void RobotController::tracking_position_callback(const robot_goal::msg::Goal::SharedPtr msg) {
            goal_x_ = std::round(msg->goal_x * 1000.0) / 1000.0;
            goal_y_ = std::round(msg->goal_y * 1000.0) / 1000.0;
        }

        void RobotController::leader_heading_callback(const heading_msg::msg::Heading::SharedPtr msg) {
            leader_heading_ = msg->heading;
        }

        void RobotController::current_position_callback(const robot_goal::msg::Goal::SharedPtr msg) {
            current_x_ = std::round(msg->goal_x * 1000.0) / 1000.0;
            current_y_ = std::round(msg->goal_y * 1000.0) / 1000.0;
        }

        void RobotController::position_mapping_callback(const position_mapping_msg::msg::PositionMapping::SharedPtr msg) {
            position_mapping_.clear();
            for (const auto& position : msg->data) {
                position_mapping_.push_back({position.x, position.y});
            }
        }

        void RobotController::velocity_mapping_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            velocity_mapping_.clear();
            for (const auto& value : msg->data) {
                velocity_mapping_.push_back(static_cast<float>(value));
            }
        }

        void RobotController::heading_mapping_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            heading_mapping_.clear();
            for (const auto& value : msg->data) {
                heading_mapping_.push_back(static_cast<float>(value)); // Cast double to float
            }
        }

        void RobotController::arrived_at_goal_callback(const std_msgs::msg::Bool::SharedPtr msg) {
            auto arrived_at_goal_ = msg->data;
            if (arrived_at_goal_) {
                stop_bot();
                std::cout << namespace_ << " arrived" << std::endl;
            }
        }

        void RobotController::heartbeat_timer_callback() {
            if (!is_started_) {
                auto message = std_msgs::msg::String();
                message.data = namespace_;
                heartbeat_publisher_->publish(message);
            }
        }

        void RobotController::move_bot(float linear_x, float angular_z) {
            linear_x_velocity_ = linear_x;
            angular_z_velocity_ = angular_z;

            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = linear_x;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = angular_z;

            self_twist_publisher_->publish(twist);
        }

        void RobotController::stop_bot() {
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;

            self_twist_publisher_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "%s arrived", namespace_.c_str());
        }

        bool RobotController::is_arrived() {
            return FormationUtils::arrived_at_goal(current_x_, current_y_, goal_x_, goal_y_);
        }

        void RobotController::execute() {
            if (!is_started_) {
                return;
            }
            if (is_leader_) {
                RCLCPP_INFO(this->get_logger(), "-----------------------------------");
                RCLCPP_INFO(this->get_logger(), "Leader Robot");
                RCLCPP_INFO(this->get_logger(), "Heading: %.3f", current_imu_heading_);
                RCLCPP_INFO(this->get_logger(), "Position x: %.3f", current_x_);
                RCLCPP_INFO(this->get_logger(), "Position y: %.3f", current_y_);
            

                std::vector<std::pair<int, std::array<float, 2>>> robot_formation_position_list =
                            FormationUtils::get_all_position_in_formation(current_x_, current_y_, current_imu_heading_,
                                          follower_robot_id_list_, 1.0); 
                
                for (const auto &item : robot_formation_position_list) {
                    int robot_id = item.first;
                    std::string namespace_ = "turtlebot" + std::to_string(robot_id);
                    float position_x = item.second[0];
                    float position_y = item.second[1];
                                  
                    RCLCPP_INFO(this->get_logger(), "Robot id: %d", robot_id);
                    RCLCPP_INFO(this->get_logger(), "Position x: %.3f", position_x);
                    RCLCPP_INFO(this->get_logger(), "Position y: %.3f", position_y);
                                  
                    // Preparing tracking position message
                    robot_goal::msg::Goal msg;
                    msg.goal_x = position_x;
                    msg.goal_y = position_y;

                    auto tracking_position_publisher = this->create_publisher<robot_goal::msg::Goal>(
                        "/" + namespace_ + "/tracking_position", 10);
                        tracking_position_publisher->publish(msg);
                }

                if (is_arrived() && is_rendezvoused_) {
                    std_msgs::msg::Bool arrived_msg;
                    arrived_msg.data = true;
                    arrive_at_goal_publisher_->publish(arrived_msg);
                }
            } else {
                is_in_formation_ = is_arrived();
            }

            auto [linear_x_change, angular_z_change] = control_protocol_.execute_control(
                *this, position_mapping_, velocity_mapping_, heading_mapping_);
            
            float target_angular_velocity;
            if (std::abs(angular_z_change) > MAX_ANGLE_VEL) {
                target_angular_velocity = angular_z_change / std::abs(angular_z_change) * MAX_ANGLE_VEL;
            } else {
                target_angular_velocity = angular_z_change;
            }

            float target_linear_velocity;
            if (std::abs(linear_x_change) > MAX_LINEAR_VEL) {
                target_linear_velocity = linear_x_change / std::abs(linear_x_change) * MAX_LINEAR_VEL;
            } else {
                target_linear_velocity = linear_x_change;
            }

            move_bot(target_linear_velocity, target_angular_velocity);
        }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread executor_thread([executor]() { executor->spin(); });

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        node->execute();
        loop_rate.sleep();
    }

    executor->cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}






