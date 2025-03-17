cmake_minimum_required(VERSION 3.8)
project(robot_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(velocity_msg REQUIRED)
find_package(heading_msg REQUIRED)
find_package(robot_goal REQUIRED)

add_executable(robot_controller_cpp src/RobotController.cpp)
ament_target_dependencies(robot_controller_cpp rclcpp std_msgs gazebo_msgs sensor_msgs velocity_msg heading_msg robot_goal)

install(TARGETS
  robot_controller_cpp
  DESTINATION lib/${PROJECT_NAME})

ament_package()
