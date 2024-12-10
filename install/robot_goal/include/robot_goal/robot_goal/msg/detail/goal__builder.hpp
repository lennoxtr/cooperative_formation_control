// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_goal:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_GOAL__MSG__DETAIL__GOAL__BUILDER_HPP_
#define ROBOT_GOAL__MSG__DETAIL__GOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_goal/msg/detail/goal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_goal
{

namespace msg
{

namespace builder
{

class Init_Goal_goal_y
{
public:
  explicit Init_Goal_goal_y(::robot_goal::msg::Goal & msg)
  : msg_(msg)
  {}
  ::robot_goal::msg::Goal goal_y(::robot_goal::msg::Goal::_goal_y_type arg)
  {
    msg_.goal_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_goal::msg::Goal msg_;
};

class Init_Goal_goal_x
{
public:
  Init_Goal_goal_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Goal_goal_y goal_x(::robot_goal::msg::Goal::_goal_x_type arg)
  {
    msg_.goal_x = std::move(arg);
    return Init_Goal_goal_y(msg_);
  }

private:
  ::robot_goal::msg::Goal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_goal::msg::Goal>()
{
  return robot_goal::msg::builder::Init_Goal_goal_x();
}

}  // namespace robot_goal

#endif  // ROBOT_GOAL__MSG__DETAIL__GOAL__BUILDER_HPP_
