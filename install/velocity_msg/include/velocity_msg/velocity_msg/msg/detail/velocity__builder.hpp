// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from velocity_msg:msg/Velocity.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_MSG__MSG__DETAIL__VELOCITY__BUILDER_HPP_
#define VELOCITY_MSG__MSG__DETAIL__VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "velocity_msg/msg/detail/velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace velocity_msg
{

namespace msg
{

namespace builder
{

class Init_Velocity_linear_y
{
public:
  explicit Init_Velocity_linear_y(::velocity_msg::msg::Velocity & msg)
  : msg_(msg)
  {}
  ::velocity_msg::msg::Velocity linear_y(::velocity_msg::msg::Velocity::_linear_y_type arg)
  {
    msg_.linear_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::velocity_msg::msg::Velocity msg_;
};

class Init_Velocity_linear_x
{
public:
  explicit Init_Velocity_linear_x(::velocity_msg::msg::Velocity & msg)
  : msg_(msg)
  {}
  Init_Velocity_linear_y linear_x(::velocity_msg::msg::Velocity::_linear_x_type arg)
  {
    msg_.linear_x = std::move(arg);
    return Init_Velocity_linear_y(msg_);
  }

private:
  ::velocity_msg::msg::Velocity msg_;
};

class Init_Velocity_robot_id
{
public:
  Init_Velocity_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Velocity_linear_x robot_id(::velocity_msg::msg::Velocity::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_Velocity_linear_x(msg_);
  }

private:
  ::velocity_msg::msg::Velocity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::velocity_msg::msg::Velocity>()
{
  return velocity_msg::msg::builder::Init_Velocity_robot_id();
}

}  // namespace velocity_msg

#endif  // VELOCITY_MSG__MSG__DETAIL__VELOCITY__BUILDER_HPP_
