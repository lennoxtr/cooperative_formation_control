// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from heading_msg:msg/Heading.idl
// generated code does not contain a copyright notice

#ifndef HEADING_MSG__MSG__DETAIL__HEADING__BUILDER_HPP_
#define HEADING_MSG__MSG__DETAIL__HEADING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "heading_msg/msg/detail/heading__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace heading_msg
{

namespace msg
{

namespace builder
{

class Init_Heading_heading
{
public:
  explicit Init_Heading_heading(::heading_msg::msg::Heading & msg)
  : msg_(msg)
  {}
  ::heading_msg::msg::Heading heading(::heading_msg::msg::Heading::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return std::move(msg_);
  }

private:
  ::heading_msg::msg::Heading msg_;
};

class Init_Heading_robot_id
{
public:
  Init_Heading_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Heading_heading robot_id(::heading_msg::msg::Heading::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_Heading_heading(msg_);
  }

private:
  ::heading_msg::msg::Heading msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::heading_msg::msg::Heading>()
{
  return heading_msg::msg::builder::Init_Heading_robot_id();
}

}  // namespace heading_msg

#endif  // HEADING_MSG__MSG__DETAIL__HEADING__BUILDER_HPP_
