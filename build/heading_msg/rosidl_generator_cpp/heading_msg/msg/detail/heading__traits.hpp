// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from heading_msg:msg/Heading.idl
// generated code does not contain a copyright notice

#ifndef HEADING_MSG__MSG__DETAIL__HEADING__TRAITS_HPP_
#define HEADING_MSG__MSG__DETAIL__HEADING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "heading_msg/msg/detail/heading__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace heading_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Heading & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Heading & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Heading & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace heading_msg

namespace rosidl_generator_traits
{

[[deprecated("use heading_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const heading_msg::msg::Heading & msg,
  std::ostream & out, size_t indentation = 0)
{
  heading_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use heading_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const heading_msg::msg::Heading & msg)
{
  return heading_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<heading_msg::msg::Heading>()
{
  return "heading_msg::msg::Heading";
}

template<>
inline const char * name<heading_msg::msg::Heading>()
{
  return "heading_msg/msg/Heading";
}

template<>
struct has_fixed_size<heading_msg::msg::Heading>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<heading_msg::msg::Heading>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<heading_msg::msg::Heading>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HEADING_MSG__MSG__DETAIL__HEADING__TRAITS_HPP_
