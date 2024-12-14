// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from heading_msg:msg/Heading.idl
// generated code does not contain a copyright notice

#ifndef HEADING_MSG__MSG__DETAIL__HEADING__STRUCT_HPP_
#define HEADING_MSG__MSG__DETAIL__HEADING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__heading_msg__msg__Heading __attribute__((deprecated))
#else
# define DEPRECATED__heading_msg__msg__Heading __declspec(deprecated)
#endif

namespace heading_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Heading_
{
  using Type = Heading_<ContainerAllocator>;

  explicit Heading_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->heading = 0.0f;
    }
  }

  explicit Heading_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->heading = 0.0f;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _heading_type =
    float;
  _heading_type heading;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    heading_msg::msg::Heading_<ContainerAllocator> *;
  using ConstRawPtr =
    const heading_msg::msg::Heading_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<heading_msg::msg::Heading_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<heading_msg::msg::Heading_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      heading_msg::msg::Heading_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<heading_msg::msg::Heading_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      heading_msg::msg::Heading_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<heading_msg::msg::Heading_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<heading_msg::msg::Heading_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<heading_msg::msg::Heading_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__heading_msg__msg__Heading
    std::shared_ptr<heading_msg::msg::Heading_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__heading_msg__msg__Heading
    std::shared_ptr<heading_msg::msg::Heading_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Heading_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    return true;
  }
  bool operator!=(const Heading_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Heading_

// alias to use template instance with default allocator
using Heading =
  heading_msg::msg::Heading_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace heading_msg

#endif  // HEADING_MSG__MSG__DETAIL__HEADING__STRUCT_HPP_
