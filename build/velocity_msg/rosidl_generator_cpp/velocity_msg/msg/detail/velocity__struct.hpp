// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from velocity_msg:msg/Velocity.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_MSG__MSG__DETAIL__VELOCITY__STRUCT_HPP_
#define VELOCITY_MSG__MSG__DETAIL__VELOCITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__velocity_msg__msg__Velocity __attribute__((deprecated))
#else
# define DEPRECATED__velocity_msg__msg__Velocity __declspec(deprecated)
#endif

namespace velocity_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Velocity_
{
  using Type = Velocity_<ContainerAllocator>;

  explicit Velocity_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->linear_x = 0.0f;
      this->linear_y = 0.0f;
    }
  }

  explicit Velocity_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->linear_x = 0.0f;
      this->linear_y = 0.0f;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _linear_x_type =
    float;
  _linear_x_type linear_x;
  using _linear_y_type =
    float;
  _linear_y_type linear_y;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__linear_x(
    const float & _arg)
  {
    this->linear_x = _arg;
    return *this;
  }
  Type & set__linear_y(
    const float & _arg)
  {
    this->linear_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    velocity_msg::msg::Velocity_<ContainerAllocator> *;
  using ConstRawPtr =
    const velocity_msg::msg::Velocity_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<velocity_msg::msg::Velocity_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<velocity_msg::msg::Velocity_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      velocity_msg::msg::Velocity_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<velocity_msg::msg::Velocity_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      velocity_msg::msg::Velocity_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<velocity_msg::msg::Velocity_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<velocity_msg::msg::Velocity_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<velocity_msg::msg::Velocity_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__velocity_msg__msg__Velocity
    std::shared_ptr<velocity_msg::msg::Velocity_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__velocity_msg__msg__Velocity
    std::shared_ptr<velocity_msg::msg::Velocity_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Velocity_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->linear_x != other.linear_x) {
      return false;
    }
    if (this->linear_y != other.linear_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Velocity_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Velocity_

// alias to use template instance with default allocator
using Velocity =
  velocity_msg::msg::Velocity_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace velocity_msg

#endif  // VELOCITY_MSG__MSG__DETAIL__VELOCITY__STRUCT_HPP_
