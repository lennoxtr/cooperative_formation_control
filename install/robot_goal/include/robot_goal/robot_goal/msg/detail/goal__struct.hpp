// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_goal:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_GOAL__MSG__DETAIL__GOAL__STRUCT_HPP_
#define ROBOT_GOAL__MSG__DETAIL__GOAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_goal__msg__Goal __attribute__((deprecated))
#else
# define DEPRECATED__robot_goal__msg__Goal __declspec(deprecated)
#endif

namespace robot_goal
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Goal_
{
  using Type = Goal_<ContainerAllocator>;

  explicit Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->goal_x = 0.0f;
      this->goal_y = 0.0f;
    }
  }

  explicit Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->goal_x = 0.0f;
      this->goal_y = 0.0f;
    }
  }

  // field types and members
  using _goal_x_type =
    float;
  _goal_x_type goal_x;
  using _goal_y_type =
    float;
  _goal_y_type goal_y;

  // setters for named parameter idiom
  Type & set__goal_x(
    const float & _arg)
  {
    this->goal_x = _arg;
    return *this;
  }
  Type & set__goal_y(
    const float & _arg)
  {
    this->goal_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_goal::msg::Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_goal::msg::Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_goal::msg::Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_goal::msg::Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_goal::msg::Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_goal::msg::Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_goal::msg::Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_goal::msg::Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_goal::msg::Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_goal::msg::Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_goal__msg__Goal
    std::shared_ptr<robot_goal::msg::Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_goal__msg__Goal
    std::shared_ptr<robot_goal::msg::Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Goal_ & other) const
  {
    if (this->goal_x != other.goal_x) {
      return false;
    }
    if (this->goal_y != other.goal_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Goal_

// alias to use template instance with default allocator
using Goal =
  robot_goal::msg::Goal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_goal

#endif  // ROBOT_GOAL__MSG__DETAIL__GOAL__STRUCT_HPP_
