// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_goal:msg/Goal.idl
// generated code does not contain a copyright notice
#include "robot_goal/msg/detail/goal__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_goal__msg__Goal__init(robot_goal__msg__Goal * msg)
{
  if (!msg) {
    return false;
  }
  // goal_x
  // goal_y
  return true;
}

void
robot_goal__msg__Goal__fini(robot_goal__msg__Goal * msg)
{
  if (!msg) {
    return;
  }
  // goal_x
  // goal_y
}

bool
robot_goal__msg__Goal__are_equal(const robot_goal__msg__Goal * lhs, const robot_goal__msg__Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_x
  if (lhs->goal_x != rhs->goal_x) {
    return false;
  }
  // goal_y
  if (lhs->goal_y != rhs->goal_y) {
    return false;
  }
  return true;
}

bool
robot_goal__msg__Goal__copy(
  const robot_goal__msg__Goal * input,
  robot_goal__msg__Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_x
  output->goal_x = input->goal_x;
  // goal_y
  output->goal_y = input->goal_y;
  return true;
}

robot_goal__msg__Goal *
robot_goal__msg__Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_goal__msg__Goal * msg = (robot_goal__msg__Goal *)allocator.allocate(sizeof(robot_goal__msg__Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_goal__msg__Goal));
  bool success = robot_goal__msg__Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_goal__msg__Goal__destroy(robot_goal__msg__Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_goal__msg__Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_goal__msg__Goal__Sequence__init(robot_goal__msg__Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_goal__msg__Goal * data = NULL;

  if (size) {
    data = (robot_goal__msg__Goal *)allocator.zero_allocate(size, sizeof(robot_goal__msg__Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_goal__msg__Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_goal__msg__Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_goal__msg__Goal__Sequence__fini(robot_goal__msg__Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_goal__msg__Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_goal__msg__Goal__Sequence *
robot_goal__msg__Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_goal__msg__Goal__Sequence * array = (robot_goal__msg__Goal__Sequence *)allocator.allocate(sizeof(robot_goal__msg__Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_goal__msg__Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_goal__msg__Goal__Sequence__destroy(robot_goal__msg__Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_goal__msg__Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_goal__msg__Goal__Sequence__are_equal(const robot_goal__msg__Goal__Sequence * lhs, const robot_goal__msg__Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_goal__msg__Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_goal__msg__Goal__Sequence__copy(
  const robot_goal__msg__Goal__Sequence * input,
  robot_goal__msg__Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_goal__msg__Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_goal__msg__Goal * data =
      (robot_goal__msg__Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_goal__msg__Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_goal__msg__Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_goal__msg__Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
