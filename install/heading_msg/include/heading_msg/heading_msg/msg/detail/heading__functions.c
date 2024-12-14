// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from heading_msg:msg/Heading.idl
// generated code does not contain a copyright notice
#include "heading_msg/msg/detail/heading__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
heading_msg__msg__Heading__init(heading_msg__msg__Heading * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  // heading
  return true;
}

void
heading_msg__msg__Heading__fini(heading_msg__msg__Heading * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  // heading
}

bool
heading_msg__msg__Heading__are_equal(const heading_msg__msg__Heading * lhs, const heading_msg__msg__Heading * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  return true;
}

bool
heading_msg__msg__Heading__copy(
  const heading_msg__msg__Heading * input,
  heading_msg__msg__Heading * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  output->robot_id = input->robot_id;
  // heading
  output->heading = input->heading;
  return true;
}

heading_msg__msg__Heading *
heading_msg__msg__Heading__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  heading_msg__msg__Heading * msg = (heading_msg__msg__Heading *)allocator.allocate(sizeof(heading_msg__msg__Heading), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(heading_msg__msg__Heading));
  bool success = heading_msg__msg__Heading__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
heading_msg__msg__Heading__destroy(heading_msg__msg__Heading * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    heading_msg__msg__Heading__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
heading_msg__msg__Heading__Sequence__init(heading_msg__msg__Heading__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  heading_msg__msg__Heading * data = NULL;

  if (size) {
    data = (heading_msg__msg__Heading *)allocator.zero_allocate(size, sizeof(heading_msg__msg__Heading), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = heading_msg__msg__Heading__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        heading_msg__msg__Heading__fini(&data[i - 1]);
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
heading_msg__msg__Heading__Sequence__fini(heading_msg__msg__Heading__Sequence * array)
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
      heading_msg__msg__Heading__fini(&array->data[i]);
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

heading_msg__msg__Heading__Sequence *
heading_msg__msg__Heading__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  heading_msg__msg__Heading__Sequence * array = (heading_msg__msg__Heading__Sequence *)allocator.allocate(sizeof(heading_msg__msg__Heading__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = heading_msg__msg__Heading__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
heading_msg__msg__Heading__Sequence__destroy(heading_msg__msg__Heading__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    heading_msg__msg__Heading__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
heading_msg__msg__Heading__Sequence__are_equal(const heading_msg__msg__Heading__Sequence * lhs, const heading_msg__msg__Heading__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!heading_msg__msg__Heading__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
heading_msg__msg__Heading__Sequence__copy(
  const heading_msg__msg__Heading__Sequence * input,
  heading_msg__msg__Heading__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(heading_msg__msg__Heading);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    heading_msg__msg__Heading * data =
      (heading_msg__msg__Heading *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!heading_msg__msg__Heading__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          heading_msg__msg__Heading__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!heading_msg__msg__Heading__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
