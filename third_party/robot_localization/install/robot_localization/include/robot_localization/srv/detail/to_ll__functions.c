// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_localization:srv/ToLL.idl
// generated code does not contain a copyright notice
#include "robot_localization/srv/detail/to_ll__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Include directives for member types
// Member `map_point`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
robot_localization__srv__ToLL_Request__init(robot_localization__srv__ToLL_Request * msg)
{
  if (!msg) {
    return false;
  }
  // map_point
  if (!geometry_msgs__msg__Point__init(&msg->map_point)) {
    robot_localization__srv__ToLL_Request__fini(msg);
    return false;
  }
  return true;
}

void
robot_localization__srv__ToLL_Request__fini(robot_localization__srv__ToLL_Request * msg)
{
  if (!msg) {
    return;
  }
  // map_point
  geometry_msgs__msg__Point__fini(&msg->map_point);
}

robot_localization__srv__ToLL_Request *
robot_localization__srv__ToLL_Request__create()
{
  robot_localization__srv__ToLL_Request * msg = (robot_localization__srv__ToLL_Request *)malloc(sizeof(robot_localization__srv__ToLL_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_localization__srv__ToLL_Request));
  bool success = robot_localization__srv__ToLL_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
robot_localization__srv__ToLL_Request__destroy(robot_localization__srv__ToLL_Request * msg)
{
  if (msg) {
    robot_localization__srv__ToLL_Request__fini(msg);
  }
  free(msg);
}


bool
robot_localization__srv__ToLL_Request__Sequence__init(robot_localization__srv__ToLL_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  robot_localization__srv__ToLL_Request * data = NULL;
  if (size) {
    data = (robot_localization__srv__ToLL_Request *)calloc(size, sizeof(robot_localization__srv__ToLL_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_localization__srv__ToLL_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_localization__srv__ToLL_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_localization__srv__ToLL_Request__Sequence__fini(robot_localization__srv__ToLL_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_localization__srv__ToLL_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_localization__srv__ToLL_Request__Sequence *
robot_localization__srv__ToLL_Request__Sequence__create(size_t size)
{
  robot_localization__srv__ToLL_Request__Sequence * array = (robot_localization__srv__ToLL_Request__Sequence *)malloc(sizeof(robot_localization__srv__ToLL_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = robot_localization__srv__ToLL_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
robot_localization__srv__ToLL_Request__Sequence__destroy(robot_localization__srv__ToLL_Request__Sequence * array)
{
  if (array) {
    robot_localization__srv__ToLL_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `ll_point`
#include "geographic_msgs/msg/detail/geo_point__functions.h"

bool
robot_localization__srv__ToLL_Response__init(robot_localization__srv__ToLL_Response * msg)
{
  if (!msg) {
    return false;
  }
  // ll_point
  if (!geographic_msgs__msg__GeoPoint__init(&msg->ll_point)) {
    robot_localization__srv__ToLL_Response__fini(msg);
    return false;
  }
  return true;
}

void
robot_localization__srv__ToLL_Response__fini(robot_localization__srv__ToLL_Response * msg)
{
  if (!msg) {
    return;
  }
  // ll_point
  geographic_msgs__msg__GeoPoint__fini(&msg->ll_point);
}

robot_localization__srv__ToLL_Response *
robot_localization__srv__ToLL_Response__create()
{
  robot_localization__srv__ToLL_Response * msg = (robot_localization__srv__ToLL_Response *)malloc(sizeof(robot_localization__srv__ToLL_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_localization__srv__ToLL_Response));
  bool success = robot_localization__srv__ToLL_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
robot_localization__srv__ToLL_Response__destroy(robot_localization__srv__ToLL_Response * msg)
{
  if (msg) {
    robot_localization__srv__ToLL_Response__fini(msg);
  }
  free(msg);
}


bool
robot_localization__srv__ToLL_Response__Sequence__init(robot_localization__srv__ToLL_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  robot_localization__srv__ToLL_Response * data = NULL;
  if (size) {
    data = (robot_localization__srv__ToLL_Response *)calloc(size, sizeof(robot_localization__srv__ToLL_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_localization__srv__ToLL_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_localization__srv__ToLL_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_localization__srv__ToLL_Response__Sequence__fini(robot_localization__srv__ToLL_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_localization__srv__ToLL_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_localization__srv__ToLL_Response__Sequence *
robot_localization__srv__ToLL_Response__Sequence__create(size_t size)
{
  robot_localization__srv__ToLL_Response__Sequence * array = (robot_localization__srv__ToLL_Response__Sequence *)malloc(sizeof(robot_localization__srv__ToLL_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = robot_localization__srv__ToLL_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
robot_localization__srv__ToLL_Response__Sequence__destroy(robot_localization__srv__ToLL_Response__Sequence * array)
{
  if (array) {
    robot_localization__srv__ToLL_Response__Sequence__fini(array);
  }
  free(array);
}
