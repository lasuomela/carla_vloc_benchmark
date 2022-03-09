// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_localization:srv/GetState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__GET_STATE__STRUCT_H_
#define ROBOT_LOCALIZATION__SRV__DETAIL__GET_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'time_stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'frame_id'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/GetState in the package robot_localization.
typedef struct robot_localization__srv__GetState_Request
{
  builtin_interfaces__msg__Time time_stamp;
  rosidl_runtime_c__String frame_id;
} robot_localization__srv__GetState_Request;

// Struct for a sequence of robot_localization__srv__GetState_Request.
typedef struct robot_localization__srv__GetState_Request__Sequence
{
  robot_localization__srv__GetState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__GetState_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/GetState in the package robot_localization.
typedef struct robot_localization__srv__GetState_Response
{
  double state[15];
  double covariance[225];
} robot_localization__srv__GetState_Response;

// Struct for a sequence of robot_localization__srv__GetState_Response.
typedef struct robot_localization__srv__GetState_Response__Sequence
{
  robot_localization__srv__GetState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__GetState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__GET_STATE__STRUCT_H_
