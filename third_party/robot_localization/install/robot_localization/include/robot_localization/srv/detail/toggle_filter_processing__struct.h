// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_localization:srv/ToggleFilterProcessing.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__STRUCT_H_
#define ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/ToggleFilterProcessing in the package robot_localization.
typedef struct robot_localization__srv__ToggleFilterProcessing_Request
{
  bool on;
} robot_localization__srv__ToggleFilterProcessing_Request;

// Struct for a sequence of robot_localization__srv__ToggleFilterProcessing_Request.
typedef struct robot_localization__srv__ToggleFilterProcessing_Request__Sequence
{
  robot_localization__srv__ToggleFilterProcessing_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__ToggleFilterProcessing_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/ToggleFilterProcessing in the package robot_localization.
typedef struct robot_localization__srv__ToggleFilterProcessing_Response
{
  bool status;
} robot_localization__srv__ToggleFilterProcessing_Response;

// Struct for a sequence of robot_localization__srv__ToggleFilterProcessing_Response.
typedef struct robot_localization__srv__ToggleFilterProcessing_Response__Sequence
{
  robot_localization__srv__ToggleFilterProcessing_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__ToggleFilterProcessing_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__STRUCT_H_
