// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_localization:srv/SetDatum.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_DATUM__STRUCT_H_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_DATUM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'geo_pose'
#include "geographic_msgs/msg/detail/geo_pose__struct.h"

// Struct defined in srv/SetDatum in the package robot_localization.
typedef struct robot_localization__srv__SetDatum_Request
{
  geographic_msgs__msg__GeoPose geo_pose;
} robot_localization__srv__SetDatum_Request;

// Struct for a sequence of robot_localization__srv__SetDatum_Request.
typedef struct robot_localization__srv__SetDatum_Request__Sequence
{
  robot_localization__srv__SetDatum_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__SetDatum_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/SetDatum in the package robot_localization.
typedef struct robot_localization__srv__SetDatum_Response
{
  uint8_t structure_needs_at_least_one_member;
} robot_localization__srv__SetDatum_Response;

// Struct for a sequence of robot_localization__srv__SetDatum_Response.
typedef struct robot_localization__srv__SetDatum_Response__Sequence
{
  robot_localization__srv__SetDatum_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__SetDatum_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_DATUM__STRUCT_H_
