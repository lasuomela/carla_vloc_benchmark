// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_localization:srv/SetPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__STRUCT_H_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.h"

// Struct defined in srv/SetPose in the package robot_localization.
typedef struct robot_localization__srv__SetPose_Request
{
  geometry_msgs__msg__PoseWithCovarianceStamped pose;
} robot_localization__srv__SetPose_Request;

// Struct for a sequence of robot_localization__srv__SetPose_Request.
typedef struct robot_localization__srv__SetPose_Request__Sequence
{
  robot_localization__srv__SetPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__SetPose_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/SetPose in the package robot_localization.
typedef struct robot_localization__srv__SetPose_Response
{
  uint8_t structure_needs_at_least_one_member;
} robot_localization__srv__SetPose_Response;

// Struct for a sequence of robot_localization__srv__SetPose_Response.
typedef struct robot_localization__srv__SetPose_Response__Sequence
{
  robot_localization__srv__SetPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__SetPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__STRUCT_H_
