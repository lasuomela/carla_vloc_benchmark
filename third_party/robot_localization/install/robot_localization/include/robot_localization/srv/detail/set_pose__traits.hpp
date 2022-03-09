// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_localization:srv/SetPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__TRAITS_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__TRAITS_HPP_

#include "robot_localization/srv/detail/set_pose__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance_stamped__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::SetPose_Request>()
{
  return "robot_localization::srv::SetPose_Request";
}

template<>
inline const char * name<robot_localization::srv::SetPose_Request>()
{
  return "robot_localization/srv/SetPose_Request";
}

template<>
struct has_fixed_size<robot_localization::srv::SetPose_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseWithCovarianceStamped>::value> {};

template<>
struct has_bounded_size<robot_localization::srv::SetPose_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseWithCovarianceStamped>::value> {};

template<>
struct is_message<robot_localization::srv::SetPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::SetPose_Response>()
{
  return "robot_localization::srv::SetPose_Response";
}

template<>
inline const char * name<robot_localization::srv::SetPose_Response>()
{
  return "robot_localization/srv/SetPose_Response";
}

template<>
struct has_fixed_size<robot_localization::srv::SetPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_localization::srv::SetPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_localization::srv::SetPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::SetPose>()
{
  return "robot_localization::srv::SetPose";
}

template<>
inline const char * name<robot_localization::srv::SetPose>()
{
  return "robot_localization/srv/SetPose";
}

template<>
struct has_fixed_size<robot_localization::srv::SetPose>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_localization::srv::SetPose_Request>::value &&
    has_fixed_size<robot_localization::srv::SetPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_localization::srv::SetPose>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_localization::srv::SetPose_Request>::value &&
    has_bounded_size<robot_localization::srv::SetPose_Response>::value
  >
{
};

template<>
struct is_service<robot_localization::srv::SetPose>
  : std::true_type
{
};

template<>
struct is_service_request<robot_localization::srv::SetPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_localization::srv::SetPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__TRAITS_HPP_
