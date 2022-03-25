// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_localization:srv/SetDatum.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_DATUM__TRAITS_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_DATUM__TRAITS_HPP_

#include "robot_localization/srv/detail/set_datum__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'geo_pose'
#include "geographic_msgs/msg/detail/geo_pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::SetDatum_Request>()
{
  return "robot_localization::srv::SetDatum_Request";
}

template<>
inline const char * name<robot_localization::srv::SetDatum_Request>()
{
  return "robot_localization/srv/SetDatum_Request";
}

template<>
struct has_fixed_size<robot_localization::srv::SetDatum_Request>
  : std::integral_constant<bool, has_fixed_size<geographic_msgs::msg::GeoPose>::value> {};

template<>
struct has_bounded_size<robot_localization::srv::SetDatum_Request>
  : std::integral_constant<bool, has_bounded_size<geographic_msgs::msg::GeoPose>::value> {};

template<>
struct is_message<robot_localization::srv::SetDatum_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::SetDatum_Response>()
{
  return "robot_localization::srv::SetDatum_Response";
}

template<>
inline const char * name<robot_localization::srv::SetDatum_Response>()
{
  return "robot_localization/srv/SetDatum_Response";
}

template<>
struct has_fixed_size<robot_localization::srv::SetDatum_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_localization::srv::SetDatum_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_localization::srv::SetDatum_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::SetDatum>()
{
  return "robot_localization::srv::SetDatum";
}

template<>
inline const char * name<robot_localization::srv::SetDatum>()
{
  return "robot_localization/srv/SetDatum";
}

template<>
struct has_fixed_size<robot_localization::srv::SetDatum>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_localization::srv::SetDatum_Request>::value &&
    has_fixed_size<robot_localization::srv::SetDatum_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_localization::srv::SetDatum>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_localization::srv::SetDatum_Request>::value &&
    has_bounded_size<robot_localization::srv::SetDatum_Response>::value
  >
{
};

template<>
struct is_service<robot_localization::srv::SetDatum>
  : std::true_type
{
};

template<>
struct is_service_request<robot_localization::srv::SetDatum_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_localization::srv::SetDatum_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_DATUM__TRAITS_HPP_
