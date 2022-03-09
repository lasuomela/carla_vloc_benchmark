// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_localization:srv/ToLL.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__TRAITS_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__TRAITS_HPP_

#include "robot_localization/srv/detail/to_ll__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'map_point'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::ToLL_Request>()
{
  return "robot_localization::srv::ToLL_Request";
}

template<>
inline const char * name<robot_localization::srv::ToLL_Request>()
{
  return "robot_localization/srv/ToLL_Request";
}

template<>
struct has_fixed_size<robot_localization::srv::ToLL_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<robot_localization::srv::ToLL_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<robot_localization::srv::ToLL_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'll_point'
#include "geographic_msgs/msg/detail/geo_point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::ToLL_Response>()
{
  return "robot_localization::srv::ToLL_Response";
}

template<>
inline const char * name<robot_localization::srv::ToLL_Response>()
{
  return "robot_localization/srv/ToLL_Response";
}

template<>
struct has_fixed_size<robot_localization::srv::ToLL_Response>
  : std::integral_constant<bool, has_fixed_size<geographic_msgs::msg::GeoPoint>::value> {};

template<>
struct has_bounded_size<robot_localization::srv::ToLL_Response>
  : std::integral_constant<bool, has_bounded_size<geographic_msgs::msg::GeoPoint>::value> {};

template<>
struct is_message<robot_localization::srv::ToLL_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::ToLL>()
{
  return "robot_localization::srv::ToLL";
}

template<>
inline const char * name<robot_localization::srv::ToLL>()
{
  return "robot_localization/srv/ToLL";
}

template<>
struct has_fixed_size<robot_localization::srv::ToLL>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_localization::srv::ToLL_Request>::value &&
    has_fixed_size<robot_localization::srv::ToLL_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_localization::srv::ToLL>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_localization::srv::ToLL_Request>::value &&
    has_bounded_size<robot_localization::srv::ToLL_Response>::value
  >
{
};

template<>
struct is_service<robot_localization::srv::ToLL>
  : std::true_type
{
};

template<>
struct is_service_request<robot_localization::srv::ToLL_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_localization::srv::ToLL_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__TRAITS_HPP_
