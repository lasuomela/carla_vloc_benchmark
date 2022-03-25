// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_localization:srv/GetState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__GET_STATE__TRAITS_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__GET_STATE__TRAITS_HPP_

#include "robot_localization/srv/detail/get_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'time_stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::GetState_Request>()
{
  return "robot_localization::srv::GetState_Request";
}

template<>
inline const char * name<robot_localization::srv::GetState_Request>()
{
  return "robot_localization/srv/GetState_Request";
}

template<>
struct has_fixed_size<robot_localization::srv::GetState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_localization::srv::GetState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_localization::srv::GetState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::GetState_Response>()
{
  return "robot_localization::srv::GetState_Response";
}

template<>
inline const char * name<robot_localization::srv::GetState_Response>()
{
  return "robot_localization/srv/GetState_Response";
}

template<>
struct has_fixed_size<robot_localization::srv::GetState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_localization::srv::GetState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_localization::srv::GetState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::GetState>()
{
  return "robot_localization::srv::GetState";
}

template<>
inline const char * name<robot_localization::srv::GetState>()
{
  return "robot_localization/srv/GetState";
}

template<>
struct has_fixed_size<robot_localization::srv::GetState>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_localization::srv::GetState_Request>::value &&
    has_fixed_size<robot_localization::srv::GetState_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_localization::srv::GetState>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_localization::srv::GetState_Request>::value &&
    has_bounded_size<robot_localization::srv::GetState_Response>::value
  >
{
};

template<>
struct is_service<robot_localization::srv::GetState>
  : std::true_type
{
};

template<>
struct is_service_request<robot_localization::srv::GetState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_localization::srv::GetState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__GET_STATE__TRAITS_HPP_
