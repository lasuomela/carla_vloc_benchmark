// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_localization:srv/ToLL.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__BUILDER_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__BUILDER_HPP_

#include "robot_localization/srv/detail/to_ll__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_ToLL_Request_map_point
{
public:
  Init_ToLL_Request_map_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::ToLL_Request map_point(::robot_localization::srv::ToLL_Request::_map_point_type arg)
  {
    msg_.map_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::ToLL_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::ToLL_Request>()
{
  return robot_localization::srv::builder::Init_ToLL_Request_map_point();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_ToLL_Response_ll_point
{
public:
  Init_ToLL_Response_ll_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::ToLL_Response ll_point(::robot_localization::srv::ToLL_Response::_ll_point_type arg)
  {
    msg_.ll_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::ToLL_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::ToLL_Response>()
{
  return robot_localization::srv::builder::Init_ToLL_Response_ll_point();
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__BUILDER_HPP_
