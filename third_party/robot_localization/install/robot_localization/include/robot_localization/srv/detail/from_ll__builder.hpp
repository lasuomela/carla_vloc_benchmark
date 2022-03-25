// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_localization:srv/FromLL.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__FROM_LL__BUILDER_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__FROM_LL__BUILDER_HPP_

#include "robot_localization/srv/detail/from_ll__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_FromLL_Request_ll_point
{
public:
  Init_FromLL_Request_ll_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::FromLL_Request ll_point(::robot_localization::srv::FromLL_Request::_ll_point_type arg)
  {
    msg_.ll_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::FromLL_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::FromLL_Request>()
{
  return robot_localization::srv::builder::Init_FromLL_Request_ll_point();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_FromLL_Response_map_point
{
public:
  Init_FromLL_Response_map_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::FromLL_Response map_point(::robot_localization::srv::FromLL_Response::_map_point_type arg)
  {
    msg_.map_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::FromLL_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::FromLL_Response>()
{
  return robot_localization::srv::builder::Init_FromLL_Response_map_point();
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__FROM_LL__BUILDER_HPP_
