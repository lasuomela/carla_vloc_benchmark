// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_localization:srv/ToggleFilterProcessing.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__BUILDER_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__BUILDER_HPP_

#include "robot_localization/srv/detail/toggle_filter_processing__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_ToggleFilterProcessing_Request_on
{
public:
  Init_ToggleFilterProcessing_Request_on()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::ToggleFilterProcessing_Request on(::robot_localization::srv::ToggleFilterProcessing_Request::_on_type arg)
  {
    msg_.on = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::ToggleFilterProcessing_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::ToggleFilterProcessing_Request>()
{
  return robot_localization::srv::builder::Init_ToggleFilterProcessing_Request_on();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_ToggleFilterProcessing_Response_status
{
public:
  Init_ToggleFilterProcessing_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::ToggleFilterProcessing_Response status(::robot_localization::srv::ToggleFilterProcessing_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::ToggleFilterProcessing_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::ToggleFilterProcessing_Response>()
{
  return robot_localization::srv::builder::Init_ToggleFilterProcessing_Response_status();
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__BUILDER_HPP_
