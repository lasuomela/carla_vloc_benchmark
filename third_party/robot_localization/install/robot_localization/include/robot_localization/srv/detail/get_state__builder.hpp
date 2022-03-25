// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_localization:srv/GetState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__GET_STATE__BUILDER_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__GET_STATE__BUILDER_HPP_

#include "robot_localization/srv/detail/get_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_GetState_Request_frame_id
{
public:
  explicit Init_GetState_Request_frame_id(::robot_localization::srv::GetState_Request & msg)
  : msg_(msg)
  {}
  ::robot_localization::srv::GetState_Request frame_id(::robot_localization::srv::GetState_Request::_frame_id_type arg)
  {
    msg_.frame_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::GetState_Request msg_;
};

class Init_GetState_Request_time_stamp
{
public:
  Init_GetState_Request_time_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetState_Request_frame_id time_stamp(::robot_localization::srv::GetState_Request::_time_stamp_type arg)
  {
    msg_.time_stamp = std::move(arg);
    return Init_GetState_Request_frame_id(msg_);
  }

private:
  ::robot_localization::srv::GetState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::GetState_Request>()
{
  return robot_localization::srv::builder::Init_GetState_Request_time_stamp();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_GetState_Response_covariance
{
public:
  explicit Init_GetState_Response_covariance(::robot_localization::srv::GetState_Response & msg)
  : msg_(msg)
  {}
  ::robot_localization::srv::GetState_Response covariance(::robot_localization::srv::GetState_Response::_covariance_type arg)
  {
    msg_.covariance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::GetState_Response msg_;
};

class Init_GetState_Response_state
{
public:
  Init_GetState_Response_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetState_Response_covariance state(::robot_localization::srv::GetState_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_GetState_Response_covariance(msg_);
  }

private:
  ::robot_localization::srv::GetState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::GetState_Response>()
{
  return robot_localization::srv::builder::Init_GetState_Response_state();
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__GET_STATE__BUILDER_HPP_
