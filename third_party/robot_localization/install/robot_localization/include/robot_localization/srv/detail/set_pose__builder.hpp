// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_localization:srv/SetPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__BUILDER_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__BUILDER_HPP_

#include "robot_localization/srv/detail/set_pose__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_SetPose_Request_pose
{
public:
  Init_SetPose_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::SetPose_Request pose(::robot_localization::srv::SetPose_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::SetPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::SetPose_Request>()
{
  return robot_localization::srv::builder::Init_SetPose_Request_pose();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::SetPose_Response>()
{
  return ::robot_localization::srv::SetPose_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__BUILDER_HPP_
