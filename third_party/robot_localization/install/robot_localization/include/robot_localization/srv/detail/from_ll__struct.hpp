// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_localization:srv/FromLL.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__FROM_LL__STRUCT_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__FROM_LL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'll_point'
#include "geographic_msgs/msg/detail/geo_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_localization__srv__FromLL_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_localization__srv__FromLL_Request __declspec(deprecated)
#endif

namespace robot_localization
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FromLL_Request_
{
  using Type = FromLL_Request_<ContainerAllocator>;

  explicit FromLL_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ll_point(_init)
  {
    (void)_init;
  }

  explicit FromLL_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ll_point(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _ll_point_type =
    geographic_msgs::msg::GeoPoint_<ContainerAllocator>;
  _ll_point_type ll_point;

  // setters for named parameter idiom
  Type & set__ll_point(
    const geographic_msgs::msg::GeoPoint_<ContainerAllocator> & _arg)
  {
    this->ll_point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_localization::srv::FromLL_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_localization::srv::FromLL_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_localization::srv::FromLL_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_localization::srv::FromLL_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::FromLL_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::FromLL_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::FromLL_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::FromLL_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_localization::srv::FromLL_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_localization::srv::FromLL_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_localization__srv__FromLL_Request
    std::shared_ptr<robot_localization::srv::FromLL_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_localization__srv__FromLL_Request
    std::shared_ptr<robot_localization::srv::FromLL_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FromLL_Request_ & other) const
  {
    if (this->ll_point != other.ll_point) {
      return false;
    }
    return true;
  }
  bool operator!=(const FromLL_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FromLL_Request_

// alias to use template instance with default allocator
using FromLL_Request =
  robot_localization::srv::FromLL_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_localization


// Include directives for member types
// Member 'map_point'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_localization__srv__FromLL_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_localization__srv__FromLL_Response __declspec(deprecated)
#endif

namespace robot_localization
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FromLL_Response_
{
  using Type = FromLL_Response_<ContainerAllocator>;

  explicit FromLL_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_point(_init)
  {
    (void)_init;
  }

  explicit FromLL_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_point(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _map_point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _map_point_type map_point;

  // setters for named parameter idiom
  Type & set__map_point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->map_point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_localization::srv::FromLL_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_localization::srv::FromLL_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_localization::srv::FromLL_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_localization::srv::FromLL_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::FromLL_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::FromLL_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::FromLL_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::FromLL_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_localization::srv::FromLL_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_localization::srv::FromLL_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_localization__srv__FromLL_Response
    std::shared_ptr<robot_localization::srv::FromLL_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_localization__srv__FromLL_Response
    std::shared_ptr<robot_localization::srv::FromLL_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FromLL_Response_ & other) const
  {
    if (this->map_point != other.map_point) {
      return false;
    }
    return true;
  }
  bool operator!=(const FromLL_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FromLL_Response_

// alias to use template instance with default allocator
using FromLL_Response =
  robot_localization::srv::FromLL_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_localization

namespace robot_localization
{

namespace srv
{

struct FromLL
{
  using Request = robot_localization::srv::FromLL_Request;
  using Response = robot_localization::srv::FromLL_Response;
};

}  // namespace srv

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__FROM_LL__STRUCT_HPP_
