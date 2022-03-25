// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_localization:srv/ToggleFilterProcessing.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__STRUCT_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__robot_localization__srv__ToggleFilterProcessing_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_localization__srv__ToggleFilterProcessing_Request __declspec(deprecated)
#endif

namespace robot_localization
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ToggleFilterProcessing_Request_
{
  using Type = ToggleFilterProcessing_Request_<ContainerAllocator>;

  explicit ToggleFilterProcessing_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->on = false;
    }
  }

  explicit ToggleFilterProcessing_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->on = false;
    }
  }

  // field types and members
  using _on_type =
    bool;
  _on_type on;

  // setters for named parameter idiom
  Type & set__on(
    const bool & _arg)
  {
    this->on = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_localization__srv__ToggleFilterProcessing_Request
    std::shared_ptr<robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_localization__srv__ToggleFilterProcessing_Request
    std::shared_ptr<robot_localization::srv::ToggleFilterProcessing_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToggleFilterProcessing_Request_ & other) const
  {
    if (this->on != other.on) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToggleFilterProcessing_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToggleFilterProcessing_Request_

// alias to use template instance with default allocator
using ToggleFilterProcessing_Request =
  robot_localization::srv::ToggleFilterProcessing_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_localization


#ifndef _WIN32
# define DEPRECATED__robot_localization__srv__ToggleFilterProcessing_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_localization__srv__ToggleFilterProcessing_Response __declspec(deprecated)
#endif

namespace robot_localization
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ToggleFilterProcessing_Response_
{
  using Type = ToggleFilterProcessing_Response_<ContainerAllocator>;

  explicit ToggleFilterProcessing_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
    }
  }

  explicit ToggleFilterProcessing_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
    }
  }

  // field types and members
  using _status_type =
    bool;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const bool & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_localization__srv__ToggleFilterProcessing_Response
    std::shared_ptr<robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_localization__srv__ToggleFilterProcessing_Response
    std::shared_ptr<robot_localization::srv::ToggleFilterProcessing_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToggleFilterProcessing_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToggleFilterProcessing_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToggleFilterProcessing_Response_

// alias to use template instance with default allocator
using ToggleFilterProcessing_Response =
  robot_localization::srv::ToggleFilterProcessing_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_localization

namespace robot_localization
{

namespace srv
{

struct ToggleFilterProcessing
{
  using Request = robot_localization::srv::ToggleFilterProcessing_Request;
  using Response = robot_localization::srv::ToggleFilterProcessing_Response;
};

}  // namespace srv

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__STRUCT_HPP_
