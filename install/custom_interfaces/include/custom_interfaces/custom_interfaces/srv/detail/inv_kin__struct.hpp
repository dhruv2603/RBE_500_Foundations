// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:srv/InvKin.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__INV_KIN__STRUCT_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__INV_KIN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__InvKin_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__InvKin_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InvKin_Request_
{
  using Type = InvKin_Request_<ContainerAllocator>;

  explicit InvKin_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    (void)_init;
  }

  explicit InvKin_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::InvKin_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::InvKin_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::InvKin_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::InvKin_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::InvKin_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::InvKin_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::InvKin_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::InvKin_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::InvKin_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::InvKin_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__InvKin_Request
    std::shared_ptr<custom_interfaces::srv::InvKin_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__InvKin_Request
    std::shared_ptr<custom_interfaces::srv::InvKin_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InvKin_Request_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const InvKin_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InvKin_Request_

// alias to use template instance with default allocator
using InvKin_Request =
  custom_interfaces::srv::InvKin_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces


// Include directives for member types
// Member 'joint_vals'
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__InvKin_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__InvKin_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InvKin_Response_
{
  using Type = InvKin_Response_<ContainerAllocator>;

  explicit InvKin_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_vals(_init)
  {
    (void)_init;
  }

  explicit InvKin_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_vals(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _joint_vals_type =
    std_msgs::msg::Float32MultiArray_<ContainerAllocator>;
  _joint_vals_type joint_vals;

  // setters for named parameter idiom
  Type & set__joint_vals(
    const std_msgs::msg::Float32MultiArray_<ContainerAllocator> & _arg)
  {
    this->joint_vals = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::InvKin_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::InvKin_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::InvKin_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::InvKin_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::InvKin_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::InvKin_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::InvKin_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::InvKin_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::InvKin_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::InvKin_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__InvKin_Response
    std::shared_ptr<custom_interfaces::srv::InvKin_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__InvKin_Response
    std::shared_ptr<custom_interfaces::srv::InvKin_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InvKin_Response_ & other) const
  {
    if (this->joint_vals != other.joint_vals) {
      return false;
    }
    return true;
  }
  bool operator!=(const InvKin_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InvKin_Response_

// alias to use template instance with default allocator
using InvKin_Response =
  custom_interfaces::srv::InvKin_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace srv
{

struct InvKin
{
  using Request = custom_interfaces::srv::InvKin_Request;
  using Response = custom_interfaces::srv::InvKin_Response;
};

}  // namespace srv

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__INV_KIN__STRUCT_HPP_
