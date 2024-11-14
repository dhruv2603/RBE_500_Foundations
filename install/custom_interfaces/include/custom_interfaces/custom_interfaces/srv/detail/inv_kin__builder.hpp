// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/InvKin.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__INV_KIN__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__INV_KIN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/inv_kin__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_InvKin_Request_pose
{
public:
  Init_InvKin_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::InvKin_Request pose(::custom_interfaces::srv::InvKin_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::InvKin_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::InvKin_Request>()
{
  return custom_interfaces::srv::builder::Init_InvKin_Request_pose();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_InvKin_Response_joint_vals
{
public:
  Init_InvKin_Response_joint_vals()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::InvKin_Response joint_vals(::custom_interfaces::srv::InvKin_Response::_joint_vals_type arg)
  {
    msg_.joint_vals = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::InvKin_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::InvKin_Response>()
{
  return custom_interfaces::srv::builder::Init_InvKin_Response_joint_vals();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__INV_KIN__BUILDER_HPP_
