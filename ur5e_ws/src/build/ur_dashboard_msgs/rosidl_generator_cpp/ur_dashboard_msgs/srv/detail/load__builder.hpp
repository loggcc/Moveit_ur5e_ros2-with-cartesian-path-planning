// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ur_dashboard_msgs:srv/Load.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__SRV__DETAIL__LOAD__BUILDER_HPP_
#define UR_DASHBOARD_MSGS__SRV__DETAIL__LOAD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ur_dashboard_msgs/srv/detail/load__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ur_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_Load_Request_filename
{
public:
  Init_Load_Request_filename()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ur_dashboard_msgs::srv::Load_Request filename(::ur_dashboard_msgs::srv::Load_Request::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::Load_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::srv::Load_Request>()
{
  return ur_dashboard_msgs::srv::builder::Init_Load_Request_filename();
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_Load_Response_success
{
public:
  explicit Init_Load_Response_success(::ur_dashboard_msgs::srv::Load_Response & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::srv::Load_Response success(::ur_dashboard_msgs::srv::Load_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::Load_Response msg_;
};

class Init_Load_Response_answer
{
public:
  Init_Load_Response_answer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Load_Response_success answer(::ur_dashboard_msgs::srv::Load_Response::_answer_type arg)
  {
    msg_.answer = std::move(arg);
    return Init_Load_Response_success(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::Load_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::srv::Load_Response>()
{
  return ur_dashboard_msgs::srv::builder::Init_Load_Response_answer();
}

}  // namespace ur_dashboard_msgs

#endif  // UR_DASHBOARD_MSGS__SRV__DETAIL__LOAD__BUILDER_HPP_
