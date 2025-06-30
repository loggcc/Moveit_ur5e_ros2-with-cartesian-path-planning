// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ur_dashboard_msgs:srv/IsProgramRunning.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_RUNNING__BUILDER_HPP_
#define UR_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_RUNNING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ur_dashboard_msgs/srv/detail/is_program_running__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ur_dashboard_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::srv::IsProgramRunning_Request>()
{
  return ::ur_dashboard_msgs::srv::IsProgramRunning_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_IsProgramRunning_Response_success
{
public:
  explicit Init_IsProgramRunning_Response_success(::ur_dashboard_msgs::srv::IsProgramRunning_Response & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::srv::IsProgramRunning_Response success(::ur_dashboard_msgs::srv::IsProgramRunning_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::IsProgramRunning_Response msg_;
};

class Init_IsProgramRunning_Response_program_running
{
public:
  explicit Init_IsProgramRunning_Response_program_running(::ur_dashboard_msgs::srv::IsProgramRunning_Response & msg)
  : msg_(msg)
  {}
  Init_IsProgramRunning_Response_success program_running(::ur_dashboard_msgs::srv::IsProgramRunning_Response::_program_running_type arg)
  {
    msg_.program_running = std::move(arg);
    return Init_IsProgramRunning_Response_success(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::IsProgramRunning_Response msg_;
};

class Init_IsProgramRunning_Response_answer
{
public:
  Init_IsProgramRunning_Response_answer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IsProgramRunning_Response_program_running answer(::ur_dashboard_msgs::srv::IsProgramRunning_Response::_answer_type arg)
  {
    msg_.answer = std::move(arg);
    return Init_IsProgramRunning_Response_program_running(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::IsProgramRunning_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::srv::IsProgramRunning_Response>()
{
  return ur_dashboard_msgs::srv::builder::Init_IsProgramRunning_Response_answer();
}

}  // namespace ur_dashboard_msgs

#endif  // UR_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_RUNNING__BUILDER_HPP_
