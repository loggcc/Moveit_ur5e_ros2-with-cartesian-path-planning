// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ur_dashboard_msgs:srv/GetProgramState.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__SRV__DETAIL__GET_PROGRAM_STATE__BUILDER_HPP_
#define UR_DASHBOARD_MSGS__SRV__DETAIL__GET_PROGRAM_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ur_dashboard_msgs/srv/detail/get_program_state__struct.hpp"
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
auto build<::ur_dashboard_msgs::srv::GetProgramState_Request>()
{
  return ::ur_dashboard_msgs::srv::GetProgramState_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_GetProgramState_Response_success
{
public:
  explicit Init_GetProgramState_Response_success(::ur_dashboard_msgs::srv::GetProgramState_Response & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::srv::GetProgramState_Response success(::ur_dashboard_msgs::srv::GetProgramState_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::GetProgramState_Response msg_;
};

class Init_GetProgramState_Response_answer
{
public:
  explicit Init_GetProgramState_Response_answer(::ur_dashboard_msgs::srv::GetProgramState_Response & msg)
  : msg_(msg)
  {}
  Init_GetProgramState_Response_success answer(::ur_dashboard_msgs::srv::GetProgramState_Response::_answer_type arg)
  {
    msg_.answer = std::move(arg);
    return Init_GetProgramState_Response_success(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::GetProgramState_Response msg_;
};

class Init_GetProgramState_Response_program_name
{
public:
  explicit Init_GetProgramState_Response_program_name(::ur_dashboard_msgs::srv::GetProgramState_Response & msg)
  : msg_(msg)
  {}
  Init_GetProgramState_Response_answer program_name(::ur_dashboard_msgs::srv::GetProgramState_Response::_program_name_type arg)
  {
    msg_.program_name = std::move(arg);
    return Init_GetProgramState_Response_answer(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::GetProgramState_Response msg_;
};

class Init_GetProgramState_Response_state
{
public:
  Init_GetProgramState_Response_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetProgramState_Response_program_name state(::ur_dashboard_msgs::srv::GetProgramState_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_GetProgramState_Response_program_name(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::GetProgramState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::srv::GetProgramState_Response>()
{
  return ur_dashboard_msgs::srv::builder::Init_GetProgramState_Response_state();
}

}  // namespace ur_dashboard_msgs

#endif  // UR_DASHBOARD_MSGS__SRV__DETAIL__GET_PROGRAM_STATE__BUILDER_HPP_
