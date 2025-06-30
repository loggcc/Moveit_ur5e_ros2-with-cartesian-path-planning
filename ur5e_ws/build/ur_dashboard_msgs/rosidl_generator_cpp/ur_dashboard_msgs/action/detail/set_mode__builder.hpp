// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ur_dashboard_msgs:action/SetMode.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__BUILDER_HPP_
#define UR_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ur_dashboard_msgs/action/detail/set_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ur_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_Goal_play_program
{
public:
  explicit Init_SetMode_Goal_play_program(::ur_dashboard_msgs::action::SetMode_Goal & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::action::SetMode_Goal play_program(::ur_dashboard_msgs::action::SetMode_Goal::_play_program_type arg)
  {
    msg_.play_program = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_Goal msg_;
};

class Init_SetMode_Goal_stop_program
{
public:
  explicit Init_SetMode_Goal_stop_program(::ur_dashboard_msgs::action::SetMode_Goal & msg)
  : msg_(msg)
  {}
  Init_SetMode_Goal_play_program stop_program(::ur_dashboard_msgs::action::SetMode_Goal::_stop_program_type arg)
  {
    msg_.stop_program = std::move(arg);
    return Init_SetMode_Goal_play_program(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_Goal msg_;
};

class Init_SetMode_Goal_target_robot_mode
{
public:
  Init_SetMode_Goal_target_robot_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Goal_stop_program target_robot_mode(::ur_dashboard_msgs::action::SetMode_Goal::_target_robot_mode_type arg)
  {
    msg_.target_robot_mode = std::move(arg);
    return Init_SetMode_Goal_stop_program(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::action::SetMode_Goal>()
{
  return ur_dashboard_msgs::action::builder::Init_SetMode_Goal_target_robot_mode();
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_Result_message
{
public:
  explicit Init_SetMode_Result_message(::ur_dashboard_msgs::action::SetMode_Result & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::action::SetMode_Result message(::ur_dashboard_msgs::action::SetMode_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_Result msg_;
};

class Init_SetMode_Result_success
{
public:
  Init_SetMode_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Result_message success(::ur_dashboard_msgs::action::SetMode_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetMode_Result_message(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::action::SetMode_Result>()
{
  return ur_dashboard_msgs::action::builder::Init_SetMode_Result_success();
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_Feedback_current_safety_mode
{
public:
  explicit Init_SetMode_Feedback_current_safety_mode(::ur_dashboard_msgs::action::SetMode_Feedback & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::action::SetMode_Feedback current_safety_mode(::ur_dashboard_msgs::action::SetMode_Feedback::_current_safety_mode_type arg)
  {
    msg_.current_safety_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_Feedback msg_;
};

class Init_SetMode_Feedback_current_robot_mode
{
public:
  Init_SetMode_Feedback_current_robot_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Feedback_current_safety_mode current_robot_mode(::ur_dashboard_msgs::action::SetMode_Feedback::_current_robot_mode_type arg)
  {
    msg_.current_robot_mode = std::move(arg);
    return Init_SetMode_Feedback_current_safety_mode(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::action::SetMode_Feedback>()
{
  return ur_dashboard_msgs::action::builder::Init_SetMode_Feedback_current_robot_mode();
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_SendGoal_Request_goal
{
public:
  explicit Init_SetMode_SendGoal_Request_goal(::ur_dashboard_msgs::action::SetMode_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::action::SetMode_SendGoal_Request goal(::ur_dashboard_msgs::action::SetMode_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_SendGoal_Request msg_;
};

class Init_SetMode_SendGoal_Request_goal_id
{
public:
  Init_SetMode_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_SendGoal_Request_goal goal_id(::ur_dashboard_msgs::action::SetMode_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SetMode_SendGoal_Request_goal(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::action::SetMode_SendGoal_Request>()
{
  return ur_dashboard_msgs::action::builder::Init_SetMode_SendGoal_Request_goal_id();
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_SendGoal_Response_stamp
{
public:
  explicit Init_SetMode_SendGoal_Response_stamp(::ur_dashboard_msgs::action::SetMode_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::action::SetMode_SendGoal_Response stamp(::ur_dashboard_msgs::action::SetMode_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_SendGoal_Response msg_;
};

class Init_SetMode_SendGoal_Response_accepted
{
public:
  Init_SetMode_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_SendGoal_Response_stamp accepted(::ur_dashboard_msgs::action::SetMode_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_SetMode_SendGoal_Response_stamp(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::action::SetMode_SendGoal_Response>()
{
  return ur_dashboard_msgs::action::builder::Init_SetMode_SendGoal_Response_accepted();
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_GetResult_Request_goal_id
{
public:
  Init_SetMode_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ur_dashboard_msgs::action::SetMode_GetResult_Request goal_id(::ur_dashboard_msgs::action::SetMode_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::action::SetMode_GetResult_Request>()
{
  return ur_dashboard_msgs::action::builder::Init_SetMode_GetResult_Request_goal_id();
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_GetResult_Response_result
{
public:
  explicit Init_SetMode_GetResult_Response_result(::ur_dashboard_msgs::action::SetMode_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::action::SetMode_GetResult_Response result(::ur_dashboard_msgs::action::SetMode_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_GetResult_Response msg_;
};

class Init_SetMode_GetResult_Response_status
{
public:
  Init_SetMode_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_GetResult_Response_result status(::ur_dashboard_msgs::action::SetMode_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_SetMode_GetResult_Response_result(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::action::SetMode_GetResult_Response>()
{
  return ur_dashboard_msgs::action::builder::Init_SetMode_GetResult_Response_status();
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_FeedbackMessage_feedback
{
public:
  explicit Init_SetMode_FeedbackMessage_feedback(::ur_dashboard_msgs::action::SetMode_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::action::SetMode_FeedbackMessage feedback(::ur_dashboard_msgs::action::SetMode_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_FeedbackMessage msg_;
};

class Init_SetMode_FeedbackMessage_goal_id
{
public:
  Init_SetMode_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_FeedbackMessage_feedback goal_id(::ur_dashboard_msgs::action::SetMode_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SetMode_FeedbackMessage_feedback(msg_);
  }

private:
  ::ur_dashboard_msgs::action::SetMode_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::action::SetMode_FeedbackMessage>()
{
  return ur_dashboard_msgs::action::builder::Init_SetMode_FeedbackMessage_goal_id();
}

}  // namespace ur_dashboard_msgs

#endif  // UR_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__BUILDER_HPP_
