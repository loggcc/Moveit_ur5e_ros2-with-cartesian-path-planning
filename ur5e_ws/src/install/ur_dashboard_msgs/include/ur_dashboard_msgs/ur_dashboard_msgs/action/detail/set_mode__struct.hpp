// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ur_dashboard_msgs:action/SetMode.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__STRUCT_HPP_
#define UR_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_Goal __attribute__((deprecated))
#else
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_Goal __declspec(deprecated)
#endif

namespace ur_dashboard_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SetMode_Goal_
{
  using Type = SetMode_Goal_<ContainerAllocator>;

  explicit SetMode_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_robot_mode = 0;
      this->stop_program = false;
      this->play_program = false;
    }
  }

  explicit SetMode_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_robot_mode = 0;
      this->stop_program = false;
      this->play_program = false;
    }
  }

  // field types and members
  using _target_robot_mode_type =
    int8_t;
  _target_robot_mode_type target_robot_mode;
  using _stop_program_type =
    bool;
  _stop_program_type stop_program;
  using _play_program_type =
    bool;
  _play_program_type play_program;

  // setters for named parameter idiom
  Type & set__target_robot_mode(
    const int8_t & _arg)
  {
    this->target_robot_mode = _arg;
    return *this;
  }
  Type & set__stop_program(
    const bool & _arg)
  {
    this->stop_program = _arg;
    return *this;
  }
  Type & set__play_program(
    const bool & _arg)
  {
    this->play_program = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_Goal
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_Goal
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_Goal_ & other) const
  {
    if (this->target_robot_mode != other.target_robot_mode) {
      return false;
    }
    if (this->stop_program != other.stop_program) {
      return false;
    }
    if (this->play_program != other.play_program) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_Goal_

// alias to use template instance with default allocator
using SetMode_Goal =
  ur_dashboard_msgs::action::SetMode_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace ur_dashboard_msgs


#ifndef _WIN32
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_Result __attribute__((deprecated))
#else
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_Result __declspec(deprecated)
#endif

namespace ur_dashboard_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SetMode_Result_
{
  using Type = SetMode_Result_<ContainerAllocator>;

  explicit SetMode_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetMode_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_Result
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_Result
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_Result_

// alias to use template instance with default allocator
using SetMode_Result =
  ur_dashboard_msgs::action::SetMode_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace ur_dashboard_msgs


#ifndef _WIN32
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_Feedback __declspec(deprecated)
#endif

namespace ur_dashboard_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SetMode_Feedback_
{
  using Type = SetMode_Feedback_<ContainerAllocator>;

  explicit SetMode_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_robot_mode = 0;
      this->current_safety_mode = 0;
    }
  }

  explicit SetMode_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_robot_mode = 0;
      this->current_safety_mode = 0;
    }
  }

  // field types and members
  using _current_robot_mode_type =
    int8_t;
  _current_robot_mode_type current_robot_mode;
  using _current_safety_mode_type =
    int8_t;
  _current_safety_mode_type current_safety_mode;

  // setters for named parameter idiom
  Type & set__current_robot_mode(
    const int8_t & _arg)
  {
    this->current_robot_mode = _arg;
    return *this;
  }
  Type & set__current_safety_mode(
    const int8_t & _arg)
  {
    this->current_safety_mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_Feedback
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_Feedback
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_Feedback_ & other) const
  {
    if (this->current_robot_mode != other.current_robot_mode) {
      return false;
    }
    if (this->current_safety_mode != other.current_safety_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_Feedback_

// alias to use template instance with default allocator
using SetMode_Feedback =
  ur_dashboard_msgs::action::SetMode_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace ur_dashboard_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "ur_dashboard_msgs/action/detail/set_mode__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_SendGoal_Request __declspec(deprecated)
#endif

namespace ur_dashboard_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SetMode_SendGoal_Request_
{
  using Type = SetMode_SendGoal_Request_<ContainerAllocator>;

  explicit SetMode_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit SetMode_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const ur_dashboard_msgs::action::SetMode_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_SendGoal_Request
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_SendGoal_Request
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_SendGoal_Request_

// alias to use template instance with default allocator
using SetMode_SendGoal_Request =
  ur_dashboard_msgs::action::SetMode_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace ur_dashboard_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_SendGoal_Response __declspec(deprecated)
#endif

namespace ur_dashboard_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SetMode_SendGoal_Response_
{
  using Type = SetMode_SendGoal_Response_<ContainerAllocator>;

  explicit SetMode_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit SetMode_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_SendGoal_Response
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_SendGoal_Response
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_SendGoal_Response_

// alias to use template instance with default allocator
using SetMode_SendGoal_Response =
  ur_dashboard_msgs::action::SetMode_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace ur_dashboard_msgs

namespace ur_dashboard_msgs
{

namespace action
{

struct SetMode_SendGoal
{
  using Request = ur_dashboard_msgs::action::SetMode_SendGoal_Request;
  using Response = ur_dashboard_msgs::action::SetMode_SendGoal_Response;
};

}  // namespace action

}  // namespace ur_dashboard_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_GetResult_Request __declspec(deprecated)
#endif

namespace ur_dashboard_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SetMode_GetResult_Request_
{
  using Type = SetMode_GetResult_Request_<ContainerAllocator>;

  explicit SetMode_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit SetMode_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_GetResult_Request
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_GetResult_Request
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_GetResult_Request_

// alias to use template instance with default allocator
using SetMode_GetResult_Request =
  ur_dashboard_msgs::action::SetMode_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace ur_dashboard_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_GetResult_Response __declspec(deprecated)
#endif

namespace ur_dashboard_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SetMode_GetResult_Response_
{
  using Type = SetMode_GetResult_Response_<ContainerAllocator>;

  explicit SetMode_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit SetMode_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const ur_dashboard_msgs::action::SetMode_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_GetResult_Response
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_GetResult_Response
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_GetResult_Response_

// alias to use template instance with default allocator
using SetMode_GetResult_Response =
  ur_dashboard_msgs::action::SetMode_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace ur_dashboard_msgs

namespace ur_dashboard_msgs
{

namespace action
{

struct SetMode_GetResult
{
  using Request = ur_dashboard_msgs::action::SetMode_GetResult_Request;
  using Response = ur_dashboard_msgs::action::SetMode_GetResult_Response;
};

}  // namespace action

}  // namespace ur_dashboard_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__ur_dashboard_msgs__action__SetMode_FeedbackMessage __declspec(deprecated)
#endif

namespace ur_dashboard_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SetMode_FeedbackMessage_
{
  using Type = SetMode_FeedbackMessage_<ContainerAllocator>;

  explicit SetMode_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit SetMode_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const ur_dashboard_msgs::action::SetMode_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_FeedbackMessage
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_dashboard_msgs__action__SetMode_FeedbackMessage
    std::shared_ptr<ur_dashboard_msgs::action::SetMode_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_FeedbackMessage_

// alias to use template instance with default allocator
using SetMode_FeedbackMessage =
  ur_dashboard_msgs::action::SetMode_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace ur_dashboard_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace ur_dashboard_msgs
{

namespace action
{

struct SetMode
{
  /// The goal message defined in the action definition.
  using Goal = ur_dashboard_msgs::action::SetMode_Goal;
  /// The result message defined in the action definition.
  using Result = ur_dashboard_msgs::action::SetMode_Result;
  /// The feedback message defined in the action definition.
  using Feedback = ur_dashboard_msgs::action::SetMode_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = ur_dashboard_msgs::action::SetMode_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = ur_dashboard_msgs::action::SetMode_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = ur_dashboard_msgs::action::SetMode_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct SetMode SetMode;

}  // namespace action

}  // namespace ur_dashboard_msgs

#endif  // UR_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__STRUCT_HPP_
