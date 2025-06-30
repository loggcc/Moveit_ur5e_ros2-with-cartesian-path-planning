// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ur_dashboard_msgs:msg/SafetyMode.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__MSG__DETAIL__SAFETY_MODE__STRUCT_HPP_
#define UR_DASHBOARD_MSGS__MSG__DETAIL__SAFETY_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ur_dashboard_msgs__msg__SafetyMode __attribute__((deprecated))
#else
# define DEPRECATED__ur_dashboard_msgs__msg__SafetyMode __declspec(deprecated)
#endif

namespace ur_dashboard_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SafetyMode_
{
  using Type = SafetyMode_<ContainerAllocator>;

  explicit SafetyMode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
    }
  }

  explicit SafetyMode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NORMAL =
    1u;
  static constexpr uint8_t REDUCED =
    2u;
  static constexpr uint8_t PROTECTIVE_STOP =
    3u;
  static constexpr uint8_t RECOVERY =
    4u;
  static constexpr uint8_t SAFEGUARD_STOP =
    5u;
  static constexpr uint8_t SYSTEM_EMERGENCY_STOP =
    6u;
  static constexpr uint8_t ROBOT_EMERGENCY_STOP =
    7u;
  static constexpr uint8_t VIOLATION =
    8u;
  static constexpr uint8_t FAULT =
    9u;
  static constexpr uint8_t VALIDATE_JOINT_ID =
    10u;
  static constexpr uint8_t UNDEFINED_SAFETY_MODE =
    11u;
  static constexpr uint8_t AUTOMATIC_MODE_SAFEGUARD_STOP =
    12u;
  static constexpr uint8_t SYSTEM_THREE_POSITION_ENABLING_STOP =
    13u;

  // pointer types
  using RawPtr =
    ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_dashboard_msgs__msg__SafetyMode
    std::shared_ptr<ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_dashboard_msgs__msg__SafetyMode
    std::shared_ptr<ur_dashboard_msgs::msg::SafetyMode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SafetyMode_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const SafetyMode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SafetyMode_

// alias to use template instance with default allocator
using SafetyMode =
  ur_dashboard_msgs::msg::SafetyMode_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::NORMAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::REDUCED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::PROTECTIVE_STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::RECOVERY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::SAFEGUARD_STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::SYSTEM_EMERGENCY_STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::ROBOT_EMERGENCY_STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::VIOLATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::FAULT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::VALIDATE_JOINT_ID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::UNDEFINED_SAFETY_MODE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::AUTOMATIC_MODE_SAFEGUARD_STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SafetyMode_<ContainerAllocator>::SYSTEM_THREE_POSITION_ENABLING_STOP;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ur_dashboard_msgs

#endif  // UR_DASHBOARD_MSGS__MSG__DETAIL__SAFETY_MODE__STRUCT_HPP_
