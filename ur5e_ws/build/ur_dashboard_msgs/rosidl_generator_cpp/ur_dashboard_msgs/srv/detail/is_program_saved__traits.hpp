// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ur_dashboard_msgs:srv/IsProgramSaved.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_SAVED__TRAITS_HPP_
#define UR_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_SAVED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ur_dashboard_msgs/srv/detail/is_program_saved__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ur_dashboard_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsProgramSaved_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsProgramSaved_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IsProgramSaved_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ur_dashboard_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ur_dashboard_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ur_dashboard_msgs::srv::IsProgramSaved_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ur_dashboard_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ur_dashboard_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ur_dashboard_msgs::srv::IsProgramSaved_Request & msg)
{
  return ur_dashboard_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ur_dashboard_msgs::srv::IsProgramSaved_Request>()
{
  return "ur_dashboard_msgs::srv::IsProgramSaved_Request";
}

template<>
inline const char * name<ur_dashboard_msgs::srv::IsProgramSaved_Request>()
{
  return "ur_dashboard_msgs/srv/IsProgramSaved_Request";
}

template<>
struct has_fixed_size<ur_dashboard_msgs::srv::IsProgramSaved_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ur_dashboard_msgs::srv::IsProgramSaved_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ur_dashboard_msgs::srv::IsProgramSaved_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ur_dashboard_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsProgramSaved_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: answer
  {
    out << "answer: ";
    rosidl_generator_traits::value_to_yaml(msg.answer, out);
    out << ", ";
  }

  // member: program_name
  {
    out << "program_name: ";
    rosidl_generator_traits::value_to_yaml(msg.program_name, out);
    out << ", ";
  }

  // member: program_saved
  {
    out << "program_saved: ";
    rosidl_generator_traits::value_to_yaml(msg.program_saved, out);
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsProgramSaved_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: answer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "answer: ";
    rosidl_generator_traits::value_to_yaml(msg.answer, out);
    out << "\n";
  }

  // member: program_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "program_name: ";
    rosidl_generator_traits::value_to_yaml(msg.program_name, out);
    out << "\n";
  }

  // member: program_saved
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "program_saved: ";
    rosidl_generator_traits::value_to_yaml(msg.program_saved, out);
    out << "\n";
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IsProgramSaved_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ur_dashboard_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ur_dashboard_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ur_dashboard_msgs::srv::IsProgramSaved_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ur_dashboard_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ur_dashboard_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ur_dashboard_msgs::srv::IsProgramSaved_Response & msg)
{
  return ur_dashboard_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ur_dashboard_msgs::srv::IsProgramSaved_Response>()
{
  return "ur_dashboard_msgs::srv::IsProgramSaved_Response";
}

template<>
inline const char * name<ur_dashboard_msgs::srv::IsProgramSaved_Response>()
{
  return "ur_dashboard_msgs/srv/IsProgramSaved_Response";
}

template<>
struct has_fixed_size<ur_dashboard_msgs::srv::IsProgramSaved_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ur_dashboard_msgs::srv::IsProgramSaved_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ur_dashboard_msgs::srv::IsProgramSaved_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ur_dashboard_msgs::srv::IsProgramSaved>()
{
  return "ur_dashboard_msgs::srv::IsProgramSaved";
}

template<>
inline const char * name<ur_dashboard_msgs::srv::IsProgramSaved>()
{
  return "ur_dashboard_msgs/srv/IsProgramSaved";
}

template<>
struct has_fixed_size<ur_dashboard_msgs::srv::IsProgramSaved>
  : std::integral_constant<
    bool,
    has_fixed_size<ur_dashboard_msgs::srv::IsProgramSaved_Request>::value &&
    has_fixed_size<ur_dashboard_msgs::srv::IsProgramSaved_Response>::value
  >
{
};

template<>
struct has_bounded_size<ur_dashboard_msgs::srv::IsProgramSaved>
  : std::integral_constant<
    bool,
    has_bounded_size<ur_dashboard_msgs::srv::IsProgramSaved_Request>::value &&
    has_bounded_size<ur_dashboard_msgs::srv::IsProgramSaved_Response>::value
  >
{
};

template<>
struct is_service<ur_dashboard_msgs::srv::IsProgramSaved>
  : std::true_type
{
};

template<>
struct is_service_request<ur_dashboard_msgs::srv::IsProgramSaved_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ur_dashboard_msgs::srv::IsProgramSaved_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // UR_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_SAVED__TRAITS_HPP_
