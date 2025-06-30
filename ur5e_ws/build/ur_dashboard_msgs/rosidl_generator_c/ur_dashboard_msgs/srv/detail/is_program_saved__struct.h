// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur_dashboard_msgs:srv/IsProgramSaved.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_SAVED__STRUCT_H_
#define UR_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_SAVED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/IsProgramSaved in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__srv__IsProgramSaved_Request
{
  uint8_t structure_needs_at_least_one_member;
} ur_dashboard_msgs__srv__IsProgramSaved_Request;

// Struct for a sequence of ur_dashboard_msgs__srv__IsProgramSaved_Request.
typedef struct ur_dashboard_msgs__srv__IsProgramSaved_Request__Sequence
{
  ur_dashboard_msgs__srv__IsProgramSaved_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__srv__IsProgramSaved_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'answer'
// Member 'program_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/IsProgramSaved in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__srv__IsProgramSaved_Response
{
  rosidl_runtime_c__String answer;
  rosidl_runtime_c__String program_name;
  /// is the current program saved?
  bool program_saved;
  /// Did the dashboard server call succeed?
  bool success;
} ur_dashboard_msgs__srv__IsProgramSaved_Response;

// Struct for a sequence of ur_dashboard_msgs__srv__IsProgramSaved_Response.
typedef struct ur_dashboard_msgs__srv__IsProgramSaved_Response__Sequence
{
  ur_dashboard_msgs__srv__IsProgramSaved_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__srv__IsProgramSaved_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_SAVED__STRUCT_H_
