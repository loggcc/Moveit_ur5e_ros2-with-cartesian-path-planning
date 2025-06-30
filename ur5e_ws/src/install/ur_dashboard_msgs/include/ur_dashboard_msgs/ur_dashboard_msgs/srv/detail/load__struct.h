// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur_dashboard_msgs:srv/Load.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__SRV__DETAIL__LOAD__STRUCT_H_
#define UR_DASHBOARD_MSGS__SRV__DETAIL__LOAD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'filename'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Load in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__srv__Load_Request
{
  rosidl_runtime_c__String filename;
} ur_dashboard_msgs__srv__Load_Request;

// Struct for a sequence of ur_dashboard_msgs__srv__Load_Request.
typedef struct ur_dashboard_msgs__srv__Load_Request__Sequence
{
  ur_dashboard_msgs__srv__Load_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__srv__Load_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'answer'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Load in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__srv__Load_Response
{
  rosidl_runtime_c__String answer;
  bool success;
} ur_dashboard_msgs__srv__Load_Response;

// Struct for a sequence of ur_dashboard_msgs__srv__Load_Response.
typedef struct ur_dashboard_msgs__srv__Load_Response__Sequence
{
  ur_dashboard_msgs__srv__Load_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__srv__Load_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR_DASHBOARD_MSGS__SRV__DETAIL__LOAD__STRUCT_H_
