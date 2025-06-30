// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur_dashboard_msgs:srv/Popup.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__SRV__DETAIL__POPUP__STRUCT_H_
#define UR_DASHBOARD_MSGS__SRV__DETAIL__POPUP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Popup in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__srv__Popup_Request
{
  rosidl_runtime_c__String message;
} ur_dashboard_msgs__srv__Popup_Request;

// Struct for a sequence of ur_dashboard_msgs__srv__Popup_Request.
typedef struct ur_dashboard_msgs__srv__Popup_Request__Sequence
{
  ur_dashboard_msgs__srv__Popup_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__srv__Popup_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'answer'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Popup in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__srv__Popup_Response
{
  rosidl_runtime_c__String answer;
  bool success;
} ur_dashboard_msgs__srv__Popup_Response;

// Struct for a sequence of ur_dashboard_msgs__srv__Popup_Response.
typedef struct ur_dashboard_msgs__srv__Popup_Response__Sequence
{
  ur_dashboard_msgs__srv__Popup_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__srv__Popup_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR_DASHBOARD_MSGS__SRV__DETAIL__POPUP__STRUCT_H_
