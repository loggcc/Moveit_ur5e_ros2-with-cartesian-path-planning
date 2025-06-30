// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur_dashboard_msgs:msg/ProgramState.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__MSG__DETAIL__PROGRAM_STATE__STRUCT_H_
#define UR_DASHBOARD_MSGS__MSG__DETAIL__PROGRAM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STOPPED'.
static const char * const ur_dashboard_msgs__msg__ProgramState__STOPPED = "STOPPED";

/// Constant 'PLAYING'.
static const char * const ur_dashboard_msgs__msg__ProgramState__PLAYING = "PLAYING";

/// Constant 'PAUSED'.
static const char * const ur_dashboard_msgs__msg__ProgramState__PAUSED = "PAUSED";

// Include directives for member types
// Member 'state'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ProgramState in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__msg__ProgramState
{
  rosidl_runtime_c__String state;
} ur_dashboard_msgs__msg__ProgramState;

// Struct for a sequence of ur_dashboard_msgs__msg__ProgramState.
typedef struct ur_dashboard_msgs__msg__ProgramState__Sequence
{
  ur_dashboard_msgs__msg__ProgramState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__msg__ProgramState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR_DASHBOARD_MSGS__MSG__DETAIL__PROGRAM_STATE__STRUCT_H_
