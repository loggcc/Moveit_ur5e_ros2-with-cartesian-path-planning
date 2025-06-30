// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur_dashboard_msgs:action/SetMode.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__STRUCT_H_
#define UR_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/SetMode in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__action__SetMode_Goal
{
  /// Target modes can be one of
  /// - 3: ROBOT_MODE_POWER_OFF
  /// - 5: ROBOT_MODE_IDLE
  /// - 7: ROBOT_MODE_RUNNING
  int8_t target_robot_mode;
  /// Stop program execution before restoring the target mode. Can be used together with 'play_program'.
  bool stop_program;
  /// Play the currently loaded program after target mode is reached.#
  /// NOTE: Requesting mode RUNNING in combination with this will make the robot continue the motion it
  /// was doing before. This might probably lead into the same problem (protective stop, EM-Stop due to
  /// faulty motion, etc.) If you want to be safe, set the 'stop_program' flag below and manually play
  /// the program after robot state is returned to normal.
  /// This flag will only be used when requesting mode RUNNING
  bool play_program;
} ur_dashboard_msgs__action__SetMode_Goal;

// Struct for a sequence of ur_dashboard_msgs__action__SetMode_Goal.
typedef struct ur_dashboard_msgs__action__SetMode_Goal__Sequence
{
  ur_dashboard_msgs__action__SetMode_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__action__SetMode_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/SetMode in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__action__SetMode_Result
{
  bool success;
  rosidl_runtime_c__String message;
} ur_dashboard_msgs__action__SetMode_Result;

// Struct for a sequence of ur_dashboard_msgs__action__SetMode_Result.
typedef struct ur_dashboard_msgs__action__SetMode_Result__Sequence
{
  ur_dashboard_msgs__action__SetMode_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__action__SetMode_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/SetMode in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__action__SetMode_Feedback
{
  int8_t current_robot_mode;
  int8_t current_safety_mode;
} ur_dashboard_msgs__action__SetMode_Feedback;

// Struct for a sequence of ur_dashboard_msgs__action__SetMode_Feedback.
typedef struct ur_dashboard_msgs__action__SetMode_Feedback__Sequence
{
  ur_dashboard_msgs__action__SetMode_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__action__SetMode_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "ur_dashboard_msgs/action/detail/set_mode__struct.h"

/// Struct defined in action/SetMode in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__action__SetMode_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  ur_dashboard_msgs__action__SetMode_Goal goal;
} ur_dashboard_msgs__action__SetMode_SendGoal_Request;

// Struct for a sequence of ur_dashboard_msgs__action__SetMode_SendGoal_Request.
typedef struct ur_dashboard_msgs__action__SetMode_SendGoal_Request__Sequence
{
  ur_dashboard_msgs__action__SetMode_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__action__SetMode_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/SetMode in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__action__SetMode_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} ur_dashboard_msgs__action__SetMode_SendGoal_Response;

// Struct for a sequence of ur_dashboard_msgs__action__SetMode_SendGoal_Response.
typedef struct ur_dashboard_msgs__action__SetMode_SendGoal_Response__Sequence
{
  ur_dashboard_msgs__action__SetMode_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__action__SetMode_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/SetMode in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__action__SetMode_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} ur_dashboard_msgs__action__SetMode_GetResult_Request;

// Struct for a sequence of ur_dashboard_msgs__action__SetMode_GetResult_Request.
typedef struct ur_dashboard_msgs__action__SetMode_GetResult_Request__Sequence
{
  ur_dashboard_msgs__action__SetMode_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__action__SetMode_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.h"

/// Struct defined in action/SetMode in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__action__SetMode_GetResult_Response
{
  int8_t status;
  ur_dashboard_msgs__action__SetMode_Result result;
} ur_dashboard_msgs__action__SetMode_GetResult_Response;

// Struct for a sequence of ur_dashboard_msgs__action__SetMode_GetResult_Response.
typedef struct ur_dashboard_msgs__action__SetMode_GetResult_Response__Sequence
{
  ur_dashboard_msgs__action__SetMode_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__action__SetMode_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.h"

/// Struct defined in action/SetMode in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__action__SetMode_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  ur_dashboard_msgs__action__SetMode_Feedback feedback;
} ur_dashboard_msgs__action__SetMode_FeedbackMessage;

// Struct for a sequence of ur_dashboard_msgs__action__SetMode_FeedbackMessage.
typedef struct ur_dashboard_msgs__action__SetMode_FeedbackMessage__Sequence
{
  ur_dashboard_msgs__action__SetMode_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__action__SetMode_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__STRUCT_H_
