// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from ur_dashboard_msgs:action/SetMode.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ur_dashboard_msgs/action/detail/set_mode__struct.h"
#include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ur_dashboard_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _SetMode_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMode_Goal_type_support_ids_t;

static const _SetMode_Goal_type_support_ids_t _SetMode_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMode_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMode_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMode_Goal_type_support_symbol_names_t _SetMode_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ur_dashboard_msgs, action, SetMode_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_dashboard_msgs, action, SetMode_Goal)),
  }
};

typedef struct _SetMode_Goal_type_support_data_t
{
  void * data[2];
} _SetMode_Goal_type_support_data_t;

static _SetMode_Goal_type_support_data_t _SetMode_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMode_Goal_message_typesupport_map = {
  2,
  "ur_dashboard_msgs",
  &_SetMode_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_SetMode_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_SetMode_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMode_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMode_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ur_dashboard_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_Goal)() {
  return &::ur_dashboard_msgs::action::rosidl_typesupport_c::SetMode_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ur_dashboard_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _SetMode_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMode_Result_type_support_ids_t;

static const _SetMode_Result_type_support_ids_t _SetMode_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMode_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMode_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMode_Result_type_support_symbol_names_t _SetMode_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ur_dashboard_msgs, action, SetMode_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_dashboard_msgs, action, SetMode_Result)),
  }
};

typedef struct _SetMode_Result_type_support_data_t
{
  void * data[2];
} _SetMode_Result_type_support_data_t;

static _SetMode_Result_type_support_data_t _SetMode_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMode_Result_message_typesupport_map = {
  2,
  "ur_dashboard_msgs",
  &_SetMode_Result_message_typesupport_ids.typesupport_identifier[0],
  &_SetMode_Result_message_typesupport_symbol_names.symbol_name[0],
  &_SetMode_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMode_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMode_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ur_dashboard_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_Result)() {
  return &::ur_dashboard_msgs::action::rosidl_typesupport_c::SetMode_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ur_dashboard_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _SetMode_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMode_Feedback_type_support_ids_t;

static const _SetMode_Feedback_type_support_ids_t _SetMode_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMode_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMode_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMode_Feedback_type_support_symbol_names_t _SetMode_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ur_dashboard_msgs, action, SetMode_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_dashboard_msgs, action, SetMode_Feedback)),
  }
};

typedef struct _SetMode_Feedback_type_support_data_t
{
  void * data[2];
} _SetMode_Feedback_type_support_data_t;

static _SetMode_Feedback_type_support_data_t _SetMode_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMode_Feedback_message_typesupport_map = {
  2,
  "ur_dashboard_msgs",
  &_SetMode_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_SetMode_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_SetMode_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMode_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMode_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ur_dashboard_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_Feedback)() {
  return &::ur_dashboard_msgs::action::rosidl_typesupport_c::SetMode_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ur_dashboard_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _SetMode_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMode_SendGoal_Request_type_support_ids_t;

static const _SetMode_SendGoal_Request_type_support_ids_t _SetMode_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMode_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMode_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMode_SendGoal_Request_type_support_symbol_names_t _SetMode_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ur_dashboard_msgs, action, SetMode_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_dashboard_msgs, action, SetMode_SendGoal_Request)),
  }
};

typedef struct _SetMode_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _SetMode_SendGoal_Request_type_support_data_t;

static _SetMode_SendGoal_Request_type_support_data_t _SetMode_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMode_SendGoal_Request_message_typesupport_map = {
  2,
  "ur_dashboard_msgs",
  &_SetMode_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SetMode_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SetMode_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMode_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMode_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ur_dashboard_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_SendGoal_Request)() {
  return &::ur_dashboard_msgs::action::rosidl_typesupport_c::SetMode_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ur_dashboard_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _SetMode_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMode_SendGoal_Response_type_support_ids_t;

static const _SetMode_SendGoal_Response_type_support_ids_t _SetMode_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMode_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMode_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMode_SendGoal_Response_type_support_symbol_names_t _SetMode_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ur_dashboard_msgs, action, SetMode_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_dashboard_msgs, action, SetMode_SendGoal_Response)),
  }
};

typedef struct _SetMode_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _SetMode_SendGoal_Response_type_support_data_t;

static _SetMode_SendGoal_Response_type_support_data_t _SetMode_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMode_SendGoal_Response_message_typesupport_map = {
  2,
  "ur_dashboard_msgs",
  &_SetMode_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SetMode_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SetMode_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMode_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMode_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ur_dashboard_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_SendGoal_Response)() {
  return &::ur_dashboard_msgs::action::rosidl_typesupport_c::SetMode_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ur_dashboard_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _SetMode_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMode_SendGoal_type_support_ids_t;

static const _SetMode_SendGoal_type_support_ids_t _SetMode_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMode_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMode_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMode_SendGoal_type_support_symbol_names_t _SetMode_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ur_dashboard_msgs, action, SetMode_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_dashboard_msgs, action, SetMode_SendGoal)),
  }
};

typedef struct _SetMode_SendGoal_type_support_data_t
{
  void * data[2];
} _SetMode_SendGoal_type_support_data_t;

static _SetMode_SendGoal_type_support_data_t _SetMode_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMode_SendGoal_service_typesupport_map = {
  2,
  "ur_dashboard_msgs",
  &_SetMode_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_SetMode_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_SetMode_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SetMode_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMode_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ur_dashboard_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_SendGoal)() {
  return &::ur_dashboard_msgs::action::rosidl_typesupport_c::SetMode_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ur_dashboard_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _SetMode_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMode_GetResult_Request_type_support_ids_t;

static const _SetMode_GetResult_Request_type_support_ids_t _SetMode_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMode_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMode_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMode_GetResult_Request_type_support_symbol_names_t _SetMode_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ur_dashboard_msgs, action, SetMode_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_dashboard_msgs, action, SetMode_GetResult_Request)),
  }
};

typedef struct _SetMode_GetResult_Request_type_support_data_t
{
  void * data[2];
} _SetMode_GetResult_Request_type_support_data_t;

static _SetMode_GetResult_Request_type_support_data_t _SetMode_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMode_GetResult_Request_message_typesupport_map = {
  2,
  "ur_dashboard_msgs",
  &_SetMode_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SetMode_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SetMode_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMode_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMode_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ur_dashboard_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_GetResult_Request)() {
  return &::ur_dashboard_msgs::action::rosidl_typesupport_c::SetMode_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ur_dashboard_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _SetMode_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMode_GetResult_Response_type_support_ids_t;

static const _SetMode_GetResult_Response_type_support_ids_t _SetMode_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMode_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMode_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMode_GetResult_Response_type_support_symbol_names_t _SetMode_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ur_dashboard_msgs, action, SetMode_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_dashboard_msgs, action, SetMode_GetResult_Response)),
  }
};

typedef struct _SetMode_GetResult_Response_type_support_data_t
{
  void * data[2];
} _SetMode_GetResult_Response_type_support_data_t;

static _SetMode_GetResult_Response_type_support_data_t _SetMode_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMode_GetResult_Response_message_typesupport_map = {
  2,
  "ur_dashboard_msgs",
  &_SetMode_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SetMode_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SetMode_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMode_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMode_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ur_dashboard_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_GetResult_Response)() {
  return &::ur_dashboard_msgs::action::rosidl_typesupport_c::SetMode_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ur_dashboard_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _SetMode_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMode_GetResult_type_support_ids_t;

static const _SetMode_GetResult_type_support_ids_t _SetMode_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMode_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMode_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMode_GetResult_type_support_symbol_names_t _SetMode_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ur_dashboard_msgs, action, SetMode_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_dashboard_msgs, action, SetMode_GetResult)),
  }
};

typedef struct _SetMode_GetResult_type_support_data_t
{
  void * data[2];
} _SetMode_GetResult_type_support_data_t;

static _SetMode_GetResult_type_support_data_t _SetMode_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMode_GetResult_service_typesupport_map = {
  2,
  "ur_dashboard_msgs",
  &_SetMode_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_SetMode_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_SetMode_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SetMode_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMode_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ur_dashboard_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_GetResult)() {
  return &::ur_dashboard_msgs::action::rosidl_typesupport_c::SetMode_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__struct.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ur_dashboard_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _SetMode_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMode_FeedbackMessage_type_support_ids_t;

static const _SetMode_FeedbackMessage_type_support_ids_t _SetMode_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMode_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMode_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMode_FeedbackMessage_type_support_symbol_names_t _SetMode_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ur_dashboard_msgs, action, SetMode_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_dashboard_msgs, action, SetMode_FeedbackMessage)),
  }
};

typedef struct _SetMode_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _SetMode_FeedbackMessage_type_support_data_t;

static _SetMode_FeedbackMessage_type_support_data_t _SetMode_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMode_FeedbackMessage_message_typesupport_map = {
  2,
  "ur_dashboard_msgs",
  &_SetMode_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_SetMode_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_SetMode_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMode_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMode_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ur_dashboard_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_FeedbackMessage)() {
  return &::ur_dashboard_msgs::action::rosidl_typesupport_c::SetMode_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "ur_dashboard_msgs/action/set_mode.h"
// already included above
// #include "ur_dashboard_msgs/action/detail/set_mode__type_support.h"

static rosidl_action_type_support_t _ur_dashboard_msgs__action__SetMode__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode)()
{
  // Thread-safe by always writing the same values to the static struct
  _ur_dashboard_msgs__action__SetMode__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_SendGoal)();
  _ur_dashboard_msgs__action__SetMode__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_GetResult)();
  _ur_dashboard_msgs__action__SetMode__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _ur_dashboard_msgs__action__SetMode__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, ur_dashboard_msgs, action, SetMode_FeedbackMessage)();
  _ur_dashboard_msgs__action__SetMode__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_ur_dashboard_msgs__action__SetMode__typesupport_c;
}

#ifdef __cplusplus
}
#endif
