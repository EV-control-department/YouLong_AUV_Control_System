// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:action/BasicMotion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/action/basic_motion.h"


#ifndef UV_MSGS__ACTION__DETAIL__BASIC_MOTION__STRUCT_H_
#define UV_MSGS__ACTION__DETAIL__BASIC_MOTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'WMOVE'.
/**
  * 世界系步进, target=[dx, dy, dz, dyaw]
 */
enum
{
  uv_msgs__action__BasicMotion_Goal__WMOVE = 1
};

/// Constant 'BMOVE'.
/**
  * 机体系步进, target=[dx, dy, dz, dyaw]
 */
enum
{
  uv_msgs__action__BasicMotion_Goal__BMOVE = 2
};

/// Constant 'SET'.
/**
  * 绝对定位, target=[x, y, z, yaw]
 */
enum
{
  uv_msgs__action__BasicMotion_Goal__SET = 3
};

/// Constant 'WTRAVEL'.
/**
  * 世界系直线, target=[dx, dy, dz]
 */
enum
{
  uv_msgs__action__BasicMotion_Goal__WTRAVEL = 4
};

/// Constant 'BTRAVEL'.
/**
  * 机体系直线, target=[dx, dy, dz]
 */
enum
{
  uv_msgs__action__BasicMotion_Goal__BTRAVEL = 5
};

/// Constant 'START'.
/**
  * 初始化里程计原点（无视 axes/target/timeout）
 */
enum
{
  uv_msgs__action__BasicMotion_Goal__START = 6
};

// Include directives for member types
// Member 'axes'
#include "rosidl_runtime_c/string.h"
// Member 'target'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in action/BasicMotion in the package uv_msgs.
typedef struct uv_msgs__action__BasicMotion_Goal
{
  uint8_t cmd_type;
  /// 要动的轴: "x","y","z","rz" 任意组合, 空=全部
  rosidl_runtime_c__String axes;
  /// [x, y, z, yaw]
  rosidl_runtime_c__float__Sequence target;
  /// 秒, ≤0=默认60s
  float timeout;
} uv_msgs__action__BasicMotion_Goal;

// Struct for a sequence of uv_msgs__action__BasicMotion_Goal.
typedef struct uv_msgs__action__BasicMotion_Goal__Sequence
{
  uv_msgs__action__BasicMotion_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__action__BasicMotion_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/BasicMotion in the package uv_msgs.
typedef struct uv_msgs__action__BasicMotion_Result
{
  bool success;
  rosidl_runtime_c__String message;
} uv_msgs__action__BasicMotion_Result;

// Struct for a sequence of uv_msgs__action__BasicMotion_Result.
typedef struct uv_msgs__action__BasicMotion_Result__Sequence
{
  uv_msgs__action__BasicMotion_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__action__BasicMotion_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/BasicMotion in the package uv_msgs.
typedef struct uv_msgs__action__BasicMotion_Feedback
{
  float distance_remaining;
} uv_msgs__action__BasicMotion_Feedback;

// Struct for a sequence of uv_msgs__action__BasicMotion_Feedback.
typedef struct uv_msgs__action__BasicMotion_Feedback__Sequence
{
  uv_msgs__action__BasicMotion_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__action__BasicMotion_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "uv_msgs/action/detail/basic_motion__struct.h"

/// Struct defined in action/BasicMotion in the package uv_msgs.
typedef struct uv_msgs__action__BasicMotion_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  uv_msgs__action__BasicMotion_Goal goal;
} uv_msgs__action__BasicMotion_SendGoal_Request;

// Struct for a sequence of uv_msgs__action__BasicMotion_SendGoal_Request.
typedef struct uv_msgs__action__BasicMotion_SendGoal_Request__Sequence
{
  uv_msgs__action__BasicMotion_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__action__BasicMotion_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/BasicMotion in the package uv_msgs.
typedef struct uv_msgs__action__BasicMotion_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} uv_msgs__action__BasicMotion_SendGoal_Response;

// Struct for a sequence of uv_msgs__action__BasicMotion_SendGoal_Response.
typedef struct uv_msgs__action__BasicMotion_SendGoal_Response__Sequence
{
  uv_msgs__action__BasicMotion_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__action__BasicMotion_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  uv_msgs__action__BasicMotion_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  uv_msgs__action__BasicMotion_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/BasicMotion in the package uv_msgs.
typedef struct uv_msgs__action__BasicMotion_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  uv_msgs__action__BasicMotion_SendGoal_Request__Sequence request;
  uv_msgs__action__BasicMotion_SendGoal_Response__Sequence response;
} uv_msgs__action__BasicMotion_SendGoal_Event;

// Struct for a sequence of uv_msgs__action__BasicMotion_SendGoal_Event.
typedef struct uv_msgs__action__BasicMotion_SendGoal_Event__Sequence
{
  uv_msgs__action__BasicMotion_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__action__BasicMotion_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/BasicMotion in the package uv_msgs.
typedef struct uv_msgs__action__BasicMotion_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} uv_msgs__action__BasicMotion_GetResult_Request;

// Struct for a sequence of uv_msgs__action__BasicMotion_GetResult_Request.
typedef struct uv_msgs__action__BasicMotion_GetResult_Request__Sequence
{
  uv_msgs__action__BasicMotion_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__action__BasicMotion_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.h"

/// Struct defined in action/BasicMotion in the package uv_msgs.
typedef struct uv_msgs__action__BasicMotion_GetResult_Response
{
  int8_t status;
  uv_msgs__action__BasicMotion_Result result;
} uv_msgs__action__BasicMotion_GetResult_Response;

// Struct for a sequence of uv_msgs__action__BasicMotion_GetResult_Response.
typedef struct uv_msgs__action__BasicMotion_GetResult_Response__Sequence
{
  uv_msgs__action__BasicMotion_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__action__BasicMotion_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  uv_msgs__action__BasicMotion_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  uv_msgs__action__BasicMotion_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/BasicMotion in the package uv_msgs.
typedef struct uv_msgs__action__BasicMotion_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  uv_msgs__action__BasicMotion_GetResult_Request__Sequence request;
  uv_msgs__action__BasicMotion_GetResult_Response__Sequence response;
} uv_msgs__action__BasicMotion_GetResult_Event;

// Struct for a sequence of uv_msgs__action__BasicMotion_GetResult_Event.
typedef struct uv_msgs__action__BasicMotion_GetResult_Event__Sequence
{
  uv_msgs__action__BasicMotion_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__action__BasicMotion_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.h"

/// Struct defined in action/BasicMotion in the package uv_msgs.
typedef struct uv_msgs__action__BasicMotion_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  uv_msgs__action__BasicMotion_Feedback feedback;
} uv_msgs__action__BasicMotion_FeedbackMessage;

// Struct for a sequence of uv_msgs__action__BasicMotion_FeedbackMessage.
typedef struct uv_msgs__action__BasicMotion_FeedbackMessage__Sequence
{
  uv_msgs__action__BasicMotion_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__action__BasicMotion_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__ACTION__DETAIL__BASIC_MOTION__STRUCT_H_
