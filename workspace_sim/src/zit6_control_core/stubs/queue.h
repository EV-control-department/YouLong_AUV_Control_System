#pragma once

// 宿主 FreeRTOS queue.h 桩 — QueueHandle_t 为 void*。控制核不直接使用队列,
// 只是 MotionContext 头文件里声明的字段类型。宿主驱动走 nav_state_/current_setpoint_
// 的 LockedField 注入,不走 SITL 队列。
using QueueHandle_t = void *;
