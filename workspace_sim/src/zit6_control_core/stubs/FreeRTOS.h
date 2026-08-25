#pragma once

// 宿主(Host) FreeRTOS 桩 — 供在本机编译/驱动固件原样控制核使用。
// 与固件自带 tests/lib/FreeRTOS.h 同款:临界区与任务 API 变成空操作。
// 单线程宿主无需中断保护;LockedField 因此退化为裸读写。

#include <cstdint>

using TickType_t = unsigned long;
using BaseType_t = long;
using UBaseType_t = unsigned long;
using TaskHandle_t = void *;

// 关键区 — 单线程宿主为空操作
#define taskENTER_CRITICAL()   do { } while (0)
#define taskEXIT_CRITICAL()    do { } while (0)

#define portTICK_PERIOD_MS 1
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

#define configMAX_PRIORITIES 56
#define configTICK_RATE_HZ   1000

inline TickType_t xTaskGetTickCount() { return 0; }
inline void vTaskDelayUntil(const TickType_t *, TickType_t) {}
inline void vTaskDelay(TickType_t) {}
