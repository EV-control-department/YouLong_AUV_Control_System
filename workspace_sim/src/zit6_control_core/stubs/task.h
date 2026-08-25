#pragma once

// 宿主 FreeRTOS task.h 桩 — 仅包含 FreeRTOS.h(空临界区宏)。
// 控制核用不到 task 创建/调度 API。
#include "FreeRTOS.h"
