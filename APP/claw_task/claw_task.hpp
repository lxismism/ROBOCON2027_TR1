/**
 * @file claw_task.hpp
 * @brief 夹爪执行任务头文件（仅负责按指令控制 rise/rotate/grip 三个电机）
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

void clawTask(void *argument);

#ifdef __cplusplus
}
#endif

