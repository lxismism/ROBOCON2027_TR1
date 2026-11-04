/**
 * @file control_task.h
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-29 2:03
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#ifdef __cplusplus
extern "C"{
#endif

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

void controlTask(void *argument);

#ifdef __cplusplus
}
#endif

