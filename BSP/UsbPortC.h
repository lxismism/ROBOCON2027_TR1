/**
 * @file UsbPortC.h
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void UsbPort_onRxFromIsr(const uint8_t *data, uint32_t len);
void UsbPort_onTxCpltFromIsr(void);

#ifdef __cplusplus
}
#endif