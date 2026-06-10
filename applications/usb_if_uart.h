/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ExLink USB UART 接口 —— 2× CDC-ACM (复合设备 IF3..IF6), F1 回环 echo。
 *   CDC1: IF3 comm (EP4 notify) + IF4 data (EP5 bulk)  -> 主机看到 COM/ttyACM #1
 *   CDC2: IF5 comm (EP6 notify) + IF6 data (EP7 bulk)  -> 主机看到 COM/ttyACM #2
 *
 * 命名: usb_if_<功能> 系列之一 (ctrl/dap/dapv1/uart/...)。
 * F1 只做**回环**: 主机往串口写什么, 原样回读 (验证 CDC 数据通路 + 枚举)。
 * **真正接到片上 UART (DMA + ring buffer) 的桥接是工作包 F4**, 届时把 echo 换成
 * UART TX/RX 即可, 接口/端点不变 (§6.3 冻结)。
 */
#ifndef EXLINK_USB_IF_UART_H
#define EXLINK_USB_IF_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void usb_if_uart_add(uint8_t busid);            /* 注册 2x CDC (IF3..IF6) + 数据端点 */
void usb_if_uart_on_configured(uint8_t busid);  /* 挂两路 OUT 读 */
void usb_if_uart_on_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* EXLINK_USB_IF_UART_H */
