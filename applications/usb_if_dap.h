/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ExLink USB CMSIS-DAP v2 接口 (复合设备 IF1, vendor bulk EP2, WinUSB)。
 * CMSIS-DAP v2 = bulk 传输 (pyOCD / probe-rs / OpenOCD>=0.11 首选)。
 * v1 (HID 兜底) 是独立接口, 见 usb_if_dapv1.h。
 *
 * 命名: usb_if_<功能> 系列之一 (ctrl/dap/dapv1/uart/...)。
 * F1 只把接口"枚举出来 + 端点不挂死"(OUT 排空桩); **真正的 CMSIS-DAP v2 时序 /
 * SWD-JTAG / flash 算法是 F3 的活**, 届时把真逻辑挂到 EP2 即可, 端点编号不变 (§6.3 冻结)。
 */
#ifndef EXLINK_USB_IF_DAP_H
#define EXLINK_USB_IF_DAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void usb_if_dap_add(uint8_t busid);            /* 注册 IF1 (+EP2 in/out) + DAP/SPI init */
void usb_if_dap_on_configured(uint8_t busid);  /* 挂 DAP OUT 读 (防 host 写阻塞) */
void usb_if_dap_on_reset(void);

/* 阻塞发送一个 DAP 响应包经 EP2 IN (引擎线程调用, 含 ZLP 收尾)。0 成功, 负数失败。 */
int usb_if_dap_send(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* EXLINK_USB_IF_DAP_H */
