/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ExLink USB CMSIS-DAP v1 接口 (复合设备 IF2, HID interrupt EP3)。
 * CMSIS-DAP v1 = HID 传输, 作老 OpenOCD (<0.11) 等不支持 v2 的 host 的**兜底**。
 * v2 (bulk, 首选) 是独立接口, 见 usb_if_dap.h。
 *
 * 命名: usb_if_<功能> 系列之一 (ctrl/dap/dapv1/uart/...)。
 * F1 只把接口"枚举出来 + 端点不挂死"(OUT 排空桩); **真正的 CMSIS-DAP v1 时序是
 * F3 的活**, 届时把真逻辑挂到 EP3 即可, 端点编号不变 (§6.3 冻结)。
 */
#ifndef EXLINK_USB_IF_DAPV1_H
#define EXLINK_USB_IF_DAPV1_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void usb_if_dapv1_add(uint8_t busid);            /* 注册 IF2 HID (+EP3 in/out) */
void usb_if_dapv1_on_configured(uint8_t busid);  /* 挂 HID OUT 读 (防 host 写阻塞) */
void usb_if_dapv1_on_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* EXLINK_USB_IF_DAPV1_H */
