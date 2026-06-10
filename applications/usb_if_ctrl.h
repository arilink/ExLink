/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ExLink USB 控制通道接口 (复合设备 IF0, vendor bulk EP1)。
 * 命令帧 (MAGIC/OP/LEN/PAYLOAD/CRC16, PING/PONG 等) 的承载端点; 描述符在
 * usb_device.c, 本文件只管运行时。
 *
 * 命名: usb_if_<功能> 系列之一 (ctrl/dap/dapv1/uart/...), 与 spi/i2c/can 后续接口一致。
 * 线程模型: OUT 回调 (ISR) 只把字节塞进 RX 环 + 释放信号量; 帧分发线程消费。
 */
#ifndef EXLINK_USB_IF_CTRL_H
#define EXLINK_USB_IF_CTRL_H

#include <stdint.h>
#include <stddef.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 复合设备装配 (由 exlink_usb_init 调用, 在 usbd_initialize 之前):
 * 注册控制通道接口 (IF0) + EP1 in/out, 初始化 RX 环 / 信号量 / TX 状态。 */
void usb_if_ctrl_add(uint8_t busid);

/* USB 事件 hook (由 usb_device.c 的共享事件处理器调用)。 */
void usb_if_ctrl_on_configured(uint8_t busid);  /* 配置完成: 挂首个 OUT 读 */
void usb_if_ctrl_on_reset(void);                /* RESET/断开: 复位 TX/配置状态 */

/* 经控制通道 EP1 IN 发送一帧 (阻塞直至上一帧发送完, 含 ZLP 处理)。
 * 返回 0 成功, 负数失败 (未配置 / 超时)。 */
int usb_if_ctrl_send(const uint8_t *data, uint32_t len);

/* RX 环: 帧分发线程用。 take 信号量后排空环。
 *   - usb_if_ctrl_rx_sem      : OUT 回调收到数据时 release
 *   - usb_if_ctrl_rx_getchar  : 从环取 1 字节, 有则写 *out 返回 1, 空返回 0
 */
extern struct rt_semaphore usb_if_ctrl_rx_sem;
int usb_if_ctrl_rx_getchar(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* EXLINK_USB_IF_CTRL_H */
