/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ExLink 命令帧分发: op-code 路由 + 帧分发线程。
 * 线程从 usb_if_ctrl 的 RX 环取字节, 喂解析器, 完整帧路由到 handler, 响应经 usb_if_ctrl 回发。
 */
#ifndef EXLINK_FRAME_DISPATCH_H
#define EXLINK_FRAME_DISPATCH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 路由一个已校验的命令帧, 组好响应写入 resp 缓冲。
 *   op            : 帧 op-code
 *   payload/len   : 帧载荷 (可为 NULL/0)
 *   resp/resp_cap : 输出缓冲 (调用方提供, 至少 EXLINK_MAX_FRAME)
 * 返回响应帧总字节数 (含帧头+CRC); 返回 0 表示该 op 无响应 (静默)。
 *
 * 传输无关 —— 不直接碰 USB, 便于单元测试。
 */
size_t exlink_dispatch(uint8_t op, const uint8_t *payload, uint16_t len,
                       uint8_t *resp, size_t resp_cap);

/* 创建并启动帧分发线程 (依赖 usb_if_ctrl 已 init)。 返回 0 成功, 负数失败。 */
int exlink_frame_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* EXLINK_FRAME_DISPATCH_H */
