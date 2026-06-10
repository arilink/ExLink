/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ExLink CMSIS-DAP 引擎线程 (F3a)。
 * 镜像 frame_dispatch 的线程模型: USB OUT 回调 (ISR) 把 DAP 请求包塞进环 + 放信号量;
 * 本线程消费, 调官方 DAP_ExecuteCommand() 处理, 响应经 EP2 IN 发回。
 * 绝不在 ISR/回调里跑 SWD 传输 (TransferBlock 可达毫秒级)。
 */
#ifndef EXLINK_DAP_ENGINE_H
#define EXLINK_DAP_ENGINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 创建并启动 DAP 引擎线程。 返回 0 成功, 负数失败。 (依赖 usb_if_dap 已 init) */
int dap_engine_thread_start(void);

/* 由 usb_if_dap 的 OUT 回调调用: 提交一个 DAP 请求包到引擎环。
 * data/len = 收到的 bulk OUT 内容。 满则丢弃并计数 (返回 -1)。 ISR 上下文安全。 */
int dap_engine_submit(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* EXLINK_DAP_ENGINE_H */
