/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink USB 复合设备 (F1) —— 装配核心公共头
 * ============================================================================
 *
 *  本头是复合设备**接口号 / 端点地址的单一真相源**, 对应 ARCHITECTURE.md §6.3
 *  冻结的 USB1 (= 物理 USB0) 控制平面端点规划。 描述符 (usb_device.c) 与各功能
 *  模块 (usb_if_ctrl / usb_if_dap / usb_if_dapv1 / usb_if_uart) 共用这里的常量, 防止两处漂移。
 *
 *  复合设备 (bDeviceClass=0xEF/0x02/0x01, IAD): 7 接口 / 8 端点 (EP0~EP7 用满)
 *
 *    IF  EP(地址)              类型        类             功能            真逻辑
 *    --  ---------------------  ----------  -------------  --------------  ------
 *    0   EP1 OUT 0x01/IN 0x81   bulk        Vendor/WinUSB  命令通道(M1)    —
 *    1   EP2 OUT 0x02/IN 0x82   bulk        Vendor/WinUSB  CMSIS-DAP v2    F3
 *    2   EP3 OUT 0x03/IN 0x83   interrupt   HID            CMSIS-DAP v1    F3
 *    3   EP4 IN 0x84            int(notify) CDC1 Comm      UART1 透传      F4
 *    4   EP5 OUT 0x05/IN 0x85   bulk        CDC1 Data      (回环 echo)     F4
 *    5   EP6 IN 0x86            int(notify) CDC2 Comm      UART2 透传      F4
 *    6   EP7 OUT 0x07/IN 0x87   bulk        CDC2 Data      (回环 echo)     F4
 *
 *  F1 仅做到"枚举成功 + WinUSB 自动绑"; DAP/UART 真逻辑见 F3/F4。
 *
 *  ── 后续开发备注: SWO Stream 端点回收 (2026-06-03) ──────────────────────────
 *  注 (2026-06-09 HPM5361 迁移): HPM5361 有 **16 端点** (vs 6450/6200 的 8), 下述"端点用满/
 *  砍 v1 腾 EP3"的压力在 5361 上不存在 —— SWO streaming 可直接新增 bulk IN, 无需牺牲 v1。
 *  以下 8-端点方案分析保留作历史 / 8-端点平台参考。
 *  上表 EP0~EP7 在 **8-端点控制器** (HPM6450 / HPM6200, USB_SOC_DCD_MAX_ENDPOINT_COUNT=8)
 *  上已用满, 零空余。 若后续要做 CMSIS-DAP v2 的 **SWO streaming** (需在 IF1 再加 1 个
 *  bulk IN, 即 ARM 固件文档里 DAP 接口的"端点3"), 8-端点平台上的端点来源 =
 *  **砍掉已废弃的 CMSIS-DAP v1 (IF2 / EP3)**:
 *    · v1(HID) 已被 ARM 标 deprecated; 目标 host (pyOCD/probe-rs/OpenOCD>=0.11) 全走 v2。
 *    · 删 IF2 + EP3 → 腾出端点号 3 → 改作 SWO bulk IN (0x83 复用为 SWO IN)。
 *  注 1: 若 Link 主芯片改用 **16-端点控制器** (HPM5300 / HPM6800), 无端点压力,
 *        v1 与 SWO 可并存, 本回收非必需 (届时删掉本备注)。
 *  注 2: SWO 可先上 **polled 模式** (trace 走 EP2 响应端点, 0 新增端点);
 *        仅当 trace 带宽不够、要 streaming 时才需要这个第三 bulk IN 端点。
 *  现状: v1(IF2) 仍保留为枚举桩 (usb_if_dapv1.c), 本次不动代码, 仅记录回收方案。
 * ============================================================================
 */
#ifndef EXLINK_USB_DEVICE_H
#define EXLINK_USB_DEVICE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 端点地址 (§6.3 冻结) ---- */
#define EXLINK_EP_CMD_OUT    0x01u   /* IF0 命令通道 bulk OUT */
#define EXLINK_EP_CMD_IN     0x81u   /* IF0 命令通道 bulk IN  */
#define EXLINK_EP_DAP_OUT    0x02u   /* IF1 CMSIS-DAP v2 bulk OUT */
#define EXLINK_EP_DAP_IN     0x82u   /* IF1 CMSIS-DAP v2 bulk IN  */
#define EXLINK_EP_HID_OUT    0x03u   /* IF2 CMSIS-DAP v1 HID int OUT */
#define EXLINK_EP_HID_IN     0x83u   /* IF2 CMSIS-DAP v1 HID int IN  */
#define EXLINK_EP_CDC1_INT   0x84u   /* IF3 CDC1 notify (int IN) */
#define EXLINK_EP_CDC1_OUT   0x05u   /* IF4 CDC1 data bulk OUT */
#define EXLINK_EP_CDC1_IN    0x85u   /* IF4 CDC1 data bulk IN  */
#define EXLINK_EP_CDC2_INT   0x86u   /* IF5 CDC2 notify (int IN) */
#define EXLINK_EP_CDC2_OUT   0x07u   /* IF6 CDC2 data bulk OUT */
#define EXLINK_EP_CDC2_IN    0x87u   /* IF6 CDC2 data bulk IN  */

/* ---- 接口号 (与 config 描述符顺序 + usbd_add_interface 注册顺序严格一致) ---- */
#define EXLINK_IFNUM_CMD     0u
#define EXLINK_IFNUM_DAP     1u
#define EXLINK_IFNUM_HID     2u
#define EXLINK_IFNUM_CDC1    3u      /* comm; data = 4 */
#define EXLINK_IFNUM_CDC2    5u      /* comm; data = 6 */
#define EXLINK_IFNUM_TOTAL   7u

/* ---- HID (CMSIS-DAP v1) report descriptor 长度 (实体在 usb_if_dapv1.c) ----
 * config 描述符里的 HID class descriptor 用它填 wItemLength, 必须与实体一致。
 */
#define EXLINK_HID_REPORT_DESC_SIZE  33u

/* bulk 端点包长: HS 构建 512, FS 64 (随协商速度运行时再判 ZLP) */
#ifdef CONFIG_USB_HS
#define EXLINK_BULK_MPS      512u
#else
#define EXLINK_BULK_MPS      64u
#endif

/* HID / CDC-notify 中断端点包长 (CMSIS-DAP v1 用 64; CDC notify 由宏内定 8) */
#define EXLINK_HID_MPS       64u

/* 初始化并启动 USB0 复合设备:
 *   注册复合描述符 -> 按接口号顺序挂各功能模块的接口/端点 -> usbd_initialize。
 *   busid    : CherryUSB bus id (单设备用 0)
 *   reg_base : USB 控制器寄存器基址 (CONFIG_HPM_USBD_BASE)
 */
void exlink_usb_init(uint8_t busid, uint32_t reg_base);

#ifdef __cplusplus
}
#endif

#endif /* EXLINK_USB_DEVICE_H */
