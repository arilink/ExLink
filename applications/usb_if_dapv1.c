/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink USB CMSIS-DAP v1 接口 —— 复合设备 IF2 (HID interrupt, EP3)
 * ============================================================================
 *
 *  CMSIS-DAP **v1** (HID 传输), 作老 OpenOCD (<0.11) 等不支持 v2 的 host 的兜底。
 *  Windows 走内建 HID 类驱动 (无需 WinUSB)。 v2 (bulk, 首选) 见 usb_if_dap.c。
 *
 *  F1 范围: 仅枚举 + 端点桩 (OUT 收到即排空并重挂读, 不回数据)。
 *
 *  >>> 决策待定 (2026-06-03): v1(HID) 已被 ARM 标 deprecated, 目标 host 全走 v2。 <<<
 *  本接口是"是否实现 vs 直接删除"的候选 —— 在 8-端点芯片 (HPM6450/6200) 上, 删掉它
 *  (IF2/EP3) 正是给 CMSIS-DAP v2 SWO streaming 让出端点的来源。 端点回收方案见
 *  usb_device.h 头部"SWO Stream 端点回收"备注。 暂保留枚举桩, 不投入 v1 协议实现。
 * ============================================================================
 */
#include <rtthread.h>

#include "usbd_core.h"
#include "usbd_hid.h"

#include "usb_device.h"
#include "usb_if_dapv1.h"

/* ---- CMSIS-DAP v1 标准 HID report descriptor (vendor 0xFF00, 64B in/out) ----
 * 长度必须等于 usb_device.h 的 EXLINK_HID_REPORT_DESC_SIZE (=33), 否则 config 里
 * HID class descriptor 的 wItemLength 与实体不符, 主机取 report 描述符会失败。
 */
static const uint8_t s_dapv1_hid_report_desc[EXLINK_HID_REPORT_DESC_SIZE] = {
    0x06, 0x00, 0xFF,  /* Usage Page (Vendor Defined 0xFF00) */
    0x09, 0x01,        /* Usage (0x01) */
    0xA1, 0x01,        /* Collection (Application) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00,  /*   Logical Maximum (255) */
    0x75, 0x08,        /*   Report Size (8) */
    0x95, 0x40,        /*   Report Count (64) */
    0x09, 0x01,        /*   Usage (0x01) */
    0x81, 0x02,        /*   Input (Data,Var,Abs) */
    0x95, 0x40,        /*   Report Count (64) */
    0x09, 0x01,        /*   Usage (0x01) */
    0x91, 0x02,        /*   Output (Data,Var,Abs) */
    0x95, 0x01,        /*   Report Count (1) */
    0x09, 0x01,        /*   Usage (0x01) */
    0xB1, 0x02,        /*   Feature (Data,Var,Abs) */
    0xC0               /* End Collection */
};

/* nocache DMA 收缓冲 (独立) */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t s_hid_out_buf[EXLINK_HID_MPS];

/* ===========================================================================
 *  端点回调 (F1: 排空桩)
 * ========================================================================= */
static void hid_out_cb(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)ep;
    (void)nbytes;
    /* F3: 此处接 CMSIS-DAP v1 命令处理。 F1 仅排空并重挂读, 不回数据。 */
    usbd_ep_start_read(busid, EXLINK_EP_HID_OUT, s_hid_out_buf, EXLINK_HID_MPS);
}
static void hid_in_cb(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid; (void)ep; (void)nbytes;
}

static struct usbd_endpoint s_hid_out_ep = { .ep_addr = EXLINK_EP_HID_OUT, .ep_cb = hid_out_cb };
static struct usbd_endpoint s_hid_in_ep  = { .ep_addr = EXLINK_EP_HID_IN,  .ep_cb = hid_in_cb  };

static struct usbd_interface s_hid_intf;   /* IF2: HID 接口 (由 usbd_hid_init_intf 填) */

/* ===========================================================================
 *  装配 + 事件 hook
 *  注意: 须在 usb_if_dap (IF1) 之后注册, 与 config 描述符顺序一致 (接口号自增)。
 * ========================================================================= */
void usb_if_dapv1_add(uint8_t busid)
{
    /* IF2: CMSIS-DAP v1 HID (EP3) */
    usbd_add_interface(busid, usbd_hid_init_intf(busid, &s_hid_intf,
                                                 s_dapv1_hid_report_desc,
                                                 sizeof(s_dapv1_hid_report_desc)));
    usbd_add_endpoint(busid, &s_hid_out_ep);
    usbd_add_endpoint(busid, &s_hid_in_ep);
}

void usb_if_dapv1_on_configured(uint8_t busid)
{
    /* 挂 OUT 读, 避免 host 向 HID 写入时端点 NAK 挂死 */
    usbd_ep_start_read(busid, EXLINK_EP_HID_OUT, s_hid_out_buf, EXLINK_HID_MPS);
}

void usb_if_dapv1_on_reset(void)
{
    /* F1 无运行时状态需复位 */
}
