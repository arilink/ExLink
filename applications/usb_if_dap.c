/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink USB CMSIS-DAP v2 接口 —— 复合设备 IF1 (vendor bulk, EP2, WinUSB)
 * ============================================================================
 *
 *  CMSIS-DAP v2 (bulk, pyOCD/probe-rs/OpenOCD>=0.11 首选), MS OS 2.0 自动绑 WinUSB。
 *
 *  F3a: 端点桩升级为真 DAP 通路:
 *    OUT 回调 (ISR) -> dap_engine_submit() 入环 + 放信号量 -> 重挂读 (不在 ISR 跑 SWD)
 *    引擎线程处理后 -> usb_if_dap_send() 阻塞发回 EP2 IN (含 ZLP 收尾)
 *  SWD 时序在 third_party/cmsis-dap (官方 DAP.c + 自写 dap_swd_spi.c, HPM SPI 硬件)。
 *  v1 (HID 兜底) 独立接口见 usb_if_dapv1.c (F3a 仍为桩, 留 F3b)。
 * ============================================================================
 */
#include <rtthread.h>
#include <string.h>

#include "usbd_core.h"
#include "DAP.h"            /* DAP_Setup */

#include "usb_device.h"
#include "usb_if_dap.h"
#include "dap_engine.h"
#include "dap_swd_spi.h"

/* nocache DMA 缓冲: OUT 收 / IN 发 各一块 (独立, 不与命令通道共用) */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t s_dap_out_buf[EXLINK_BULK_MPS];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t s_dap_in_buf[EXLINK_BULK_MPS];

static volatile bool s_tx_busy = false;
static uint8_t       s_busid   = 0;

/* ---- 调试计数 (非 static, 供 dap_stat msh 命令读) ---- */
volatile uint32_t g_dap_out_cb_calls = 0;   /* OUT 端点回调被调次数 (host 写入到达) */
volatile uint32_t g_dap_out_zero     = 0;   /* 其中 nbytes==0 的次数 */
volatile uint32_t g_dap_in_cb_calls  = 0;   /* IN 端点回调被调次数 (响应发出完成) */

/* ===========================================================================
 *  端点回调
 * ========================================================================= */
static void dap_out_cb(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)ep;
    g_dap_out_cb_calls++;
    if (nbytes > 0) {
        dap_engine_submit(s_dap_out_buf, nbytes);   /* 入引擎环 + 放信号量 (ISR 安全) */
    } else {
        g_dap_out_zero++;
    }
    /* 重挂下一次 OUT 读 */
    usbd_ep_start_read(busid, EXLINK_EP_DAP_OUT, s_dap_out_buf, EXLINK_BULK_MPS);
}

static void dap_in_cb(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    g_dap_in_cb_calls++;
    /* MPS 整数倍且非 0 -> 补 ZLP, 否则清忙标志 */
    if (nbytes > 0 && (nbytes % usbd_get_ep_mps(busid, ep)) == 0) {
        usbd_ep_start_write(busid, EXLINK_EP_DAP_IN, NULL, 0);
    } else {
        s_tx_busy = false;
    }
}

static struct usbd_endpoint s_dap_out_ep = { .ep_addr = EXLINK_EP_DAP_OUT, .ep_cb = dap_out_cb };
static struct usbd_endpoint s_dap_in_ep  = { .ep_addr = EXLINK_EP_DAP_IN,  .ep_cb = dap_in_cb  };

static struct usbd_interface s_dap_intf;   /* IF1: 裸 vendor 接口 */

/* ===========================================================================
 *  TX —— 阻塞发送一个 DAP 响应包 (引擎线程调用; 等上一包完成 + 本包 ZLP 收尾)
 * ========================================================================= */
int usb_if_dap_send(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }
    if (len > sizeof(s_dap_in_buf)) {
        return -3;
    }

    const int TX_TIMEOUT_MS = 1000;   /* DAP 响应不应久等 */

    for (int waited = 0; s_tx_busy; waited++) {
        if (waited >= TX_TIMEOUT_MS) {
            return -4;
        }
        rt_thread_mdelay(1);
    }

    rt_memcpy(s_dap_in_buf, data, len);
    s_tx_busy = true;
    int ret = usbd_ep_start_write(s_busid, EXLINK_EP_DAP_IN, s_dap_in_buf, len);
    if (ret < 0) {
        s_tx_busy = false;
        return -5;
    }

    for (int waited = 0; s_tx_busy; waited++) {
        if (waited >= TX_TIMEOUT_MS) {
            return -6;
        }
        rt_thread_mdelay(1);
    }
    return 0;
}

/* ===========================================================================
 *  装配 + 事件 hook
 * ========================================================================= */
void usb_if_dap_add(uint8_t busid)
{
    s_busid   = busid;
    s_tx_busy = false;

    /* DAP 命令层 + SWD-over-SPI 传输层一次性初始化 (在 usbd_initialize 之前) */
    DAP_Setup();
    dap_swd_spi_init();

    /* IF1: CMSIS-DAP v2 vendor bulk (EP2) */
    usbd_add_interface(busid, &s_dap_intf);
    usbd_add_endpoint(busid, &s_dap_out_ep);
    usbd_add_endpoint(busid, &s_dap_in_ep);
}

void usb_if_dap_on_configured(uint8_t busid)
{
    s_tx_busy = false;
    /* 挂第一次 OUT 读 */
    usbd_ep_start_read(busid, EXLINK_EP_DAP_OUT, s_dap_out_buf, EXLINK_BULK_MPS);
}

void usb_if_dap_on_reset(void)
{
    s_tx_busy = false;
}
