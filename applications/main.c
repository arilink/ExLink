/*
 * Copyright (c) 2021 HPMicro
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * ExLink 固件入口 (F1 复合设备):
 *   - 启动 USB0 复合设备 (cmd / DAP-v2 / DAP-v1 HID / 2x CDC, 见 usb_device.c)
 *   - 启动帧分发线程 (解析命令帧 -> 路由 -> 回响应)
 *   - LED 心跳线程 (存活指示)
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "rtt_board.h"
#include "usb_config.h"

#include "usb_device.h"
#include "frame_dispatch.h"
#include "dap_engine.h"

static void led_heartbeat_entry(void *arg);

int main(void)
{
    app_init_led_pins();
    app_init_usb_pins();

    /* USB 设备中断优先级 (沿用 BSP demo 设定) */
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 2);

    /* 启动 USB0 复合设备 (cmd + DAP-v2 + DAP-v1 HID + 2x CDC) 与帧分发线程 */
    exlink_usb_init(0, CONFIG_HPM_USBD_BASE);
    if (exlink_frame_thread_start() != 0) {
        rt_kprintf("[exlink] frame dispatch thread start failed\n");
    }
    /* 启动 CMSIS-DAP 引擎线程 (F3a: SWD over SPI; 处理 EP2 上的 DAP v2 命令) */
    if (dap_engine_thread_start() != 0) {
        rt_kprintf("[exlink] dap engine thread start failed\n");
    }

    rt_kprintf("[exlink] F3a up (USB0: cmd/DAP-v2(SWD)/HID/2xCDC); cmd PING/PONG, DAP via pyOCD\n");

    /* LED 心跳 (低优先级, 存活指示) */
    rt_thread_t led_thread = rt_thread_create("led_hb", led_heartbeat_entry,
                                              RT_NULL, 512, 20, 10);
    if (led_thread != RT_NULL) {
        rt_thread_startup(led_thread);
    }

    return 0;
}

static void led_heartbeat_entry(void *arg)
{
    (void)arg;
    while (1) {
#ifdef APP_LED0
        app_led_write(APP_LED0, APP_LED_ON);
        rt_thread_mdelay(500);
        app_led_write(APP_LED0, APP_LED_OFF);
        rt_thread_mdelay(500);
#else
        rt_thread_mdelay(1000);
#endif
    }
}
