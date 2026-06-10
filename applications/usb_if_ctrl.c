/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink USB 控制通道接口 —— 复合设备 IF0 (vendor bulk EP1) 运行时
 * ============================================================================
 *
 *  描述符 (device/config/MS OS 2.0/BOS/strings) 统一在 usb_device.c;
 *  本文件只保留控制通道的**运行时逻辑**: EP1 in/out 端点回调 + RX 无锁环 + 阻塞 TX。
 *  承载命令帧 (MAGIC/OP/LEN/PAYLOAD/CRC16, PING/PONG 等); 注册/初始化拆成
 *  usb_if_ctrl_add() + on_configured/on_reset hook, 由复合设备装配核心驱动。
 * ============================================================================
 */
#include <rtthread.h>
#include <string.h>

#include "usbd_core.h"
#include "usb_device.h"    /* EXLINK_EP_CMD_*, EXLINK_BULK_MPS */
#include "usb_if_ctrl.h"
#include "frame_proto.h"   /* EXLINK_MAX_FRAME */

/* ===========================================================================
 *  RX 无锁 SPSC 环 (生产者 = OUT 回调/ISR, 消费者 = 帧分发线程)
 *  容量 2 的幂, 用 mask 取模; 满则丢弃尾部新字节并计数。
 * ========================================================================= */
#define EXLINK_RX_RING_SIZE  8192u   /* 2 的幂; >= MAX_FRAME(4103) 可容纳单个最大帧 */
#define EXLINK_RX_RING_MASK  (EXLINK_RX_RING_SIZE - 1u)

static uint8_t           s_rx_ring[EXLINK_RX_RING_SIZE];
static volatile uint16_t s_rx_head;  /* 生产者写 */
static volatile uint16_t s_rx_tail;  /* 消费者写 */
static volatile uint32_t s_rx_dropped;

struct rt_semaphore usb_if_ctrl_rx_sem;

/* DMA 端点缓冲 (nocache, 对齐) —— OUT 收, IN 发各一块 */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t s_ep_out_buf[EXLINK_BULK_MPS];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t s_ep_in_buf[EXLINK_MAX_FRAME];

static volatile bool s_configured = false;
static volatile bool s_tx_busy    = false;
static uint8_t       s_busid      = 0;

/* 生产者: 把 n 字节塞入环, 满则丢弃并计数 (ISR 上下文, 不阻塞) */
static void rx_ring_put(const uint8_t *data, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        uint16_t next = (uint16_t)((s_rx_head + 1u) & EXLINK_RX_RING_MASK);
        if (next == s_rx_tail) {
            s_rx_dropped++;   /* 环满 */
            return;
        }
        s_rx_ring[s_rx_head] = data[i];
        s_rx_head = next;
    }
}

/* 消费者: 取 1 字节 */
int usb_if_ctrl_rx_getchar(uint8_t *out)
{
    if (s_rx_tail == s_rx_head) {
        return 0;   /* 空 */
    }
    *out = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & EXLINK_RX_RING_MASK);
    return 1;
}

/* ===========================================================================
 *  端点回调
 * ========================================================================= */
static void exlink_ctrl_out_cb(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)ep;
    if (nbytes > 0) {
        rx_ring_put(s_ep_out_buf, nbytes);
        rt_sem_release(&usb_if_ctrl_rx_sem);
    }
    /* 重新挂下一次 OUT 读 */
    usbd_ep_start_read(busid, EXLINK_EP_CMD_OUT, s_ep_out_buf, EXLINK_BULK_MPS);
}

static void exlink_ctrl_in_cb(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    /* 若本次发送长度为 MPS 整数倍且非 0, 补一个 ZLP 让主机知道传输结束 */
    if (nbytes > 0 && (nbytes % usbd_get_ep_mps(busid, ep)) == 0) {
        usbd_ep_start_write(busid, EXLINK_EP_CMD_IN, NULL, 0);
    } else {
        s_tx_busy = false;
    }
}

static struct usbd_endpoint exlink_ctrl_out_ep = {
    .ep_addr = EXLINK_EP_CMD_OUT,
    .ep_cb   = exlink_ctrl_out_cb,
};
static struct usbd_endpoint exlink_ctrl_in_ep = {
    .ep_addr = EXLINK_EP_CMD_IN,
    .ep_cb   = exlink_ctrl_in_cb,
};

static struct usbd_interface exlink_ctrl_intf;   /* 裸 vendor 接口 (无 class init) */

/* ===========================================================================
 *  装配 + 事件 hook
 * ========================================================================= */
void usb_if_ctrl_add(uint8_t busid)
{
    s_busid      = busid;
    s_rx_head    = 0;
    s_rx_tail    = 0;
    s_rx_dropped = 0;
    s_configured = false;
    s_tx_busy    = false;

    rt_sem_init(&usb_if_ctrl_rx_sem, "exl_rx", 0, RT_IPC_FLAG_FIFO);

    usbd_add_interface(busid, &exlink_ctrl_intf);
    usbd_add_endpoint(busid, &exlink_ctrl_out_ep);
    usbd_add_endpoint(busid, &exlink_ctrl_in_ep);
}

void usb_if_ctrl_on_configured(uint8_t busid)
{
    s_configured = true;
    s_tx_busy    = false;
    /* 配置完成: 挂第一次 OUT 读 */
    usbd_ep_start_read(busid, EXLINK_EP_CMD_OUT, s_ep_out_buf, EXLINK_BULK_MPS);
}

void usb_if_ctrl_on_reset(void)
{
    s_configured = false;
    s_tx_busy    = false;
}

/* ===========================================================================
 *  TX —— 阻塞发送一帧 (等上一帧完成 + 等本帧 ZLP 收尾)
 * ========================================================================= */
int usb_if_ctrl_send(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }
    if (!s_configured) {
        return -2;   /* 未枚举/未配置 */
    }
    if (len > sizeof(s_ep_in_buf)) {
        return -3;
    }

    /* 控制通道低频, 用 1ms 步进的计数忙等 (上限 500ms) 而非 tick 回绕比较 */
    const int TX_TIMEOUT_MS = 500;

    /* 等待上一次发送完成 */
    for (int waited = 0; s_tx_busy; waited++) {
        if (waited >= TX_TIMEOUT_MS) {
            return -4;   /* 超时 */
        }
        rt_thread_mdelay(1);
    }

    rt_memcpy(s_ep_in_buf, data, len);
    s_tx_busy = true;
    int ret = usbd_ep_start_write(s_busid, EXLINK_EP_CMD_IN, s_ep_in_buf, len);
    if (ret < 0) {
        s_tx_busy = false;
        return -5;
    }

    /* 等待本帧 (及可能的 ZLP) 真正发完 */
    for (int waited = 0; s_tx_busy; waited++) {
        if (waited >= TX_TIMEOUT_MS) {
            return -6;   /* 超时 (数据可能已部分发出) */
        }
        rt_thread_mdelay(1);
    }
    return 0;
}
