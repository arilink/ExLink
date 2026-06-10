/*
 * Copyright (c) 2026 LabForge / ExLink
 * Portions derived from CherryUSB cdc_acm_multi_template.c (Apache-2.0, sakumisu).
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink USB CDC-ACM × 2 —— 复合设备 IF3..IF6, F1 回环 echo
 * ============================================================================
 *
 *  两路 CDC ACM 虚拟串口, 各自独立 nocache 收/发缓冲 (不共用一块全局 buf)。
 *  OUT 收到数据 -> 拷到 tx 缓冲 -> 从同口 IN 原样回写 (echo) -> 重挂 OUT 读。
 *
 *  >>> 真正接片上 UART (DMA + ring buffer) 的透传 = 工作包 F4。 <<<
 *  F4 时把 echo (cdc_port_out) 换成"写 UART TX", 并由 UART RX 触发"写 CDC IN" 即可。
 * ============================================================================
 */
#include <rtthread.h>
#include <string.h>

#include "usbd_core.h"
#include "usbd_cdc_acm.h"

#include "usb_device.h"
#include "usb_if_uart.h"

typedef struct {
    uint8_t       out_ep;     /* 数据 OUT (host->dev) */
    uint8_t       in_ep;      /* 数据 IN  (dev->host) */
    uint8_t      *rx;         /* nocache 收缓冲 */
    uint8_t      *tx;         /* nocache 发缓冲 */
    volatile bool tx_busy;
} cdc_port_t;

/* 各口独立 nocache DMA 缓冲 */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t s_cdc1_rx[EXLINK_BULK_MPS];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t s_cdc1_tx[EXLINK_BULK_MPS];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t s_cdc2_rx[EXLINK_BULK_MPS];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t s_cdc2_tx[EXLINK_BULK_MPS];

static cdc_port_t s_cdc[2] = {
    { EXLINK_EP_CDC1_OUT, EXLINK_EP_CDC1_IN, s_cdc1_rx, s_cdc1_tx, false },
    { EXLINK_EP_CDC2_OUT, EXLINK_EP_CDC2_IN, s_cdc2_rx, s_cdc2_tx, false },
};

/* OUT 完成: echo 回 IN (若上一帧未发完则本帧丢弃, 仅 F1 回环, 人手速够用), 重挂读 */
static void cdc_port_out(uint8_t busid, cdc_port_t *p, uint32_t nbytes)
{
    if (nbytes > 0 && !p->tx_busy) {
        rt_memcpy(p->tx, p->rx, nbytes);
        p->tx_busy = true;
        usbd_ep_start_write(busid, p->in_ep, p->tx, nbytes);
    }
    usbd_ep_start_read(busid, p->out_ep, p->rx, EXLINK_BULK_MPS);
}

/* IN 完成: MPS 整数倍补 ZLP, 否则清忙标志 */
static void cdc_port_in(uint8_t busid, cdc_port_t *p, uint32_t nbytes)
{
    if (nbytes > 0 && (nbytes % usbd_get_ep_mps(busid, p->in_ep)) == 0) {
        usbd_ep_start_write(busid, p->in_ep, NULL, 0);
    } else {
        p->tx_busy = false;
    }
}

static void cdc1_out_cb(uint8_t busid, uint8_t ep, uint32_t nbytes) { (void)ep; cdc_port_out(busid, &s_cdc[0], nbytes); }
static void cdc1_in_cb (uint8_t busid, uint8_t ep, uint32_t nbytes) { (void)ep; cdc_port_in (busid, &s_cdc[0], nbytes); }
static void cdc2_out_cb(uint8_t busid, uint8_t ep, uint32_t nbytes) { (void)ep; cdc_port_out(busid, &s_cdc[1], nbytes); }
static void cdc2_in_cb (uint8_t busid, uint8_t ep, uint32_t nbytes) { (void)ep; cdc_port_in (busid, &s_cdc[1], nbytes); }

static struct usbd_endpoint s_cdc1_out_ep = { .ep_addr = EXLINK_EP_CDC1_OUT, .ep_cb = cdc1_out_cb };
static struct usbd_endpoint s_cdc1_in_ep  = { .ep_addr = EXLINK_EP_CDC1_IN,  .ep_cb = cdc1_in_cb  };
static struct usbd_endpoint s_cdc2_out_ep = { .ep_addr = EXLINK_EP_CDC2_OUT, .ep_cb = cdc2_out_cb };
static struct usbd_endpoint s_cdc2_in_ep  = { .ep_addr = EXLINK_EP_CDC2_IN,  .ep_cb = cdc2_in_cb  };

/* 每路 CDC = comm + data 两个接口 */
static struct usbd_interface s_cdc1_comm, s_cdc1_data;
static struct usbd_interface s_cdc2_comm, s_cdc2_data;

/* ===========================================================================
 *  装配 + 事件 hook
 *  注意: 顺序必须 CDC1(comm,data) 再 CDC2(comm,data), 与 config 描述符一致。
 * ========================================================================= */
void usb_if_uart_add(uint8_t busid)
{
    /* CDC1: IF3 comm + IF4 data */
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &s_cdc1_comm));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &s_cdc1_data));
    usbd_add_endpoint(busid, &s_cdc1_out_ep);
    usbd_add_endpoint(busid, &s_cdc1_in_ep);

    /* CDC2: IF5 comm + IF6 data */
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &s_cdc2_comm));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &s_cdc2_data));
    usbd_add_endpoint(busid, &s_cdc2_out_ep);
    usbd_add_endpoint(busid, &s_cdc2_in_ep);
}

void usb_if_uart_on_configured(uint8_t busid)
{
    for (int i = 0; i < 2; i++) {
        s_cdc[i].tx_busy = false;
        usbd_ep_start_read(busid, s_cdc[i].out_ep, s_cdc[i].rx, EXLINK_BULK_MPS);
    }
}

void usb_if_uart_on_reset(void)
{
    s_cdc[0].tx_busy = false;
    s_cdc[1].tx_busy = false;
}
