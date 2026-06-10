/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink CMSIS-DAP 引擎线程 (F3a)
 * ============================================================================
 *
 *  请求环 (生产者 = usb_if_dap 的 OUT 回调/ISR, 消费者 = 本线程):
 *  每槽存一个完整 bulk OUT 包 (DAP 请求, ≤ DAP_PACKET_SIZE)。 线程取出后调官方
 *  DAP_ExecuteCommand(req, resp) 处理, 低 16 位为响应长度, 经 usb_if_dap_send 发回。
 *
 *  DAP 命令可能慢 (SWD TransferBlock @ 1MHz 毫秒级), 故必须在独立线程跑, 不占 USB ISR。
 * ============================================================================
 */
#include <rtthread.h>
#include <string.h>

#include "usbd_core.h"     /* USB_NOCACHE_RAM_SECTION / USB_MEM_ALIGNX */
#include "DAP.h"           /* DAP_ExecuteCommand 声明 */
#include "DAP_config.h"    /* DAP_PACKET_SIZE / DAP_PACKET_COUNT */

#include "dap_engine.h"
#include "usb_if_dap.h"

/* 请求环: 要能同时容纳 host 背靠背发来的 DAP_PACKET_COUNT 条命令。
 * 环用"next==tail 即满"判定 → 必须留一个空槽, 故 N 个槽实际只能存 N-1 条。
 * 因此槽数 = DAP_PACKET_COUNT + 1, 才能真正存下 DAP_PACKET_COUNT 条 (否则最后一条
 * 被丢弃, pyOCD 等不到响应而 "Timeout reading from probe")。 */
#define DAP_REQ_SLOTS   (DAP_PACKET_COUNT + 1u)

typedef struct {
    uint32_t len;
    uint8_t  buf[DAP_PACKET_SIZE];
} dap_req_slot_t;

/* nocache (DAP 处理本身不 DMA, 但与 USB 缓冲同区, 保持一致对齐) */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static dap_req_slot_t s_req[DAP_REQ_SLOTS];
static volatile uint16_t s_req_head;   /* 生产者写 (ISR) */
static volatile uint16_t s_req_tail;   /* 消费者写 (线程) */
static volatile uint32_t s_req_dropped;

/* ---- 调试计数 ---- */
#define DAP_ENGINE_DEBUG    1               /* 1: 每条命令打印一行 (定位用, 联调后置 0) */
static volatile uint32_t s_req_submitted;   /* 累计入环成功 (ISR 递增) */
static volatile uint32_t s_req_processed;   /* 累计引擎处理完 */
static volatile int      s_last_send_ret;   /* 最近一次 usb_if_dap_send 返回值 */

/* 响应缓冲 (线程私有, 经 usb_if_dap_send 拷到 nocache IN 缓冲) */
static uint8_t s_resp[DAP_PACKET_SIZE];

static struct rt_semaphore s_dap_sem;

#define DAP_ENGINE_THREAD_PRIO    12     /* 同帧分发 */
#define DAP_ENGINE_STACK_SIZE     4096

/* ---- 生产者: ISR 上下文, 不阻塞 ---- */
int dap_engine_submit(const uint8_t *data, uint32_t len)
{
    uint16_t next = (uint16_t)((s_req_head + 1u) % DAP_REQ_SLOTS);
    if (next == s_req_tail) {
        s_req_dropped++;          /* 环满: 丢弃 (DAP 上层会超时重试) */
        return -1;
    }
    if (len > DAP_PACKET_SIZE) {
        len = DAP_PACKET_SIZE;
    }
    rt_memcpy(s_req[s_req_head].buf, data, len);
    s_req[s_req_head].len = len;
    s_req_head = next;
    s_req_submitted++;
    rt_sem_release(&s_dap_sem);
    return 0;
}

/* ---- 消费者线程 ---- */
static void dap_engine_entry(void *arg)
{
    (void)arg;
    for (;;) {
        rt_sem_take(&s_dap_sem, RT_WAITING_FOREVER);

        while (s_req_tail != s_req_head) {
            dap_req_slot_t *slot = &s_req[s_req_tail];

            /* 官方命令处理器: 返回值低 16 位 = 响应字节数, 高 16 位 = 消耗的请求字节数 */
            uint8_t  cmd_id = slot->buf[0];
            uint16_t inlen  = (uint16_t)slot->len;
            uint32_t r = DAP_ExecuteCommand(slot->buf, s_resp);
            uint32_t resp_len = r & 0xFFFFu;

            if (resp_len > 0u) {
                s_last_send_ret = usb_if_dap_send(s_resp, resp_len);
            }
            s_req_processed++;

            /* 先释放槽位 (推进 tail), 再打印调试 —— 否则 UART 打印会拖慢 tail,
             * 让背靠背来的下一条在窗口期被误判为环满而丢弃。 */
            s_req_tail = (uint16_t)((s_req_tail + 1u) % DAP_REQ_SLOTS);
#if DAP_ENGINE_DEBUG
            rt_kprintf("[dap] cmd=0x%02x in=%u resp=%u send=%d\n",
                       cmd_id, inlen, resp_len, s_last_send_ret);
#endif
        }
    }
}

int dap_engine_thread_start(void)
{
    s_req_head = 0;
    s_req_tail = 0;
    s_req_dropped = 0;
    rt_sem_init(&s_dap_sem, "dap", 0, RT_IPC_FLAG_FIFO);

    rt_thread_t tid = rt_thread_create("dap_eng", dap_engine_entry, RT_NULL,
                                       DAP_ENGINE_STACK_SIZE, DAP_ENGINE_THREAD_PRIO, 10);
    if (tid == RT_NULL) {
        return -1;
    }
    rt_thread_startup(tid);
    return 0;
}

/* ===========================================================================
 *  调试: msh 命令 dap_stat —— 一眼看出响应链断在哪一环
 *  USB OUT cb -> submit(入环) -> 引擎 processed -> usb_if_dap_send -> USB IN cb
 * ========================================================================= */
#if DAP_ENGINE_DEBUG
extern volatile uint32_t g_dap_out_cb_calls;   /* usb_if_dap.c */
extern volatile uint32_t g_dap_out_zero;
extern volatile uint32_t g_dap_in_cb_calls;

static int dap_stat(int argc, char **argv)
{
    (void)argc; (void)argv;
    rt_kprintf("--- DAP link counters ---\n");
    rt_kprintf("USB OUT cb calls : %u (zero-len: %u)\n", g_dap_out_cb_calls, g_dap_out_zero);
    rt_kprintf("ring submitted   : %u\n", s_req_submitted);
    rt_kprintf("ring dropped     : %u\n", s_req_dropped);
    rt_kprintf("engine processed : %u\n", s_req_processed);
    rt_kprintf("last send ret    : %d\n", s_last_send_ret);
    rt_kprintf("USB IN cb calls  : %u\n", g_dap_in_cb_calls);
    rt_kprintf("head/tail        : %u / %u\n", s_req_head, s_req_tail);
    return 0;
}
MSH_CMD_EXPORT(dap_stat, show CMSIS-DAP v2 link counters);
#endif
