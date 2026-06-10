/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ExLink 命令帧分发实现。 见 frame_dispatch.h。
 */
#include <rtthread.h>
#include <string.h>

#include "frame_proto.h"
#include "frame_parser.h"
#include "frame_dispatch.h"
#include "usb_if_ctrl.h"
#include "rtt_board.h"   /* BOARD_NAME via board.h */

#ifndef BOARD_NAME
#define BOARD_NAME "exlink"
#endif

/* ---------------------------------------------------------------------------
 *  op handlers —— 组好响应载荷后用 exlink_frame_build 封帧
 * ------------------------------------------------------------------------- */

/* 回一个 ERROR 帧 (op=0xFF, 载荷 = err_code)。 返回帧长。 */
static size_t build_error(exlink_err_t err, uint8_t *resp, size_t resp_cap)
{
    uint8_t code = (uint8_t)err;
    return exlink_frame_build(EXLINK_OP_ERROR, &code, 1, resp, resp_cap);
}

/* PING -> PONG: proto_version(u16) + fw_major + fw_minor + board_name(NUL 结尾) */
static size_t handle_ping(uint8_t *resp, size_t resp_cap)
{
    uint8_t payload[4 + 32];
    size_t  n = 0;

    payload[n++] = (uint8_t)(EXLINK_PROTO_VERSION & 0xFFu);          /* proto lo */
    payload[n++] = (uint8_t)((EXLINK_PROTO_VERSION >> 8) & 0xFFu);   /* proto hi */
    payload[n++] = (uint8_t)EXLINK_FW_VERSION_MAJOR;
    payload[n++] = (uint8_t)EXLINK_FW_VERSION_MINOR;

    const char *name = BOARD_NAME;
    size_t maxname = sizeof(payload) - n - 1;   /* 留 1 字节给 NUL */
    size_t i = 0;
    while (name[i] != '\0' && i < maxname) {
        payload[n++] = (uint8_t)name[i++];
    }
    payload[n++] = '\0';

    return exlink_frame_build(EXLINK_OP_PING, payload, (uint16_t)n, resp, resp_cap);
}

/* ECHO -> 原样回传 payload (回环自测: 验任意长度数据往返 + CRC + EP ZLP 边界) */
static size_t handle_echo(const uint8_t *payload, uint16_t len, uint8_t *resp, size_t resp_cap)
{
    return exlink_frame_build(EXLINK_OP_ECHO, payload, len, resp, resp_cap);
}

size_t exlink_dispatch(uint8_t op, const uint8_t *payload, uint16_t len,
                       uint8_t *resp, size_t resp_cap)
{
    switch (op) {
    case EXLINK_OP_PING:
        return handle_ping(resp, resp_cap);

    case EXLINK_OP_ECHO:
        return handle_echo(payload, len, resp, resp_cap);

    /* 已冻结但 M1 未实现的 op: 统一回 NOT_IMPLEMENTED */
    case EXLINK_OP_UART_OPEN:
    case EXLINK_OP_UART_WRITE:
    case EXLINK_OP_SPI_XFER:
    case EXLINK_OP_SPI_SLAVE_TABLE:
    case EXLINK_OP_I2C_XFER:
    case EXLINK_OP_I2C_SLAVE_TABLE:
    case EXLINK_OP_CAN_SEND:
    case EXLINK_OP_DAP_PASSTHROUGH:
        return build_error(EXLINK_ERR_NOT_IMPLEMENTED, resp, resp_cap);

    default:
        return build_error(EXLINK_ERR_UNKNOWN_OP, resp, resp_cap);
    }
}

/* ---------------------------------------------------------------------------
 *  帧分发线程
 * ------------------------------------------------------------------------- */
#define EXLINK_DISPATCH_THREAD_PRIO   12
#define EXLINK_DISPATCH_STACK_SIZE    4096   /* 含 parser(4K payload) + resp(4K) 都在 .bss/static, 栈本身够用 */

static exlink_parser_t s_parser;

/* parser 与响应缓冲放静态区, 避免吃线程栈 (各约 4KB) */
static uint8_t s_resp[EXLINK_MAX_FRAME];

static void exlink_frame_thread_entry(void *arg)
{
    (void)arg;
    exlink_parser_init(&s_parser);

    for (;;) {
        /* 等 OUT 回调通知有新数据 (周期性超时兜底, 防丢唤醒) */
        rt_sem_take(&usb_if_ctrl_rx_sem, rt_tick_from_millisecond(200));

        uint8_t b;
        while (usb_if_ctrl_rx_getchar(&b) == 1) {
            exlink_frame_result_t r = exlink_parser_feed(&s_parser, b);
            size_t resp_len = 0;

            switch (r) {
            case EXLINK_FR_OK:
                resp_len = exlink_dispatch(s_parser.op, s_parser.payload,
                                           s_parser.len, s_resp, sizeof(s_resp));
                break;
            case EXLINK_FR_BAD_CRC:
                resp_len = exlink_frame_build(EXLINK_OP_ERROR,
                                              (const uint8_t[]){ (uint8_t)EXLINK_ERR_BAD_CRC },
                                              1, s_resp, sizeof(s_resp));
                break;
            case EXLINK_FR_BAD_LEN:
                resp_len = exlink_frame_build(EXLINK_OP_ERROR,
                                              (const uint8_t[]){ (uint8_t)EXLINK_ERR_BAD_LEN },
                                              1, s_resp, sizeof(s_resp));
                break;
            case EXLINK_FR_NONE:
            default:
                break;
            }

            if (resp_len > 0) {
                usb_if_ctrl_send(s_resp, (uint32_t)resp_len);
            }
        }
    }
}

int exlink_frame_thread_start(void)
{
    rt_thread_t tid = rt_thread_create("exl_disp",
                                       exlink_frame_thread_entry, RT_NULL,
                                       EXLINK_DISPATCH_STACK_SIZE,
                                       EXLINK_DISPATCH_THREAD_PRIO, 10);
    if (tid == RT_NULL) {
        return -1;
    }
    rt_thread_startup(tid);
    return 0;
}
