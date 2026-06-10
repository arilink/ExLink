/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ExLink 命令帧 codec 实现。 见 frame_parser.h。
 */
#include "frame_parser.h"

/* ---------------------------------------------------------------------------
 *  CRC-16/CCITT-FALSE  (poly=0x1021, init=0xFFFF, refin/out=false, xorout=0x0000)
 *  位反/异或全无, 所以无需查表也很快; 这里用逐位实现保持确定性与可移植性。
 * ------------------------------------------------------------------------- */
uint16_t exlink_crc16_update(uint16_t crc, const uint8_t *buf, size_t len)
{
    if (buf == NULL) {
        return crc;
    }
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)buf[i] << 8);
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ EXLINK_CRC16_POLY);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint16_t exlink_crc16(const uint8_t *buf, size_t len)
{
    return exlink_crc16_update((uint16_t)EXLINK_CRC16_INIT, buf, len);
}

/* ---------------------------------------------------------------------------
 *  组帧
 * ------------------------------------------------------------------------- */
size_t exlink_frame_build(uint8_t op, const uint8_t *payload, uint16_t payload_len,
                          uint8_t *out, size_t out_cap)
{
    if (out == NULL) {
        return 0;
    }
    if (payload_len > EXLINK_MAX_PAYLOAD) {
        return 0;
    }
    size_t total = (size_t)EXLINK_FRAME_OVERHEAD + (size_t)payload_len;
    if (out_cap < total) {
        return 0;
    }

    /* 头部 */
    out[0] = EXLINK_MAGIC0;
    out[1] = EXLINK_MAGIC1;
    out[2] = op;
    out[3] = (uint8_t)(payload_len & 0xFFu);          /* LEN lo */
    out[4] = (uint8_t)((payload_len >> 8) & 0xFFu);    /* LEN hi */

    /* 载荷 */
    for (uint16_t i = 0; i < payload_len; i++) {
        out[EXLINK_HEADER_LEN + i] = payload ? payload[i] : 0u;
    }

    /* CRC 覆盖 MAGIC..PAYLOAD = out[0 .. HEADER_LEN+payload_len) */
    uint16_t crc = exlink_crc16(out, (size_t)EXLINK_HEADER_LEN + payload_len);
    out[EXLINK_HEADER_LEN + payload_len]     = (uint8_t)(crc & 0xFFu);         /* CRC lo */
    out[EXLINK_HEADER_LEN + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFFu);  /* CRC hi */

    return total;
}

/* ---------------------------------------------------------------------------
 *  解析状态机
 * ------------------------------------------------------------------------- */
void exlink_parser_init(exlink_parser_t *p)
{
    if (p == NULL) {
        return;
    }
    p->state    = EXLINK_PS_MAGIC0;
    p->op       = 0;
    p->len      = 0;
    p->idx      = 0;
    p->crc_calc = (uint16_t)EXLINK_CRC16_INIT;
    p->crc_rx   = 0;
}

exlink_frame_result_t exlink_parser_feed(exlink_parser_t *p, uint8_t byte)
{
    if (p == NULL) {
        return EXLINK_FR_NONE;
    }

    switch (p->state) {
    case EXLINK_PS_MAGIC0:
        if (byte == EXLINK_MAGIC0) {
            p->crc_calc = exlink_crc16_update((uint16_t)EXLINK_CRC16_INIT, &byte, 1);
            p->state = EXLINK_PS_MAGIC1;
        }
        /* 否则停在 MAGIC0, 跳过噪声字节 (重同步) */
        break;

    case EXLINK_PS_MAGIC1:
        if (byte == EXLINK_MAGIC1) {
            p->crc_calc = exlink_crc16_update(p->crc_calc, &byte, 1);
            p->state = EXLINK_PS_OP;
        } else if (byte == EXLINK_MAGIC0) {
            /* 连续 0x4C: 保持在期待 MAGIC1, 重置 CRC 起点 */
            p->crc_calc = exlink_crc16_update((uint16_t)EXLINK_CRC16_INIT, &byte, 1);
        } else {
            p->state = EXLINK_PS_MAGIC0;
        }
        break;

    case EXLINK_PS_OP:
        p->op = byte;
        p->crc_calc = exlink_crc16_update(p->crc_calc, &byte, 1);
        p->state = EXLINK_PS_LEN_LO;
        break;

    case EXLINK_PS_LEN_LO:
        p->len = byte;
        p->crc_calc = exlink_crc16_update(p->crc_calc, &byte, 1);
        p->state = EXLINK_PS_LEN_HI;
        break;

    case EXLINK_PS_LEN_HI:
        p->len |= (uint16_t)((uint16_t)byte << 8);
        p->crc_calc = exlink_crc16_update(p->crc_calc, &byte, 1);
        if (p->len > EXLINK_MAX_PAYLOAD) {
            /* 非法长度: 丢弃并重同步 */
            exlink_parser_init(p);
            return EXLINK_FR_BAD_LEN;
        }
        p->idx = 0;
        p->state = (p->len == 0) ? EXLINK_PS_CRC_LO : EXLINK_PS_PAYLOAD;
        break;

    case EXLINK_PS_PAYLOAD:
        p->payload[p->idx++] = byte;
        p->crc_calc = exlink_crc16_update(p->crc_calc, &byte, 1);
        if (p->idx >= p->len) {
            p->state = EXLINK_PS_CRC_LO;
        }
        break;

    case EXLINK_PS_CRC_LO:
        p->crc_rx = byte;
        p->state = EXLINK_PS_CRC_HI;
        break;

    case EXLINK_PS_CRC_HI:
        p->crc_rx |= (uint16_t)((uint16_t)byte << 8);
        {
            exlink_frame_result_t r = (p->crc_rx == p->crc_calc)
                                          ? EXLINK_FR_OK
                                          : EXLINK_FR_BAD_CRC;
            /* 复位等待下一帧, 但保留 op/len/payload 供调用方在本次返回后读取 */
            p->state    = EXLINK_PS_MAGIC0;
            p->crc_calc = (uint16_t)EXLINK_CRC16_INIT;
            return r;
        }

    default:
        exlink_parser_init(p);
        break;
    }

    return EXLINK_FR_NONE;
}
