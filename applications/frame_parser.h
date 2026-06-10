/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ExLink 命令帧 codec: CRC-16/CCITT-FALSE + 字节流解析状态机 + 组帧。
 * 传输无关 (不依赖 USB), 单元可测。
 */
#ifndef EXLINK_FRAME_PARSER_H
#define EXLINK_FRAME_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include "frame_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- CRC-16/CCITT-FALSE ---- */

/* 增量计算: 以 crc 为起点喂入 buf[0..len), 返回更新后的 CRC。
 * 整帧一次性计算时传 crc = EXLINK_CRC16_INIT。 */
uint16_t exlink_crc16_update(uint16_t crc, const uint8_t *buf, size_t len);

/* 便捷: 对 buf[0..len) 从 INIT 起算一遍 (等价 update(INIT, buf, len))。 */
uint16_t exlink_crc16(const uint8_t *buf, size_t len);

/* ---- 组帧 ----
 * 把 (op, payload[0..payload_len)) 编码成完整帧写入 out。
 * out 必须至少 EXLINK_FRAME_OVERHEAD + payload_len 字节。
 * 返回写入的总字节数; 若 payload_len 越界或 out 为空返回 0。
 */
size_t exlink_frame_build(uint8_t op, const uint8_t *payload, uint16_t payload_len,
                          uint8_t *out, size_t out_cap);

/* ---- 解析状态机 (逐字节喂入, 单生产者) ---- */

typedef enum {
    EXLINK_PS_MAGIC0 = 0,
    EXLINK_PS_MAGIC1,
    EXLINK_PS_OP,
    EXLINK_PS_LEN_LO,
    EXLINK_PS_LEN_HI,
    EXLINK_PS_PAYLOAD,
    EXLINK_PS_CRC_LO,
    EXLINK_PS_CRC_HI,
} exlink_parse_state_t;

typedef struct {
    exlink_parse_state_t state;
    uint8_t  op;
    uint16_t len;            /* 期望 payload 长度 */
    uint16_t idx;            /* payload 已收字节数 */
    uint16_t crc_calc;       /* 累积 CRC (覆盖 MAGIC..PAYLOAD) */
    uint16_t crc_rx;         /* 收到的 CRC */
    uint8_t  payload[EXLINK_MAX_PAYLOAD];
} exlink_parser_t;

/* 解析结果 (单字节喂入的返回值) */
typedef enum {
    EXLINK_FR_NONE = 0,      /* 帧未完成, 继续喂 */
    EXLINK_FR_OK,            /* 一个完整且 CRC 正确的帧就绪 (见 parser->op/len/payload) */
    EXLINK_FR_BAD_CRC,       /* 帧结构完整但 CRC 不匹配 */
    EXLINK_FR_BAD_LEN,       /* LEN 超过 EXLINK_MAX_PAYLOAD, 帧被丢弃 */
} exlink_frame_result_t;

void exlink_parser_init(exlink_parser_t *p);

/* 喂入一个字节, 推进状态机。 返回 EXLINK_FR_* 。
 * 收到 OK/BAD_CRC/BAD_LEN 后, 解析器自动复位到等待下一帧 (MAGIC0)。 */
exlink_frame_result_t exlink_parser_feed(exlink_parser_t *p, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* EXLINK_FRAME_PARSER_H */
