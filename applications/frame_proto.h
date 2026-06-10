/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink 命令帧协议 —— 跨端契约 (FROZEN, Sprint 0 / 2026-06-01)
 * ============================================================================
 *
 *  本文件是 ExLink 固件 <-> 上位机 (ExLink-Tool) 命令帧的 **单一真相源**。
 *  上位机端 frame_proto.py 是本文件的镜像 (手工同步, 或脚本生成 + CI 字节级校验)。
 *
 *  >>> 任何 op-code / 帧布局 / 载荷布局变更, 必须双端同步, 由 Boss 仲裁。 <<<
 *
 *  帧布局 (little-endian):
 *
 *    +--------+--------+--------+------------------+----------+
 *    | MAGIC  |  OP    |  LEN   |     PAYLOAD      |  CRC16   |
 *    | 2 byte | 1 byte | 2 byte |    LEN bytes     |  2 byte  |
 *    +--------+--------+--------+------------------+----------+
 *
 *    MAGIC = 0x4C 0x4B  ('L','K'), 在线序中先 0x4C 后 0x4B
 *    OP    = 操作码 (见 exlink_op_t)
 *    LEN   = PAYLOAD 字节数 (uint16, little-endian)
 *    CRC16 = CRC-16/CCITT-FALSE, 覆盖 MAGIC..PAYLOAD (即除 CRC16 自身外的全部),
 *            在线序中 little-endian (先低字节后高字节)
 *
 *  请求-响应约定:
 *    - 正常响应复用请求的 OP (PC 发 OP=0x01 PING, MCU 回 OP=0x01 PONG 载荷)
 *    - 任何失败回 OP=0xFF ERROR, 载荷首字节为 exlink_err_t 错误码
 *    - MCU 主动异步上报用 OP=0xFE LOG / 0x12 UART_READ / 0x41 CAN_RECV 等
 * ============================================================================
 */
#ifndef EXLINK_FRAME_PROTO_H
#define EXLINK_FRAME_PROTO_H

#include <stdint.h>

/* ---- 协议版本 (PONG 上报; major.minor, 变更帧布局/语义时 bump) ---- */
#define EXLINK_PROTO_VERSION_MAJOR  0x01u
#define EXLINK_PROTO_VERSION_MINOR  0x00u
#define EXLINK_PROTO_VERSION        ((uint16_t)((EXLINK_PROTO_VERSION_MAJOR << 8) | \
                                                 EXLINK_PROTO_VERSION_MINOR))

/* ---- 帧常量 ---- */
#define EXLINK_MAGIC0               0x4Cu   /* 'L' */
#define EXLINK_MAGIC1               0x4Bu   /* 'K' */

#define EXLINK_MAX_PAYLOAD          4096u   /* PAYLOAD 上限 (字节) */
#define EXLINK_HEADER_LEN           5u      /* MAGIC(2) + OP(1) + LEN(2) */
#define EXLINK_CRC_LEN              2u      /* CRC16(2) */
#define EXLINK_FRAME_OVERHEAD       (EXLINK_HEADER_LEN + EXLINK_CRC_LEN) /* = 7 */
#define EXLINK_MAX_FRAME            (EXLINK_FRAME_OVERHEAD + EXLINK_MAX_PAYLOAD)

/* ---- CRC-16/CCITT-FALSE 参数 (与上位机字节级一致) ----
 *   poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
 *   注意: 这不是 CRC-16/X25, 也不是 hpm_crc_drv 的 crc_preset_crc16_x25。
 */
#define EXLINK_CRC16_POLY           0x1021u
#define EXLINK_CRC16_INIT           0xFFFFu
#define EXLINK_CRC16_XOROUT         0x0000u

/* ---- 操作码 (op-code) 全表 ----
 * 即使 M1 只实现 PING, 全表在 Sprint 0 一并冻结, 防止双端漂移。
 * 未实现的 op 由 dispatcher 统一回 ERROR(NOT_IMPLEMENTED)。
 */
typedef enum {
    EXLINK_OP_PING            = 0x01u, /* PC->MCU 心跳, 返 PONG + 版本 (见 exlink_pong_t) */
    EXLINK_OP_ECHO            = 0x02u, /* PC<->MCU 回环: 原样返回 payload (bring-up 数据通道自测)
                                        * 新增 op, 经 Boss 仲裁 2026-06-02 (原 Sprint 0 表未含) */

    EXLINK_OP_UART_OPEN       = 0x10u, /* PC->MCU 开 UART 通道 (波特率/数据位/校验) */
    EXLINK_OP_UART_WRITE      = 0x11u, /* PC->MCU 写 UART 数据 */
    EXLINK_OP_UART_READ       = 0x12u, /* MCU->PC UART 读到数据上报 */

    EXLINK_OP_SPI_XFER        = 0x20u, /* PC->MCU SPI 主模式收发 */
    EXLINK_OP_SPI_SLAVE_TABLE = 0x21u, /* PC->MCU 配置 SPI slave 响应表 (mode A/B/C) */

    EXLINK_OP_I2C_XFER        = 0x30u, /* PC->MCU I2C 主模式收发 */
    EXLINK_OP_I2C_SLAVE_TABLE = 0x31u, /* PC->MCU 配置 I2C slave 响应表 */

    EXLINK_OP_CAN_SEND        = 0x40u, /* PC->MCU CAN-FD 发送 */
    EXLINK_OP_CAN_RECV        = 0x41u, /* MCU->PC CAN-FD 接收上报 */

    EXLINK_OP_DAP_PASSTHROUGH = 0x80u, /* 双向 CMSIS-DAP v2 帧透传 (实际走 HID 端点更高效) */

    EXLINK_OP_LOG             = 0xFEu, /* MCU->PC log 上报 (ASCII 文本) */
    EXLINK_OP_ERROR           = 0xFFu, /* MCU->PC 错误码 + 描述 (载荷见 exlink_error_payload_t) */
} exlink_op_t;

/* ---- 错误码 (ERROR 帧载荷首字节) ---- */
typedef enum {
    EXLINK_ERR_OK              = 0x00u, /* 无错 (一般不单独回, 用响应帧表示成功) */
    EXLINK_ERR_BAD_CRC         = 0x01u, /* CRC16 校验失败 */
    EXLINK_ERR_BAD_MAGIC       = 0x02u, /* MAGIC 不匹配 */
    EXLINK_ERR_BAD_LEN         = 0x03u, /* LEN 超界 / 帧长非法 */
    EXLINK_ERR_UNKNOWN_OP      = 0x04u, /* 未知 op-code */
    EXLINK_ERR_NOT_IMPLEMENTED = 0x05u, /* op 已定义但本固件版本未实现 */
    EXLINK_ERR_BUSY            = 0x06u, /* 资源忙 (通道占用 / TX 阻塞) */
    EXLINK_ERR_BAD_PARAM       = 0x07u, /* 载荷参数非法 */
} exlink_err_t;

/* ============================================================================
 *  载荷布局 (packed, little-endian)
 * ============================================================================ */

/* PONG (OP=0x01 响应) 载荷:
 *   proto_version (u16 LE) | fw_major (u8) | fw_minor (u8) | board_name (变长 ASCII, 以 NUL 结尾)
 */
#define EXLINK_FW_VERSION_MAJOR  0x00u
#define EXLINK_FW_VERSION_MINOR  0x01u   /* M1 骨架版本 */

#pragma pack(push, 1)
typedef struct {
    uint16_t proto_version;  /* = EXLINK_PROTO_VERSION */
    uint8_t  fw_major;       /* = EXLINK_FW_VERSION_MAJOR */
    uint8_t  fw_minor;       /* = EXLINK_FW_VERSION_MINOR */
    /* 其后紧跟 board_name 字符串 (含结尾 NUL), 变长, 不在本 struct 内 */
} exlink_pong_hdr_t;

/* ERROR (OP=0xFF) 载荷: err_code (u8) + 可选 ASCII 描述 (变长, 不强制 NUL) */
typedef struct {
    uint8_t err_code;        /* exlink_err_t */
    /* 其后可选描述文本 */
} exlink_error_payload_t;
#pragma pack(pop)

/* ============================================================================
 *  USB 标识 (固件与主机脚本共享 -> 属契约的一部分)
 *  注意: PID 为 M1 占位值, 正式分配见 docs/硬件平台设计文档/08-USB_VID_PID分配.md
 * ============================================================================ */
#define EXLINK_USB_VID   0x34B7u   /* HPMicro VID (沿用) */
#define EXLINK_USB_PID   0x0E11u   /* 占位: 'E11' ~ ExLink; 待 doc 08 正式分配 */

#endif /* EXLINK_FRAME_PROTO_H */
