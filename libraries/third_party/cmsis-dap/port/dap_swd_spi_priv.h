/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink CMSIS-DAP SWD-over-SPI —— 内部共享头 (传输层 ↔ 诊断命令)
 * ============================================================================
 *
 *  dap_swd_spi.c (传输 / 协议 / ADIv5 寄存器访问) 与 dap_swd_diag.c (bring-up 诊断
 *  msh 命令) 共享的常量、内部函数原型、bring-up 状态变量。 两 .c 都 #include 本头,
 *  顺带在编译期校验 extern 声明与定义一致。
 *
 *  这里暴露的符号多数是为诊断命令服务的内部接口; 联调完删 dap_swd_diag.c 时,
 *  本头中仅为其暴露的部分 (s_* 状态、swd_write_bits 等) 亦可收回为传输层 static。
 * ============================================================================
 */
#ifndef EXLINK_DAP_SWD_SPI_PRIV_H
#define EXLINK_DAP_SWD_SPI_PRIV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DAP.h 的传输/ACK 常量 (避免 #include DAP.h 引整套依赖, 本地定义,
 * 数值是 CMSIS-DAP 标准, 与 DAP.h 一致)。 */
#define DAP_TRANSFER_APnDP          (1U << 0)
#define DAP_TRANSFER_RnW            (1U << 1)
#define DAP_TRANSFER_A2             (1U << 2)
#define DAP_TRANSFER_A3             (1U << 3)
#define DAP_TRANSFER_OK             (1U << 0)
#define DAP_TRANSFER_WAIT           (1U << 1)
#define DAP_TRANSFER_FAULT          (1U << 2)
#define DAP_TRANSFER_ERROR          (1U << 3)
#define DAP_TRANSFER_MISMATCH       (1U << 4)

/* ===========================================================================
 *  ADIv5 DP/AP 寄存器访问 —— 在 SWD_Transfer 之上做上电握手 / AP 选择 / MEM-AP 读内存。
 *  (数值是 ADIv5 标准, 与 arm/DAP.h 一致。)
 * ========================================================================= */
/* DP 寄存器 offset (A[3:2] 对齐, 见请求编码) */
#define DP_IDCODE          0x00u   /* read  (= ABORT 同址不同向) */
#define DP_ABORT           0x00u   /* write */
#define DP_CTRL_STAT       0x04u   /* DPBANKSEL=0 时 */
#define DP_SELECT          0x08u   /* write */
#define DP_RDBUFF          0x0Cu   /* read  (取 posted AP 读真值) */

/* MEM-AP 寄存器 offset (在选中的 AP 内) */
#define AP_CSW             0x00u   /* APBANKSEL=0x0 */
#define AP_TAR             0x04u   /* APBANKSEL=0x0 */
#define AP_DRW             0x0Cu   /* APBANKSEL=0x0 */
#define AP_IDR             0xFCu   /* APBANKSEL=0xF */

/* DP ABORT: 清全部 sticky error */
#define ABORT_STKCMPCLR    (1u << 1)
#define ABORT_STKERRCLR    (1u << 2)
#define ABORT_WDERRCLR     (1u << 3)
#define ABORT_ORUNERRCLR   (1u << 4)
#define ABORT_CLR_ALL      (ABORT_STKCMPCLR | ABORT_STKERRCLR | \
                            ABORT_WDERRCLR | ABORT_ORUNERRCLR)      /* 0x1E */

/* DP CTRL/STAT: 调试/系统域 上电请求 + 应答 */
#define CTRL_CDBGPWRUPREQ  (1u << 28)
#define CTRL_CDBGPWRUPACK  (1u << 29)
#define CTRL_CSYSPWRUPREQ  (1u << 30)
#define CTRL_CSYSPWRUPACK  (1u << 31)
#define CTRL_PWRUP_REQ     (CTRL_CDBGPWRUPREQ | CTRL_CSYSPWRUPREQ)  /* 0x50000000 */
#define CTRL_PWRUP_ACK     (CTRL_CDBGPWRUPACK | CTRL_CSYSPWRUPACK)  /* 0xA0000000 */

/* 标准 32-bit MEM-AP CSW (pyOCD/OpenOCD known-good):
 * Size=010(32bit) | AddrInc=01(single, TAR 每次 DRW 自增 size) | DeviceEn(RO) |
 * 保留位 | HPROT[1]=1(privileged) | MasterType=1(debug master)。 */
#define CSW_VALUE_32       0x23000052u

/* bring-up 重试限 */
#define SWD_WAIT_RETRIES   64    /* 单事务 WAIT 重试 */
#define SWD_PWRUP_RETRIES  100   /* CTRL/STAT 上电 ACK 轮询 */

/* ===========================================================================
 *  bring-up 可调状态 (msh swd_cfg 改; 定义在 dap_swd_spi.c)
 * ========================================================================= */
extern volatile bool    s_read_even;   /* 读采样沿: false=CPHA0(上升沿) / true=CPHA1*/
extern volatile uint8_t s_trn;         /* turnaround 时钟数 (1..4) */
extern uint32_t         s_sel_cache;   /* DP SELECT 缓存 (连接后置无效) */

/* ===========================================================================
 *  传输层内部接口 (定义在 dap_swd_spi.c; 供 dap_swd_diag.c 调用)
 * ========================================================================= */
/* 低层位 I/O (单线/两脚两套实现, 按 EXLINK_DAP_SWD_TWO_WIRE 编其一) */
void     swd_write_bits(uint32_t val, uint8_t nbits);
uint32_t swd_read_bits(uint8_t nbits);
void     swd_write_bytes(const uint8_t *data, uint32_t nbytes);
void     swd_turnaround(void);

/* 标准 2-reset 连接序列 (单次连续 spi_transfer) */
void     swd_send_connect(void);

/* nRESET(PA26) 脉冲: 拉低 low_ms 毫秒再释放, 返回释放后读回电平 (1=高)。 */
uint8_t  dap_swd_nreset_pulse(uint32_t low_ms);

/* nRESET(PA26) 停在固定电平 (推挽, 万用表量电压用): lvl=0 低 / 非0 高。 */
void     dap_swd_nreset_park(uint8_t lvl);

/* ADIv5 DP/AP 寄存器访问 (含 WAIT 重试 / SELECT 管理 / posted-read) */
uint8_t  dp_read(uint8_t r, uint32_t *out);
uint8_t  dp_write(uint8_t r, uint32_t v);
uint8_t  ap_read(uint8_t apreg, uint32_t *out);
uint8_t  ap_write(uint8_t apreg, uint32_t v);

/* 官方 CMSIS-DAP 传输原语 (亦声明于 arm/DAP.h; 此处便于 diag 不引 DAP.h) */
uint8_t  SWD_Transfer(uint32_t request, uint32_t *data);

#ifdef __cplusplus
}
#endif

#endif /* EXLINK_DAP_SWD_SPI_PRIV_H */
