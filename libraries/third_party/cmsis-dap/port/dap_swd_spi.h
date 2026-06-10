/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink CMSIS-DAP —— SWD over HPM SPI 硬件传输层 (F3a)
 * ============================================================================
 *
 *  为官方 CMSIS-DAP (third_party/cmsis-dap/arm/DAP.c) 提供 3 个 extern 传输函数
 *  (SWJ_Sequence / SWD_Sequence / SWD_Transfer), 用 HPM SPI 硬件外设打 SWD 时序
 *  (非 GPIO bit-bang, ARCHITECTURE §8)。 JTAG 留 F3b。
 *
 *  SPI 实例: SPI1 (HPM5361: SPI0 禁用保 JTAG, SPI1->DAP; 见 HPM5361 迁移约束)。
 *    SWCLK = SPI1 SCLK (PA27, 保留 LOOP_BACK 供主机自采样)
 *    SWDIO = SPI1 MOSI (PA29, 半双工 mosi_bidir)
 *    MISO  = SPI1 MISO (PA28, 仅两脚全双工模式用)
 *    nRESET= PA26 (SPI1_CS_0 覆盖成 GPIO; 见 dap_swd_spi.c swd_nreset_init)
 *  引脚来自 HPM5300EVK SPI1 (board/pinmux.c init_spi_pins); 改板/改脚时,
 *  改本文件 + board/pinmux.c + DAP_config.h 三处同步。
 * ============================================================================
 */
#ifndef EXLINK_DAP_SWD_SPI_H
#define EXLINK_DAP_SWD_SPI_H

#include <stdint.h>
#include "hpm_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DAP 用的 SPI 实例 (改实例时这一处 + pinmux + board_init_spi_clock 同步) */
#define EXLINK_DAP_SPI_BASE     HPM_SPI1

/* ===========================================================================
 *  ★ SWDIO 物理接法切换宏
 *  0 = 单线半双工 (mosi_bidir): 仅 MOSI(PA29) 一根线作 SWDIO, 写时驱动/读时三态释放,
 *      方向靠 trans_mode (write_only/read_only) 切, turnaround 处即切向。 无需外接元件。
 *  1 = 两脚全双工 (CherryDAP 式): MOSI(PA29) 经 ~1K 串阻接 SWDIO (写), MISO(PA28) 直连
 *      SWDIO (读)。 MOSI 恒输出/MISO 恒输入不切向, 读相位 MOSI 出 1 经 1K 成弱上拉,
 *      目标 ~50ohm 强驱动盖过。 需焊 1K 串阻 + 飞 MISO 线。
 *  二者只切换底层收发与 init 的 mosi_bidir; 上层 SWJ_Sequence/SWD_Transfer/msh 命令通用。
 * ========================================================================= */
#ifndef EXLINK_DAP_SWD_TWO_WIRE
#define EXLINK_DAP_SWD_TWO_WIRE   0
#endif

/* bring-up SWCLK 频率 (保守 1MHz; §8 物理上限 50MHz, 绿了再升) */
#define EXLINK_DAP_SWCLK_HZ_DEFAULT   1000000u

/* 初始化 DAP 的 SPI 外设 + 引脚 (clock / format=Mode0 LSB-first / mosi_bidir)。
 * 由 usb_if_dap_add() 在 usbd_initialize() 之前调用一次。 */
void dap_swd_spi_init(void);

/* 设置 SWCLK 频率 (DAP_SWJ_Clock 命令用; F3a 可选, 默认固定 1MHz)。 */
void dap_swd_spi_set_clock(uint32_t hz);

#ifdef __cplusplus
}
#endif

#endif /* EXLINK_DAP_SWD_SPI_H */
