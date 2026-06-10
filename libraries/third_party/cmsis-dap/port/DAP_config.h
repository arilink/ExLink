/*
 * Copyright (c) 2026 LabForge / ExLink
 * Structure modeled on ARM-software/CMSIS_5 DAP_config.h template (Apache-2.0).
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink CMSIS-DAP 配置 (F3a) —— SWD-only, 传输走 HPM SPI 硬件
 * ============================================================================
 *
 *  本文件被官方 DAP.c #include。 提供:
 *   - 能力/包大小宏 (DAP_SWD=1 / DAP_JTAG=0 / SWO=0 / UART=0 -> 报 SWD-only 能力)
 *   - DAP.c 直接用的 PIN_* / PORT_* / LED_* / RESET / TIMESTAMP 宏
 *  真正的 SWD 时序在 dap_swd_spi.c (SWJ_Sequence/SWD_Sequence/SWD_Transfer, SPI 硬件)。
 *
 *  PIN_* 内联原语仅被极少用到的 DAP_SWJ_Pins 命令调用 (pyOCD 的 identify 走
 *  SWJ_Sequence, 不走 SWJ_Pins)。 这里给 SWCLK/SWDIO 接 GPIO pad 直访 (HPM_GPIO),
 *  nRESET / JTAG 引脚在 F3a 作桩 (返回安全默认), 真 nRESET / JTAG 留 F3b。
 * ============================================================================
 */
#ifndef __DAP_CONFIG_H__
#define __DAP_CONFIG_H__

/* ---------------------------------------------------------------------------
 *  编译器宏垫片 (RISC-V / HPM, 无 ARM CMSIS-Core)
 *  官方 DAP.c / DAP.h 用 __STATIC_INLINE 等 (本由 ARM cmsis_compiler.h 提供)。
 *  本工程是 RISC-V + GCC, 这里直接给出 GCC 等价定义, 不依赖外部 cmsis 头,
 *  保证无论 include 顺序如何, 本文件内的内联函数都能正确解析。
 * ------------------------------------------------------------------------- */
#ifndef __STATIC_INLINE
#define __STATIC_INLINE        static inline
#endif
#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE   __attribute__((always_inline)) static inline
#endif
#ifndef __INLINE
#define __INLINE               inline
#endif
#ifndef __WEAK
#define __WEAK                 __attribute__((weak))
#endif
#ifndef __USED
#define __USED                 __attribute__((used))
#endif
#ifndef __NO_RETURN
#define __NO_RETURN            __attribute__((__noreturn__))
#endif
#ifndef __ASM
#define __ASM                  __asm
#endif

#include "hpm_soc.h"
#include "hpm_gpio_drv.h"
#include "hpm_clock_drv.h"
#include "board.h"

/* ===========================================================================
 *  DAP 能力 / 标识 / 包参数
 * ========================================================================= */
#define DAP_SWD                 1       /* 支持 SWD */
#define DAP_JTAG                0       /* F3a 不支持 JTAG (DAP.c 编译掉 JTAG 区段) */
#define DAP_JTAG_DEV_CNT        0
#define DAP_DEFAULT_PORT        1       /* 默认 SWD */
#define DAP_DEFAULT_SWJ_CLOCK   1000000 /* 默认 1MHz */

/* SWO 全关 (F3a 不做) */
#define SWO_UART                0
#define SWO_MANCHESTER          0
#define SWO_STREAM              0
#define SWO_BUFFER_SIZE         16U     /* 仅占位, SWO 关时不实际用 */

/* CMSIS-DAP v2 UART (DAP command 传 UART) 关; CDC 已由 usb_if_uart 独立提供 */
#define DAP_UART                0
#define DAP_UART_USB_COM_PORT   0

/* 包大小: 必须与 USB bulk MPS 一致 (HS=512); 见 usb_device.h EXLINK_BULK_MPS */
#ifdef CONFIG_USB_HS
#define DAP_PACKET_SIZE         512U
#else
#define DAP_PACKET_SIZE         64U
#endif
#define DAP_PACKET_COUNT        2U

/* MCU 核心时钟 (DAP.c 仅用于注释/SWD 慢速延时计算; SWD 时序实走 SPI 不靠它) */
#ifndef CPU_CLOCK
#define CPU_CLOCK               480000000U   /* HPM5361 max 480MHz */
#endif
#define IO_PORT_WRITE_CYCLES    2U
#define DAP_SWD_TURNAROUND      1U      /* turnaround 时钟数 (默认 1) */
#define DAP_SWD_IDLE_CYCLES     0U

/* 时间戳关 (TIMESTAMP_CLOCK=0 -> DAP_Info 报不支持) */
#define TIMESTAMP_CLOCK         0U
#define DAP_TIMESTAMP           0

/* 身份串 (DAP_Info 用)。 注: DAP_FW_VER 由 DAP.h 自带定义 (USB v2 = "2.1.1"), 此处不重定义。 */
#define DAP_VENDOR              "LabForge"
#define DAP_PRODUCT             "ExLink CMSIS-DAP"
#define DAP_SER_NUM             "EXLINK-F1-0001"
#define DAP_TARGET_DEVICE_FIXED 0
#define DAP_TARGET_DEVICE_VENDOR  ""
#define DAP_TARGET_DEVICE_NAME    ""
#define DAP_TARGET_BOARD_VENDOR   ""
#define DAP_TARGET_BOARD_NAME     ""

/* ===========================================================================
 *  GPIO 直访 (仅 DAP_SWJ_Pins 用; SWCLK/SWDIO = SPI1 pad PA27/PA29)
 *  PA27 = GPIO_DO_GPIOA[27], PA29 = GPIO_DO_GPIOA[29] (HPM port A)
 *  注: 正常 SWD 时序由 SPI 外设占用这些 pad; SWJ_Pins 仅在 pyOCD 罕用路径触发,
 *      F3a 给出能编译且语义合理的实现 (读写 GPIO), 不追求与 SPI 占用的严格切换。
 * ========================================================================= */
#define DAP_SWCLK_GPIO_CTRL     HPM_GPIO0
#define DAP_SWCLK_GPIO_PORT     GPIO_DO_GPIOA
#define DAP_SWCLK_GPIO_PIN      27u
#define DAP_SWDIO_GPIO_CTRL     HPM_GPIO0
#define DAP_SWDIO_GPIO_PORT     GPIO_DO_GPIOA
#define DAP_SWDIO_GPIO_PIN      29u
/* nRESET = PA26 (pinmux 默认配 SPI1_CS_0, 单线 SWD 不用硬件 CS; dap_swd_spi.c 的
 * swd_nreset_init() 把它覆盖成 GPIO 作 nRESET, 物理已接目标 NRST)。 */
#define DAP_nRESET_GPIO_CTRL    HPM_GPIO0
#define DAP_nRESET_GPIO_PORT    GPIO_DO_GPIOA
#define DAP_nRESET_GPIO_PIN     26u

/* ===========================================================================
 *  DAP.c 必需的端口/引脚原语
 * ========================================================================= */

/* 端口建立 / 关闭: 委托给 SPI 传输层 (在 dap_swd_spi.c 实现) */
extern void dap_swd_spi_port_setup(void);   /* 配置 SWCLK/SWDIO 为 SPI, 进 SWD 模式 */
extern void dap_swd_spi_port_off(void);      /* 释放, 引脚高阻 */

__STATIC_INLINE void PORT_SWD_SETUP(void) { dap_swd_spi_port_setup(); }
__STATIC_INLINE void PORT_JTAG_SETUP(void) { /* F3a: 无 JTAG */ }
__STATIC_INLINE void PORT_OFF(void) { dap_swd_spi_port_off(); }

/* DAP_Setup() 调用: 上电默认引脚状态 (端口先 OFF, 由 Connect 再 SETUP) */
__STATIC_INLINE void DAP_SETUP(void) { dap_swd_spi_port_off(); }

/* ---- SWCLK/TCK ---- (SWJ_Pins 用) */
__STATIC_INLINE uint32_t PIN_SWCLK_TCK_IN(void)
{ return gpio_read_pin(DAP_SWCLK_GPIO_CTRL, DAP_SWCLK_GPIO_PORT, DAP_SWCLK_GPIO_PIN); }
__STATIC_INLINE void PIN_SWCLK_TCK_SET(void)
{ gpio_write_pin(DAP_SWCLK_GPIO_CTRL, DAP_SWCLK_GPIO_PORT, DAP_SWCLK_GPIO_PIN, 1); }
__STATIC_INLINE void PIN_SWCLK_TCK_CLR(void)
{ gpio_write_pin(DAP_SWCLK_GPIO_CTRL, DAP_SWCLK_GPIO_PORT, DAP_SWCLK_GPIO_PIN, 0); }

/* ---- SWDIO/TMS ---- */
__STATIC_INLINE uint32_t PIN_SWDIO_TMS_IN(void)
{ return gpio_read_pin(DAP_SWDIO_GPIO_CTRL, DAP_SWDIO_GPIO_PORT, DAP_SWDIO_GPIO_PIN); }
__STATIC_INLINE void PIN_SWDIO_TMS_SET(void)
{ gpio_write_pin(DAP_SWDIO_GPIO_CTRL, DAP_SWDIO_GPIO_PORT, DAP_SWDIO_GPIO_PIN, 1); }
__STATIC_INLINE void PIN_SWDIO_TMS_CLR(void)
{ gpio_write_pin(DAP_SWDIO_GPIO_CTRL, DAP_SWDIO_GPIO_PORT, DAP_SWDIO_GPIO_PIN, 0); }

/* SWDIO 单 bit 读 / 写 (SWD_Sequence 兜底用; 正常走 SPI 传输层) */
__STATIC_INLINE uint32_t PIN_SWDIO_IN(void)
{ return gpio_read_pin(DAP_SWDIO_GPIO_CTRL, DAP_SWDIO_GPIO_PORT, DAP_SWDIO_GPIO_PIN); }
__STATIC_INLINE void PIN_SWDIO_OUT(uint32_t bit)
{ gpio_write_pin(DAP_SWDIO_GPIO_CTRL, DAP_SWDIO_GPIO_PORT, DAP_SWDIO_GPIO_PIN, (uint8_t)(bit & 1u)); }
__STATIC_INLINE void PIN_SWDIO_OUT_ENABLE(void)
{ gpio_set_pin_output(DAP_SWDIO_GPIO_CTRL, DAP_SWDIO_GPIO_PORT, DAP_SWDIO_GPIO_PIN); }
__STATIC_INLINE void PIN_SWDIO_OUT_DISABLE(void)
{ gpio_set_pin_input(DAP_SWDIO_GPIO_CTRL, DAP_SWDIO_GPIO_PORT, DAP_SWDIO_GPIO_PIN); }

/* ---- JTAG 引脚 (F3a 桩) ---- */
__STATIC_INLINE uint32_t PIN_TDI_IN(void)  { return 0u; }
__STATIC_INLINE void     PIN_TDI_OUT(uint32_t bit) { (void)bit; }
__STATIC_INLINE uint32_t PIN_TDO_IN(void)  { return 0u; }
__STATIC_INLINE uint32_t PIN_nTRST_IN(void) { return 0u; }
__STATIC_INLINE void     PIN_nTRST_OUT(uint32_t bit) { (void)bit; }

/* ---- nRESET ---- (PA26, 物理接目标 NRST; pinmux 默认 SPI1_CS_0, 由 swd_nreset_init 覆盖成 GPIO)
 * 标准开漏 (同 J-Link/DAPLink): 复位=输出低拉 NRST; 释放=高阻, 靠**外部上拉电阻**(线上焊
 * 10kΩ 到目标 3.3V)拉高。 目标看门狗/复位键可自行拉低 NRST 而不与探针对顶 (线与)。
 * 注: 若本板 PA26/SPI1_CS_0 有板载弱下拉, 内部上拉不足以拉高, 必须焊外部上拉。 */
__STATIC_INLINE uint32_t PIN_nRESET_IN(void)
{ return gpio_read_pin(DAP_nRESET_GPIO_CTRL, DAP_nRESET_GPIO_PORT, DAP_nRESET_GPIO_PIN); }
__STATIC_INLINE void PIN_nRESET_OUT(uint32_t bit)
{
    if (bit & 1u) {
        /* 释放: 输入高阻 (外部上拉拉高 NRST) */
        gpio_set_pin_input(DAP_nRESET_GPIO_CTRL, DAP_nRESET_GPIO_PORT, DAP_nRESET_GPIO_PIN);
    } else {
        /* 复位: 输出低 (先置 0 再切输出, 避免先输出残留高再翻低的毛刺) */
        gpio_write_pin(DAP_nRESET_GPIO_CTRL, DAP_nRESET_GPIO_PORT, DAP_nRESET_GPIO_PIN, 0);
        gpio_set_pin_output(DAP_nRESET_GPIO_CTRL, DAP_nRESET_GPIO_PORT, DAP_nRESET_GPIO_PIN);
    }
}

/* ---- LED (绑板载 LED 或 no-op) ---- */
__STATIC_INLINE void LED_CONNECTED_OUT(uint32_t bit) { (void)bit; }
__STATIC_INLINE void LED_RUNNING_OUT(uint32_t bit)   { (void)bit; }

/* ---- Reset Target: 硬复位脉冲 (拉低 NRST 一段再释放) ----
 * 返回 1 = 已实现 device-specific reset (使 pyOCD DAP_ResetTarget / connect-under-reset 生效)。
 * STM32 NRST 低脉冲 >=20us 即可; 这里给 1ms 充裕量, 释放后再等 10ms 让目标稳定复位完成。 */
__STATIC_INLINE uint8_t RESET_TARGET(void)
{
    PIN_nRESET_OUT(0);       /* 拉低 NRST */
    board_delay_ms(1);       /* 保持低 (>=20us, 给足 1ms) */
    PIN_nRESET_OUT(1);       /* 释放 (高阻, 目标上拉) */
    board_delay_ms(10);      /* 等目标复位完成 + 稳定 */
    return 1u;
}

/* ---- 时间戳 (TIMESTAMP_CLOCK=0, DAP 报不支持; 给个安全实现) ---- */
__STATIC_INLINE uint32_t TIMESTAMP_GET(void) { return 0u; }

/* ===========================================================================
 *  DAP_Info 字符串 getter (官方 DAP.c 调用; 官方模板把它们放在本头)
 *  拷字符串 (含结尾 NUL) 到 str, 返回长度 (含 NUL); 空串返回 0。
 * ========================================================================= */
#include <string.h>
__STATIC_INLINE uint8_t exlink_dap_copy_str(char *str, const char *src)
{
    if (src == NULL || src[0] == '\0') {
        return 0u;
    }
    uint8_t n = (uint8_t)(strlen(src) + 1u);   /* 含 NUL */
    memcpy(str, src, n);
    return n;
}
__STATIC_INLINE uint8_t DAP_GetVendorString(char *str)  { return exlink_dap_copy_str(str, DAP_VENDOR); }
__STATIC_INLINE uint8_t DAP_GetProductString(char *str) { return exlink_dap_copy_str(str, DAP_PRODUCT); }
__STATIC_INLINE uint8_t DAP_GetSerNumString(char *str)  { return exlink_dap_copy_str(str, DAP_SER_NUM); }
__STATIC_INLINE uint8_t DAP_GetTargetDeviceVendorString(char *str) { (void)str; return 0u; }
__STATIC_INLINE uint8_t DAP_GetTargetDeviceNameString(char *str)   { (void)str; return 0u; }
__STATIC_INLINE uint8_t DAP_GetTargetBoardVendorString(char *str)  { (void)str; return 0u; }
__STATIC_INLINE uint8_t DAP_GetTargetBoardNameString(char *str)    { (void)str; return 0u; }
__STATIC_INLINE uint8_t DAP_GetProductFirmwareVersionString(char *str) { (void)str; return 0u; }

#endif /* __DAP_CONFIG_H__ */
