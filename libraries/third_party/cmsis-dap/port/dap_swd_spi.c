/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink CMSIS-DAP SWD-over-SPI 传输层 (F3a)
 * ============================================================================
 *
 *  实现官方 DAP.c 期望的 3 个 extern (见 arm/DAP.h):
 *      void     SWJ_Sequence (uint32_t count, const uint8_t *data);
 *      void     SWD_Sequence (uint32_t info,  const uint8_t *swdo, uint8_t *swdi);
 *      uint8_t  SWD_Transfer  (uint32_t request, uint32_t *data);
 *  以及 DAP_config.h 委托的 dap_swd_spi_port_setup/off。
 *
 *  技术: 每个 SWD 线相位 = 一次"变长 SPI 帧"。 SPI 固定 Mode0 + LSB-first;
 *  每相位仅改 data_len_in_bits(1..32) 与 trans_mode(write_only/read_only),
 *  靠 mosi_bidir 在写(驱动 SWDIO)/读(释放采样)间切换, 方向切换处即 turnaround。
 *
 *  正确性靠板上逻辑分析仪调 (采样相位 / turnaround 位置); 见计划"验证"节。
 * ============================================================================
 */
#include <string.h>

#include "hpm_soc.h"
#include "hpm_spi_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include "board.h"

#include "dap_swd_spi.h"
#include "dap_swd_spi_priv.h"   /* DAP_TRANSFER / DP / AP 常量 + 内部接口/状态声明 */

/* turnaround 时钟数: F3a 固定 1 (SWD 默认, pyOCD 默认值)。 与 DAP_config.h 同值;
 * 本文件不 #include DAP_config.h (避免引 PIN/GPIO 依赖), 故本地定义。 */
#ifndef DAP_SWD_TURNAROUND
#define DAP_SWD_TURNAROUND          1U
#endif

#define SPI_BASE   EXLINK_DAP_SPI_BASE

/* --- 板级引脚映射 (HPM5300EVK SPI1; 改板只动这一块 + board/pinmux.c + DAP_config.h) ---
 * SWDIO = SPI1 MOSI = PA29 (单线 mosi_bidir 时驱动脚, 需较强驱动 + 弱上拉)。
 * nRESET = SPI1_CS_0 = PA26, 覆盖成 GPIOA[26] 作开漏复位脚。 */
#define SWDIO_PAD               IOC_PAD_PA29
#define NRESET_PAD              IOC_PAD_PA26
#define NRESET_PAD_FUNC_GPIO    IOC_PA26_FUNC_CTL_GPIO_A_26
#define NRESET_GPIO_CTRL        HPM_GPIO0
#define NRESET_GPIO_PORT        GPIO_DO_GPIOA
#define NRESET_GPIO_PIN         26u

/* --- bring-up 运行时可调参数 (msh `swd_cfg` 改, 免重烧录扫参; 声明在 priv 头) --- */
/* 读采样沿: false=CPHA0(Mode0, 上升沿采, SWD 标准) / true=CPHA1(下降沿)。
 * SWD 主从两侧"下降沿驱动/上升沿采", Mode0 (false) 正确; 保留 true 仅供扫参。 */
volatile bool    s_read_even = false;
/* turnaround 时钟数 (1..4), 默认 1。 */
volatile uint8_t s_trn       = DAP_SWD_TURNAROUND;

#if EXLINK_DAP_SWD_TWO_WIRE
/* ===========================================================================
 *  两脚全双工 (EXLINK_DAP_SWD_TWO_WIRE=1): MOSI 恒输出 / MISO 恒输入, 不切方向
 *    MOSI(PA29) 经 ~1K 串阻接 SWDIO (写); MISO(PA28) 直连 SWDIO (读)。
 *    读相位 MOSI 出全 1 (经 1K 弱上拉), 目标 ~50ohm 强驱动盖过, MISO 采真值。
 * ========================================================================= */
static uint32_t swd_xfer_bits(uint32_t mosi_val, uint8_t nbits)
{
    spi_control_config_t ctrl = { 0 };
    uint8_t wbuf[4];
    uint8_t rbuf[4] = { 0 };
    wbuf[0] = (uint8_t)(mosi_val & 0xFFu);
    wbuf[1] = (uint8_t)((mosi_val >> 8) & 0xFFu);
    wbuf[2] = (uint8_t)((mosi_val >> 16) & 0xFFu);
    wbuf[3] = (uint8_t)((mosi_val >> 24) & 0xFFu);

    spi_set_data_bits(SPI_BASE, nbits);
    /* 全双工同步收发: 整相位 = 1 个数据单元 (DATALEN=nbits, ≤32) -> wcount=rcount=1
     * (一个 unit = nbits 个连续时钟)。 */
    ctrl.common_config.trans_mode = spi_trans_write_read_together;
    spi_transfer(SPI_BASE, &ctrl, NULL, NULL, wbuf, 1, rbuf, 1);

    uint32_t v = (uint32_t)rbuf[0] | ((uint32_t)rbuf[1] << 8) |
                 ((uint32_t)rbuf[2] << 16) | ((uint32_t)rbuf[3] << 24);
    if (nbits < 32u) { v &= ((1u << nbits) - 1u); }
    return v;
}

void swd_write_bits(uint32_t val, uint8_t nbits) { (void)swd_xfer_bits(val, nbits); }
uint32_t swd_read_bits(uint8_t nbits) { return swd_xfer_bits(0xFFFFFFFFu, nbits); }

#else  /* ----------------------- 单线半双工 (默认) ----------------------- */
/* ===========================================================================
 *  单线 mosi_bidir (EXLINK_DAP_SWD_TWO_WIRE=0): 仅 MOSI(PA29) 作 SWDIO
 *    write_only: 驱动 SWDIO 输出; read_only: 释放(三态)采样。 方向靠 trans_mode 切。
 * ========================================================================= */
static volatile bool s_drive = true;   /* 当前是否处于驱动(写)态, 减少冗余 mode 切换 */

static void spi_set_drive(bool drive)
{
    if (drive != s_drive) {
        spi_set_transfer_mode(SPI_BASE, drive ? spi_trans_write_only : spi_trans_read_only);
        /* 读相位采样沿可调 (s_read_even); 写恒 CPHA0。 仅改 TRANSFMT.CPHA, 空闲调用安全。 */
        spi_set_clock_phase(SPI_BASE,
            drive ? spi_sclk_sampling_odd_clk_edges
                  : (s_read_even ? spi_sclk_sampling_even_clk_edges
                                 : spi_sclk_sampling_odd_clk_edges));
        s_drive = drive;
    }
}

/* 写 nbits 位 (1..32), LSB-first 驱动 SWDIO。 用官方 spi_transfer 单次启帧。
 * mosi_bidir 已在 init 经 spi_format_init 设入 TRANSFMT (不在 control_config 里)。 */
void swd_write_bits(uint32_t val, uint8_t nbits)
{
    spi_control_config_t ctrl = { 0 };
    uint8_t buf[4];

    spi_set_drive(true);
    spi_set_data_bits(SPI_BASE, nbits);   /* 整相位 = 1 个 DATALEN=nbits 的数据单元 */

    buf[0] = (uint8_t)(val & 0xFFu);
    buf[1] = (uint8_t)((val >> 8) & 0xFFu);
    buf[2] = (uint8_t)((val >> 16) & 0xFFu);
    buf[3] = (uint8_t)((val >> 24) & 0xFFu);

    /* write_only 驱动输出。 wcount=1: 一次事务发 1 个 unit = 恰好 nbits 个连续时钟。 */
    ctrl.common_config.trans_mode = spi_trans_write_only;
    spi_transfer(SPI_BASE, &ctrl, NULL, NULL, buf, 1, NULL, 0);
}

/* 读 nbits 位 (1..32), 释放 SWDIO 采样, 返回低位对齐值。 */
uint32_t swd_read_bits(uint8_t nbits)
{
    spi_control_config_t ctrl = { 0 };
    uint8_t buf[4] = { 0 };

    spi_set_drive(false);
    spi_set_data_bits(SPI_BASE, nbits);   /* 整相位 = 1 个 DATALEN=nbits 的数据单元 */

    /* read_only 释放三态采样。 rcount=1: 读 1 个 unit = 恰好 nbits 个连续时钟。 */
    ctrl.common_config.trans_mode = spi_trans_read_only;
    spi_transfer(SPI_BASE, &ctrl, NULL, NULL, NULL, 0, buf, 1);

    uint32_t v = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                 ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    if (nbits < 32u) { v &= ((1u << nbits) - 1u); }
    return v;
}
#endif /* EXLINK_DAP_SWD_TWO_WIRE */

/* 连续字节流写 (DATALEN=8, 单次事务 wcount=nbytes): SWCLK 全程不停, 无分段间隙。
 * 供线复位 / JTAG->SWD 切换等字节对齐序列; 两脚法亦用 write_only 单向打出 (忽略 MISO)。 */
void swd_write_bytes(const uint8_t *data, uint32_t nbytes)
{
    spi_control_config_t ctrl = { 0 };
#if !EXLINK_DAP_SWD_TWO_WIRE
    spi_set_drive(true);
#endif
    spi_set_data_bits(SPI_BASE, 8);
    ctrl.common_config.trans_mode = spi_trans_write_only;
    while (nbytes > 0u) {
        uint32_t chunk = (nbytes > 256u) ? 256u : nbytes;   /* 防御性分段 (远小于 SoC 上限) */
        spi_transfer(SPI_BASE, &ctrl, NULL, NULL, (uint8_t *)data, chunk, NULL, 0);
        data   += chunk;
        nbytes -= chunk;
    }
}

/* turnaround: 空转 s_trn 拍 (单线=释放采样丢弃 / 两脚=读 1)。 */
void swd_turnaround(void)
{
    (void)swd_read_bits(s_trn ? s_trn : 1u);
}

/* MOSI(PA29=SWDIO) pad: board 的 init_spi_pins 给的是内部下拉 (PS=0), 且驱动偏弱。
 * 单线法里 PA29 是写驱动脚, 这里设较强驱动 (DS=6) + 弱上拉 (PS=1), 保证驱动 SWDIO 到
 * 稳定逻辑电平; 读相位释放(三态)时 SWDIO 由目标内部上拉 + 本弱上拉维持线空闲高。 */
static void swd_pad_pullup(void)
{
    HPM_IOC->PAD[SWDIO_PAD].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(6)
                                    | IOC_PAD_PAD_CTL_PE_SET(1)
                                    | IOC_PAD_PAD_CTL_PS_SET(1);
}

/* nRESET(PA26): pinmux 默认把 PA26 配成 SPI1_CS_0, 但单线 SWD 不用硬件 CS。 这里覆盖成 GPIO
 * 作 nRESET (物理已接目标 NRST)。 标准开漏: 复位=输出低拉 NRST; 释放=输入高阻, 靠**外部
 * 上拉电阻**(线上焊 10kΩ 到目标 3.3V)拉高。 与 J-Link/DAPLink 一致, 目标可自复位不对顶。
 * 注: 若本板 PA26/CS_0 有板载弱下拉, 内部上拉不足, 必须靠外部上拉; 初态 = 高阻(释放)。 */
static void swd_nreset_init(void)
{
    HPM_IOC->PAD[NRESET_PAD].FUNC_CTL = NRESET_PAD_FUNC_GPIO;
    HPM_IOC->PAD[NRESET_PAD].PAD_CTL  = IOC_PAD_PAD_CTL_DS_SET(6)
                                      | IOC_PAD_PAD_CTL_PE_SET(1)
                                      | IOC_PAD_PAD_CTL_PS_SET(1);   /* 内部弱上拉(辅助; 主要靠外部上拉) */
    gpio_set_pin_input(NRESET_GPIO_CTRL, NRESET_GPIO_PORT, NRESET_GPIO_PIN);   /* 释放态 (高阻) */
}

/* nRESET 脉冲 (诊断/通用): 拉低 NRST low_ms 毫秒再释放(高阻), 返回释放后读回电平 (1=高)。
 * 供 msh swd_rst 用; 也可被上层复位流程调用。 标准开漏: 释放靠外部上拉 (线上 10kΩ 到目标 3.3V)。 */
uint8_t dap_swd_nreset_pulse(uint32_t low_ms)
{
    swd_nreset_init();                                   /* PA26 GPIO 开漏, 初态高阻 */
    gpio_write_pin(NRESET_GPIO_CTRL, NRESET_GPIO_PORT, NRESET_GPIO_PIN, 0);    /* 复位: 输出低 */
    gpio_set_pin_output(NRESET_GPIO_CTRL, NRESET_GPIO_PORT, NRESET_GPIO_PIN);
    board_delay_ms(low_ms ? low_ms : 1u);
    gpio_set_pin_input(NRESET_GPIO_CTRL, NRESET_GPIO_PORT, NRESET_GPIO_PIN);   /* 释放: 高阻 (外部上拉拉高) */
    board_delay_ms(2u);
    return gpio_read_pin(NRESET_GPIO_CTRL, NRESET_GPIO_PORT, NRESET_GPIO_PIN);
}

/* nRESET(PA26) 停在固定电平 (GPIO 推挽), 便于万用表量真实电压 (类比 swd_drive 之于 PA29)。
 *   lvl=0 输出低 / lvl 非0 输出高。 留在 GPIO 输出态, 量完需 reset 或重 init 恢复。
 *   注: 正常 nRESET 释放应是高阻, 这里为方便量电压用推挽强驱动。 */
void dap_swd_nreset_park(uint8_t lvl)
{
    HPM_IOC->PAD[NRESET_PAD].FUNC_CTL = NRESET_PAD_FUNC_GPIO;
    HPM_IOC->PAD[NRESET_PAD].PAD_CTL  = IOC_PAD_PAD_CTL_DS_SET(7);   /* 最强驱动 */
    gpio_write_pin(NRESET_GPIO_CTRL, NRESET_GPIO_PORT, NRESET_GPIO_PIN, lvl ? 1u : 0u);
    gpio_set_pin_output(NRESET_GPIO_CTRL, NRESET_GPIO_PORT, NRESET_GPIO_PIN);
}

static inline uint32_t parity32(uint32_t v)
{
    v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
    return v & 1u;
}

/* ===========================================================================
 *  初始化 / 端口建立
 * ========================================================================= */
void dap_swd_spi_init(void)
{
    spi_timing_config_t timing = { 0 };
    spi_format_config_t fmt = { 0 };

    /* 引脚: SCLK=PA27 / MOSI=PA29(单线 SWDIO) / MISO=PA28(两脚模式采样); board 侧 pinmux */
    init_spi_pins(SPI_BASE);
    swd_pad_pullup();   /* PA29(MOSI=SWDIO) 强驱动 + 弱上拉 */
    swd_nreset_init();  /* PA26 覆盖成 GPIO 作 nRESET, 初态释放(高阻) */

    /* 时钟: 源频取板级, sclk 设 1MHz bring-up */
    timing.master_config.clk_src_freq_in_hz = board_init_spi_clock(SPI_BASE);
    timing.master_config.sclk_freq_in_hz = EXLINK_DAP_SWCLK_HZ_DEFAULT;
    spi_master_timing_init(SPI_BASE, &timing);

    /* 格式: Mode0 (cpol=0 空闲低, cpha=0 上升沿采样), LSB-first。
     * mosi_bidir 随接法宏切换: 单线=true(MOSI 双向三态) / 两脚=false(全双工 MOSI+MISO)。 */
    fmt.common_config.data_len_in_bits = 8;     /* 默认 8, 各相位运行时改 */
    fmt.common_config.data_merge = false;
#if EXLINK_DAP_SWD_TWO_WIRE
    fmt.common_config.mosi_bidir = false;       /* 两脚全双工: MOSI 恒输出 + MISO 恒输入 */
#else
    fmt.common_config.mosi_bidir = true;        /* 单线 SWDIO 双向 (mosi_bidir 半双工) */
#endif
    fmt.common_config.lsb = true;               /* SWD LSB-first */
    fmt.common_config.mode = spi_master_mode;
    fmt.common_config.cpol = spi_sclk_low_idle;
    fmt.common_config.cpha = s_read_even ? spi_sclk_sampling_even_clk_edges
                                         : spi_sclk_sampling_odd_clk_edges;
    spi_format_init(SPI_BASE, &fmt);

#if !EXLINK_DAP_SWD_TWO_WIRE
    s_drive = true;
    spi_set_transfer_mode(SPI_BASE, spi_trans_write_only);   /* 单线: 起始驱动态 */
#endif
}

void dap_swd_spi_set_clock(uint32_t hz)
{
    spi_timing_config_t timing = { 0 };
    timing.master_config.clk_src_freq_in_hz = board_init_spi_clock(SPI_BASE);
    if (hz == 0u || hz > timing.master_config.clk_src_freq_in_hz) {
        hz = timing.master_config.clk_src_freq_in_hz;
    }
    timing.master_config.sclk_freq_in_hz = hz;
    spi_master_timing_init(SPI_BASE, &timing);
}

/* DAP_config.h 委托: 进入 SWD 模式 / 关闭 */
void dap_swd_spi_port_setup(void)
{
    init_spi_pins(SPI_BASE);
    swd_pad_pullup();
    swd_nreset_init();   /* PA26 覆盖成 GPIO 作 nRESET, 初态释放(高阻) */
#if !EXLINK_DAP_SWD_TWO_WIRE
    s_drive = true;
    spi_set_transfer_mode(SPI_BASE, spi_trans_write_only);   /* 单线: 进驱动态 */
#endif
}

void dap_swd_spi_port_off(void)
{
#if EXLINK_DAP_SWD_TWO_WIRE
    /* 全双工: 无方向态可切; SWDIO 空闲由目标内部上拉 + MOSI 出 1 维持。 */
#else
    spi_set_transfer_mode(SPI_BASE, spi_trans_read_only);    /* 单线: 置读(释放)态 */
    s_drive = false;
#endif
}

/* ===========================================================================
 *  官方 DAP.c 期望的 3 个传输函数
 * ========================================================================= */

/* SWJ_Sequence: 发 count 位 (LSB-first), 主机驱动 SWDIO。 用于线复位 / JTAG->SWD 切换。 */
void SWJ_Sequence(uint32_t count, const uint8_t *data)
{
    /* 整字节部分一次连续打出 (SWCLK 不断); 末尾不足一字节的余位单独发。
     * 线复位 / 切换均字节对齐 -> 走单次事务: 56 拍线复位 = 一段连续 56 时钟。 */
    uint32_t full = count >> 3;
    uint8_t  rem  = (uint8_t)(count & 7u);
    if (full > 0u) {
        swd_write_bytes(data, full);
    }
    if (rem > 0u) {
        swd_write_bits(data[full], rem);
    }
}

/* SWD_Sequence: 通用时钟序列。 info bit7=输入(1)/输出(0), 低6位=时钟数(0 表示 64)。 */
void SWD_Sequence(uint32_t info, const uint8_t *swdo, uint8_t *swdi)
{
    uint32_t n = info & 0x3Fu;
    if (n == 0u) {
        n = 64u;
    }

    if (info & 0x80u) {
        /* 输入: 读 n 位到 swdi (LSB-first, 按字节填) */
        uint32_t got = 0, idx = 0;
        uint32_t remain = n;
        while (remain > 0u) {
            uint8_t m = (remain >= 32u) ? 32u : (uint8_t)remain;
            got = swd_read_bits(m);
            for (uint8_t b = 0; b < m; b += 8u) {
                swdi[idx++] = (uint8_t)((got >> b) & 0xFFu);
            }
            remain -= m;
        }
    } else {
        /* 输出: 发 n 位 (同 SWJ) */
        uint32_t idx = 0, remain = n;
        while (remain > 0u) {
            uint8_t m = (remain >= 32u) ? 32u : (uint8_t)remain;
            uint32_t word = 0;
            for (uint8_t b = 0; b < m; b += 8u) {
                word |= (uint32_t)swdo[idx++] << b;
            }
            swd_write_bits(word, m);
            remain -= m;
        }
    }
}

/* SWD_Transfer: 一次 SWD 读/写事务。 request 低 4 位 = APnDP/RnW/A2/A3。
 * 读: *data 收 32 位; 写: 发 *data。 返回 ACK (DAP_TRANSFER_OK/WAIT/FAULT [| ERROR])。 */
uint8_t SWD_Transfer(uint32_t request, uint32_t *data)
{
    uint32_t ack;
    uint32_t parity;
    uint8_t  req_byte;

    /* ---- 请求相位 (8 bit, 主机驱动) ----
     * 位序 (LSB-first 上线): Start=1, APnDP, RnW, A2, A3, Parity, Stop=0, Park=1
     */
    parity = (request >> 0) ^ (request >> 1) ^ (request >> 2) ^ (request >> 3);
    parity &= 1u;
    req_byte = (uint8_t)(0x81u                              /* Start=1(bit0), Park=1(bit7) */
                | ((request & 0x0Fu) << 1)                  /* APnDP/RnW/A2/A3 -> bit1..4 */
                | (parity << 5));                            /* Parity -> bit5; Stop=0(bit6) */
    swd_write_bits(req_byte, 8);

    /* ---- turnaround (主机->目标) ---- */
    swd_turnaround();

    /* ---- ACK (3 bit, 目标驱动) ---- */
    ack = swd_read_bits(3);

    if (ack == DAP_TRANSFER_OK) {
        if (request & DAP_TRANSFER_RnW) {
            /* 读: 32 数据 + 1 parity (目标继续驱动), 然后 turnaround 回主机 */
            uint32_t val = swd_read_bits(32);
            uint32_t par = swd_read_bits(1);
            swd_turnaround();
            if (parity32(val) != (par & 1u)) {
                ack |= DAP_TRANSFER_ERROR;  /* parity 错 */
            }
            if (data) {
                *data = val;
            }
        } else {
            /* 写: turnaround 回主机, 再发 32 数据 + 1 parity */
            swd_turnaround();
            uint32_t val = data ? *data : 0u;
            swd_write_bits(val, 32);
            swd_write_bits(parity32(val), 1);
        }
        return (uint8_t)ack;
    }

    /* WAIT / FAULT: 协议要求把 data 相位的时钟补齐 (dummy), 再 turnaround。
     * 简化: 读方向已无需补; 这里统一做一次 turnaround。 */
    swd_turnaround();
    return (uint8_t)ack;
}

/* ===========================================================================
 *  ADIv5 寄存器访问层 (在 SWD_Transfer 之上) —— 仅依赖 SWD_Transfer + DAP_TRANSFER_*。
 *  request = [APnDP][RnW] | (offset & 0x0C); A2/A3 恰对齐寄存器 offset[3:2]。
 * ========================================================================= */

/* WAIT 重试包装: 目标回 WAIT (AP busy) 时重发, 至多 SWD_WAIT_RETRIES 次。 */
static uint8_t swd_xfer_retry(uint32_t request, uint32_t *data)
{
    uint8_t ack;
    int tries = SWD_WAIT_RETRIES;
    do {
        ack = SWD_Transfer(request, data);
    } while (((ack & 7u) == DAP_TRANSFER_WAIT) && (--tries > 0));
    return ack;
}

/* DP SELECT 缓存: 记上次写入值, 命中则免重写。 连接(line reset)后必置无效。
 * (定义在此, 声明在 priv 头; bring-up 诊断的 swd_powerup_prologue 会改它。) */
uint32_t s_sel_cache;

/* 确保 SELECT.APBANKSEL = apreg[7:4] (APSEL=0, DPBANKSEL=0); 命中缓存则跳过。 */
static uint8_t swd_select_ap(uint8_t apreg)
{
    uint32_t sel = (uint32_t)(apreg & 0xF0u);          /* APBANKSEL = apreg[7:4] */
    if (sel != s_sel_cache) {
        uint8_t ack = swd_xfer_retry(DP_SELECT & 0x0Cu, &sel);   /* req 0x8 (write) */
        if ((ack & 7u) == DAP_TRANSFER_OK) {
            s_sel_cache = sel;
        }
        return ack;
    }
    return DAP_TRANSFER_OK;
}

/* DP 读/写 (DP 读非 posted, 同事务即得值)。 */
uint8_t dp_read(uint8_t r, uint32_t *out)
{
    return swd_xfer_retry(DAP_TRANSFER_RnW | (uint32_t)(r & 0x0Cu), out);
}
uint8_t dp_write(uint8_t r, uint32_t v)
{
    return swd_xfer_retry((uint32_t)(r & 0x0Cu), &v);
}

/* AP 写: 先对齐 APBANKSEL 再写 AP 寄存器。 */
uint8_t ap_write(uint8_t apreg, uint32_t v)
{
    uint8_t ack = swd_select_ap(apreg);
    if ((ack & 7u) != DAP_TRANSFER_OK) {
        return ack;
    }
    return swd_xfer_retry(DAP_TRANSFER_APnDP | (uint32_t)(apreg & 0x0Cu), &v);
}

/* AP 读 (posted/流水线): AP 读事务返回的是"上一次" AP 读的值, 真值要下一次事务取。
 * 序: 选 bank -> 发 AP 读(丢弃) -> 读 DP RDBUFF 拿真值。 此序不可省。 */
uint8_t ap_read(uint8_t apreg, uint32_t *out)
{
    uint32_t dummy = 0;
    uint8_t ack = swd_select_ap(apreg);
    if ((ack & 7u) != DAP_TRANSFER_OK) {
        return ack;
    }
    ack = swd_xfer_retry(DAP_TRANSFER_APnDP | DAP_TRANSFER_RnW |
                         (uint32_t)(apreg & 0x0Cu), &dummy);   /* posted, 丢弃 */
    if ((ack & 7u) != DAP_TRANSFER_OK) {
        return ack;
    }
    return swd_xfer_retry(DAP_TRANSFER_RnW | (uint32_t)(DP_RDBUFF & 0x0Cu), out); /* req 0xE 真值 */
}

/* ===========================================================================
 *  标准 ADIv5 JTAG->SWD 连接序列 (pyOCD/OpenOCD 同款 2-reset) —— 整条拼成单一字节流,
 *  一次 spi_transfer 连续打出 (SWCLK 全程不停, 段间无缝)。 LSB-first,
 *  共 56+16+56+8 = 136 bit = 17 byte:
 *    ① line reset 56 (SWDIO=1)  ② JTAG->SWD 0xE79E(16)  ③ line reset 56 (switch 后)
 *    ④ idle 8 (SWDIO=0, 读前 framing)
 *  协议要点: 每个 line reset(连续高)后必须有 idle 下降沿(SWDIO 拉低)标记 reset 结束,
 *  DP 才离开 reset 态; 末段 idle 低 -> 紧接 DPIDR 读的 Start=1 形成清晰 低->高 边沿。 */
static const uint8_t k_swd_connect[17] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   /* ① line reset 56 (SWDIO=1) */
    0x9E, 0xE7,                                 /* ② JTAG->SWD 0xE79E (LSB-first) */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   /* ③ line reset 56 (switch 后) */
    0x00,                                       /* ④ idle 8 (SWDIO=0, 读前 framing) */
};

void swd_send_connect(void)
{
    swd_write_bytes(k_swd_connect, sizeof(k_swd_connect));   /* 单次连续事务 */
}

