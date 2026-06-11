/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink CMSIS-DAP SWD-over-SPI —— bring-up 诊断 msh 命令 (可整体删除)
 * ============================================================================
 *
 *  联调脚手架: 在固件侧直接跑 SWD 连接/上电/读内存、量引脚电平、判 SPI 三态,
 *  绕开 pyOCD / USB / 逻辑分析仪。 产品态 ADIv5 走上位机 CMSIS-DAP, 联调完本文件
 *  连同 SConscript 里的一行、dap_swd_spi_priv.h 中仅为其暴露的符号, 可一并移除。
 *
 *  命令: dap_tx swd_cfg swd_id swd_read swd_pwr swd_mem swd_seq swd_raw
 *        swd_pin swd_drive swd_clk swd_rdtest swd_trit
 *  传输 / 协议 / 寄存器访问层在 dap_swd_spi.c; 跨文件共享符号在 dap_swd_spi_priv.h。
 * ============================================================================
 */
#include <rtthread.h>          /* rt_kprintf, rt_thread_mdelay, MSH_CMD_EXPORT */
#include <stdlib.h>            /* atoi (swd_cfg/drive/clk), strtoul (swd_mem) */

#include "hpm_soc.h"           /* HPM_IOC, IOC_PAD_PA29/PA27, IOC_PExx_FUNC_CTL_*, HPM_GPIO0 */
#include "hpm_gpio_drv.h"      /* gpio_set_pin_input/output, gpio_read_pin/write_pin, GPIO_DO_GPIOA */
#include "board.h"             /* board pin/SoC glue (与传输层一致) */

#include "dap_swd_spi.h"       /* dap_swd_spi_init / dap_swd_spi_set_clock + EXLINK_* */
#include "dap_swd_spi_priv.h"  /* 暴露的低层位 I/O + dp_/ap_ + s_* 状态 + ADIv5/DAP_TRANSFER 常量 */

/* ===========================================================================
 *  自测: msh `dap_tx [n]` —— 直接在 SPI1 上连续打方波,
 *  把"SWD 突发"变成"连续输出", 方便逻辑分析仪/示波器/万用表抓 PA27(SCLK)/PA29(SWDIO)。
 *  抓得到方波 -> SPI 外设+pad OK; 抓不到 -> SPI1 外设没真输出 (时钟/pinmux/实例问题)。
 * ========================================================================= */
static int dap_tx(int argc, char **argv)
{
    int forever = 0;
    if (argc > 1 && (argv[1][0] == 'f' || argv[1][0] == 'F')) {
        forever = 1;   /* dap_tx f  -> 无限发, 直到复位; 方便慢时基/示波器抓 */
    }

    /* 确保 SPI 已初始化 + 进驱动(写)态 */
    dap_swd_spi_init();

    if (forever) {
        rt_kprintf("dap_tx: FOREVER 0x55 on SPI1 (SCLK=PA27, MOSI/SWDIO=PA29). Reset board to stop.\n");
        for (;;) {
            swd_write_bits(0x55u, 8);
        }
        /* unreachable */
    }

    const int n = 2000000;   /* ~16s @1MHz, 覆盖 2s/div 的窗口 */
    rt_kprintf("dap_tx: writing %d frames of 0x55 (~16s) on SPI1 (SCLK=PA27, MOSI/SWDIO=PA29)...\n", n);
    /* 0x55 = 0101_0101: 8bit 帧产生连续翻转, 便于看波形 */
    for (int i = 0; i < n; i++) {
        swd_write_bits(0x55u, 8);
    }
    rt_kprintf("dap_tx: done.\n");
    return 0;
}
MSH_CMD_EXPORT(dap_tx, SPI1 SWD self-test: continuous square wave on PA27/PA29);

/* ===========================================================================
 *  bring-up 地面真值: 固件自己跑 SWD 连接 + 读 DPIDR, 直接打印 ACK/IDCODE。
 *  绕开 pyOCD 与 LA 解码, 用于扫采样沿 / turnaround / 时钟。
 *
 *    swd_cfg [read_cpha] [trn] [khz]   设参数 (read_cpha: 0=上升沿读 1=下降沿读)
 *    swd_id                            连接并读 DPIDR; 成功例: STM32F103 = 0x1BA01477
 *
 *  典型扫法: swd_cfg 0 1 100  -> swd_id ; 不行则 swd_cfg 1 1 100 -> swd_id ;
 *            再试 trn=2: swd_cfg 0 2 100 / swd_cfg 1 2 100 ...
 * ========================================================================= */
static int swd_cfg(int argc, char **argv)
{
    if (argc >= 2) s_read_even = (atoi(argv[1]) != 0);
    if (argc >= 3) { int t = atoi(argv[2]); if (t >= 1 && t <= 4) s_trn = (uint8_t)t; }
    if (argc >= 4) { int k = atoi(argv[3]); if (k > 0) dap_swd_spi_set_clock((uint32_t)k * 1000u); }
    rt_kprintf("swd_cfg: read_cpha=%d (%s)  trn=%d  (khz only if given)\n",
               s_read_even, s_read_even ? "falling/even" : "rising/odd", s_trn);
    return 0;
}
MSH_CMD_EXPORT(swd_cfg, set SWD read-edge/turnaround/clock: swd_cfg [cpha0|1] [trn1-4] [khz]);

static int swd_id(int argc, char **argv)
{
    (void)argc; (void)argv;

    dap_swd_spi_init();   /* 重置 SPI 到当前参数态 */

    swd_send_connect();   /* 连续连接序列 (单事务, 对齐 J-Link), 末尾 idle 已就绪 */

    /* 读 DPIDR (ReadDP 0x0): 连接后第一个事务, 把 DP 拉出 line-reset */
    uint32_t data = 0;
    uint8_t  ack  = SWD_Transfer(DAP_TRANSFER_RnW, &data);

    const char *as = (ack == DAP_TRANSFER_OK)   ? "OK"
                   : (ack == DAP_TRANSFER_WAIT)  ? "WAIT"
                   : (ack == DAP_TRANSFER_FAULT) ? "FAULT"
                   : (ack & DAP_TRANSFER_ERROR)  ? "OK+PARITY_ERR" : "???";
    rt_kprintf("swd_id: cfg{read_cpha=%d trn=%d}  ACK=0x%X (%s)  DPIDR=0x%08X\n",
               s_read_even, s_trn, ack, as, data);
    if (ack == DAP_TRANSFER_OK && data != 0u && data != 0xFFFFFFFFu) {
        rt_kprintf("swd_id: >>> LINK UP, DPIDR looks valid <<<\n");
    }
    return 0;
}
MSH_CMD_EXPORT(swd_id, SWD bring-up: connect + read DPIDR raw (bypass pyOCD/LA));

/* swd_freq [mhz]: 设 SWCLK 频率 (默认 10) + 自测连接读 DPIDR, 用于扫 5361 SWCLK 上限。
 *   先打印 SPI1 时钟源频率 (SCLK 硬件上限 = src/2), 再设频, 再读 DPIDR 验该频率可用。
 *   读对 DPIDR=0x1BA01477 -> 该频率 OK; 错 -> 超频或采样失稳, 退回上一档。
 *   典型扫法 (接目标): swd_freq 10 -> swd_freq 20 -> swd_freq 30 ... 直到读错, 定上限。 */
static int swd_freq(int argc, char **argv)
{
    uint32_t mhz = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 10u;
    if (mhz == 0u) { mhz = 1u; }
    uint32_t hz = mhz * 1000000u;

    uint32_t src = board_init_spi_clock(EXLINK_DAP_SPI_BASE);   /* SPI1 时钟源 */
    rt_kprintf("swd_freq: SPI1 clk src = %u Hz (SCLK max ~= src/2 = %u Hz)\n",
               src, src / 2u);

    dap_swd_spi_set_clock(hz);     /* 设频 (内部超 src 会 clamp 到 src) */
    dap_swd_spi_init();            /* 重置 SPI 到当前参数态 (含新频率) */
    dap_swd_spi_set_clock(hz);     /* init 会用 DEFAULT 复位频率, 这里再设回请求值 */

    swd_send_connect();
    uint32_t data = 0;
    uint8_t  ack  = SWD_Transfer(DAP_TRANSFER_RnW, &data);

    rt_kprintf("swd_freq: req %u MHz  ACK(3bit)=0x%X  DPIDR=0x%08X -> %s\n",
               mhz, ack & 7u, data,
               ((ack & 7u) == DAP_TRANSFER_OK && data == 0x1BA01477u) ? "OK (freq usable)"
             : ((ack & 7u) == DAP_TRANSFER_OK) ? "ACK OK but DPIDR unexpected (check target)"
             : "FAIL (overclock/sampling unstable, back off)");
    return 0;
}
MSH_CMD_EXPORT(swd_freq, set SWCLK freq MHz + selftest read DPIDR to sweep 5361 ceiling: swd_freq [mhz]);

/* swd_read: 连接序列 -> 读 DPIDR, 重点报告 ACK (目标到底有没有回应)。
 * bring-up 第二步 —— swd_seq 波形确认 OK 后, 接目标板跑本命令:
 *   ACK=OK         -> 目标在线且回应, DPIDR 应 = 0x1BA01477 (STM32F103)
 *   ACK=WAIT/FAULT -> 目标在线 (协议已通), 只是该次事务忙/错
 *   ACK=其它(常 0)  -> 目标没驱动 ACK: 没接好 / 单线三态对顶 / 采样沿; 跑 swd_trit 查三态
 * (与 swd_id 功能等价; swd_id 偏 IDCODE 判定, 本命令偏 ACK 诊断。) */
static int swd_read(int argc, char **argv)
{
    (void)argc; (void)argv;
    uint32_t data = 0;
    uint8_t  ack;

    dap_swd_spi_init();      /* 复位 SPI 到当前 swd_cfg 参数态 */
    swd_send_connect();      /* ① 标准 2-reset 连接序列 (连续单事务) */
    ack = SWD_Transfer(DAP_TRANSFER_RnW, &data);   /* ② 读 DPIDR; 内部 req 后 turnaround 切 read */

    rt_kprintf("swd_read: cfg{read_cpha=%d trn=%d}  ACK(3bit)=0x%X -> ",
               s_read_even, s_trn, ack & 7u);
    switch (ack & 7u) {
    case DAP_TRANSFER_OK:
        rt_kprintf("OK  DPIDR=0x%08X%s\n", data,
                   (ack & DAP_TRANSFER_ERROR) ? "  (PARITY ERR!)" : "");
        if (!(ack & DAP_TRANSFER_ERROR) && data != 0u && data != 0xFFFFFFFFu) {
            rt_kprintf("swd_read: >>> target ACKed and DPIDR valid, LINK UP <<<\n");
        }
        break;
    case DAP_TRANSFER_WAIT:  rt_kprintf("WAIT (target online, busy this xfer)\n");  break;
    case DAP_TRANSFER_FAULT: rt_kprintf("FAULT (target online, error)\n");   break;
    default:
        rt_kprintf("no valid ACK -> target not driving (bad wire/tri-state clash/sample edge). run swd_trit.\n");
        break;
    }
    return 0;
}
MSH_CMD_EXPORT(swd_read, connect + read DPIDR focus ACK (target alive? STM32F103=1BA01477));

/* ===========================================================================
 *  ADIv5 bring-up 自测: 上电握手 + AP IDR (swd_pwr) / MEM-AP 读内存 (swd_mem)。
 *  在固件侧把"连接 -> 上电 -> 访问 AP -> 读内存"整条跑通, 不依赖 pyOCD/USB。
 *  产品态 ADIv5 仍走上位机; 这两条与 swd_id/swd_read 同属可删的 bring-up 命令。
 * ========================================================================= */

/* ack(3bit) 译文 (与 swd_read 同风格) */
static const char *ack_str(uint8_t ack)
{
    switch (ack & 7u) {
    case DAP_TRANSFER_OK:    return "OK";
    case DAP_TRANSFER_WAIT:  return "WAIT";
    case DAP_TRANSFER_FAULT: return "FAULT";
    default:                 return "NO-ACK";
    }
}

/* 上电 prologue (步骤 1-6): 连接 -> DPIDR -> ABORT 清 sticky -> SELECT=0 ->
 * CTRL/STAT 请求上电 -> 轮询 ACK。 返回 DAP_TRANSFER_OK 表示整段成功; 逐步打印。
 * dpidr_out 可为 NULL。 */
static uint8_t swd_powerup_prologue(uint32_t *dpidr_out)
{
    uint32_t v = 0;
    uint8_t  ack;
    int      tries;

    /* ① 连接 (line reset 会清目标 SELECT -> 缓存置无效) */
    dap_swd_spi_init();
    swd_send_connect();
    s_sel_cache = 0xFFFFFFFFu;

    /* ② 读 DPIDR (把 DP 拉出 line-reset; 必须是连接后第一个事务) */
    ack = dp_read(DP_IDCODE, &v);
    rt_kprintf("  [DPIDR ] ACK=%s  0x%08X\n", ack_str(ack), v);
    if (dpidr_out) { *dpidr_out = v; }
    if ((ack & 7u) != DAP_TRANSFER_OK) { return ack; }

    /* ③ ABORT = 0x1E 清全部 sticky (清上次遗留, 防 FAULT 卡死) */
    ack = dp_write(DP_ABORT, ABORT_CLR_ALL);
    rt_kprintf("  [ABORT ] ACK=%s\n", ack_str(ack));
    if ((ack & 7u) != DAP_TRANSFER_OK) { return ack; }

    /* ④ SELECT = 0 (DPBANKSEL=0 保证 CTRL/STAT 是真 CTRL/STAT; APBANKSEL=0/APSEL=0) */
    ack = dp_write(DP_SELECT, 0u);
    s_sel_cache = 0u;
    rt_kprintf("  [SELECT] ACK=%s  (=0)\n", ack_str(ack));
    if ((ack & 7u) != DAP_TRANSFER_OK) { return ack; }

    /* ⑤ CTRL/STAT 请求调试+系统域上电 */
    ack = dp_write(DP_CTRL_STAT, CTRL_PWRUP_REQ);
    rt_kprintf("  [CTRLwr] ACK=%s  (req 0x%08X)\n", ack_str(ack), (unsigned)CTRL_PWRUP_REQ);
    if ((ack & 7u) != DAP_TRANSFER_OK) { return ack; }

    /* ⑥ 轮询直到 CDBGPWRUPACK & CSYSPWRUPACK 都置位 */
    for (tries = SWD_PWRUP_RETRIES; tries > 0; tries--) {
        ack = dp_read(DP_CTRL_STAT, &v);
        if ((ack & 7u) != DAP_TRANSFER_OK) {
            rt_kprintf("  [CTRLrd] ACK=%s  (poll abort)\n", ack_str(ack));
            return ack;
        }
        if ((v & CTRL_PWRUP_ACK) == CTRL_PWRUP_ACK) { break; }
        rt_thread_mdelay(1);
    }
    rt_kprintf("  [CTRLrd] CTRL/STAT=0x%08X  PWRUP_ACK=%s\n",
               v, ((v & CTRL_PWRUP_ACK) == CTRL_PWRUP_ACK) ? "YES" : "NO(timeout)");
    if ((v & CTRL_PWRUP_ACK) != CTRL_PWRUP_ACK) { return DAP_TRANSFER_WAIT; }

    return DAP_TRANSFER_OK;
}

/* swd_pwr: 上电握手 + 读 AP IDR (验证 MEM-AP 存在)。 STM32F1 AHB-AP IDR=0x24770011。 */
static int swd_pwr(int argc, char **argv)
{
    (void)argc; (void)argv;
    uint32_t idr = 0;
    uint8_t  ack;

    rt_kprintf("swd_pwr: cfg{read_cpha=%d trn=%d} ADIv5 power-up:\n", s_read_even, s_trn);
    if ((swd_powerup_prologue(NULL) & 7u) != DAP_TRANSFER_OK) {
        rt_kprintf("swd_pwr: power-up FAILED (see stuck step above)\n");
        return 0;
    }
    ack = ap_read(AP_IDR, &idr);
    rt_kprintf("  [AP IDR] ACK=%s  IDR=0x%08X\n", ack_str(ack), idr);
    if ((ack & 7u) == DAP_TRANSFER_OK && idr != 0u && idr != 0xFFFFFFFFu) {
        rt_kprintf("swd_pwr: >>> debug domain powered + AP online, MEM-AP READY <<<\n");
    }
    return 0;
}
MSH_CMD_EXPORT(swd_pwr, ADIv5 debug power-up + read AP IDR (STM32F1 AHB-AP=24770011));

/* swd_mem [addr]: 上电 -> MEM-AP 读一个 32-bit 字。 默认 0xE0042000 (DBGMCU_IDCODE,
 * 低16位=DEV_ID, F105/F107=0x418)。 带参覆盖地址 (支持 0x 前缀)。 */
static int swd_mem(int argc, char **argv)
{
    uint32_t addr = (argc >= 2) ? (uint32_t)strtoul(argv[1], NULL, 0) : 0xE0042000u;
    uint32_t val  = 0;
    uint8_t  ack;

    rt_kprintf("swd_mem: cfg{read_cpha=%d trn=%d} read [0x%08X]:\n", s_read_even, s_trn, addr);
    if ((swd_powerup_prologue(NULL) & 7u) != DAP_TRANSFER_OK) {
        rt_kprintf("swd_mem: power-up FAILED\n");
        return 0;
    }

    /* MEM-AP: CSW(32bit) -> TAR(地址) -> DRW(读, posted->RDBUFF) */
    ack = ap_write(AP_CSW, CSW_VALUE_32);
    if ((ack & 7u) != DAP_TRANSFER_OK) { rt_kprintf("  [CSW   ] ACK=%s (abort)\n", ack_str(ack)); return 0; }
    ack = ap_write(AP_TAR, addr);
    if ((ack & 7u) != DAP_TRANSFER_OK) { rt_kprintf("  [TAR   ] ACK=%s (abort)\n", ack_str(ack)); return 0; }
    ack = ap_read(AP_DRW, &val);
    rt_kprintf("  [DRW   ] ACK=%s\n", ack_str(ack));
    rt_kprintf("swd_mem: [0x%08X] = 0x%08X  (ACK=%s)\n", addr, val, ack_str(ack));
    if ((ack & 7u) == DAP_TRANSFER_OK && addr == 0xE0042000u) {
        rt_kprintf("swd_mem: DBGMCU DEV_ID=0x%03X (F105/F107=0x418)\n", val & 0xFFFu);
    }
    return 0;
}
MSH_CMD_EXPORT(swd_mem, ADIv5 MEM-AP read one 32-bit word: swd_mem [addr] (def DBGMCU 0xE0042000));

/* 发标准连接序列 (无读事务), 整条连续 SWCLK 单次打出, 供 LA 抓波形对齐 J-Link。
 * 解码应见: line reset56 / JTAG->SWD(E79E) / line reset56 / idle8, 段间无缝 (一次 spi_transfer)。
 *   swd_seq     默认只发一次 (LA 单次触发后再跑本命令)
 *   swd_seq f   无限重发 (轮间 32clk 低分隔), 供示波器自由运行 */
static int swd_seq(int argc, char **argv)
{
    int forever = (argc > 1 && (argv[1][0] == 'f' || argv[1][0] == 'F'));
    static const uint8_t idle_sep[4] = { 0, 0, 0, 0 };   /* 32 clk 低: 轮间分隔 (长于中段 idle8) */

    dap_swd_spi_init();
    if (forever) {
        rt_kprintf("swd_seq: FOREVER continuous connect-seq on PA27/PA29 (idle-separated). Reset to stop.\n");
        for (;;) {
            swd_send_connect();
            swd_write_bytes(idle_sep, sizeof(idle_sep));
        }
        /* unreachable */
    }
    rt_kprintf("swd_seq: ONE-SHOT connect-seq (17B, continuous SWCLK, no read) on PA27(SWCLK)/PA29(SWDIO).\n");
    swd_send_connect();
    rt_kprintf("swd_seq: sent once. Re-arm LA single-trigger and re-run to recapture.\n");
    return 0;
}
MSH_CMD_EXPORT(swd_seq, send std SWD connect-seq ONCE continuous for LA capture; f=forever);

/* 原始位 dump: 连接 -> 发 DPIDR 请求 -> turnaround -> 连读 8 字节原始位 hex 打印。
 * 不判 ACK / 不管对齐 —— 只回答一个问题: 目标到底有没有在我们的线上驱动过任何东西?
 *   出现任何非 0 (尤其能看出 1BA01477 的影子) => 目标活着, 只是对齐/采样要调。
 *   全 0x00                                  => 目标在我们线上彻底沉默 => 切换序列没落地。 */
static int swd_raw(int argc, char **argv)
{
    (void)argc; (void)argv;

    dap_swd_spi_init();
    swd_send_connect();   /* 连续连接序列 (单事务), 末尾 idle 已就绪 */

    swd_write_bits(0xA5u, 8);   /* DPIDR 读请求 (RnW=1, addr0), LSB-first 上线 = 1,0,1,0,0,1,0,1 */
    swd_turnaround();

    rt_kprintf("swd_raw: 8 raw bytes after req+trn (LSB-first in time):\n  ");
    for (int i = 0; i < 8; i++) {
        uint32_t v = swd_read_bits(8);
        rt_kprintf("%02X ", v & 0xFFu);
    }
    rt_kprintf("\n");
    return 0;
}
MSH_CMD_EXPORT(swd_raw, dump 8 raw bytes after DPIDR req to see if target drives anything);

/* PA29/SWDIO 静态电平诊断: 切 GPIO 输入, 分别内部上拉/下拉读。
 *   上拉=1 下拉=0 -> 引脚悬空(跟随内部拉) -> 无外部驱动, SPI 读 0 是 MOSI 没三态(固件)
 *   上拉=0 下拉=0 -> 外部硬拉低(短路/探错脚/GND/目标按住)
 *   上拉=1 下拉=1 -> 外部硬拉高 */
static int swd_pin(int argc, char **argv)
{
    (void)argc; (void)argv;

    HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_GPIO_A_29;
    gpio_set_pin_input(HPM_GPIO0, GPIO_DO_GPIOA, 29u);

    HPM_IOC->PAD[IOC_PAD_PA29].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    for (volatile int i = 0; i < 200000; i++) { __asm volatile("nop"); }
    uint8_t hi = gpio_read_pin(HPM_GPIO0, GPIO_DO_GPIOA, 29u);

    HPM_IOC->PAD[IOC_PAD_PA29].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);
    for (volatile int i = 0; i < 200000; i++) { __asm volatile("nop"); }
    uint8_t lo = gpio_read_pin(HPM_GPIO0, GPIO_DO_GPIOA, 29u);

    rt_kprintf("swd_pin PA29(SWDIO) GPIO-in: pull-up=%d  pull-down=%d\n", hi, lo);
    if (hi == 1 && lo == 0)
        rt_kprintf("  => floating (follows internal pull): no external drive -> SPI reads 0 = MOSI not tri-stated (fw/SPI cfg)\n");
    else if (hi == 0 && lo == 0)
        rt_kprintf("  => input stuck LOW: weak pull-up cannot win, external strong pull-down/drive\n");
    else if (hi == 1 && lo == 1)
        rt_kprintf("  => input stuck HIGH: external strong pull-up/drive\n");
    else
        rt_kprintf("  => abnormal combo\n");

    /* ---- 推挽输出自测: 证明 pad 能驱动 + 读路径正常, 并区分外部下拉 vs 死短路 ---- */
    HPM_IOC->PAD[IOC_PAD_PA29].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7);   /* 最强驱动, 不带内部拉 */
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 29u, 1);
    gpio_set_pin_output(HPM_GPIO0, GPIO_DO_GPIOA, 29u);
    for (volatile int i = 0; i < 200000; i++) { __asm volatile("nop"); }
    uint8_t oh = gpio_read_pin(HPM_GPIO0, GPIO_DO_GPIOA, 29u);
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 29u, 0);
    for (volatile int i = 0; i < 200000; i++) { __asm volatile("nop"); }
    uint8_t ol = gpio_read_pin(HPM_GPIO0, GPIO_DO_GPIOA, 29u);
    gpio_set_pin_input(HPM_GPIO0, GPIO_DO_GPIOA, 29u);

    rt_kprintf("swd_pin PA29 push-pull selftest: out-high reads %d, out-low reads %d\n", oh, ol);
    if (oh == 1 && ol == 0) {
        rt_kprintf("  => pad drives & read-path OK. With input-stuck-low above => SWDIO net has external pull-down/buffer\n");
        rt_kprintf("     (tip: measure PA29 R-to-GND/VDD; check schematic SWDIO net for pull-down or direction buffer)\n");
    } else if (oh == 0) {
        rt_kprintf("  => out-high reads 0 => PA29 shorted to GND, or read-path fault -> ohmmeter PA29-to-GND\n");
    }

    dap_swd_spi_init();   /* 还原 SPI 复用 + SWD 上拉 */
    return 0;
}
MSH_CMD_EXPORT(swd_pin, probe PA29/SWDIO static level (release vs short diag));

/* 把 PA29 停在固定电平 (GPIO 推挽), 便于用万用表量真实电压。
 *   swd_drive 1  -> 输出高, 量 PA29 对地电压: ≈VDD 才正常; 仍≈0 => 被板载器件/短路摁死
 *   swd_drive 0  -> 输出低
 * 量完务必 reset 板子或重 init (此命令把 PA29 留在 GPIO 输出态)。 */
static int swd_drive(int argc, char **argv)
{
    int lvl = (argc >= 2) ? (atoi(argv[1]) != 0) : 1;
    HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_GPIO_A_29;
    HPM_IOC->PAD[IOC_PAD_PA29].PAD_CTL  = IOC_PAD_PAD_CTL_DS_SET(7);
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 29u, (uint8_t)lvl);
    gpio_set_pin_output(HPM_GPIO0, GPIO_DO_GPIOA, 29u);
    rt_kprintf("swd_drive: PA29 = %d (push-pull). Measure PA29-to-GND voltage.\n", lvl);
    return 0;
}
MSH_CMD_EXPORT(swd_drive, park PA29/SWDIO at fixed level for voltage probing: swd_drive 0|1);

/* 把 PA27(SWCLK) 停在固定电平, 验证 SWCLK 线是否通到目标 PA14。
 *   swd_clk 1 -> 量目标 PA14 应 ≈VDD; swd_clk 0 -> 应 ≈0。 量完复位/重 init 恢复。 */
static int swd_clk(int argc, char **argv)
{
    int lvl = (argc >= 2) ? (atoi(argv[1]) != 0) : 1;
    HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL = IOC_PA27_FUNC_CTL_GPIO_A_27;
    HPM_IOC->PAD[IOC_PAD_PA27].PAD_CTL  = IOC_PAD_PAD_CTL_DS_SET(7);
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 27u, (uint8_t)lvl);
    gpio_set_pin_output(HPM_GPIO0, GPIO_DO_GPIOA, 27u);
    rt_kprintf("swd_clk: PA27 = %d (push-pull). Measure target PA14-to-GND voltage.\n", lvl);
    return 0;
}
MSH_CMD_EXPORT(swd_clk, park PA27/SWCLK at fixed level for continuity probing: swd_clk 0|1);

/* SPI 读路径外部环回测试: 把 SWDIO 用杜邦线接 3.3V 或 GND, 再跑本命令。
 *   接 3.3V 应读回全 0xFF; 接 GND 应读回全 0x00。
 *   若接 3.3V 仍读 0x00 => SPI 的 mosi_bidir 读相位采不到 PA29 (固件/pad/SPI 配置问题)。
 *   若 3.3V->0xFF / GND->0x00 => SPI 读路径正常 => 问题在目标侧(没驱动/没接)。 */
static int swd_rdtest(int argc, char **argv)
{
    (void)argc; (void)argv;
    dap_swd_spi_init();

    rt_kprintf("swd_rdtest: tie MISO(PA28)/SWDIO to 3.3V or GND, watch MISO read value:\n");
    for (int i = 0; i < 8; i++) {
        uint32_t v = swd_read_bits(8);   /* 全双工: MOSI 出 1, 采 MISO */
        rt_kprintf("  read[%d] = 0x%02X\n", i, v & 0xFFu);
    }
    return 0;
}
MSH_CMD_EXPORT(swd_rdtest, MISO read-path test: tie MISO/SWDIO to 3V3/GND then run);

/* ★ 决定性三态判定 (无需目标 / 无需外接元件): 断开 SWDIO 一切外部连接后跑。
 *   PA29 已配内部上拉(PS=1)。 先 write 0x00(驱动低)->切读->采8位=A; 再 write 0xFF(驱动高)->切读->采8位=B。
 *     A=B=0xFF        => 读相位 MOSI 真三态(浮空跟随内部上拉) -> 三态 OK, 根因在别处(采样沿/时序)。
 *     A=0x00 B=0xFF   => 读到的是"自己上次写的值" -> 读相位 MOSI 没三态! 读 ACK 时 MCU 仍 ~50ohm 驱动这根线,
 *                        和目标对顶 -> 线被拉到中点 -> 采成 0 => 这正是 ACK 恒 0 的根因(与 cpha/trn 无关)。
 *     两次相同但非0xFF => 线被某固定外部源主导, 检查接线/板载 buffer。 */
static int swd_trit(int argc, char **argv)
{
    (void)argc; (void)argv;
    dap_swd_spi_init();   /* 含 swd_pad_pullup(): PA29 内部上拉 */

    swd_write_bits(0x00u, 8);          /* MOSI 出低 */
    uint32_t a = swd_read_bits(8);     /* 全双工读 MISO */
    swd_write_bits(0xFFu, 8);          /* MOSI 出高 */
    uint32_t b = swd_read_bits(8);     /* 全双工读 MISO */

    rt_kprintf("swd_trit: after-drive-LOW read=0x%02X  after-drive-HIGH read=0x%02X\n",
               a & 0xFFu, b & 0xFFu);
    if ((a & 0xFFu) == 0xFFu && (b & 0xFFu) == 0xFFu)
        rt_kprintf("  => both 0xFF: read-phase MOSI truly tri-stated (follows pull-up). tri-state OK -> root cause is sample edge/timing, not clash.\n");
    else if ((a & 0xFFu) != (b & 0xFFu))
        rt_kprintf("  => read = last written value! read-phase MOSI not released -> clashes with target = ACK-stuck-0 root cause. need hw change or other read method.\n");
    else
        rt_kprintf("  => both same but not 0xFF(0x%02X): line dominated by fixed external source, check wiring/onboard buffer.\n", a & 0xFFu);
    return 0;
}
MSH_CMD_EXPORT(swd_trit, decisive tri-state test no target needed: drive then read detect MOSI release);

/* nRESET(PA26) 脉冲自测: 拉低目标 NRST 一段再释放, 读回电平 (IOC 操作在 dap_swd_spi.c)。
 * 无目标: 释放后 PA26 浮空跟随内部弱上拉 -> 读回 1。
 * 接目标: 拉低期间目标被复位; 释放后目标上拉拉高 -> 读回 1。 LA 抓 PA26 可看脉冲。
 *   swd_rst        默认拉低 10ms 再释放
 *   swd_rst <ms>   自定义拉低时长 */
static int swd_rst(int argc, char **argv)
{
    int ms = (argc >= 2) ? atoi(argv[1]) : 10;
    if (ms <= 0) ms = 10;

    rt_kprintf("swd_rst: NRST(PA26) LOW for %d ms ...\n", ms);
    uint8_t lvl = dap_swd_nreset_pulse((uint32_t)ms);
    rt_kprintf("swd_rst: released, PA26 reads %d %s\n", lvl,
               lvl ? "(high: target/internal pull-up OK)" : "(low: still held? check wiring/target in reset)");
    return 0;
}
MSH_CMD_EXPORT(swd_rst, pulse target NRST(PA26) low then release: swd_rst [ms]);

/* nRESET(PA26) 停在固定电平 (推挽强驱动), 用万用表量 PA26 对地电压确认引脚是否真在动。
 *   swd_rstlvl 1 -> PA26 输出高 (量应 ≈VDD); swd_rstlvl 0 -> 输出低 (量应 ≈0)。
 * 比 swd_rst 的 10ms 脉冲好测 (停住不动)。 量完 reset 板子或重 init 恢复。 */
static int swd_rstlvl(int argc, char **argv)
{
    int lvl = (argc >= 2) ? (atoi(argv[1]) != 0) : 1;
    dap_swd_nreset_park((uint8_t)lvl);
    rt_kprintf("swd_rstlvl: PA26 = %d (push-pull). Measure PA26-to-GND voltage (high ~= VDD, low ~= 0).\n", lvl);
    return 0;
}
MSH_CMD_EXPORT(swd_rstlvl, park NRST(PA26) at fixed level for voltage probing: swd_rstlvl 0|1);
