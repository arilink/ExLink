/*
 * Copyright (c) 2026 LabForge / ExLink
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  cmsis_compiler.h —— RISC-V (HPM) 垫片
 * ============================================================================
 *
 *  官方 CMSIS-DAP 的 DAP.h 写死 `#include "cmsis_compiler.h"` (本为 ARM
 *  CMSIS-Core 提供)。 本工程是 RISC-V (HPM5361 + GCC), 无 ARM CMSIS-Core,
 *  故在 port/ 下提供同名垫片, 只定义 DAP.c / DAP_config.h 实际用到的编译器宏
 *  (__STATIC_INLINE / __STATIC_FORCEINLINE / __WEAK / __NO_RETURN / __ASM),
 *  全部用 GCC 内建属性实现, 与 ARM 版语义一致。
 *
 *  本文件被 arm/DAP.h #include; arm/ 与 port/ 均在工程 include 路径上,
 *  port/ 优先命中本垫片。
 * ============================================================================
 */
#ifndef __CMSIS_COMPILER_H
#define __CMSIS_COMPILER_H

#include <stdint.h>

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

#ifndef __PACKED
#define __PACKED               __attribute__((packed, aligned(1)))
#endif

#ifndef __ALIGNED
#define __ALIGNED(x)           __attribute__((aligned(x)))
#endif

#endif /* __CMSIS_COMPILER_H */
