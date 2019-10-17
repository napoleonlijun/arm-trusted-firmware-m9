/*
 * Copyright (c) 2014-2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __ROCKCHIP_PLAT_PRIVATE_MACROS_S__
#define __ROCKCHIP_PLAT_PRIVATE_MACROS_S__

#include <rk3366_def.h>
#include <uart_16550.h>

#define ddr_ret_req	(1 << 12)
#define ddr_ret_de_req	(1 << 13)
#define pmu_mode_bus_pd (1 << 2)
#define PMU_PWRMD_COM	0x18
#define CRU_APLL_CON3	0x0c
#define PLL_SLOW_MODE	(0x0 << 8 | 0x3 << 24)

.macro console_sram_init
	/* Check pdbus */
	ldr	x0, pmu_base
	ldr     w1, [x0, #PMU_PWRMD_COM]
	and	w1, w1, #pmu_mode_bus_pd
	cmp     w1, #0x0
	b.eq	exit_console_sram_init

	ldr	x0, uart2_base
	ldr	x1, console_clk
	ldr	x2, console_baudrate
	/* Program the baudrate */
	/* Divisor =  Uart clock / (16 * baudrate) */
	lsl	w2, w2, #4
	udiv	w2, w1, w2
	and	w1, w2, #0xff		/* w1 = DLL */
	lsr	w2, w2, #8
	and	w2, w2, #0xff		/* w2 = DLLM */
	ldr	w3, [x0, #UARTLCR]
	orr	w3, w3, #UARTLCR_DLAB
	str	w3, [x0, #UARTLCR]	/* enable DLL, DLLM programming */
	str	w1, [x0, #UARTDLL]	/* program DLL */
	str	w2, [x0, #UARTDLLM]	/* program DLLM */
	mov	w2, #~UARTLCR_DLAB
	and	w3, w3, w2
	str	w3, [x0, #UARTLCR]	/* disable DLL, DLLM programming */

	/* 8n1 */
	mov	w3, #3
	str	w3, [x0, #UARTLCR]
	/* no interrupt */
	mov	w3, #0
	str	w3, [x0, #UARTIER]
	/* enable fifo, DMA */
	mov	w3, #(UARTFCR_FIFOEN | UARTFCR_DMAEN)
	str	w3, [x0, #UARTFCR]
	/* DTR + RTS */
	mov	w3, #3
	str	w3, [x0, #UARTMCR]
	mov	w0, #1
	dsb	sy
exit_console_sram_init:
.endm

.macro console_sram_putc ch
	ldr	x0, uart2_base
	mov	w1, \ch
	str	w1, [x0]
	dsb	sy
.endm

.macro apll_slow_mode
	ldr	x0, cru_base
	ldr	w1, [x0, #CRU_APLL_CON3]
	mov	w1, #PLL_SLOW_MODE
	str	w1, [x0, #CRU_APLL_CON3]
	dsb	sy
.endm

.macro ddr_retention_dereq
	ldr	x0, pmu_base
	ldr	w1, [x0, #PMU_PWRMD_COM]
	and	w2, w1, #ddr_ret_req
	cmp	w2, #0x0
	b.eq	exit_ddr_retention_dereq
	orr	w1, w1, #ddr_ret_de_req
	str	w1, [x0, #PMU_PWRMD_COM]
	dsb	sy
exit_ddr_retention_dereq:
.endm

/*
 * Called in sram before ddr resumed, the functions should not be embeded here
 */
.macro plat_resume_early
	apll_slow_mode
	console_sram_init
	console_sram_putc 'A'
.endm

/*
 * Called in sram after ddr resumed, the functions can be embeded here
 */
.macro plat_resume_late
	console_sram_putc 'B'
	ddr_retention_dereq
	console_sram_putc 'C'
.endm

/*
 * Declare the variable which will be used in sram
 */
.macro plat_declare_variable
	pmu_base:
		.quad	PMU_BASE
	uart2_base:
		.quad	UART2_BASE
	cru_base:
		.quad	CRU_BASE
	console_clk:
		.quad	RK3366_UART_CLOCK
	console_baudrate:
		.quad	RK3366_BAUDRATE
.endm

#endif /* __ROCKCHIP_PLAT_PRIVATE_MACROS_S__ */
