/*
 * Copyright (c) 2017, ARM Limited and Contributors. All rights reserved.
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

#include <console.h>
#include <platform_def.h>
#include <plat_private.h>
#include <uart_16550.h>
#include <uart.h>

struct uart_debug {
	uint32_t uart_dll;
	uint32_t uart_dlh;
	uint32_t uart_ier;
	uint32_t uart_fcr;
	uint32_t uart_mcr;
	uint32_t uart_lcr;
};

static struct uart_debug debug_port_save;

void rockchip_uart_debug_save(void)
{
	debug_port_save.uart_lcr = mmio_read_32(PLAT_RK_UART_BASE + UARTLCR);
	debug_port_save.uart_ier = mmio_read_32(PLAT_RK_UART_BASE + UARTIER);
	debug_port_save.uart_mcr = mmio_read_32(PLAT_RK_UART_BASE + UARTMCR);
	mmio_write_32(PLAT_RK_UART_BASE + UARTLCR,
	debug_port_save.uart_lcr | UARTLCR_DLAB);
	debug_port_save.uart_dll = mmio_read_32(PLAT_RK_UART_BASE + UARTDLL);
	debug_port_save.uart_dlh = mmio_read_32(PLAT_RK_UART_BASE + UARTDLLM);
	mmio_write_32(PLAT_RK_UART_BASE + UARTLCR, debug_port_save.uart_lcr);
}

void rockchip_uart_debug_restore(void)
{
	uint32_t uart_lcr;

	mmio_write_32(PLAT_RK_UART_BASE + UARTSRR,
		      XMIT_FIFO_RESET | RCVR_FIFO_RESET | UART_RESET);

	uart_lcr = mmio_read_32(PLAT_RK_UART_BASE + UARTLCR);
	mmio_write_32(PLAT_RK_UART_BASE + UARTMCR, DIAGNOSTIC_MODE);
	mmio_write_32(PLAT_RK_UART_BASE + UARTLCR, uart_lcr | UARTLCR_DLAB);
	mmio_write_32(PLAT_RK_UART_BASE + UARTDLL, debug_port_save.uart_dll);
	mmio_write_32(PLAT_RK_UART_BASE + UARTDLLM, debug_port_save.uart_dlh);
	mmio_write_32(PLAT_RK_UART_BASE + UARTLCR, debug_port_save.uart_lcr);
	mmio_write_32(PLAT_RK_UART_BASE + UARTIER, debug_port_save.uart_ier);
	mmio_write_32(PLAT_RK_UART_BASE + UARTFCR, UARTFCR_FIFOEN);
	mmio_write_32(PLAT_RK_UART_BASE + UARTMCR, debug_port_save.uart_mcr);
}
