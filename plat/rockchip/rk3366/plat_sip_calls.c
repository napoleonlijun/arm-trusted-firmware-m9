/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
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
#include <debug.h>
#include <mmio.h>
#include <platform_def.h>
#include <plat_sip_calls.h>
#include <rockchip_sip_svc.h>
#include <runtime_svc.h>
#include <rk3366_def.h>
#include <rockchip_sip_svc.h>
#include <soc.h>

#define RK_SIP_ATF_VER			0x82000001
#define RK_SIP_WR_PM_CTRBITS		0x82000003
#define RK_SIP_UARTDBG_CFG64		0xc2000005

#define UARTDBG_CFG_PRINT_PORT		0xf7

#define SECURE_REG_RD	0
#define SECURE_REG_WR	1

uint32_t g_uart_port = 2;
uint32_t g_uart_base = PLAT_RK_UART_BASE;
uint32_t g_uart_baudrate = PLAT_RK_UART_BAUDRATE;

uint32_t atf_version(void)
{
	return ((MAJOR_VERSION << 16) | MINOR_VERSION);
}

uint32_t pm_ctrbits_rw(uint32_t val, uint32_t rw)
{
	if (rw == SECURE_REG_WR)
		rkpm_set_ctrbits(val);
	else
		return rkpm_get_ctrbits();

	return SIP_RET_SUCCESS;
}

static uint32_t UART_BASE[] = {
	UART0_BASE,
	UART0_BASE,
	UART2_BASE,
	UART3_BASE,
};

unsigned int uartdbg_set_print_port(uint32_t port, uint32_t baudrate)
{
	if (port > UART_MAX_PORT || port == 1)
		return -1;

	g_uart_port = port;
	g_uart_base = UART_BASE[port];
	g_uart_baudrate = baudrate;

	/* only init console base */
	console_init(UART_BASE[port], 0, 0);

	return 0;
}

static uint64_t uartdbg_smc_handler(uint64_t x1,
				    uint64_t x2,
				    uint64_t fun_id,
				    void *handle)
{
	switch (fun_id) {
	case UARTDBG_CFG_PRINT_PORT:
		return uartdbg_set_print_port(x1, x2);
	default:
		return SMC_UNK;
	}
}

uint64_t rockchip_plat_sip_handler(uint32_t smc_fid,
				   uint64_t x1,
				   uint64_t x2,
				   uint64_t x3,
				   uint64_t x4,
				   void *cookie,
				   void *handle,
				   uint64_t flags)
{
	switch (smc_fid) {
	case RK_SIP_ATF_VER:
		SMC_RET1(handle, atf_version());

	case RK_SIP_WR_PM_CTRBITS:
		SMC_RET2(handle, 0, pm_ctrbits_rw(x1, x3));

	case RK_SIP_UARTDBG_CFG64:
		SMC_RET1(handle, uartdbg_smc_handler(x1, x2, x3, handle));

	default:
		ERROR("%s: unhandled SMC (0x%x)\n", __func__, smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}
}

