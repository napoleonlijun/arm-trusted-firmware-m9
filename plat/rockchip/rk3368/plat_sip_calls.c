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

#include <assert.h>
#include <cpu_data.h>
#include <debug.h>
#include <delay_timer.h>
#include <mmio.h>
#include <platform.h>
#include <plat_private.h>
#include <plat_sip_calls.h>
#include <pmu.h>
#include <rockchip_sip_svc.h>
#include <runtime_svc.h>
#include <fiq_dfs.h>

int dfs_wait_cpus_wfe(void)
{
	uint32_t val, tgt_wfe, pd_st, wfe_st, loop = 2500;

	/* unmask current cpu */
	tgt_wfe = 0xff & (~(1 << plat_my_core_pos()));

	/* unmask offline(power down) cpus */
	val = mmio_read_32(PMU_BASE + PMU_PWRDN_ST);
	pd_st = (val & 0x0f) | ((val >> 1) & 0xf0); /* bit[8:5][3:0] */
	tgt_wfe &= ~(pd_st);

	/* read wfe status on cpus */
	val = (mmio_read_32(PMU_BASE + PMU_CORE_PWR_ST));
	wfe_st = ((val >> 2) & 0xf) | ((val >> 8) & 0xf0); /* bit[15:12][5:2] */

	/* wait timeout 5ms */
	while (((wfe_st & tgt_wfe) != tgt_wfe) && loop > 0) {
		udelay(2);
		loop--;
		/* read wfe status on cpus */
		val = (mmio_read_32(PMU_BASE + PMU_CORE_PWR_ST));
		wfe_st = ((val >> 2) & 0xf) | ((val >> 8) & 0xf0);
	}

	return ((wfe_st & tgt_wfe) == tgt_wfe) ? 0 : 1;
}

static int mcu_dfs_handler(uint64_t func,
		       uint64_t irq,
		       uint64_t tgt_cpu,
		       struct arm_smccc_res *res)
{
	int ret;

	switch (func) {
	case FIQ_INIT_HANDLER:
		ret = mcu_dfs_governor_register(irq, tgt_cpu, res);
		break;

	default:
		return SMC_UNK;
	}

	return ret;
}

#pragma weak suspend_mode_handler

int suspend_mode_handler(uint64_t mode_id,
			 uint64_t config1,
			 uint64_t config2)
{
	return 0;
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
	int ret = SIP_RET_DENIED;
	struct arm_smccc_res res = {0};

	switch (smc_fid) {
	case RK_SIP_MCU_EL3FIQ_CFG:
		ret = mcu_dfs_handler(x1, x2, x3, &res);
		SMC_RET4(handle, ret, res.a1, res.a2, res.a3);

	case RK_SIP_SUSPEND_MODE32:
		ret = suspend_mode_handler(x1, x2, x3);
		SMC_RET1(handle, ret);

	default:
		ERROR("%s: unhandled SMC (0x%x)\n", __func__, smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}
}
