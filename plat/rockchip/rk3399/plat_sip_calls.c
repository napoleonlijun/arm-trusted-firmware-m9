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

#include <debug.h>
#include <mmio.h>
#include <psci.h>
#include <rockchip_sip_svc.h>
#include <runtime_svc.h>
#include <dram.h>
#include <plat_private.h>
#include <plat_sip_calls.h>
#include <arch_helpers.h>
#include <platform.h>
#include <dfs.h>

#pragma weak suspend_mode_handler

int suspend_mode_handler(uint64_t mode_id, uint64_t config1, uint64_t config2)
{
	return 0;
}

static int ddr_smc_handler(uint64_t arg0, uint64_t arg1,
		    uint64_t id, struct arm_smccc_res *res)
{
	switch (id) {
	case CONFIG_DRAM_INIT:
		dram_dfs_init();
		break;
	case CONFIG_DRAM_SET_RATE:
		return ddr_set_rate((uint32_t)arg0);
	case CONFIG_DRAM_ROUND_RATE:
		return ddr_round_rate((uint32_t)arg0);
	case CONFIG_DRAM_GET_RATE:
		return ddr_get_rate();
	case CONFIG_DRAM_SET_PARAM:
		dts_timing_receive((uint32_t)arg0, (uint32_t)arg1);
		break;
	default:
		return SIP_RET_INVALID_PARAMS;
	}

	return SIP_RET_SUCCESS;
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
	struct arm_smccc_res res;

	switch (smc_fid) {

	case RK_SIP_DDR_CFG32:
		ret = ddr_smc_handler(x1, x2, x3, &res);
		SMC_RET4(handle, ret, res.a1, res.a2, res.a3);

	case RK_SIP_SUSPEND_MODE32:
		SMC_RET1(handle, suspend_mode_handler(x1, x2, x3));

	case RK_SIP_SHARE_MEM32:
		ret = share_mem_page_get_handler(x1, x2, &res);
		SMC_RET4(handle, ret, res.a1, res.a2, res.a3);

	default:
		ERROR("%s: unhandled SMC (0x%x)\n", __func__, smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}
}
