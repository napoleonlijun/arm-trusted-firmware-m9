/*
 * Copyright (C) 2017, Fuzhou Rockchip Electronics Co., Ltd.
 * All rights reserved.
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

#include <arch.h>
#include <arch_helpers.h>
#include <assert.h>
#include <debug.h>
#include <gic_v2.h>
#include <interrupt_mgmt.h>
#include <platform.h>
#include <plat_private.h>
#include <rockchip_exceptions.h>
#include <rockchip_sip_svc.h>
#include <delay_timer.h>
#include "fiq_dfs.h"

void dcsw_op_level1(uint32_t setway);
void dcsw_op_level2(uint32_t setway);

static __sramdata volatile uint32_t cpu_stop_trigger;
static __sramdata volatile uint32_t cpu_stop_st[PLATFORM_CORE_COUNT];
static __sramdata __aligned(16)
		uint64_t cpu_stop_sp[PLATFORM_CORE_COUNT][CPU_STOP_SP_CNT];

static uint32_t dfs_governor;
static uint64_t cpu_daif[PLATFORM_CORE_COUNT];
/* must be non-cacheable and volatile !! */
static uint64_t volatile mcu_dfs_done_flag
#if USE_COHERENT_MEM
__section("tzfw_coherent_mem")
#endif
;

#pragma weak fiq_dfs_wait_cpus_wfe

int fiq_dfs_wait_cpus_wfe(void)
{
	ERROR("%s not implement by platform! panic\n", __func__);
	panic();

	return SIP_RET_NOT_SUPPORTED;
}

void fiq_dfs_flush_l1(void)
{
	dcsw_op_level1(DCCISW);
}

void fiq_dfs_flush_l2(void)
{
	dcsw_op_level2(DCCISW);
}

void fiq_dfs_prefetch(void (*prefetch_fn)(void))
{
	size_t sram_size;
	unsigned long start, end, n;

#ifndef PLAT_SKIP_DFS_TLB_DCACHE_MAINTENANCE
	/* invalidate tlb */
	tlbialle3();
#endif

	/* prefetch sram text section */
	sram_size = (char *)&__bl31_sram_text_end -
		    (char *)&__bl31_sram_text_start;
	start = (unsigned long)&__bl31_sram_text_start;
	end = (unsigned long)&__bl31_sram_text_start + sram_size;

	for ( ; start < end; start += 0x1000)
		__asm volatile("ldr %0, [%1]\n" : "=r" (n) : "r"(start));

	/* prefetch sram data section */
	sram_size = (char *)&__bl31_sram_data_end -
		    (char *)&__bl31_sram_data_start;
	start = (unsigned long)&__bl31_sram_data_start;
	end = (unsigned long)&__bl31_sram_data_start + sram_size;

	for ( ; start < end; start += 0x1000)
		__asm volatile("ldr %0, [%1]\n" : "=r" (n) : "r"(start));

	if (prefetch_fn)
		prefetch_fn();

	dsb();
}

int wait_cpu_stop_st_timeout(uint32_t cpu)
{
	uint32_t loop = 1000 * 1000; /* wait 1s */

	while ((cpu_stop_st[cpu] != CPUS_STOP_ST_IN) && loop > 0) {
		udelay(1);
		loop--;
	}

	if (cpu_stop_st[cpu] == CPUS_STOP_ST_IN)
		return 0;

	ERROR("%s:The cpu_stop_st is error! %d:%x\n",
	      __func__, cpu, cpu_stop_st[cpu]);
	return -1;
}

__sramfunc void cpu_stop_for_event(uint32_t cpu)
{
	cpu_stop_st[cpu] = CPUS_STOP_ST_IN;

	/* cpu_stop_trigger[cpu] must be prefetched !! */
	while (cpu_stop_trigger == CPUS_STOP_EN) {
		dsb();
		isb();
		__asm volatile("wfe");
	}

	cpu_stop_st[cpu] = CPUS_STOP_ST_OUT;
}

static uint64_t fiq_cpu_stop_handler(uint32_t id,
				     uint32_t flags,
				     void *handle,
				     void *cookie)
{
	uint64_t save_sp;
	uint32_t cpu = plat_my_core_pos();

	/* disable local cpu's all exceptions */
	cpu_daif[cpu] = read_daif();
	write_daifset(DISABLE_ALL_EXCEPTIONS);

#ifndef PLAT_SKIP_DFS_TLB_DCACHE_MAINTENANCE
	/* flush dcache */
	dcsw_op_level1(DCCISW);
	dcsw_op_level2(DCCISW);
	/* necessary! invalidate tlb of el3 */
	tlbialle3();
#endif
	dsb();
	isb();
	/* set sp to sram */
	save_sp = rockchip_get_sp();

	assert(!(save_sp & 0xf));

	rockchip_set_sp((uint64_t)(&cpu_stop_sp[cpu][CPU_STOP_SP_TOP]));
	dsb();
	isb();

	/* stop myself, wfe in sram */
	cpu_stop_for_event(cpu);

	/* restore sp */
	rockchip_set_sp(save_sp);

	/* restore exceptions */
	write_daif(cpu_daif[cpu]);

	return 0;
}

static void gic_set_sgir_except_self(size_t it)
{
	gicd_write_sgir(PLAT_RK_GICD_BASE, REQ_SGI_EXCEPT_SELF | it);
}

void fiq_cpu_stop_info_init(void)
{
	int i;

	for (i = 0; i < PLATFORM_CORE_COUNT; i++)
		cpu_stop_st[i] = CPUS_STOP_ST_OUT;

	cpu_stop_trigger = CPUS_STOP_DIS;
}

int fiq_dfs_stop_cpus(void)
{
	int ret = 0;

	cpu_stop_trigger = CPUS_STOP_EN;
	dsb();

	/* send sgi to notify other cpus into wfe */
	gic_set_sgir_except_self(RK_IRQ_SEC_SGI_6);

	/* wait other cpus all into wfe, return 0 on success */
	if (dfs_governor == DFS_GOVERNOR_CPU)
		ret = fiq_dfs_wait_cpus_wfe();

	return ret;
}

void fiq_dfs_active_cpus(void)
{
	cpu_stop_trigger = CPUS_STOP_DIS;
	dsb();
	__asm("sev");
}

static int mcu_dfs_request_handler(uint32_t irqstat, uint32_t flags,
				   void *handle, void *cookie)
{
	/* don't have to wait cpus wfe, MCU will do that */
	fiq_dfs_stop_cpus();

#ifndef PLAT_SKIP_DFS_TLB_DCACHE_MAINTENANCE
	fiq_dfs_flush_l1();
	fiq_dfs_flush_l2();
#endif

	fiq_dfs_prefetch(NULL);

#if USE_COHERENT_MEM
	/* wait MCU dfs done flag. 0: done */
	while (mcu_dfs_done_flag)
		;
#else
	ERROR("%s: USE_COHERENT_MEM is not define!\n", __func__);
#endif
	/* active cpus */
	fiq_dfs_active_cpus();

	return SIP_RET_SUCCESS;
}

/******************************************************************************/
int cpu_dfs_governor_register(void)
{
	int ret;

	ret = register_secfiq_handler(RK_IRQ_SEC_SGI_6, fiq_cpu_stop_handler);
	if (ret)
		return ret;

	dfs_governor = DFS_GOVERNOR_CPU;
	fiq_cpu_stop_info_init();
	return 0;
}

uint64_t mcu_dfs_governor_register(uint32_t dfs_irq, uint32_t tgt_cpu,
				   struct arm_smccc_res *res)
{
	int err;

	/* register MCU fiq handler, that is: mcu_dfs_request_handler() */
	err = register_secfiq_handler(dfs_irq,
				      (interrupt_type_handler_t)
				      mcu_dfs_request_handler);
	if (err)
		return err;

	/* register stop cpus handler, triggered by mcu_dfs_request_handler() */
	err = cpu_dfs_governor_register();
	if (err)
		return err;

	/* enable MCU fiq and set target cpu */
	plat_rockchip_gic_fiq_enable(dfs_irq, tgt_cpu);

	/* MCU dfs done flag */
	res->a1 = (uint64_t)&mcu_dfs_done_flag;
	dfs_governor = DFS_GOVERNOR_MCU;

	return SIP_RET_SUCCESS;
}

#if 0
demo(should check both power down and wfe state on other cpus):

static int fiq_dfs_wait_cpus_wfe(void)
{
	uint32_t tgt_wfe, pd_st, wfe_st, loop = 2500;
	uint32_t wfe_reg = GRF_BASE + GRF_CPU_STATUS(1), wfe_msk = 0x0f;
	uint32_t pd_reg = PMU_BASE + PMU_PWRDN_ST, pd_msk = 0x0f;

	/* unmask current cpu */
	tgt_wfe = 0x0f & ~BIT(plat_my_core_pos());

	/* unmask offline(power down) cpus */
	pd_st = mmio_read_32(pd_reg) & pd_msk;
	tgt_wfe &= ~(pd_st);

	/* timeout 5ms */
	wfe_st = mmio_read_32(wfe_reg) & wfe_msk;
	while (((wfe_st & tgt_wfe) != tgt_wfe) && loop > 0) {
	       udelay(2);
	       loop--;
	       wfe_st = mmio_read_32(wfe_reg) & wfe_msk;
	}

	return ((wfe_st & tgt_wfe) == tgt_wfe) ? 0 : 1;
}
#endif
