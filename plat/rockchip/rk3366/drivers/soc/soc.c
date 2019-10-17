/*
 * Copyright (C) 2016, Fuzhou Rockchip Electronics Co., Ltd.
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

#include <mmio.h>
#include <debug.h>
#include <console.h>
#include <arch_helpers.h>
#include <platform_def.h>
#include <rockchip_sip_svc.h>
#include <rk3366_def.h>
#include <delay_timer.h>
#include <plat_private.h>
#include <soc.h>
#include <pmu.h>


/* Table of regions to map using the MMU. */
const mmap_region_t plat_rk_mmap[] = {
	MAP_REGION_FLAT(UART0_BASE, UART0_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(UART2_BASE, UART2_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(UART3_BASE, UART3_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(I2C_AUDIO_BASE, I2C_AUDIO_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(I2C_PMU_BASE, I2C_PMU_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(PMUSRAM_BASE, PMUSRAM_SIZE,
			MT_MEMORY | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(PMU_BASE, PMU_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(SGRF_BASE, SGRF_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GPIO0_BASE, GPIO0_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GPIO1_BASE, GPIO1_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GPIO2_BASE, GPIO2_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GPIO3_BASE, GPIO3_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GPIO4_BASE, GPIO4_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GPIO5_BASE, GPIO5_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(CRU_BASE, CRU_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GRF_BASE, GRF_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(NTIME_BASE, NTIME_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(STIME_BASE, STIME_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(SERVICE_BASE, SERVICE_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(DDR_SGRF_BASE, DDR_SGRF_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GIC400_BASE, GIC400_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(INTMEM_BASE, INTMEM_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	{ 0 }
};

/* The RockChip power domain tree descriptor */
const unsigned char rockchip_power_domain_tree_desc[] = {
	/* No of root nodes */
	PLATFORM_SYSTEM_COUNT,
	/* No of children for the root node */
	PLATFORM_CLUSTER_COUNT,
	/* No of children for the first cluster node */
	PLATFORM_CLUSTER0_CORE_COUNT,
};

#define BANK(n)		(n)

static uint32_t GPIO_BASE[] = {
	GPIO0_BASE, GPIO1_BASE, GPIO2_BASE,
	GPIO3_BASE, GPIO4_BASE, GPIO5_BASE
};

#define gpio_read32(n, offset) mmio_read_32(GPIO_BASE[n] + offset)
#define gpio_write32(n, v, offset)\
	do { mmio_write_32(GPIO_BASE[n] + offset, v); dsb(); } while (0)

struct rk_soc_data {
	uint32_t pmugrf_soc_con2;
	uint32_t plls_con[END_PLL][PLL_CONS_CNT];
	uint32_t clk_sel[CRU_CLKSEL_NUMS];
	uint32_t clk_gate[CRU_CLKSEL_NUMS];
	uint32_t i2c[I2C_END][10];
	uint32_t iomux[16];
	uint32_t gpio_dir[6];
	uint32_t gpio_data[6];
	uint32_t grf_soc_con[18];
	uint32_t ntimer[NTIME_CHS][3];
	uint32_t stimer[STIME_CHS][3];
};

#if USE_COHERENT_MEM
struct rk_soc_data rksoc __attribute__ ((section("tzfw_coherent_mem")));
#else
struct rk_soc_data rksoc;
#endif


void secure_timer_init(void)
{
	/* enable pclk stimer and stimer, 0:enable, 1:disable */
	sgrf_write32(0x00230000, SGRF_SOC_CON(3));

	stimer_write32(CHN(1), 0xffffffff, TIMER_LOADE_COUNT0);
	stimer_write32(CHN(1), 0xffffffff, TIMER_LOADE_COUNT1);
	/* auto reload & enable the timer */
	stimer_write32(CHN(1), TIMER_EN, TIMER_CONTROL_REG);
}

void sgrf_init(void)
{
	/* setting all slave ip into no-secure */
	ddrsgrf_write32(0xffff0000, DDR_SGRF_CON(0));
	sgrf_write32(0xf0002, SGRF_SOC_CON(2));
	sgrf_write32(0xffff0000, SGRF_SOC_CON(7));
	sgrf_write32(0xffff0800, SGRF_SOC_CON(8));
	sgrf_write32(0xffff0000, SGRF_SOC_CON(9));
	sgrf_write32(0xffff0000, SGRF_SOC_CON(10));

	/* secure dma to no sesure */
	sgrf_write32(SGRF_BUSDMAC_CON0_NS, SGRF_BUSDMAC_CON(0));
	sgrf_write32(SGRF_BUSDMAC_CON1_NS, SGRF_BUSDMAC_CON(1));
	dsb();

	/* rst dma1, dma2 */
	cru_write32(SOFTRST_DMA1_MSK | (SOFTRST_DMA1_MSK << 16),
		    CRU_SOFTRSTS_CON(1));
	cru_write32(SOFTRST_DMA2_MSK | (SOFTRST_DMA2_MSK << 16),
		    CRU_SOFTRSTS_CON(4));
	dsb();
	/* release dma1, dma2 rst*/
	cru_write32((SOFTRST_DMA1_MSK << 16), CRU_SOFTRSTS_CON(1));
	cru_write32((SOFTRST_DMA2_MSK << 16), CRU_SOFTRSTS_CON(4));
}

void plat_rockchip_soc_init(void)
{
	secure_timer_init();
	sgrf_init();

	NOTICE("BL31: Release version: %d.%d\n", MAJOR_VERSION, MINOR_VERSION);
}

static uint32_t pm_ctrbits =
	(0
	| RKPM_CTR_ARMOFF
	| RKPM_CTR_PWR_DMNS
	| RKPM_CTR_GTCLKS
	| RKPM_CTR_PLLS
	| RKPM_CTR_PD_WIFIBT
	| RKPM_CTR_PMU_PVTM32K
	| RKPM_CTR_PLL_DEEP
	| RKPM_CTR_DIS_24M
	| RKPM_CTR_DDR_GATING
	| RKPM_CTR_DDR_SREF
	);

void rkpm_set_ctrbits(uint32_t val)
{
	pm_ctrbits = val;
}

uint32_t rkpm_get_ctrbits(void)
{
	return pm_ctrbits;
}

uint32_t rkpm_chk_ctrbits(uint32_t bit)
{
	return (pm_ctrbits & bit);
}

/***************************  clk gating *******************************/
static uint32_t clk_ungt_msk[CRU_CLKGATE_NUMS] = {
	0xffff,
	0xffff,
	0xf830,
	0x3c0f,
	/*gate:4-7*/
	0x0,
	0x0819,
	0xff00,
	0x0309,
	/*gate:8-11*/
	0xf7e8,
	0xffff,
	0xffff,/*reserve*/
	0xffff,/*reserve*/
	/*gate:12-15*/
	0xfa74,
	0xffe4,
	0xffff,/*reserve*/
	0xff00,
	/*gate:16-19*/
	0xb800,
	0x0210,
	0xfffc,
	0x0543,
	/*gate:20-23*/
	0x17a0,
	0xff78,
	0xf3ff,
	0xffff,
	/*gate:24*/
	0xffc0
};

void gating_clks(void)
{
	int i;

	if (!rkpm_chk_ctrbits(RKPM_CTR_GTCLKS))
		return;

	for (i = 0; i < CRU_CLKGATE_NUMS; i++) {
		rksoc.clk_gate[i] = cru_read32(CRU_CLKGATE_CON(i));
		CLK_UNGATING(clk_ungt_msk[i], CRU_CLKGATE_CON(i));
	}

	/* aclk_dmac_bus */
	cru_write32(0x08000000, CRU_CLKGATE_CON(12));
}

void restore_clks(void)
{
	int i;

	if (!rkpm_chk_ctrbits(RKPM_CTR_GTCLKS))
		return;

	for (i = 0; i < CRU_CLKGATE_NUMS; i++)
		cru_write32(rksoc.clk_gate[i] | 0xffff0000, CRU_CLKGATE_CON(i));
}

/********************** plls power down ***************************************/
static void pll_suspend(uint32_t pll_id)
{
	rksoc.plls_con[pll_id][0] = cru_read32(PLL_CONS((pll_id), 0));
	rksoc.plls_con[pll_id][1] = cru_read32(PLL_CONS((pll_id), 1));
	rksoc.plls_con[pll_id][2] = cru_read32(PLL_CONS((pll_id), 2));
	rksoc.plls_con[pll_id][3] = cru_read32(PLL_CONS((pll_id), 3));

	cru_write32(PLL_POWER_DOWN, PLL_CONS((pll_id), 3));
}

void power_down_plls(void)
{
	uint32_t i;

	if (!rkpm_chk_ctrbits(RKPM_CTR_PLLS))
		return;

	if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF)) {
		for (i = 0; i < CRU_CLKSEL_NUMS; i++)
			rksoc.clk_sel[i] = cru_read32(CRU_CLKSEL_CON(i));
	} else {
		rksoc.clk_sel[0] = cru_read32(CRU_CLKSEL_CON(0));
		rksoc.clk_sel[1] = cru_read32(CRU_CLKSEL_CON(1));
		rksoc.clk_sel[4] = cru_read32(CRU_CLKSEL_CON(4));
		rksoc.clk_sel[6] = cru_read32(CRU_CLKSEL_CON(6));
		rksoc.clk_sel[8] = cru_read32(CRU_CLKSEL_CON(8));
		rksoc.clk_sel[9] = cru_read32(CRU_CLKSEL_CON(9));
		rksoc.clk_sel[10] = cru_read32(CRU_CLKSEL_CON(10));
		rksoc.clk_sel[11] = cru_read32(CRU_CLKSEL_CON(11));
		rksoc.clk_sel[12] = cru_read32(CRU_CLKSEL_CON(12));
		rksoc.clk_sel[27] = cru_read32(CRU_CLKSEL_CON(27));
		rksoc.clk_sel[29] = cru_read32(CRU_CLKSEL_CON(29));
		rksoc.clk_sel[31] = cru_read32(CRU_CLKSEL_CON(31));
		rksoc.clk_sel[37] = cru_read32(CRU_CLKSEL_CON(37));
		rksoc.clk_sel[43] = cru_read32(CRU_CLKSEL_CON(43));
		rksoc.clk_sel[53] = cru_read32(CRU_CLKSEL_CON(53));
		rksoc.pmugrf_soc_con2 = pmugrf_read32(PMUGRF_SOC_CON(2));
	}

	cru_write32(PLL_SLOW_MODE, PLL_CONS((APLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((NPLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((CPLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((GPLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((MPLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((WPLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((BPLL_ID), 3));

	/* pd_bus clk */
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x1f)
		      | REG_WMSK_BITS(0, 8, 0x3)
		      | REG_WMSK_BITS(0, 12, 0x7)
		      , CRU_CLKSEL_CON(8));
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x3f), CRU_CLKSEL_CON(27));
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x3f), CRU_CLKSEL_CON(31));
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x3f), CRU_CLKSEL_CON(53));

	/* crypto */
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x1f), CRU_CLKSEL_CON(6));

	/* mcu clk */
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x1f), CRU_CLKSEL_CON(12));
	pmugrf_write32(0 | REG_WMSK_BITS(0, 10, 0x7), PMUGRF_SOC_CON(2));

	/* pmu alive */
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x1f) | REG_WMSK_BITS(0, 8, 0x1f),
		    CRU_CLKSEL_CON(10));

	/* peri aclk hclk pclk */
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x1f), CRU_CLKSEL_CON(9));
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x1f), CRU_CLKSEL_CON(11));
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0xff), CRU_CLKSEL_CON(29));
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x1f), CRU_CLKSEL_CON(43));

	pll_suspend(NPLL_ID);
	pll_suspend(CPLL_ID);
	pll_suspend(GPLL_ID);
	pll_suspend(MPLL_ID);
	pll_suspend(WPLL_ID);
	pll_suspend(BPLL_ID);

	/* core */
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x1f) | REG_WMSK_BITS(0, 8, 0x1f),
		    CRU_CLKSEL_CON(0));
	cru_write32(0 | REG_WMSK_BITS(0, 0, 0x1f) | REG_WMSK_BITS(0, 8, 0x1f),
		    CRU_CLKSEL_CON(1));
	cru_write32(0 | REG_WMSK_BITS(0, 8, 0x1f), CRU_CLKSEL_CON(4));

	pll_suspend(APLL_ID);
}

static void pll_wait_lock(uint32_t pll_idx)
{
	uint32_t val, delay = 500;

	val = cru_read32(PLL_CONS((pll_idx), 3));

	if (val & PLL_IS_POWER_DOWN)
		return;

	while (delay > 0) {
		if ((cru_read32(PLL_CONS(pll_idx, 2)) & PLL_IS_LOCKED))
			break;
		udelay(2);
		delay--;
	}

	if (delay == 0)
		ERROR("unlock-pll: %d\n", pll_idx);
}

static void pll_resume(uint32_t pll_idx)
{
	uint32_t pllcon0, pllcon1, pllcon2, pllcon3;

	pllcon0 = rksoc.plls_con[pll_idx][0] | 0xffff0000;
	pllcon1 = rksoc.plls_con[pll_idx][1] | 0xffff0000;
	pllcon2 = rksoc.plls_con[pll_idx][2];
	pllcon3 = rksoc.plls_con[pll_idx][3] | 0xffff0000;
	cru_write32(pllcon0, PLL_CONS(pll_idx, 0));
	cru_write32(pllcon1, PLL_CONS(pll_idx, 1));
	cru_write32(pllcon2, PLL_CONS(pll_idx, 2));
	cru_write32(pllcon3, PLL_CONS(pll_idx, 3));

	if (!(pllcon3 && PLL_IS_POWER_DOWN))
		cru_write32(PLL_POWER_UP, PLL_CONS((pll_idx), 3));
}

static void plls_resume(void)
{
	pll_resume(APLL_ID);
	pll_resume(NPLL_ID);
	pll_resume(CPLL_ID);
	pll_resume(GPLL_ID);
	pll_resume(MPLL_ID);
	pll_resume(WPLL_ID);
	pll_resume(BPLL_ID);

	/* waiting lock state */
	udelay(200);

	pll_wait_lock(APLL_ID);
	pll_wait_lock(NPLL_ID);
	pll_wait_lock(CPLL_ID);
	pll_wait_lock(GPLL_ID);
	pll_wait_lock(MPLL_ID);
	pll_wait_lock(WPLL_ID);
	pll_wait_lock(BPLL_ID);
}

void restore_plls(void)
{
	uint32_t i;

	if (!rkpm_chk_ctrbits(RKPM_CTR_PLLS))
		return;

	plls_resume();

	if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF)) {
		for (i = 0; i < CRU_CLKSEL_NUMS; i++) {
			if (i == 28 || i == 32 || i == 34 || i == 36 ||
			    i == 40 || i == 42 || i == 44 || i == 54)
				cru_write32(rksoc.clk_sel[i],
					    CRU_CLKSEL_CON(i));
			else
				cru_write32(rksoc.clk_sel[i] | 0xffff0000,
					    CRU_CLKSEL_CON(i));
		}
	} else {
		/* pd_bus clk */
		cru_write32(rksoc.clk_sel[8]
				| REG_W_MSK(0, 0x1f)
				| REG_W_MSK(8, 0x3)
				| REG_W_MSK(12, 0x7),
				CRU_CLKSEL_CON(8));
		cru_write32(rksoc.clk_sel[27] | REG_W_MSK(0, 0x3f),
			    CRU_CLKSEL_CON(27));
		cru_write32(rksoc.clk_sel[31] | REG_W_MSK(0, 0x3f),
			    CRU_CLKSEL_CON(31));
		cru_write32(rksoc.clk_sel[53] | REG_W_MSK(0, 0x3f),
			    CRU_CLKSEL_CON(53));

		/* mcu clk */
		cru_write32(rksoc.clk_sel[12] | REG_W_MSK(0, 0x1f),
			    CRU_CLKSEL_CON(12));
		pmugrf_write32(rksoc.pmugrf_soc_con2 | REG_W_MSK(10, 0x7),
			       PMUGRF_SOC_CON(2));

		/* crypto */
		cru_write32(rksoc.clk_sel[6] | REG_W_MSK(0, 0x1f),
			    CRU_CLKSEL_CON(6));

		/* peri aclk hclk pclk */
		cru_write32(rksoc.clk_sel[9] | REG_W_MSK(0, 0x1f),
			    CRU_CLKSEL_CON(9));
		cru_write32(rksoc.clk_sel[11] | REG_W_MSK(0, 0x1f),
			    CRU_CLKSEL_CON(11));
		cru_write32(rksoc.clk_sel[29] | REG_W_MSK(0, 0xff),
			    CRU_CLKSEL_CON(29));
		cru_write32(rksoc.clk_sel[43] | REG_W_MSK(0, 0x1f),
			    CRU_CLKSEL_CON(43));

		/* pmu alive */
		cru_write32(rksoc.clk_sel[10]
				| REG_W_MSK(0, 0x1f)
				| REG_W_MSK(8, 0x1f)
				, CRU_CLKSEL_CON(10));
		/* core */
		cru_write32(rksoc.clk_sel[0]
				| REG_W_MSK(0, 0x1f) | REG_W_MSK(8, 0x1f)
				, CRU_CLKSEL_CON(0));
		cru_write32(rksoc.clk_sel[1]
				| REG_W_MSK(0, 0x1f) | REG_W_MSK(8, 0x1f)
				, CRU_CLKSEL_CON(1));

		cru_write32(rksoc.clk_sel[4] | REG_W_MSK(8, 0x1f),
			    CRU_CLKSEL_CON(4));
	}

	cru_write32(PLL_NORM_MODE, PLL_CONS((APLL_ID), 3));
	cru_write32(PLL_NORM_MODE, PLL_CONS((BPLL_ID), 3));
	cru_write32(PLL_NORM_MODE, PLL_CONS((WPLL_ID), 3));
	cru_write32(PLL_NORM_MODE, PLL_CONS((MPLL_ID), 3));
	cru_write32(PLL_NORM_MODE, PLL_CONS((GPLL_ID), 3));
	cru_write32(PLL_NORM_MODE, PLL_CONS((CPLL_ID), 3));
	cru_write32(PLL_NORM_MODE, PLL_CONS((NPLL_ID), 3));
}

void plls_32khz_config(void)
{
	if (!rkpm_chk_ctrbits(RKPM_CTR_PLL_DEEP))
		return;

	cru_write32(PLL_DEEP_MODE, PLL_CONS((NPLL_ID), 3));
	cru_write32(PLL_DEEP_MODE, PLL_CONS((CPLL_ID), 3));
	cru_write32(PLL_DEEP_MODE, PLL_CONS((GPLL_ID), 3));
	cru_write32(PLL_DEEP_MODE, PLL_CONS((MPLL_ID), 3));
	cru_write32(PLL_DEEP_MODE, PLL_CONS((WPLL_ID), 3));
	cru_write32(PLL_DEEP_MODE, PLL_CONS((BPLL_ID), 3));
	cru_write32(PLL_DEEP_MODE, PLL_CONS((APLL_ID), 3));
}

void restore_plls_24mhz(void)
{
	if (!rkpm_chk_ctrbits(RKPM_CTR_PLL_DEEP))
		return;

	cru_write32(PLL_SLOW_MODE, PLL_CONS((APLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((NPLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((CPLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((GPLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((MPLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((WPLL_ID), 3));
	cru_write32(PLL_SLOW_MODE, PLL_CONS((BPLL_ID), 3));
}

void regs_save(uint32_t *data, uint32_t base, uint32_t start, uint32_t end)
{
	uint32_t i, cnt = (end - start) / 4 + 1;

	for (i = 0; i < cnt; i++)
		data[i] = mmio_read_32(base + start + i * 4);
}

void regs_restore(uint32_t *data, uint32_t base,
		  uint32_t start, uint32_t end, uint32_t w_msk)
{
	uint32_t i, cnt = (end - start) / 4 + 1;

	for (i = 0; i < cnt; i++)
		mmio_write_32((base + start + i * 4), data[i] | w_msk);
}

static uint32_t i2c_phy[I2C_END] = {I2C_PMU_BASE, I2C_AUDIO_BASE};
static uint32_t i2c_pin[I2C_END] = {0x0a30, 0x4d10};

void gpio_set_in_output(uint8_t port, uint8_t bank, uint8_t bgpio, uint8_t type)
{
	uint32_t dir, val, b_gpio;

	bank -= 0xa;
	b_gpio = bank * 8 + bgpio;

	if (type == RKPM_GPIO_OUTPUT)
		dir = 0x1;
	else
		dir = 0x0;

	val = mmio_read_32(GPIO_BASE[port] + GPIO_SWPORT_DDR);
	val |= dir << b_gpio;
	mmio_write_32(GPIO_BASE[port] + GPIO_SWPORT_DDR, val);
}

static void i2c_suspend(uint32_t ch)
{
	uint32_t b_addr = i2c_phy[ch], pin = i2c_pin[ch];
	uint32_t i, port, bank, b_gpio;

	port = PIN_PORT(pin);
	bank = PIN_BANK(pin);
	b_gpio = PIN_IO(pin);

	/* sda, scl */
	for (i = 0; i < 2; i++) {
		pin_set_fun(port, bank, b_gpio + i, 0x0);
		gpio_set_in_output(port, bank, b_gpio + i, RKPM_GPIO_INPUT);
	}

	regs_save(&rksoc.i2c[ch][0], b_addr, 0x0, 0x18);
}

static void i2c_resume(int ch)
{
	uint32_t b_addr = i2c_phy[ch], pin = i2c_pin[ch];
	uint32_t i, port, bank, b_gpio;

	port = PIN_PORT(pin);
	bank = PIN_BANK(pin);
	b_gpio = PIN_IO(pin);

	for (i = 0; i < 2; i++)
		pin_set_fun(port, bank, b_gpio + i, PIN_FUN(0x01));

	regs_restore(&rksoc.i2c[ch][0], b_addr, 0x0, 0x18, 0x0);
}

/* gpio2(A-D) ~ gpio5(A-D): include reverved iomux */
void pins_to_gpio(void)
{
	uint32_t i, offset;

	/* iomux: gpio */
	for (i = 0; i < 16; i++) {
		offset = 0x10 + 4 * i;
		rksoc.iomux[i] = grf_read32(offset);
		if (offset == GPIO5A_IOMUX)
			grf_write32(0xfff00000, offset);/*uart2: 0x5a0, 0x5a1*/
		else
			grf_write32(0xffff0000, offset);
	}

	/* dir: input */
	for (i = 2; i < 6; i++) {
		rksoc.gpio_dir[i] = gpio_read32(BANK(i), GPIO_SWPORT_DDR);
		rksoc.gpio_data[i] = gpio_read32(BANK(i), GPIO_SWPORT_DR);
		gpio_write32(BANK(i), RKPM_GPIO_INPUT, GPIO_SWPORT_DDR);
	}
}

void restore_pins(void)
{
	uint32_t i, offset;

	/* dir & data*/
	for (i = 2; i < 6; i++) {
		gpio_write32(BANK(i), rksoc.gpio_dir[i], GPIO_SWPORT_DDR);
		gpio_write32(BANK(i), rksoc.gpio_data[i], GPIO_SWPORT_DR);
	}

	/* iomux */
	for (i = 0; i < 16; i++) {
		offset = 0x10 + 4 * i;
		grf_write32(0xffff0000 | rksoc.iomux[i], offset);
	}
}

static void suspend_ntimers(void)
{
	uint32_t i;

	for (i = 0; i < NTIME_CHS; i++) {
		if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF)) {
			rksoc.ntimer[i][0] =
				ntimer_read32(CHN(i), TIMER_LOADE_COUNT0);
			rksoc.ntimer[i][1] =
				ntimer_read32(CHN(i), TIMER_LOADE_COUNT1);
		}

		rksoc.ntimer[i][2] = ntimer_read32(CHN(i), TIMER_CONTROL_REG);
		ntimer_write32(CHN(i), 0, TIMER_CONTROL_REG);
	}
}

static void resume_ntimers(void)
{
	uint32_t i;

	for (i = 0; i < NTIME_CHS; i++) {
		if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF)) {
			ntimer_write32(CHN(i), rksoc.ntimer[i][0],
				       TIMER_LOADE_COUNT0);
			ntimer_write32(CHN(i), rksoc.ntimer[i][1],
				       TIMER_LOADE_COUNT1);
		}

		ntimer_write32(CHN(i), rksoc.ntimer[i][2], TIMER_CONTROL_REG);
	}
}

static void suspend_stimers(void)
{
	uint32_t i;

	for (i = 0; i < STIME_CHS; i++) {
		if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF)) {
			rksoc.stimer[i][0] =
				stimer_read32(CHN(i), TIMER_LOADE_COUNT0);
			rksoc.stimer[i][1] =
				stimer_read32(CHN(i), TIMER_LOADE_COUNT1);
		}

		rksoc.stimer[i][2] = stimer_read32(CHN(i), TIMER_CONTROL_REG);
		stimer_write32(CHN(i), 0, TIMER_CONTROL_REG);
	}

	/* disable stimer0,1 */
	sgrf_write32(0x00030003, SGRF_SOC_CON(3));
}

static void resume_stimers(void)
{
	uint32_t i;

	for (i = 0; i < STIME_CHS; i++) {
		if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF)) {
			stimer_write32(CHN(i), rksoc.stimer[i][0],
				       TIMER_LOADE_COUNT0);
			stimer_write32(CHN(i), rksoc.stimer[i][1],
				       TIMER_LOADE_COUNT1);
		}

		stimer_write32(CHN(i), rksoc.stimer[i][2], TIMER_CONTROL_REG);
	}

	/* enable stimer0,1 */
	sgrf_write32(0x00030000, SGRF_SOC_CON(3));
}

void suspend_timers(void)
{
	if (!rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF | RKPM_CTR_DIS_24M))
		return;

	suspend_stimers();
	suspend_ntimers();
}

void restore_timers(void)
{
	if (!rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF | RKPM_CTR_DIS_24M))
		return;

	resume_stimers();
	resume_ntimers();
}

void peri_suspend(void)
{
	if (rkpm_chk_ctrbits(RKPM_CTR_PD_PERI))
		i2c_suspend(I2C_PMU);

	if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGPD | RKPM_CTR_ARMOFF_LOGOFF))
		i2c_suspend(I2C_AUDIO);

	if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF))
		pins_to_gpio();
}

void restore_peri(void)
{
	if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF))
		restore_pins();

	if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGPD | RKPM_CTR_ARMOFF_LOGOFF))
		i2c_resume(I2C_AUDIO);

	if (rkpm_chk_ctrbits(RKPM_CTR_PD_PERI))
		i2c_resume(I2C_PMU);
}

int is_gpio_wakeup(void)
{
	return mmio_read_32(GPIO0_BASE + GPIO_INT_STATUS) ? 1 : 0;
}
