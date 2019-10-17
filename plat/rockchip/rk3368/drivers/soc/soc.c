/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
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

#include <arch_helpers.h>
#include <debug.h>
#include <mmio.h>
#include <platform_def.h>
#include <plat_private.h>
#include <rk3368_def.h>
#include <soc.h>

/* Aggregate of all devices in the first GB */
#define RK3368_DEV_RNG0_BASE	0xff000000
#define RK3368_DEV_RNG0_SIZE	0x00ff0000

/* Table of regions to map using the MMU. */
const mmap_region_t plat_rk_mmap[] = {
	MAP_REGION_FLAT(RK3368_DEV_RNG0_BASE, RK3368_DEV_RNG0_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(PMUSRAM_BASE, PMUSRAM_SIZE,
			MT_MEMORY | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(MCUOS_BASE, MCUOS_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(SHARE_MEM_BASE, SHARE_MEM_SIZE,
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
	/* No of children for the second cluster node */
	PLATFORM_CLUSTER1_CORE_COUNT
};

static uint64_t delay_per_us_cycles = DLY_PER_US_CYCL_800M;

void delay_time_calib_set(uint64_t calib)
{
	delay_per_us_cycles = calib;
}

uint64_t arch_counter_get_cntpct(void)
{
	uint64_t cval;

	isb();
	cval = read_cntpct_el0();
	return cval;
}

void delay_sycle(uint64_t sycles)
{
	while (sycles--)
		barrier();
}

void usdelay(uint32_t us)
{
	uint64_t sycles;

	sycles = (delay_per_us_cycles * us) / PER_US_CYSLES_JUST;

	while (sycles--)
		barrier();
}

void regs_updata_bits(uintptr_t addr, uint32_t val,
		      uint32_t mask, uint32_t shift)
{
	uint32_t tmp, orig;

	orig = mmio_read_32(addr);

	tmp = orig & ~(mask << shift);
	tmp |= (val & mask) << shift;

	if (tmp != orig)
		mmio_write_32(addr, tmp);
	dsb();
}

/******************************************************************
 *gpio func
 *pin=0x0a21  gpio0a2, port=0, bank=a, b_gpio=2, fun=1
 ******************************************************************/
void pin_set_fun(uint8_t port, uint8_t bank, uint8_t b_gpio, uint8_t fun)
{
	uint32_t off_set;

	bank -= 0xa;

	if (port > 0)
		off_set = GRF_BASE + (port - 1) * 16 + bank * 4;
	else
		off_set = PMU_GRF_BASE + PMUGRF_GPIO0_IOMUX(bank);

	mmio_write_32(off_set, BITS_WITH_WMASK(fun, 0x3, b_gpio * 2));
}

void pin_set_pull(uint8_t port, uint8_t bank, uint8_t b_gpio, uint8_t pull)
{
	uint32_t off_set;

	bank -= 0xa;

	if (port > 0)
		off_set = GRF_BASE + 0x100 + (port - 1) * 16 + bank * 4;
	else
		off_set = PMU_GRF_BASE + 0x10 + bank * 4;

	mmio_write_32(off_set, BITS_WITH_WMASK(pull, 0x3, b_gpio * 2));
}

static uint32_t gpio_base[4] = {
	PMU_GPIO0_BASE,
	GPIO1_BASE,
	GPIO2_BASE,
	GPIO3_BASE
};

void gpio_set_in_output(uint8_t port,
			uint8_t bank,
			uint8_t b_gpio,
			uint8_t type)
{
	uint8_t val;

	bank -= 0xa;
	b_gpio = bank * 8 + b_gpio;

	if (type == RKPM_GPIO_OUTPUT)
		val = 0x1;
	else
		val = 0;

	regs_updata_bits(gpio_base[port] + GPIO_SWPORT_DDR, val, 0x1, b_gpio);
}

/*
 * en : 1 enable, 0 disable
 * bank, b_gpio = 0xff, en ctrl all port gpios
 */
void gpio_set_inten(uint8_t port, uint8_t bank, uint8_t b_gpio, uint8_t en)
{
	uint32_t val;

	if (b_gpio == 0xff && bank == 0xff) {
		bank -= 0xa;
		b_gpio = bank * 8 + b_gpio;

		if (en == 1)
			val = 0xffffffff;
		else
			val = 0;

		mmio_write_32(gpio_base[port] + GPIO_INTEN, val);
	} else {
		bank -= 0xa;
		b_gpio = bank * 8 + b_gpio;

		if (en == 1)
			val = 0x1;
		else
			val = 0;

		regs_updata_bits(gpio_base[port] + GPIO_INTEN,
				 val,
				 0x1,
				 b_gpio);
	}
}

void gpio_int_clr(uint8_t port, uint8_t bank, uint8_t b_gpio)
{
	bank -= 0xa;
	b_gpio = bank * 8 + b_gpio;

	regs_updata_bits(gpio_base[port] + GPIO_PORTS_EOI, 0x1, 0x1, b_gpio);
}

void gpio_set_output_level(uint8_t port,
			   uint8_t bank,
			   uint8_t b_gpio,
			   uint8_t level)
{
	uint8_t val;

	bank -= 0xa;
	b_gpio = bank * 8 + b_gpio;

	if (level == RKPM_GPIO_OUT_H)
		val = 0x1;
	else
		val = 0;

	regs_updata_bits(gpio_base[port] + GPIO_SWPORT_DR, val, 0x1, b_gpio);
}

void __dead2 rockchip_soc_soft_reset(void)
{
	mmio_write_32(CRU_BASE + PLL_CONS((GPLL_ID), 3), PLL_SLOW_BITS);
	mmio_write_32(CRU_BASE + PLL_CONS((CPLL_ID), 3), PLL_SLOW_BITS);
	mmio_write_32(CRU_BASE + PLL_CONS((NPLL_ID), 3), PLL_SLOW_BITS);
	mmio_write_32(CRU_BASE + PLL_CONS((ABPLL_ID), 3), PLL_SLOW_BITS);
	mmio_write_32(CRU_BASE + PLL_CONS((ALPLL_ID), 3), PLL_SLOW_BITS);

	mmio_write_32(CRU_BASE + CRU_GLB_SRST_FST, CRU_RST_FIRST_TRIGER);
	dsb();

	/*
	 * Maybe the HW needs some times to reset the system,
	 * so we do not hope the core to excute valid codes.
	 */
	while (1)
	;
}

void rockchip_soc_soft_reset_config(void)
{
	uint32_t temp_val;

	mmio_write_32(PMU_GRF_BASE + PMUGRF_SOC_CON0,
		      BITS_WITH_WMASK(0, 1, 10));

	temp_val = mmio_read_32(CRU_BASE + CRU_GLB_RST_CON);
	/*
	 * Set pmu and wtd and tsadc is reseted by first reset!
	 */
	temp_val &= ~CRU_RST_PMU_MSK;
	temp_val |= CRU_RST_TSADC_FIRST | CRU_RST_WTD_FIRST;

	mmio_write_32(CRU_BASE + CRU_GLB_RST_CON, temp_val);
}

void secure_timer_init(void)
{
	mmio_write_32(STIMER1_BASE + TIMER_LOADE_COUNT0, 0xffffffff);
	mmio_write_32(STIMER1_BASE + TIMER_LOADE_COUNT1, 0xffffffff);

	/* auto reload & enable the timer */
	mmio_write_32(STIMER1_BASE + TIMER_CONTROL_REG, TIMER_EN);
}

void sgrf_init(void)
{
	/* setting all configurable ip into no-secure */
	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(5), SGRF_SOC_CON_NS);
	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(6), SGRF_SOC_CON7_BITS);
	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(7), SGRF_SOC_CON_NS);

	/* secure dma to no sesure */
	mmio_write_32(SGRF_BASE + SGRF_BUSDMAC_CON(0), SGRF_BUSDMAC_CON0_NS);
	mmio_write_32(SGRF_BASE + SGRF_BUSDMAC_CON(1), SGRF_BUSDMAC_CON1_NS);
	dsb();

	/* rst dma1 and mcu noc*/
	mmio_write_32(CRU_BASE + CRU_SOFTRSTS_CON(1),
		      BIT_WITH_WMSK(RST_DMA1_SHIFT) |
		      BIT_WITH_WMSK(RST_MCU_NOC_SHIFT));
	dsb();
	/* release dma1 and mcu noc rst*/
	mmio_write_32(CRU_BASE + CRU_SOFTRSTS_CON(1),
		      WMSK_BIT(RST_DMA1_SHIFT) | WMSK_BIT(RST_MCU_NOC_SHIFT));
}

void plat_rockchip_soc_init(void)
{
	secure_timer_init();
	rockchip_soc_soft_reset_config();
	sgrf_init();
	NOTICE("BL31:Rockchip release version: v%d.%d\n",
	       MAJOR_VERSION, MINOR_VERSION);
}
