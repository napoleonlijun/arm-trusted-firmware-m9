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

#ifndef __SOC_H__
#define __SOC_H__

enum plls_id {
	ABPLL_ID = 0,
	ALPLL_ID,
	DPLL_ID,
	CPLL_ID,
	GPLL_ID,
	NPLL_ID,
	END_PLL_ID,
};

/***************************************************************************
 * secure timer
 ***************************************************************************/
#define RK_NTIMES 2
#define RK_NTIME_CHS 6
#define RK_STIME_CHS 2

#define TIMER_LOADE_COUNT0	0x00
#define TIMER_LOADE_COUNT1	0x04
#define TIMER_CURRENT_VALUE0	0x08
#define TIMER_CURRENT_VALUE1	0x0C
#define TIMER_CONTROL_REG	0x10
#define TIMER_INTSTATUS		0x18

#define TIMER_EN		0x1

#define STIMER1_BASE		(STIME_BASE + 0x20)

#define CYCL_24M_CNT_US(us)	(24 * us)
#define CYCL_24M_CNT_MS(ms)	(ms * CYCL_24M_CNT_US(1000))

/************************************************************************
 * sgrf reg, offset
 ************************************************************************/
#define SGRF_SOC_CON(n)		(0x0 + (n) * 4)
#define SGRF_BUSDMAC_CON(n)	(0x100 + (n) * 4)

#define SGRF_SOC_CON_NS		0xffff0000

/************************************************************************
 * con6[2]pmusram is security.
 * con6[6]stimer is security.
 ************************************************************************/
#define PMUSRAM_S_SHIFT		2
#define PMUSRAM_S		1
#define STIMER_S_SHIFT		6
#define STIMER_S		1
#define SGRF_SOC_CON7_BITS	((0xffff << 16) | \
				 (PMUSRAM_S << PMUSRAM_S_SHIFT) | \
				 (STIMER_S << STIMER_S_SHIFT))

#define SGRF_BUSDMAC_CON0_NS	0xfffcfff8
#define SGRF_BUSDMAC_CON1_NS	0xffff0fff

/*
 * sgrf_soc_con1~2, mask and offset
 */
#define CPU_BOOT_ADDR_WMASK	0xffff0000
#define CPU_BOOT_ADDR_ALIGN	16

/***************************************************************************
 * cru reg, offset
 ***************************************************************************/
#define CRU_SOFTRST_CON		0x300
#define CRU_SOFTRSTS_CON(n)	(CRU_SOFTRST_CON + ((n) * 4))
#define CRU_SOFTRSTS_CON_CNT	15

#define SOFTRST_DMA1		0x40004
#define SOFTRST_DMA2		0x10001

#define RST_DMA1_MSK		0x4
#define RST_DMA2_MSK		0x0

#define RST_DMA1_SHIFT		14
#define RST_MCU_NOC_SHIFT	2

#define CRU_CLKSEL_CON		0x100
#define CRU_CLKSELS_CON(i)	(CRU_CLKSEL_CON + ((i) * 4))
#define CRU_CLKSEL_CON_CNT	56

#define CRU_CLKGATE_CON		0x200
#define CRU_CLKGATES_CON(i)	(CRU_CLKGATE_CON + ((i) * 4))
#define CRU_CLKGATES_CON_CNT	25

#define CRU_GLB_SRST_FST	0x280
#define CRU_GLB_SRST_SND	0x284
#define CRU_GLB_RST_CON		0x388

#define CRU_CONS_GATEID(i)	(16 * (i))
#define GATE_ID(reg, bit)	((reg * 16) + bit)

#define CRU_RST_TSADC_FIRST	BIT(0)
#define CRU_RST_WTD_FIRST	BIT(1)

#define CRU_RST_PMU_FIRST	(0x0 << 2)
#define CRU_RST_PMU_SECOND	(0x1 << 2)
#define CRU_RST_PMU_NOT		(0x2 << 2)
#define CRU_RST_PMU_MSK		(0x3 << 2)

#define CRU_RST_FIRST_TRIGER		(0xfdb9)
#define CRU_RST_SECOND_TRIGER		(0xeca8)

/***************************************************************************
 * pll
 ***************************************************************************/
#define PLL_PWR_DN_MSK		BIT(1)
#define PLL_LOCK_MSK		BIT(31)
#define PLL_PWR_DN		BITS_WITH_WMASK(1, 0x1, 1)
#define PLL_PWR_ON		BITS_WITH_WMASK(0, 0x1, 1)
#define PLL_RESET		BITS_WITH_WMASK(1, 0x1, 5)
#define PLL_RESET_RESUME	BITS_WITH_WMASK(0, 0x1, 5)
#define PLL_BYPASS_MSK		BIT(0)
#define PLL_BYPASS_W_MSK	BITS_WMSK(PLL_BYPASS_MSK, 0)
#define PLL_BYPASS		BITS_WITH_WMASK(1, 0x1, 0)
#define PLL_NO_BYPASS		BITS_WITH_WMASK(0, 0x1, 0)
#define PLL_MODE_SHIFT		8
#define PLL_MODE_MSK		0x3
#define PLLS_MODE_WMASK		BITS_WMSK(PLL_MODE_MSK, PLL_MODE_SHIFT)
#define PLL_SLOW		0x0
#define PLL_NORM		0x1
#define PLL_DEEP		0x2
#define PLL_SLOW_BITS		BITS_WITH_WMASK(PLL_SLOW, 0x3, 8)
#define PLL_NORM_BITS		BITS_WITH_WMASK(PLL_NORM, 0x3, 8)
#define PLL_DEEP_BITS		BITS_WITH_WMASK(PLL_DEEP, 0x3, 8)

#define PLL_CONS(id, i)		((id) * 0x10 + ((i) * 4))

#define regs_updata_bit_set(addr, shift) \
		regs_updata_bits((addr), 0x1, 0x1, (shift))
#define regs_updata_bit_clr(addr, shift) \
		regs_updata_bits((addr), 0x0, 0x1, (shift))

void regs_updata_bits(uintptr_t addr, uint32_t val,
		      uint32_t mask, uint32_t shift);

/************************* gpio **********************************/
#define RKPM_GPIO_INPUT		(0)
#define RKPM_GPIO_OUTPUT	(1)

#define RKPM_GPIO_OUT_L		(0)
#define RKPM_GPIO_OUT_H		(1)

#define RKPM_GPIO_PULL_Z	(0)
#define RKPM_GPIO_PULL_UP	(0x1)
#define RKPM_GPIO_PULL_DN	(0x2)
#define RKPM_GPIO_PULL_RPTR	(0x3)

#define PIN_PORT(a)		((a >> 12) & 0xf)
#define PIN_BANK(a)		((a >> 8) & 0xf)
#define PIN_IO(a)		((a >> 4) & 0xf)
#define PIN_FUN(a)		((a) & 0xf)

#define PARAM_PIN(port, bank, io, fun) \
	((port << 12) | ((bank) << 8) | (io << 4) | (fun << 0))

/* GPIO control registers */
#define GPIO_SWPORT_DR		0x00
#define GPIO_SWPORT_DDR		0x04
#define GPIO_INTEN		0x30
#define GPIO_INTMASK		0x34
#define GPIO_INTTYPE_LEVEL	0x38
#define GPIO_INT_POLARITY	0x3c
#define GPIO_INT_STATUS		0x40
#define GPIO_INT_RAWSTATUS	0x44
#define GPIO_DEBOUNCE		0x48
#define GPIO_PORTS_EOI		0x4c
#define GPIO_EXT_PORT		0x50
#define GPIO_LS_SYNC		0x60

/************************************************************************
 *     TFW_DATA_BASE define
 ***********************************************************************/
/* base from TFW_DATA_BASE*/

#define TFW_DATA_MCUOS_BASE	(TFW_DATA_BASE)
#define TFW_DATA_MCUOS_SIZE	(64 * 1024)

#define TFW_DATA_IMEMBANK_BASE	(TFW_DATA_MCUOS_BASE + TFW_DATA_MCUOS_SIZE)
#define TFW_DATA_IMEMBANK_SIZE	(64 * 1024)

#define TFW_DATA_FIQ_BASE \
		(TFW_DATA_IMEMBANK_BASE + TFW_DATA_IMEMBANK_SIZE)
#define TFW_DATA_FIQ_SIZE	(8 * 1024)

#define TFW_DATA_MCUDATA_BASE	(TFW_DATA_FIQ_BASE + TFW_DATA_FIQ_SIZE)
#define TFW_DATA_MCUDATA_SIZE	(4 * 1024)

/****************************************************
 * pmu con, reg
 ***************************************************/
#define PM_PWRDM_CPUS_MSK	(0x1ef)
#define PM_PWRDM_SCUS_MSK	(0x210)

#define PM_PWRDM_CPUSL_MSK	(0xf)
#define PM_PWRDM_CPUSB_MSK	(0xf << 5)

#define PMU_CPU_WFE_MSK		(0x1)
#define PMU_CPU_WFI_MSK		(0x10)

#define CRU_GATEID_CONS(ID)	(CRU_CLKGATE_CON + (ID / 16) * 4)
#define CRU_UNGATING_OPS(id) \
	mmio_write_32(CRU_BASE + CRU_GATEID_CONS((id)), \
		      BITS_WITH_WMASK(0, 0x1, (id) % 16))
#define CRU_GATING_OPS(id) \
	mmio_write_32(CRU_BASE + CRU_GATEID_CONS((id)), \
		      BITS_WITH_WMASK(1, 0x1, (id) % 16))

#define CRU_RST1_MCU_BITS	0x3
#define CRU_RST1_MCU_BITS_SHIFT 12

/** mcu location **/
#define MCUOS_BASE		(TZRAM_BASE + 0x80000)
#define MCUOS_SIZE		(64 * 1024)

/****************************************************
 * pmu_grf con, reg
 ***************************************************/
#define PMU_PVTM_GATE_EN	BIT(3)

#define PMUGRF_SOC_CON0		0x100

#define PMUGRF_PVTM_CON0	0x180
#define PMUGRF_PVTM_CON1	0x184
#define PMUGRF_PVTM_ST0		0x190
#define PMUGRF_PVTM_ST1		0x194
#define PMUGRF_OS_REG0		0x200
#define PMUGRF_GPIO0_IOMUX(n)	((n) * 4)

enum rk3688_pmugrf_soc_con0 {
	pgrf_soc_32k_src = 6,
	pgrf_soc_pwm2_sel,
	pgrf_soc_ddrphy_bufen_core,
	pgrf_soc_ddrphy_bufen_io = 9,
	pgrf_soc_pmu_rst_hd
};

enum rk3688_pmugrf_pvtm_con0 {
	pgrf_pvtm_st = 0,
	pgrf_pvtm_en
};

/**************************** periph ctrl ******************************/
#define PER_TIMER_CNT_NS	(41)
#define PER_US_CYSLES_JUST	(1000)
#define DLY_PER_US_CYCL_800M	(135 * PER_US_CYSLES_JUST)
#define DLY_PER_US_CYCL_24M	(8 * PER_US_CYSLES_JUST)

#define barrier()		__asm__ __volatile__("" : : : "memory")
#define nop()			__asm__ __volatile__("nop")

void pin_set_fun(uint8_t port, uint8_t bank, uint8_t b_gpio, uint8_t fun);
void gpio_set_in_output(uint8_t port,
			uint8_t bank,
			uint8_t b_gpio,
			uint8_t type);
void peri_pin_to_gpio(uint32_t cfg);
void peri_pin_to_restore(uint32_t cfg);

void delay_time_calib_set(uint64_t calib);
uint64_t arch_counter_get_cntpct(void);
void delay_sycle(uint64_t sycles);
void usdelay(uint32_t us);

#endif /* __SOC_H__ */
