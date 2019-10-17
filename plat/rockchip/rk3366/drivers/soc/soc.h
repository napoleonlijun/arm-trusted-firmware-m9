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

#ifndef __SOC_H__
#define __SOC_H__

#ifndef BIT
#define BIT(nr)			(1 << (nr))
#endif

/*****************************************************************************
 * secure timer
 *****************************************************************************/
#define TIMER_LOADE_COUNT0	0x00
#define TIMER_LOADE_COUNT1	0x04
#define TIMER_CURRENT_VALUE0	0x08
#define TIMER_CURRENT_VALUE1	0x0C
#define TIMER_CONTROL_REG	0x10
#define TIMER_INTSTATUS		0x18
#define TIMER_EN		0x1

#define STIMER1_BASE           (STIME_BASE + 0x20)
#define CHN(n)			(n)
#define NTIME_CHS		6
#define STIME_CHS		2
/*****************************************************************************
 * sgrf reg, offset
 *****************************************************************************/
#define DDR_SGRF_CON(n)		(0x0 + (n) * 4)

#define SGRF_SOC_CON(n)		(0x0 + (n) * 4)
#define SGRF_BUSDMAC_CON(n)	(0x100 + (n) * 4)

#define SGRF_BUSDMAC_CON0_NS	0xfffcfff8
#define SGRF_BUSDMAC_CON1_NS	0xffff0fff

#define CPU_BOOT_ADDR_WMASK	0xffff0000
#define CPU_BOOT_ADDR_ALIGN	16

/*****************************************************************************
 * cru reg, offset
 *****************************************************************************/
#define CRU_SOFTRST_CON		0x300
#define CRU_SOFTRSTS_CON(n)	(CRU_SOFTRST_CON + ((n) * 4))

#define SOFTRST_DMA1_MSK	0x4
#define SOFTRST_DMA2_MSK	0x0

void platform_soc_init(void);
extern const unsigned char rockchip_power_domain_tree_desc[];

/************************ info define ****************************************/
#define PM_INFO_LEVEL	1
#if PM_INFO_LEVEL > 0
#define PM_INFO(...)	tf_printf(__VA_ARGS__)
#else
#define PM_INFO(...)
#endif

#if PM_INFO_LEVEL > 1
#define PM_DBG(...)	tf_printf(__VA_ARGS__)
#else
#define PM_DBG(...)
#endif

/************************ read/write define ********************************/
#define REG_W_MSK(bits_shift, msk)	    ((msk) << ((bits_shift) + 16))
#define REG_SET_BITS(bits, bits_shift, msk) (((bits) & (msk)) << (bits_shift))
#define REG_WMSK_BITS(bits, bits_shift, msk) (REG_W_MSK(bits_shift, msk) |\
					    REG_SET_BITS(bits, bits_shift, msk))

#define pmu_read32(offset) mmio_read_32(PMU_BASE + offset)
#define pmu_write32(v, offset)\
	do { mmio_write_32(PMU_BASE + offset, v); dsb(); } while (0)

#define pmugrf_read32(offset) mmio_read_32(PMUGRF_BASE + offset)
#define pmugrf_write32(v, offset)\
	do { mmio_write_32(PMUGRF_BASE + offset, v); dsb(); } while (0)

#define cru_read32(offset) mmio_read_32(CRU_BASE + offset)
#define cru_write32(v, offset)\
	do { mmio_write_32(CRU_BASE + offset, v); dsb(); } while (0)

#define grf_read32(offset) mmio_read_32(GRF_BASE + offset)
#define grf_write32(v, offset)\
	do { mmio_write_32(GRF_BASE + offset, v); dsb(); } while (0)

#define sgrf_read32(offset) mmio_read_32(SGRF_BASE + offset)
#define sgrf_write32(v, offset)\
	do { mmio_write_32(SGRF_BASE + offset, v); dsb(); } while (0)

#define ddrsgrf_read32(offset) mmio_read_32(DDR_SGRF_BASE + offset)
#define ddrsgrf_write32(v, offset)\
	do { mmio_write_32(DDR_SGRF_BASE + offset, v); dsb(); } while (0)

#define stimer_read32(n, offset) mmio_read_32(STIME_BASE + 0x20 * n + offset)
#define stimer_write32(n, v, offset)\
	do { mmio_write_32(STIME_BASE + 0x20 * n + offset, v); dsb();} while (0)

#define ntimer_read32(n, offset) mmio_read_32(NTIME_BASE + 0x20 * n + offset)
#define ntimer_write32(n, v, offset)\
	do { mmio_write_32(NTIME_BASE + 0x20 * n + offset, v); dsb();} while (0)

/************************ cru define ******************************************/
enum plls_id {
	APLL_ID = 0,
	RESERVE,
	DPLL_ID,
	CPLL_ID,
	GPLL_ID,
	NPLL_ID,
	MPLL_ID,
	WPLL_ID,
	BPLL_ID,
	END_PLL = 9,
};

#define PLL_IS_LOCKED		(0x1 << 31)
#define PLL_IS_POWER_DOWN	(0x1 << 0)
#define PLL_CONS_CNT		4
#define PLL_CONS(id, i)		((id) * 0x10 + ((i) * 4))
#define PLL_SLOW_MODE		REG_WMSK_BITS(0, 8, 0x3)
#define PLL_NORM_MODE		REG_WMSK_BITS(1, 8, 0x3)
#define PLL_DEEP_MODE		REG_WMSK_BITS(2, 8, 0x3)
#define PLL_POWER_DOWN		REG_WMSK_BITS(1, 0, 0x1)
#define PLL_POWER_UP		REG_WMSK_BITS(0, 0, 0x1)
#define PLL_BYPASS		REG_WMSK_BITS(1, 1, 0x1)
#define PLL_NO_BYPASS		REG_WMSK_BITS(0, 1, 0x1)

#define SRST_FST_VALUE		0xfdb9
#define CRU_GLB_SRST_FST_VALUE  0x280
#define CRU_GLB_RST_CON		0x388
#define CRU_CLKSEL_NUMS		56
#define CRU_CLKSEL_CON(i)	(0x100 + ((i) * 4))
#define CRU_CLKGATE_NUMS	25
#define CRU_CLKGATE_CON(i)	(0x200 + ((i) * 4))
#define CRU_GATEID_CONS(ID)	(0x200 + (ID / 16) * 4)
#define CRU_CONS_GATEID(i)	(16 * (i))
#define GATE_ID(reg, bit)	((reg * 16) + bit)

#define CLK_GATING(msk, con)	mmio_write_32(CRU_BASE + con, \
					(msk << 16) | 0xffff)
#define CLK_UNGATING(msk, con)	mmio_write_32(CRU_BASE + con, \
					((~msk) << 16) | 0xffff)
#define CRU_UNGATING_OPS(id)	mmio_write_32(CRU_BASE + CRU_GATEID_CONS(id), \
					REG_WMSK_BITS(0, (id) % 16, 0x1))
#define CRU_GATING_OPS(id)	mmio_write_32(CRU_BASE + CRU_GATEID_CONS(id), \
					REG_WMSK_BITS(1, (id) % 16, 0x1))
/************************ axi qos define **************************************/
#define CPU_AXI_QOS_PRIORITY		0x08
#define CPU_AXI_QOS_MODE		0x0c
#define CPU_AXI_QOS_BANDWIDTH		0x10
#define CPU_AXI_QOS_SATURATION		0x14
#define CPU_AXI_QOS_EXTCONTROL		0x18
#define CPU_AXI_QOS_NUM_REGS		5

#define CPU_AXI_SAVE_QOS(array, base) do { \
	array[0] = mmio_read_32(base + CPU_AXI_QOS_PRIORITY); \
	array[1] = mmio_read_32(base + CPU_AXI_QOS_MODE); \
	array[2] = mmio_read_32(base + CPU_AXI_QOS_BANDWIDTH); \
	array[3] = mmio_read_32(base + CPU_AXI_QOS_SATURATION); \
	array[4] = mmio_read_32(base + CPU_AXI_QOS_EXTCONTROL); \
} while (0)

#define CPU_AXI_RESTORE_QOS(array, base) do { \
	mmio_write_32(base + CPU_AXI_QOS_PRIORITY, array[0]); \
	mmio_write_32(base + CPU_AXI_QOS_MODE, array[1]); \
	mmio_write_32(base + CPU_AXI_QOS_BANDWIDTH, array[2]); \
	mmio_write_32(base + CPU_AXI_QOS_SATURATION, array[3]); \
	mmio_write_32(base + CPU_AXI_QOS_EXTCONTROL, array[4]); \
} while (0)

#define SAVE_QOS(array, NAME)      CPU_AXI_SAVE_QOS(array,\
					CPU_AXI_##NAME##_QOS_BASE)
#define RESTORE_QOS(array, NAME)   CPU_AXI_RESTORE_QOS(array,\
					CPU_AXI_##NAME##_QOS_BASE)

/***************************  AXI QOS ****************************************/
#define SERVICE_DMA_BASE		0xffa80000
#define SERVICE_CPU_BASE		0xffa90000
#define SERVICE_WIFIBT_BASE		0xffaa0000
#define SERVICE_PERI_BASE		0xffab0000
#define SERVICE_MSCH_BASE		0xffac0000
#define SERVICE_VIO_BASE		0xffad0000
#define SERVICE_VIDEO_BASE		0xffae0000
#define SERVICE_GPU_BASE		0xffaf0000
/* service dma */
#define CPU_AXI_DMAC_QOS_BASE		(SERVICE_DMA_BASE + 0x0)
#define CPU_AXI_CRYPTO_QOS_BASE	        (SERVICE_DMA_BASE + 0x80)
#define CPU_AXI_DCF_QOS_BASE		(SERVICE_DMA_BASE + 0x100)
/* service wifibt */
#define CPU_AXI_BT_DMA_QOS_BASE		(SERVICE_WIFIBT_BASE + 0x000)
#define CPU_AXI_WIFI_DMA_QOS_BASE	(SERVICE_WIFIBT_BASE + 0x100)
/* service peri */
#define CPU_AXI_PERI0_QOS_BASE		(SERVICE_PERI_BASE + 0x0)
#define CPU_AXI_PERI1_QOS_BASE		(SERVICE_PERI_BASE + 0x80)
#define CPU_AXI_USB3_QOS_BASE		(SERVICE_PERI_BASE + 0x180)
/* service vio */
#define CPU_AXI_IEP_QOS_BASE		(SERVICE_VIO_BASE + 0x0)
#define CPU_AXI_ISP_R0_QOS_BASE		(SERVICE_VIO_BASE + 0x80)
#define CPU_AXI_ISP_R1_QOS_BASE		(SERVICE_VIO_BASE + 0x100)
#define CPU_AXI_ISP_W0_QOS_BASE		(SERVICE_VIO_BASE + 0x180)
#define CPU_AXI_ISP_W1_QOS_BASE		(SERVICE_VIO_BASE + 0x200)
#define CPU_AXI_VOP0_W_QOS_BASE		(SERVICE_VIO_BASE + 0x300)
#define CPU_AXI_RGA_R_QOS_BASE		(SERVICE_VIO_BASE + 0x380)
#define CPU_AXI_RGA_W_QOS_BASE		(SERVICE_VIO_BASE + 0x400)
#define CPU_AXI_VOP0_R_QOS_BASE		(SERVICE_VIO_BASE + 0x480)
#define CPU_AXI_VOP1_R_QOS_BASE		(SERVICE_VIO_BASE + 0x580)
#define CPU_AXI_HDCP_QOS_BASE		(SERVICE_VIO_BASE + 0x600)
/* service video */
#define CPU_AXI_RKVDEC_R_QOS_BASE	(SERVICE_VIDEO_BASE + 0x0)
#define CPU_AXI_RKVDEC_W_QOS_BASE	(SERVICE_VIDEO_BASE + 0x80)
#define CPU_AXI_VPU_R_QOS_BASE		(SERVICE_VIDEO_BASE + 0x100)
#define CPU_AXI_VPU_W_QOS_BASE		(SERVICE_VIDEO_BASE + 0x180)
/* service gpu */
#define CPU_AXI_GPU_QOS_BASE		(SERVICE_GPU_BASE + 0x0)
/* service cpu */
#define CPU_AXI_CPU_R_QOS_BASE		(SERVICE_CPU_BASE + 0x100)
#define CPU_AXI_CPU_W_QOS_BASE		(SERVICE_CPU_BASE + 0x180)
/* service bus */
#define CPU_AXI_MSCH_BASE		(SERVICE_MSCH_BASE + 0x0)
/************************ grf define ****************************************/
#define PMUGRF_SOC_CON_CNT	8
#define PMUGRF_SOC_CON(n)	(0x100 + (n) * 4)
#define GRF_SOC_CON(n)		(0x400 + (n) * 4)
#define PMUGRF_IOMUX(n)		(0x000 + (n) * 4)
#define PMUGRF_DLL_CON(n)	(0x180 + (n) * 4)
#define PMUGRF_DLL_STATUS(n)	(0x190 + (n) * 4)

/************************ peri define ****************************************/
#define PMU_IO_WKUP_DIS		0x0
#define PMU_IO_WKUP_P		0x1
#define PMU_IO_WKUP_N		0x2
#define PMU_IO_WKUP_PN		(PMU_IO_WKUP_P | PMU_IO_WKUP_N)

enum I2C_IDS {
	I2C_PMU = 0,
	I2C_AUDIO,
	I2C_END
};

#define GPIO5A_IOMUX 0x40

/************************ gpio define ****************************************/
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

#define PARAM_PIN(port, bank, io, fun)\
	((port << 12) | ((bank) << 8) | (io << 4) | (fun << 0))

/* GPIO control registers */
#define GPIO_SWPORT_DR		0x00
#define GPIO_SWPORT_DDR		0x04
#define GPIO_INTEN		0x30
#define GPIO_INTMASK		0x34
#define GPIO_INTTYPE_LEVE	0x38
#define GPIO_INT_POLARITY	0x3c
#define GPIO_INT_STATUS		0x40
#define GPIO_INT_RAWSTATUS	0x44
#define GPIO_DEBOUNCE		0x48
#define GPIO_PORTS_EOI		0x4c
#define GPIO_EXT_PORT		0x50
#define GPIO_LS_SYNC		0x60

/*************** pm ctrl bits define *********************************/
#define RKPM_CTR_ARMWFI		BIT(1)
#define RKPM_CTR_ARMPD		BIT(2)
#define RKPM_CTR_ARMOFF		BIT(3)
#define RKPM_CTR_ARMOFF_LOGPD	BIT(4)
#define RKPM_CTR_ARMOFF_LOGOFF	BIT(5)
#define RKPM_CTR_PWR_DMNS	BIT(6)
#define RKPM_CTR_PLLS		BIT(7)
#define RKPM_CTR_GTCLKS		BIT(8)
#define RKPM_CTR_PLL_DEEP	BIT(9)
#define RKPM_CTR_PMU_PD_PLL	BIT(10)
#define RKPM_CTR_PMU_EXT32K	BIT(11)
#define RKPM_CTR_PMU_PVTM32K	BIT(12)
#define RKPM_CTR_DIS_24M	BIT(13)
#define RKPM_CTR_PD_PERI	BIT(14)
#define RKPM_CTR_PD_WIFIBT	BIT(15)
#define RKPM_CTR_DDR_SREF	BIT(16)
#define RKPM_CTR_DDR_GATING	BIT(17)
#define RKPM_CTR_PMUDBG		BIT(18)
#define RKPM_CTR_AUTO_WAKEUP	BIT(19)

/*************** pmctrl bits define *********************************/
#define RESUME_ARMPLL_NRM	BIT(0)
#define RESUME_PLLS_SLOW	BIT(1)
#define RESUME_DDR		BIT(2)
#define RESUME_UARTDBG		BIT(3)

/***************** function ********************************/
void gating_clks(void);
void restore_clks(void);
void power_down_plls(void);
void restore_plls(void);
void rkpm_set_ctrbits(uint32_t val);
uint32_t rkpm_get_ctrbits(void);
uint32_t rkpm_chk_ctrbits(uint32_t bit);
void peri_suspend(void);
void restore_peri(void);
void plls_32khz_config(void);
void restore_plls_24mhz(void);
void suspend_timers(void);
void restore_timers(void);
int is_gpio_wakeup(void);

#endif /* __SOC_H__ */
