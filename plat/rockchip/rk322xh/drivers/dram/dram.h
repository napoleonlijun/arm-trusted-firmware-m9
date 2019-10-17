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

#ifndef __SOC_ROCKCHIP_RK3X2X_DRAM_H__
#define __SOC_ROCKCHIP_RK3X2X_DRAM_H__
#include <plat_private.h>
#include <stdint.h>

#define KHZ	(1000)
#define MHZ	(1000 * KHZ)
#define GHZ	(1000 * MHZ)

#define DLL_ON_2_ON	0
#define DLL_ON_2_OFF	1
#define DLL_OFF_2_ON	2
#define DLL_OFF_2_OFF	3

#define SYNC_WITH_LCDC_FRAME_INTR
#define PHY_READ_DQS_DLL_SWITCH_FREQ	(680)

#define DDR_PHY_BASE_ADDR		DDR_PHY_BASE
#define DDRC_BASE_ADDR			DDR_UPCTL_BASE
#define SERVER_MSCH0_BASE_ADDR		SERVER_MSCH_BASE
#define UMCTL2_REGS_FREQ(n)		(DDRC_BASE_ADDR + (0x2000 * (n)))
/* ddr phy registers define */
#define PHY_REG(n)			(DDR_PHY_BASE_ADDR + 4 * (n))

/* VOP */
#define VOP_SYS_CTRL			(0x8)
#define VOP_INTR_CLEAR0			(0xe4)
#define VOP_INTR_RAW_STATUS0		(0xec)
#define VOP_WIN0_CTRL0			(0x100)
#define VOP_WIN1_CTRL0			(0x200)
#define VOP_WIN2_CTRL0			(0x300)

#define WIN_EN				(1)
#define VOP_CLEAR_FLAG0			((0x1 << (16 + 3)) | (0x1 << 3))
#define VOP_CLEAR_FLAG1			((0x1 << (16 + 4)) | (0x1 << 4))
#define VOP_STAND_BY			(1 << 22)
#define VOP_FLAG1_STATUS		(1 << 4)
#define VOP_FLAG0_STATUS		(1 << 3)

/* CRU */
/* CRU_PLL_CON0 */
#define POSTDIV1_SHIFT			(12)
#define POSTDIV1_MASK			(7)
#define FBDIV_MASK			(0xFFF)
#define PB(n)				((0x1 << (15 + 16)) | ((n) << 15))
#define POSTDIV1(n)			((POSTDIV1_MASK <<\
						(POSTDIV1_SHIFT + 16)) |\
						((n) << POSTDIV1_SHIFT))
#define FBDIV(n)			((FBDIV_MASK << 16) | (n))

/* CRU_PLL_CON1 */
#define RSTMODE(n)			((0x1 << (15 + 16)) | ((n) << 15))
#define RST(n)				((0x1 << (14 + 16)) | ((n) << 14))
#define PD(n)				((0x1 << (13 + 16)) | ((n) << 13))
#define DSMPD(n)			((0x1 << (12 + 16)) | ((n) << 12))
#define LOCK(n)				(((n) >> 10) & 0x1)
#define POSTDIV2_SHIFT			(6)
#define POSTDIV2_MASK			(7)
#define POSTDIV2(n)			((POSTDIV2_MASK <<\
						(POSTDIV2_SHIFT + 16)) |\
						((n) << POSTDIV2_SHIFT))
#define REFDIV_MASK			(0x3f)
#define REFDIV(n)			((REFDIV_MASK << 16) | (n))
/* CRU_CRU_MODE */
#define DPLL_NORMAL_MODE		(((0x1 << 4) << 16) | (1 << 4))
#define DPLL_SLOW_MODE			(((0x1 << 4) << 16) | (0 << 4))

/* CRU_CLKGATE_CON18 */
#define DDR_MSCH_UPCTL_EN_MASK		((0x7 << 4) << 16)
#define DDR_MSCH_UPCTL_EN_SHIFT		(4)
/* CRU_CLKGATE_CON21 */
#define HCLK_VOP_EN			(1 << 3)
#define CLK_DISABLE			(1)

/* GRF */
/* GRF_SOC_CON2 */
#define GRF_CON_DDRPHY_BUFFEREN_MASK	(0x3 << (12 + 16))
#define GRF_CON_DDRPHY_BUFFEREN_EN	(0x2 << 12)
#define GRF_CON_DDRPHY_BUFFEREN_DIS	(0x1 << 12)
/* GRF_SOC_CON5 */
#define MSCH_PWR_IDLEREQ_MASK		(1 << (3 + 16))
#define MSCH_PWR_IDLEREQ_EN		(1 << 3)
#define MSCH_PWR_IDLEREQ_DIS		(0 << 3)
#define OTHER_MASTER_STALL_RESPONSE	((0x1f << (10 + 16)) | 0x1f << 10)
/* GRF_SOC_CON6 */
#define MSCH_SRV_FW_FWR			(1 << 13)
#define MSCH_FWR_LINK			(1 << 12)
#define CORE_FWR_BUS_LINK		(1 << 9)
#define CORE_REQ_LINK			(1 << 6)
/* GRF_SOC_STATUS0 */
#define GRF_DDR_PLLLOCK_SHIFT		(5)
#define GRF_DDR_PLLLOCK_MASK		(1)
/* GRF_SOC_STATUS1 */
#define MSCH_PWR_IDLE			(1 << 13)

/* ddr pctl registers define */
#define DDR_PCTL2_MSTR			0x0
#define DDR_PCTL2_STAT			0x4
#define DDR_PCTL2_MSTR1			0x8
#define DDR_PCTL2_MRCTRL0		0x10
#define DDR_PCTL2_MRCTRL1		0x14
#define DDR_PCTL2_MRSTAT		0x18
#define DDR_PCTL2_MRCTRL2		0x1c
#define DDR_PCTL2_DERATEEN		0x20
#define DDR_PCTL2_DERATEINT		0x24
#define DDR_PCTL2_PWRCTL		0x30
#define DDR_PCTL2_PWRTMG		0x34
#define DDR_PCTL2_HWLPCTL		0x38
#define DDR_PCTL2_RFSHCTL0		0x50
#define DDR_PCTL2_RFSHCTL1		0x54
#define DDR_PCTL2_RFSHCTL2		0x58
#define DDR_PCTL2_RFSHCTL4		0x5c
#define DDR_PCTL2_RFSHCTL3		0x60
#define DDR_PCTL2_RFSHTMG		0x64
#define DDR_PCTL2_RFSHTMG1		0x68
#define DDR_PCTL2_RFSHCTL5		0x6c
#define DDR_PCTL2_INIT0			0xd0
#define DDR_PCTL2_INIT1			0xd4
#define DDR_PCTL2_INIT2			0xd8
#define DDR_PCTL2_INIT3			0xdc
#define DDR_PCTL2_INIT4			0xe0
#define DDR_PCTL2_INIT5			0xe4
#define DDR_PCTL2_INIT6			0xe8
#define DDR_PCTL2_INIT7			0xec
#define DDR_PCTL2_DIMMCTL		0xf0
#define DDR_PCTL2_RANKCTL		0xf4
#define DDR_PCTL2_CHCTL			0xfc
#define DDR_PCTL2_DRAMTMG0		0x100
#define DDR_PCTL2_DRAMTMG1		0x104
#define DDR_PCTL2_DRAMTMG2		0x108
#define DDR_PCTL2_DRAMTMG3		0x10c
#define DDR_PCTL2_DRAMTMG4		0x110
#define DDR_PCTL2_DRAMTMG5		0x114
#define DDR_PCTL2_DRAMTMG6		0x118
#define DDR_PCTL2_DRAMTMG7		0x11c
#define DDR_PCTL2_DRAMTMG8		0x120
#define DDR_PCTL2_DRAMTMG9		0x124
#define DDR_PCTL2_DRAMTMG10		0x128
#define DDR_PCTL2_DRAMTMG11		0x12c
#define DDR_PCTL2_DRAMTMG12		0x130
#define DDR_PCTL2_DRAMTMG13		0x134
#define DDR_PCTL2_DRAMTMG14		0x138
#define DDR_PCTL2_DRAMTMG15		0x13c
#define DDR_PCTL2_DRAMTMG16		0x140
#define DDR_PCTL2_ZQCTL0		0x180
#define DDR_PCTL2_ZQCTL1		0x184
#define DDR_PCTL2_ZQCTL2		0x188
#define DDR_PCTL2_ZQSTAT		0x18c
#define DDR_PCTL2_DFITMG0		0x190
#define DDR_PCTL2_DFITMG1		0x194
#define DDR_PCTL2_DFILPCFG0		0x198
#define DDR_PCTL2_DFILPCFG1		0x19c
#define DDR_PCTL2_DFIUPD0		0x1a0
#define DDR_PCTL2_DFIUPD1		0x1a4
#define DDR_PCTL2_DFIUPD2		0x1a8
#define DDR_PCTL2_DFIMISC		0x1b0
#define DDR_PCTL2_DFITMG2		0x1b4
#define DDR_PCTL2_DFITMG3		0x1b8
#define DDR_PCTL2_DFISTAT		0x1bc
#define DDR_PCTL2_DBICTL		0x1c0
#define DDR_PCTL2_ADDRMAP0		0x200
#define DDR_PCTL2_ADDRMAP1		0x204
#define DDR_PCTL2_ADDRMAP2		0x208
#define DDR_PCTL2_ADDRMAP3		0x20c
#define DDR_PCTL2_ADDRMAP4		0x210
#define DDR_PCTL2_ADDRMAP5		0x214
#define DDR_PCTL2_ADDRMAP6		0x218
#define DDR_PCTL2_ADDRMAP7		0x21c
#define DDR_PCTL2_ADDRMAP8		0x220
#define DDR_PCTL2_ADDRMAP9		0x224
#define DDR_PCTL2_ADDRMAP10		0x228
#define DDR_PCTL2_ADDRMAP11		0x22c
#define DDR_PCTL2_ODTCFG		0x240
#define DDR_PCTL2_ODTMAP		0x244
#define DDR_PCTL2_SCHED			0x250
#define DDR_PCTL2_SCHED1		0x254
#define DDR_PCTL2_PERFHPR1		0x25c
#define DDR_PCTL2_PERFLPR1		0x264
#define DDR_PCTL2_PERFWR1		0x26c
#define DDR_PCTL2_DQMAP0		0x280
#define DDR_PCTL2_DQMAP1		0x284
#define DDR_PCTL2_DQMAP2		0x288
#define DDR_PCTL2_DQMAP3		0x28c
#define DDR_PCTL2_DQMAP4		0x290
#define DDR_PCTL2_DQMAP5		0x294
#define DDR_PCTL2_DBG0			0x300
#define DDR_PCTL2_DBG1			0x304
#define DDR_PCTL2_DBGCAM		0x308
#define DDR_PCTL2_DBGCMD		0x30c
#define DDR_PCTL2_DBGSTAT		0x310
#define DDR_PCTL2_SWCTL			0x320
#define DDR_PCTL2_SWSTAT		0x324
#define DDR_PCTL2_POISONCFG		0x36c
#define DDR_PCTL2_POISONSTAT		0x370
#define DDR_PCTL2_ADVECCINDEX		0x374
#define DDR_PCTL2_ADVECCSTAT		0x378
#define DDR_PCTL2_PSTAT			0x3fc
#define DDR_PCTL2_PCCFG			0x400
#define DDR_PCTL2_PCFGRN		0x404
#define DDR_PCTL2_PCFGWN		0x408
#define DDR_PCTL2_PCTRLN		0x490

/* PCTL2_MRSTAT */
#define PCTL2_FREQUENCY_MODE_MASK	(1)
#define PCTL2_FREQUENCY_MODE_SHIFT	(29)
#define PCTL2_DLL_OFF_MODE		(1 << 15)
#define PCTL2_MR_WR_BUSY		(1 << 0)
/* PCTL2_STAT */
#define PCTL2_SELFREF_TYPE_MASK		(3 << 4)
#define PCTL2_SELFREF_TYPE_SR_NOT_AUTO	(2 << 4)
#define PCTL2_OPERATING_MODE_MASK	(7)
#define PCTL2_OPERATING_MODE_INIT	(1)
#define PCTL2_OPERATING_MODE_SR		(3)
/* PCTL2_MRCTRL0 */
#define PCTL2_MR_WR			(1u << 31)
#define PCTL2_MR_ADDR_SHIFT		(12)
#define PCTL2_MR_RANK_SHIFT		(4)
#define PCTL2_MR_TYPE_WR		(0)
#define PCTL2_MR_TYPE_RD		(1)
/* PCTL2_MRCTRL1 */
#define PCTL2_MR_ADDRESS_SHIFT		(8)
#define PCTL2_MR_DATA_MASK		(0xff)
/* PCTL2_DERATEEN */
#define PCTL2_DERATE_ENABLE		(1)
/* PCTL2_PWRCTL */
#define PCTL2_SELFREF_SW		(1 << 5)
#define PCTL2_POWERDOWN_EN		(1 << 1)
#define PCTL2_SELFREF_EN		(1)
/* PCTL2_PWRTMG */
#define PCTL2_SELFREF_TO_X32_MASK	(0xFF)
#define PCTL2_SELFREF_TO_X32_SHIFT	(16)
#define PCTL2_POWERDOWN_TO_X32_MASK	(0x1F)
/* PCTL2_INIT3 */
#define PCTL2_MR0_SHIFT			(16)
#define PCTL2_LPDDR3_MR1_SHIFT		(16)
/* PCTL2_INIT4 */
#define PCTL2_MR2_SHIFT			(16)
#define PCTL2_LPDDR3_MR3_SHIFT		(16)
/* PCTL2_INIT6 */
#define PCTL2_MR4_SHIFT			(16)
#define PCTL2_MR_MASK			(0xffff)
/* PCTL2_RFSHCTL3 */
#define PCTL2_DIS_AUTO_REFRESH		(1)
/* PCTL2_ZQCTL0 */
#define PCTL2_DIS_AUTO_ZQ		(1ul << 31)
#define PCTL2_DIS_SRX_ZQCL		(1 << 30)
/* PCTL2_DFILPCFG0 */
#define PCTL2_DFI_LP_EN_SR		(1 << 8)
#define PCTL2_DFI_LP_EN_SR_MASK		(1 << 8)
#define PCTL2_DFI_LP_EN_SR_SHIFT	(8)
/* PCTL2_DFIMISC */
#define PCTL2_DFI_INIT_COMPLETE_EN	(1)
/* PCTL2_DFISTAT */
#define PCTL2_DFI_LP_ACK		(1 << 1)
#define PCTL2_DFI_INIT_COMPLETE		(1)
/* PCTL2_DBG1 */
#define PCTL2_DIS_HIF			(1 << 1)
/* PCTL2_DBGCAM */
#define PCTL2_DBG_WR_Q_EMPTY		(1 << 26)
#define PCTL2_DBG_RD_Q_EMPTY		(1 << 25)
#define PCTL2_DBG_LPR_Q_DEPTH_MASK	(0xffff << 8)
#define PCTL2_DBG_LPR_Q_DEPTH_EMPTY	(0x0 << 8)
/* PCTL2_DBGCMD */
#define PCTL2_RANK1_REFRESH		(1 << 1)
#define PCTL2_RANK0_REFRESH		(1)
/* PCTL2_DBGSTAT */
#define PCTL2_RANK1_REFRESH_BUSY	(1 << 1)
#define PCTL2_RANK0_REFRESH_BUSY	(1)
/* PCTL2_SWCTL */
#define PCTL2_SW_DONE			(1)
#define PCTL2_SW_DONE_CLEAR		(0)
/* PCTL2_SWSTAT */
#define PCTL2_SW_DONE_ACK		(1)
/* PCTL2_PSTAT */
#define PCTL2_WR_PORT_BUSY_0		(1 << 16)
#define PCTL2_RD_PORT_BUSY_0		(1)
/* PCTL2_PCTRLn */
#define PCTL2_PORT_EN			(1)

/* PHYREG00 */
#define PHY_DIGITAL_DERESET		(1 << 3)
#define PHY_ANALOG_DERESET		(1 << 2)
#define PHY_CHANNEL_SELECT_SHIFT	(4)
#define PHY_CHANNEL_SELECT_MASK		(0xf)
#define PHY_CHANNEL_SELECT_16		(3)
/* PHYREG01 */
#define PHY_DDR2			(0)
#define PHY_LPDDR2			(1)
#define PHY_DDR3			(2)
#define PHY_LPDDR3			(3)
#define PHY_DDR4			(4)
#define PHY_BL_4			(0 << 2)
#define PHY_BL_8			(1 << 2)
/* PHYREG02 */
#define PHY_DTT_EN			(1 << 0)
#define PHY_DTT_DISB			(0 << 0)
#define PHY_WRITE_LEVELING_EN		(1 << 2)
#define PHY_WRITE_LEVELING_DISB		(0 << 2)
#define PHY_SELECT_CS0			(2)
#define PHY_SELECT_CS1			(1)
#define PHY_SELECT_CS0_1		(0)
#define PHY_WRITE_LEVELING_SELECTCS(n)	(n << 6)
#define PHY_DATA_TRAINING_SELECTCS(n)	(n << 4)
#define PHY_GATING_CALIBRATION_MASK	(0x33)
#define PHY_GATING_CALIBRATION_EN	(1)
#define PHY_GATING_CALIBRATION_DIS	(0)
#define PHY_GATING_CALIBRATION_CS0	(0x20)
/* PHYREG12 */
#define PHY_CMD_PRCOMP_MASK		(0x1f)
#define PHY_CMD_PRCOMP_SHIFT		(3)
/* PHYREG13 */
#define PHY_CMD_DLL_BYPASS_90		(1 << 4)
/* PHYREG14 */
#define PHY_CK_DLL_BYPASS_90		(1 << 3)
/* PHYREG26 */
#define PHY_DQ_DLL_BYPASS_90		(1 << 4)
/* PHYREG27 */
#define PHY_DQS_DLL_BYPASS_90		(1 << 3)
/* PHYREG28 */
#define PHY_READ_DQS_DLL_22_5		(1)
#define PHY_READ_DQS_DLL_45		(2)
/* PHYREG29 */
#define PHY_WEAK_PULL_UP		(1 << 1)
#define PHY_WEAK_PULL_DOWN		(1)
/* PHYREGA4 */
#define PHY_DLL_BYPASS_MODE		(0x1F)
/* PHYREGED */
#define PHY_PLL_PD			(1 << 1)
/* PHYREGEE */
#define PHY_PLL_PRE_DIV_MASK		(0x1f)
/* PHYREGEF */
#define PHY_INTER_PLL			(1 << 7)
#define PHY_PLL_POST_DIV_MASK		(7)
/* PHYREG106 */
#define PHY_PRE_PLL_FB_DIV_11_8_MASK	(0xf)
#define PHY_PRE_PLL_FB_DIV_11_8_SHIFT	(8)
/* PHYREG107 */
#define PHY_PRE_PLL_FB_DIV_7_0_MASK	(0xff)

#define PHY_DDR3_RON_RTT_DISABLE	(0)
#define PHY_DDR3_RON_RTT_451ohm		(1)
#define PHY_DDR3_RON_RTT_225ohm		(2)
#define PHY_DDR3_RON_RTT_150ohm		(3)
#define PHY_DDR3_RON_RTT_112ohm		(4)
#define PHY_DDR3_RON_RTT_90ohm		(5)
#define PHY_DDR3_RON_RTT_75ohm		(6)
#define PHY_DDR3_RON_RTT_64ohm		(7)
#define PHY_DDR3_RON_RTT_56ohm		(16)
#define PHY_DDR3_RON_RTT_50ohm		(17)
#define PHY_DDR3_RON_RTT_45ohm		(18)
#define PHY_DDR3_RON_RTT_41ohm		(19)
#define PHY_DDR3_RON_RTT_37ohm		(20)
#define PHY_DDR3_RON_RTT_34ohm		(21)
#define PHY_DDR3_RON_RTT_33ohm		(22)
#define PHY_DDR3_RON_RTT_30ohm		(23)
#define PHY_DDR3_RON_RTT_28ohm		(24)
#define PHY_DDR3_RON_RTT_26ohm		(25)
#define PHY_DDR3_RON_RTT_25ohm		(26)
#define PHY_DDR3_RON_RTT_23ohm		(27)
#define PHY_DDR3_RON_RTT_22ohm		(28)
#define PHY_DDR3_RON_RTT_21ohm		(29)
#define PHY_DDR3_RON_RTT_20ohm		(30)
#define PHY_DDR3_RON_RTT_19ohm		(31)

#define PHY_DDR4_LPDDR3_RON_RTT_DISABLE (0)
#define PHY_DDR4_LPDDR3_RON_RTT_480ohm	(1)
#define PHY_DDR4_LPDDR3_RON_RTT_240ohm	(2)
#define PHY_DDR4_LPDDR3_RON_RTT_160ohm	(3)
#define PHY_DDR4_LPDDR3_RON_RTT_120ohm	(4)
#define PHY_DDR4_LPDDR3_RON_RTT_96ohm	(5)
#define PHY_DDR4_LPDDR3_RON_RTT_80ohm	(6)
#define PHY_DDR4_LPDDR3_RON_RTT_68ohm	(7)
#define PHY_DDR4_LPDDR3_RON_RTT_60ohm	(16)
#define PHY_DDR4_LPDDR3_RON_RTT_53ohm	(17)
#define PHY_DDR4_LPDDR3_RON_RTT_48ohm	(18)
#define PHY_DDR4_LPDDR3_RON_RTT_43ohm	(19)
#define PHY_DDR4_LPDDR3_RON_RTT_40ohm	(20)
#define PHY_DDR4_LPDDR3_RON_RTT_37ohm	(21)
#define PHY_DDR4_LPDDR3_RON_RTT_34ohm	(22)
#define PHY_DDR4_LPDDR3_RON_RTT_32ohm	(23)
#define PHY_DDR4_LPDDR3_RON_RTT_30ohm	(24)
#define PHY_DDR4_LPDDR3_RON_RTT_28ohm	(25)
#define PHY_DDR4_LPDDR3_RON_RTT_26ohm	(26)
#define PHY_DDR4_LPDDR3_RON_RTT_25ohm	(27)
#define PHY_DDR4_LPDDR3_RON_RTT_24ohm	(28)
#define PHY_DDR4_LPDDR3_RON_RTT_22ohm	(29)
#define PHY_DDR4_LPDDR3_RON_RTT_21ohm	(30)
#define PHY_DDR4_LPDDR3_RON_RTT_20ohm	(31)

/* noc registers define */
#define DDRCONF				0x8
#define DDRTIMING			0xc
#define DDRMODE				0x10
#define READLATENCY			0x14
#define AGING0				0x18
#define AGING1				0x1c
#define AGING2				0x20
#define AGING3				0x24
#define AGING4				0x28
#define AGING5				0x2c
#define ACTIVATE			0x38
#define DEVTODEV			0x3c
#define DDR4TIMING			0x40

/*
 * sys_reg bitfield struct
 * [31]		row_3_4_ch1
 * [30]		row_3_4_ch0
 * [29:28]	chinfo
 * [27]		rank_ch1
 * [26:25]	col_ch1
 * [24]		bk_ch1
 * [23:22]	cs0_row_ch1
 * [21:20]	cs1_row_ch1
 * [19:18]	bw_ch1
 * [17:16]	dbw_ch1;
 * [15:13]	ddrtype
 * [12]		channelnum
 * [11]		rank_ch0
 * [10:9]	col_ch0
 * [8]		bk_ch0
 * [7:6]	cs0_row_ch0
 * [5:4]	cs1_row_ch0
 * [3:2]	bw_ch0
 * [1:0]	dbw_ch0
*/
#define SYS_REG_ENC_ROW_3_4(n)		((n) << 30)
#define SYS_REG_DEC_ROW_3_4(n)		((n >> 30) & 0x1)
#define SYS_REG_ENC_CHINFO()		(1 << 28)
#define SYS_REG_ENC_DDRTYPE(n)		((n) << 13)
#define SYS_REG_DEC_DDRTYPE(n)		(((n) >> 13) & 0x7)
#define SYS_REG_ENC_NUM_CH(n)		(((n) - 1) << 12)
#define SYS_REG_DEC_NUM_CH(n)		(1 + ((n >> 12) & 0x1))
#define SYS_REG_ENC_RANK(n)		(((n) - 1) << 11)
#define SYS_REG_DEC_RANK(n)		(1 + ((n >> 11) & 0x1))
#define SYS_REG_ENC_COL(n)		(((n) - 9) << 9)
#define SYS_REG_DEC_COL(n)		(9 + ((n >> 9) & 0x3))
#define SYS_REG_ENC_BK(n)		(((n) == 3 ? 0 : 1) << 8)
#define SYS_REG_DEC_BK(n)		(3 - ((n >> 8) & 0x1))
#define SYS_REG_ENC_CS0_ROW(n)		(((n) - 13) << 6)
#define SYS_REG_DEC_CS0_ROW(n)		(13 + ((n >> 6) & 0x3))
#define SYS_REG_ENC_CS1_ROW(n)		(((n) - 13) << 4)
#define SYS_REG_DEC_CS1_ROW(n)		(13 + ((n >> 4) & 0x3))
#define SYS_REG_ENC_BW(n)		((2 >> (n)) << 2)
#define SYS_REG_DEC_BW(n)		(2 >> ((n >> 2) & 0x3))
#define SYS_REG_ENC_DBW(n)		((2 >> (n)) << 0)
#define SYS_REG_DEC_DBW(n)		(2 >> ((n >> 0) & 0x3))

union noc_ddrtiming {
	uint32_t d32;
	struct {
		unsigned acttoact : 6;
		unsigned rdtomiss : 6;
		unsigned wrtomiss : 6;
		unsigned burstlen : 3;
		unsigned rdtowr : 5;
		unsigned wrtord : 5;
		unsigned bwratio : 1;
	} b;
} NOC_TIMING_T;

union noc_activate {
	uint32_t d32;
	struct {
		unsigned rrd : 4;
		unsigned faw : 6;
		unsigned fawbank : 1;
		unsigned reserved1 : 21;
	} b;
};

union noc_devtodev {
	uint32_t d32;
	struct {
		unsigned busrdtord : 2;
		unsigned busrdtowr : 2;
		unsigned buswrtord : 2;
		unsigned reserved2 : 26;
	} b;
};

union noc_ddr4timing {
	uint32_t d32;
	struct {
		unsigned ccdl : 3;
		unsigned wrtordl : 5;
		unsigned rrdl : 4;
		unsigned reserved2 : 20;
	} b;
};


union noc_ddrmode {
	uint32_t d32;
	struct {
		unsigned autoprecharge : 1;
		unsigned bwratioextended : 1;
		unsigned reserved3 : 30;
	} b;
};

struct rk3328_msch_timings {
	union noc_ddrtiming ddrtiming;
	union noc_ddrmode ddrmode;
	uint32_t readlatency;
	union noc_activate activate;
	union noc_devtodev devtodev;
	union noc_ddr4timing ddr4timing;
	uint32_t agingx0;
};

struct rk3328_sdram_channel {
	unsigned char rank;
	unsigned char col;
	/* 3:8bank, 2:4bank */
	unsigned char bk;
	/* channel buswidth, 2:32bit, 1:16bit, 0:8bit */
	unsigned char bw;
	/* die buswidth, 2:32bit, 1:16bit, 0:8bit */
	unsigned char dbw;
	unsigned char row_3_4;
	unsigned char cs0_row;
	unsigned char cs1_row;
	unsigned int ddrconfig;
};

#endif
