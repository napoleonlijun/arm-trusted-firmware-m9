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

#include <debug.h>
#include <mmio.h>
#include <plat_private.h>
#include <arch_helpers.h>
#include "dfs.h"
#include "dram.h"
#include "dram_spec_timing.h"
#include <string.h>
#include <soc.h>
#include <pmu.h>
#include <rockchip_sip_svc.h>
#include <delay_timer.h>
#include <fiq_dfs.h>
#include <sram.h>
#include <platform.h>

#define DTS_PAR_OFFSET	(4096)
#define vop_read32(offset)	mmio_read_32(VOP_BASE + (offset))
#define vop_write32(offset, v)	mmio_write_32(VOP_BASE + (offset), v)

struct set_rate_rel_timing {
	struct rk3328_msch_timings noc_timings;
	uint32_t rfshtmg;
	uint32_t init3;
	uint32_t init4;
	uint32_t init6;
	uint32_t init7;
	uint32_t dramtmg0;
	uint32_t dramtmg1;
	uint32_t dramtmg2;
	uint32_t dramtmg3;
	uint32_t dramtmg4;
	uint32_t dramtmg5;
	uint32_t dramtmg6;
	uint32_t dramtmg8;
	uint32_t dramtmg9;
	uint32_t dramtmg14;
	uint32_t zqctl0;
	uint32_t dfitmg0;
	uint32_t odtcfg;

	uint32_t phyreg0a;
	uint32_t phyreg0c;
	uint32_t phy_odt;
	uint32_t mr11;
};

struct rk3328_ddr_sram_param {
	/* need to init before scale frequency */
	struct rk3328_sdram_channel ch;
	uint32_t dramtype;
	struct drv_odt_lp_config drv_odt_lp_cfg;
	uint32_t sr_idle_en;
	uint8_t b_deskew;
	uint8_t ca_skew[15];
	uint8_t cs0_skew[44];
	uint8_t cs1_skew[44];
	/* tmp value used in scale frequency */
	uint64_t save_sp;
	uint32_t wait_flag0;

	uint64_t set_rate_start;
	uint64_t set_rate_end;
	uint64_t set_rate_delay;
	uint64_t stop_cpu_start;
	uint64_t stop_cpu_end;
	uint64_t stop_cpu_delay;

	uint32_t refdiv;
	uint32_t postdiv1;
	uint32_t postdiv2;
	uint32_t fbdiv;
	uint32_t clkgate[29];
	struct set_rate_rel_timing	timing[2];
	/* sram stack leave last, for future extend */
	uint32_t sram_sp[1024 / 4];
};

static __sramdata struct rk3328_ddr_sram_param sram_param;

struct rk3328_ddr_param {
	uint32_t current_index;
	uint32_t index_freq[2];
	uint32_t low_power_stat;
	/*
	 * timing_config and dram_timing
	 * place here only for debug and avoid stack overflow.
	 * so, only use timing_config and dram_timing as local parameter
	 * before dram_get_parameter(), don't use as global parameter.
	 */
	struct timing_related_config timing_config;
	struct dram_timing_t dram_timing;
};

static struct rk3328_ddr_param dram_param;

struct share_params {
	/* these parameters, not use in RK322xh */
	uint32_t hz;
	uint32_t lcdc_type;
	uint32_t vop;
	uint32_t vop_dclk_mode;
	uint32_t sr_idle_en;
	uint32_t addr_mcu_el3;
	/*
	 * 1: need to wait flag1
	 * 0: never wait flag1
	 */
	uint32_t wait_flag1;
	/*
	 * 1: need to wait flag1
	 * 0: never wait flag1
	 */
	uint32_t wait_flag0;
	/* if need, add parameter after */
};

static struct ddr_dts_config_timing dts_parameter = {
	.available = 0
};

static struct sdram_default_config ddr3_default_config = {
	.bl = 8,
	.ap = 0,
	.burst_ref_cnt = 1,
	.zqcsi = 0
};

static struct drv_odt_lp_config ddr3_drv_odt_default_config = {
	.ddr_speed_bin = DDR3_DEFAULT,
	.pd_idle = 0,
	.sr_idle = 0,
	.sr_mc_gate_idle = 0,
	.srpd_lite_idle = 0,
	.standby_idle = 0,

	.auto_pd_dis_freq = 1066,
	.auto_sr_dis_freq = 800,
	.dram_dll_dis_freq = 300,
	.phy_dll_dis_freq = 400,
	.dram_odt_dis_freq = 666,

	.dram_side_drv = 40,
	.dram_side_dq_odt = 120,
	.dram_side_ca_odt = 120,

	.phy_side_ca_drv = 34,
	.phy_side_ck_cs_drv = 45,
	.phy_side_dq_drv = 34,
	.phy_side_odt = 225,
	.phy_side_odt_dis_freq = 666,
};

static struct sdram_default_config lpddr3_default_config = {
	.bl = 8,
	.ap = 0,
	.burst_ref_cnt = 1,
	.zqcsi = 0
};

static struct drv_odt_lp_config lpddr3_drv_odt_default_config = {
	.ddr_speed_bin = DDR3_DEFAULT,
	.pd_idle = 0,
	.sr_idle = 0,
	.sr_mc_gate_idle = 0,
	.srpd_lite_idle = 0,
	.standby_idle = 0,

	.auto_pd_dis_freq = 1066,
	.auto_sr_dis_freq = 800,
	.dram_dll_dis_freq = 300,
	.phy_dll_dis_freq = 400,
	.dram_odt_dis_freq = 666,

	.dram_side_drv = 40,
	.dram_side_dq_odt = 240,
	.dram_side_ca_odt = 240,

	.phy_side_ca_drv = 34,
	.phy_side_ck_cs_drv = 43,
	.phy_side_dq_drv = 34,
	.phy_side_odt = 240,
	.phy_side_odt_dis_freq = 666,
};

static struct sdram_default_config ddr4_default_config = {
	.bl = 8,
	.ap = 0,
	.burst_ref_cnt = 1,
	.zqcsi = 0,
	.rd_dbi = 0
};

static struct drv_odt_lp_config ddr4_drv_odt_default_config = {
	.ddr_speed_bin = DDR4_DEFAULT,
	.pd_idle = 0,
	.sr_idle = 0,
	.sr_mc_gate_idle = 0,
	.srpd_lite_idle = 0,
	.standby_idle = 0,

	.auto_pd_dis_freq = 1066,
	.auto_sr_dis_freq = 800,
	.dram_dll_dis_freq = 625,
	.phy_dll_dis_freq = 400,
	.dram_odt_dis_freq = 666,

	.dram_side_drv = 34,
	.dram_side_dq_odt = 240,
	.dram_side_ca_odt = 240,

	.phy_side_ca_drv = 34,
	.phy_side_ck_cs_drv = 43,
	.phy_side_dq_drv = 34,
	.phy_side_odt = 240,
	.phy_side_odt_dis_freq = 666,
};

static inline void rockchip_en_el3_abort(void)
{
	uint32_t scr_el3;

	scr_el3 = read_scr();
	scr_el3 |= SCR_EA_BIT;
	write_scr(scr_el3);
	isb();
}

static unsigned int get_cs_die_capability(struct rk3328_sdram_channel *ch,
					  unsigned int cs,
					  unsigned int dramtype)
{
	unsigned int die;
	unsigned long long cs_cap;
	unsigned int row[2];
	unsigned int bg;

	row[0] = ch->cs0_row;
	row[1] = ch->cs1_row;
	die = 1 << (ch->bw - ch->dbw);
	if (dramtype == DDR4)
		bg = (ch->dbw == 1) ? 1 : 2;
	else
		bg = 0;
	cs_cap = (1llu << (row[cs] +
			ch->bk +
			ch->col +
			bg +
			ch->bw));
	if (ch->row_3_4)
		cs_cap = cs_cap * 3 / 4;
	return (cs_cap / die);
}

static uint32_t get_phy_drv_odt_val(uint32_t dram_type, uint32_t drv_odt)
{
	uint32_t ret;

	if (dram_type == DDR3) {
		switch (drv_odt) {
		case 225:
			ret = PHY_DDR3_RON_RTT_225ohm;
			break;
		case 150:
			ret = PHY_DDR3_RON_RTT_150ohm;
			break;
		case 75:
			ret = PHY_DDR3_RON_RTT_75ohm;
			break;
		case 64:
			ret = PHY_DDR3_RON_RTT_64ohm;
			break;
		case 50:
			ret = PHY_DDR3_RON_RTT_50ohm;
			break;
		case 41:
			ret = PHY_DDR3_RON_RTT_41ohm;
			break;
		default:
			ret = PHY_DDR3_RON_RTT_34ohm;
			break;
		}
	} else {
		switch (drv_odt) {
		case 240:
			ret = PHY_DDR4_LPDDR3_RON_RTT_240ohm;
			break;
		case 120:
			ret = PHY_DDR4_LPDDR3_RON_RTT_120ohm;
			break;
		case 80:
			ret = PHY_DDR4_LPDDR3_RON_RTT_80ohm;
			break;
		case 60:
			ret = PHY_DDR4_LPDDR3_RON_RTT_60ohm;
			break;
		case 48:
			ret = PHY_DDR4_LPDDR3_RON_RTT_48ohm;
			break;
		case 40:
			ret = PHY_DDR4_LPDDR3_RON_RTT_40ohm;
			break;
		default:
			ret = PHY_DDR4_LPDDR3_RON_RTT_34ohm;
			break;
		}
	}
	return ret;
}

/*
 * input : dram_type, dts_timing
 * output : drv_config
 */
static void drv_odt_lp_cfg_init(uint32_t dram_type,
				struct ddr_dts_config_timing *dts_timing,
				struct drv_odt_lp_config *drv_config)
{
	if ((dts_timing) && (dts_timing->available)) {
		if (dram_type == DDR3)
			drv_config->ddr_speed_bin = dts_timing->ddr3_speed_bin;
		else if (dram_type == DDR4)
			drv_config->ddr_speed_bin = dts_timing->ddr4_speed_bin;
		drv_config->pd_idle = dts_timing->pd_idle;
		drv_config->sr_idle = dts_timing->sr_idle;
		drv_config->sr_mc_gate_idle = dts_timing->sr_mc_gate_idle;
		drv_config->srpd_lite_idle = dts_timing->srpd_lite_idle;
		drv_config->standby_idle = dts_timing->standby_idle;
		drv_config->auto_pd_dis_freq = dts_timing->auto_pd_dis_freq;
		drv_config->auto_sr_dis_freq = dts_timing->auto_sr_dis_freq;
		if (dram_type == DDR3)
			drv_config->dram_dll_dis_freq =
				dts_timing->ddr3_dll_dis_freq;
		else if (dram_type == DDR4)
			drv_config->dram_dll_dis_freq =
				dts_timing->ddr4_dll_dis_freq;
		drv_config->phy_dll_dis_freq = dts_timing->phy_dll_dis_freq;
	}

	switch (dram_type) {
	case DDR3:
		if ((dts_timing) && (dts_timing->available)) {
			drv_config->dram_odt_dis_freq =
				dts_timing->ddr3_odt_dis_freq;
			drv_config->dram_side_drv = dts_timing->ddr3_drv;
			drv_config->dram_side_dq_odt = dts_timing->ddr3_odt;
			drv_config->phy_side_ca_drv =
				dts_timing->phy_ddr3_ca_drv;
			drv_config->phy_side_ck_cs_drv =
				dts_timing->phy_ddr3_ck_drv;
			drv_config->phy_side_dq_drv =
				dts_timing->phy_ddr3_dq_drv;
			drv_config->phy_side_odt = dts_timing->phy_ddr3_odt;
			drv_config->phy_side_odt_dis_freq =
				dts_timing->phy_ddr3_odt_dis_freq;
		} else {
			memcpy(drv_config, &ddr3_drv_odt_default_config,
			       sizeof(struct drv_odt_lp_config));
		}
		break;
	case LPDDR3:
		if ((dts_timing) && (dts_timing->available)) {
			drv_config->dram_odt_dis_freq =
				dts_timing->lpddr3_odt_dis_freq;
			drv_config->dram_side_drv = dts_timing->lpddr3_drv;
			drv_config->dram_side_dq_odt = dts_timing->lpddr3_odt;
			drv_config->phy_side_ca_drv =
				dts_timing->phy_lpddr3_ca_drv;
			drv_config->phy_side_ck_cs_drv =
				dts_timing->phy_lpddr3_ck_drv;
			drv_config->phy_side_dq_drv =
				dts_timing->phy_lpddr3_dq_drv;
			drv_config->phy_side_odt = dts_timing->phy_lpddr3_odt;
			drv_config->phy_side_odt_dis_freq =
				dts_timing->phy_lpddr3_odt_dis_freq;

		} else {
			memcpy(drv_config, &lpddr3_drv_odt_default_config,
			       sizeof(struct drv_odt_lp_config));
		}
		break;
	case DDR4:
	default:
		if ((dts_timing) && (dts_timing->available)) {
			drv_config->dram_odt_dis_freq =
				dts_timing->ddr4_odt_dis_freq;
			drv_config->dram_side_drv = dts_timing->ddr4_drv;
			drv_config->dram_side_dq_odt = dts_timing->ddr4_odt;
			drv_config->phy_side_ca_drv =
				dts_timing->phy_ddr4_ca_drv;
			drv_config->phy_side_ck_cs_drv =
				dts_timing->phy_ddr4_ck_drv;
			drv_config->phy_side_dq_drv =
				dts_timing->phy_ddr4_dq_drv;
			drv_config->phy_side_odt = dts_timing->phy_ddr4_odt;
			drv_config->phy_side_odt_dis_freq =
				dts_timing->phy_ddr4_odt_dis_freq;
		} else {
			memcpy(drv_config, &ddr4_drv_odt_default_config,
			       sizeof(struct drv_odt_lp_config));
		}
		break;
	}

	if (!((dts_timing) && (dts_timing->available))) {
		drv_config->phy_side_ca_drv =
			get_phy_drv_odt_val(dram_type,
					    drv_config->phy_side_ca_drv);
		drv_config->phy_side_ck_cs_drv =
			get_phy_drv_odt_val(dram_type,
					    drv_config->phy_side_ck_cs_drv);
		drv_config->phy_side_dq_drv =
			get_phy_drv_odt_val(dram_type,
					    drv_config->phy_side_dq_drv);
		drv_config->phy_side_odt =
			get_phy_drv_odt_val(dram_type,
					    drv_config->phy_side_odt);
	}
}

/*
 * input : sdram_params
 * output : ptiming_config
 */
static void dram_param_init(struct rk3328_ddr_sram_param *p_sram_param,
			    struct rk3328_ddr_param *p_dram_param)
{
	uint32_t j;
	uint32_t cur_mode;
	uint32_t dest_mode;
	struct timing_related_config *ptiming_config =
			&p_dram_param->timing_config;

	cur_mode = (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_MSTR) >>
				PCTL2_FREQUENCY_MODE_SHIFT) &
				PCTL2_FREQUENCY_MODE_MASK;
	dest_mode = (~cur_mode) & PCTL2_FREQUENCY_MODE_MASK;
	p_dram_param->current_index = cur_mode;
	/*
	 * index_freq[] use to skip pre_set_rate()
	 * do not use index_freq[cur_mode] = ddr_get_rate()
	 * here, because pre_set_rate() never called when
	 * system power up.
	 * it must be called one time, then it can skip
	 */
	p_dram_param->index_freq[cur_mode] = 0;
	p_dram_param->index_freq[dest_mode] = 0;

	ptiming_config->dram_info[0].speed_rate =
		p_sram_param->drv_odt_lp_cfg.ddr_speed_bin;
	ptiming_config->dram_info[0].cs_cnt = p_sram_param->ch.rank;
	for (j = 0; j < p_sram_param->ch.rank; j++) {
		ptiming_config->dram_info[0].per_die_capability[j] =
		    get_cs_die_capability(&p_sram_param->ch,
					  j,
					  p_sram_param->dramtype);
	}
	ptiming_config->dram_info[0].die_bw = 8 << p_sram_param->ch.dbw;

	ptiming_config->dram_type = p_sram_param->dramtype;
	ptiming_config->ch_cnt = 1;
	switch (p_sram_param->dramtype) {
	case DDR3:
		ptiming_config->bl = ddr3_default_config.bl;
		ptiming_config->ap = ddr3_default_config.ap;
		break;
	case LPDDR3:
		ptiming_config->bl = lpddr3_default_config.bl;
		ptiming_config->ap = lpddr3_default_config.ap;
		break;
	case DDR4:
		ptiming_config->bl = ddr4_default_config.bl;
		ptiming_config->ap = ddr4_default_config.ap;
		ptiming_config->rdbi = 0;
		ptiming_config->wdbi = 0;
		break;
	}
	ptiming_config->dramds =
			p_sram_param->drv_odt_lp_cfg.dram_side_drv;
	ptiming_config->dramodt =
			p_sram_param->drv_odt_lp_cfg.dram_side_dq_odt;
	ptiming_config->caodt =
			p_sram_param->drv_odt_lp_cfg.dram_side_ca_odt;
}

static uint32_t get_phypll(void)
{
	uint32_t refdiv, postdiv1, fbdiv;

	refdiv = mmio_read_32(PHY_REG(0xee)) & PHY_PLL_PRE_DIV_MASK;
	postdiv1 = mmio_read_32(PHY_REG(0xef)) & PHY_PLL_POST_DIV_MASK;
	fbdiv = ((mmio_read_32(PHY_REG(0x106)) &
			PHY_PRE_PLL_FB_DIV_11_8_MASK) <<
			PHY_PRE_PLL_FB_DIV_11_8_SHIFT) |
		(mmio_read_32(PHY_REG(0x107)) & PHY_PRE_PLL_FB_DIV_7_0_MASK);
	return ((24 * fbdiv) / (refdiv * (1 << postdiv1)));
}

static uint32_t get_dpll(void)
{
	uint32_t refdiv, postdiv1, postdiv2, fbdiv;

	postdiv1 = (mmio_read_32(CRU_BASE + PLL_CONS(DPLL_ID, 0)) >>
			POSTDIV1_SHIFT) & POSTDIV1_MASK;
	fbdiv = mmio_read_32(CRU_BASE + PLL_CONS(DPLL_ID, 0)) & FBDIV_MASK;
	postdiv2 = (mmio_read_32(CRU_BASE + PLL_CONS(DPLL_ID, 1)) >>
			POSTDIV2_SHIFT) & POSTDIV2_MASK;
	refdiv = mmio_read_32(CRU_BASE + PLL_CONS(DPLL_ID, 1)) & REFDIV_MASK;
	return ((24 * fbdiv) / (refdiv * postdiv1 * postdiv2));
}

static __sramfunc uint32_t set_phypll(unsigned int mhz, uint32_t set)
{
	int delay = 1000;

	if (set == 0) {
		sram_param.refdiv = 1;
		if (mhz <= 300)
			sram_param.postdiv1 = 3; /* 8: 1 << 3*/
		else if (mhz <= 600)
			sram_param.postdiv1 = 2; /* 4: 1 << 2 */
		else if (mhz <= 1200)
			sram_param.postdiv1 = 1; /* 2: 1 << 1 */
		else
			sram_param.postdiv1 = 0; /* 1: 1 << 0 */

		sram_param.fbdiv = (mhz * sram_param.refdiv *
			(1 << sram_param.postdiv1)) / 24;
		return ((24 * sram_param.fbdiv) /
			(sram_param.refdiv * (1 << sram_param.postdiv1)));
	} else if (set == 1) {
		/* select DDR PHY internal PLL */
		mmio_setbits_32(PHY_REG(0xef), PHY_INTER_PLL);
		/* power down pll */
		mmio_setbits_32(PHY_REG(0xed), PHY_PLL_PD);
		mmio_write_32(PHY_REG(0xee), sram_param.refdiv);
		mmio_clrsetbits_32(PHY_REG(0xef), PHY_PLL_POST_DIV_MASK,
				   sram_param.postdiv1);
		mmio_clrsetbits_32(PHY_REG(0x106),
				   PHY_PRE_PLL_FB_DIV_11_8_MASK,
				   (sram_param.fbdiv >>
				   PHY_PRE_PLL_FB_DIV_11_8_SHIFT) &
				   PHY_PRE_PLL_FB_DIV_11_8_MASK);
		mmio_write_32(PHY_REG(0x107), sram_param.fbdiv &
			PHY_PRE_PLL_FB_DIV_7_0_MASK);
		/* power up pll */
		mmio_clrbits_32(PHY_REG(0xed), PHY_PLL_PD);
	} else {
		while (delay > 0) {
			sram_udelay(1);
			if ((mmio_read_32(GRF_BASE + GRF_SOC_STATUS(0)) >>
				GRF_DDR_PLLLOCK_SHIFT) &
				GRF_DDR_PLLLOCK_MASK)
				break;
			delay--;
		}
	}
	return 0;
}

static __sramfunc uint32_t set_dpll(unsigned int mhz, uint32_t set)
{
	int delay = 1000;

	if (set == 0) {
		sram_param.refdiv = 1;
		if (mhz <= 300) {
			sram_param.postdiv1 = 4;
			sram_param.postdiv2 = 2;
		} else if (mhz <= 400) {
			sram_param.postdiv1 = 6;
			sram_param.postdiv2 = 1;
		} else if (mhz <= 600) {
			sram_param.postdiv1 = 4;
			sram_param.postdiv2 = 1;
		} else if (mhz <= 800) {
			sram_param.postdiv1 = 3;
			sram_param.postdiv2 = 1;
		} else if (mhz <= 1600) {
			sram_param.postdiv1 = 2;
			sram_param.postdiv2 = 1;
		} else {
			sram_param.postdiv1 = 1;
			sram_param.postdiv2 = 1;
		}
		sram_param.fbdiv = (mhz *
				sram_param.refdiv *
				sram_param.postdiv1 *
				sram_param.postdiv2) / 24;
		return ((24 * sram_param.fbdiv) /
				(sram_param.refdiv *
				sram_param.postdiv1 *
				sram_param.postdiv2));
	} else if (set == 1) {
		/* select system PLL */
		mmio_clrbits_32(PHY_REG(0xef), PHY_INTER_PLL);
		mmio_write_32(CRU_BASE + CRU_CRU_MODE,
			      DPLL_SLOW_MODE);
		mmio_write_32(CRU_BASE + PLL_CONS(DPLL_ID, 0),
			      POSTDIV1(sram_param.postdiv1) |
			      FBDIV(sram_param.fbdiv));
		mmio_write_32(CRU_BASE + PLL_CONS(DPLL_ID, 1),
			      DSMPD(1) | POSTDIV2(sram_param.postdiv2) |
			      REFDIV(sram_param.refdiv));
	} else {
		while (delay > 0) {
			sram_udelay(1);
			if (LOCK(mmio_read_32(CRU_BASE + PLL_CONS(DPLL_ID, 1))))
				break;
			delay--;
		}
		mmio_write_32(CRU_BASE + CRU_CRU_MODE,
			      DPLL_NORMAL_MODE);
	}
	return 0;
}

/*
 * set = 0 : ddr_round_rate, and PLL setting will be record in sram_param
 * set = 1,2 : pll two setting step
 */
__sramfunc uint32_t ddr_set_pll(uint32_t mhz, uint32_t set)
{
	/*
	if ((mmio_read_32(PHY_REG(0xef)) & PHY_INTER_PLL) ||
	    ((!(mmio_read_32(PHY_REG(0xef)) & PHY_INTER_PLL)) &&
	    (set == 2)))
	*/
	if (mmio_read_32(PHY_REG(0xef)) & PHY_INTER_PLL)
		return set_phypll(mhz * 2, set) / 2;
	else
		return set_dpll(mhz * 2, set) / 2;
}

__sramfunc void sw_set_req(void)
{
	/* clear sw_done=0 */
	mmio_write_32(DDRC_BASE_ADDR + DDR_PCTL2_SWCTL, PCTL2_SW_DONE_CLEAR);
}

__sramfunc void sw_set_ack(void)
{
	/* set sw_done=1 */
	mmio_write_32(DDRC_BASE_ADDR + DDR_PCTL2_SWCTL, PCTL2_SW_DONE);
	while (1) {
		/* wait programming done */
		if (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_SWSTAT) &
				PCTL2_SW_DONE_ACK)
			break;
	}
}

__sramfunc void update_refresh_reg(void)
{
	mmio_write_32(DDRC_BASE_ADDR + DDR_PCTL2_RFSHCTL3,
		      mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_RFSHCTL3) ^
		      (1 << 1));
}

static __sramfunc void phy_dll_bypass_set(struct rk3328_ddr_sram_param *p_sram_param,
					  uint32_t mhz)
{
	uint32_t tmp;

	mmio_setbits_32(PHY_REG(0x13), PHY_CMD_DLL_BYPASS_90);
	mmio_clrbits_32(PHY_REG(0x14), PHY_CK_DLL_BYPASS_90);
	mmio_setbits_32(PHY_REG(0x26), PHY_DQ_DLL_BYPASS_90);
	mmio_clrbits_32(PHY_REG(0x27), PHY_DQS_DLL_BYPASS_90);
	mmio_setbits_32(PHY_REG(0x36), PHY_DQ_DLL_BYPASS_90);
	mmio_clrbits_32(PHY_REG(0x37), PHY_DQS_DLL_BYPASS_90);
	mmio_setbits_32(PHY_REG(0x46), PHY_DQ_DLL_BYPASS_90);
	mmio_clrbits_32(PHY_REG(0x47), PHY_DQS_DLL_BYPASS_90);
	mmio_setbits_32(PHY_REG(0x56), PHY_DQ_DLL_BYPASS_90);
	mmio_clrbits_32(PHY_REG(0x57), PHY_DQS_DLL_BYPASS_90);

	if (mhz <= p_sram_param->drv_odt_lp_cfg.phy_dll_dis_freq)
		/* DLL bypass */
		mmio_setbits_32(PHY_REG(0xa4), PHY_DLL_BYPASS_MODE);
	else
		mmio_clrbits_32(PHY_REG(0xa4), PHY_DLL_BYPASS_MODE);
	if (mhz <= PHY_READ_DQS_DLL_SWITCH_FREQ)
		tmp = PHY_READ_DQS_DLL_45;
	else
		tmp = PHY_READ_DQS_DLL_22_5;
	mmio_write_32(PHY_REG(0x28), tmp);
	mmio_write_32(PHY_REG(0x38), tmp);
	mmio_write_32(PHY_REG(0x48), tmp);
	mmio_write_32(PHY_REG(0x58), tmp);
}


static void gen_ddr3_params(struct timing_related_config *timing_config,
			    struct dram_timing_t *pdram_timing,
			    uint32_t bw,
			    struct set_rate_rel_timing *timing)
{
	/* PCTL register */
	timing->rfshtmg = ((pdram_timing->trefi / (2 * 32)) << 16) |
					((pdram_timing->trfc + 1) / 2);
	timing->init3 = (pdram_timing->mr[0] << 16) |
				pdram_timing->mr[1];
	timing->init4 = (pdram_timing->mr[2] << 16) |
				pdram_timing->mr[3];
	timing->dramtmg0 = (((pdram_timing->cwl +
				(pdram_timing->bl / 2) +
				pdram_timing->twr) / 2) << 24) |
			(((pdram_timing->tfaw + 1) / 2) << 16) |
			(((pdram_timing->tras_max - 1) / 2048) << 8) |
			(pdram_timing->tras_min / 2);
	if (pdram_timing->mr[0] & (1 << 12)) {
		timing->dramtmg1 = (((pdram_timing->txp +
					1) / 2) << 16) |
			(((pdram_timing->trtp) / 2) << 8) |
			((pdram_timing->trc + 1) / 2);
	} else {
		timing->dramtmg1 = (((pdram_timing->txpdll +
					1) / 2) << 16) |
			(((pdram_timing->trtp) / 2) << 8) |
			((pdram_timing->trc + 1) / 2);
	}
	timing->dramtmg2 = (((pdram_timing->cwl + 1) / 2) << 24) |
			(((pdram_timing->cl + 1) / 2) << 16) |
			(((pdram_timing->cl +
				(pdram_timing->bl /  2) +
				2 - pdram_timing->cwl + 1) / 2) << 8) |
			((pdram_timing->cwl +
				(pdram_timing->bl /  2) +
				pdram_timing->twtr + 1) / 2);
	timing->dramtmg4 = (((pdram_timing->trcd - pdram_timing->al +
				1) / 2) << 24) |
			(((pdram_timing->tccd + 1) / 2) << 16) |
			(((pdram_timing->trrd + 1) / 2) << 8) |
			((pdram_timing->trp / 2) + 1);
	timing->dramtmg5 = (((pdram_timing->tcksrx + 1) / 2) << 24) |
			(((pdram_timing->tcksre + 1) / 2) << 16) |
			(((pdram_timing->tcke + 1 + 1) / 2) << 8) |
			((pdram_timing->tcke + 1) / 2);
	timing->dramtmg8 = (((pdram_timing->txsr + 31) / 32 +
				1) << 24) |
			(3 << 16) |
			(((pdram_timing->txsr + 31) / 32 + 1) << 8) |
				((pdram_timing->txsnr + 63) / 64 + 1);
	timing->odtcfg = (6 << 24) |
			(6 << 8) |
			((pdram_timing->cl - pdram_timing->cwl) << 2);
	timing->dramtmg3 = (((pdram_timing->tmrd + 1) / 2) << 20) |
				(((pdram_timing->tmrd + 1) / 2) << 12) |
				((pdram_timing->tmod + 1) / 2);
	timing->zqctl0 = (((pdram_timing->tzqoper + 1) / 2) << 16) |
				((pdram_timing->tzqcs + 1) / 2);
	timing->dfitmg0 = (7 << 24) |
				(((pdram_timing->cl - 3) / 2) << 16) |
				((pdram_timing->cwl - 3) / 2);
}

static void gen_ddr4_params(struct timing_related_config *timing_config,
			    struct dram_timing_t *pdram_timing,
			    uint32_t bw,
			    struct set_rate_rel_timing *timing)
{
	uint32_t tmp, tmp2;

	/* PCTL register */
	timing->rfshtmg = ((pdram_timing->trefi / (2 * 32)) << 16) |
					((pdram_timing->trfc + 1) / 2);
	timing->init3 = (pdram_timing->mr[0] << 16) |
				pdram_timing->mr[1];
	timing->init4 = (pdram_timing->mr[2] << 16) |
				pdram_timing->mr[3];
	timing->init6 = (pdram_timing->mr[4] << 16) |
				pdram_timing->mr[5];
	timing->init7 = pdram_timing->mr[6];
	timing->dramtmg0 = (((pdram_timing->cwl +
			(pdram_timing->bl / 2) +
			pdram_timing->twr) / 2) << 24) |
			(((pdram_timing->tfaw + 1) / 2) << 16) |
			(((pdram_timing->tras_max - 1) / 2048) << 8) |
			(pdram_timing->tras_min / 2);
	tmp = (pdram_timing->al + pdram_timing->trtp) / 2;
	tmp2 = (pdram_timing->al + pdram_timing->cl +
			pdram_timing->bl / 2 -
			pdram_timing->trp) / 2;
	tmp = (tmp > tmp2) ? tmp : tmp2;
	timing->dramtmg1 = (((pdram_timing->txp + 1) / 2) << 16) |
				(tmp << 8) |
				((pdram_timing->trc + 1) / 2);

	/* write preample = 1 */
	timing->dramtmg2 = (((pdram_timing->al + pdram_timing->cwl +
				1) / 2) << 24) |
			(((pdram_timing->al + pdram_timing->cl +
				1) / 2) << 16) |
			(((pdram_timing->cl +
				(pdram_timing->bl / 2) +
				2 - pdram_timing->cwl + 1) / 2) << 8) |
			((pdram_timing->cwl +
				(pdram_timing->bl /  2) +
				pdram_timing->twtr_l + 1) / 2 + 1);
	timing->dramtmg4 = (((pdram_timing->trcd - pdram_timing->al +
				1) / 2) << 24) |
			(((pdram_timing->tccd_l + 1) / 2) << 16) |
			(((pdram_timing->trrd_l + 1) / 2) << 8) |
			((pdram_timing->trp / 2) + 1);
	timing->dramtmg5 = (((pdram_timing->tcksrx + 1) / 2) << 24) |
			(((pdram_timing->tcksre + 1) / 2) << 16) |
			(((pdram_timing->tckesr + 1) / 2) << 8) |
			((pdram_timing->tcke + 1) / 2);
	timing->dramtmg8 = (((pdram_timing->txs_fast + 63) / 64 +
				1) << 24) |
			(((pdram_timing->txs_abort + 63) / 64 +
				1) << 16) |
			(((pdram_timing->txsr + 63) / 64 + 1) << 8) |
			((pdram_timing->txsnr + 63) / 64 + 1);
	timing->dramtmg9 = (((pdram_timing->tccd + 1) / 2) << 16) |
			(((pdram_timing->trrd + 1) / 2) << 8) |
			((pdram_timing->cwl +
				pdram_timing->bl / 2 +
				pdram_timing->twtr + 1) / 2);
	timing->odtcfg = (6 << 24) |
			(6 << 8) |
			((pdram_timing->cl - pdram_timing->cwl) << 2);
	timing->dramtmg3 = (((pdram_timing->tmrd + 1) / 2) << 20) |
				(((pdram_timing->tmrd + 1) / 2) << 12) |
				((pdram_timing->tmod + 1) / 2);
	timing->zqctl0 = (((pdram_timing->tzqoper + 1) / 2) << 16) |
				((pdram_timing->tzqcs + 1) / 2);
	timing->dfitmg0 = (7 << 24) |
				(((pdram_timing->cl - 3) / 2) << 16) |
				((pdram_timing->cwl - 3) / 2);
}

static void gen_lpddr3_params(struct timing_related_config *timing_config,
			      struct dram_timing_t *pdram_timing,
			      uint32_t bw,
			      struct set_rate_rel_timing *timing)
{
	/* PCTL register */
	timing->rfshtmg = ((pdram_timing->trefi / (2 * 32)) << 16) |
					((pdram_timing->trfc + 1) / 2);
	timing->init3 = (pdram_timing->mr[1] << 16) |
				pdram_timing->mr[2];
	timing->init4 = (pdram_timing->mr[3] << 16);
	timing->dramtmg0 = (((pdram_timing->cwl +
				(pdram_timing->bl / 2) +
			pdram_timing->twr + 1) / 2) << 24) |
			(((pdram_timing->tfaw + 1) / 2) << 16) |
			(((pdram_timing->tras_max - 1) / 2048) << 8) |
			(pdram_timing->tras_min / 2);
	timing->dramtmg1 = (((pdram_timing->txp + 1) / 2) << 16) |
			(((pdram_timing->trtp) / 2) << 8) |
			((pdram_timing->trc + 1) / 2);
	timing->dramtmg2 = (((pdram_timing->cwl + 1) / 2) << 24) |
			(((pdram_timing->cl + 1) / 2) << 16) |
			(((pdram_timing->cl +
				(pdram_timing->bl /  2) +
				pdram_timing->tdqsck_max +
				1 - pdram_timing->cwl + 1) / 2) << 8) |
			((pdram_timing->cwl +
				(pdram_timing->bl /  2) +
				pdram_timing->twtr + 1 + 1) / 2);
	timing->dramtmg4 = (((pdram_timing->trcd -
				pdram_timing->al + 1) / 2) << 24) |
			(((pdram_timing->tccd + 1) / 2) << 16) |
			(((pdram_timing->trrd + 1) / 2) << 8) |
			((pdram_timing->trp / 2) + 1);
	timing->dramtmg5 = (((pdram_timing->tcksrx + 1) / 2) << 24) |
			(((pdram_timing->tcksre + 1) / 2) << 16) |
			(((pdram_timing->tckesr + 1) / 2) << 8) |
			((pdram_timing->tckesr + 1) / 2);
	timing->dramtmg6 = (2 << 24) |
			(2 << 16) |
			((pdram_timing->txp + 2 + 1) / 2);
	timing->dramtmg8 = (((pdram_timing->txsnr + 63) /
				64 + 1) << 24) |
			(3 << 16) |
			(((pdram_timing->txsr + 63) / 64 + 1) << 8) |
				((pdram_timing->txsnr + 63) / 64 + 1);
	timing->dramtmg14 = ((pdram_timing->txsr + 1) / 2);
	timing->odtcfg = ((7 + pdram_timing->todton) << 24) |
			(((pdram_timing->cwl - 1 -
				pdram_timing->todton) & 0x1f) << 16) |
			((5 + pdram_timing->tdqsck_max -
				pdram_timing->tdqsck +
				pdram_timing->todton) << 8) |
			(((pdram_timing->cl + pdram_timing->tdqsck -
				1 - pdram_timing->todton) &
				0x1f) << 2);
	timing->dramtmg3 = (((pdram_timing->tmrd + 1) / 2) << 20) |
				(((pdram_timing->tmrd + 1) / 2) << 12) |
				((pdram_timing->tmod + 1) / 2);
	timing->zqctl0 = (((pdram_timing->tzqoper + 1) / 2) << 16) |
				((pdram_timing->tzqcs + 1) / 2);
	timing->dfitmg0 = (7 << 24) |
				(((pdram_timing->cl - 2) / 2) << 16) |
				((pdram_timing->cwl - 2) / 2);
	timing->mr11 = pdram_timing->mr11;
}

static void gen_noc_params(struct timing_related_config *timing_config,
			   struct dram_timing_t *pdram_timing,
			   uint32_t bw,
			   struct set_rate_rel_timing *timing)
{
	if (bw == 2)
		timing->noc_timings.ddrtiming.b.bwratio = 0;
	else
		timing->noc_timings.ddrtiming.b.bwratio = 1;
	timing->noc_timings.ddrtiming.b.wrtord = (pdram_timing->cwl +
				pdram_timing->twtr) / 2;
	timing->noc_timings.ddrtiming.b.rdtowr = (pdram_timing->cl -
				pdram_timing->cwl + 2) / 2;
	timing->noc_timings.ddrtiming.b.burstlen = (pdram_timing->bl / 2) / 2;
	timing->noc_timings.ddrtiming.b.wrtomiss = (pdram_timing->cwl +
				pdram_timing->twr +
				pdram_timing->trp +
				pdram_timing->trcd) / 2;
	timing->noc_timings.ddrtiming.b.rdtomiss = (pdram_timing->trtp +
				pdram_timing->trp +
				pdram_timing->trcd -
				(pdram_timing->bl / 2)) / 2;
	timing->noc_timings.ddrtiming.b.acttoact = (pdram_timing->trc / 2);
	timing->noc_timings.ddrmode.d32 = 0;
	timing->noc_timings.readlatency = (pdram_timing->trp +
					pdram_timing->trcd +
					pdram_timing->cl +
					pdram_timing->bl / 2)
					/ 2 + 22;
	timing->noc_timings.agingx0 = 0xff;
	timing->noc_timings.activate.b.fawbank = 1;
	timing->noc_timings.activate.b.faw = pdram_timing->tfaw / 2;
	timing->noc_timings.activate.b.rrd = pdram_timing->trrd / 2;
	timing->noc_timings.devtodev.d32 = (1 << 4) | (1 << 2) | 1;
	if (timing_config->dram_type == DDR4)
		timing->noc_timings.ddr4timing.d32 =
				((pdram_timing->trrd_l / 2) << 8) |
				(((pdram_timing->cwl +
					pdram_timing->bl / 2 +
					pdram_timing->twtr_l) / 2) << 3) |
				(pdram_timing->tccd_l / 2);
	else
		timing->noc_timings.ddr4timing.d32 =
				((pdram_timing->trrd / 2) << 8) |
				(((pdram_timing->cwl +
					pdram_timing->twtr) / 2) << 3) |
				(pdram_timing->tccd / 2);
}

/*
 * input : timing_config, pdram_timing, bw
 * output : timing
 */
static void gen_rk3328_ddr_params(struct timing_related_config *timing_config,
				  struct dram_timing_t *pdram_timing,
				  uint32_t bw,
				  struct set_rate_rel_timing *timing)
{
	/* PHY register */
	timing->phyreg0a = pdram_timing->cl;
	timing->phyreg0c = pdram_timing->cwl;

	if (timing_config->dram_type == DDR3)
		gen_ddr3_params(timing_config, pdram_timing, bw, timing);
	else if (timing_config->dram_type == DDR4)
		gen_ddr4_params(timing_config, pdram_timing, bw, timing);
	else
		gen_lpddr3_params(timing_config, pdram_timing, bw, timing);
	gen_noc_params(timing_config, pdram_timing, bw, timing);
}

static __sramfunc void ddr_update_odt(struct rk3328_ddr_sram_param *p_sram_param,
				      uint32_t dest_mode)
{
	struct drv_odt_lp_config *drv_odt_lp_cfg;

	drv_odt_lp_cfg = &p_sram_param->drv_odt_lp_cfg;
	/* DS */
	mmio_write_32(PHY_REG(0x11), drv_odt_lp_cfg->phy_side_ca_drv);
	mmio_clrsetbits_32(PHY_REG(0x12),
			   PHY_CMD_PRCOMP_MASK << PHY_CMD_PRCOMP_SHIFT,
			   drv_odt_lp_cfg->phy_side_ca_drv <<
			   PHY_CMD_PRCOMP_SHIFT);
	mmio_write_32(PHY_REG(0x16), drv_odt_lp_cfg->phy_side_ck_cs_drv);
	mmio_write_32(PHY_REG(0x18), drv_odt_lp_cfg->phy_side_ck_cs_drv);
	mmio_write_32(PHY_REG(0x20), drv_odt_lp_cfg->phy_side_dq_drv);
	mmio_write_32(PHY_REG(0x2f), drv_odt_lp_cfg->phy_side_dq_drv);
	mmio_write_32(PHY_REG(0x30), drv_odt_lp_cfg->phy_side_dq_drv);
	mmio_write_32(PHY_REG(0x3f), drv_odt_lp_cfg->phy_side_dq_drv);
	mmio_write_32(PHY_REG(0x40), drv_odt_lp_cfg->phy_side_dq_drv);
	mmio_write_32(PHY_REG(0x4f), drv_odt_lp_cfg->phy_side_dq_drv);
	mmio_write_32(PHY_REG(0x50), drv_odt_lp_cfg->phy_side_dq_drv);
	mmio_write_32(PHY_REG(0x5f), drv_odt_lp_cfg->phy_side_dq_drv);
	/* ODT */
	if (p_sram_param->timing[dest_mode].phy_odt) {
		mmio_write_32(PHY_REG(0x21), drv_odt_lp_cfg->phy_side_odt);
		mmio_write_32(PHY_REG(0x2e), drv_odt_lp_cfg->phy_side_odt);
		mmio_write_32(PHY_REG(0x31), drv_odt_lp_cfg->phy_side_odt);
		mmio_write_32(PHY_REG(0x3e), drv_odt_lp_cfg->phy_side_odt);
		mmio_write_32(PHY_REG(0x41), drv_odt_lp_cfg->phy_side_odt);
		mmio_write_32(PHY_REG(0x4e), drv_odt_lp_cfg->phy_side_odt);
		mmio_write_32(PHY_REG(0x51), drv_odt_lp_cfg->phy_side_odt);
		mmio_write_32(PHY_REG(0x5e), drv_odt_lp_cfg->phy_side_odt);
	} else {
		mmio_write_32(PHY_REG(0x21), 0);
		mmio_write_32(PHY_REG(0x2e), 0);
		mmio_write_32(PHY_REG(0x31), 0);
		mmio_write_32(PHY_REG(0x3e), 0);
		mmio_write_32(PHY_REG(0x41), 0);
		mmio_write_32(PHY_REG(0x4e), 0);
		mmio_write_32(PHY_REG(0x51), 0);
		mmio_write_32(PHY_REG(0x5e), 0);
	}
}

static __sramfunc void ddr_update_skew(struct rk3328_ddr_sram_param *p_sram_param)
{
	uint32_t n;

	if (p_sram_param->b_deskew)
		return;

	p_sram_param->b_deskew = 1;
	for (n = 0; n < ARRAY_SIZE(p_sram_param->ca_skew); n++)
		mmio_write_32(PHY_REG(0xb0 + n), p_sram_param->ca_skew[n]);

	for (n = 0; n < ARRAY_SIZE(p_sram_param->cs0_skew); n++)
		mmio_write_32(PHY_REG(0x70 + n), p_sram_param->cs0_skew[n]);

	for (n = 0; n < ARRAY_SIZE(p_sram_param->cs1_skew); n++)
		mmio_write_32(PHY_REG(0xc0 + n), p_sram_param->cs1_skew[n]);
}

/*
 * input : dts_timing
 * output : p_sram_param
 */
static void ddr_sram_param_init(struct ddr_dts_config_timing *dts_timing,
				struct rk3328_ddr_sram_param *p_sram_param)
{
	uint32_t os_reg2_val;
	struct rk3328_sdram_channel *ch = &p_sram_param->ch;
	uint32_t n;

	/* init sram_param */
	os_reg2_val = mmio_read_32(GRF_BASE + GRF_OS_REG(2));
	VERBOSE("os_reg2_val = 0x%x\n", os_reg2_val);
	p_sram_param->dramtype = SYS_REG_DEC_DDRTYPE(os_reg2_val);
	ch->rank = SYS_REG_DEC_RANK(os_reg2_val);
	ch->col = SYS_REG_DEC_COL(os_reg2_val);
	ch->bk = SYS_REG_DEC_BK(os_reg2_val);
	ch->bw = SYS_REG_DEC_BW(os_reg2_val);
	ch->dbw = SYS_REG_DEC_DBW(os_reg2_val);
	ch->row_3_4 = SYS_REG_DEC_ROW_3_4(os_reg2_val);
	ch->cs0_row = SYS_REG_DEC_CS0_ROW(os_reg2_val);
	ch->cs1_row = SYS_REG_DEC_CS1_ROW(os_reg2_val);

	drv_odt_lp_cfg_init(p_sram_param->dramtype, dts_timing,
			    &p_sram_param->drv_odt_lp_cfg);

	memset((void *)p_sram_param->sram_sp, 0x5a,
	       sizeof(p_sram_param->sram_sp));
	p_sram_param->set_rate_delay = 0;
	p_sram_param->stop_cpu_delay = 0;
	p_sram_param->sr_idle_en = 0;
	p_sram_param->b_deskew = 0;

	if (!dts_timing)
		goto no_dts;

	if (!(dts_timing->available))
		goto no_dts;

	for (n = 0; n < ARRAY_SIZE(dts_timing->ca_skew); n++)
		p_sram_param->ca_skew[n] = (uint8_t)dts_timing->ca_skew[n];

	for (n = 0; n < ARRAY_SIZE(dts_timing->cs0_skew); n++)
		p_sram_param->cs0_skew[n] = (uint8_t)dts_timing->cs0_skew[n];

	for (n = 0; n < ARRAY_SIZE(dts_timing->cs1_skew); n++)
		p_sram_param->cs1_skew[n] = (uint8_t)dts_timing->cs1_skew[n];
	return;
no_dts:
	for (n = 0; n < ARRAY_SIZE(dts_timing->ca_skew); n++)
		p_sram_param->ca_skew[n] = 0x77;

	for (n = 0; n < ARRAY_SIZE(dts_timing->cs0_skew); n++) {
		if ((n % 11) == 10)
			p_sram_param->cs0_skew[n] = 0x7;
		else
			p_sram_param->cs0_skew[n] = 0x77;
	}

	for (n = 0; n < ARRAY_SIZE(dts_timing->cs1_skew); n++) {
		if ((n % 11) == 10)
			p_sram_param->cs1_skew[n] = 0x7;
		else
			p_sram_param->cs1_skew[n] = 0x77;
	}
}

/* input : dts_timing, p_sram_param */
static void ddr_related_init(struct ddr_dts_config_timing *dts_timing,
			     struct rk3328_ddr_sram_param *p_sram_param)
{
	/* init other */
	mmio_write_32(GRF_BASE + GRF_SOC_CON(5), OTHER_MASTER_STALL_RESPONSE);
	mmio_write_32(GRF_BASE + GRF_SOC_CON(6), 0xffffffff &
				(~(MSCH_SRV_FW_FWR | MSCH_FWR_LINK |
				CORE_FWR_BUS_LINK | CORE_REQ_LINK)));
	/* need to update drv,odt,low power mode here */
	/*
	ddr_update_odt();
	*/
	/*
	 * pd_idle, sr_idle never update here
	 * because these setting is static, in other words
	 * upctl2 not support modify these setting after start
	 */
}

__sramfunc void update_pctl_timing(uint32_t dest_mode,
				   struct set_rate_rel_timing *timing,
				   uint32_t dramtype)
{
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_RFSHTMG,
		      timing->rfshtmg);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_INIT3,
		      timing->init3);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_INIT4,
		      timing->init4);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DRAMTMG0,
		      timing->dramtmg0);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DRAMTMG1,
		      timing->dramtmg1);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DRAMTMG2,
		      timing->dramtmg2);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DRAMTMG3,
		      timing->dramtmg3);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DRAMTMG4,
		      timing->dramtmg4);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DRAMTMG5,
		      timing->dramtmg5);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DRAMTMG8,
		      timing->dramtmg8);
	mmio_clrsetbits_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_ZQCTL0,
			   0x7ffffff,
			   timing->zqctl0);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DFITMG0,
		      timing->dfitmg0);
	mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_ODTCFG,
		      timing->odtcfg);
	if (dramtype == DDR4) {
		mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_INIT6,
			      timing->init6);
		mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_INIT7,
			      timing->init7);
		mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DRAMTMG9,
			      timing->dramtmg9);
	} else if (dramtype == LPDDR3) {
		mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DRAMTMG6,
			      timing->dramtmg6);
		mmio_write_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_DRAMTMG14,
			      timing->dramtmg14);
	}
}

static void pre_set_rate(uint32_t mhz, uint32_t dest_mode,
			 struct rk3328_ddr_sram_param *p_sram_param)
{
	struct set_rate_rel_timing *timing = &(p_sram_param->timing[dest_mode]);

	dram_param.timing_config.freq = mhz;
	if (mhz <= p_sram_param->drv_odt_lp_cfg.dram_dll_dis_freq)
		dram_param.timing_config.dllbp = 1;
	else
		dram_param.timing_config.dllbp = 0;
	if (mhz <= p_sram_param->drv_odt_lp_cfg.dram_odt_dis_freq) {
		dram_param.timing_config.odt = 0;
		dram_param.timing_config.dramodt = 0;
	} else {
		dram_param.timing_config.odt = 1;
		dram_param.timing_config.dramodt =
			p_sram_param->drv_odt_lp_cfg.dram_side_dq_odt;
	}
	if (mhz <= p_sram_param->drv_odt_lp_cfg.phy_side_odt_dis_freq)
		timing->phy_odt = 0;
	else
		timing->phy_odt = 1;

	dram_get_parameter(&dram_param.timing_config, &dram_param.dram_timing);
	gen_rk3328_ddr_params(&dram_param.timing_config,
			      &dram_param.dram_timing,
			      p_sram_param->ch.bw,
			      timing);

	dram_param.index_freq[dest_mode] = mhz;
}

static void post_set_rate(uint32_t mhz, uint32_t dest_mode,
			  struct rk3328_ddr_sram_param *p_sram_param)
{
	struct set_rate_rel_timing *timing;

	timing = &(p_sram_param->timing[dest_mode]);
	/* noc timing here to reduce port disable time */
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + DDRTIMING,
		      timing->noc_timings.ddrtiming.d32);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + DDRMODE,
		      timing->noc_timings.ddrmode.d32);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + READLATENCY,
		      timing->noc_timings.readlatency);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + ACTIVATE,
		      timing->noc_timings.activate.d32);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + DEVTODEV,
		      timing->noc_timings.devtodev.d32);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + DDR4TIMING,
		      timing->noc_timings.ddr4timing.d32);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + AGING0,
		      timing->noc_timings.agingx0);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + AGING1,
		      timing->noc_timings.agingx0);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + AGING2,
		      timing->noc_timings.agingx0);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + AGING3,
		      timing->noc_timings.agingx0);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + AGING4,
		      timing->noc_timings.agingx0);
	mmio_write_32(SERVER_MSCH0_BASE_ADDR + AGING5,
		      timing->noc_timings.agingx0);

	dram_param.current_index = dest_mode;

	if (mhz < p_sram_param->drv_odt_lp_cfg.auto_pd_dis_freq)
		mmio_setbits_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRCTL,
				PCTL2_POWERDOWN_EN);
	else if (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRTMG) &
				PCTL2_POWERDOWN_TO_X32_MASK)
		mmio_clrbits_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRCTL,
				PCTL2_POWERDOWN_EN);
	if (p_sram_param->sr_idle_en &&
	    (mhz < p_sram_param->drv_odt_lp_cfg.auto_sr_dis_freq))
		mmio_setbits_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRCTL,
				PCTL2_SELFREF_EN);
	else if (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRTMG) &
				(PCTL2_SELFREF_TO_X32_MASK <<
				 PCTL2_SELFREF_TO_X32_SHIFT))
		mmio_clrbits_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRCTL,
				PCTL2_SELFREF_EN);
}

/*
 * rank = 1: cs0
 * rank = 2: cs1
 * rank = 3: cs0 & cs1
 * note: be careful of keep mr original val
 */
static __sramfunc void write_mr(uint32_t rank, uint32_t mr_num,
				uint32_t arg , uint32_t dramtype)
{
	while (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_MRSTAT) &
			    PCTL2_MR_WR_BUSY)
		continue;
	if ((dramtype == DDR3) || (dramtype == DDR4)) {
		mmio_write_32(DDRC_BASE_ADDR + DDR_PCTL2_MRCTRL0,
			      (mr_num << PCTL2_MR_ADDR_SHIFT) |
			      (rank << PCTL2_MR_RANK_SHIFT) |
			      PCTL2_MR_TYPE_WR);
		mmio_write_32(DDRC_BASE_ADDR + DDR_PCTL2_MRCTRL1, arg);
	} else {
		mmio_write_32(DDRC_BASE_ADDR + DDR_PCTL2_MRCTRL0,
			      (rank << PCTL2_MR_RANK_SHIFT) |
			      PCTL2_MR_TYPE_WR);
		mmio_write_32(DDRC_BASE_ADDR + DDR_PCTL2_MRCTRL1,
			      (mr_num << PCTL2_MR_ADDRESS_SHIFT) |
			      (arg & PCTL2_MR_DATA_MASK));
	}

	mmio_setbits_32(DDRC_BASE_ADDR + DDR_PCTL2_MRCTRL0, PCTL2_MR_WR);
	while (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_MRCTRL0) & PCTL2_MR_WR)
		continue;
	while (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_MRSTAT) &
			    PCTL2_MR_WR_BUSY)
		continue;
}

static __sramfunc int data_training(uint32_t cs, uint32_t dramtype)
{
	uint32_t ret;
	uint32_t i, cnt;
	int32_t timeout = 50;
	uint32_t deskew_val;
	uint32_t gate_val = 0;
	uint32_t tmp;

	if (dramtype == DDR4) {
		mmio_clrsetbits_32(PHY_REG(0x29),
				   PHY_WEAK_PULL_UP | PHY_WEAK_PULL_DOWN,
				   0);
		mmio_clrsetbits_32(PHY_REG(0x39),
				   PHY_WEAK_PULL_UP | PHY_WEAK_PULL_DOWN,
				   0);
		mmio_clrsetbits_32(PHY_REG(0x49),
				   PHY_WEAK_PULL_UP | PHY_WEAK_PULL_DOWN,
				   0);
		mmio_clrsetbits_32(PHY_REG(0x59),
				   PHY_WEAK_PULL_UP | PHY_WEAK_PULL_DOWN,
				   0);
	}
	/* choose training cs */
	mmio_clrsetbits_32(PHY_REG(2), PHY_GATING_CALIBRATION_MASK,
			   (PHY_GATING_CALIBRATION_CS0 >> cs));
	/* enable gate training */
	mmio_clrsetbits_32(PHY_REG(2), PHY_GATING_CALIBRATION_MASK,
			   (PHY_GATING_CALIBRATION_CS0 >> cs) |
			   PHY_GATING_CALIBRATION_EN);
	while (((mmio_read_32(PHY_REG(0xff)) & 0xF) != 0xF) &&
	       (timeout > 0)) {
		sram_udelay(1);
		timeout--;
	}
	ret = mmio_read_32(PHY_REG(0xff));
	if (ret & 0x10) {
		ret = 1;
	} else {
		ret = (ret & 0xf) ^ (mmio_read_32(PHY_REG(0)) >>
					PHY_CHANNEL_SELECT_SHIFT);
		ret = (ret == 0) ? 0 : 1;
	}
	if (!ret) {
		/* disable gate training */
		mmio_clrsetbits_32(PHY_REG(2), PHY_GATING_CALIBRATION_MASK,
				   (PHY_GATING_CALIBRATION_CS0 >> cs) |
				   PHY_GATING_CALIBRATION_DIS);
		if (((mmio_read_32(PHY_REG(0)) >>
				PHY_CHANNEL_SELECT_SHIFT) &
				PHY_CHANNEL_SELECT_MASK) ==
				PHY_CHANNEL_SELECT_16)
			cnt = 2;
		else
			cnt = 4;
		for (i = 0; i < cnt; i++) {
			tmp = mmio_read_32(PHY_REG(0xfb + i));
			gate_val = (tmp > gate_val) ? tmp : gate_val;
		}
		/* fix rx deskew switching bug */
		deskew_val = (gate_val >> 3) + 1;
		deskew_val = (deskew_val > 0x1f) ? 0x1f : deskew_val;
		mmio_clrsetbits_32(PHY_REG(0x6e), 0xc, (deskew_val & 0x3) << 2);
		mmio_clrsetbits_32(PHY_REG(0x6f), 0x7 << 4,
				   (deskew_val & 0x1c) << 2);
	}

	if (dramtype == DDR4) {
		mmio_clrsetbits_32(PHY_REG(0x29),
				   PHY_WEAK_PULL_UP | PHY_WEAK_PULL_DOWN,
				   PHY_WEAK_PULL_UP);
		mmio_clrsetbits_32(PHY_REG(0x39),
				   PHY_WEAK_PULL_UP | PHY_WEAK_PULL_DOWN,
				   PHY_WEAK_PULL_UP);
		mmio_clrsetbits_32(PHY_REG(0x49),
				   PHY_WEAK_PULL_UP | PHY_WEAK_PULL_DOWN,
				   PHY_WEAK_PULL_UP);
		mmio_clrsetbits_32(PHY_REG(0x59),
				   PHY_WEAK_PULL_UP | PHY_WEAK_PULL_DOWN,
				   PHY_WEAK_PULL_UP);
	}

	/*
	 * measured total time from exit self-refresh to enable
	 * auto-refresh, use 58.7us at 200MHz
	 */
	if (dramtype == LPDDR3)
		cnt = 15;
	else
		cnt = 8;
	for (i = 0; i < cnt; i++) {
		/* compensate 4 refresh */
		mmio_setbits_32(DDRC_BASE_ADDR + DDR_PCTL2_DBGCMD,
				PCTL2_RANK1_REFRESH | PCTL2_RANK0_REFRESH);
		while (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_DBGSTAT) &
					(PCTL2_RANK1_REFRESH_BUSY |
					PCTL2_RANK0_REFRESH_BUSY))
			continue;
	}
	return ret;
}

static __sramfunc void idle_port(void)
{
	uint32_t i;

	for (i = 0; i < 29; i++)
		sram_param.clkgate[i] =
			mmio_read_32(CRU_BASE + CRU_CLKGATE_CON(i));

	for (i = 0; i < 29; i++)
		mmio_write_32(CRU_BASE + CRU_CLKGATE_CON(i),
			      0xffff0000);

	mmio_write_32(GRF_BASE + GRF_SOC_CON(5),
		      MSCH_PWR_IDLEREQ_MASK | MSCH_PWR_IDLEREQ_EN);
	dsb();

	while (!(mmio_read_32(GRF_BASE + GRF_SOC_STATUS(1)) & MSCH_PWR_IDLE))
		continue;
}

static __sramfunc void deidle_port(void)
{
	uint32_t i;

	mmio_write_32(GRF_BASE + GRF_SOC_CON(5),
		      MSCH_PWR_IDLEREQ_MASK | MSCH_PWR_IDLEREQ_DIS);
	dsb();

	while (mmio_read_32(GRF_BASE + GRF_SOC_STATUS(1)) & MSCH_PWR_IDLE)
		continue;

	for (i = 0; i < 29; i++)
		mmio_write_32(CRU_BASE + CRU_CLKGATE_CON(i),
			      0xffff0000 | sram_param.clkgate[i]);
}

static __sramfunc void register_prefetch(void)
{
	unsigned long n;

	__asm volatile ("ldr %0,[%1]\n" : "=r" (n) : "r"(CRU_BASE));
	__asm volatile ("ldr %0,[%1]\n" : "=r" (n) : "r"(GRF_BASE));
	__asm volatile ("ldr %0,[%1]\n" : "=r" (n) : "r"(UMCTL2_REGS_FREQ(0)));
	__asm volatile ("ldr %0,[%1]\n" : "=r" (n) : "r"(UMCTL2_REGS_FREQ(1) +
							DDR_PCTL2_DERATEEN));
	__asm volatile ("ldr %0,[%1]\n" : "=r" (n) : "r"(PHY_REG(0)));
	isb();
	dsb();
}

static void stop_cpus(void)
{
	int timeout = 1500;

	while ((0 != fiq_dfs_stop_cpus()) &&
	       (timeout >= 0)) {
		fiq_dfs_active_cpus();
		/* delay to let current cpu not occupy bus
		 * and let other cpus works a little while
		 */
		sram_udelay(1);
		timeout--;
	}
}

static void start_cpus(void)
{
	fiq_dfs_active_cpus();
}

#ifdef SYNC_WITH_LCDC_FRAME_INTR
/*
 * return 0: vop disabled now
 * return 1: vop enabled now
 */
static uint32_t check_vop_en(void)
{
	if (mmio_read_32(CRU_BASE + CRU_CLKGATE_CON(21)) &
			HCLK_VOP_EN)
		return 0;
	else
		return 1;
}

/*
 * return 0: some win enabled now
 * return 1: all win disable now
 */
static uint32_t check_all_win_disable(void)
{
	if ((vop_read32(VOP_WIN0_CTRL0) & WIN_EN) |
		(vop_read32(VOP_WIN1_CTRL0) & WIN_EN) |
		(vop_read32(VOP_WIN2_CTRL0) & WIN_EN))
		return 0;
	else
		return 1;
}

static void clear_vop_flag1(void)
{
	vop_write32(VOP_INTR_CLEAR0, VOP_CLEAR_FLAG1);
	sram_udelay(1);
}

static void clear_vop_flag0(void)
{
	vop_write32(VOP_INTR_CLEAR0, VOP_CLEAR_FLAG0);
	sram_udelay(1);
}

static void poll_vop_flag1(void)
{
	int timeout;

	timeout = 32000;
	while ((((vop_read32(VOP_SYS_CTRL) & VOP_STAND_BY) == 0) &&
		((vop_read32(VOP_INTR_RAW_STATUS0) & VOP_FLAG1_STATUS) == 0)) &&
		(timeout >= 0)) {
		sram_udelay(1);
		timeout--;
	}
	if (timeout <= 0)
		VERBOSE("wait flag1 timeout\n");
}

static __sramfunc void poll_vop_flag0(void)
{
	int timeout;

	timeout = 32000;
	while ((((vop_read32(VOP_SYS_CTRL) & VOP_STAND_BY) == 0) &&
		((vop_read32(VOP_INTR_RAW_STATUS0) & VOP_FLAG0_STATUS) == 0)) &&
		(timeout >= 0)) {
		sram_udelay(1);
		timeout--;
	}
	if (timeout <= 0) {
		INFO("wait flag0 timeout\n");
		/*
		 * force to scale ddr frequency
		 * need flush L1/L2, because INFO()
		 * have dirty it
		 */
		fiq_dfs_flush_l1();
		fiq_dfs_flush_l2();
	}
}

/*
 * return 0: flag0 not gone
 * return 1: flag0 have gone
 */
static uint32_t check_vop_flag0_gone(void)
{
	if (((vop_read32(VOP_SYS_CTRL) & VOP_STAND_BY) == 0) &&
	    (vop_read32(VOP_INTR_RAW_STATUS0) & VOP_FLAG0_STATUS))
		return 1;
	else
		return 0;
}

/*
 * return 0: never wait flag0, direct scale frequency
 * return 1: need to wait flag0 in scale frequency sram function
 */
static uint32_t wait_vop_vbank(struct share_params *p)
{
	uint32_t wait_flag0 = 0;

	/* wait for lcdc line flag intrrupt */
	while (1) {
		if (!check_vop_en()) {
			/* vop clk disable
			 * let other cpu enter wfe
			 * and break immediately
			 */
			sram_param.stop_cpu_start = read_cntpct_el0();
			stop_cpus();
			sram_param.stop_cpu_end = read_cntpct_el0();
			if ((sram_param.stop_cpu_end -
			    sram_param.stop_cpu_start) >
			    sram_param.stop_cpu_delay)
				sram_param.stop_cpu_delay =
					(sram_param.stop_cpu_end -
					 sram_param.stop_cpu_start);
			break;
		}
		if (check_all_win_disable()) {
			/* no win enable
			 * let other cpu enter wfe
			 * and break immediately
			 */
			sram_param.stop_cpu_start = read_cntpct_el0();
			stop_cpus();
			sram_param.stop_cpu_end = read_cntpct_el0();
			if ((sram_param.stop_cpu_end -
			    sram_param.stop_cpu_start) >
			    sram_param.stop_cpu_delay)
				sram_param.stop_cpu_delay =
					(sram_param.stop_cpu_end -
					 sram_param.stop_cpu_start);
			break;
		}

		if (p->wait_flag1) {
			clear_vop_flag1();
			poll_vop_flag1();
		}

		if (p->wait_flag0)
			clear_vop_flag0();

		sram_param.stop_cpu_start = read_cntpct_el0();
		stop_cpus();
		sram_param.stop_cpu_end = read_cntpct_el0();
		if ((sram_param.stop_cpu_end -
		    sram_param.stop_cpu_start) >
		    sram_param.stop_cpu_delay)
			sram_param.stop_cpu_delay =
				(sram_param.stop_cpu_end -
				 sram_param.stop_cpu_start);

		if (!p->wait_flag0)
			break;

		if (check_vop_flag0_gone()) {
			/* flag0 have gone, let other cpu start */
			start_cpus();
		} else {
			wait_flag0 = 1;
			break;
		}
	}

	return wait_flag0;
}
#endif

__sramfunc void ddr_set_rate_sram(uint32_t mhz, uint32_t dest_mode,
				  struct rk3328_ddr_sram_param *p_sram_param)
{
	uint32_t dfi_lp_en_sr = 0;
	uint32_t derate_enable = 0;
	uint32_t pwrctl;
	uint32_t cur_mode;
	uint32_t init3;
	uint32_t dll_mode;
	uint32_t cnt;
	uint32_t dis_auto_zq = 0;
	uint32_t dis_auto_refresh = 0;
	uint32_t dramtype = p_sram_param->dramtype;
	struct set_rate_rel_timing *timing;

	fiq_dfs_flush_l1();
	fiq_dfs_flush_l2();
#ifdef SYNC_WITH_LCDC_FRAME_INTR
	if (p_sram_param->wait_flag0)
		poll_vop_flag0();
#endif
	p_sram_param->set_rate_start = read_cntpct_el0();
	fiq_dfs_prefetch(register_prefetch);

	cur_mode = (~dest_mode) & 1;
	timing = &(p_sram_param->timing[dest_mode]);
	init3 = mmio_read_32(UMCTL2_REGS_FREQ(cur_mode) + DDR_PCTL2_INIT3);
	if (dramtype == LPDDR3) {
		dll_mode = DLL_ON_2_ON;
	} else {
		if (((init3 & 1) == 0) && ((timing->init3 & 1) == 0))
			dll_mode = DLL_ON_2_ON;
		else if (((init3 & 1) == 0) && ((timing->init3 & 1) == 1))
			dll_mode = DLL_ON_2_OFF;
		else if (((init3 & 1) == 1) && ((timing->init3 & 1) == 0))
			dll_mode = DLL_OFF_2_ON;
		else
			dll_mode = DLL_OFF_2_OFF;
	}

	pwrctl = mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRCTL);
	mmio_write_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRCTL, 0);
	while ((mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_STAT) &
			     PCTL2_OPERATING_MODE_MASK) !=
			     PCTL2_OPERATING_MODE_INIT)
		continue;

	idle_port();
	mmio_clrbits_32(DDRC_BASE_ADDR + DDR_PCTL2_PCTRLN, PCTL2_PORT_EN);
	while (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_PSTAT) &
			    (PCTL2_WR_PORT_BUSY_0 | PCTL2_RD_PORT_BUSY_0))
		continue;
	if (mmio_read_32(UMCTL2_REGS_FREQ(cur_mode) + DDR_PCTL2_DERATEEN) &
			PCTL2_DERATE_ENABLE) {
		derate_enable = 1;
		mmio_clrbits_32(UMCTL2_REGS_FREQ(cur_mode) +
				DDR_PCTL2_DERATEEN,
				PCTL2_DERATE_ENABLE);
		mmio_clrbits_32(UMCTL2_REGS_FREQ(dest_mode) +
				DDR_PCTL2_DERATEEN,
				PCTL2_DERATE_ENABLE);
	}
	mmio_setbits_32(DDRC_BASE_ADDR + DDR_PCTL2_DBG1, PCTL2_DIS_HIF);
	if (!(mmio_read_32(UMCTL2_REGS_FREQ(cur_mode) + DDR_PCTL2_ZQCTL0) &
			PCTL2_DIS_AUTO_ZQ)) {
		dis_auto_zq = 1;
	}
	mmio_setbits_32(UMCTL2_REGS_FREQ(cur_mode) + DDR_PCTL2_ZQCTL0,
			PCTL2_DIS_AUTO_ZQ);
	mmio_setbits_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_ZQCTL0,
			PCTL2_DIS_AUTO_ZQ);
	cnt = 0;
	while (1) {
		if ((mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_DBGCAM) &
			(PCTL2_DBG_LPR_Q_DEPTH_MASK |
			PCTL2_DBG_WR_Q_EMPTY |
			PCTL2_DBG_RD_Q_EMPTY))
			== (PCTL2_DBG_LPR_Q_DEPTH_EMPTY |
			PCTL2_DBG_WR_Q_EMPTY |
			PCTL2_DBG_RD_Q_EMPTY)) {
			cnt++;
		}
		if (cnt >= 4)
			break;
	}
	/* MRS disable RTT_NOM */
	/* MRS disable RTT_WR */
	if (dramtype == LPDDR3) {
		write_mr(3, 11, 0, dramtype);
	} else {
		write_mr(3, 1, init3 & ~DDR3_RTT_NOM_MASK,
			 dramtype);
		if (dll_mode == DLL_ON_2_OFF) {
			/* MRS disable RTT_NOM */
			/* MRS disable RTT_WR */
			/* MRS disable RTT_PARK */
			/* MRS disable DLL */
			write_mr(3, 1, init3 | DDR3_DLL_DISABLE, dramtype);
		}
	}
	/* write MR6 for DDR4 tDLLK */
	if (dramtype == DDR4)
		write_mr(3, 6, timing->init7 & 0xffff, dramtype);
	if (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_DFILPCFG0) &
			PCTL2_DFI_LP_EN_SR_MASK) {
		dfi_lp_en_sr = 1;
		mmio_clrbits_32(DDRC_BASE_ADDR + DDR_PCTL2_DFILPCFG0,
				PCTL2_DFI_LP_EN_SR);
		while (mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_DFISTAT) &
				    PCTL2_DFI_LP_ACK)
			continue;
	}
	while ((mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_STAT) &
			     PCTL2_OPERATING_MODE_MASK) ==
			     PCTL2_OPERATING_MODE_SR)
		continue;
	if (!(mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_RFSHCTL3) &
				PCTL2_DIS_AUTO_REFRESH))
		dis_auto_refresh = 1;
	mmio_setbits_32(DDRC_BASE_ADDR + DDR_PCTL2_RFSHCTL3,
			PCTL2_DIS_AUTO_REFRESH);
	update_refresh_reg();
	/* enter self-refresh */
	mmio_setbits_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRCTL, PCTL2_SELFREF_SW);
	while (1) {
		if (((mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_STAT) &
				   PCTL2_SELFREF_TYPE_MASK) ==
				   PCTL2_SELFREF_TYPE_SR_NOT_AUTO) &&
		((mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_STAT) &
				PCTL2_OPERATING_MODE_MASK) ==
				PCTL2_OPERATING_MODE_SR)) {
			break;
		}
	}
	/* set bufferen to 0 to avoid unexpected when phy re-init reset */
	mmio_write_32(GRF_BASE + GRF_SOC_CON(2), GRF_CON_DDRPHY_BUFFEREN_MASK |
			GRF_CON_DDRPHY_BUFFEREN_EN);
	sw_set_req();
	mmio_clrbits_32(DDRC_BASE_ADDR + DDR_PCTL2_DFIMISC,
			PCTL2_DFI_INIT_COMPLETE_EN);
	sw_set_ack();
	sw_set_req();
	if (dll_mode == DLL_OFF_2_ON)
		mmio_clrbits_32(DDRC_BASE_ADDR + DDR_PCTL2_MSTR,
				PCTL2_DLL_OFF_MODE);
	else if (dll_mode == DLL_ON_2_OFF)
		mmio_setbits_32(DDRC_BASE_ADDR + DDR_PCTL2_MSTR,
				PCTL2_DLL_OFF_MODE);
	sw_set_ack();
	sw_set_req();
	mmio_setbits_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_ZQCTL0,
			PCTL2_DIS_SRX_ZQCL);
	mmio_setbits_32(UMCTL2_REGS_FREQ(cur_mode) + DDR_PCTL2_ZQCTL0,
			PCTL2_DIS_SRX_ZQCL);
	sw_set_ack();
	mmio_write_32(CRU_BASE + CRU_CLKGATE_CON(18),
		      DDR_MSCH_UPCTL_EN_MASK |
		      (0x7 << DDR_MSCH_UPCTL_EN_SHIFT));
	/* phy soft reset */
	mmio_clrbits_32(PHY_REG(0), PHY_ANALOG_DERESET | PHY_DIGITAL_DERESET);
	/* set PLL here */
	ddr_set_pll(mhz, 1);
	ddr_set_pll(mhz, 2);
	/* PHY setting here
	 * PHY soft reset
	 * CL,CWL,bypass
	 */
	phy_dll_bypass_set(p_sram_param, mhz);
	/* soft de-reset analog logic */
	mmio_setbits_32(PHY_REG(0), PHY_ANALOG_DERESET);
	sram_udelay(5);
	/* soft de-reset digital core */
	mmio_setbits_32(PHY_REG(0), PHY_DIGITAL_DERESET);
	sram_udelay(1);
	/* set bufferen to 1 (IO retention) */
	mmio_write_32(GRF_BASE + GRF_SOC_CON(2), GRF_CON_DDRPHY_BUFFEREN_MASK |
			GRF_CON_DDRPHY_BUFFEREN_DIS);
	ddr_update_odt(p_sram_param, dest_mode);
	ddr_update_skew(p_sram_param);
	mmio_write_32(CRU_BASE + CRU_CLKGATE_CON(18),
		      DDR_MSCH_UPCTL_EN_MASK |
		      (0x0 << DDR_MSCH_UPCTL_EN_SHIFT));

	while ((mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_DFISTAT) &
	       PCTL2_DFI_INIT_COMPLETE) != PCTL2_DFI_INIT_COMPLETE)
		continue;

	sw_set_req();
	update_pctl_timing(dest_mode, timing, p_sram_param->dramtype);
	update_refresh_reg();
	sw_set_ack();

	sw_set_req();
	mmio_clrsetbits_32(DDRC_BASE_ADDR + DDR_PCTL2_MSTR,
			   PCTL2_FREQUENCY_MODE_MASK <<
			   PCTL2_FREQUENCY_MODE_SHIFT,
			   dest_mode << PCTL2_FREQUENCY_MODE_SHIFT);
	sw_set_ack();

	update_refresh_reg();

	sw_set_req();
	/* set dfi_init_complete_en */
	mmio_setbits_32(DDRC_BASE_ADDR + DDR_PCTL2_DFIMISC,
			PCTL2_DFI_INIT_COMPLETE_EN);
	sw_set_ack();

	mmio_clrsetbits_32(DDRC_BASE_ADDR + DDR_PCTL2_DFILPCFG0,
			   PCTL2_DFI_LP_EN_SR_MASK,
			   dfi_lp_en_sr << PCTL2_DFI_LP_EN_SR_SHIFT);
	/* wait 2*tREFI to avoid violate the JEDEC */
	/* exit self-refresh */
	mmio_clrbits_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRCTL, PCTL2_SELFREF_SW);
	while ((mmio_read_32(DDRC_BASE_ADDR + DDR_PCTL2_STAT) &
	       PCTL2_OPERATING_MODE_MASK) == PCTL2_OPERATING_MODE_SR)
		continue;
	mmio_write_32(PHY_REG(0xA), timing->phyreg0a);
	mmio_write_32(PHY_REG(0xC), timing->phyreg0c);
	/* send MRS/MRW here */
	if (dramtype == LPDDR3) {
		write_mr(3, 1, (timing->init3 >> PCTL2_LPDDR3_MR1_SHIFT) &
			 PCTL2_MR_MASK,
			 dramtype);
		write_mr(3, 2, timing->init3 & PCTL2_MR_MASK,
			 dramtype);
		write_mr(3, 3, (timing->init4 >> PCTL2_LPDDR3_MR3_SHIFT) &
			 PCTL2_MR_MASK,
			 dramtype);
		write_mr(3, 11, timing->mr11, dramtype);
	} else {
		if (dll_mode == DLL_OFF_2_ON) {
			/* MRS to enable DLL */
			write_mr(3, 1, timing->init3 & PCTL2_MR_MASK,
				 dramtype);
			/* MRS to reset DLL */
			write_mr(3, 0, (timing->init3 >> PCTL2_MR0_SHIFT) &
				 PCTL2_MR_MASK,
				 dramtype);
			sram_udelay(2);
			/* MRS other MR */
			/* MRS re-enable RTT_NOM */
			/* MRS re-enable RTT_PARK */
		} else {
			/* MRS other MR */
			write_mr(3, 1, timing->init3 & PCTL2_MR_MASK,
				 dramtype);
			write_mr(3, 0, (timing->init3 >> PCTL2_MR0_SHIFT) &
				 PCTL2_MR_MASK,
				 dramtype);
		}
		write_mr(3, 2, (timing->init4 >> PCTL2_MR2_SHIFT) &
			 PCTL2_MR_MASK,
			 dramtype);
		if (dramtype == DDR4) {
			write_mr(3, 3, timing->init4 & PCTL2_MR_MASK,
				 dramtype);
			write_mr(3, 4, (timing->init6 >> PCTL2_MR4_SHIFT) &
				 PCTL2_MR_MASK,
				 dramtype);
			write_mr(3, 5, timing->init6 & PCTL2_MR_MASK,
				 dramtype);
			write_mr(3, 6, timing->init7 & PCTL2_MR_MASK,
				 dramtype);
		}
	}
	/* PHY training */
	if (data_training(0, dramtype))
		__asm volatile ("b .");
	if (dis_auto_zq)
		mmio_clrbits_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_ZQCTL0,
				PCTL2_DIS_AUTO_ZQ);
	if (dis_auto_refresh) {
		mmio_clrbits_32(DDRC_BASE_ADDR + DDR_PCTL2_RFSHCTL3,
				PCTL2_DIS_AUTO_REFRESH);
		update_refresh_reg();
	}
	sw_set_req();
	mmio_clrbits_32(UMCTL2_REGS_FREQ(dest_mode) + DDR_PCTL2_ZQCTL0,
			PCTL2_DIS_SRX_ZQCL);
	mmio_clrbits_32(UMCTL2_REGS_FREQ(cur_mode) + DDR_PCTL2_ZQCTL0,
			PCTL2_DIS_SRX_ZQCL);
	sw_set_ack();
	mmio_clrbits_32(DDRC_BASE_ADDR + DDR_PCTL2_DBG1, PCTL2_DIS_HIF);
	if (derate_enable)
		mmio_setbits_32(UMCTL2_REGS_FREQ(dest_mode) +
				DDR_PCTL2_DERATEEN,
				PCTL2_DERATE_ENABLE);
	mmio_setbits_32(DDRC_BASE_ADDR + DDR_PCTL2_PCTRLN, PCTL2_PORT_EN);
	deidle_port();
	mmio_write_32(DDRC_BASE_ADDR + DDR_PCTL2_PWRCTL, pwrctl);

	p_sram_param->set_rate_end = read_cntpct_el0();
}

uint32_t ddr_get_rate(void)
{
	if (mmio_read_32(PHY_REG(0xef)) & (1 << 7))
		return (get_phypll() * MHZ) / 2;
	else
		return (get_dpll() * MHZ) / 2;
}

uint32_t ddr_set_rate(uint32_t page_type)
{
	uint64_t base_addr;
	struct share_params *p;
	uint32_t mhz;
	uint32_t dest_mode;
	uint64_t daif;

	if (page_type != SHARE_PAGE_TYPE_DDR)
		goto err;
	if (0 != share_mem_type2page_base(page_type, &base_addr))
		goto err;

	p = (struct share_params *)base_addr;
	VERBOSE("base_addr:%lx, hz=%d\n", base_addr, p->hz);
	if (p->hz < MHZ) {
		if (p->hz < 786)
			goto err;
		mhz = p->hz;
	} else {
		if (p->hz < (786 * MHZ))
			goto err;
		mhz = p->hz / MHZ;
	}

	dest_mode = (~(dram_param.current_index)) & 1;
	VERBOSE("current cpu : %x\n", plat_my_core_pos());
	pre_set_rate(mhz, dest_mode, &sram_param);

#ifdef SYNC_WITH_LCDC_FRAME_INTR
	sram_param.wait_flag0 = wait_vop_vbank(p);
#else
	sram_param.wait_flag0 = 0;
	/* other cpu enter wfe */
	sram_param.stop_cpu_start = read_cntpct_el0();
	stop_cpus();
	sram_param.stop_cpu_end = read_cntpct_el0();
	if ((sram_param.stop_cpu_end - sram_param.stop_cpu_start) >
			sram_param.stop_cpu_delay)
		sram_param.stop_cpu_delay =
			sram_param.stop_cpu_end - sram_param.stop_cpu_start;
#endif

	daif = read_daif();
	write_daifset(DISABLE_ALL_EXCEPTIONS);
	rockchip_en_el3_abort();

	sram_param.save_sp = rockchip_get_sp();
	dsb();
	rockchip_set_sp(((uint64_t)&sram_param.sram_sp +
			(uint64_t)sizeof(sram_param.sram_sp)) & (~0xF));
	ddr_set_rate_sram(mhz, dest_mode, &sram_param);
	rockchip_set_sp(sram_param.save_sp);

	write_daif(daif);
	start_cpus();

	post_set_rate(mhz, dest_mode, &sram_param);
	if ((sram_param.set_rate_end - sram_param.set_rate_start) >
			sram_param.set_rate_delay)
		sram_param.set_rate_delay =
			(sram_param.set_rate_end - sram_param.set_rate_start);
	VERBOSE("stop cpu use %ld us, max delay %ld us\n",
		(sram_param.stop_cpu_end - sram_param.stop_cpu_start) / 24,
		 sram_param.stop_cpu_delay / 24);
	VERBOSE("ddr scale freq use %ld us, max %ld us\n",
		(sram_param.set_rate_end - sram_param.set_rate_start) / 24,
		 sram_param.set_rate_delay / 24);

	if (p->hz < MHZ)
		return mhz;
	else
		return mhz * MHZ;
err:
	return 0;
}

uint32_t ddr_round_rate(uint32_t page_type)
{
	uint64_t base_addr;
	uint32_t ret;
	struct share_params *p;

	if (page_type != SHARE_PAGE_TYPE_DDR)
		goto err;
	if (0 != share_mem_type2page_base(page_type, &base_addr))
		goto err;
	p = (struct share_params *)base_addr;
	VERBOSE("base_addr:%lx, hz=%d\n", base_addr, p->hz);
	if (p->hz < MHZ) {
		if (p->hz < 786)
			goto err;
		ret = ddr_set_pll(p->hz, 0);
	} else {
		if (p->hz < (786 * MHZ))
			goto err;
		ret = ddr_set_pll(p->hz / MHZ, 0)
			* MHZ;
	}
	return ret;
err:
	return 0;
}

void ddr_set_auto_self_refresh(uint32_t page_type)
{
	uint64_t base_addr;
	struct share_params *p;

	if (page_type != SHARE_PAGE_TYPE_DDR)
		goto err;
	if (0 != share_mem_type2page_base(page_type, &base_addr))
		goto err;
	p = (struct share_params *)base_addr;
	VERBOSE("base_addr:%lx\n", base_addr);
	sram_param.sr_idle_en = p->sr_idle_en;
err:
	return;
}

void ddr_dfs_init(uint32_t page_type)
{
	uint64_t base_addr;

	if (page_type != SHARE_PAGE_TYPE_DDR)
		return;
	if (0 != share_mem_type2page_base(page_type, &base_addr))
		return;
	VERBOSE("base_addr:%lx\n", base_addr);
	memcpy((void *)&dts_parameter,
	       (void *)(base_addr + DTS_PAR_OFFSET),
		sizeof(dts_parameter));
	ddr_sram_param_init(&dts_parameter, &sram_param);
	dram_param_init(&sram_param, &dram_param);
	ddr_related_init(&dts_parameter, &sram_param);
}

uint32_t ddr_get_version(void)
{
	/* version V1.01 */
	return 0x101;
}

/*****************************************************************************
 * for fiq cpu stop
 *****************************************************************************/
static uint32_t fiq_dfs_get_poweroff_cpus_msk(void)
{
	uint32_t pd_reg, apm_reg;
	uint32_t pd_en_bit, apm_en_bit, apm_off_bit;
	uint32_t off_msk, i;

	pd_reg = mmio_read_32(PMU_BASE + PMU_PWRDN_CON);

	off_msk = 0;

	for (i = 0; i < PLATFORM_CORE_COUNT; i++) {
		apm_reg = mmio_read_32(PMU_BASE + PMU_CPUAPM_CON(i));
		pd_en_bit = pd_reg & BIT(i);

		apm_en_bit = apm_reg & BIT(core_pm_en);
		apm_off_bit = apm_reg & BIT(core_pm_int_wakeup_en);

		if (pd_en_bit && !apm_en_bit) {
			off_msk |= BIT(i);
		} else if (!pd_en_bit && apm_en_bit) {
			if (!apm_off_bit)
				off_msk |= BIT(i);
		} else if (pd_en_bit && apm_en_bit) {
			ERROR("%s: %x,%x", __func__, pd_reg, apm_reg);
		}
	}

	return off_msk;
}

int fiq_dfs_wait_cpus_wfe(void)
{
	int i, loop = 1000 * 1000;
	uint32_t pwroff_cpus, wfe_st, tgt_wfe;
	uint32_t cpu_cur = plat_my_core_pos();
	/*
	 * The system ensure that the cpu is not up again.
	 */
	pwroff_cpus = fiq_dfs_get_poweroff_cpus_msk();

	/*
	 * The online cpu is in stop status
	 */
	for (i = 0; i < PLATFORM_CORE_COUNT; i++) {
		if (i == cpu_cur)
			continue;
		if (pwroff_cpus & BIT(i))
			continue;
		if (wait_cpu_stop_st_timeout(i))
			return -1;
	}

	tgt_wfe = (~pwroff_cpus) & CPUS_PWRDM_MSK;
	tgt_wfe &= (~BIT(cpu_cur));

	do {
		wfe_st = mmio_read_32(GRF_BASE + GRF_CPU_STATUS(1)) &
				      CPUS_PWRDM_MSK;
		udelay(1);
		loop--;
	} while ((wfe_st & tgt_wfe) != tgt_wfe && loop > 0);

	if (loop <= 0)
		return -1;
	else
		return 0;
}
