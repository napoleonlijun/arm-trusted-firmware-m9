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

#ifndef __PMU_H__
#define __PMU_H__

/*****************************************************************************
 * The ways of cores power domain contorlling
 *****************************************************************************/
enum pmu_pd_state {
	pmu_pd_on = 0,
	pmu_pd_off = 1
};

enum cores_pm_ctr_mode {
	core_pwr_pd = 0,
	core_pwr_wfi = 1,
	core_pwr_wfi_int = 2
};

enum pmu_cores_pm_by_wfi {
	core_pm_en = 0,
	core_pm_int_wakeup_en,
	core_pm_resv,
	core_pm_sft_wakeup_en
};

#define CORES_PM_DISABLE	0x0

extern void *pmu_cpuson_entrypoint_start;
extern void *pmu_cpuson_entrypoint_end;
extern uint64_t cpuson_entry_point[PLATFORM_CORE_COUNT];
extern uint32_t cpuson_flags[PLATFORM_CORE_COUNT];

/*****************************************************************************
 * pmugrf iomux
 *****************************************************************************/
#define GPIO0A_IOMUX		0x0
#define GPIO0B_IOMUX		0x4
#define GPIO0C_IOMUX		0x8
#define GPIO0D_IOMUX		0xC

/*****************************************************************************
 * pmu con,reg
 *****************************************************************************/
#define PMU_WKUP_CFG0		0x0
#define PMU_WKUP_CFG1		0x4
#define PMU_WKUP_CFG2		0x8
#define PMU_TIMEOUT_CNT		0x7c
#define PMU_PWRDN_CON		0xc
#define PMU_PWRDN_ST		0x10
#define PMU_PWRMD_CORE		0x14
#define PMU_PWRMD_COM		0x18
#define PMU_SFT_CON		0x1c
#define PMU_CORE_PWR_ST		0x38
#define PMU_BUS_IDLE_REQ	0x3c
#define PMU_BUS_IDLE_ST		0x40
#define PMU_POWER_ST		0x44
#define PMU_OSC_CNT		0x48
#define PMU_PLL_LOCK_CNT	0x4c
#define PMU_PLL_RST_CNT		0x50
#define PMU_STABLE_CNT		0x54
#define PMU_WKUP_RST_CNT	0x5c
#define PMU_CPUAPM_CON(cpu)	(0x80 + (cpu * 4))

#define CHECK_CPU_WFIE_BASE	(PMU_BASE + PMU_CORE_PWR_ST)

enum pmu_pd_id {
	PD_CPU0 = 0,
	PD_CPU1,
	PD_CPU2,
	PD_CPU3,
	PD_SCU,
	PD_WIFIBT = 8,
	PD_BUS,
	PD_PERI,
	PD_RKVDEC,
	PD_VPU,
	PD_VIDEO,
	PD_VIO,
	PD_GPU = 15,
	PD_CHIP = 18,
};

enum pmu_bus_idle_req {
	bus_idle_req_gpu = 2,
	bus_idle_req_core,
	bus_idle_req_bus = 4,
	bus_idle_req_dma,
	bus_idle_req_peri,
	bus_idle_req_video,
	bus_idle_req_vio = 8,
	bus_idle_req_wifibt,
	bus_idle_req_cxcs,
	bus_idle_req_msch,
	bus_idle_req_alive = 12,
	bus_idle_req_pmu,
};

static const int8_t pmu_bus_idle_st[] = {
	[bus_idle_req_gpu] = 2,
	[bus_idle_req_core] = 3,
	[bus_idle_req_bus] = 4,
	[bus_idle_req_dma] = 5,
	[bus_idle_req_peri] = 6,
	[bus_idle_req_video] = 7,
	[bus_idle_req_vio] = 8,
	[bus_idle_req_wifibt] = 9,
	[bus_idle_req_msch] = 11,
	[bus_idle_req_alive] = 12,
	[bus_idle_req_pmu] = 13,
	[bus_idle_req_cxcs] = 14,
};

/************************ pmu define ***************************************/
enum pmu_power_mode_core {
	pmu_mdcr_global_int_dis = 0,
	pmu_mdcr_core_src_gt,
	pmu_mdcr_res0,
	pmu_mdcr_cpu0_pd,
	pmu_mdcr_res1 = 4,
	pmu_mdcr_clr_core,
	pmu_mdcr_scu_pd,
	pmu_mdcr_core_pd,
	pmu_mdcr_l2_idle = 8,
	pmu_mdcr_l2_flush,
	pmu_mdcr_clr_cxcs,
	pmu_mdcr_res2,
	pmu_mdcr_mpll_pd = 12,
	pmu_mdcr_wpll_pd,
	pmu_mdcr_bpll_pd,
	pmu_mdcr_apll_pd,
	pmu_mdcr_dpll_pd = 16,
	pmu_mdcr_cpll_pd,
	pmu_mdcr_gpll_pd,
	pmu_mdcr_npll_pd = 19,
};

enum pmu_power_mode_common {
	pmu_mode_en = 0,
	pmu_mode_res,
	pmu_mode_bus_pd,
	pmu_mode_wkup_rst,
	pmu_mode_pll_pd = 4,
	pmu_mode_pwr_off,
	pmu_mode_pmu_use_if,
	pmu_mode_pmu_alive_use_if,
	pmu_mode_osc_dis = 8,
	pmu_mode_input_clamp,
	pmu_mode_sref_enter,
	pmu_mode_ddrc_gt,
	pmu_mode_ddrio_ret = 12,
	pmu_mode_ddrio_ret_dreq,
	pmu_mode_clr_pmu,
	pmu_mode_clr_alive,
	pmu_mode_clr_bus = 16,
	pmu_mode_clr_dma,
	pmu_mode_clr_msch,
	pmu_mode_clr_wifibt,
	pmu_mode_clr_peri = 20,
	pmu_mode_clr_video,
	pmu_mode_clr_vio,
	pmu_mode_clr_gpu,
	pmu_mode_clr_cxcs = 24,
	pmu_mode_clr_upctl,
	pmu_mode_res0,
	pmu_mode_res1,
	pmu_mode_res2 = 28,
	pmu_mode_wait_wkup_cfg_begin,
};

enum pmugrf_rst_hold {
	chip_32k_src = 0,
	ddrphy_bufen_core_en,
	ddrphy_bufen_io_out_en,
	ddrphy_bufen_io_oen_en,
	pd_pmu_rst_hold_niuh = 4,
	pd_pmu_rst_hold_niup,
	pd_pmu_rst_hold_gpio0,
	pd_pmu_rst_hold_gpio1,
	pd_pmu_rst_hold_pmugrf = 8,
	pd_pmu_rst_hold_sgrf,
	pd_pmu_rst_hold_pmupvtm,
	pd_pmu_rst_hold_pmumcu,
	pd_pmu_rst_hold_pmui2c = 12,
	pd_pmu_rst_hold_intmem1,
	gpio0_int_wakeup_en,
	gpio1_int_wakeup_en = 15,
};

enum pmu_sft_con {
	pmu_sft_ret_cfg = 11,
};

/*****************************************************************************
 * power domain on or off
 *****************************************************************************/

enum pmu_bus_req {
	idle_dis = 0,
	idle_en = 1
};

#define CYCLE_32K_MS(ms)	(32 * ms)
#define CYCLE_24M_US(us)	(24 * us)
#define CYCLE_24M_MS(ms)	(ms * CYCLE_24M_US(1000))

#define CPU_WFIE_MSK		0x44
#define PD_CHECK_LOOP		500
#define WFEI_CHECK_LOOP		500
#define BUS_CHECK_LOOP		500
#define PVTM_CHECK_LOOP		500
#define TIMEROUT_32khz_S(n)	(32 * 1000 * n)

#define PMU_CORE_WAKEUP		BIT(1)
#define PMU_GPIO_WAKEUP		BIT(2)
#define PMU_CORE_GPIO_WAKEUP	(PMU_CORE_WAKEUP | PMU_GPIO_WAKEUP)

void pin_set_fun(uint8_t port, uint8_t bank, uint8_t b_gpio, uint8_t fun);

#endif /* __PMU_H__ */
