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
#include <assert.h>
#include <bakery_lock.h>
#include <console.h>
#include <platform.h>
#include <delay_timer.h>
#include <arch_helpers.h>
#include <platform_def.h>
#include <plat_private.h>
#include <rk3366_def.h>
#include <pmu_sram.h>
#include <errno.h>
#include <soc.h>
#include <pmu.h>

static struct psram_data_t *psram_sleep_cfg =
		(struct psram_data_t *)PSRAM_DT_BASE;

static uint32_t cores_pd_cfg_info[PLATFORM_CORE_COUNT]
#if USE_COHERENT_MEM
__attribute__ ((section("tzfw_coherent_mem")))
#endif
;/* coheront */

extern uint32_t g_uart_port, g_uart_base, g_uart_baudrate;

/*****************************************************************/
struct rk_pmu_data {
	uint32_t pwrdn_st;
	uint32_t sleep_cnt;
	uint32_t pmu_sft_con;
	uint32_t pmugrf_soc0_pmu_config;
	uint32_t pmugrf_soc0_pvtm_config;
	uint32_t clk_gate7;
	uint32_t pvtm_con[2];
	uint32_t pmugrf_dll_con[2];
	uint32_t clk_gate2;
	uint32_t clk_gate13;
	uint32_t cpuapm_con0;
	uint32_t gpio0c_iomux;

	/* Qos */
	uint32_t rkvdec_w_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t vpu_r_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t vpu_w_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t iep_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t isp_r0_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t isp_r1_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t isp_w0_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t isp_w1_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t vop0_w_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t rga_r_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t rga_w_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t vop0_r_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t vop1_r_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t hdcp_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t gpu_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t peri0_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t peri1_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t usb3_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t rkvdec_r_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t wifi_dma_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t bt_dma_qos[CPU_AXI_QOS_NUM_REGS];
};

#if USE_COHERENT_MEM
struct rk_pmu_data rkpmu __attribute__ ((section("tzfw_coherent_mem")));
#else
struct rk_pmu_data rkpmu;
#endif

static uint32_t cpu_warm_boot_addr;
int skip_suspend;

void plat_rockchip_pmusram_prepare(void)
{
	uint32_t *sram_dst, *sram_src;
	size_t sram_size;

	/*
	 * pmu sram code and data prepare
	 */
	sram_dst = (uint32_t *)PMUSRAM_BASE;
	sram_src = (uint32_t *)&pmu_cpuson_entrypoint_start;
	sram_size = (uint32_t *)&pmu_cpuson_entrypoint_end -
		    (uint32_t *)sram_src;

	u32_align_cpy(sram_dst, sram_src, sram_size);

	psram_sleep_cfg->sp = PSRAM_DT_BASE;
}

static inline uint32_t pmu_power_domain_st(uint32_t pd)
{
	return (pmu_read32(PMU_PWRDN_ST) & BIT(pd)) ? pmu_pd_off : pmu_pd_on;
}

static int pmu_power_domain_ctr(uint32_t pd, uint32_t tgt)
{
	uint32_t val, loop = 0;

	val = pmu_read32(PMU_PWRDN_CON);
	if (tgt == pmu_pd_off)
		val |= BIT(pd);
	else
		val &= ~BIT(pd);

	pmu_write32(val, PMU_PWRDN_CON);
	dsb();

	while (!(pmu_power_domain_st(pd) == tgt) && (loop < PD_CHECK_LOOP)) {
		udelay(2);
		loop++;
	}

	if (pmu_power_domain_st(pd) != tgt)
		WARN("%s: %d, %d, 0x%x, 0x%x , error!\n",
		     __func__, pd, tgt, val, pmu_read32(PMU_PWRDN_ST));

	return 0;
}

static int check_cpu_wfie(uint32_t cpu)
{
	uint32_t loop = 0, wfie_msk = CPU_WFIE_MSK << cpu;

	while (!(pmu_read32(PMU_CORE_PWR_ST) & wfie_msk) &&
	       (loop < WFEI_CHECK_LOOP)) {
		udelay(1);
		loop++;
	}

	if ((pmu_read32(PMU_CORE_PWR_ST) & wfie_msk) == 0) {
		WARN("%s: %d, %d, error!\n", __func__, cpu, wfie_msk);
		return -EINVAL;
	}

	return 0;
}

static inline uint32_t get_cpus_pwr_domain_cfg_info(uint32_t cpu_id)
{
	return cores_pd_cfg_info[cpu_id];
}

static inline void set_cpus_pwr_domain_cfg_info(uint32_t cpu_id, uint32_t value)
{
	cores_pd_cfg_info[cpu_id] = value;
#if !USE_COHERENT_MEM
	flush_dcache_range((uintptr_t) &cores_pd_cfg_info[cpu_id],
			    sizeof(uint32_t));
#endif
}

static int cpus_power_domain_on(uint32_t cpu_id)
{
	uint32_t cpu_pd, apm_value, cfg_info, loop = 0;

	cpu_pd = PD_CPU0 + cpu_id;
	cfg_info = get_cpus_pwr_domain_cfg_info(cpu_id);

	if (cfg_info == core_pwr_pd) {
		/* disable apm cfg */
		pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));
		if (pmu_power_domain_st(cpu_pd) == pmu_pd_on) {
			pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));
			pmu_power_domain_ctr(cpu_pd, pmu_pd_off);
		}
		pmu_power_domain_ctr(cpu_pd, pmu_pd_on);
	} else {
		/* wait cpu down */
		while (pmu_power_domain_st(cpu_pd) == pmu_pd_on && loop < 100) {
			udelay(2);
			loop++;
		}

		/* return error if can't wait cpu down */
		if (pmu_power_domain_st(cpu_pd) == pmu_pd_on) {
			WARN("%s:can't wait cpu down\n", __func__);
			return -EINVAL;
		}

		/* power up cpu in power down state */
		apm_value = BIT(core_pm_sft_wakeup_en);
		pmu_write32(apm_value, PMU_CPUAPM_CON(cpu_id));
	}

	return 0;
}

static int cpus_power_domain_off(uint32_t cpu_id, uint32_t pd_cfg)
{
	uint32_t cpu_pd, apm_value;

	cpu_pd = PD_CPU0 + cpu_id;
	if (pmu_power_domain_st(cpu_pd) == pmu_pd_off)
		return 0;

	if (pd_cfg == core_pwr_pd) {
		if (check_cpu_wfie(cpu_id))
			return -EINVAL;

		/* disable apm cfg */
		pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));

		set_cpus_pwr_domain_cfg_info(cpu_id, pd_cfg);
		pmu_power_domain_ctr(cpu_pd, pmu_pd_off);
	} else {
		set_cpus_pwr_domain_cfg_info(cpu_id, pd_cfg);
		apm_value = BIT(core_pm_en) | BIT(core_pm_resv);
		if (pd_cfg == core_pwr_wfi_int)
			apm_value |= BIT(core_pm_int_wakeup_en);
		pmu_write32(apm_value, PMU_CPUAPM_CON(cpu_id));
	}

	return 0;
}

static void nonboot_cpus_off(void)
{
	uint32_t boot_cpu, cpu;

	boot_cpu = MPIDR_AFFLVL0_VAL(read_mpidr_el1());

	/* turn off noboot cpus */
	for (cpu = 0; cpu < PLATFORM_CORE_COUNT; cpu++) {
		if (cpu == boot_cpu)
			continue;
		cpus_power_domain_off(cpu, core_pwr_pd);
	}
}

static int pmu_bus_idle(uint32_t req, uint32_t idle)
{
	uint32_t bit = pmu_bus_idle_st[req];
	uint32_t idle_mask, idle_target, val;
	uint32_t loop = 0;

	idle_mask = BIT(bit) | BIT(bit + 16);
	idle_target = (idle << bit) | (idle << (bit + 16));

	val = pmu_read32(PMU_BUS_IDLE_REQ);
	if (idle)
		val |= BIT(req);
	else
		val &= ~BIT(req);

	pmu_write32(val, PMU_BUS_IDLE_REQ);
	dsb();

	while (((pmu_read32(PMU_BUS_IDLE_ST) & idle_mask) != idle_target) &&
	       (loop < BUS_CHECK_LOOP)) {
		loop++;
		udelay(2);
	}

	if ((pmu_read32(PMU_BUS_IDLE_ST) & idle_mask) != idle_target)
		WARN("%s:st=%x(%x, %x, %x)\n", __func__,
		     pmu_read32(PMU_BUS_IDLE_ST), val, idle_mask, idle_target);

	return 0;
}

static int pmu_set_power_domain(enum pmu_pd_id pd, enum pmu_pd_state tgt)
{
	uint32_t pwrdn_st = pmu_read32(PMU_PWRDN_ST) & BIT(pd);
	enum pmu_pd_state state = pwrdn_st ? pmu_pd_off : pmu_pd_on;

	if (state == tgt)
		return 0;

	if (tgt == pmu_pd_off) {
		if ((pd == PD_RKVDEC) || (pd == PD_VPU)) {
			/* do nothing */
		} else if (pd == PD_VIO) {
			SAVE_QOS(rkpmu.iep_qos, IEP);
			SAVE_QOS(rkpmu.isp_r0_qos, ISP_R0);
			SAVE_QOS(rkpmu.isp_r1_qos, ISP_R1);
			SAVE_QOS(rkpmu.isp_w0_qos, ISP_W0);
			SAVE_QOS(rkpmu.isp_w1_qos, ISP_W1);
			SAVE_QOS(rkpmu.vop0_w_qos, VOP0_W);
			SAVE_QOS(rkpmu.rga_r_qos, RGA_R);
			SAVE_QOS(rkpmu.rga_w_qos, RGA_W);
			SAVE_QOS(rkpmu.vop0_r_qos, VOP0_R);
			SAVE_QOS(rkpmu.vop1_r_qos, VOP1_R);
			SAVE_QOS(rkpmu.hdcp_qos, HDCP);
			pmu_bus_idle(bus_idle_req_vio, idle_en);
		} else if (pd == PD_VIDEO) {
			SAVE_QOS(rkpmu.rkvdec_r_qos, RKVDEC_R);
			SAVE_QOS(rkpmu.rkvdec_w_qos, RKVDEC_W);
			SAVE_QOS(rkpmu.vpu_r_qos, VPU_R);
			SAVE_QOS(rkpmu.vpu_w_qos, VPU_W);
			pmu_bus_idle(bus_idle_req_video, idle_en);
		} else if (pd == PD_GPU) {
			SAVE_QOS(rkpmu.gpu_qos, GPU);
			pmu_bus_idle(bus_idle_req_gpu, idle_en);
		} else if (pd == PD_PERI) {
			SAVE_QOS(rkpmu.peri0_qos, PERI0);
			SAVE_QOS(rkpmu.peri1_qos, PERI1);
			SAVE_QOS(rkpmu.usb3_qos, USB3);
			pmu_bus_idle(bus_idle_req_peri, idle_en);
		} else if (pd == PD_WIFIBT) {
			SAVE_QOS(rkpmu.wifi_dma_qos, WIFI_DMA);
			SAVE_QOS(rkpmu.bt_dma_qos, BT_DMA);
			pmu_bus_idle(bus_idle_req_wifibt, idle_en);
		}
	}

	pmu_power_domain_ctr(pd, tgt);

	if (tgt == pmu_pd_on) {
		if ((pd == PD_RKVDEC) || (pd == PD_VPU)) {
			/* do nothing */
		} else if (pd == PD_VIO) {
			pmu_bus_idle(bus_idle_req_vio, idle_dis);
			RESTORE_QOS(rkpmu.iep_qos, IEP);
			RESTORE_QOS(rkpmu.isp_r0_qos, ISP_R0);
			RESTORE_QOS(rkpmu.isp_r1_qos, ISP_R1);
			RESTORE_QOS(rkpmu.isp_w0_qos, ISP_W0);
			RESTORE_QOS(rkpmu.isp_w1_qos, ISP_W1);
			RESTORE_QOS(rkpmu.vop0_w_qos, VOP0_W);
			RESTORE_QOS(rkpmu.rga_r_qos, RGA_R);
			RESTORE_QOS(rkpmu.rga_w_qos, RGA_W);
			RESTORE_QOS(rkpmu.vop0_r_qos, VOP0_R);
			RESTORE_QOS(rkpmu.vop1_r_qos, VOP1_R);
			RESTORE_QOS(rkpmu.hdcp_qos, HDCP);
		} else if (pd == PD_VIDEO) {
			pmu_bus_idle(bus_idle_req_video, idle_dis);
			RESTORE_QOS(rkpmu.rkvdec_r_qos, RKVDEC_R);
			RESTORE_QOS(rkpmu.rkvdec_w_qos, RKVDEC_W);
			RESTORE_QOS(rkpmu.vpu_r_qos, VPU_R);
			RESTORE_QOS(rkpmu.vpu_w_qos, VPU_W);
		} else if (pd == PD_GPU) {
			pmu_bus_idle(bus_idle_req_gpu, idle_dis);
			RESTORE_QOS(rkpmu.gpu_qos, GPU);
		} else if (pd == PD_PERI) {
			pmu_bus_idle(bus_idle_req_peri, idle_dis);
			RESTORE_QOS(rkpmu.peri0_qos, PERI0);
			RESTORE_QOS(rkpmu.peri1_qos, PERI1);
			RESTORE_QOS(rkpmu.usb3_qos, USB3);
		} else if (pd == PD_WIFIBT) {
			pmu_bus_idle(bus_idle_req_wifibt, idle_dis);
			RESTORE_QOS(rkpmu.wifi_dma_qos, WIFI_DMA);
			RESTORE_QOS(rkpmu.bt_dma_qos, BT_DMA);
		}
	}

	return 0;
}

static void rkpm_ctrbits_info(void)
{
	PM_DBG("\nConfig: 0x%x, times: %d\n",
	       rkpm_get_ctrbits(), ++rkpmu.sleep_cnt);

	if (rkpm_chk_ctrbits(RKPM_CTR_AUTO_WAKEUP))
		PM_DBG("RKPM_CTR_AUTO_WAKEUP\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_PMUDBG))
		PM_DBG("RKPM_CTR_PMUDBG\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_PLL_DEEP))
		PM_DBG("RKPM_CTR_PLL_DEEP\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_PWR_DMNS))
		PM_DBG("POWER DOMAIN DOWN\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_PLLS))
		PM_DBG("PLLS DOWN\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_GTCLKS))
		PM_DBG("GATING CLKS\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_PLLS))
		PM_DBG("PLL DOWN\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_PLL_DEEP))
		PM_DBG("PLL DEEP\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_DIS_24M))
		PM_DBG("DIS 24M\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_PD_PERI))
		PM_DBG("PD PERI\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_PD_WIFIBT))
		PM_DBG("PD WIFIBT\n");
	if (rkpm_chk_ctrbits(RKPM_CTR_PMU_EXT32K))
		PM_DBG("32K EXT\n");
	else if (rkpm_chk_ctrbits(RKPM_CTR_PMU_PVTM32K))
		PM_DBG("32K INT\n");
}

static void enable_console_clk(void)
{
	rkpmu.clk_gate2 = cru_read32(CRU_CLKGATE_CON(2));
	rkpmu.clk_gate13 = cru_read32(CRU_CLKGATE_CON(13));

	/* g13_5: pclk_uart2_en; gate2_4: uart2_src_en */
	cru_write32(REG_WMSK_BITS(0, 4, 0x1), CRU_CLKGATE_CON(2));
	cru_write32(REG_WMSK_BITS(0, 5, 0x1), CRU_CLKGATE_CON(13));
}

static void restore_console_clk(void)
{
	uint8_t bit;

	bit = (rkpmu.clk_gate2 & (1 << 4)) ? 1 : 0;
	cru_write32(REG_WMSK_BITS(bit, 4, 0x1), CRU_CLKGATE_CON(2));
	bit = (rkpmu.clk_gate13 & (1 << 5)) ? 1 : 0;
	cru_write32(REG_WMSK_BITS(bit, 5, 0x1), CRU_CLKGATE_CON(13));
}

static void prepare(void)
{
	enable_console_clk();
	rkpm_ctrbits_info();
}

static void finish(void)
{
	restore_console_clk();
}

static void config_info(void)
{
	uint8_t pll_id;
	uint32_t val, pll_st = 0;
	static uint32_t sleep_times;

	for (pll_id = APLL_ID; pll_id < END_PLL; pll_id++) {
		val = cru_read32(PLL_CONS(pll_id, 3));
		pll_st |= (val & 0x1) << pll_id;
	}

	PM_INFO("(%d.%d %x %x %x %x %x %d)\n",
		MAJOR_VERSION,
		MINOR_VERSION,
		rkpm_get_ctrbits(),
		pmu_read32(PMU_PWRMD_CORE),
		pmu_read32(PMU_PWRMD_COM),
		pmu_read32(PMU_PWRDN_ST),
		pll_st, ++sleep_times);
}

static void power_down_domains(void)
{
	uint32_t i, clks_save[CRU_CLKSEL_NUMS];

	if (!rkpm_chk_ctrbits(RKPM_CTR_PWR_DMNS))
		return;

	rkpmu.pwrdn_st = pmu_read32(PMU_PWRDN_ST);

	/* enable clks */
	for (i = 0; i < CRU_CLKSEL_NUMS; i++)
		clks_save[i] = cru_read32(CRU_CLKGATE_CON(i));

	for (i = 0; i < CRU_CLKSEL_NUMS; i++)
		cru_write32(0xffff0000, CRU_CLKGATE_CON(i));

	if (!(rkpmu.pwrdn_st & BIT(PD_GPU)))
		pmu_set_power_domain(PD_GPU, pmu_pd_off);

	if (!(rkpmu.pwrdn_st & BIT(PD_RKVDEC)))
		pmu_set_power_domain(PD_RKVDEC, pmu_pd_off);

	if (!(rkpmu.pwrdn_st & BIT(PD_VPU)))
		pmu_set_power_domain(PD_VPU, pmu_pd_off);

	if (!(rkpmu.pwrdn_st & BIT(PD_VIDEO)))
		pmu_set_power_domain(PD_VIDEO, pmu_pd_off);

	if (!(rkpmu.pwrdn_st & BIT(PD_VIO)))
		pmu_set_power_domain(PD_VIO, pmu_pd_off);

	if (rkpm_chk_ctrbits(RKPM_CTR_PD_WIFIBT)) {
		if (!(rkpmu.pwrdn_st & BIT(PD_WIFIBT)))
			pmu_set_power_domain(PD_WIFIBT, pmu_pd_off);
	}

	if (rkpm_chk_ctrbits(RKPM_CTR_PD_PERI)) {
		if (!(rkpmu.pwrdn_st & BIT(PD_PERI)))
			pmu_set_power_domain(PD_PERI, pmu_pd_off);
	}

	/* restore clks */
	for (i = 0; i < CRU_CLKSEL_NUMS; i++)
		cru_write32(clks_save[i] | 0xffff0000, CRU_CLKGATE_CON(i));
}

static void restore_domains(void)
{
	uint32_t i, clks_save[CRU_CLKSEL_NUMS];

	if (!rkpm_chk_ctrbits(RKPM_CTR_PWR_DMNS))
		return;

	/* enable clks */
	for (i = 0; i < CRU_CLKSEL_NUMS; i++)
		clks_save[i] = cru_read32(CRU_CLKGATE_CON(i));

	for (i = 0; i < CRU_CLKSEL_NUMS; i++)
		cru_write32(0xffff0000, CRU_CLKGATE_CON(i));

	if (rkpm_chk_ctrbits(RKPM_CTR_PD_PERI)) {
		if (!(rkpmu.pwrdn_st & BIT(PD_PERI)))
			pmu_set_power_domain(PD_PERI, pmu_pd_on);
	}

	if (rkpm_chk_ctrbits(RKPM_CTR_PD_WIFIBT)) {
		if (!(rkpmu.pwrdn_st & BIT(PD_WIFIBT)))
			pmu_set_power_domain(PD_WIFIBT, pmu_pd_on);
	}

	if (!(rkpmu.pwrdn_st & BIT(PD_VIO)))
		pmu_set_power_domain(PD_VIO, pmu_pd_on);

	if (!(rkpmu.pwrdn_st & BIT(PD_VIDEO)))
		pmu_set_power_domain(PD_VIDEO, pmu_pd_on);

	if (!(rkpmu.pwrdn_st & BIT(PD_VPU)))
		pmu_set_power_domain(PD_VPU, pmu_pd_on);

	if (!(rkpmu.pwrdn_st & BIT(PD_RKVDEC)))
		pmu_set_power_domain(PD_RKVDEC, pmu_pd_on);

	if (!(rkpmu.pwrdn_st & BIT(PD_GPU)))
		pmu_set_power_domain(PD_GPU, pmu_pd_on);

	/* restore clks */
	for (i = 0; i < CRU_CLKSEL_NUMS; i++)
		cru_write32(clks_save[i] | 0xffff0000, CRU_CLKGATE_CON(i));
}

static void pmu_wakeup_way(int8_t way)
{
	uint32_t val;

	val = pmu_read32(PMU_WKUP_CFG2);

	if (way & PMU_CORE_WAKEUP)
		val |= BIT(0);

	if (way & PMU_GPIO_WAKEUP)
		val |= BIT(2);

	pmu_write32(val, PMU_WKUP_CFG2);
}

void pmu_gpio0_wakeup(int8_t port, int8_t bank, int8_t b_gpio, int8_t type)
{
	uint32_t bit, cfg0, cfg1;

	if (port)
		return;

	bank -= 0xa;
	bit = bank * 8 + b_gpio;
	cfg0 = pmu_read32(PMU_WKUP_CFG0);
	cfg1 = pmu_read32(PMU_WKUP_CFG1);
	if (type & PMU_IO_WKUP_P) {
		cfg0 |= REG_SET_BITS(1, bit, 0x1);
		pmu_write32(cfg0, PMU_WKUP_CFG0);
	}

	if (type & PMU_IO_WKUP_N) {
		cfg1 |= REG_SET_BITS(1, bit, 0x1);
		pmu_write32(cfg1, PMU_WKUP_CFG1);
	}
}

void pin_set_fun(uint8_t port, uint8_t bank, uint8_t b_gpio, uint8_t fun)
{
	uint32_t addr;

	bank -= 0xa;
	if (port > 0)
		addr = GRF_BASE + (port - 1) * 16 + bank * 4;
	else
		addr = PMUGRF_BASE + PMUGRF_IOMUX(bank);

	mmio_write_32(addr, REG_WMSK_BITS(fun, b_gpio * 2, 0x3));
}

static void pvtm_32khz_config(void)
{
	uint32_t loop = 0;

	if (!rkpm_chk_ctrbits(RKPM_CTR_PMU_EXT32K | RKPM_CTR_PMU_PVTM32K))
		return;

	rkpmu.pmugrf_dll_con[0] = pmugrf_read32(PMUGRF_DLL_CON(0));
	rkpmu.pmugrf_dll_con[1] = pmugrf_read32(PMUGRF_DLL_CON(1));
	rkpmu.pmugrf_soc0_pvtm_config = pmugrf_read32(PMUGRF_SOC_CON(0));

	if (rkpm_chk_ctrbits(RKPM_CTR_PMU_EXT32K)) {
		/* extern 32k*/
		pmugrf_write32(REG_WMSK_BITS(0, 0, 0x1), PMUGRF_SOC_CON(0));
	} else {
		/* inner pvtm */
		pmugrf_write32(REG_WMSK_BITS(1, 0, 0x1), PMUGRF_SOC_CON(0));
		dsb();

		/* enable pmu_pvtm clk */
		rkpmu.clk_gate7 = cru_read32(CRU_CLKGATE_CON(7));
		cru_write32(0x00080000, CRU_CLKGATE_CON(7));

		/* pvtm pmu osc en */
		pmugrf_write32(0x200, PMUGRF_DLL_CON(1));
		pmugrf_write32(REG_WMSK_BITS(1, 1, 0x1), PMUGRF_DLL_CON(0));
		dsb();

		/* pvtm start */
		pmugrf_write32(REG_WMSK_BITS(1, 0, 0x1), PMUGRF_DLL_CON(0));
		while (!(pmugrf_read32(PMUGRF_DLL_STATUS(0)) & 0x1) &&
		       (loop < PVTM_CHECK_LOOP)) {
			udelay(2);
			loop++;
		}

		if (!(pmugrf_read32(PMUGRF_DLL_STATUS(0)) & 0x1)) {
			WARN("pvtm lock failed\n");
			/* use extern 32khz */
			pmugrf_write32(REG_WMSK_BITS(0, 0, 0x1),
				       PMUGRF_SOC_CON(0));
		}
	}
}

static void restore_pvtm_32khz(void)
{
	if (!rkpm_chk_ctrbits(RKPM_CTR_PMU_PVTM32K | RKPM_CTR_PMU_EXT32K))
		return;

	/* pvtm 32k */
	if (rkpm_chk_ctrbits(RKPM_CTR_PMU_PVTM32K)) {
		pmugrf_write32(rkpmu.pmugrf_dll_con[0] | 0xffff000,
			       PMUGRF_DLL_CON(0));
		pmugrf_write32(rkpmu.pmugrf_dll_con[1] | 0xffff000,
			       PMUGRF_DLL_CON(1));
		pmugrf_write32(rkpmu.pmugrf_soc0_pvtm_config | 0xffff000,
			       PMUGRF_SOC_CON(0));
		cru_write32(rkpmu.clk_gate7 | 0xffff0000, CRU_CLKGATE_CON(7));
	}
}

static void enable_pmudbg_pins(void)
{
	if (!rkpm_chk_ctrbits(RKPM_CTR_PMUDBG))
		return;

	/* pmu debug pins: [gpio0c6, gpio0c4 ~ gpio0c0], MSB-> LSB */
	pin_set_fun(0x0, 0xc, 0x6, 0x3);
	pin_set_fun(0x0, 0xc, 0x4, 0x1);
	pin_set_fun(0x0, 0xc, 0x3, 0x1);
	pin_set_fun(0x0, 0xc, 0x2, 0x2);
	pin_set_fun(0x0, 0xc, 0x1, 0x2);
	pin_set_fun(0x0, 0xc, 0x0, 0x3);
}

static void enable_auto_wakeup(void)
{
	uint32_t val;

	if (!rkpm_chk_ctrbits(RKPM_CTR_AUTO_WAKEUP))
		return;

	val = pmu_read32(PMU_WKUP_CFG2) | BIT(10);
	pmu_write32(val, PMU_WKUP_CFG2);
	pmu_write32(TIMEROUT_32khz_S(3), PMU_TIMEOUT_CNT);
}

static void restore_auto_wakeup(void)
{
	uint32_t val;

	if (!rkpm_chk_ctrbits(RKPM_CTR_AUTO_WAKEUP))
		return;

	val = pmu_read32(PMU_WKUP_CFG2);
	val &= ~BIT(10);
	pmu_write32(val, PMU_WKUP_CFG2);
}

static void restore_pmudbg_pins(void)
{
	if (!rkpm_chk_ctrbits(RKPM_CTR_PMUDBG))
		return;

	pmugrf_write32(rkpmu.gpio0c_iomux | 0xffff0000, GPIO0C_IOMUX);
}

static void pmu_suspend(void)
{
	uint32_t pwrmd_core, pwrmd_com, val;

	/* boot from pmu sram  */
	sgrf_write32((PMUSRAM_BASE >> CPU_BOOT_ADDR_ALIGN) |
		     CPU_BOOT_ADDR_WMASK,
		     SGRF_SOC_CON(1));

	psram_sleep_cfg->ddr_flag = 0;
	rkpmu.pmugrf_soc0_pmu_config = pmugrf_read32(PMUGRF_SOC_CON(0));
	rkpmu.pmu_sft_con = pmu_read32(PMU_SFT_CON);
	rkpmu.cpuapm_con0 = pmu_read32(PMU_CPUAPM_CON(0));
	rkpmu.gpio0c_iomux = pmugrf_read32(GPIO0C_IOMUX);

	/* pmic pwroff */
	pin_set_fun(0x0, 0xa, 0x1, 0x1);

	/* disable cpu apm */
	pmu_write32(0, PMU_CPUAPM_CON(0));

	/* enable debug */
	enable_pmudbg_pins();
	enable_auto_wakeup();

	/* core, gpio wakeup */
	pmu_wakeup_way(PMU_CORE_GPIO_WAKEUP);
	pmu_gpio0_wakeup(0x0, 0xb, 0x3, PMU_IO_WKUP_PN);/* pwr key */
	pmu_gpio0_wakeup(0x0, 0xa, 0x2, PMU_IO_WKUP_PN);/* pmic int */

	/* common setting */
	pwrmd_core = BIT(pmu_mdcr_cpu0_pd)
			| BIT(pmu_mdcr_scu_pd)
			| BIT(pmu_mdcr_l2_flush)
			| BIT(pmu_mdcr_l2_idle)
			| BIT(pmu_mdcr_clr_core)
			| BIT(pmu_mdcr_global_int_dis)
			;
	pwrmd_com = BIT(pmu_mode_en) | BIT(pmu_mode_wait_wkup_cfg_begin);

	if (rkpm_chk_ctrbits(RKPM_CTR_ARMPD)) {
		PM_INFO("-armpd-");
		pin_set_fun(0x0, 0xa, 0x1, 0x0);
	} else if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF)) {
		PM_INFO("-armoff-");
		pwrmd_core |= BIT(pmu_mdcr_core_pd);
		//pwrmd_com |= BIT(pmu_mode_ddrio_ret) /*it's better to set !!*/
		pwrmd_com |= BIT(pmu_mode_clr_msch)
			| BIT(pmu_mode_clr_bus)
			| BIT(pmu_mode_clr_dma)
			| BIT(pmu_mode_clr_pmu)
			| BIT(pmu_mode_clr_alive)
			| BIT(pmu_mode_pwr_off)
			;
	} else if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGPD)) {
		PM_INFO("-armoff-logpd-");
		pwrmd_core |= BIT(pmu_mdcr_core_pd);
		pwrmd_com |= BIT(pmu_mode_pwr_off)
				| BIT(pmu_mode_clr_upctl)
				| BIT(pmu_mode_wkup_rst)
				| BIT(pmu_mode_ddrio_ret)
				| BIT(pmu_mode_bus_pd)
				| BIT(pmu_mode_clr_bus)
				| BIT(pmu_mode_clr_dma)
				| BIT(pmu_mode_clr_pmu)
				| BIT(pmu_mode_clr_alive)
				| BIT(pmu_mode_clr_msch)
				;
		if (!rkpm_chk_ctrbits(RKPM_CTR_PD_PERI))
			pwrmd_com |= BIT(pmu_mode_clr_peri);

		/* pd_pmu hold : niuh ~ intmem1*/
		pmugrf_write32(REG_WMSK_BITS(0x3ff, pd_pmu_rst_hold_niuh,
					     0x3ff), PMUGRF_SOC_CON(0));
	} else if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGOFF)) {
		PM_INFO("-armoff-logoff-");
		pwrmd_core |= BIT(pmu_mdcr_core_pd);
		pwrmd_com |= BIT(pmu_mode_pwr_off)
				| BIT(pmu_mode_ddrio_ret)
				| BIT(pmu_mode_bus_pd)
				| BIT(pmu_mode_clr_bus)
				| BIT(pmu_mode_clr_dma)
				| BIT(pmu_mode_clr_pmu)
				| BIT(pmu_mode_clr_alive)
				| BIT(pmu_mode_clr_msch)
				| BIT(pmu_mode_wkup_rst)
				| BIT(pmu_mode_input_clamp)
				;
		if (!rkpm_chk_ctrbits(RKPM_CTR_PD_PERI))
			pwrmd_com |= BIT(pmu_mode_clr_peri);

		/* pd_pmu hold : niuh ~ intmem1*/
		pmugrf_write32(REG_WMSK_BITS(0x3ff, pd_pmu_rst_hold_niuh,
					     0x3ff), PMUGRF_SOC_CON(0));
	} else {
		pwrmd_com = 0;
		pwrmd_core = 0;
	}

	if (rkpm_chk_ctrbits(RKPM_CTR_DDR_GATING))
		pwrmd_com |= BIT(pmu_mode_ddrc_gt);
	if (rkpm_chk_ctrbits(RKPM_CTR_DDR_SREF))
		pwrmd_com |= BIT(pmu_mode_sref_enter);
	if (rkpm_chk_ctrbits(RKPM_CTR_PMU_PD_PLL))
		pwrmd_com |= BIT(pmu_mode_pll_pd);
	if (rkpm_chk_ctrbits(RKPM_CTR_DIS_24M))
		pwrmd_com |= BIT(pmu_mode_osc_dis);
	if (rkpm_chk_ctrbits(RKPM_CTR_PMU_EXT32K | RKPM_CTR_PMU_PVTM32K))
		pwrmd_com |= BIT(pmu_mode_pmu_use_if)/*switch to 32khz*/
			    | BIT(pmu_mode_pmu_alive_use_if);

	/* ddr io retention ctrl */
	if (pwrmd_com & BIT(pmu_mode_ddrio_ret)) {
		/* disable software control */
		val = pmu_read32(PMU_SFT_CON) | BIT(7);
		val &= ~BIT(pmu_sft_ret_cfg);
		pmu_write32(val, PMU_SFT_CON);
		if (pwrmd_com & BIT(pmu_mode_bus_pd))/*io*/
			val = REG_WMSK_BITS(0x1, ddrphy_bufen_core_en, 0x3);
		else/*core*/
			val = REG_WMSK_BITS(0x2, ddrphy_bufen_core_en, 0x3);

		pmugrf_write32(val, PMUGRF_SOC_CON(0));
	}

	/* pll resume is 24M, advice: lock cnt: 24*500, rst cnt: 24*5 */
	pmu_write32(CYCLE_24M_MS(2), PMU_PLL_LOCK_CNT);
	pmu_write32(CYCLE_24M_US(100), PMU_PLL_RST_CNT);
	if (pwrmd_com & BIT(pmu_mode_pmu_use_if)) {
		/* 32k */
		pmu_write32(CYCLE_32K_MS(10), PMU_WKUP_RST_CNT);
		pmu_write32(CYCLE_32K_MS(10), PMU_STABLE_CNT);
		pmu_write32(CYCLE_32K_MS(50), PMU_OSC_CNT);
	} else {
		/* 24M */
		pmu_write32(CYCLE_24M_MS(2), PMU_STABLE_CNT);
	}

	pmu_write32(pwrmd_core, PMU_PWRMD_CORE);
	pmu_write32(pwrmd_com, PMU_PWRMD_COM);
	dsb();
}

static void restore_pmu(void)
{
	/* boot from ddr  */
	sgrf_write32((cpu_warm_boot_addr >> CPU_BOOT_ADDR_ALIGN) |
		     CPU_BOOT_ADDR_WMASK,
		     SGRF_SOC_CON(1));

	pmu_write32(rkpmu.cpuapm_con0, PMU_CPUAPM_CON(0));
	restore_pmudbg_pins();
	restore_auto_wakeup();

	if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGPD | RKPM_CTR_ARMOFF_LOGOFF))
		plat_rockchip_gic_init();

	if (rkpm_chk_ctrbits(RKPM_CTR_ARMOFF_LOGPD | RKPM_CTR_ARMOFF_LOGOFF))
		console_init(g_uart_base, RK3366_UART_CLOCK,
			     g_uart_baudrate);

	pin_set_fun(0x0, 0xa, 0x1, 0x0);

	pmu_write32(0, PMU_PWRMD_COM);
	pmu_write32(0, PMU_PWRMD_CORE);
	pmu_write32(0, PMU_WKUP_CFG0);
	pmu_write32(0, PMU_WKUP_CFG1);

	pmugrf_write32(rkpmu.pmugrf_soc0_pmu_config | 0xffff0000,
		       PMUGRF_SOC_CON(0));
	pmu_write32(rkpmu.pmu_sft_con, PMU_SFT_CON);
}

static int cores_pwr_domain_on(unsigned long mpidr, uint64_t entrypoint)
{
	uint32_t cpu_id = plat_core_pos_by_mpidr(mpidr);

	assert(cpuson_flags[cpu_id] == 0);
	cpuson_flags[cpu_id] = PMU_CPU_HOTPLUG;
	cpuson_entry_point[cpu_id] = entrypoint;
	dsb();

	cpus_power_domain_on(cpu_id);

	return 0;
}

static int cores_pwr_domain_off(void)
{
	uint32_t cpu_id = plat_my_core_pos();

	cpus_power_domain_off(cpu_id, core_pwr_wfi);

	return 0;
}

static int cores_pwr_domain_suspend(void)
{
	uint32_t cpu_id = plat_my_core_pos();

	assert(cpuson_flags[cpu_id] == 0);
	cpuson_flags[cpu_id] = PMU_CPU_AUTO_PWRDN;
	cpuson_entry_point[cpu_id] = plat_get_sec_entrypoint();
	dsb();

	cpus_power_domain_off(cpu_id, core_pwr_wfi_int);

	return 0;
}

static int cores_pwr_domain_on_finish(void)
{
	uint32_t cpu_id = plat_my_core_pos();

	pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));

	return 0;
}

static int cores_pwr_domain_resume(void)
{
	uint32_t cpu_id = plat_my_core_pos();

	pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));

	return 0;
}

static void __dead2 soc_sys_global_soft_reset(void)
{
	cru_write32(SRST_FST_VALUE, CRU_GLB_SRST_FST_VALUE);

	/*
	 * Maybe the HW needs some times to reset the system,
	 * so we do not hope the core to excute valid codes.
	 */
	while (1)
		;
}

static int sys_pwr_domain_suspend(void)
{
	prepare();

	putchar('\n');
	putchar('0');

	peri_suspend();
	putchar('1');

	power_down_domains();
	putchar('2');

	gating_clks();
	putchar('3');

	power_down_plls();
	putchar('4');

	pvtm_32khz_config();
	pmu_suspend();
	putchar('5');

	suspend_timers();
	putchar('6');

	plls_32khz_config();
	putchar('7');

	if (is_gpio_wakeup()) {
		skip_suspend = 1;
		INFO("gpio0 wakeup interrupt\n");
	} else {
		skip_suspend = 0;
	}

	config_info();

	return 0;
}

static int sys_pwr_domain_resume(void)
{
	putchar('7');

	restore_plls_24mhz();
	putchar('6');

	restore_timers();
	putchar('5');

	restore_pmu();
	restore_pvtm_32khz();
	putchar('4');

	restore_plls();
	putchar('3');

	restore_clks();
	putchar('2');

	restore_domains();
	putchar('1');

	restore_peri();
	putchar('0');

	finish();

	return 0;
}

static struct rockchip_pm_ops_cb pm_ops = {
	.cores_pwr_dm_on = cores_pwr_domain_on,
	.cores_pwr_dm_off = cores_pwr_domain_off,
	.cores_pwr_dm_on_finish = cores_pwr_domain_on_finish,
	.cores_pwr_dm_suspend = cores_pwr_domain_suspend,
	.cores_pwr_dm_resume = cores_pwr_domain_resume,
	.sys_pwr_dm_suspend = sys_pwr_domain_suspend,
	.sys_pwr_dm_resume = sys_pwr_domain_resume,
	.sys_gbl_soft_reset = soc_sys_global_soft_reset,
};

void bl31_plat_runtime_setup(void)
{
	/* do nothing */
}

void plat_rockchip_pmu_init(void)
{
	uint32_t cpu;

	plat_setup_rockchip_pm_ops(&pm_ops);

	for (cpu = 0; cpu < PLATFORM_CORE_COUNT; cpu++)
		cpuson_flags[cpu] = 0;

	cpu_warm_boot_addr = (uint64_t)platform_cpu_warmboot;
	psram_sleep_cfg->ddr_func = 0x00;
	psram_sleep_cfg->ddr_data = 0x00;
	psram_sleep_cfg->ddr_flag = 0x00;
	psram_sleep_cfg->boot_mpidr = read_mpidr_el1() & 0xffff;

	/* boot from ddr  */
	sgrf_write32((cpu_warm_boot_addr >> CPU_BOOT_ADDR_ALIGN) |
		     CPU_BOOT_ADDR_WMASK,
		     SGRF_SOC_CON(1));

	nonboot_cpus_off();

	INFO("%s: pd status 0x%x\n", __func__, pmu_read32(PMU_PWRDN_ST));
}
