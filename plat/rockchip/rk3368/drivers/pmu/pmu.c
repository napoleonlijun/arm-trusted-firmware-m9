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
#include <assert.h>
#include <bl31.h>
#include <console.h>
#include <debug.h>
#include <delay_timer.h>
#include <ddr_rk3368.h>
#include <errno.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <plat_private.h>
#include <pmu.h>
#include <pmu_com.h>
#include <pmu_sram.h>
#include <rk3368_def.h>
#include <rockchip_exceptions.h>
#include <rockchip_sip_svc.h>
#include <soc.h>
#include <uart.h>

struct rksoc_cpu_info_s {
	uint32_t slp_cfg;
	uint32_t suspend_dbg_en;
	uint32_t suspend_wkup_cfg;
};

#if USE_COHERENT_MEM
struct rksoc_cpu_info_s rk_cpuinfo
	__attribute__ ((section("tzfw_coherent_mem")));
#else
struct rksoc_cpu_info_s rk_cpuinfo;
#endif

struct rk_pmu_deepsleep_data_s {
	uint32_t sleep_cnt;
	uint32_t pwrdn_st;
	uint32_t pmu_sft_con;
	uint32_t pmugrf_soc_con0;
	uint32_t pmugrf_gpio0a_iomux;
	uint32_t clk_gate2;
	uint32_t clk_gate7;
	uint32_t clk_gate13;
	uint32_t plls_con[END_PLL_ID][4];
	uint32_t clk_sel[CRU_CLKSEL_CON_CNT];
	uint32_t clk_gate[CRU_CLKGATES_CON_CNT];
	uint32_t pgrf_pvtm_con[2];
	uint32_t grf_soc_con[18];
	uint32_t mcu_data[MCUOS_SIZE / 4];
	uint32_t imem_data[RK_IMEM_DDRCODE_LENGTH / 4];
	/* service peri */
	uint32_t peri_qos[CPU_AXI_QOS_NUM_REGS];
	/* service vio */
	uint32_t iep_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t isp_r0_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t isp_r1_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t isp_w0_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t isp_w1_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t vip_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t vop_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t rga_r_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t rga_w_qos[CPU_AXI_QOS_NUM_REGS];
	/* service video */
	uint32_t hevc_r_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t vpu_r_qos[CPU_AXI_QOS_NUM_REGS];
	uint32_t vpu_w_qos[CPU_AXI_QOS_NUM_REGS];
	/* service gpu */
	uint32_t gpu_qos[CPU_AXI_QOS_NUM_REGS];
};

#if USE_COHERENT_MEM && RKSLEEP_DATA_COHERENT
struct rk_pmu_deepsleep_data_s rksleep_data
	__attribute__ ((section("tzfw_coherent_mem")));
#else
struct rk_pmu_deepsleep_data_s rksleep_data;
#endif

struct rk_peri_deepsleep_data_s {
	uint32_t iomux[4][4];
	uint32_t grf_soc_con[18];
	uint32_t timer[RK_NTIMES][RK_NTIME_CHS][3];
	uint32_t stimer[RK_STIME_CHS][3];
	uint32_t sgrf_soc_con[10];
};

#if USE_COHERENT_MEM
struct rk_peri_deepsleep_data_s rksleep_peridata
	__attribute__ ((section("tzfw_coherent_mem")));
#else
struct rk_peri_deepsleep_data_s rksleep_peridata;
#endif

static const int8_t pmu_idle_req_map[] = {
	[bus_ide_req_clst_l] = 0,
	[bus_ide_req_clst_b] = 1,
	[bus_ide_req_gpu] = 2,
	[bus_ide_req_core] = 3,
	[bus_ide_req_bus] = 4,
	[bus_ide_req_dma] = 5,
	[bus_ide_req_peri] = 6,
	[bus_ide_req_video] = 7,
	[bus_ide_req_vio] = 8,
	[bus_ide_req_res0] = 9,
	[bus_ide_req_cxcs] = 10,
	[bus_ide_req_alive] = 11,
	[bus_ide_req_pmu] = 12,
	[bus_ide_req_msch] = 13,
	[bus_ide_req_cci] = 14,
	[bus_ide_req_cci400] = 15,
};

static const int8_t pmu_pd_map[] = {
	[PD_CPUL0] = 0,
	[PD_CPUL1] = 1,
	[PD_CPUL2] = 2,
	[PD_CPUL3] = 3,
	[PD_SCUL]  = 4,
	[PD_CPUB0] = 5,
	[PD_CPUB1] = 6,
	[PD_CPUB2] = 7,
	[PD_CPUB3] = 8,
	[PD_SCUB] = 9,
	[PD_PERI] = 13,
	[PD_VIDEO] = 14,
	[PD_VIO] = 15,
	[PD_GPU0] = 16,
	[PD_GPU1] = 17,
};

static const int8_t pmu_st_map[] = {
	[PD_CPUL0] = 0,
	[PD_CPUL1] = 1,
	[PD_CPUL2] = 2,
	[PD_CPUL3] = 3,
	[PD_SCUL]  = 4,
	[PD_CPUB0] = 5,
	[PD_CPUB1] = 6,
	[PD_CPUB2] = 7,
	[PD_CPUB3] = 8,
	[PD_SCUB] = 9,
	[PD_PERI] = 12,
	[PD_VIDEO] = 13,
	[PD_VIO] = 14,
	[PD_GPU0] = 15,
	[PD_GPU1] = 16,
};

DEFINE_BAKERY_LOCK(rockchip_pd_lock);

static struct psram_data_t *psram_sleep_cfg =
	(struct psram_data_t *)PSRAM_DT_BASE;

static uint32_t cpu_warm_boot_addr;

#include <pmu_prt_dbg.c>

void rk3368_flash_l2_b(void)
{
	uint32_t wait_cnt = 0;

	regs_updata_bit_set(PMU_BASE + PMU_SFT_CON, pmu_sft_l2flsh_clst_b);
	dsb();

	while (!(mmio_read_32(PMU_BASE + PMU_CORE_PWR_ST)
		& BIT(clst_b_l2_flsh_done))) {
		wait_cnt++;
		if (!(wait_cnt % MAX_WAIT_COUNT))
			WARN("%s:reg %x, wait\n", __func__,
			     mmio_read_32(PMU_BASE + PMU_CORE_PWR_ST));
	}

	regs_updata_bit_clr(PMU_BASE + PMU_SFT_CON, pmu_sft_l2flsh_clst_b);
}

static inline int rk3368_pmu_bus_idle(uint32_t req, uint32_t idle)
{
	uint32_t mask = BIT(req);
	uint32_t idle_mask = 0;
	uint32_t idle_target = 0;
	uint32_t val;
	uint32_t wait_cnt = 0;

	switch (req) {
	case bus_ide_req_clst_l:
		idle_mask = BIT(pmu_idle_ack_cluster_l);
		idle_target = (idle << pmu_idle_ack_cluster_l);
		break;

	case bus_ide_req_clst_b:
		idle_mask = BIT(pmu_idle_ack_cluster_b);
		idle_target = (idle << pmu_idle_ack_cluster_b);
		break;

	case bus_ide_req_cxcs:
		idle_mask = BIT(pmu_idle_ack_cxcs);
		idle_target = ((!idle) << pmu_idle_ack_cxcs);
		break;

	case bus_ide_req_cci400:
		idle_mask = BIT(pmu_idle_ack_cci400);
		idle_target = ((!idle) << pmu_idle_ack_cci400);
		break;

	case bus_ide_req_gpu:
		idle_mask = BIT(pmu_idle_ack_gpu) | BIT(pmu_idle_gpu);
		idle_target = (idle << pmu_idle_ack_gpu) |
			      (idle << pmu_idle_gpu);
		break;

	case bus_ide_req_core:
		idle_mask = BIT(pmu_idle_ack_core) | BIT(pmu_idle_core);
		idle_target = (idle << pmu_idle_ack_core) |
			      (idle << pmu_idle_core);
		break;

	case bus_ide_req_bus:
		idle_mask = BIT(pmu_idle_ack_bus) | BIT(pmu_idle_bus);
		idle_target = (idle << pmu_idle_ack_bus) |
			      (idle << pmu_idle_bus);
		break;
	case bus_ide_req_dma:
		idle_mask = BIT(pmu_idle_ack_dma) | BIT(pmu_idle_dma);
		idle_target = (idle << pmu_idle_ack_dma) |
			      (idle << pmu_idle_dma);
		break;

	case bus_ide_req_peri:
		idle_mask = BIT(pmu_idle_ack_peri) | BIT(pmu_idle_peri);
		idle_target = (idle << pmu_idle_ack_peri) |
			      (idle << pmu_idle_peri);
		break;

	case bus_ide_req_video:
		idle_mask = BIT(pmu_idle_ack_video) | BIT(pmu_idle_video);
		idle_target = (idle << pmu_idle_ack_video) |
			      (idle << pmu_idle_video);
		break;

	case bus_ide_req_vio:
		idle_mask = BIT(pmu_idle_ack_vio) | BIT(pmu_idle_vio);
		idle_target = (pmu_idle_ack_vio) |
			      (idle << pmu_idle_vio);
		break;

	case bus_ide_req_alive:
		idle_mask = BIT(pmu_idle_ack_alive) | BIT(pmu_idle_alive);
		idle_target = (idle << pmu_idle_ack_alive) |
			      (idle << pmu_idle_alive);
		break;

	case bus_ide_req_pmu:
		idle_mask = BIT(pmu_idle_ack_pmu) | BIT(pmu_idle_pmu);
		idle_target = (idle << pmu_idle_ack_pmu) |
			      (idle << pmu_idle_pmu);
		break;

	case bus_ide_req_msch:
		idle_mask = BIT(pmu_idle_ack_msch) | BIT(pmu_idle_msch);
		idle_target = (idle << pmu_idle_ack_msch) |
			      (idle << pmu_idle_msch);
		break;

	case bus_ide_req_cci:
		idle_mask = BIT(pmu_idle_ack_cci) | BIT(pmu_idle_cci);
		idle_target = (idle << pmu_idle_ack_cci) |
			      (idle << pmu_idle_cci);
		break;

	default:
		ERROR("%s: Unsupported the idle request\n", __func__);
		break;
	}

	val = mmio_read_32(PMU_BASE + PMU_BUS_IDE_REQ);
	if (idle)
		val |=	mask;
	else
		val &= ~mask;

	mmio_write_32(PMU_BASE + PMU_BUS_IDE_REQ, val);

	while ((mmio_read_32(PMU_BASE +
	       PMU_BUS_IDE_ST) & idle_mask) != idle_target) {
		wait_cnt++;
		if (!(wait_cnt % MAX_WAIT_COUNT))
			WARN("%s:st=%x(%x)\n", __func__,
			     mmio_read_32(PMU_BASE + PMU_BUS_IDE_ST),
			     idle_mask);
	}

	return 0;
}

static inline uint32_t rk3368_pmu_power_domain_st(enum pmu_pdid pd)
{
	/* 1'b0: power on, 1'b1: power off */
	return (mmio_read_32(PMU_BASE + PMU_PWRDN_ST) & BIT(pmu_st_map[pd])) ?
		pmu_power_off : pmu_power_on;
}

static void rk3368_do_pmu_set_power_domain(enum pmu_pdid pd,
					   pmu_power_state_t on)
{
	int8_t pd_id = pmu_pd_map[pd];
	uint32_t val = mmio_read_32(PMU_BASE + PMU_PWRDN_CON);

	if (on)
		val &= ~BIT(pd_id);
	else
		val |=  BIT(pd_id);

	mmio_write_32(PMU_BASE + PMU_PWRDN_CON, val);
	dsb();

	while (rk3368_pmu_power_domain_st(pd_id) != on)
		;
}

static int rk3368_pmu_set_power_domain(enum pmu_pdid pd,
				       pmu_power_state_t on)
{
	if (rk3368_pmu_power_domain_st(pd) == on)
		goto out;

	if (!on) {
		if (pd == PD_VIO) {
			SAVE_QOS(rksleep_data.iep_qos, VIO0_IEP);
			SAVE_QOS(rksleep_data.isp_r0_qos, VIO0_ISP_R0);
			SAVE_QOS(rksleep_data.isp_r1_qos, VIO0_ISP_R1);
			SAVE_QOS(rksleep_data.isp_w0_qos, VIO0_ISP_W0);
			SAVE_QOS(rksleep_data.isp_w1_qos, VIO0_ISP_W1);
			SAVE_QOS(rksleep_data.vip_qos, VIO_VIP);
			SAVE_QOS(rksleep_data.vop_qos, VIO1_VOP);
			SAVE_QOS(rksleep_data.rga_r_qos, VIO1_RGA_R);
			SAVE_QOS(rksleep_data.rga_w_qos, VIO1_RGA_W);
			rk3368_pmu_bus_idle(bus_ide_req_vio, 1);
		} else if (pd == PD_VIDEO) {
			SAVE_QOS(rksleep_data.hevc_r_qos, HEVC_R);
			SAVE_QOS(rksleep_data.vpu_r_qos, VPU_R);
			SAVE_QOS(rksleep_data.vpu_w_qos, VPU_W);
			rk3368_pmu_bus_idle(bus_ide_req_video, 1);
		} else if (pd == PD_GPU0) {
			/* do nothing */
		} else if (pd == PD_GPU1) {
			SAVE_QOS(rksleep_data.gpu_qos, GPU);
			rk3368_pmu_bus_idle(bus_ide_req_gpu, 1);
		} else if (pd == PD_PERI) {
			SAVE_QOS(rksleep_data.peri_qos, PERI);
			rk3368_pmu_bus_idle(bus_ide_req_peri, 1);
		}
	}

	rk3368_do_pmu_set_power_domain(pd, on);

	if (on) {
		if (pd == PD_VIO) {
			rk3368_pmu_bus_idle(bus_ide_req_vio, 0);
			RESTORE_QOS(rksleep_data.iep_qos, VIO0_IEP);
			RESTORE_QOS(rksleep_data.isp_r0_qos, VIO0_ISP_R0);
			RESTORE_QOS(rksleep_data.isp_r1_qos, VIO0_ISP_R1);
			RESTORE_QOS(rksleep_data.isp_w0_qos, VIO0_ISP_W0);
			RESTORE_QOS(rksleep_data.isp_w1_qos, VIO0_ISP_W1);
			RESTORE_QOS(rksleep_data.vip_qos, VIO_VIP);
			RESTORE_QOS(rksleep_data.vop_qos, VIO1_VOP);
			RESTORE_QOS(rksleep_data.rga_r_qos, VIO1_RGA_R);
			RESTORE_QOS(rksleep_data.rga_w_qos, VIO1_RGA_W);
		} else if (pd == PD_VIDEO) {
			rk3368_pmu_bus_idle(bus_ide_req_video, 0);
			RESTORE_QOS(rksleep_data.hevc_r_qos, HEVC_R);
			RESTORE_QOS(rksleep_data.vpu_r_qos, VPU_R);
			RESTORE_QOS(rksleep_data.vpu_w_qos, VPU_W);
		} else if (pd == PD_GPU0) {
			/* do nothing */
		} else if (pd == PD_GPU1) {
			rk3368_pmu_bus_idle(bus_ide_req_gpu, 0);
			RESTORE_QOS(rksleep_data.gpu_qos, GPU);
		} else if (pd == PD_PERI) {
			rk3368_pmu_bus_idle(bus_ide_req_peri, 0);
			RESTORE_QOS(rksleep_data.peri_qos, PERI);
		}
	}

out:
	return 0;
}

static inline enum pmu_pdid cluster_cpu_to_pmu_cpu_scu_id(
				uint32_t cluster_id,
				uint32_t cpu_id,
				uint32_t afflvl)
{
	enum pmu_pdid pd_id;

	if (afflvl == MPIDR_AFFLVL0)
		pd_id = PD_CPUL0 + cluster_id * 5 + cpu_id;
	else
		pd_id = PD_SCUL + cluster_id * 5;

	return pd_id;
}

#define is_scu_dm(pd_id)	(BIT(pd_id) & PM_PWRDM_SCUS_MSK)

static int scus_power_domain(uint32_t cluster_id,
			     uint32_t cpu_id,
			     uint32_t afflvl,
			     pmu_power_state_t on)
{
	enum pmu_pdid pd_id =
		cluster_cpu_to_pmu_cpu_scu_id(cluster_id, cpu_id, afflvl);

	if (!is_scu_dm(pd_id)) {
		ERROR("%s:not scus pd(%d)\n", __func__, pd_id);
		while (1)
			;
	}

	if (rk3368_pmu_power_domain_st(pd_id) != on)
		rk3368_do_pmu_set_power_domain(pd_id, on);

	return 0;
}

static int cpus_id_power_domain(uint32_t cluster,
				uint32_t cpu,
				uint32_t pd_state,
				uint32_t wfie_msk)
{
	uint32_t pd;
	uint32_t cpu_id;

	if (cluster)
		pd = PD_CPUB0 + cpu;
	else
		pd = PD_CPUL0 + cpu;

	if (pmu_power_domain_st(pd) == pd_state)
		return 0;

	if (pd_state == pmu_pd_off) {
		cpu_id = cluster * PLATFORM_CLUSTER0_CORE_COUNT + cpu;
		if (check_cpu_wfie(cpu_id, wfie_msk))
			return -EINVAL;
	}

	return pmu_power_domain_ctr(pd, pd_state);
}

static void nonboot_cpus_off(void)
{
	uint32_t boot_cpu, boot_cluster, cpu;

	boot_cpu = MPIDR_AFFLVL0_VAL(read_mpidr_el1());
	boot_cluster = MPIDR_AFFLVL1_VAL(read_mpidr_el1());

	/* turn off noboot cpus */
	for (cpu = 0; cpu < PLATFORM_CLUSTER0_CORE_COUNT; cpu++) {
		if (!boot_cluster && (cpu == boot_cpu))
			continue;
		cpus_id_power_domain(0, cpu, pmu_pd_off, CKECK_WFEI_MSK);
	}

	for (cpu = 0; cpu < PLATFORM_CLUSTER1_CORE_COUNT; cpu++) {
		if (boot_cluster && (cpu == boot_cpu))
			continue;
		cpus_id_power_domain(1, cpu, pmu_pd_off, CKECK_WFEI_MSK);
	}
}

void rk3368_cpu_pwr_dm_off(uint32_t target_idx)
{
	uint32_t cluster, cpu;

	cluster = target_idx >= PLATFORM_CLUSTER0_CORE_COUNT ? 1 : 0;
	cpu = cluster ? target_idx - PLATFORM_CLUSTER0_CORE_COUNT : target_idx;

	cpus_id_power_domain(cluster, cpu, pmu_pd_off, CKECK_WFEI_MSK);
}

int rockchip_soc_cores_pwr_dm_on(unsigned long mpidr,
				 uint64_t entrypoint)
{
	uint32_t cpu, cluster;
	uint32_t cpuon_id;

	cpu = MPIDR_AFFLVL0_VAL(mpidr);
	cluster = MPIDR_AFFLVL1_VAL(mpidr);

	/* Make sure the cpu is off, Before power up the cpu! */
	cpus_id_power_domain(cluster, cpu, pmu_pd_off, CKECK_WFEI_MSK);

	cpuon_id = (cluster * PLATFORM_CLUSTER0_CORE_COUNT) + cpu;
	assert(cpuon_id < PLATFORM_CORE_COUNT);
	assert(cpuson_flags[cpuon_id] == 0);
	cpuson_flags[cpuon_id] = PMU_CPU_HOTPLUG;
	cpuson_entry_point[cpuon_id] = entrypoint;

	/* Switch boot addr to pmusram */
	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(1 + cluster),
		      (cpu_warm_boot_addr >> CPU_BOOT_ADDR_ALIGN) |
		      CPU_BOOT_ADDR_WMASK);
	dsb();

	cpus_id_power_domain(cluster, cpu, pmu_pd_on, CKECK_WFEI_MSK);

	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(1 + cluster),
		      (COLD_BOOT_BASE >> CPU_BOOT_ADDR_ALIGN) |
		      CPU_BOOT_ADDR_WMASK);

	return 0;
}

int rockchip_soc_cores_pwr_dm_on_finish(void)
{
	return 0;
}

#define PMU_IO_WKUP_P 0x1
#define PMU_IO_WKUP_N 0x2

void pmu_gpio_wakeup(int8_t port,
		     int8_t bank,
		     int8_t b_gpio,
		     int8_t typ)
{
	bank -= 0xa;

	if (port)
		return;

	if (typ & PMU_IO_WKUP_P)
		regs_updata_bit_set(PMU_BASE + PMU_WKUP_CFG0,
				    bank * 8 + b_gpio);

	if (typ & PMU_IO_WKUP_N)
		regs_updata_bit_set(PMU_BASE + PMU_WKUP_CFG1,
				    bank * 8 + b_gpio);

	if (!typ) {
		regs_updata_bit_clr(PMU_BASE + PMU_WKUP_CFG0,
				    bank * 8 + b_gpio);
		regs_updata_bit_clr(PMU_BASE + PMU_WKUP_CFG1,
				    bank * 8 + b_gpio);
	}
}

static void pmu_sleep_mode_config(uint32_t status)
{
	uint32_t pwrmd_core, pwrmd_com;
	uint32_t temp_val;

	mmio_write_32(PMU_BASE + PMU_WKUP_CFG2,
		      rk_cpuinfo.suspend_wkup_cfg);

	/* common setting */
	pwrmd_core = BIT(pmu_mdcr_cpu0_pd) |
		     BIT(pmu_mdcr_scu_l_pd) |
		     BIT(pmu_mdcr_l2_flush) |
		     BIT(pmu_mdcr_l2_idle) |
		     BIT(pmu_mdcr_clr_clst_l) |
		     BIT(pmu_mdcr_clr_core) |
		     BIT(pmu_mdcr_clr_cci);

	pwrmd_com = BIT(pmu_mode_en) |
		    BIT(pmu_mode_sref_enter) |
		    BIT(pmu_mode_ddrc_gt);

	if (status & SLP_ARMPD) {
		/* arm scu+cpu0 power domain power dn */
		VERBOSE("arm pd\n");

		/* sleep pin invalid*/
		pin_set_fun(0x0, 0xa, 0x0, IOMUX_GPIO);

		/* set default wakeup int : core int */
		regs_updata_bits(PMU_BASE + PMU_WKUP_CFG2, 0x3, 0x3, 0);
		regs_updata_bits(PMU_BASE + PMU_WKUP_CFG2, 0x0, 0x1, 2);

	} else if (status & SLP_ARMOFF) {
		/* arm off */
		VERBOSE("arm off\n");

		pin_set_fun(0x0, 0xa, 0x0, IOMUX_PMIC_SLP);

		/* set default wakeup int : core int */
		regs_updata_bits(PMU_BASE + PMU_WKUP_CFG2, 0x3, 0x3, 0);
		regs_updata_bits(PMU_BASE + PMU_WKUP_CFG2, 0x0, 0x1, 2);

		if (status & SLP_PMU_PLLS_PWRDN) {
			pwrmd_com |= BIT(pmu_mode_ddrio_ret) |
				     BIT(pmu_mode_clr_msch) |
				     BIT(pmu_mode_clr_bus) |
				     BIT(pmu_mode_clr_dma) |
				     BIT(pmu_mode_clr_pmu) |
				     BIT(pmu_mode_clr_alive);
		}

		pwrmd_core |= BIT(pmu_mdcr_core_pd);
		pwrmd_com |= BIT(pmu_mode_pwr_off);
	} else if (status & SLP_ARMOFF_LOGPD) {
		/* arm off + logic pd */
		VERBOSE("arm off + logic pd\n");

		pin_set_fun(0x0, 0xa, 0x0, IOMUX_PMIC_SLP);

		/*  pwr key*/
		pmu_gpio_wakeup(0x0, 0xa, 0x2, 0x3);
		/*  pmic rtc*/
		pmu_gpio_wakeup(0x0, 0xa, 0x1, 0x3);

		/* PMU_RST_HOLD */
		mmio_write_32(PMU_GRF_BASE + PMUGRF_SOC_CON0,
			      BITS_WITH_WMASK(1, 1, pgrf_soc_pmu_rst_hd));

		pwrmd_core |= BIT(pmu_mdcr_core_pd);

		pwrmd_com |= BIT(pmu_mode_pwr_off) |
			     BIT(pmu_mode_ddrio_ret) |
			     BIT(pmu_mode_bus_pd) |
			     BIT(pmu_mode_clr_bus) |
			     BIT(pmu_mode_clr_dma) |
			     BIT(pmu_mode_clr_pmu) |
			     BIT(pmu_mode_clr_alive) |
			     BIT(pmu_mode_clr_msch);

		if (!(status & SLP_SFT_PD_PERI))
			pwrmd_com |= BIT(pmu_mode_clr_peri);
	} else if (status & SLP_ARMOFF_LOGOFF) {
		/* arm off + logic off */
		VERBOSE("arm off + logic off\n");

		pin_set_fun(0x0, 0xa, 0x0, IOMUX_PMIC_SLP);

		/*  pwr key*/
		pmu_gpio_wakeup(0x0, 0xa, 0x2, 0x3);
		/*  pmic rtc*/
		pmu_gpio_wakeup(0x0, 0xa, 0x1, 0x3);

		/* PMU_RST_HOLD */
		mmio_write_32(PMU_GRF_BASE + PMUGRF_SOC_CON0,
			      BITS_WITH_WMASK(1, 1, pgrf_soc_pmu_rst_hd));

		pwrmd_core |= BIT(pmu_mdcr_core_pd);

		pwrmd_com |= BIT(pmu_mode_pwr_off) |
			     BIT(pmu_mode_ddrio_ret) |
			     BIT(pmu_mode_bus_pd) |
			     BIT(pmu_mode_clr_bus) |
			     BIT(pmu_mode_clr_dma) |
			     BIT(pmu_mode_clr_peri) |
			     BIT(pmu_mode_clr_pmu) |
			     BIT(pmu_mode_clr_alive) |
			     BIT(pmu_mode_clr_msch) |
			     BIT(pmu_mode_wkup_rst) |
			     BIT(pmu_mode_input_clamp);
	} else {
		pwrmd_com = 0;
		pwrmd_core = 0;
	}

	if (status & SLP_PMU_PLLS_PWRDN)
		pwrmd_com |= BIT(pmu_mode_pll_pd);

	if (status & SLP_PMU_DIS_OSC)
		pwrmd_com |= BIT(pmu_mode_osc_dis);

	if (status & SLP_PMU_PMUALIVE_32K) {
		pwrmd_com |= BIT(pmu_mode_pmu_use_if) |
			     BIT(pmu_mode_pmu_alive_use_if);
	}
	/*
	 * ddr io ret ctrl
	 * first switch io ctrl by soc
	 * (RK3368_PMU_SFT_CON:pmu_sft_ddrio_ret_cfg:0)
	 * then  select io ctrl by extio(pgrf_soc_ddrphy_bufen_io)
	 * or entio(pgrf_soc_ddrphy_bufen_core)
	 * if pd bus pd, we must select extio ctrl
	 * 0 enable, 1 disable
	 */
	if (pwrmd_com & BIT(pmu_mode_ddrio_ret)) {
		psram_sleep_cfg->ddr_flag = 1;
		regs_updata_bit_clr(PMU_BASE + PMU_SFT_CON,
				    pmu_sft_ddrio_ret_cfg);
		dsb();
		dsb();
		if (pwrmd_com & BIT(pmu_mode_bus_pd))
			temp_val = BITS_WITH_WMASK(0x1,
						   0x3,
						   pgrf_soc_ddrphy_bufen_core);
		else
			temp_val = BITS_WITH_WMASK(0x2,
						   0x3,
						   pgrf_soc_ddrphy_bufen_core);

		mmio_write_32(PMU_GRF_BASE + PMUGRF_SOC_CON0, temp_val);
	}

	/*
	 * set sys resume delay, for pll resume is 24M,
	 * RK3368_PMU_PLLLOCK_CNT: ic: 24*500
	 * RK3368_PMU_PLLRST_CNT: ic: 24*5
	 */
	mmio_write_32(PMU_BASE + PMU_PLLLOCK_CNT, 24 * 500 * 4);
	mmio_write_32(PMU_BASE + PMU_PLLRST_CNT, 24 * 5 * 20);

	if (pwrmd_com & BIT(pmu_mode_pmu_use_if)) {
		/* 32k */
		mmio_write_32(PMU_BASE + PMU_WKUPRST_CNT, 32 * 10);
		mmio_write_32(PMU_BASE + PMU_STABLE_CNT, 32 * 50);
		mmio_write_32(PMU_BASE + PMU_OSC_CNT, 32 * 50);
		mmio_write_32(PMU_BASE + PMU_DDRIO_PWR_CNT, 32 * 2);
	} else {
		/* 24M */
		mmio_write_32(PMU_BASE + PMU_STABLE_CNT, 24 * 1000 * 2);
	}

	mmio_write_32(PMU_BASE + PMU_PWRMD_CORE, pwrmd_core);
	mmio_write_32(PMU_BASE + PMU_PWRMD_COM, pwrmd_com);
	dsb();
}

static void ddr_suspend_save(void)
{
	mmio_write_32(CRU_BASE + CRU_CLKGATES_CON(12), 0x60000000);
	ddr_reg_save(1, psram_sleep_cfg->ddr_data);
}

static void pll_sleep_32k_pvtm_config(uint32_t cfg)
{
	if (!(cfg & SLP_PMU_PMUALIVE_32K) && !(cfg & SLP_SFT_PLLS_DEEP))
		return;

	/*
	 *bit[6]: sel 32k or pvtm
	 *bit[7]: sel pwm2 or 32k
	 */
	if (cfg & SLP_SFT_32K_EXT) {
		mmio_write_32(PMU_GRF_BASE + PMUGRF_SOC_CON0,
			      BITS_WITH_WMASK(0, 0x3, pgrf_soc_32k_src));
	} else {
		/* enable pmu pvtm: gate7[3] */
		rksleep_data.clk_gate7 =
			mmio_read_32(CRU_BASE + CRU_CLKGATES_CON(7));
		mmio_write_32(CRU_BASE + CRU_CLKGATES_CON(7), 0x00080000);

		mmio_write_32(PMU_GRF_BASE + PMUGRF_SOC_CON0,
			      BITS_WITH_WMASK(1, 0x1, pgrf_soc_32k_src));

		rksleep_data.pgrf_pvtm_con[0] =
			mmio_read_32(PMU_GRF_BASE + PMUGRF_PVTM_CON0);
		rksleep_data.pgrf_pvtm_con[1] =
			mmio_read_32(PMU_GRF_BASE + PMUGRF_PVTM_CON1);

		mmio_write_32(PMU_GRF_BASE + PMUGRF_PVTM_CON0,
			      BITS_WITH_WMASK(1, 0x1, pgrf_pvtm_en));
		dsb();
		mmio_write_32(PMU_GRF_BASE + PMUGRF_PVTM_CON1, 0x200);
		dsb();

		mmio_write_32(PMU_GRF_BASE + PMUGRF_PVTM_CON0,
			      BITS_WITH_WMASK(1, 0x1, pgrf_pvtm_st));

		while (!(mmio_read_32(PMU_GRF_BASE + PMUGRF_PVTM_ST0) & 0x1))
			;

		VERBOSE("%s:pvtm:con0=0x%x, con1=0x%x, st0=0x%x, st1=0x%x\n",
			__func__,
			mmio_read_32(PMU_GRF_BASE + PMUGRF_PVTM_CON0),
			mmio_read_32(PMU_GRF_BASE + PMUGRF_PVTM_CON1),
			mmio_read_32(PMU_GRF_BASE + PMUGRF_PVTM_ST0),
			mmio_read_32(PMU_GRF_BASE + PMUGRF_PVTM_ST1));
	}
}

static void pll_sleep_32k_pvtm_config_restore(uint32_t cfg)
{
	uint32_t gate7;

	if (!(cfg & SLP_PMU_PMUALIVE_32K) && !(cfg & SLP_SFT_PLLS_DEEP))
		return;

	/* restore pmu pvtm */
	if (!(cfg & SLP_SFT_32K_EXT)) {
		gate7 = rksleep_data.clk_gate7 & PMU_PVTM_GATE_EN;
		mmio_write_32(CRU_BASE + CRU_CLKGATES_CON(7),
			      0x00080000 | gate7);
		mmio_write_32(PMU_GRF_BASE + PMUGRF_PVTM_CON0,
			      rksleep_data.pgrf_pvtm_con[0] | 0xffff0000);
		mmio_write_32(PMU_GRF_BASE + PMUGRF_PVTM_CON1,
			      rksleep_data.pgrf_pvtm_con[1]);
	}
}

static void ntimers_suspend(uint32_t cfg)
{
	uint32_t tm, ch, base_adr;

	for (tm = 0; tm < RK_NTIMES; tm++)
		for (ch = 0; ch < RK_NTIME_CHS; ch++) {
			base_adr = RK_TIMER0_BASE + 0x10000 * tm + 0x20 * ch;
			rksleep_peridata.timer[tm][ch][0] =
				mmio_read_32(base_adr + TIMER_LOADE_COUNT0);
			rksleep_peridata.timer[tm][ch][1] =
				mmio_read_32(base_adr + TIMER_LOADE_COUNT1);
			rksleep_peridata.timer[tm][ch][2] =
				mmio_read_32(base_adr + TIMER_CONTROL_REG);
			mmio_write_32(base_adr + TIMER_CONTROL_REG, 0);
		}
}

static void ntimers_resume(uint32_t cfg)
{
	uint32_t tm, ch, base_adr;

	for (tm = 0; tm < RK_NTIMES; tm++)
		for (ch = 0; ch < RK_NTIME_CHS; ch++) {
			base_adr = RK_TIMER0_BASE + 0x10000 * tm + 0x20 * ch;
			mmio_write_32(base_adr + TIMER_LOADE_COUNT0,
				      rksleep_peridata.timer[tm][ch][0]);
			mmio_write_32(base_adr + TIMER_LOADE_COUNT1,
				      rksleep_peridata.timer[tm][ch][1]);
			mmio_write_32(base_adr + TIMER_CONTROL_REG,
				      rksleep_peridata.timer[tm][ch][2]);
		}
}

static void stimers_suspend(uint32_t cfg)
{
	uint32_t ch, base_adr;

	for (ch = 0; ch < RK_STIME_CHS; ch++) {
		base_adr = RK_STIME_BASE + 0x20 * ch;
		rksleep_peridata.stimer[ch][0] =
			mmio_read_32(base_adr + TIMER_LOADE_COUNT0);
		rksleep_peridata.stimer[ch][1] =
			mmio_read_32(base_adr + TIMER_LOADE_COUNT1);
		rksleep_peridata.stimer[ch][2] =
			mmio_read_32(base_adr + TIMER_CONTROL_REG);
		mmio_write_32(base_adr + TIMER_CONTROL_REG, 0);
	}

	rksleep_peridata.sgrf_soc_con[3] =
		mmio_read_32(SGRF_BASE + SGRF_SOC_CON(3));
	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(3),
		      BITS_WITH_WMASK(0x23, 0x23, 0));
}

static void stimers_resume(uint32_t cfg)
{
	uint32_t ch, base_adr;

	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(3),
		      rksleep_peridata.sgrf_soc_con[3] | BITS_WMSK(0x23, 0));

	for (ch = 0; ch < RK_STIME_CHS; ch++) {
		base_adr = RK_STIME_BASE + 0x20 * ch;
		mmio_write_32(base_adr + TIMER_LOADE_COUNT0,
			      rksleep_peridata.stimer[ch][0]);
		mmio_write_32(base_adr + TIMER_LOADE_COUNT1,
			      rksleep_peridata.stimer[ch][1]);
		mmio_write_32(base_adr + TIMER_CONTROL_REG,
			      rksleep_peridata.stimer[ch][2]);
	}
}

static void timers_suspend(uint32_t cfg)
{
	if (cfg & (SLP_PMU_DIS_OSC | SLP_ARMOFF_LOGOFF)) {
		ntimers_suspend(cfg);
		stimers_suspend(cfg);
	}
}

static void timers_resume(uint32_t cfg)
{
	if (cfg & (SLP_PMU_DIS_OSC | SLP_ARMOFF_LOGOFF)) {
		stimers_resume(cfg);
		ntimers_resume(cfg);
	}
}

static uint32_t pin_iomux[4][4] = {
	[1][0] = 0,
	[1][1] = 0,
	[1][2] = 0,
	[1][3] = 0,
	[2][0] = 0xa << 10, /* uart2 */
	[2][1] = 0,
	[2][2] = 0,
	[2][3] = 0,
	[3][0] = 0,
	[3][1] = 0,
	[3][2] = 0,
	[3][3] = 0xa << 10, /* uart3 */
};

#define SLP_PERI_PIN_CFG_BITS (SLP_SFT_PD_PERI | SLP_ARMOFF_LOGOFF)

void peri_pin_to_gpio(uint32_t cfg)
{
	uint32_t port, bank, addr_base;

	if (!(cfg & SLP_PERI_PIN_CFG_BITS))
		return;

	for (port = 1; port < 4; port++)
		for (bank = 0; bank < 4; bank++) {
			addr_base = GRF_BASE + (port - 1) * 16 + bank * 4;
			rksleep_peridata.iomux[port][bank] =
					mmio_read_32(addr_base);
			mmio_write_32(addr_base,
				      0xffff0000 | pin_iomux[port][bank]);
		}

	/* save pwm: RK PWM */
	rksleep_peridata.grf_soc_con[15] =
			mmio_read_32(GRF_BASE + GRF_SOC_CON(15));
}

void peri_pin_to_restore(uint32_t cfg)
{
	uint32_t port, bank, addr_base;

	if (!(cfg & SLP_PERI_PIN_CFG_BITS))
		return;

	for (port = 1; port < 4; port++)
		for (bank = 0; bank < 4; bank++) {
			addr_base = GRF_BASE + (port - 1) * 16 + bank * 4;
			mmio_write_32(addr_base,
				      0xffff0000 |
				      rksleep_peridata.iomux[port][bank]);
		}

	/* resume pwm: RK PWM */
	mmio_write_32(GRF_BASE + GRF_SOC_CON(15),
		      0xffff0000 | rksleep_peridata.grf_soc_con[15]);
}

static int rk3368_sys_set_power_domain(enum pmu_pdid pd, uint32_t on)
{
	uint32_t clks_save[CRU_CLKGATES_CON_CNT];
	uint32_t i, ret;

	for (i = 0; i < CRU_CLKGATES_CON_CNT; i++)
		clks_save[i] = mmio_read_32(CRU_BASE + CRU_CLKGATES_CON(i));

	for (i = 0; i < CRU_CLKGATES_CON_CNT; i++)
		mmio_write_32(CRU_BASE + CRU_CLKGATES_CON(i), 0xffff0000);

	if (((pd == PD_PERI) || (pd == PD_GPU0) || (pd == PD_GPU1) ||
	     (pd == PD_VIO) || (pd == PD_VIDEO)))
		ret = rk3368_pmu_set_power_domain(pd, on);

	for (i = 0; i < CRU_CLKGATES_CON_CNT; i++)
		mmio_write_32(CRU_BASE + CRU_CLKGATES_CON(i),
			      clks_save[i] | 0xffff0000);

	return ret;
}

static void rk_pm_soc_pd_suspend(uint32_t cfg)
{
	rksleep_data.pwrdn_st = mmio_read_32(PMU_BASE + PMU_PWRDN_ST);

	if (!(rksleep_data.pwrdn_st & BIT(pmu_st_map[PD_GPU1])))
		rk3368_sys_set_power_domain(PD_GPU1, pmu_power_off);

	if (!(rksleep_data.pwrdn_st & BIT(pmu_st_map[PD_GPU0])))
		rk3368_sys_set_power_domain(PD_GPU0, pmu_power_off);

	if (!(rksleep_data.pwrdn_st & BIT(pmu_st_map[PD_VIO])))
		rk3368_sys_set_power_domain(PD_VIO, pmu_power_off);

	if (!(rksleep_data.pwrdn_st & BIT(pmu_st_map[PD_VIDEO])))
		rk3368_sys_set_power_domain(PD_VIDEO, pmu_power_off);

	if (cfg & SLP_SFT_PD_PERI)
		if (!(rksleep_data.pwrdn_st & BIT(pmu_st_map[PD_PERI])))
			rk3368_sys_set_power_domain(PD_PERI, pmu_power_off);
}

static void rk_pm_soc_pd_resume(uint32_t cfg)
{
	if (cfg & SLP_SFT_PD_PERI)
		if (!(rksleep_data.pwrdn_st & BIT(pmu_st_map[PD_PERI])))
			rk3368_sys_set_power_domain(PD_PERI, pmu_power_on);

	if (!(rksleep_data.pwrdn_st & BIT(pmu_st_map[PD_VIDEO])))
		rk3368_sys_set_power_domain(PD_VIDEO, pmu_power_on);

	if (!(rksleep_data.pwrdn_st & BIT(pmu_st_map[PD_VIO])))
		rk3368_sys_set_power_domain(PD_VIO, pmu_power_on);

	if (!(rksleep_data.pwrdn_st & BIT(pmu_st_map[PD_GPU0])))
		rk3368_sys_set_power_domain(PD_GPU0, pmu_power_on);

	if (!(rksleep_data.pwrdn_st & BIT(pmu_st_map[PD_GPU1])))
		rk3368_sys_set_power_domain(PD_GPU1, pmu_power_on);
}

void mcu_suspend(uint32_t cfg)
{
	uint32_t i, loop = 0;

	while (!(mmio_read_32(PMU_BASE + PMU_CORE_PWR_ST) &
		BIT(mcu_sleeping))) {
		loop++;
		if (!(loop % MAX_WAIT_COUNT))
			WARN("%s waitting: mcu wfi\n", __func__);
	}

	if (!(cfg & (SLP_ARMOFF_LOGPD | SLP_ARMOFF_LOGOFF)))
		return;

	for (i = 0; i < 5; i++)
		rksleep_data.grf_soc_con[10 + i] =
			mmio_read_32(GRF_BASE + GRF_SOC_CON(10 + i));

	mmio_write_32(CRU_BASE + CRU_SOFTRSTS_CON(1),
		      BITS_WITH_WMASK(0x3,
				      CRU_RST1_MCU_BITS,
				      CRU_RST1_MCU_BITS_SHIFT));

	u32_align_cpy(rksleep_data.mcu_data,
		      (unsigned int *)MCUOS_BASE, MCUOS_SIZE / 4);
}

void mcu_resume(uint32_t cfg)
{
	uint32_t i;

	if (!(cfg & (SLP_ARMOFF_LOGPD | SLP_ARMOFF_LOGOFF)))
		return;

	u32_align_cpy((unsigned int *)MCUOS_BASE,
		      rksleep_data.mcu_data,
		      MCUOS_SIZE / 4);

	for (i = 0; i < 5; i++)
		mmio_write_32(GRF_BASE + GRF_SOC_CON(10 + i),
			      rksleep_data.grf_soc_con[10 + i] | 0xffff0000);

	mmio_write_32(CRU_BASE + CRU_SOFTRSTS_CON(1),
		      BITS_WITH_WMASK(0x0,
				      CRU_RST1_MCU_BITS,
				      CRU_RST1_MCU_BITS_SHIFT));
}

static void imem_code_save(uint32_t cfg)
{
	uint32_t pwrmd_com = mmio_read_32(PMU_BASE + PMU_PWRMD_COM);

	if (!(pwrmd_com & BIT(pmu_mode_bus_pd)))
		return;

	u32_align_cpy(rksleep_data.imem_data,
		      (unsigned int *)RK_IMEM_DDRCODE_BASE,
		      RK_IMEM_DDRCODE_LENGTH / 4);
}

static void imem_code_resume(uint32_t cfg)
{
	uint32_t pwrmd_com = mmio_read_32(PMU_BASE + PMU_PWRMD_COM);

	if (!(pwrmd_com & BIT(pmu_mode_bus_pd)))
		return;

	u32_align_cpy((unsigned int *)RK_IMEM_DDRCODE_BASE,
		      rksleep_data.imem_data,
		      RK_IMEM_DDRCODE_LENGTH / 4);
}

#define CLK_MSK_GATING(msk, con) \
	mmio_write_32(CRU_BASE + (con), ((msk) << 16) | 0xffff)
#define CLK_MSK_UNGATING(msk, con) \
	mmio_write_32(CRU_BASE + (con), ((~(msk)) << 16) | 0xffff)

static uint32_t clk_ungt_msk[] = {
	0xffff,
	0xffff,
	0xf020,
	0x000f,
	/* gate:4-7 */
	0x0,
	0x0,
	0xe000,
	0xffff,
	/* gate:8-11 */
	0x0,
	0xffff,
	0xffff,
	0xffff,
	/* gate:12-15 */
	0xffff,
	0xffdf,
	0xffff,
	0xffff,
	/* gate:16-19 */
	0x0,
	0x0,
	0xffff,
	0x0877,
	/* gate:20-23 */
	0x0f00,
	0x0,
	0x0f0f, /* stime 12:13 */
	0x003f,
	/* gate:24 */
	0x0 /* timex0-timex5 */
};

static uint32_t uarts_clk_en[] = {
	GATE_ID(2, 0), GATE_ID(2, 2), GATE_ID(2, 4),
	GATE_ID(2, 6), GATE_ID(2, 8),
};

static uint32_t uarts_pclk_en[] = {
	GATE_ID(19, 7), GATE_ID(19, 8), GATE_ID(13, 5),
	GATE_ID(19, 9), GATE_ID(19, 10),
};

void gtclks_suspend(void)
{
	int i;

	for (i = 0; i < CRU_CLKGATES_CON_CNT; i++) {
		rksleep_data.clk_gate[i] =
			mmio_read_32(CRU_BASE + CRU_CLKGATES_CON(i));
		mmio_write_32(CRU_BASE + CRU_CLKGATES_CON(i),
			      ~clk_ungt_msk[i] | 0xffff0000);
	}
	/* clk for m3 */
	mmio_write_32(CRU_BASE + CRU_CLKGATES_CON(1), 0x00080000);
	/* clk and pclk of UARTn(n=0, 1, 2...) */
	CRU_UNGATING_OPS(uarts_pclk_en[RK_UART_PORT]);
	CRU_UNGATING_OPS(uarts_clk_en[RK_UART_PORT]);
}

void gtclks_resume(void)
{
	int i;

	for (i = 0; i < CRU_CLKGATES_CON_CNT; i++)
		mmio_write_32(CRU_BASE + CRU_CLKGATES_CON(i),
			      rksleep_data.clk_gate[i] | 0xffff0000);
}

#define PLL_CON3_RES_WMSK		BITS_WMSK(0x7, 2)
#define PLL_LOCK_MAX_WAIT_CNT		600000U
static void pm_pll_wait_lock(uint32_t pll_idx)
{
	uint32_t delay = PLL_LOCK_MAX_WAIT_CNT;
	uint32_t reg_con3 = mmio_read_32(CRU_BASE + PLL_CONS((pll_idx), 3));

	if (reg_con3 && PLL_PWR_DN_MSK)
		return;

	dsb();
	dsb();
	dsb();
	dsb();
	dsb();
	dsb();
	while (delay > 0) {
		if (mmio_read_32(CRU_BASE + PLL_CONS(pll_idx, 1)) &
		      PLL_LOCK_MSK)
			break;
		delay--;
	}
	if (delay == 0)
		WARN("unlock-pll:%d\n", pll_idx);
}

static void plls_enter_deep(uint32_t cfg)
{
	if (cfg & SLP_SFT_PLLS_DEEP) {
		mmio_write_32(CRU_BASE + PLL_CONS((NPLL_ID), 3),
			      PLL_DEEP_BITS);
		mmio_write_32(CRU_BASE + PLL_CONS((CPLL_ID), 3),
			      PLL_DEEP_BITS);
		mmio_write_32(CRU_BASE + PLL_CONS((ABPLL_ID), 3),
			      PLL_DEEP_BITS);
		mmio_write_32(CRU_BASE + PLL_CONS((GPLL_ID), 3),
			      PLL_DEEP_BITS);
		mmio_write_32(CRU_BASE + PLL_CONS((ALPLL_ID), 3),
			      PLL_DEEP_BITS);
	}
}

static void plls_enter_slow(uint32_t cfg)
{
	if (cfg & SLP_SFT_PLLS_DEEP) {
		mmio_write_32(CRU_BASE + PLL_CONS((ALPLL_ID), 3),
			      PLL_SLOW_BITS);
		mmio_write_32(CRU_BASE + PLL_CONS((GPLL_ID), 3),
			      PLL_SLOW_BITS);
		mmio_write_32(CRU_BASE + PLL_CONS((CPLL_ID), 3),
			      PLL_SLOW_BITS);
		mmio_write_32(CRU_BASE + PLL_CONS((NPLL_ID), 3),
			      PLL_SLOW_BITS);
		mmio_write_32(CRU_BASE + PLL_CONS((ABPLL_ID), 3),
			      PLL_SLOW_BITS);
	}
}

static void plls_suspend(uint32_t pll_id, uint32_t cfg)
{
	rksleep_data.plls_con[pll_id][0] =
		mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 0));
	rksleep_data.plls_con[pll_id][1] =
		mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 1));
	rksleep_data.plls_con[pll_id][2] =
		mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 2));
	rksleep_data.plls_con[pll_id][3] =
		mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 3));

	mmio_write_32(CRU_BASE + PLL_CONS((pll_id), 3), PLL_SLOW_BITS);

	if (cfg & SLP_PMU_PLLS_PWRDN)
		mmio_write_32(CRU_BASE + PLL_CONS((pll_id), 3), PLL_BYPASS);
	else
		mmio_write_32(CRU_BASE + PLL_CONS((pll_id), 3), PLL_PWR_DN);
}

static void pm_plls_suspend(uint32_t cfg)
{
	uint32_t i;

	if (cfg & SLP_ARMOFF_LOGOFF) {
		for (i = 0; i < CRU_CLKSEL_CON_CNT; i++)
			rksleep_data.clk_sel[i] =
				mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(i));
	} else {
		rksleep_data.clk_sel[0] =
			mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(0));
		rksleep_data.clk_sel[1] =
			mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(1));
		rksleep_data.clk_sel[2] =
			mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(2));
		rksleep_data.clk_sel[3] =
			mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(3));
		rksleep_data.clk_sel[4] =
			mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(4));
		rksleep_data.clk_sel[5] =
			mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(5));
		rksleep_data.clk_sel[8] =
			mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(8));
		rksleep_data.clk_sel[9] =
			mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(9));
		rksleep_data.clk_sel[10] =
			mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(10));
		rksleep_data.clk_sel[12] =
			mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(12));
	}

	plls_suspend(NPLL_ID, cfg);
	plls_suspend(CPLL_ID, cfg);
	plls_suspend(GPLL_ID, cfg);

	/* set 1, pdbus pll is gpll */
	mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(8),
		      BITS_WITH_WMASK(1, 0x1, 7));
	/* pd_bus clk */
	mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(8),
		      BITS_WITH_WMASK(0, 0x1f, 0) |
		      BITS_WITH_WMASK(0, 0x3, 8) |
		      BITS_WITH_WMASK(0, 0x7, 12));

       /* set mcu clk sel */
	mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(12),
		      BITS_WITH_WMASK(1, 0x1, 7));

	/* mcu clk */
	mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(12),
		      BITS_WITH_WMASK(0, 0x1f, 0) |
		      BITS_WITH_WMASK(0, 0x7, 8));

	/* peri aclk hclk pclk */
	mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(9),
		      BITS_WITH_WMASK(0, 0x1f, 0) |
		      BITS_WITH_WMASK(0, 0x3, 8) |
		      BITS_WITH_WMASK(0, 0x3, 12));

	/* pmu alive */
	mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(10),
		      BITS_WITH_WMASK(0, 0x1f, 0) |
		      BITS_WITH_WMASK(0, 0x1f, 8));

	plls_suspend(ABPLL_ID, cfg);
	plls_suspend(ALPLL_ID, cfg);

	/* core_b */
	mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(0),
		      BITS_WITH_WMASK(0, 0x1f, 0) |
		      BITS_WITH_WMASK(0, 0x1f, 8));

	mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(1),
		      BITS_WITH_WMASK(0, 0x1f, 0) |
		      BITS_WITH_WMASK(0, 0x1f, 8));

	/* pd_core_l */
	mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(2),
		      BITS_WITH_WMASK(0, 0x1f, 0) |
		      BITS_WITH_WMASK(0, 0x1f, 8));

	mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(3),
		      BITS_WITH_WMASK(0, 0x1f, 0) |
		      BITS_WITH_WMASK(0, 0x1f, 8));

	delay_time_calib_set(DLY_PER_US_CYCL_24M);
}

static inline void plls_resume_check_reg(uint32_t pll_id)
{
	if ((rksleep_data.plls_con[pll_id][0] !=
	      mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 0))) ||
	     (rksleep_data.plls_con[pll_id][1] !=
	     mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 1))) ||
	     (rksleep_data.plls_con[pll_id][2] !=
	     mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 2)))) {
		ERROR("%s:pll_id%d\n", __func__, pll_id);
		ERROR("0x%x = 0x%x\n",
		      rksleep_data.plls_con[pll_id][0],
		      mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 0)));
		ERROR("0x%x = 0x%x\n",
		      rksleep_data.plls_con[pll_id][1],
		      mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 1)));
		ERROR("0x%x = 0x%x\n",
		      rksleep_data.plls_con[pll_id][2],
		      mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 2)));
		ERROR("0x%x = 0x%x\n",
		      rksleep_data.plls_con[pll_id][3],
		      mmio_read_32(CRU_BASE + PLL_CONS((pll_id), 3)));
		while (1)
			;
	}
}

static inline void plls_resume_pre(uint32_t pll_id)
{
	uint32_t pllcon0, pllcon1, pllcon2, pllcon3;

	if (rksleep_data.plls_con[pll_id][3] & PLL_PWR_DN_MSK) {
		pllcon0 = rksleep_data.plls_con[pll_id][0] | 0xffff0000;
		pllcon1 = rksleep_data.plls_con[pll_id][1] | 0xffff0000;
		pllcon2 = rksleep_data.plls_con[pll_id][2] | 0xffff0000;
		pllcon3 = rksleep_data.plls_con[pll_id][3] | 0xffff0000;

		mmio_write_32(CRU_BASE + PLL_CONS(pll_id, 0), pllcon0);
		mmio_write_32(CRU_BASE + PLL_CONS(pll_id, 1), pllcon1);
		mmio_write_32(CRU_BASE + PLL_CONS(pll_id, 2), pllcon2);
		mmio_write_32(CRU_BASE + PLL_CONS(pll_id, 3), pllcon3);

		return;
	}

	mmio_write_32(CRU_BASE + PLL_CONS((pll_id), 3), PLL_SLOW_BITS);

	mmio_write_32(CRU_BASE + PLL_CONS((pll_id), 3),
		      rksleep_data.plls_con[pll_id][3] | PLL_CON3_RES_WMSK);

	mmio_write_32(CRU_BASE + PLL_CONS((pll_id), 3), PLL_PWR_ON);
	mmio_write_32(CRU_BASE + PLL_CONS((pll_id), 3), PLL_NO_BYPASS);

	pllcon0 = rksleep_data.plls_con[pll_id][0];
	pllcon1 = rksleep_data.plls_con[pll_id][1];
	pllcon2 = rksleep_data.plls_con[pll_id][2];

	/* enter rest */
	mmio_write_32(CRU_BASE + PLL_CONS(pll_id, 3), PLL_RESET);
	mmio_write_32(CRU_BASE + PLL_CONS(pll_id, 0),
		      pllcon0 | BITS_WMSK(0xf, 0) | BITS_WMSK(0x3f, 8));
	mmio_write_32(CRU_BASE + PLL_CONS(pll_id, 1), pllcon1);
	mmio_write_32(CRU_BASE + PLL_CONS(pll_id, 2), pllcon2);
}

static inline void plls_resume(uint32_t cfg)
{
	if ((cfg & SLP_PMU_PLLS_PWRDN) && !(cfg & SLP_ARMOFF_LOGOFF)) {
		plls_resume_check_reg(ABPLL_ID);
		plls_resume_check_reg(ALPLL_ID);
		plls_resume_check_reg(GPLL_ID);
		plls_resume_check_reg(CPLL_ID);
		plls_resume_check_reg(NPLL_ID);
	} else {
		plls_resume_pre(ABPLL_ID);
		plls_resume_pre(ALPLL_ID);
		plls_resume_pre(GPLL_ID);
		plls_resume_pre(CPLL_ID);
		plls_resume_pre(NPLL_ID);

		usdelay(6);

		if (!(rksleep_data.plls_con[ABPLL_ID][3] & PLL_PWR_DN_MSK))
			mmio_write_32(CRU_BASE + PLL_CONS(ABPLL_ID, 3),
				      PLL_RESET_RESUME);

		if (!(rksleep_data.plls_con[ALPLL_ID][3] & PLL_PWR_DN_MSK))
			mmio_write_32(CRU_BASE + PLL_CONS(ALPLL_ID, 3),
				      PLL_RESET_RESUME);

		if (!(rksleep_data.plls_con[GPLL_ID][3] & PLL_PWR_DN_MSK))
			mmio_write_32(CRU_BASE + PLL_CONS(GPLL_ID, 3),
				      PLL_RESET_RESUME);

		if (!(rksleep_data.plls_con[CPLL_ID][3] & PLL_PWR_DN_MSK))
			mmio_write_32(CRU_BASE + PLL_CONS(CPLL_ID, 3),
				      PLL_RESET_RESUME);

		if (!(rksleep_data.plls_con[NPLL_ID][3] & PLL_PWR_DN_MSK))
			mmio_write_32(CRU_BASE + PLL_CONS(NPLL_ID, 3),
				      PLL_RESET_RESUME);

		usdelay(200);
	}

	pm_pll_wait_lock(ABPLL_ID);
	pm_pll_wait_lock(ALPLL_ID);
	pm_pll_wait_lock(GPLL_ID);
	pm_pll_wait_lock(CPLL_ID);
	pm_pll_wait_lock(NPLL_ID);

	/*
	 * in 24M, 8 syscles is 1us
	 */
	delay_sycle(5);

	mmio_write_32(CRU_BASE + PLL_CONS(ABPLL_ID, 3),
		      rksleep_data.plls_con[ABPLL_ID][3] | PLL_BYPASS_W_MSK);
	mmio_write_32(CRU_BASE + PLL_CONS(ALPLL_ID, 3),
		      rksleep_data.plls_con[ALPLL_ID][3] | PLL_BYPASS_W_MSK);
	mmio_write_32(CRU_BASE + PLL_CONS(GPLL_ID, 3),
		      rksleep_data.plls_con[GPLL_ID][3] | PLL_BYPASS_W_MSK);
	mmio_write_32(CRU_BASE + PLL_CONS(CPLL_ID, 3),
		      rksleep_data.plls_con[CPLL_ID][3] | PLL_BYPASS_W_MSK);
	mmio_write_32(CRU_BASE + PLL_CONS(NPLL_ID, 3),
		      rksleep_data.plls_con[NPLL_ID][3] | PLL_BYPASS_W_MSK);
}

static void pm_plls_resume(uint32_t cfg)
{
	uint32_t i;
	/* resume ABPLL_ID ALPLL_ID GPLL_ID CPLL_ID NPLL_ID */
	plls_resume(cfg);
	if (cfg & SLP_ARMOFF_LOGOFF) {
		for (i = 0; i < CRU_CLKSEL_CON_CNT; i++)
			if (i == 28 || i == 32 || i == 34 || i == 36 ||
			    i == 40 || i == 42 || i == 44 || i == 54)
				mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(i),
					      rksleep_data.clk_sel[i]);
			else
				mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(i),
					      rksleep_data.clk_sel[i] |
					      0xffff0000);
	} else {
		/* resume core */
		mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(0),
			      rksleep_data.clk_sel[0] |
			      BITS_WMSK(0x1f, 0) |
			      BITS_WMSK(0x1f, 8));

		mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(1),
			      rksleep_data.clk_sel[1] |
			      BITS_WMSK(0x1f, 0) |
			      BITS_WMSK(0x1f, 8));

		mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(2),
			      rksleep_data.clk_sel[2] |
			      BITS_WMSK(0x1f, 0) |
			      BITS_WMSK(0x1f, 8));

		mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(3),
			      rksleep_data.clk_sel[3] |
			      BITS_WMSK(0x1f, 0) |
			      BITS_WMSK(0x1f, 8));
	}

	/* alpll abpll mode resume */
	mmio_write_32(CRU_BASE + PLL_CONS(ABPLL_ID, 3),
		      rksleep_data.plls_con[ABPLL_ID][3] | PLLS_MODE_WMASK);

	mmio_write_32(CRU_BASE + PLL_CONS(ALPLL_ID, 3),
		      rksleep_data.plls_con[ALPLL_ID][3] | PLLS_MODE_WMASK);

	if (!(cfg & SLP_ARMOFF_LOGOFF)) {
		/* pd bus clk */
		mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(8),
			      rksleep_data.clk_sel[8] |
			      BITS_WMSK(0x1f, 0) |
			      BITS_WMSK(0x3, 8) |
			      BITS_WMSK(0x7, 12));

		/* pdbus pll resume */
		mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(8),
			      rksleep_data.clk_sel[8] |
			      BITS_WMSK(0x1, 7));

		mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(12),
			      rksleep_data.clk_sel[12] |
			      BITS_WMSK(0x1f, 0) |
			      BITS_WMSK(0x7, 8));

		/* mcu */
		mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(12),
			      rksleep_data.clk_sel[12] |
			      BITS_WMSK(0x1, 7));

		/* peri aclk hclk pclk */
		mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(9),
			      rksleep_data.clk_sel[9] |
			      BITS_WMSK(0x1f, 0)  |
			      BITS_WMSK(0x3, 8) |
			      BITS_WMSK(0x3, 12));

		/* pmu alive  */
		mmio_write_32(CRU_BASE + CRU_CLKSELS_CON(10),
			      rksleep_data.clk_sel[10] |
			      BITS_WMSK(0x1f, 0) |
			      BITS_WMSK(0x1f, 8));
	}
	/* cpll gpll npll */
	mmio_write_32(CRU_BASE + PLL_CONS(GPLL_ID, 3),
		      rksleep_data.plls_con[GPLL_ID][3] |
		      PLLS_MODE_WMASK);
	mmio_write_32(CRU_BASE + PLL_CONS(CPLL_ID, 3),
		      rksleep_data.plls_con[CPLL_ID][3] |
		      PLLS_MODE_WMASK);

	mmio_write_32(CRU_BASE + PLL_CONS(NPLL_ID, 3),
		      rksleep_data.plls_con[NPLL_ID][3] |
		      PLLS_MODE_WMASK);

	delay_time_calib_set(DLY_PER_US_CYCL_800M);
}

void soc_sleep_config(uint32_t cfg)
{
	timers_suspend(cfg);

	putchar('6');

	peri_pin_to_gpio(cfg);
	putchar('7');

	rk_pm_soc_pd_suspend(cfg);
	putchar('8');

	mcu_suspend(cfg);
	putchar('9');

	imem_code_save(cfg);
	putchar('a');

	/* * if pdbus pd, its clk must be no gate* */
	gtclks_suspend();
	putchar('b');

	pm_plls_suspend(cfg);
	putchar('c');

#if !USE_COHERENT_MEM && RKSLEEP_DATA_COHERENT
	flush_dcache_range((uint64_t)rksleep_data.sleep_cnt,
			   sizeof(rksleep_data));
#endif
}

void soc_sleep_config_restore(uint32_t cfg)
{
	pm_plls_resume(cfg);
	putchar('c');

	gtclks_resume();
	putchar('b');

	imem_code_resume(cfg);
	putchar('a');

	mcu_resume(cfg);
	putchar('9');

	rk_pm_soc_pd_resume(cfg);
	putchar('8');

	peri_pin_to_restore(cfg);
	putchar('7');

	timers_resume(cfg);
	putchar('6');
}

static void pmu_scu_b_pwrdn(void)
{
	uint32_t wait_cnt = 0;

	if ((mmio_read_32(PMU_BASE + PMU_PWRDN_ST) &
	     PM_PWRDM_CPUSB_MSK) != PM_PWRDM_CPUSB_MSK) {
		ERROR("%s: not all cpus is off\n", __func__);
		return;
	}

	rk3368_flash_l2_b();

	regs_updata_bit_set(PMU_BASE + PMU_SFT_CON, pmu_sft_acinactm_clst_b);

	while (!(mmio_read_32(PMU_BASE +
	       PMU_CORE_PWR_ST) & BIT(clst_b_l2_wfi))) {
		wait_cnt++;
		if (!(wait_cnt % MAX_WAIT_COUNT))
			ERROR("%s:wait cluster-b l2(%x)\n", __func__,
			      mmio_read_32(PMU_BASE + PMU_CORE_PWR_ST));
	}
	rk3368_pmu_bus_idle(bus_ide_req_clst_b, 1);

	scus_power_domain(1, 0, 1, pmu_power_off);
}

void pmu_scu_b_pwrup(void)
{
	scus_power_domain(1, 0, 1, pmu_power_on);
	regs_updata_bit_clr(PMU_BASE + PMU_SFT_CON, pmu_sft_acinactm_clst_b);
	rk3368_pmu_bus_idle(bus_ide_req_clst_b, 0);
}

static void pmu_set_sleep_mode(void)
{
	uint32_t cfg = rk_cpuinfo.slp_cfg;

	rksleep_data.pmugrf_gpio0a_iomux = mmio_read_32(PMU_GRF_BASE);
	rksleep_data.pmugrf_soc_con0 =
		mmio_read_32(PMU_GRF_BASE + PMUGRF_SOC_CON0);
	rksleep_data.pmu_sft_con = mmio_read_32(PMU_BASE + PMU_SFT_CON);
	putchar('2');

	ddr_suspend_save();
	putchar('3');
	pll_sleep_32k_pvtm_config(cfg);
	putchar('4');
	pmu_sleep_mode_config(cfg);
	putchar('5');
	soc_sleep_config(cfg);
	putchar('d');
	regs_updata_bit_set(PMU_BASE + PMU_PWRMD_CORE, pmu_mdcr_global_int_dis);
	regs_updata_bit_set(PMU_BASE + PMU_SFT_CON, pmu_sft_glbl_int_dis_b);

	pmu_scu_b_pwrdn();
	putchar('e');

	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(1),
		      (PMUSRAM_BASE >> CPU_BOOT_ADDR_ALIGN) |
		      CPU_BOOT_ADDR_WMASK);
	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(2),
		      (PMUSRAM_BASE >> CPU_BOOT_ADDR_ALIGN) |
		      CPU_BOOT_ADDR_WMASK);
	putchar('f');
}

void pmu_set_sleep_mode_restore(void)
{
	uint32_t cfg = rk_cpuinfo.slp_cfg;
	uint32_t pwrmd_com = mmio_read_32(PMU_BASE + PMU_PWRMD_COM);

	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(1),
		      (COLD_BOOT_BASE >> CPU_BOOT_ADDR_ALIGN) |
		      CPU_BOOT_ADDR_WMASK);
	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(2),
		      (COLD_BOOT_BASE >> CPU_BOOT_ADDR_ALIGN) |
		      CPU_BOOT_ADDR_WMASK);
	putchar('e');

	pmu_scu_b_pwrup();
	putchar('d');

	/* must do clear interrupt */
	regs_updata_bit_clr(PMU_BASE + PMU_PWRMD_CORE, pmu_mdcr_global_int_dis);
	regs_updata_bit_clr(PMU_BASE + PMU_SFT_CON, pmu_sft_glbl_int_dis_b);

	soc_sleep_config_restore(cfg);
	putchar('5');

	if (pwrmd_com & BIT(pmu_mode_bus_pd))
		plat_rockchip_gic_init();
	putchar('4');

	mmio_write_32(PMU_BASE + PMU_PWRMD_COM, 0);
	mmio_write_32(PMU_BASE + PMU_PWRMD_CORE, 0);
	mmio_write_32(PMU_BASE + PMU_WKUP_CFG0, 0);
	mmio_write_32(PMU_BASE + PMU_WKUP_CFG1, 0);

	pll_sleep_32k_pvtm_config_restore(cfg);
	putchar('3');

	/*
	 * first disable ddr io ret
	 *(RK3368_PMUGRF_SOC_CON0:
	 * pgrf_soc_ddrphy_bufen_core pgrf_soc_ddrphy_bufen_io)
	 * then enable ddr io ret ctr by software (RK3368_PMU_SFT_CON)
	 */
	mmio_write_32(PMU_GRF_BASE + PMUGRF_SOC_CON0,
		      rksleep_data.pmugrf_soc_con0 | 0xffff0000);
	mmio_write_32(PMU_BASE + PMU_SFT_CON, rksleep_data.pmu_sft_con);

	mmio_write_32(PMU_GRF_BASE,
		      rksleep_data.pmugrf_gpio0a_iomux |
		      BITS_SHIFT(GPIO0A0_SEL_MSK,
				 GPIO0A0_SEL_SHIFT + 16));
	putchar('2');
}

int rockchip_soc_sys_pwr_dm_suspend(void)
{
	uint32_t cfg = rk_cpuinfo.slp_cfg;

	if ((rk_cpuinfo.suspend_dbg_en) ||
	    (rk_cpuinfo.slp_cfg  & SLP_ARMOFF_LOGPD) ||
	    (rk_cpuinfo.slp_cfg  & SLP_ARMOFF_LOGOFF)) {
		rockchip_uart_debug_save();
		if ((rk_cpuinfo.suspend_dbg_en))
			console_init(RK3368_UART_DBG_BASE,
				     RK3368_UART_CLOCK,
				     RK3368_BAUDRATE);
	}

	putchar('\n');
	dbg_pm_dump_inten();
	dbg_pmu_sleep_enter_info(cfg);
	putchar('0');
	psram_sleep_cfg->ddr_flag = 0;

	nonboot_cpus_off();
	putchar('1');

	pmu_set_sleep_mode();
	putchar('A');

	dbg_pmu_sleep_mode_tst_info(cfg);

	return 0;
}

int rockchip_soc_sys_pwr_dm_resume(void)
{
	uint32_t cfg = rk_cpuinfo.slp_cfg;
	uint32_t pwrmd_com = mmio_read_32(PMU_BASE + PMU_PWRMD_COM);

	if (pwrmd_com & BIT(pmu_mode_bus_pd))
		console_init(RK3368_UART_DBG_BASE,
			     RK3368_UART_CLOCK,
			     RK3368_BAUDRATE);
	putchar('f');

	pmu_set_sleep_mode_restore();
	putchar('1');

	if (!(cfg & (SLP_ARMOFF_LOGPD | SLP_ARMOFF_LOGOFF)))
		dbg_pm_dump_irq();
	putchar('0');

	dbg_pmu_resume_info();

	if ((rk_cpuinfo.suspend_dbg_en) ||
	    (pwrmd_com & BIT(pmu_mode_bus_pd))) {
		console_uninit();
		rockchip_uart_debug_restore();
	}

	return 0;
}

void __dead2 rockchip_soc_sys_pd_pwr_dn_wfi(void)
{
	uint32_t cfg = rk_cpuinfo.slp_cfg;

	plls_enter_deep(cfg);
	putchar('S');
	if (mmio_read_32(PMU_GPIO0_BASE + GPIO_INT_STATUS)) {
		INFO("gpio int =%x\n",
		     mmio_read_32(PMU_GPIO0_BASE + GPIO_INT_STATUS));
		plls_enter_slow(cfg);
		disable_mmu_icache_el3();
		bl31_warm_entrypoint();
		while (1)
			;
	} else {
		psci_power_down_wfi();
	}
}

void plat_rockchip_pmusram_prepare(void)
{
	uint32_t *sram_dst, *sram_src;
	size_t sram_size = 2;
	uint32_t code_size;

	/* pmu sram code and data prepare */
	sram_dst = (uint32_t *)PMUSRAM_BASE;
	sram_src = (uint32_t *)&pmu_cpuson_entrypoint_start;
	sram_size = (uint32_t *)&pmu_cpuson_entrypoint_end -
		    (uint32_t *)sram_src;
	u32_align_cpy(sram_dst, sram_src, sram_size);

	/* ddr code */
	sram_dst += sram_size;
	sram_src = ddr_get_resume_code_base();
	code_size = ddr_get_resume_code_size();
	u32_align_cpy(sram_dst, sram_src, code_size / 4);
	psram_sleep_cfg->ddr_func = (uint64_t)sram_dst;

	/* ddr data */
	sram_dst += (code_size / 4);
	psram_sleep_cfg->ddr_data = (uint64_t)sram_dst;

	assert((uint64_t)(sram_dst + ddr_get_resume_data_size() / 4)
						 < PSRAM_SP_BOTTOM);
	psram_sleep_cfg->sp = PSRAM_SP_TOP;
}

static uint32_t check_sleep_config(uint32_t slp_cfg)
{
	if (slp_cfg & SLP_PMU_DIS_OSC)
		if ((slp_cfg & SLP_PMU_DIS_OSC_CDT) != SLP_PMU_DIS_OSC_CDT) {
			slp_cfg &= ~SLP_PMU_DIS_OSC;
			WARN("%s: dis osc condition is not enough\n", __func__);
		}

	return slp_cfg;
}

int suspend_mode_handler(uint64_t mode_id,
			 uint64_t config1,
			 uint64_t config2)
{
	switch (mode_id) {
	case SUSPEND_MODE_CONFIG:
		rk_cpuinfo.slp_cfg = check_sleep_config(config1);
		return 0;

	case SUSPEND_DEBUG_ENABLE:
		rk_cpuinfo.suspend_dbg_en = config1;
		return 0;

	case WKUP_SOURCE_CONFIG:
		rk_cpuinfo.suspend_wkup_cfg = config1;
		return 0;

	default:
		ERROR("%s: unhandled sip (0x%lx)\n", __func__, mode_id);
		return -1;
	}
}

static void set_sys_sleep_mode_default(void)
{
	uint32_t slp_cfg = 0;

	slp_cfg |= SLP_ARMOFF_LOGPD
		| SLP_PMU_PLLS_PWRDN
		| SLP_PMU_PMUALIVE_32K
		| SLP_SFT_PLLS_DEEP
		| SLP_PMU_DIS_OSC
		| SLP_SFT_PD_NBSCUS
		/* | SLP_SFT_PD_PERI */
		;

	rk_cpuinfo.slp_cfg = check_sleep_config(slp_cfg);
	rk_cpuinfo.suspend_dbg_en = 0;
	rk_cpuinfo.suspend_wkup_cfg = 0;
}

void plat_rockchip_pmu_init(void)
{
	uint32_t cpu;

	/* register requires 32bits mode, switch it to 32 bits */
	cpu_warm_boot_addr = (uint64_t)platform_cpu_warmboot;

	for (cpu = 0; cpu < PLATFORM_CORE_COUNT; cpu++)
		cpuson_flags[cpu] = 0;

	psram_sleep_cfg->boot_mpidr = read_mpidr_el1() & 0xffff;

	nonboot_cpus_off();
	rk_register_interrupt_routing_model();

	INFO("%s(%d): pd status %x\n", __func__, __LINE__,
	     mmio_read_32(PMU_BASE + PMU_PWRDN_ST));

	set_sys_sleep_mode_default();

	rockchip_ddr_resume_init();
}
