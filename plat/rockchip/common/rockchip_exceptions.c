/*
 * Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
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
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
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
#include <plat_private.h>
#include <arch_helpers.h>
#include <assert.h>
#include <bl_common.h>
#include <bl31.h>
#include <context_mgmt.h>
#include <cpu_data.h>
#include <debug.h>
#include <gic_common.h>
#include <interrupt_mgmt.h>
#include <platform_def.h>
#include <platform.h>
#include <runtime_svc.h>
#include <string.h>
#include <pmu.h>
#include <rockchip_sip_svc.h>

#define UARTDBG_CFG_INIT		0xf0
#define UARTDBG_CFG_OSHDL_TO_OS		0xf1
#define UARTDBG_CFG_OSHDL_CPUSW		0xf3
#define UARTDBG_CFG_OSHDL_ENDBG		0xf4
#define UARTDBG_CFG_OSHDL_DISDBG	0xf5
#define UARTDBG_CFG_SET_SHARE_MEM	0xf6
#define UARTDBG_CFG_PRINT_PORT		0xf7
#define UARTDBG_CFG_FIQ_ENABLE		0xf8
#define UARTDBG_CFG_FIQ_DISABLE		0xf9

#define enable_irq()			write_daifclr(DAIF_IRQ_BIT)
#define enable_fiq()			write_daifclr(DAIF_FIQ_BIT)
#define enable_serror()			write_daifclr(DAIF_ABT_BIT)
#define enable_debug_exceptions()	write_daifclr(DAIF_DBG_BIT)
#define disable_irq()			write_daifset(DAIF_IRQ_BIT)
#define disable_fiq()			write_daifset(DAIF_FIQ_BIT)
#define disable_serror()		write_daifset(DAIF_ABT_BIT)
#define disable_debug_exceptions()	write_daifset(DAIF_DBG_BIT)

#define IRQ_NUM_MAX		182
#define GICC_IAR_INT_ID_MASK	0x3ff
#define GIC_RET_SUCCESS		0
#define GIC_RET_ERROR		-1
#define GIC_RET_ERRORID		-2
#define GIC_RET_SERVING		-10

#define MPDIR_CRT(cluster, cpu) \
	((cluster) << MPIDR_AFF1_SHIFT | (cpu) << MPIDR_AFF0_SHIFT)

static interrupt_type_handler_t rockchip_secfiq_handler[IRQ_NUM_MAX];
static cpu_context_t fiq_ns_context[PLATFORM_CORE_COUNT];

static uint8_t rockchip_secure_interrupt_setup;

struct uartdbg_uart_info_t {
	uint64_t os_handler;
	uint32_t irq_id;
	uint32_t irqstat; /* for end of the irq */
	uint32_t dbg_en;
};

struct uartdbg_el1_st_t {
	uint32_t el3_daif;
	uint32_t spsr_el3;
	/*
	 * linux os may modify sp_el1,so we need to resume it.
	 * if we use cm_el1_sysregs_context_save() to save sysreg,
	 * sp_el1 is not need to saving
	 */
	uint64_t sp_el1;
	void *tf_ctx;
	char *fiq_dbg_ctx;
};

static struct uartdbg_el1_st_t uartdbg_el_data;
static struct uartdbg_uart_info_t uartdbg_uart_info;
static uint8_t boot_cpu_msk, boot_cpu;

#ifndef SPD_opteed
static cpu_context_t rk_sec_context[PLATFORM_CORE_COUNT];
#endif
static uint64_t gic_handle_irq(uint32_t id,
			       uint32_t flags,
			       void *handle,
			       void *cookie)
{
	if (rockchip_secfiq_handler[id])
		return rockchip_secfiq_handler[id] (id, flags, handle, cookie);
	else
		return GIC_RET_ERRORID;
}

static uint32_t uartdbg_set_print_port(uint32_t uart_base, uint32_t baudrate)
{
	return SIP_RET_NOT_SUPPORTED;
}

uint32_t get_uart_irq_id(void)
{
	return uartdbg_uart_info.irq_id;
}

/******************************************************************
 ** linux os handler
 ******************************************************************/
static uint64_t gic_handle_ipi(uint32_t id,
			       uint32_t flags,
			       void *handle,
			       void *cookie)
{
	assert(handle == cm_get_context(NON_SECURE));

	if (rockchip_secfiq_handler[id])
		return rockchip_secfiq_handler[id] (id, flags, handle, cookie);
	else
		return GIC_RET_ERRORID;
}

static uint64_t gic_handle_except(uint32_t id,
				  uint32_t flags,
				  void *handle,
				  void *cookie)
{
	uint32_t irqstat, irqnr;
	uint64_t ret;

	/*
	 * Check that this function is called with SP_EL0 as the stack
	 * pointer
	 */
	do {
		irqstat = plat_ic_acknowledge_interrupt();
		irqnr = irqstat & GICC_IAR_INT_ID_MASK;

		if (irqnr > 15 && irqnr < 1021) {
			ret = gic_handle_irq(irqnr, flags, handle, cookie);
			/*
			 * 1. Only fiq_debugger will return GIC_RET_SERVING,
			 *    we hope do EOI by ourself but not here.
			 * 2. Other irqs do plat_ic_end_of_interrupt here.
			 */
			if (ret != GIC_RET_SERVING)
				plat_ic_end_of_interrupt(irqstat);
			continue;
		}

		if (irqnr < 16) {
			plat_ic_end_of_interrupt(irqstat);

			ret = gic_handle_ipi(irqnr, flags, handle, cookie);
			if (ret == GIC_RET_ERRORID)
				return 0;
			continue;
		}
		break;
	} while (1);

	return 0;
}

static unsigned int gic_get_cpuif_id(void)
{
	unsigned int val;

	val = mmio_read_32(PLAT_RK_GICD_BASE + GICD_ITARGETSR);
	return val & GIC_TARGET_CPU_MASK;
}

/*
 * cpu_context[] nosecure init is runtime_svc_init()
 * so rk_register_interrupt_routing_model call following runtime_svc_init()
 */
void rk_register_interrupt_routing_model(void)
{
	uint64_t rc, flags;
	uint32_t intr_type;
	unsigned int gic_version;
#ifndef SPD_opteed
	uint64_t mpidr;
	uint32_t linear_id, cluster, cpu;
#endif

#ifndef PLAT_SKIP_OPTEE_S_EL1_INT_REGISTER
	rockchip_secure_interrupt_setup = 0;
	return;
#endif

#ifndef SPD_opteed
	for (cluster = 0; cluster < PLATFORM_CLUSTER_COUNT; cluster++) {
		if (cluster == 0) {
			for (cpu = 0; cpu < PLATFORM_CLUSTER0_CORE_COUNT; cpu++) {
				mpidr = MPDIR_CRT(cluster, cpu);
				linear_id = plat_core_pos_by_mpidr(mpidr);

				set_cpu_data_by_index(mpidr,
						      cpu_context[SECURE],
						      (void *)&rk_sec_context[linear_id]);
			}
		}

		if (cluster == 1) {
			for (cpu = 0; cpu < PLATFORM_CLUSTER1_CORE_COUNT; cpu++) {
				mpidr = MPDIR_CRT(cluster, cpu);
				linear_id = plat_core_pos_by_mpidr(mpidr);
				set_cpu_data_by_index(mpidr,
						      cpu_context[SECURE],
						      (void *)&rk_sec_context[linear_id]);
			}
		}
	}
	INFO("Using rkfiq sec cpu_context!\n");
#else
	INFO("Using opteed sec cpu_context!\n");
#endif

	/*
	 * if set_interrupt_rm_flag(flags, NON_SECURE);
	 * it means Routed to EL3 from NS. Taken to S-EL1 from Secure
	 */
	flags = 0;
	set_interrupt_rm_flag(flags, NON_SECURE);

	/*
	 * if set_interrupt_rm_flag(flags, NON_SECURE);
	 * and set_interrupt_rm_flag(flags, SECURE);
	 * it means Routed to EL3 from NS and Secure
	 * INTR_SEL1_VALID_RM1
	 */

	gic_version = plat_rockchip_gic_version();
	if (gic_version == ARCH_REV_GICV2)
		intr_type = INTR_TYPE_S_EL1;
	else if (gic_version == ARCH_REV_GICV3)
		intr_type = INTR_TYPE_EL3;
	else
		panic();

	rc = register_interrupt_type_handler(intr_type,
					     gic_handle_except,
					     flags);
	if (rc)
		panic();

	boot_cpu = plat_my_core_pos();
	boot_cpu_msk = gic_get_cpuif_id();
	INFO("boot cpu mask: %d\n", boot_cpu_msk);

	rockchip_secure_interrupt_setup = 1;
}

int32_t register_secfiq_handler(uint32_t id, interrupt_type_handler_t handler)
{
#ifndef PLAT_SKIP_OPTEE_S_EL1_INT_REGISTER
	return SIP_RET_NOT_SUPPORTED;
#endif

	if (!rockchip_secure_interrupt_setup)
		return SIP_RET_NOT_SUPPORTED;

	if (id >= IRQ_NUM_MAX)
		return SIP_RET_INVALID_PARAMS;

	rockchip_secfiq_handler[id] = handler;

	return SIP_RET_SUCCESS;
}

/***************************uart_dbg****************************/
static void uartdbg_to_oshdl_handler(uint64_t handler, void *handle_ctx)
{
	el3_state_t *el3_state;
	gp_regs_t *gp_state;
	uint32_t spsr_el3;
	uint32_t scr_el3;
	char *fiq_dbg_ctx;
	void *fiq_ret_ctx;
	gp_regs_t *el1_gp_state;
	size_t cpudata_size = sizeof(cpu_context_t);
	uint32_t cpu = plat_core_pos_by_mpidr(read_mpidr_el1());
	uint64_t sp_el0, cpsr_el1;

	assert(handle_ctx == cm_get_context(NON_SECURE));

	/* save: cpsr_el1, sp_el1 */
	cm_el1_sysregs_context_save(NON_SECURE);
	el1_gp_state = get_gpregs_ctx(cm_get_context(NON_SECURE));
	sp_el0 = read_ctx_reg(el1_gp_state, CTX_GPREG_SP_EL0);
	el3_state = get_el3state_ctx(cm_get_context(NON_SECURE));
	cpsr_el1 = read_ctx_reg(el3_state, CTX_SPSR_EL3);

	uartdbg_el_data.tf_ctx = handle_ctx;
	fiq_ret_ctx = &fiq_ns_context[cpu];

	fiq_dbg_ctx = uartdbg_el_data.fiq_dbg_ctx + cpu * cpudata_size;
	memcpy((char *)(fiq_dbg_ctx + CTX_EL3STATE_OFFSET + CTX_SPSR_EL3),
		&cpsr_el1, sizeof(cpsr_el1));
	if (uartdbg_uart_info.dbg_en) {
		if (!uartdbg_el_data.tf_ctx)
			ERROR("uartdbg_el_data.tf_ctx err\n");
		if (!fiq_dbg_ctx)
			ERROR("fiq_dbg_ctx error\n");
		memcpy(fiq_dbg_ctx, uartdbg_el_data.tf_ctx, cpudata_size);
	}

	cm_set_context(fiq_ret_ctx, NON_SECURE);
	cm_set_next_eret_context(NON_SECURE);

	/* when taking an exception to EL3,
	 * holds the address to return to. ELR_EL3
	 */
	el3_state = get_el3state_ctx(fiq_ret_ctx);

	write_ctx_reg(el3_state, CTX_ELR_EL3, handler);

	/* disable all exception */
	/* SPSR_EL3: Holds the saved processor state
	 * when an exception is taken to EL3
	 */
	write_ctx_reg(el3_state, CTX_SPSR_EL3, uartdbg_el_data.spsr_el3);
	spsr_el3 = read_ctx_reg(el3_state, CTX_SPSR_EL3);
	spsr_el3 |= (DISABLE_ALL_EXCEPTIONS << SPSR_DAIF_SHIFT);

	write_ctx_reg(el3_state, CTX_SPSR_EL3, spsr_el3);
	spsr_el3 = read_ctx_reg(el3_state, CTX_SPSR_EL3);

	/* all exception can not target to el3 */
	/* SCR_EL3: Secure Configuration Register*/
	scr_el3 = read_scr();
	scr_el3 &= ~(SCR_FIQ_BIT | SCR_IRQ_BIT | SCR_EA_BIT);
	write_ctx_reg(el3_state, CTX_SCR_EL3, scr_el3);

	gp_state = get_gpregs_ctx(fiq_ret_ctx);

	write_ctx_reg(gp_state, CTX_GPREG_SP_EL0, sp_el0);
	/*
	  * oshdl_handler maybe change sp_el1,
	  * but fiq debug need the sp_el1 for kernel info
	  */
	write_ctx_reg(gp_state, CTX_GPREG_X0, uartdbg_el_data.sp_el1);
	write_ctx_reg(gp_state, CTX_GPREG_X1,
		      cpu * cpudata_size);
	write_ctx_reg(gp_state, CTX_GPREG_X2, cpu);
}

static uint64_t uartdbg_oshdl_to_os_smc_handler(void *handle)
{
	assert(handle == cm_get_context(NON_SECURE));

	cm_set_context(uartdbg_el_data.tf_ctx, NON_SECURE);
	cm_set_next_eret_context(NON_SECURE);
	cm_el1_sysregs_context_restore(NON_SECURE);

	plat_ic_end_of_interrupt(uartdbg_uart_info.irqstat);

	write_daif(uartdbg_el_data.el3_daif);
	/*
	 * cm_el1_sysregs_context_restore() will reset sp_el1
	 */
	isb();
	dsb();

	return SIP_RET_SUCCESS;
}

static uint64_t uartdbg_sw_cpu_smc_handler(void *handle, uint64_t mpidr)
{
	uint32_t tgt_cpu = plat_core_pos_by_mpidr(mpidr);

	if (tgt_cpu >= PLATFORM_CORE_COUNT)
		return SIP_RET_INVALID_PARAMS;

	plat_rockchip_gic_set_itargetsr(uartdbg_uart_info.irq_id, tgt_cpu);

	return SIP_RET_SUCCESS;
}

static uint64_t uartdbg_irq_handler(uint32_t irqstat,
				    uint32_t flags,
				    void *handle,
				    void *cookie)
{
	uartdbg_el_data.el3_daif = read_daif();
	uartdbg_el_data.sp_el1 = read_sp_el1();
	uartdbg_uart_info.irqstat = irqstat;

	/* MMU not enable(the time that cpu boots up in EL2) */
	if (GET_EL(read_spsr_el3()) == MODE_EL2) {
		plat_rockchip_gic_fiq_disable(uartdbg_uart_info.irq_id);
		return GIC_RET_SUCCESS;
	}

	disable_irq();
	disable_fiq();
	disable_serror();
	disable_debug_exceptions();
	uartdbg_to_oshdl_handler(uartdbg_uart_info.os_handler, handle);

	return GIC_RET_SERVING;
}

static uint64_t uartdbg_init_smc_handler(uint64_t uart_irq_id,
					 uint64_t os_handler,
					 struct arm_smccc_res *res)
{
	int ret;
	uint64_t page_base;

	ret = register_secfiq_handler(uart_irq_id, uartdbg_irq_handler);
	if (ret) {
		ERROR("register secure fiq handler fail: %d\n", ret);
		return ret;
	}

	plat_rockchip_gic_fiq_enable(uart_irq_id, boot_cpu);

	uartdbg_uart_info.irq_id = uart_irq_id;
	uartdbg_uart_info.os_handler = os_handler;
	uartdbg_uart_info.dbg_en = 0;
	uartdbg_el_data.spsr_el3 = read_spsr_el3();

	ret = share_mem_type2page_base(SHARE_PAGE_TYPE_UARTDBG, &page_base);
	if (ret) {
		ERROR("get uartdbg share memory fail: %d\n", ret);
		return ret;
	}

	uartdbg_el_data.fiq_dbg_ctx = (void *)page_base;

	assert(share_mem_type2page_size(SHARE_PAGE_TYPE_UARTDBG) >=
	       (PLATFORM_CORE_COUNT * (sizeof(cpu_context_t))));

	res->a1 = (uint64_t)uartdbg_el_data.fiq_dbg_ctx;

	return SIP_RET_SUCCESS;
}

static int uartdbg_enable_fiq(uint32_t tgt_cpu)
{
	plat_rockchip_gic_fiq_enable(uartdbg_uart_info.irq_id, tgt_cpu);

	return SIP_RET_SUCCESS;
}

static int uartdbg_disable_fiq(void)
{
	plat_rockchip_gic_fiq_disable(uartdbg_uart_info.irq_id);

	return SIP_RET_SUCCESS;
}

uint64_t fiq_debugger_smc_handler(uint64_t fun_id,
				  void *handle,
				  uint64_t x1,
				  uint64_t x2,
				  struct arm_smccc_res *res)
{
	switch (fun_id) {
	case UARTDBG_CFG_INIT:
		return uartdbg_init_smc_handler(x1, x2, res);

	case UARTDBG_CFG_OSHDL_TO_OS:
		return uartdbg_oshdl_to_os_smc_handler(handle);

	case UARTDBG_CFG_OSHDL_CPUSW:
		return uartdbg_sw_cpu_smc_handler(handle, x1);

	case UARTDBG_CFG_OSHDL_ENDBG:
		uartdbg_uart_info.dbg_en = 1;
		return SIP_RET_SUCCESS;

	case UARTDBG_CFG_OSHDL_DISDBG:
		uartdbg_uart_info.dbg_en = 0;
		return SIP_RET_SUCCESS;

	case UARTDBG_CFG_PRINT_PORT:
		return uartdbg_set_print_port(x1, x2);

	case UARTDBG_CFG_FIQ_ENABLE:
		return uartdbg_enable_fiq(x1);

	case UARTDBG_CFG_FIQ_DISABLE:
		return uartdbg_disable_fiq();

	default:
		return SMC_UNK;
	}
}
