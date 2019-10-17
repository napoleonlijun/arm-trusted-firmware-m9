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

#include <bl_common.h>
#include <gic_common_private.h>
#include <gicv2.h>
#include <gicv2_private.h>
#include <platform_def.h>
#include <plat_private.h>
#include <utils.h>
#include <gic_common_private.h>

/******************************************************************************
 * The following functions are defined as weak to allow a platform to override
 * the way the GICv2 driver is initialised and used.
 *****************************************************************************/
#pragma weak plat_rockchip_gic_driver_init
#pragma weak plat_rockchip_gic_init
#pragma weak plat_rockchip_gic_cpuif_enable
#pragma weak plat_rockchip_gic_cpuif_disable
#pragma weak plat_rockchip_gic_pcpu_init
#pragma weak plat_rockchip_gic_fiq_enable
#pragma weak plat_rockchip_gic_fiq_disable
#pragma weak plat_rockchip_gic_set_itargetsr

/******************************************************************************
 * On a GICv2 system, the Group 1 secure interrupts are treated as Group 0
 * interrupts.
 *****************************************************************************/
const unsigned int g0_interrupt_array[] = {
	PLAT_RK_G1S_IRQS,
};

/*
 * Ideally `rockchip_gic_data` structure definition should be a `const` but it
 * is kept as modifiable for overwriting with different GICD and GICC base when
 * running on FVP with VE memory map.
 */
gicv2_driver_data_t rockchip_gic_data = {
	.gicd_base = PLAT_RK_GICD_BASE,
	.gicc_base = PLAT_RK_GICC_BASE,
	.g0_interrupt_num = ARRAY_SIZE(g0_interrupt_array),
	.g0_interrupt_array = g0_interrupt_array,
};

/******************************************************************************
 * RockChip common helper to initialize the GICv2 only driver.
 *****************************************************************************/
void plat_rockchip_gic_driver_init(void)
{
	gicv2_driver_init(&rockchip_gic_data);
}

void plat_rockchip_gic_init(void)
{
	gicv2_distif_init();
	gicv2_pcpu_distif_init();
	gicv2_cpuif_enable();
}

/******************************************************************************
 * RockChip common helper to enable the GICv2 CPU interface
 *****************************************************************************/
void plat_rockchip_gic_cpuif_enable(void)
{
	gicv2_cpuif_enable();
}

/******************************************************************************
 * RockChip common helper to disable the GICv2 CPU interface
 *****************************************************************************/
void plat_rockchip_gic_cpuif_disable(void)
{
	gicv2_cpuif_disable();
}

/******************************************************************************
 * RockChip common helper to initialize the per cpu distributor interface
 * in GICv2
 *****************************************************************************/
void plat_rockchip_gic_pcpu_init(void)
{
	gicv2_pcpu_distif_init();
}

static void gicd_set_itargetsr(uintptr_t base,
			       unsigned int id,
			       unsigned int target)
{
	mmio_write_8(base + GICD_ITARGETSR + id, target & GIC_TARGET_CPU_MASK);
}

void plat_rockchip_gic_fiq_enable(uint32_t irq, uint8_t target_cpu)
{
	if (irq >= MIN_SPI_ID) {
		/* We have an SPI */
		gicd_clr_igroupr(PLAT_RK_GICD_BASE, irq);
		gicd_set_ipriorityr(PLAT_RK_GICD_BASE, irq,
				    GIC_HIGHEST_SEC_PRIORITY);
		gicd_set_itargetsr(PLAT_RK_GICD_BASE, irq, BIT(target_cpu));
		gicd_set_isenabler(PLAT_RK_GICD_BASE, irq);
	}
}

void plat_rockchip_gic_fiq_disable(uint32_t irq)
{
	gicd_set_icenabler(PLAT_RK_GICD_BASE, irq);
}

void plat_rockchip_gic_set_itargetsr(uint32_t irq, uint32_t target_cpu)
{
	gicd_set_itargetsr(PLAT_RK_GICD_BASE, irq, BIT(target_cpu));
}

unsigned int plat_rockchip_gic_version(void)
{
	unsigned int gic_version;

	gic_version = gicd_read_pidr2(PLAT_RK_GICD_BASE);
	gic_version = (gic_version >> PIDR2_ARCH_REV_SHIFT)
				& PIDR2_ARCH_REV_MASK;

	return gic_version;
}
