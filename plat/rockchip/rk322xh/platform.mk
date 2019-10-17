#
# Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

RK_PLAT			:=	plat/rockchip
RK_PLAT_SOC		:=	${RK_PLAT}/rk3328
RK_PLAT_COMMON		:=	${RK_PLAT}/common

RK_PLAT_SOC_PRT		:=	${RK_PLAT}/${PLAT}

PLAT_INCLUDES		:=	-Idrivers/arm/gic/common/			\
				-Idrivers/arm/gic/v2/				\
				-Iinclude/plat/common/				\
				-I${RK_PLAT_COMMON}/                            \
				-I${RK_PLAT_COMMON}/include/                    \
				-I${RK_PLAT_COMMON}/pmusram                     \
				-I${RK_PLAT_COMMON}/drivers/pmu/                \
				-I${RK_PLAT_COMMON}/drivers/parameter/		\
				-I${RK_PLAT_COMMON}/drivers/sram/               \
				-I${RK_PLAT_COMMON}/drivers/fiq/		\
				-I${RK_PLAT_SOC}/				\
				-I${RK_PLAT_SOC}/drivers/pmu/			\
				-I${RK_PLAT_SOC}/drivers/soc/			\
				-I${RK_PLAT_SOC_PRT}/				\
				-I${RK_PLAT_SOC_PRT}/include


RK_GIC_SOURCES		:=	drivers/arm/gic/common/gic_common.c		\
				drivers/arm/gic/v2/gicv2_main.c			\
				drivers/arm/gic/v2/gicv2_helpers.c		\
				plat/common/plat_gicv2.c			\
				${RK_PLAT}/common/rockchip_gicv2.c

PLAT_BL_COMMON_SOURCES	:=	lib/aarch64/xlat_tables.c			\
				plat/common/aarch64/plat_common.c		\
				plat/common/aarch64/plat_psci_common.c

BL31_SOURCES		+=	${RK_GIC_SOURCES}				\
				drivers/arm/cci/cci.c				\
				drivers/console/console.S			\
				drivers/ti/uart/16550_console.S			\
				drivers/delay_timer/delay_timer.c		\
				drivers/delay_timer/generic_delay_timer.c	\
				lib/cpus/aarch64/aem_generic.S			\
				lib/cpus/aarch64/cortex_a53.S			\
				plat/common/aarch64/platform_mp_stack.S		\
				plat/rockchip/common/rockchip_exceptions.c	\
				${RK_PLAT_COMMON}/drivers/parameter/ddr_parameter.c	\
				${RK_PLAT_COMMON}/aarch64/plat_helpers.S	\
				${RK_PLAT_COMMON}/bl31_plat_setup.c		\
				${RK_PLAT_COMMON}/pmusram/pmu_sram.c            \
				${RK_PLAT_COMMON}/pmusram/pmu_sram_cpus_on.S	\
				${RK_PLAT_COMMON}/plat_pm.c			\
				${RK_PLAT_COMMON}/plat_topology.c		\
				${RK_PLAT_COMMON}/aarch64/platform_common.c	\
				${RK_PLAT_COMMON}/rockchip_sip_svc.c		\
				${RK_PLAT_COMMON}/drivers/sram/sram.c		\
				${RK_PLAT_COMMON}/drivers/fiq/fiq_dfs.c		\
				${RK_PLAT_SOC}/drivers/soc/soc.c		\
				${RK_PLAT_SOC_PRT}/plat_sip_calls.c		\
				${RK_PLAT_SOC_PRT}/drivers/pmu/pmu.c


ROCKCHIP_PRT_INCLUDES	:=	-I${RK_PLAT_COMMON}/drivers/efuse/		\
				-I${RK_PLAT_SOC_PRT}/drivers/pwm/		\
				-I${RK_PLAT_SOC_PRT}/drivers/dram/		\
				-I${RK_PLAT_SOC_PRT}/drivers/monitor/

PLAT_INCLUDES		+=	${ROCKCHIP_PRT_INCLUDES}

ROCKCHIP_PRT		:=	${RK_PLAT_COMMON}/drivers/efuse/efuse.c		\
                    		${RK_PLAT_SOC_PRT}/drivers/pwm/pwm_remotectl.c	\
				${RK_PLAT_SOC_PRT}/drivers/dram/dfs.c		\
				${RK_PLAT_SOC_PRT}/drivers/dram/dram.c		\
				${RK_PLAT_SOC_PRT}/drivers/dram/dram_spec_timing.c	\
				${RK_PLAT_SOC_PRT}/drivers/monitor/monitor.c

BL31_SOURCES    += ${ROCKCHIP_PRT}

ENABLE_PLAT_COMPAT 	:=      0

$(eval $(call add_define,PLAT_EXTRA_LD_SCRIPT))
$(eval $(call add_define,PLAT_SKIP_OPTEE_S_EL1_INT_REGISTER))
