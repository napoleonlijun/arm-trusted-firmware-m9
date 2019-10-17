/*
 * Copyright (c) 2017, ARM Limited and Contributors. All rights reserved.
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

#include <assert.h>
#include <console.h>
#include <debug.h>
#include <delay_timer.h>
#include <errno.h>
#include <mmio.h>
#include <pm_config.h>
#include <pmu.h>
#include <platform.h>
#include <platform_def.h>
#include <plat_private.h>
#include <rk3399_def.h>
#include <rockchip_sip_svc.h>
#include <soc.h>
#include <uart_16550.h>

#define PWM0_IOMUX_PWM_EN		(1 << 0)
#define PWM1_IOMUX_PWM_EN		(1 << 1)
#define PWM2_IOMUX_PWM_EN		(1 << 2)
#define PWM3_IOMUX_PWM_EN		(1 << 3)

struct pwm_data_s {
	uint32_t iomux_bitmask;
	uint32_t enable_bitmask;
};

struct uart_debug {
	uint32_t uart_dll;
	uint32_t uart_dlh;
	uint32_t uart_ier;
	uint32_t uart_fcr;
	uint32_t uart_mcr;
	uint32_t uart_lcr;
};

static struct pwm_data_s pwm_data;
static struct uart_debug debug_port_save;
static uint32_t suspend_mode;
static uint32_t wakeup_sources;
static uint32_t pwm_regulators;
static uint32_t gpio_contrl[10];
static uint32_t suspend_debug_enable;
static uint32_t virtual_poweroff_en;
static uint32_t	suspend_apios;
static uint32_t iomux_status[12];
static uint32_t pull_mode_status[12];
static uint32_t gpio_direction[3];
static uint32_t gpio_2_4_clk_gate;

int suspend_mode_handler(uint64_t mode_id, uint64_t config1, uint64_t config2)
{
	switch (mode_id) {
	case SUSPEND_MODE_CONFIG:
		suspend_mode = config1;
		return 0;

	case WKUP_SOURCE_CONFIG:
		wakeup_sources = config1;
		return 0;

	case PWM_REGULATOR_CONFIG:
		pwm_regulators = config1;
		return 0;

	case GPIO_POWER_CONFIG:
		gpio_contrl[config1] = config2;
		return 0;

	case SUSPEND_DEBUG_ENABLE:
		suspend_debug_enable = config1;
		return 0;

	case APIOS_SUSPEND_CONFIG:
		suspend_apios = config1;
		return 0;

	case VIRTUAL_POWEROFF:
		virtual_poweroff_en = config1;
		return 0;

	default:
		ERROR("%s: unhandled sip (0x%lx)\n", __func__, mode_id);
		return -1;
	}
}

static void suspend_apios_voltage_domain(void)
{
	int i;

	if (!suspend_apios)
	return;

	/* save gpio2 ~ gpio4 iomux and pull mode */
	for (i = 0; i < 12; i++) {
		iomux_status[i] = mmio_read_32(GRF_BASE +
					       GRF_GPIO2A_IOMUX + i * 4);
		pull_mode_status[i] = mmio_read_32(GRF_BASE +
						   GRF_GPIO2A_P + i * 4);
	}

	/* store gpio2 ~ gpio4 clock gate state */
	gpio_2_4_clk_gate = (mmio_read_32(CRU_BASE + CRU_CLKGATE_CON(31)) >>
			     PCLK_GPIO2_GATE_SHIFT) & 0x07;

	/* enable gpio2 ~ gpio4 clock gate */
	mmio_write_32(CRU_BASE + CRU_CLKGATE_CON(31),
	BITS_WITH_WMASK(0, 0x07, PCLK_GPIO2_GATE_SHIFT));

	/* save gpio2 ~ gpio4 direction */
	gpio_direction[0] = mmio_read_32(GPIO2_BASE + GPIO_SWPORT_DDR);
	gpio_direction[1] = mmio_read_32(GPIO3_BASE + GPIO_SWPORT_DDR);
	gpio_direction[2] = mmio_read_32(GPIO4_BASE + GPIO_SWPORT_DDR);

	/* apio1 charge gpio3a0 ~ gpio3c7 */
	if (suspend_apios & RKPM_APIO1_SUSPEND) {
		/* set gpio3a0 ~ gpio3c7 iomux to gpio */
		mmio_write_32(GRF_BASE + GRF_GPIO3A_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);
		mmio_write_32(GRF_BASE + GRF_GPIO3B_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);
		mmio_write_32(GRF_BASE + GRF_GPIO3C_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);

		/* set gpio3a0 ~ gpio3c7 pull mode to pull none */
		mmio_write_32(GRF_BASE + GRF_GPIO3A_P, REG_SOC_WMSK | 0);
		mmio_write_32(GRF_BASE + GRF_GPIO3B_P, REG_SOC_WMSK | 0);
		mmio_write_32(GRF_BASE + GRF_GPIO3C_P, REG_SOC_WMSK | 0);

		/* set gpio3a0 ~ gpio3c7 to input */
		mmio_clrbits_32(GPIO3_BASE + GPIO_SWPORT_DDR, 0x00ffffff);
	}

	/* apio2 charge gpio2a0 ~ gpio2b4 */
	if (suspend_apios & RKPM_APIO2_SUSPEND) {
		/* set gpio2a0 ~ gpio2b4 iomux to gpio */
		mmio_write_32(GRF_BASE + GRF_GPIO2A_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);
		mmio_write_32(GRF_BASE + GRF_GPIO2B_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);

		/* set gpio2a0 ~ gpio2b4 pull mode to pull none */
		mmio_write_32(GRF_BASE + GRF_GPIO2A_P, REG_SOC_WMSK | 0);
		mmio_write_32(GRF_BASE + GRF_GPIO2B_P, REG_SOC_WMSK | 0);

		/* set gpio2a0 ~ gpio2b4 to input */
		mmio_clrbits_32(GPIO2_BASE + GPIO_SWPORT_DDR, 0x00001fff);
	}

	/* apio3 charge gpio2c0 ~ gpio2d4*/
	if (suspend_apios & RKPM_APIO3_SUSPEND) {
		/* set gpio2a0 ~ gpio2b4 iomux to gpio */
		mmio_write_32(GRF_BASE + GRF_GPIO2C_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);
		mmio_write_32(GRF_BASE + GRF_GPIO2D_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);

		/* set gpio2c0 ~ gpio2d4 pull mode to pull none */
		mmio_write_32(GRF_BASE + GRF_GPIO2C_P, REG_SOC_WMSK | 0);
		mmio_write_32(GRF_BASE + GRF_GPIO2D_P, REG_SOC_WMSK | 0);

		/* set gpio2c0 ~ gpio2d4 to input */
		mmio_clrbits_32(GPIO2_BASE + GPIO_SWPORT_DDR, 0x1fff0000);
	}

	/* apio4 charge gpio4c0 ~ gpio4c7, gpio4d0 ~ gpio4d6 */
	if (suspend_apios & RKPM_APIO4_SUSPEND) {
		mmio_write_32(GRF_BASE + GRF_GPIO2D_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);

		/* set gpio2c0 ~ gpio2d4 pull mode to pull none */
		mmio_write_32(GRF_BASE + GRF_GPIO2C_P, REG_SOC_WMSK | 0);
		mmio_write_32(GRF_BASE + GRF_GPIO2D_P, REG_SOC_WMSK | 0);

		/* set gpio2c0 ~ gpio2d4 to input */
		mmio_clrbits_32(GPIO2_BASE + GPIO_SWPORT_DDR, 0x1fff0000);
       }

	/* apio4 charge gpio4c0 ~ gpio4c7, gpio4d0 ~ gpio4d6 */
	if (suspend_apios & RKPM_APIO4_SUSPEND) {
		/* set gpio4c0 ~ gpio4d6 iomux to gpio */
		mmio_write_32(GRF_BASE + GRF_GPIO4C_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);
		mmio_write_32(GRF_BASE + GRF_GPIO4D_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);

		/* set gpio4c0 ~ gpio4d6 pull mode to pull none */
		mmio_write_32(GRF_BASE + GRF_GPIO4C_P, REG_SOC_WMSK | 0);
		mmio_write_32(GRF_BASE + GRF_GPIO4D_P, REG_SOC_WMSK | 0);

		/* set gpio4c0 ~ gpio4d6 to input */
		mmio_clrbits_32(GPIO4_BASE + GPIO_SWPORT_DDR, 0x7fff0000);
	}

	/* apio5 charge gpio3d0 ~ gpio3d7, gpio4a0 ~ gpio4a7*/
	if (suspend_apios & RKPM_APIO5_SUSPEND) {
		/* set gpio3d0 ~ gpio4a7 iomux to gpio */
		mmio_write_32(GRF_BASE + GRF_GPIO3D_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);
		mmio_write_32(GRF_BASE + GRF_GPIO4A_IOMUX,
			      REG_SOC_WMSK | GRF_IOMUX_GPIO);

		/* set gpio3d0 ~ gpio4a7 pull mode to pull none */
		mmio_write_32(GRF_BASE + GRF_GPIO3D_P, REG_SOC_WMSK | 0);
		mmio_write_32(GRF_BASE + GRF_GPIO4A_P, REG_SOC_WMSK | 0);

		/* set gpio4c0 ~ gpio4d6 to input */
		mmio_clrbits_32(GPIO3_BASE + GPIO_SWPORT_DDR, 0xff000000);
		mmio_clrbits_32(GPIO4_BASE + GPIO_SWPORT_DDR, 0x000000ff);
	}
}

static void resume_apios_voltage_domain(void)
{
	int i;

	if (!suspend_apios)
		return;

	for (i = 0; i < 12; i++) {
		mmio_write_32(GRF_BASE + GRF_GPIO2A_P + i * 4,
			      REG_SOC_WMSK | pull_mode_status[i]);
		mmio_write_32(GRF_BASE + GRF_GPIO2A_IOMUX + i * 4,
			      REG_SOC_WMSK | iomux_status[i]);
	}
	/* set gpio2 ~ gpio4 direction back to store value */
	mmio_write_32(GPIO2_BASE + GPIO_SWPORT_DDR, gpio_direction[0]);
	mmio_write_32(GPIO3_BASE + GPIO_SWPORT_DDR, gpio_direction[1]);
	mmio_write_32(GPIO4_BASE + GPIO_SWPORT_DDR, gpio_direction[2]);

	/* set gpio2 ~ gpio4 clock gate back to store value */
	mmio_write_32(CRU_BASE + CRU_CLKGATE_CON(31),
		      BITS_WITH_WMASK(gpio_2_4_clk_gate, 0x07,
		      PCLK_GPIO2_GATE_SHIFT));
}

/*
  * Disable the PWMs.
  */
static void pwm_regulator_suspend(void)
{
	uint32_t i, val;

	pwm_data.iomux_bitmask = 0;

	/* Save PWMs pinmux and change PWMs pinmux to GPIOs */
	val = mmio_read_32(GRF_BASE + GRF_GPIO4C_IOMUX);
	if (pwm_regulators & PWM0_REGULATOR_EN) {
		pwm_data.iomux_bitmask |= PWM0_IOMUX_PWM_EN;
		val = BITS_WITH_WMASK(GRF_IOMUX_GPIO, GRF_IOMUX_2BIT_MASK,
				    GRF_GPIO4C2_IOMUX_SHIFT);
		mmio_write_32(GRF_BASE + GRF_GPIO4C_IOMUX, val);
	}

	val = mmio_read_32(GRF_BASE + GRF_GPIO4C_IOMUX);
	if (pwm_regulators & PWM1_REGULATOR_EN) {
		pwm_data.iomux_bitmask |= PWM1_IOMUX_PWM_EN;
		val = BITS_WITH_WMASK(GRF_IOMUX_GPIO, GRF_IOMUX_2BIT_MASK,
				    GRF_GPIO4C6_IOMUX_SHIFT);
		mmio_write_32(GRF_BASE + GRF_GPIO4C_IOMUX, val);
	}

	val = mmio_read_32(PMUGRF_BASE + PMUGRF_GPIO1C_IOMUX);
	if (pwm_regulators & PWM2_REGULATOR_EN) {
		pwm_data.iomux_bitmask |= PWM2_IOMUX_PWM_EN;
		val = BITS_WITH_WMASK(GRF_IOMUX_GPIO, GRF_IOMUX_2BIT_MASK,
				    PMUGRF_GPIO1C3_IOMUX_SHIFT);
		mmio_write_32(PMUGRF_BASE + PMUGRF_GPIO1C_IOMUX, val);
	}

	val = mmio_read_32(PMUGRF_BASE + PMUGRF_GPIO0A_IOMUX);
	if (pwm_regulators & PWM3A_REGULATOR_EN) {
		pwm_data.iomux_bitmask |= PWM3_IOMUX_PWM_EN;
		val = BITS_WITH_WMASK(GRF_IOMUX_GPIO, GRF_IOMUX_2BIT_MASK,
				    PMUGRF_GPIO0A6_IOMUX_SHIFT);
		mmio_write_32(PMUGRF_BASE + PMUGRF_GPIO0A_IOMUX, val);
	}

	/* Disable the pwm channel */
	pwm_data.enable_bitmask = 0;
	for (i = 0; i < 4; i++) {
		if ((pwm_regulators & BIT(i)) == 0)
			continue;
		val = mmio_read_32(PWM_BASE + PWM_CTRL(i));
		if ((val & PWM_ENABLE) != PWM_ENABLE)
			continue;
		pwm_data.enable_bitmask |= (1 << i);
		mmio_write_32(PWM_BASE + PWM_CTRL(i), val & ~PWM_ENABLE);
	}
}

/*
  * Enable the PWMs.
  */
static void pwm_regulator_resume(void)
{
	uint32_t i, val;

	for (i = 0; i < 4; i++) {
		val = mmio_read_32(PWM_BASE + PWM_CTRL(i));
		if (!(pwm_data.enable_bitmask & (1 << i)))
			continue;
		mmio_write_32(PWM_BASE + PWM_CTRL(i), val | PWM_ENABLE);
	}

	/* Restore all IOMUXes */
	if (pwm_data.iomux_bitmask & PWM3_IOMUX_PWM_EN) {
		val = BITS_WITH_WMASK(PMUGRF_GPIO0A6_IOMUX_PWM,
				    GRF_IOMUX_2BIT_MASK,
				    PMUGRF_GPIO0A6_IOMUX_SHIFT);
		mmio_write_32(PMUGRF_BASE + PMUGRF_GPIO0A_IOMUX, val);
	}

	if (pwm_data.iomux_bitmask & PWM2_IOMUX_PWM_EN) {
		val = BITS_WITH_WMASK(PMUGRF_GPIO1C3_IOMUX_PWM,
				    GRF_IOMUX_2BIT_MASK,
				    PMUGRF_GPIO1C3_IOMUX_SHIFT);
		mmio_write_32(PMUGRF_BASE + PMUGRF_GPIO1C_IOMUX, val);
	}

	if (pwm_data.iomux_bitmask & PWM1_IOMUX_PWM_EN) {
		val = BITS_WITH_WMASK(GRF_GPIO4C6_IOMUX_PWM,
				    GRF_IOMUX_2BIT_MASK,
				    GRF_GPIO4C6_IOMUX_SHIFT);
		mmio_write_32(GRF_BASE + GRF_GPIO4C_IOMUX, val);
	}

	if (pwm_data.iomux_bitmask & PWM0_IOMUX_PWM_EN) {
		val = BITS_WITH_WMASK(GRF_GPIO4C2_IOMUX_PWM,
				    GRF_IOMUX_2BIT_MASK,
				    GRF_GPIO4C2_IOMUX_SHIFT);
		mmio_write_32(GRF_BASE + GRF_GPIO4C_IOMUX, val);
	}
}

static void gpio_set_low(uint32_t gpio_control)
{
	uint32_t gpio0_dr, gpio0_ddr;
	uint32_t gpio1_dr, gpio1_ddr;

	if (gpio_control != PM_INVALID_GPIO) {
		if (gpio_control >> 5) {
			gpio1_dr = mmio_read_32(GPIO1_BASE + GPIO_SWPORT_DR);
			gpio1_ddr = mmio_read_32(GPIO1_BASE + GPIO_SWPORT_DDR);
			mmio_write_32(GPIO1_BASE + GPIO_SWPORT_DR,
					  gpio1_dr & (~(BIT(gpio_control & 0x1f))));
			mmio_write_32(GPIO1_BASE + GPIO_SWPORT_DDR,
					  gpio1_ddr & (~(BIT(gpio_control & 0x1f))));
		} else {
			gpio0_dr = mmio_read_32(GPIO0_BASE + GPIO_SWPORT_DR);
			gpio0_ddr = mmio_read_32(GPIO0_BASE + GPIO_SWPORT_DDR);
			mmio_write_32(GPIO0_BASE + GPIO_SWPORT_DR,
					  gpio0_dr & (~(BIT(gpio_control & 0x1f))));
			mmio_write_32(GPIO0_BASE + GPIO_SWPORT_DDR,
					  gpio0_ddr & (~(BIT(gpio_control & 0x1f))));
		}
	}
}

static void gpio_set_high(uint32_t gpio_control)
{
	uint32_t gpio0_dr, gpio0_ddr;
	uint32_t gpio1_dr, gpio1_ddr;

	if (gpio_control != PM_INVALID_GPIO) {
		if (gpio_control >> 5) {
			gpio1_dr = mmio_read_32(GPIO1_BASE + GPIO_SWPORT_DR);
			gpio1_ddr = mmio_read_32(GPIO1_BASE + GPIO_SWPORT_DDR);
			mmio_write_32(GPIO1_BASE + GPIO_SWPORT_DR,
				      gpio1_dr | BIT(gpio_control & 0x1f));
			mmio_write_32(GPIO1_BASE + GPIO_SWPORT_DDR,
				      gpio1_ddr | BIT(gpio_control & 0x1f));
		} else {
			gpio0_dr = mmio_read_32(GPIO0_BASE + GPIO_SWPORT_DR);
			gpio0_ddr = mmio_read_32(GPIO0_BASE + GPIO_SWPORT_DDR);
			mmio_write_32(GPIO0_BASE + GPIO_SWPORT_DR,
				      gpio0_dr | BIT(gpio_control & 0x1f));
			mmio_write_32(GPIO0_BASE + GPIO_SWPORT_DDR,
				      gpio0_ddr | BIT(gpio_control & 0x1f));
		}
	}
}

static void gpio_get_info(uint32_t gpio_control)
{
	if (gpio_control != PM_INVALID_GPIO) {
		if (gpio_control >> 5) {
			if ((gpio_control & 0x1f) / 24)
				INFO("\tGPIO1_D%d\n", (gpio_control & 0x1f) % 24);
			else if ((gpio_control & 0x1f) / 16)
				INFO("\tGPIO1_C%d\n", (gpio_control & 0x1f) % 16);
			else if ((gpio_control & 0x1f) / 8)
				INFO("\tGPIO1_B%d\n", (gpio_control & 0x1f) % 8);
			else if ((gpio_control & 0x1f))
				INFO("\tGPIO1_A%d\n", (gpio_control & 0x1f));
		} else {
			if ((gpio_control & 0x1f) / 24)
				INFO("\tGPIO0_D%d\n", (gpio_control & 0x1f) % 24);
			else if ((gpio_control & 0x1f) / 16)
				INFO("\tGPIO0_C%d\n", (gpio_control & 0x1f) % 16);
			else if ((gpio_control & 0x1f) / 8)
				INFO("\tGPIO0_B%d\n", (gpio_control & 0x1f) % 8);
			else if ((gpio_control & 0x1f))
				INFO("\tGPIO0_A%d\n", (gpio_control & 0x1f));
		}
	}
}
static void gpio_power_resume(void)
{
	uint32_t i = 0;

	for (i = 0; i < 10; i++) {
		if (gpio_contrl[i] == PM_INVALID_GPIO)
			return;
		gpio_set_low(gpio_contrl[i]);
	}
}

static void gpio_power_suspend(void)
{
	uint32_t i = 0;

	for (i = 0; i < 10; i++) {
		if (gpio_contrl[i] == PM_INVALID_GPIO)
			return;
		gpio_set_high(gpio_contrl[i]);
	}
}

static void uart_debug_save(void)
{
	debug_port_save.uart_lcr = mmio_read_32(PLAT_RK_UART_BASE + UARTLCR);
	debug_port_save.uart_ier = mmio_read_32(PLAT_RK_UART_BASE + UARTIER);
	debug_port_save.uart_mcr = mmio_read_32(PLAT_RK_UART_BASE + UARTMCR);
	mmio_write_32(PLAT_RK_UART_BASE + UARTLCR,
		      debug_port_save.uart_lcr | UARTLCR_DLAB);
	debug_port_save.uart_dll = mmio_read_32(PLAT_RK_UART_BASE + UARTDLL);
	debug_port_save.uart_dlh = mmio_read_32(PLAT_RK_UART_BASE + UARTDLLM);
	mmio_write_32(PLAT_RK_UART_BASE + UARTLCR, debug_port_save.uart_lcr);
}

void uart_debug_restore(void)
{
	uint32_t uart_lcr;

	mmio_write_32(PLAT_RK_UART_BASE + UARTSRR,
		      XMIT_FIFO_RESET | RCVR_FIFO_RESET | UART_RESET);

	uart_lcr = mmio_read_32(PLAT_RK_UART_BASE + UARTLCR);
	mmio_write_32(PLAT_RK_UART_BASE + UARTMCR, DIAGNOSTIC_MODE);
	mmio_write_32(PLAT_RK_UART_BASE + UARTLCR, uart_lcr | UARTLCR_DLAB);
	mmio_write_32(PLAT_RK_UART_BASE + UARTDLL, debug_port_save.uart_dll);
	mmio_write_32(PLAT_RK_UART_BASE + UARTDLLM, debug_port_save.uart_dlh);
	mmio_write_32(PLAT_RK_UART_BASE + UARTLCR, debug_port_save.uart_lcr);
	mmio_write_32(PLAT_RK_UART_BASE + UARTIER, debug_port_save.uart_ier);
	mmio_write_32(PLAT_RK_UART_BASE + UARTFCR, UARTFCR_FIFOEN);
	mmio_write_32(PLAT_RK_UART_BASE + UARTMCR, debug_port_save.uart_mcr);
}

static void get_gpio_power_info(void)
{
	uint32_t i = 0;

	INFO("GPIO POWER INFO:\n");
	if (gpio_contrl[0] == PM_INVALID_GPIO) {
		INFO("\tnot config\n");
		return;
	}

	for (i = 0; i < 10; i++) {
		if (gpio_contrl[i] == PM_INVALID_GPIO)
			return;
		gpio_get_info(gpio_contrl[i]);
	}
}

static void pmu_sleep_config(void)
{
	uint32_t config = 0;

	if (suspend_mode == 0)
		return;
	mmio_write_32(PMUGRF_BASE + PMUGRF_GPIO1A_IOMUX,
				  BIT_WITH_WMSK(AP_PWROFF));

	config |= BIT(PMU_PWR_MODE_EN) |
				BIT(PMU_CLK_CENTER_SRC_GATE_EN) |
				BIT(PMU_ALIVE_USE_LF) |
				BIT(PMU_PMU_USE_LF);

	if (suspend_mode & RKPM_SLP_AP_PWROFF)
		config |= BIT(PMU_POWER_OFF_REQ_CFG);

	if (suspend_mode & RKPM_SLP_ARMPD)
		config |= BIT(PMU_CPU0_PD_EN) |
		       BIT(PMU_L2_FLUSH_EN) |
		       BIT(PMU_L2_IDLE_EN) |
		       BIT(PMU_SCU_PD_EN) |
		       BIT(PMU_CCI_PD_EN) |
		       BIT(PMU_CLK_CORE_SRC_GATE_EN);

	if (suspend_mode & RKPM_SLP_DDR_RET)
		config |= BIT(PMU_SREF0_ENTER_EN) |
			BIT(PMU_SREF1_ENTER_EN) |
			BIT(PMU_DDRC0_GATING_EN) |
			BIT(PMU_DDRC1_GATING_EN) |
			BIT(PMU_DDRIO0_RET_EN) |
			BIT(PMU_DDRIO1_RET_EN) |
			BIT(PMU_DDRIO_RET_HW_DE_REQ);

	if (suspend_mode & RKPM_SLP_CENTER_PD) {
		config |= BIT(PMU_CENTER_PD_EN);
		config &= ~(BIT(PMU_DDRIO_RET_HW_DE_REQ));
	}

	if (suspend_mode & RKPM_SLP_PLLPD)
		config |= BIT(PMU_PLL_PD_EN);

	if (suspend_mode & RKPM_SLP_OSC_DIS)
		config |= BIT(PMU_OSC_DIS);

	if ((!virtual_poweroff_en) && (wakeup_sources & BIT(PMU_PWM_WKUP_EN))) {
		config &= ~(BIT(PMU_OSC_DIS) | BIT(PMU_PMU_USE_LF));
		mmio_write_32(PMU_BASE + PMU_PLL_CON, PLL_PD_HW & (~(BIT(PPLL_ID))));
		/*
		 * From limited testing, need PMU stable >= 2ms, but go overkill
		 * and choose 30 ms to match testing on past SoCs.	Also let
		 * OSC have 30 ms for stabilization.
		 */
		mmio_write_32(PMU_BASE + PMU_STABLE_CNT, CYCL_24M_CNT_MS(3));
		mmio_write_32(PMU_BASE + PMU_OSC_CNT, CYCL_24M_CNT_MS(3));
	}

	if (wakeup_sources & BIT(PMU_USB_LINESTATE_WAKEUP_EN))
		config &= ~(BIT(PMU_ALIVE_USE_LF));

	mmio_write_32(PMU_BASE + PMU_WKUP_CFG4, wakeup_sources);
	mmio_write_32(PMU_BASE + PMU_PWRMODE_CON, config);
	INFO("PMU_MODE_CONG: 0x%x\n", config);
}

static void get_suspend_mode_config(void)
{
	INFO("sleep mode config[0x%x]:\n", suspend_mode);
	if (suspend_mode & RKPM_SLP_AP_PWROFF)
		INFO("\tAP_PWROFF\n");
	if (suspend_mode & RKPM_SLP_ARMPD)
		INFO("\tSLP_ARMPD\n");
	if (suspend_mode & RKPM_SLP_PLLPD)
		INFO("\tSLP_PLLPD\n");
	if (suspend_mode & RKPM_SLP_DDR_RET)
		INFO("\tDDR_RET\n");
	if (suspend_mode & RKPM_SLP_CENTER_PD)
		INFO("\tSLP_CENTER_PD\n");
	if (suspend_mode & RKPM_SLP_OSC_DIS)
		INFO("\tOSC_DIS\n");
}

static void get_wakup_source_config(void)
{
	uint32_t wkup_status = wakeup_sources;

	INFO("wakeup source config[0x%x]:\n", wakeup_sources);
	if (!wkup_status) {
		INFO("\tnot config\n");
		return;
	}
	if (wkup_status & BIT(0))
		INFO("\tCLUSTER L interrupt can wakeup system\n");
	if (wkup_status & BIT(1))
		INFO("\tCLUSTER B interrupt can wakeup system\n");
	if (wkup_status & BIT(2))
		INFO("\tGPIO interrupt can wakeup system\n");
	if (wkup_status & BIT(3))
		INFO("\tSDIO interrupt can wakeup system\n");
	if (wkup_status & BIT(4))
		INFO("\tSDMMC interrupt can wakeup system\n");
	if (wkup_status & BIT(6))
		INFO("\ttimer interrupt can wakeup system\n");
	if (wkup_status & BIT(7))
		INFO("\tUSBDEV detect can wakeup system\n");
	if (wkup_status & BIT(8))
		INFO("\tM0 software interrupt can wakeup system\n");
	if (wkup_status & BIT(9))
		INFO("\tM0 wdt interrupt can wakeup system\n");
	if (wkup_status & BIT(10))
		INFO("\ttimer out interrupt can wakeup system\n");
	if (wkup_status & BIT(11))
		INFO("\tPWM interrupt can wakeup system\n");
	if (wkup_status & BIT(13))
		INFO("\tpcie interrupt can wakeup system\n");
}

static void report_wakeup_source(void)
{
	uint32_t wkup_status;
	uint32_t gpio2_4_clk_gate;

	/* store gpio2 ~ gpio4 clock gate state */
	gpio2_4_clk_gate = (mmio_read_32(CRU_BASE + CRU_CLKGATE_CON(31)) >>
			    PCLK_GPIO2_GATE_SHIFT) & 0x07;

	wkup_status =  mmio_read_32(PMU_BASE + PMU_WAKEUP_STATUS);
	INFO("RK3399 the wake up information:\n");
	INFO("wake up status: 0x%x\n",
	     mmio_read_32(PMU_BASE + PMU_WAKEUP_STATUS));
	if (wkup_status & BIT(0))
		INFO("\tCLUSTER L interrupt wakeup\n");
	if (wkup_status & BIT(1))
		INFO("\tCLUSTER B interrupt wakeup\n");
	if (wkup_status & BIT(2)) {
		INFO("\tGPIO interrupt wakeup\n");
		mmio_write_32(CRU_BASE + CRU_CLKGATE_CON(31),
			      BITS_WITH_WMASK(0, 0x07, PCLK_GPIO2_GATE_SHIFT));
		INFO("\tGPIO0: 0x%x\n", mmio_read_32(GPIO0_BASE + 0x040));
		INFO("\tGPIO1: 0x%x\n", mmio_read_32(GPIO1_BASE + 0x040));
		INFO("\tGPIO2: 0x%x\n", mmio_read_32(GPIO2_BASE + 0x040));
		INFO("\tGPIO3: 0x%x\n", mmio_read_32(GPIO3_BASE + 0x040));
		INFO("\tGPIO4: 0x%x\n", mmio_read_32(GPIO4_BASE + 0x040));
		/* set gpio2 ~ gpio4 clock gate back to store value */
		mmio_write_32(CRU_BASE + CRU_CLKGATE_CON(31),
			      BITS_WITH_WMASK(gpio2_4_clk_gate, 0x07,
					      PCLK_GPIO2_GATE_SHIFT));

	}
	if (wkup_status & BIT(3))
		INFO("\tSDIO interrupt wakeup\n");
	if (wkup_status & BIT(4))
		INFO("\tSDMMC interrupt wakeup\n");
	if (wkup_status & BIT(6))
		INFO("\tTIMER interrupt wakeup\n");
	if (wkup_status & BIT(7))
		INFO("\tUSBDEV detect wakeup\n");
	if (wkup_status & BIT(8))
		INFO("\tM0 software interrupt wakeup\n");
	if (wkup_status & BIT(9))
		INFO("\tM0 wdt interrupt wakeup\n");
	if (wkup_status & BIT(10))
		INFO("\ttimer out interrupt wakeup\n");
	if (wkup_status & BIT(11))
		INFO("\tPWM interrupt wakeup\n");
	if (wkup_status & BIT(13))
		INFO("\tPCIE interrupt wakeup\n");
}

static void get_pwm_regulator_info(void)
{
	INFO("PWM CONFIG[0x%x]:\n", pwm_regulators);

	if (!pwm_regulators) {
		INFO("\tnot config\n");
		return;
	}
	if (pwm_regulators & PWM0_REGULATOR_EN)
		INFO("\tPWM: PWM0_REGULATOR_EN\n");
	if (pwm_regulators & PWM1_REGULATOR_EN)
		INFO("\tPWM: PWM1_REGULATOR_EN\n");
	if (pwm_regulators & PWM2_REGULATOR_EN)
		INFO("\tPWM: PWM2D_REGULATOR_EN\n");
	if (pwm_regulators & PWM3A_REGULATOR_EN)
		INFO("\tPWM: PWM3A_REGULATOR_EN\n");
	if (pwm_regulators & PWM3B_REGULATOR_EN)
		INFO("\tPWM: PWM3B_REGULATOR_EN\n");
}

static void get_apios_voltage_domain_info(void)
{
	INFO("APIOS info[0x%x]:\n", suspend_apios);

	if (!suspend_apios) {
		INFO("\tnot config\n");
		return;
	}
	if (suspend_apios & RKPM_APIO0_SUSPEND)
		INFO("\tAPIOS: APIO0 suspend\n");
	if (suspend_apios & RKPM_APIO1_SUSPEND)
		INFO("\tAPIOS: APIO1 suspend\n");
	if (suspend_apios & RKPM_APIO2_SUSPEND)
		INFO("\tAPIOS: APIO2 suspend\n");
	if (suspend_apios & RKPM_APIO3_SUSPEND)
		INFO("\tAPIOS: APIO3 suspend\n");
	if (suspend_apios & RKPM_APIO4_SUSPEND)
		INFO("\tAPIOS: APIO4 suspend\n");
	if (suspend_apios & RKPM_APIO5_SUSPEND)
		INFO("\tAPIOS: APIO5 suspend\n");
}

/*the info about the susped mode, wake up source, the gpio pin control.*/
static void pm_debug_info(void)
{
	get_suspend_mode_config();
	get_wakup_source_config();
	get_pwm_regulator_info();
	get_apios_voltage_domain_info();
	get_gpio_power_info();
}

void pmu_suspend_power(void)
{
	if (suspend_debug_enable) {
		uart_debug_save();
		console_init(PLAT_RK_UART_BASE, PLAT_RK_UART_CLOCK,
			     PLAT_RK_UART_BAUDRATE);
	}
	pm_debug_info();
	pmu_sleep_config();

	/*
	* Disabling PLLs/PWM/DVFS is approaching WFI which is
	* the last steps in suspend.
	*/
	disable_dvfs_plls();
	pwm_regulator_suspend();
	disable_nodvfs_plls();
	suspend_apios_voltage_domain();
	gpio_power_suspend();
}

void pmu_resume_power(void)
{
	gpio_power_resume();
	resume_apios_voltage_domain();
	enable_nodvfs_plls();
	pwm_regulator_resume();
	/* PWM regulators take time to come up; give 300us to be safe. */
	udelay(300);
	enable_dvfs_plls();
	report_wakeup_source();
	if (suspend_debug_enable) {
		console_uninit();
		uart_debug_restore();
	}
}
