/*
 * Copyright (C) 2016 ROCKCHIP, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <arch_helpers.h>
#include <console.h>
#include <debug.h>
#include <pwm_remotectl.h>
#include <rk3328_def.h>
#include <smcc_helpers.h>
#include <soc.h>

#define	REMOTECTL_SET_IRQ		0xf0
#define REMOTECTL_SET_PWM_CH		0xf1
#define REMOTECTL_SET_PWRKEY		0xf2
#define REMOTECTL_GET_WAKEUP_STATE	0xf3
#define REMOTECTL_ENABLE		0xf4
/* wakeup state */
#define REMOTECTL_PWRKEY_WAKEUP		0xdeadbeaf

#define USER_CODE_MSK			0xffff0000
#define USER_AND_KEY_CODE_MSK		0xffffff00

struct rkxx_remotectl_drvdata {
	/* data from NS world by SIP */
	int enable;
	int irq;
	int pwm_id;
	int key_power[MAX_PWRKEY_NUMS];

	/* self init data */
	int wakeup_state;
	int pwm_cycles_per_us;
	int state;
	int scan_data;
	uint32_t ctrl_reg;
	unsigned long early_hpr_us;
};

static __sramdata struct rkxx_remotectl_drvdata g_ddata;

static inline int is_pwm_pulse_bit1_valid(unsigned long hpr_us)
{
	return ((hpr_us >= PWM_PULSE_BIT1_MIN) &&
		(hpr_us < PWM_PULSE_BIT1_MAX));
}

static inline int is_pwm_pulse_pre_valid(unsigned long hpr_us)
{
	return ((hpr_us >= PWM_PULSE_PRE_MIN) &&
		(hpr_us < PWM_PULSE_PRE_MAX));
}

static inline int is_user_code_valid(struct rkxx_remotectl_drvdata *ddata)
{
	uint32_t i;

	for (i = 0; i < MAX_PWRKEY_NUMS; i++) {
		if ((ddata->scan_data & USER_CODE_MSK) ==
		    (ddata->key_power[i] & USER_CODE_MSK))
			return 1;
	}

	return 0;
}

static inline int is_key_code_valid(struct rkxx_remotectl_drvdata *ddata)
{
	uint32_t i;

	for (i = 0; i < MAX_PWRKEY_NUMS; i++) {
		if ((ddata->scan_data & USER_AND_KEY_CODE_MSK) ==
		    (ddata->key_power[i] & USER_AND_KEY_CODE_MSK))
			return 1;
	}

	return 0;
}

static inline int is_keycode_verify_pass(struct rkxx_remotectl_drvdata *ddata)
{
	int scan_data = ddata->scan_data;

	return (scan_data & 0xff) == ((~scan_data >> 8) & 0xff);
}

static inline uint64_t arch_counter_get_cntpct(void)
{
	uint64_t cval;

	isb();
	cval = read_cntpct_el0();

	return cval;
}

static inline uint64_t sram_get_us(uint64_t cntpct)
{
	return (arch_counter_get_cntpct() - cntpct) / SYSTEM_CNTPCT_PER_US;
}

static __sramfunc int pwm_parse_keydata(unsigned long hpr_us)
{
	struct rkxx_remotectl_drvdata *ddata = &g_ddata;
	static __sramdata int scan_data, scan_nums;

	switch (ddata->state) {
	case RMC_PRELOAD:
		if (is_pwm_pulse_pre_valid(hpr_us))
			ddata->state = RMC_USERCODE;
		else
			ddata->state = RMC_PRELOAD;

		scan_data = 0;
		scan_nums = 0;
		break;

	case RMC_USERCODE:
		if (is_pwm_pulse_bit1_valid(hpr_us))
			scan_data |= (PWM_PULSE_1 << scan_nums);
		else
			scan_data |= (PWM_PULSE_0 << scan_nums);

		scan_nums++;
		/* user code(16 bits) receive full */
		if (scan_nums >= USER_KEY_CODE_BIT_NUMS) {
			ddata->scan_data = scan_data << 16;
			if (is_user_code_valid(ddata))
				ddata->state = RMC_GETDATA;
			else
				ddata->state = RMC_PRELOAD;

			scan_data = 0;
			scan_nums = 0;
		}
		break;

	case RMC_GETDATA:
		if (is_pwm_pulse_bit1_valid(hpr_us))
			scan_data |= (PWM_PULSE_1 << scan_nums);
		else
			scan_data |= (PWM_PULSE_0 << scan_nums);

		scan_nums++;
		if (scan_nums < USER_KEY_CODE_BIT_NUMS)
			break;

		ddata->scan_data |= scan_data;
		if (is_keycode_verify_pass(ddata)) {
			if (is_key_code_valid(ddata)) {
				#if 0
				sram_putc('W');
				sram_putc('(');
				sram_printhex(ddata->scan_data);
				sram_putc(')');
				#endif
				ddata->wakeup_state = REMOTECTL_PWRKEY_WAKEUP;
				return RMC_PWRKEY;
			}
		}
		ddata->state = RMC_PRELOAD;
		scan_data = 0;
		scan_nums = 0;
		break;

	default:
		break;
	}

	return ddata->state;
}

/******************************************************************************
 *
 *				|<------ pulse 1 -------->|
 *				---------------------     -----------
 * normal pulse 1:		|                   |     |
 *			________|         s1'       |_s4'_|
 *
 *			level:	|<----- high ------>|<low>|
 *
 * <ECC bit1 pulse>
 *	high level s1' = s1 + s2 + s3 = 1690us (but s2 is lowlevel glitch)
 *	low_level  s4' = s4 = 560us
 *	early_hpr_us = s1 + s2;
 *				-----------    ------     -----------
 *				|	  |    |    |     |
 * glitch lpr pluse 1:	________|   s1    |_s2_| s3 |_s4__|
 *
*******************************************************************************/
__sramfunc int pwm_remotectl_pwrkey_wakeup(void)
{
	struct rkxx_remotectl_drvdata *ddata = &g_ddata;
	unsigned int lpr_us, hpr_us;
	int hpr_cnt, lpr_cnt, glitch_lpr2hpr_cnt;
	int key_state, val, id = ddata->pwm_id;
	uint64_t cntpct = arch_counter_get_cntpct();

	ddata->state = RMC_PRELOAD;
	ddata->early_hpr_us = 0;
	ddata->scan_data = 0;
	ddata->wakeup_state = -1;
	ddata->pwm_cycles_per_us = 1;

	while (1) {
		/* is 10ms timeout ? */
		if (sram_get_us(cntpct) >= TIMEOUT_10MS) {
			key_state = RMC_PRELOAD;
			break;
		}

		/* is interrupt ? */
		val = mmio_read_32(PWM_BASE + PWM_INTSTS);
		if ((val & PWM_CH_INT(id)) == 0)
			continue;

		/* is high and low level collect done yet ? */
		if ((val & PWM_CH_POL(id)) == 0) {
			hpr_cnt = mmio_read_32(PWM_BASE + PWM_REG_HPR(id));
			lpr_cnt = mmio_read_32(PWM_BASE + PWM_REG_LPR(id));
			lpr_us = lpr_cnt / ddata->pwm_cycles_per_us;

			if (lpr_us > PWM_PULSE_BIT0_MIN) {
				hpr_us = ddata->early_hpr_us +
					hpr_cnt / ddata->pwm_cycles_per_us;
				ddata->early_hpr_us = 0;

				/* parse hpr is enough to analize pulse */
				key_state = pwm_parse_keydata(hpr_us);
				if (key_state == RMC_PWRKEY ||
				    key_state == RMC_PRELOAD) {
					if (key_state == RMC_PRELOAD) {
						val = mmio_read_32(PWM_BASE + PWM_INTSTS);
						val |= PWM_CH_INT(id);
						mmio_write_32(PWM_BASE + PWM_INTSTS, val);
					}
					/* if powerkey wakeup, don't clear int
					 * flag to allow kernel knowns the
					 * wakeup irq and dump wakeup info.
					 */
					break;
				}
			} else {
				/* glitch lpr cnt equals to original hpr cnt */
				glitch_lpr2hpr_cnt = lpr_cnt;
				ddata->early_hpr_us +=
					(hpr_cnt + glitch_lpr2hpr_cnt)  /
						ddata->pwm_cycles_per_us;
			}
		}

		/* clear int flag */
		val = mmio_read_32(PWM_BASE + PWM_INTSTS);
		val |= PWM_CH_INT(id);
		mmio_write_32(PWM_BASE + PWM_INTSTS, val);

		/* reinit timerout value */
		cntpct = arch_counter_get_cntpct();
	}

	return !!(key_state == RMC_PWRKEY);
}

__sramfunc void pwm_remotectl_prepare(void)
{
	uint32_t val;
	struct rkxx_remotectl_drvdata *ddata = &g_ddata;

	if (!ddata->enable)
		return;

	ddata->ctrl_reg = mmio_read_32(PWM_BASE + PWM_REG_CTRL(ddata->pwm_id));

	/* prescale clock is directly used as the PWM clock source, NO div */
	val = mmio_read_32(PWM_BASE + PWM_REG_CTRL(ddata->pwm_id));
	val = (val & PWM_CLK_FREQ_MSK) | PWM_CLK_SCALE_1MHZ;
	mmio_write_32(PWM_BASE + PWM_REG_CTRL(ddata->pwm_id), val);
}

__sramfunc void pwm_remotectl_finish(void)
{
	struct rkxx_remotectl_drvdata *ddata = &g_ddata;

	if (!ddata->enable)
		return;

	mmio_write_32(PWM_BASE + PWM_REG_CTRL(ddata->pwm_id), ddata->ctrl_reg);
}

#define GIC_BASE	(0xff810000 + 0x1000)
#define NUM_INTS_PER_REG	32
#define GICD_ISPENDR(n)		(0x200 + (n) * 4)

__sramfunc int sram_gic_it_is_pending(size_t it)
{
	size_t reg_idx;
	uint32_t bit_msk, reg;

	reg_idx = it / NUM_INTS_PER_REG;
	bit_msk = 1 << (it % NUM_INTS_PER_REG);
	reg = mmio_read_32(GIC_BASE + GICD_ISPENDR(reg_idx));

	return !!(reg & bit_msk);
}

__sramfunc int pwm_remotectl_is_wakeup(void)
{
	struct rkxx_remotectl_drvdata *ddata = &g_ddata;

	if (!ddata->enable)
		return 0;

	return sram_gic_it_is_pending(ddata->irq);
}

__sramfunc int pwm_remotectl_wakeup(void)
{
	int wakeup;

	if (pwm_remotectl_is_wakeup()) {
		wakeup = pwm_remotectl_pwrkey_wakeup();
		if (!wakeup)
			return -1;
	}
	pwm_remotectl_finish();
	return 0;
}

int pwm_remotectl_sip_handler(uint32_t param, uint32_t data)
{
	struct rkxx_remotectl_drvdata *ddata = &g_ddata;
	static uint32_t num;

	switch (param) {
	case REMOTECTL_SET_IRQ:
		ddata->irq = data;
		break;
	case REMOTECTL_SET_PWM_CH:
		ddata->pwm_id = data;
		break;
	case REMOTECTL_SET_PWRKEY:
		if (num >= MAX_PWRKEY_NUMS) {
			INFO("powerkey store buffer overflow\n");
			break;
		}
		ddata->key_power[num] = data;
		num++;
		break;
	case REMOTECTL_GET_WAKEUP_STATE:
		return ddata->wakeup_state;
	case REMOTECTL_ENABLE:
		ddata->enable = data;
		break;
	default:
		return SMC_UNK;
	}

	return 0;
}
