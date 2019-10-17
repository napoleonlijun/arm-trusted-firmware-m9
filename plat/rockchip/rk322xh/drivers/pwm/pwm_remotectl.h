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

#ifndef __PWM_REMOTECTL_H__
#define __PWM_REMOTECTL_H__

#include <plat_private.h>

#define PWM_INTSTS			0x0040
#define PWM_INTEN			0x0044
#define PWM_REG_HPR(n)			(0x4 + 0x10 * (n))
#define PWM_REG_LPR(n)			(0x8 + 0x10 * (n))
#define PWM_REG_CTRL(n)			(0xc + 0x10 * (n))
#define PWM_CH_INT(n)			BIT(n)
#define PWM_CH_POL(n)			BIT((n) + 8)

/*REG_CTRL bits definitions*/
#define PWM_ENABLE			(1 << 0)
#define PWM_DISABLE			(0 << 0)

/*operation mode*/
#define PWM_MODE_CAPTURE		(0x02 << 1)
#define PWM_CLK_FREQ_MSK		0xFF008DFF
#define PWM_CLK_SCALE_1MHZ		0x000C0200

#define USER_KEY_CODE_BIT_NUMS		16

/* must more then 9ms at least */
#define TIMEOUT_10MS			(10000)

#define PWM_PULSE_PRE_MIN		4000
#define PWM_PULSE_PRE_MAX		5000

#define PWM_PULSE_BIT0_MIN		480
#define PWM_PULSE_BIT0_MAX		700

#define PWM_PULSE_BIT1_MIN		1300
#define PWM_PULSE_BIT1_MAX		2000

#define MAX_PWRKEY_NUMS			20

#define PWM_PULSE_1			(0x1)
#define PWM_PULSE_0			(0x0)
#define SYSTEM_CNTPCT_PER_US		24

typedef enum RMC_STATE {
	RMC_IDLE,
	RMC_PRELOAD,
	RMC_USERCODE,
	RMC_GETDATA,
	RMC_SEQUENCE,
	RMC_PWRKEY,
} RMC_STATE_t;

__sramfunc int pwm_remotectl_wakeup(void);
__sramfunc void pwm_remotectl_prepare(void);
int pwm_remotectl_sip_handler(uint32_t param, uint32_t data);

#endif
