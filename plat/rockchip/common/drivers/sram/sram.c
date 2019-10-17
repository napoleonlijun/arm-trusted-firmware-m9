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

#include <arch.h>
#include <arch_helpers.h>
#include <sram.h>
#include <stdlib.h>
#include <string.h>

#define ARCH_TIMER_TICKS_PER_US		(SYS_COUNTER_FREQ_IN_TICKS / 1000000)

#define UART_LSR			0x14
#define UART_FIFO_EMPTY			(BIT(6) | BIT(5))

/*********************** sram timer *******************************************/
static inline uint64_t __arch_counter_get_cntpct(void)
{
	uint64_t cval;

	isb();
	cval = read_cntpct_el0();

	return cval;
}

__sramfunc void sram_udelay(uint32_t us)
{
	uint64_t orig;
	uint64_t to_wait = ARCH_TIMER_TICKS_PER_US * us;

	orig = __arch_counter_get_cntpct();
	while (__arch_counter_get_cntpct() - orig <= to_wait)
		;
}

/*********************** sram console *****************************************/
__sramfunc void sram_putchar(char ch)
{
	mmio_write_32(PLAT_RK_UART_BASE, ch);
	if (ch == '\n')
		mmio_write_32(PLAT_RK_UART_BASE, '\r');

	while (!(mmio_read_32(PLAT_RK_UART_BASE + UART_LSR) & UART_FIFO_EMPTY))
		;
}

__sramfunc void sram_printstr(const char *str)
{
	while (*str) {
		sram_putchar(*str);
		str++;
	}
}

__sramfunc void sram_printhex(uint32_t hex)
{
	uint8_t c, i = 8;

	sram_putchar('0');
	sram_putchar('x');
	while (i--) {
		c = (hex & 0xf0000000) >> 28;
		sram_putchar(c < 0xa ? c + '0' : c - 0xa + 'a');
		hex <<= 4;
	}
}

__sramfunc void sram_printdec(int dec)
{
	int i, tmp = dec;

	if (dec < 0) {
		sram_putchar('-');
		tmp = -dec;
		dec = -dec;
	}

	for (i = 1; tmp / 10; tmp /= 10, i *= 10)
		;

	for (; i >= 1; i /= 10) {
		sram_putchar('0' + (char)(dec / i));
		dec %= i;
	}
}
