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

#include <debug.h>
#include <delay_timer.h>
#include <mmio.h>
#include <platform_def.h>
#include <efuse.h>
#include <string.h>

#if PLATFORM_IS_RK(3368)
#define EFUSE_AUTO_MODE		0
#elif PLATFORM_IS_RK(3228)
#define EFUSE_AUTO_MODE		1
#endif

#define read32(reg)			mmio_read_32(reg)
#define write32(v, reg)			mmio_write_32(reg, v)
#define efuse8_read(offset)		mmio_read_32(EFUSE8_BASE + offset)
#define efuse8_write(v, offset)		mmio_write_32(EFUSE8_BASE + offset, v)
#define efuse32_read(offset)		mmio_read_32(EFUSE32_BASE + offset)
#define efuse32_write(v, offset)	mmio_write_32(EFUSE32_BASE + offset, v)

/* global buf to store efuse data */
#if PLATFORM_IS_RK(3368)
static uint8_t efuse8_buf[32] = {0};
#endif
static uint32_t efuse32_buf[32] = {0};

enum clk_type {
	CLK = 0,
	PCLK,
};

static uint32_t enable_efuse_clk(int clk)
{
	uint32_t reg = 0;

#if PLATFORM_IS_RK(3368)
	switch (clk) {
	case CLK:
		/* enable clk */
		reg = read32(CRU_BASE + EFUSE_CLKGATE_CON);
		if (reg & EFUSE_1024_GATE_EN)
			write32(EFUSE_1024_GATE_EN << CRU_WRITE_MASK,
				CRU_BASE + EFUSE_CLKGATE_CON);
		break;
	default:
		break;
	}

#elif PLATFORM_IS_RK(3228)
	switch (clk) {
	case CLK:
		reg = read32(CRU_BASE + CRU_CLKGATE_CON2);
		/* enable efuse work clk */
		if (reg & EFUSE_SRC_CLK_EN)
			write32(EFUSE_SRC_CLK_EN << CRU_WRITE_MASK,
				CRU_BASE + CRU_CLKGATE_CON2);
		break;
	case PCLK:
		reg = read32(CRU_BASE + CRU_CLKGATE_CON15);
		/* enable efuse APB clk */
		if (reg & EFUSE_1024_PCLK_EN)
			write32(EFUSE_1024_PCLK_EN << CRU_WRITE_MASK,
				CRU_BASE + CRU_CLKGATE_CON15);
		break;
	default:
		break;
	}
#endif
	return reg;
}

static void restore_efuse_clk(int clk, uint32_t reg)
{
#if PLATFORM_IS_RK(3368)
	switch (clk) {
	case CLK:
		/* disable clk */
		if (reg & EFUSE_1024_GATE_EN)
			write32(reg | (EFUSE_1024_GATE_EN << CRU_WRITE_MASK),
				CRU_BASE + EFUSE_CLKGATE_CON);
		break;
	default:
		return;
	}
#elif PLATFORM_IS_RK(3228)
	switch (clk) {
	case CLK:
		/* disable efuse work clk */
		if (reg & EFUSE_SRC_CLK_EN)
			write32(reg | (EFUSE_SRC_CLK_EN << CRU_WRITE_MASK),
				CRU_BASE + CRU_CLKGATE_CON2);
		break;
	case PCLK:
		/* disable efuse APB clk */
		if (reg & EFUSE_1024_PCLK_EN)
			write32(reg | (EFUSE_1024_PCLK_EN << CRU_WRITE_MASK),
				CRU_BASE + CRU_CLKGATE_CON15);
		break;
	default:
		return;
	}
#endif
}

/* 8*32 bits read */
#if PLATFORM_IS_RK(3368)
int rk_efuse8_readregs(uint32_t addr, uint32_t length, uint8_t *buf)
{
	uint32_t reg;

	if (addr > 31)
		return -1;
	if (length < 1 || length > 32)
		return -1;
	if (!buf)
		return -1;

	/* check efuse_256 clk */
	reg = read32(CRU_BASE + EFUSE_CLKGATE_CON);
	/* enable clk */
	if (reg & EFUSE_256_GATE_EN)
		write32(EFUSE_256_GATE_EN << CRU_WRITE_MASK,
			CRU_BASE + EFUSE_CLKGATE_CON);

	efuse8_write(EFUSE_CSB, REG_EFUSE_CTRL);
	efuse8_write(EFUSE_LOAD | EFUSE_PGENB, REG_EFUSE_CTRL);
	udelay(2);
	do {
		efuse8_write(efuse8_read(REG_EFUSE_CTRL) &
			(~(EFUSE_A_MASK << EFUSE_A_SHIFT)), REG_EFUSE_CTRL);
		efuse8_write(efuse8_read(REG_EFUSE_CTRL) |
			((addr & EFUSE_A_MASK) << EFUSE_A_SHIFT),
			REG_EFUSE_CTRL);
		udelay(2);
		efuse8_write(efuse8_read(REG_EFUSE_CTRL) |
				EFUSE_STROBE, REG_EFUSE_CTRL);
		udelay(2);
		*buf = efuse8_read(REG_EFUSE_DOUT);
		efuse8_write(efuse8_read(REG_EFUSE_CTRL) &
				(~EFUSE_STROBE), REG_EFUSE_CTRL);
		udelay(2);
		buf++;
		addr++;
	} while (--length);

	udelay(2);
	efuse8_write(efuse8_read(REG_EFUSE_CTRL) | EFUSE_CSB, REG_EFUSE_CTRL);
	udelay(1);

	/* disable clk */
	if (reg & EFUSE_256_GATE_EN)
		write32(reg | (EFUSE_256_GATE_EN << CRU_WRITE_MASK),
			CRU_BASE + EFUSE_CLKGATE_CON);

	return 0;
}

/* 8*32 bits total, write 8 bits one time, addr from 0-31 */
int rk_efuse8_write(uint32_t addr, uint8_t val)
{
	uint32_t reg;

	if (addr > 31)
		return -1;

	/* check efuse_256 clk */
	reg = read32(CRU_BASE + EFUSE_CLKGATE_CON);
	/* enable clk */
	if (reg & EFUSE_256_GATE_EN)
		write32(EFUSE_256_GATE_EN << CRU_WRITE_MASK,
			CRU_BASE + EFUSE_CLKGATE_CON);

	efuse8_write(EFUSE_CSB | EFUSE_LOAD | EFUSE_PGENB,
		     REG_EFUSE_CTRL);
	efuse8_write(efuse8_read(REG_EFUSE_CTRL) & (~EFUSE_CSB),
		     REG_EFUSE_CTRL);
	efuse8_write(efuse8_read(REG_EFUSE_CTRL) & (~EFUSE_PGENB) &
		(~EFUSE_LOAD), REG_EFUSE_CTRL);
	udelay(2);

	while (val) {
		if (val & 0x01) {
			efuse8_write(efuse8_read(REG_EFUSE_CTRL) &
				(~(EFUSE_A_MASK << EFUSE_A_SHIFT)),
				REG_EFUSE_CTRL);
			efuse8_write(efuse8_read(REG_EFUSE_CTRL) |
				((addr & EFUSE_A_MASK) << EFUSE_A_SHIFT),
				REG_EFUSE_CTRL);
			udelay(2);
			efuse8_write(efuse8_read(REG_EFUSE_CTRL) |
				EFUSE_STROBE, REG_EFUSE_CTRL);
			udelay(10);
			efuse8_write(efuse8_read(REG_EFUSE_CTRL) &
					(~EFUSE_STROBE), REG_EFUSE_CTRL);
			udelay(2);
		}
		addr += 32;
		val = val >> 1;
	}

	udelay(2);
	efuse8_write(efuse8_read(REG_EFUSE_CTRL) | EFUSE_LOAD |
		     EFUSE_PGENB, REG_EFUSE_CTRL);
	efuse8_write(efuse8_read(REG_EFUSE_CTRL) | EFUSE_CSB,
		     REG_EFUSE_CTRL);
	udelay(1);

	/* disable clk */
	if (reg & EFUSE_256_GATE_EN)
		write32(reg | (EFUSE_256_GATE_EN << CRU_WRITE_MASK),
			CRU_BASE + EFUSE_CLKGATE_CON);

	return 0;
}
#endif /* PLATFORM_IS_RK(3368) */

/* user mode and auto mode */
int rk_efuse32_readregs(uint32_t addr, uint32_t length, uint32_t *buf)
{
	uint32_t reg0 = 0;
	uint32_t reg1 = 0;

	uint32_t efuse_base = 0;

	if (addr > 31)
		return -1;
	if (length < 1 || length > 32)
		return -1;
	if (!buf)
		return -1;

#if PLATFORM_IS_RK(3368)
	efuse_base = EFUSE32_BASE;

#elif PLATFORM_IS_RK(3228)
	if (addr < 24) {
		if (addr + length < 25)
			efuse_base = EFUSE32_BASE;
		else
			return -1;
	}
	if (addr > 23) {
		if (addr + length < 33)
			efuse_base = EFUSE8_BASE;
		else
			return -1;
	}

#endif
	reg0 = enable_efuse_clk(CLK);
	reg1 = enable_efuse_clk(PCLK);

#if EFUSE_AUTO_MODE
	do {
		write32(EFUSE_AUTO_RD | EFUSE_AUTO_ENABLE |
			((addr & EFUSE_A_MASK) << EFUSE_A_SHIFT),
			efuse_base + REG_EFUSE_AUTO_CTRL);
		udelay(2);
		while (1) {
			if (read32(efuse_base + REG_EFUSE_INT_STATUS) & 0x01)
				break;
		}
		*buf = read32(efuse_base + REG_EFUSE_DOUT);
		write32(read32(efuse_base + REG_EFUSE_AUTO_CTRL) &
			(~EFUSE_AUTO_ENABLE), efuse_base + REG_EFUSE_AUTO_CTRL);
		write32(0x07, efuse_base + REG_EFUSE_INT_STATUS);
		buf++;
		addr++;

	} while (--length);
#else
#if PLATFORM_IS_RK(3228)
	/* enable read in user mode */
	write32((read32(efuse_base + REG_EFUSE_MOD) | EFUSE_RD_ENB_USER) &
		(~EFUSE_PG_ENB_USER), efuse_base + REG_EFUSE_MOD);
#endif
	write32(EFUSE_CSB, efuse_base + REG_EFUSE_CTRL);
	write32(EFUSE_LOAD | EFUSE_PGENB, efuse_base + REG_EFUSE_CTRL);
	udelay(2);
	do {
		write32(read32(efuse_base + REG_EFUSE_CTRL) &
			(~(EFUSE_A_MASK << EFUSE_A_SHIFT)),
			efuse_base + REG_EFUSE_CTRL);
		write32(read32(efuse_base + REG_EFUSE_CTRL) |
			((addr & EFUSE_A_MASK) << EFUSE_A_SHIFT),
			efuse_base + REG_EFUSE_CTRL);

		udelay(2);
		write32(read32(efuse_base + REG_EFUSE_CTRL) |
			EFUSE_STROBE, efuse_base + REG_EFUSE_CTRL);
		udelay(2);
		*buf = read32(efuse_base + REG_EFUSE_DOUT);
		write32(read32(efuse_base + REG_EFUSE_CTRL) &
			(~EFUSE_STROBE), efuse_base + REG_EFUSE_CTRL);
		udelay(2);
		buf++;
		addr++;
	} while (--length);
	udelay(2);
	write32(read32(efuse_base + REG_EFUSE_CTRL) | EFUSE_CSB,
		efuse_base + REG_EFUSE_CTRL);
	udelay(1);
#endif /* EFUSE_AUTO_MODE */

	restore_efuse_clk(CLK, reg0);
	restore_efuse_clk(PCLK, reg1);

	return 0;
}

/*user mode and auto mode*/
int rk_efuse32_write(uint32_t addr, uint32_t val)
{
	uint32_t reg0 = 0, reg1 = 0, efuse_base = 0;

	if (addr > 31)
		return -1;

#if PLATFORM_IS_RK(3368)
	efuse_base = EFUSE32_BASE;

#elif PLATFORM_IS_RK(3228)
	if (addr < 24)
		efuse_base = EFUSE32_BASE;
	if (addr > 23 && addr < 32)
		efuse_base = EFUSE8_BASE;
#endif
	reg0 = enable_efuse_clk(CLK);
	reg1 = enable_efuse_clk(PCLK);

#if EFUSE_AUTO_MODE
	while (val) {
		if (val & 0x01) {
			write32((EFUSE_AUTO_ENABLE |
				((addr & EFUSE_A_MASK) << EFUSE_A_SHIFT))
				& (~EFUSE_AUTO_RD),
				efuse_base + REG_EFUSE_AUTO_CTRL);
			udelay(2);
			while (1) {
				if (read32(efuse_base + REG_EFUSE_INT_STATUS) &
					   0x01)
					break;
			}
			write32(read32(efuse_base + REG_EFUSE_AUTO_CTRL) &
				(~EFUSE_AUTO_ENABLE),
				efuse_base + REG_EFUSE_AUTO_CTRL);
			write32(0x07, efuse_base + REG_EFUSE_INT_STATUS);
			udelay(2);
		}
		addr += 32;
		val = val >> 1;
		udelay(2);
	}
#else
#if PLATFORM_IS_RK(3228)
	/* enable program in user mode */
	write32((read32(efuse_base + REG_EFUSE_MOD) | EFUSE_PG_ENB_USER) &
		(~EFUSE_RD_ENB_USER), efuse_base + REG_EFUSE_MOD);
#endif
	write32(EFUSE_CSB | EFUSE_LOAD | EFUSE_PGENB,
		efuse_base + REG_EFUSE_CTRL);
	write32(read32(efuse_base + REG_EFUSE_CTRL) & (~EFUSE_CSB),
		efuse_base + REG_EFUSE_CTRL);
	write32(read32(efuse_base + REG_EFUSE_CTRL) & (~EFUSE_PGENB) &
		(~EFUSE_LOAD), efuse_base + REG_EFUSE_CTRL);
	udelay(2);

	while (val) {
		if (val & 0x01) {
			write32(read32(efuse_base + REG_EFUSE_CTRL) &
				(~(EFUSE_A_MASK << EFUSE_A_SHIFT)),
				efuse_base + REG_EFUSE_CTRL);
			write32(read32(efuse_base + REG_EFUSE_CTRL) |
				((addr & EFUSE_A_MASK) << EFUSE_A_SHIFT),
				efuse_base + REG_EFUSE_CTRL);
			udelay(2);
			write32(read32(efuse_base + REG_EFUSE_CTRL) |
				EFUSE_STROBE, efuse_base + REG_EFUSE_CTRL);
			udelay(10);
			write32(read32(efuse_base + REG_EFUSE_CTRL) &
				(~EFUSE_STROBE),
				efuse_base + REG_EFUSE_CTRL);
			udelay(2);
		}
		addr += 32;
		val = val >> 1;
	}

	udelay(2);
	write32(read32(efuse_base + REG_EFUSE_CTRL) | EFUSE_LOAD |
		EFUSE_PGENB, efuse_base + REG_EFUSE_CTRL);
	write32(read32(efuse_base + REG_EFUSE_CTRL) | EFUSE_CSB,
		efuse_base + REG_EFUSE_CTRL);
	udelay(1);
#endif /* EFUSE_AUTO_MODE */

	restore_efuse_clk(CLK, reg0);
	restore_efuse_clk(PCLK, reg1);

	return 0;
}

#if PLATFORM_IS_RK(3368)
int rk_efuse8_read(uint32_t addr, uint32_t length, uint8_t *buf)
{
	if (addr > 31)
		return -1;
	if (length < 1 || length > 32)
		return -1;
	if (!buf)
		return -1;

	memcpy(buf, efuse8_buf + addr, length * sizeof(uint8_t));

	return 0;
}
#endif
int rk_efuse32_read(uint32_t addr, uint32_t length, uint32_t *buf)
{
	if (!buf)
		return -1;

#if PLATFORM_IS_RK(3368)
	if (addr > 31)
		return -1;

	if (length < 1 || length > 32)
		return -1;

	memcpy(buf, efuse32_buf + addr, length * sizeof(uint32_t));

#elif PLATFORM_IS_RK(3228)
	if (addr > 23) {
		if (length < 1 || length + addr > 32)
			return -1;

		memcpy(buf, efuse32_buf + addr, length * sizeof(uint32_t));
	} else if (addr < 24) {
		if (length < 1 || length + addr > 24)
			return -1;

		memcpy(buf, efuse32_buf + addr, length * sizeof(uint32_t));
	}
#endif
	return 0;
}

#if PLATFORM_IS_RK(3228)
void mode_init(int sec)
{
	int reg;
	uint32_t efuse_base = 0;

	if (sec == 1)
		efuse_base = EFUSE32_BASE;/*secure efuse addr*/
	if (sec == 0)
		efuse_base = EFUSE8_BASE;/*un_secure efsue addr*/

#if EFUSE_AUTO_MODE
	/* enable auto mode */
	reg = read32(efuse_base + REG_EFUSE_MOD);
	write32(reg & (~EFUSE_USER_MODE), efuse_base + REG_EFUSE_MOD);

	/* enable finish & auto_s_access_ns_err & auto_ns_access_s_err int */
	write32(0x07, efuse_base + REG_EFUSE_INT_CON);
	write32((T_CSB_P_S << 16) | T_CSB_P_L,
		efuse_base + REG_EFUSE_T_CSB_P);
	write32((T_PGENB_P_S << 16) | T_PGENB_P_L,
		efuse_base + REG_EFUSE_T_PGENB_P);
	write32((T_LOAD_P_S << 16) | T_LOAD_P_L,
		efuse_base + REG_EFUSE_T_LOAD_P);
	write32((T_ADDR_P_S << 16) | T_ADDR_P_L,
		efuse_base + REG_EFUSE_T_ADDR_P);
	write32((T_STROBE_P_S << 16) | T_STROBE_P_L,
		efuse_base + REG_EFUSE_T_STROBE_P);
	write32((T_CSB_R_S << 16) | T_CSB_R_L,
		efuse_base + REG_EFUSE_T_CSB_R);
	write32((T_PGENB_R_S << 16) | T_PGENB_R_L,
		efuse_base + REG_EFUSE_T_PGENB_R);
	write32((T_LOAD_R_S << 16) | T_LOAD_R_L,
		efuse_base + REG_EFUSE_T_LOAD_R);
	write32((T_ADDR_R_S << 16) | T_ADDR_R_L,
		efuse_base + REG_EFUSE_T_ADDR_R);
	write32((T_STROBE_R_S << 16) | T_STROBE_R_L,
		efuse_base + REG_EFUSE_T_STROBE_R);
#else
	reg = read32(efuse_base + REG_EFUSE_MOD);
	write32(reg | EFUSE_USER_MODE, efuse_base + REG_EFUSE_MOD);
#endif /* EFUSE_AUTO_MODE */
}

void rk_efuse_prog_en(int n)
{
	if (n == 1) {
		write32(read32(GPIO2_BASE) & (~(0X1 << 3)), GPIO2_BASE);
		write32((efuse_pwren | efuse_pwren << 16) |
			((0) << 7 | efuse_pwren << 16 << 1),
			GRF_BASE + GRF_GPIO2A_IOMUX);
		/*efuse program enable*/
		write32((1 << 7) | (1 << 7 << 16), SGRF_BASE +
			EFUSE_SGRF_SOC_CON5);
	} else {
		write32((0 << 7) | (1 << 7 << 16),
			SGRF_BASE + EFUSE_SGRF_SOC_CON5);
		write32(0 << 6 | efuse_pwren << 16 |
			((0) << 7 | efuse_pwren << 16 << 1),
			GRF_BASE + GRF_GPIO2A_IOMUX);
		write32(read32(GPIO2_BASE) | (0X1 << 3), GPIO2_BASE);
	}
}
#endif /* PLATFORM_IS_RK(3228) */

int rk_efuse_init(void)
{
#if PLATFORM_IS_RK(3368)
	if (rk_efuse8_readregs(0, 32, efuse8_buf)) {
		ERROR("read efuse_256 failed!!!!\n");
		return -1;
	}

	if (rk_efuse32_readregs(0, 32, efuse32_buf)) {
		ERROR("read efuse_1024 failed!!!!\n");
		return -1;
	}

#elif PLATFORM_IS_RK(3228)
	mode_init(1);
	mode_init(0);

	if (rk_efuse32_readregs(RK322XH_S_EFUSE_START,
				RK322XH_S_EFUSE_WORDS,
				efuse32_buf)) {
		ERROR("read S-efuse failed!!!!\n");
		return -1;
	}

	if (rk_efuse32_readregs(RK322XH_NS_EFUSE_START,
				RK322XH_NS_EFUSE_WORDS,
				efuse32_buf + RK322XH_NS_EFUSE_START)) {
		ERROR("read NS-efuse failed!!!!\n");
		return -1;
	}
#endif
	return 0;
}
