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

#ifndef __DDR_RK3368_H__
#define __DDR_RK3368_H__

#include <plat_private.h>

#define DDR_TAG			0x56313031
#define DDR_END_TAG		0xffffffff

#define GRF_DDRC0_CON0		0x600
#define GRF_SOC_STATUS0		0x480

#define SET_NR(n)		BITS_WITH_WMASK((n - 1), 0x3f, 8)
#define SET_NO(n)		BITS_WITH_WMASK((n - 1), 0xf, 0)
#define SET_NF(n)		((n - 1) & 0x1fff)
#define SET_NB(n)		((n - 1) & 0xfff)
#define PLLMODE(n)		BITS_WITH_WMASK(n, 0x3, 8)

/* CRU softreset ddr pctl, phy */
#define ddrmsch0_srstn_req(n)		BITS_WITH_WMASK(n, 0x1, 10)
#define ddrctrl0_psrstn_req(n)		BITS_WITH_WMASK(n, 0x1, 3)
#define ddrctrl0_srstn_req(n)		BITS_WITH_WMASK(n, 0x1, 2)
#define ddrphy0_psrstn_req(n)		BITS_WITH_WMASK(n, 0x1, 1)
#define ddrphy0_srstn_req(n)		BITS_WITH_WMASK(n, 0x1, 0)

/* PMUGRF_SOC_CON0 */
#define ddrphy_bufferen_io_en(n)	BITS_WITH_WMASK(n, 0x1, 9)
#define ddrphy_bufferen_core_en(n)	BITS_WITH_WMASK(n, 0x1, 9)

uint32_t ddr_get_resume_code_size(void);
uint32_t ddr_get_resume_data_size(void);
uint32_t *ddr_get_resume_code_base(void);
void ddr_reg_save(uint32_t pllpdstat, uint64_t base_addr);
void rockchip_ddr_resume_init(void);

#endif
