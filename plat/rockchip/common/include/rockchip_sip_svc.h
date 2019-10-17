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

#ifndef __ROCKCHIP_SIP_SVC_H__
#define __ROCKCHIP_SIP_SVC_H__

/* SMC function IDs for SiP Service queries */
#define SIP_SVC_CALL_COUNT		0x8200ff00
#define SIP_SVC_UID			0x8200ff01
#define SIP_SVC_VERSION			0x8200ff03

/* Rockchip SiP Service Calls */
#define RK_SIP_ATF_VERSION32		0x82000001
#define RK_SIP_ACCESS_REG32		0x82000002
#define RK_SIP_SUSPEND_MODE32		0x82000003
#define RK_SIP_UARTDBG_CFG64		0xc2000005
#define RK_SIP_MCU_EL3FIQ_CFG		0x82000006
#define RK_SIP_DDR_CFG32		0x82000008
#define RK_SIP_SHARE_MEM32		0x82000009
#define RK_SIP_SIP_VERSION32		0x8200000a

/* RK_SIP_SUSPEND_MODE32 child configs */
#define SUSPEND_MODE_CONFIG		0x01
#define WKUP_SOURCE_CONFIG		0x02
#define PWM_REGULATOR_CONFIG		0x03
#define GPIO_POWER_CONFIG		0x04
#define SUSPEND_DEBUG_ENABLE		0x05
#define APIOS_SUSPEND_CONFIG		0x06
#define VIRTUAL_POWEROFF		0x07

/* RK_SIP_ACCESS_REG32 read/write */
#define SEC_REG_RD			0x0
#define SEC_REG_WR			0x1

/* RK_SIP_MCU_EL3FIQ_CFG */
#define FIQ_INIT_HANDLER		0x01

/* RK_SIP_DDR_CFG32 child configs */
#define CONFIG_DRAM_INIT		0x00
#define CONFIG_DRAM_SET_RATE		0x01
#define CONFIG_DRAM_ROUND_RATE		0x02
#define CONFIG_DRAM_SET_AT_SR		0x03
#define CONFIG_DRAM_GET_BW		0x04
#define CONFIG_DRAM_GET_RATE		0x05
#define CONFIG_DRAM_CLR_IRQ		0x06
#define CONFIG_DRAM_SET_PARAM		0x07
#define CONFIG_DRAM_GET_VERSION		0x08

/* Rockchip SiP Service Calls version numbers */
#define RK_SIP_SVC_VERSION_MAJOR	0x0
#define RK_SIP_SVC_VERSION_MINOR	0x1

/* Number of ROCKCHIP SiP Calls implemented */
#define RK_COMMON_SIP_NUM_CALLS		0x4

/* SiP Service Calls Error return code */
#define SIP_RET_SUCCESS			0
/* -1 occupied by SMC_UNK */
#define SIP_RET_NOT_SUPPORTED		-2
#define SIP_RET_INVALID_PARAMS		-3
#define SIP_RET_INVALID_ADDRESS		-4
#define SIP_RET_DENIED			-5

/* Sip version */
#define SIP_IMPLEMENT_V1		(1)
#define SIP_IMPLEMENT_V2		(2)

/* Share mem page types */
typedef enum {
	SHARE_PAGE_TYPE_INVALID = 0,
	SHARE_PAGE_TYPE_UARTDBG,
	SHARE_PAGE_TYPE_DDR,
	SHARE_PAGE_TYPE_MAX,
} share_page_type_t;

/* Return data */
struct arm_smccc_res {
	unsigned long a1;
	unsigned long a2;
	unsigned long a3;
};

struct share_mem_manage {
	uint64_t page_base;
	uint64_t page_num;
	share_page_type_t page_type;
};

/* SiP Service Calls */
int sip_version_handler(struct arm_smccc_res *res);
int share_mem_type2page_base(share_page_type_t page_type, uint64_t *out_value);
uint64_t share_mem_type2page_size(share_page_type_t page_type);
int share_mem_page_get_handler(uint64_t page_num,
			       share_page_type_t page_type,
			       struct arm_smccc_res *res);

uint64_t rockchip_plat_sip_handler(uint32_t smc_fid,
				   uint64_t x1,
				   uint64_t x2,
				   uint64_t x3,
				   uint64_t x4,
				   void *cookie,
				   void *handle,
				   uint64_t flags);
#endif
