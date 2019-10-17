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

#include <mmio.h>
#include <debug.h>
#include <stdint.h>
#include <string.h>
#include <ddr_rk3368.h>
#include <rk3368_def.h>
#include <pmu.h>
#include <soc.h>

typedef union STAT_Tag {
	uint32_t d32;
	struct {
		unsigned ctl_stat : 3;
		unsigned reserved3 : 1;
		unsigned lp_trig : 3;
		unsigned reserved7_31 : 25;
	} b;
} STAT_T;

typedef union SCFG_Tag {
	uint32_t d32;
	struct {
		unsigned hw_low_power_en : 1;
		unsigned reserved1_5 : 5;
		unsigned nfifo_nif1_dis : 1;
		unsigned reserved7 : 1;
		unsigned bbflags_timing : 4;
		unsigned reserved12_31 : 20;
	} b;
} SCFG_T;

/* DDR Controller register struct */
typedef volatile struct DDR_REG_Tag {
	/* Operational State, Control, and Status Registers */
	SCFG_T SCFG;

	volatile uint32_t SCTL;
	STAT_T STAT;
	volatile uint32_t INTRSTAT;
	uint32_t reserved0[(0x40 - 0x10) / 4];
	/* Initailization Control and Status Registers */
	volatile uint32_t MCMD;
	volatile uint32_t POWCTL;
	volatile uint32_t POWSTAT;
	volatile uint32_t CMDTSTAT;
	volatile uint32_t CMDTSTATEN;
	uint32_t reserved1[(0x60 - 0x54) / 4];
	volatile uint32_t MRRCFG0;
	volatile uint32_t MRRSTAT0;
	volatile uint32_t MRRSTAT1;
	uint32_t reserved2[(0x7c - 0x6c) / 4];
	/* Memory Control and Status Registers */
	volatile uint32_t MCFG1;
	volatile uint32_t MCFG;
	volatile uint32_t PPCFG;
	volatile uint32_t MSTAT;
	volatile uint32_t LPDDR2ZQCFG;
	uint32_t reserved3;
	/* DTU Control and Status Registers */
	volatile uint32_t DTUPDES;
	volatile uint32_t DTUNA;
	volatile uint32_t DTUNE;
	volatile uint32_t DTUPRD0;
	volatile uint32_t DTUPRD1;
	volatile uint32_t DTUPRD2;
	volatile uint32_t DTUPRD3;
	volatile uint32_t DTUAWDT;
	uint32_t reserved4[(0xc0 - 0xb4) / 4];
	/* Memory Timing Registers */
	volatile uint32_t TOGCNT1U;
	volatile uint32_t TINIT;
	volatile uint32_t TRSTH;
	volatile uint32_t TOGCNT100N;
	volatile uint32_t TREFI;
	volatile uint32_t TMRD;
	volatile uint32_t TRFC;
	volatile uint32_t TRP;
	volatile uint32_t TRTW;
	volatile uint32_t TAL;
	volatile uint32_t TCL;
	volatile uint32_t TCWL;
	volatile uint32_t TRAS;
	volatile uint32_t TRC;
	volatile uint32_t TRCD;
	volatile uint32_t TRRD;
	volatile uint32_t TRTP;
	volatile uint32_t TWR;
	volatile uint32_t TWTR;
	volatile uint32_t TEXSR;
	volatile uint32_t TXP;
	volatile uint32_t TXPDLL;
	volatile uint32_t TZQCS;
	volatile uint32_t TZQCSI;
	volatile uint32_t TDQS;
	volatile uint32_t TCKSRE;
	volatile uint32_t TCKSRX;
	volatile uint32_t TCKE;
	volatile uint32_t TMOD;
	volatile uint32_t TRSTL;
	volatile uint32_t TZQCL;
	volatile uint32_t TMRR;
	volatile uint32_t TCKESR;
	volatile uint32_t TDPD;
	volatile uint32_t TREFI_MEM_DDR3;
	uint32_t reserved5[(0x180 - 0x14c) / 4];
	/* ECC Configuration, Control, and Status Registers */
	volatile uint32_t ECCCFG;
	volatile uint32_t ECCTST;
	volatile uint32_t ECCCLR;
	volatile uint32_t ECCLOG;
	uint32_t reserved6[(0x200 - 0x190) / 4];
	/* DTU Control and Status Registers */
	volatile uint32_t DTUWACTL;
	volatile uint32_t DTURACTL;
	volatile uint32_t DTUCFG;
	volatile uint32_t DTUECTL;
	volatile uint32_t DTUWD0;
	volatile uint32_t DTUWD1;
	volatile uint32_t DTUWD2;
	volatile uint32_t DTUWD3;
	volatile uint32_t DTUWDM;
	volatile uint32_t DTURD0;
	volatile uint32_t DTURD1;
	volatile uint32_t DTURD2;
	volatile uint32_t DTURD3;
	volatile uint32_t DTULFSRWD;
	volatile uint32_t DTULFSRRD;
	volatile uint32_t DTUEAF;
	/* DFI Control Registers */
	volatile uint32_t DFITCTRLDELAY;
	volatile uint32_t DFIODTCFG;
	volatile uint32_t DFIODTCFG1;
	volatile uint32_t DFIODTRANKMAP;
	/* DFI Write Data Registers */
	volatile uint32_t DFITPHYWRDATA;
	volatile uint32_t DFITPHYWRLAT;
	volatile uint32_t DFITPHYWRDATALAT;
	uint32_t reserved7;
	volatile uint32_t DFITRDDATAEN;
	volatile uint32_t DFITPHYRDLAT;
	uint32_t reserved8[(0x270 - 0x268) / 4];
	/* DFI Update Registers */
	volatile uint32_t DFITPHYUPDTYPE0;
	volatile uint32_t DFITPHYUPDTYPE1;
	volatile uint32_t DFITPHYUPDTYPE2;
	volatile uint32_t DFITPHYUPDTYPE3;
	volatile uint32_t DFITCTRLUPDMIN;
	volatile uint32_t DFITCTRLUPDMAX;
	volatile uint32_t DFITCTRLUPDDLY;
	uint32_t reserved9;
	volatile uint32_t DFIUPDCFG;
	volatile uint32_t DFITREFMSKI;
	volatile uint32_t DFITCTRLUPDI;
	uint32_t reserved10[(0x2ac - 0x29c) / 4];
	volatile uint32_t DFITRCFG0;
	volatile uint32_t DFITRSTAT0;
	volatile uint32_t DFITRWRLVLEN;
	volatile uint32_t DFITRRDLVLEN;
	volatile uint32_t DFITRRDLVLGATEEN;
	/* DFI Status Registers */
	volatile uint32_t DFISTSTAT0;
	volatile uint32_t DFISTCFG0;
	volatile uint32_t DFISTCFG1;
	uint32_t reserved11;
	volatile uint32_t DFITDRAMCLKEN;
	volatile uint32_t DFITDRAMCLKDIS;
	volatile uint32_t DFISTCFG2;
	volatile uint32_t DFISTPARCLR;
	volatile uint32_t DFISTPARLOG;
	uint32_t reserved12[(0x2f0 - 0x2e4) / 4];
	/* DFI Low Power Registers */
	volatile uint32_t DFILPCFG0;
	uint32_t reserved13[(0x300 - 0x2f4) / 4];
	/* DFI Training 2 Registers */
	volatile uint32_t DFITRWRLVLRESP0;
	volatile uint32_t DFITRWRLVLRESP1;
	volatile uint32_t DFITRWRLVLRESP2;
	volatile uint32_t DFITRRDLVLRESP0;
	volatile uint32_t DFITRRDLVLRESP1;
	volatile uint32_t DFITRRDLVLRESP2;
	volatile uint32_t DFITRWRLVLDELAY0;
	volatile uint32_t DFITRWRLVLDELAY1;
	volatile uint32_t DFITRWRLVLDELAY2;
	volatile uint32_t DFITRRDLVLDELAY0;
	volatile uint32_t DFITRRDLVLDELAY1;
	volatile uint32_t DFITRRDLVLDELAY2;
	volatile uint32_t DFITRRDLVLGATEDELAY0;
	volatile uint32_t DFITRRDLVLGATEDELAY1;
	volatile uint32_t DFITRRDLVLGATEDELAY2;
	volatile uint32_t DFITRCMD;
	uint32_t reserved14[(0x3f8 - 0x340) / 4];
	/* IP Status Registers */
	volatile uint32_t IPVR;
	volatile uint32_t IPTR;
} DDR_REG_T, *pDDR_REG_T;

static pDDR_REG_T pDDR_Reg;
static uint32_t ddrpctl_addr_phy;

/* DDR PHY register struct */
typedef volatile struct DDRPHY_REG_Tag {
	volatile uint32_t PHY_REG0;
	volatile uint32_t PHY_REG1;
	volatile uint32_t PHY_REG2;
	volatile uint32_t PHY_REG3;
	volatile uint32_t PHY_REG4;
	volatile uint32_t PHY_REG5;
	volatile uint32_t PHY_REG6;
	uint32_t reserved0[(0x2c - 0x1c) / 4];
	volatile uint32_t PHY_REGb;
	volatile uint32_t PHY_REGc;
	uint32_t reserved1[(0x44 - 0x34) / 4];
	volatile uint32_t PHY_REG11;
	volatile uint32_t PHY_REG12;
	volatile uint32_t PHY_REG13;
	volatile uint32_t PHY_REG14;
	uint32_t reserved2;
	volatile uint32_t PHY_REG16;
	uint32_t reserved3[(0x80 - 0x5c) / 4];
	volatile uint32_t PHY_REG20;
	volatile uint32_t PHY_REG21;
	uint32_t reserved4[(0x98 - 0x88) / 4];
	volatile uint32_t PHY_REG26;
	volatile uint32_t PHY_REG27;
	volatile uint32_t PHY_REG28;
	uint32_t reserved5[(0xc0 - 0xa4) / 4];
	volatile uint32_t PHY_REG30;
	volatile uint32_t PHY_REG31;
	uint32_t reserved6[(0xd8 - 0xc8) / 4];
	volatile uint32_t PHY_REG36;
	volatile uint32_t PHY_REG37;
	volatile uint32_t PHY_REG38;
	uint32_t reserved7[(0x100 - 0xe4) / 4];
	volatile uint32_t PHY_REG40;
	volatile uint32_t PHY_REG41;
	uint32_t reserved8[(0x118 - 0x108) / 4];
	volatile uint32_t PHY_REG46;
	volatile uint32_t PHY_REG47;
	volatile uint32_t PHY_REG48;
	uint32_t reserved9[(0x140 - 0x124) / 4];
	volatile uint32_t PHY_REG50;
	volatile uint32_t PHY_REG51;
	uint32_t reserved10[(0x158 - 0x148) / 4];
	volatile uint32_t PHY_REG56;
	volatile uint32_t PHY_REG57;
	volatile uint32_t PHY_REG58;
	uint32_t reserved11[(0x1c0 - 0x164) / 4];
	volatile uint32_t PHY_REG_skew_cs0data[(0x270 - 0x1c0) / 4];
	uint32_t reserved12[(0x290 - 0x270) / 4];
	volatile uint32_t PHY_REGDLL;
	uint32_t reserved13[(0x2c0 - 0x294) / 4];
	volatile uint32_t PHY_REG_skew[(0x3b0 - 0x2c0) / 4];
	volatile uint32_t PHY_REGec;
	volatile uint32_t PHY_REGed;
	volatile uint32_t PHY_REGee;
	volatile uint32_t PHY_REGef;
	volatile uint32_t PHY_REGf0;
	volatile uint32_t PHY_REGf1;
	volatile uint32_t PHY_REGf2;
	uint32_t reserved14[(0x3e8 - 0x3cc) / 4];
	volatile uint32_t PHY_REGfa;
	volatile uint32_t PHY_REGfb;
	volatile uint32_t PHY_REGfc;
	volatile uint32_t PHY_REGfd;
	volatile uint32_t PHY_REGfe;
	volatile uint32_t PHY_REGff;
} DDRPHY_REG_T, *pDDRPHY_REG_T;

static pDDRPHY_REG_T pPHY_Reg;
static uint32_t ddrphy_addr_phy;

typedef struct PCTRL_TIMING_Tag {
	uint32_t ddrFreq;
	/* Memory Timing Registers */
	uint32_t togcnt1u;
	uint32_t tinit;
	uint32_t trsth;
	uint32_t togcnt100n;
	uint32_t trefi;
	uint32_t tmrd;
	uint32_t trfc;
	uint32_t trp;
	uint32_t trtw;
	uint32_t tal;
	uint32_t tcl;
	uint32_t tcwl;
	uint32_t tras;
	uint32_t trc;
	uint32_t trcd;
	uint32_t trrd;
	uint32_t trtp;
	uint32_t twr;
	uint32_t twtr;
	uint32_t texsr;
	uint32_t txp;
	uint32_t txpdll;
	uint32_t tzqcs;
	uint32_t tzqcsi;
	uint32_t tdqs;
	uint32_t tcksre;
	uint32_t tcksrx;
	uint32_t tcke;
	uint32_t tmod;
	uint32_t trstl;
	uint32_t tzqcl;
	uint32_t tmrr;
	uint32_t tckesr;
	uint32_t tdpd;
	uint32_t trefi_mem_ddr3;
} PCTL_TIMING_T;

typedef union NOC_TIMING_Tag {
	uint32_t d32;
	struct {
		unsigned ActToAct : 6;
		unsigned RdToMiss : 6;
		unsigned WrToMiss : 6;
		unsigned BurstLen : 3;
		unsigned RdToWr : 5;
		unsigned WrToRd : 5;
		unsigned BwRatio : 1;
	} b;
} NOC_TIMING_T;

typedef union NOC_ACTIVATE_Tag {
	uint32_t d32;
	struct {
		unsigned Rrd : 4;
		unsigned Faw : 6;
		unsigned Fawbank : 1;
		unsigned reserved : 21;
	} b;
} NOC_ACTIVATE_T;

typedef volatile struct MSCH_REG_Tag {
	volatile uint32_t coreid;
	volatile uint32_t revisionid;
	volatile uint32_t ddrconf;

	volatile NOC_TIMING_T ddrtiming;
	volatile uint32_t ddrmode;
	volatile uint32_t readlatency;
	uint32_t reserved1[(0x38 - 0x18) / 4];

	volatile NOC_ACTIVATE_T activate;
	volatile uint32_t devtodev;
} MSCH_REG, *pMSCH_REG;

static pMSCH_REG pMSCH_Reg;
static uint32_t noc_addr_phy;

typedef volatile struct MSCH_SAVE_REG_Tag {
	uint32_t ddrconf;
	NOC_TIMING_T ddrtiming;
	uint32_t ddrmode;
	uint32_t readlatency;
	NOC_ACTIVATE_T activate;
	uint32_t devtodev;
} MSCH_SAVE_REG;

/* ddr suspend need save reg */
typedef struct PCTL_SAVE_REG_Tag {
	uint32_t SCFG;
	uint32_t CMDTSTATEN;
	uint32_t MCFG1;
	uint32_t MCFG;
	uint32_t PPCFG;
	PCTL_TIMING_T pctl_timing;
	/* DFI Control Registers */
	uint32_t DFITCTRLDELAY;
	uint32_t DFIODTCFG;
	uint32_t DFIODTCFG1;
	uint32_t DFIODTRANKMAP;
	/* DFI Write Data Registers */
	uint32_t DFITPHYWRDATA;
	uint32_t DFITPHYWRLAT;
	uint32_t DFITPHYWRDATALAT;
	/* DFI Read Data Registers */
	uint32_t DFITRDDATAEN;
	uint32_t DFITPHYRDLAT;
	/* DFI Update Registers */
	uint32_t DFITPHYUPDTYPE0;
	uint32_t DFITPHYUPDTYPE1;
	uint32_t DFITPHYUPDTYPE2;
	uint32_t DFITPHYUPDTYPE3;
	uint32_t DFITCTRLUPDMIN;
	uint32_t DFITCTRLUPDMAX;
	uint32_t DFITCTRLUPDDLY;
	uint32_t DFIUPDCFG;
	uint32_t DFITREFMSKI;
	uint32_t DFITCTRLUPDI;
	/* DFI Status Registers */
	uint32_t DFISTCFG0;
	uint32_t DFISTCFG1;
	uint32_t DFITDRAMCLKEN;
	uint32_t DFITDRAMCLKDIS;
	uint32_t DFISTCFG2;
	/* DFI Low Power Register */
	uint32_t DFILPCFG0;
} PCTL_SAVE_REG_T;

typedef struct DDRPHY_SAVE_REG_Tag {
	uint32_t PHY_REG0;
	uint32_t PHY_REG1;
	uint32_t PHY_REGb;
	uint32_t PHY_REGc;
	uint32_t PHY_REG11;
	uint32_t PHY_REG13;
	uint32_t PHY_REG14;
	uint32_t PHY_REG16;
	uint32_t PHY_REG20;
	uint32_t PHY_REG21;
	uint32_t PHY_REG26;
	uint32_t PHY_REG27;
	uint32_t PHY_REG28;
	uint32_t PHY_REG30;
	uint32_t PHY_REG31;
	uint32_t PHY_REG36;
	uint32_t PHY_REG37;
	uint32_t PHY_REG38;
	uint32_t PHY_REG40;
	uint32_t PHY_REG41;
	uint32_t PHY_REG46;
	uint32_t PHY_REG47;
	uint32_t PHY_REG48;
	uint32_t PHY_REG50;
	uint32_t PHY_REG51;
	uint32_t PHY_REG56;
	uint32_t PHY_REG57;
	uint32_t PHY_REG58;
	uint32_t PHY_REGDLL;
	uint32_t PHY_REGec;
	uint32_t PHY_REGed;
	uint32_t PHY_REGee;
	uint32_t PHY_REGef;
	uint32_t PHY_REGfb;
	uint32_t PHY_REGfc;
	uint32_t PHY_REGfd;
	uint32_t PHY_REGfe;
} DDRPHY_SAVE_REG_T;

typedef struct BACKUP_REG_Tag {
	uint32_t tag;
	/* any addr = 0xFFFFFFFF, indicate invalid */
	uint32_t pctlAddr;
	PCTL_SAVE_REG_T pctl;
	uint32_t phyAddr;
	DDRPHY_SAVE_REG_T phy;
	uint32_t nocAddr;
	MSCH_SAVE_REG   noc;
	uint32_t pllselect;
	uint32_t phyPLLlockaddr;
	uint32_t phyplllockmask;
	uint32_t phyplllockval;

	uint32_t pllpdstat;

	uint32_t dpllmodeAddr;
	uint32_t dpllSlowMode;
	uint32_t dpllNormalMode;
	uint32_t dpllResetAddr;
	uint32_t dpllReset;
	uint32_t dpllDeReset;
	uint32_t dpllConAddr;
	uint32_t dpllCon[4];
	uint32_t dpllLockAddr;
	uint32_t dpllLockMask;
	uint32_t dpllLockVal;

	uint32_t ddrPllSrcDivAddr;
	uint32_t ddrPllSrcDiv;
	uint32_t retenDisAddr;
	uint32_t retenDisVal;
	/* ddr relative grf register */
	uint32_t grfregAddr;
	uint32_t grfddrcreg;
	/* pctl phy soft reset func */
	uint32_t crupctlPhySoftrstAddr;
	uint32_t cruResetPctlPhy;
	uint32_t cruDeresetPhy;
	uint32_t cruDeresetPctlPhy;
	uint32_t phySoftrstAddr;
	uint32_t endTag;
} BACKUP_REG_T;

void ddr_copy(uint32_t *pDest, uint32_t *pSrc, uint32_t words)
{
	uint32_t i;

	for (i = 0; i < words; i++)
		pDest[i] = pSrc[i];
}

static uint32_t ddr_get_phy_pll_freq(void)
{
	uint32_t ret = 0;
	uint32_t fb_div, pre_div;

	fb_div = (pPHY_Reg->PHY_REGec & 0xff) |
		 ((pPHY_Reg->PHY_REGed & 0x1) << 8);
	pre_div = pPHY_Reg->PHY_REGee & 0xff;
	ret = 2 * 24 * fb_div / (4 * pre_div);

	return ret;
}

static void ddr_get_dpll_cfg(uint32_t *p)
{
	uint32_t nMHz;
	uint32_t NO, NF, NR;

	nMHz = ddr_get_phy_pll_freq();

	if (nMHz <= 150)
		NO = 6;
	else if (nMHz <= 250)
		NO = 4;
	else if (nMHz <= 500)
		NO = 2;
	else
		NO = 1;

	NR = 1;
	NF = 2 * nMHz * NR * NO / 24;
	p[0] = SET_NR(NR) | SET_NO(NO);
	p[1] = SET_NF(NF);
	p[2] = SET_NB(NF / 2);
}

/* p_ddr_reg:save reg memory first addr */
void ddr_reg_save(uint32_t pllpdstat, uint64_t base_addr)
{
	BACKUP_REG_T *p_ddr_reg = (BACKUP_REG_T *)base_addr;
	uint32_t *p;

	p_ddr_reg->tag = DDR_TAG;
	p_ddr_reg->pctlAddr = ddrpctl_addr_phy;
	p_ddr_reg->phyAddr = ddrphy_addr_phy;
	p_ddr_reg->nocAddr = noc_addr_phy;

	/* PCTLR */
	ddr_copy((uint32_t *)&p_ddr_reg->pctl.pctl_timing.togcnt1u,
		 (uint32_t *)&pDDR_Reg->TOGCNT1U,
		 35);

	p_ddr_reg->pctl.pctl_timing.trefi |= BIT(31);
	p_ddr_reg->pctl.SCFG = pDDR_Reg->SCFG.d32;
	p_ddr_reg->pctl.CMDTSTATEN = pDDR_Reg->CMDTSTATEN;
	p_ddr_reg->pctl.MCFG1 = pDDR_Reg->MCFG1;
	p_ddr_reg->pctl.MCFG = pDDR_Reg->MCFG;
	p_ddr_reg->pctl.PPCFG = pDDR_Reg->PPCFG;
	p_ddr_reg->pctl.pctl_timing.ddrFreq = pDDR_Reg->TOGCNT1U * 2;
	p_ddr_reg->pctl.DFITCTRLDELAY = pDDR_Reg->DFITCTRLDELAY;
	p_ddr_reg->pctl.DFIODTCFG = pDDR_Reg->DFIODTCFG;
	p_ddr_reg->pctl.DFIODTCFG1 = pDDR_Reg->DFIODTCFG1;
	p_ddr_reg->pctl.DFIODTRANKMAP = pDDR_Reg->DFIODTRANKMAP;
	p_ddr_reg->pctl.DFITPHYWRDATA = pDDR_Reg->DFITPHYWRDATA;
	p_ddr_reg->pctl.DFITPHYWRLAT = pDDR_Reg->DFITPHYWRLAT;
	p_ddr_reg->pctl.DFITPHYWRDATALAT = pDDR_Reg->DFITPHYWRDATALAT;
	p_ddr_reg->pctl.DFITRDDATAEN = pDDR_Reg->DFITRDDATAEN;
	p_ddr_reg->pctl.DFITPHYRDLAT = pDDR_Reg->DFITPHYRDLAT;
	p_ddr_reg->pctl.DFITPHYUPDTYPE0 = pDDR_Reg->DFITPHYUPDTYPE0;
	p_ddr_reg->pctl.DFITPHYUPDTYPE1 = pDDR_Reg->DFITPHYUPDTYPE1;
	p_ddr_reg->pctl.DFITPHYUPDTYPE2 = pDDR_Reg->DFITPHYUPDTYPE2;
	p_ddr_reg->pctl.DFITPHYUPDTYPE3 = pDDR_Reg->DFITPHYUPDTYPE3;
	p_ddr_reg->pctl.DFITCTRLUPDMIN = pDDR_Reg->DFITCTRLUPDMIN;
	p_ddr_reg->pctl.DFITCTRLUPDMAX = pDDR_Reg->DFITCTRLUPDMAX;
	p_ddr_reg->pctl.DFITCTRLUPDDLY = pDDR_Reg->DFITCTRLUPDDLY;

	p_ddr_reg->pctl.DFIUPDCFG = pDDR_Reg->DFIUPDCFG;
	p_ddr_reg->pctl.DFITREFMSKI = pDDR_Reg->DFITREFMSKI;
	p_ddr_reg->pctl.DFITCTRLUPDI = pDDR_Reg->DFITCTRLUPDI;
	p_ddr_reg->pctl.DFISTCFG0 = pDDR_Reg->DFISTCFG0;
	p_ddr_reg->pctl.DFISTCFG1 = pDDR_Reg->DFISTCFG1;
	p_ddr_reg->pctl.DFITDRAMCLKEN = pDDR_Reg->DFITDRAMCLKEN;
	p_ddr_reg->pctl.DFITDRAMCLKDIS = pDDR_Reg->DFITDRAMCLKDIS;
	p_ddr_reg->pctl.DFISTCFG2 = pDDR_Reg->DFISTCFG2;
	p_ddr_reg->pctl.DFILPCFG0 = pDDR_Reg->DFILPCFG0;

	/* PHY */
	p_ddr_reg->phy.PHY_REG0 = pPHY_Reg->PHY_REG0;
	p_ddr_reg->phy.PHY_REG1 = pPHY_Reg->PHY_REG1;
	p_ddr_reg->phy.PHY_REGb = pPHY_Reg->PHY_REGb;
	p_ddr_reg->phy.PHY_REGc = pPHY_Reg->PHY_REGc;
	p_ddr_reg->phy.PHY_REG11 = pPHY_Reg->PHY_REG11;
	p_ddr_reg->phy.PHY_REG13 = pPHY_Reg->PHY_REG13;
	p_ddr_reg->phy.PHY_REG14 = pPHY_Reg->PHY_REG14;
	p_ddr_reg->phy.PHY_REG16 = pPHY_Reg->PHY_REG16;
	p_ddr_reg->phy.PHY_REG20 = pPHY_Reg->PHY_REG20;
	p_ddr_reg->phy.PHY_REG21 = pPHY_Reg->PHY_REG21;
	p_ddr_reg->phy.PHY_REG26 = pPHY_Reg->PHY_REG26;
	p_ddr_reg->phy.PHY_REG27 = pPHY_Reg->PHY_REG27;
	p_ddr_reg->phy.PHY_REG28 = pPHY_Reg->PHY_REG28;
	p_ddr_reg->phy.PHY_REG30 = pPHY_Reg->PHY_REG30;
	p_ddr_reg->phy.PHY_REG31 = pPHY_Reg->PHY_REG31;
	p_ddr_reg->phy.PHY_REG36 = pPHY_Reg->PHY_REG36;
	p_ddr_reg->phy.PHY_REG37 = pPHY_Reg->PHY_REG37;
	p_ddr_reg->phy.PHY_REG38 = pPHY_Reg->PHY_REG38;
	p_ddr_reg->phy.PHY_REG40 = pPHY_Reg->PHY_REG40;
	p_ddr_reg->phy.PHY_REG41 = pPHY_Reg->PHY_REG41;
	p_ddr_reg->phy.PHY_REG46 = pPHY_Reg->PHY_REG46;
	p_ddr_reg->phy.PHY_REG47 = pPHY_Reg->PHY_REG47;
	p_ddr_reg->phy.PHY_REG48 = pPHY_Reg->PHY_REG48;
	p_ddr_reg->phy.PHY_REG50 = pPHY_Reg->PHY_REG50;
	p_ddr_reg->phy.PHY_REG51 = pPHY_Reg->PHY_REG51;
	p_ddr_reg->phy.PHY_REG56 = pPHY_Reg->PHY_REG56;
	p_ddr_reg->phy.PHY_REG57 = pPHY_Reg->PHY_REG57;
	p_ddr_reg->phy.PHY_REG58 = pPHY_Reg->PHY_REG58;
	p_ddr_reg->phy.PHY_REGDLL = pPHY_Reg->PHY_REGDLL;
	p_ddr_reg->phy.PHY_REGec = pPHY_Reg->PHY_REGec;
	p_ddr_reg->phy.PHY_REGed = pPHY_Reg->PHY_REGed;
	p_ddr_reg->phy.PHY_REGee = pPHY_Reg->PHY_REGee;
	p_ddr_reg->phy.PHY_REGef = 0;

	if (pPHY_Reg->PHY_REG2 & 0x2) {
		p = (uint32_t *)pPHY_Reg;
		p_ddr_reg->phy.PHY_REGfb = p[0xb0 / 4];
		p_ddr_reg->phy.PHY_REGfc = p[0xf0 / 4];
		p_ddr_reg->phy.PHY_REGfd = p[0x130 / 4];
		p_ddr_reg->phy.PHY_REGfe = p[0x170 / 4];
	} else {
		p_ddr_reg->phy.PHY_REGfb = pPHY_Reg->PHY_REGfb;
		p_ddr_reg->phy.PHY_REGfc = pPHY_Reg->PHY_REGfc;
		p_ddr_reg->phy.PHY_REGfd = pPHY_Reg->PHY_REGfd;
		p_ddr_reg->phy.PHY_REGfe = pPHY_Reg->PHY_REGfe;
	}

	/* NOC */
	p_ddr_reg->noc.ddrconf = pMSCH_Reg->ddrconf;
	p_ddr_reg->noc.ddrtiming.d32 = pMSCH_Reg->ddrtiming.d32;
	p_ddr_reg->noc.ddrmode = pMSCH_Reg->ddrmode;
	p_ddr_reg->noc.readlatency = pMSCH_Reg->readlatency;
	p_ddr_reg->noc.activate.d32 = pMSCH_Reg->activate.d32;
	p_ddr_reg->noc.devtodev = pMSCH_Reg->devtodev;

	p_ddr_reg->pllselect = pPHY_Reg->PHY_REGef & 0x1;

	p_ddr_reg->phyPLLlockaddr = GRF_BASE + GRF_SOC_STATUS0;
	p_ddr_reg->phyplllockmask = BIT(15);
	p_ddr_reg->phyplllockval = 0;

	/* PLLPD */
	p_ddr_reg->pllpdstat = pllpdstat;
	/* DPLL */
	p_ddr_reg->dpllmodeAddr = CRU_BASE + PLL_CONS(DPLL_ID, 3);
	p_ddr_reg->dpllSlowMode = BITS_WITH_WMASK(0x0, 0x3, 8) |
				  BITS_WITH_WMASK(0x0, 0x1, 1);
	p_ddr_reg->dpllNormalMode = BITS_WITH_WMASK(0x1, 0x3, 8);
	p_ddr_reg->dpllResetAddr = CRU_BASE + PLL_CONS(DPLL_ID, 3);
	p_ddr_reg->dpllReset = BITS_WITH_WMASK(0x1, 0x1, 5);
	p_ddr_reg->dpllDeReset = BITS_WITH_WMASK(0x0, 0x1, 5);
	p_ddr_reg->dpllConAddr = CRU_BASE + PLL_CONS(DPLL_ID, 0);
	if (p_ddr_reg->pllselect == 0) {
		p_ddr_reg->dpllCon[0] =
				mmio_read_32(CRU_BASE + PLL_CONS(DPLL_ID, 0)) |
				0xffff0000;
		p_ddr_reg->dpllCon[1] =
			mmio_read_32(CRU_BASE + PLL_CONS(DPLL_ID, 1)) & 0xffff;
		p_ddr_reg->dpllCon[2] =
			mmio_read_32(CRU_BASE + PLL_CONS(DPLL_ID, 2)) & 0xffff;
		p_ddr_reg->dpllCon[3] =
				mmio_read_32(CRU_BASE + PLL_CONS(DPLL_ID, 3)) |
				0xffff0000;
	} else {
		ddr_get_dpll_cfg(&p_ddr_reg->dpllCon[0]);
	}
	p_ddr_reg->pllselect = 0;
	p_ddr_reg->dpllLockAddr = CRU_BASE + PLL_CONS(DPLL_ID, 1);
	p_ddr_reg->dpllLockMask = PLL_LOCK_MSK;
	p_ddr_reg->dpllLockVal = PLL_LOCK_MSK;

	/* SET_DDR_PLL_SRC */
	p_ddr_reg->ddrPllSrcDivAddr = CRU_BASE + CRU_CLKSELS_CON(13);
	p_ddr_reg->ddrPllSrcDiv =
		BITS_WITH_WMASK(mmio_read_32(CRU_BASE + CRU_CLKSELS_CON(13)) &
				0x13,
				0x13,
				0);

	p_ddr_reg->retenDisAddr = PMU_BASE + PMU_PWRMD_COM;
	p_ddr_reg->retenDisVal = BIT(pmu_mode_ddrio_ret_deq);

	p_ddr_reg->grfregAddr = GRF_BASE + GRF_DDRC0_CON0;
	p_ddr_reg->grfddrcreg =
		BITS_WITH_WMASK(mmio_read_32(GRF_BASE + GRF_DDRC0_CON0) & 0x1c,
				0x1c,
				0);

	/* pctl phy soft reset */
	p_ddr_reg->crupctlPhySoftrstAddr = CRU_BASE + CRU_SOFTRSTS_CON(10);
	p_ddr_reg->cruResetPctlPhy = ddrctrl0_psrstn_req(1) |
				     ddrctrl0_srstn_req(1) |
				     ddrphy0_psrstn_req(1) |
				     ddrphy0_srstn_req(1);
	p_ddr_reg->cruDeresetPhy = ddrctrl0_psrstn_req(1) |
				   ddrctrl0_srstn_req(1) |
				   ddrphy0_psrstn_req(0) |
				   ddrphy0_srstn_req(0);
	p_ddr_reg->cruDeresetPctlPhy = ddrctrl0_psrstn_req(0) |
				       ddrctrl0_srstn_req(0) |
				       ddrphy0_psrstn_req(0) |
				       ddrphy0_srstn_req(0);
	p_ddr_reg->phySoftrstAddr = (uint64_t)&pPHY_Reg->PHY_REG0;

	p_ddr_reg->endTag = DDR_END_TAG;
}

static __aligned(4) unsigned int ddr_reg_resume[] = {
	#include "rk3368_ddr_reg_resume_V1.05.bin"
};

unsigned int *ddr_get_resume_code_base(void)
{
	return (unsigned int *)ddr_reg_resume;
}

uint32_t ddr_get_resume_code_size(void)
{
	return sizeof(ddr_reg_resume);
}

uint32_t ddr_get_resume_data_size(void)
{
	return sizeof(BACKUP_REG_T);
}

void rockchip_ddr_resume_init(void)
{
	pDDR_Reg = (pDDR_REG_T)DDR_PCTL_BASE;
	ddrpctl_addr_phy = DDR_PCTL_BASE;

	pPHY_Reg = (pDDRPHY_REG_T)DDR_PHY_BASE;
	ddrphy_addr_phy = DDR_PHY_BASE;

	pMSCH_Reg = (pMSCH_REG)SERVICE_BUS_BASE;
	noc_addr_phy = SERVICE_BUS_BASE;
}

