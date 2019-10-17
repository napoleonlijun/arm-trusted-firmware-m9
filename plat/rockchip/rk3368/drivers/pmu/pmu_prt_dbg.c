#ifndef	PRT_DBG_C
#define PRT_DBG_C

static inline void print_hex(uint32_t val)
{
	int i;
	unsigned char tmp;

	putchar('0');
	putchar('x');
	for (i = 0; i < 8; val <<= 4, ++i) {
		tmp = (val & 0xf0000000) >> 28;
		if (tmp < 10)
			putchar('0' + tmp);
		else
			putchar('a' + tmp - 10);
	}
}

static void regs_dump(int32_t base,
		      uint32_t start_offset,
		      uint32_t end_offset)
{
	uint32_t i;

	for (i = start_offset; i <= end_offset; i += 4) {
		if (i % 32 == 0)
			tf_printf("\n0x%x: ", base + i);
		print_hex(mmio_read_32(base + i));
		putchar(' ');
		putchar(' ');
		putchar(' ');
		putchar(' ');
	}
	tf_printf("\n");
}

#define GICD_ISPENDR		0x200

static uint32_t GPIO_BASE[] = {
	PMU_GPIO0_BASE,
	GPIO1_BASE,
	GPIO2_BASE,
	GPIO3_BASE,
};

#define DUMP_GPIO_INTEN(ID) \
	do { \
		uint32_t en;\
		en = mmio_read_32(GPIO_BASE[ID] + GPIO_INTEN); \
		if (en) { \
			tf_printf("GPIO%d_INTEN: 0x%x\n", ID, en); \
		} \
	} while (0)

static inline void dbg_pm_dump_inten(void)
{
	DUMP_GPIO_INTEN(0);
	DUMP_GPIO_INTEN(1);
	DUMP_GPIO_INTEN(2);
	DUMP_GPIO_INTEN(3);
}

static inline int fls(int x)
{
	int r = 32;

	if (!x)
		return 0;
	if (!(x & 0xffff0000u)) {
		x <<= 16;
		r -= 16;
	}
	if (!(x & 0xff000000u)) {
		x <<= 8;
		r -= 8;
	}
	if (!(x & 0xf0000000u)) {
		x <<= 4;
		r -= 4;
	}
	if (!(x & 0xc0000000u)) {
		x <<= 2;
		r -= 2;
	}
	if (!(x & 0x80000000u)) {
		x <<= 1;
		r -= 1;
	}
	return r;
}

static inline void dbg_pm_dump_irq(void)
{
	uint32_t gic_ispendr, irq_gpio, val, i, bank, pin;
	uint32_t irqs[4], irq = 0;
	static const char * const str_bank[] = {"a", "b", "c", "d"};

	gic_ispendr = mmio_read_32(RK3368_GICD_BASE + GICD_ISPENDR + 12);
	irq_gpio = (gic_ispendr >> 17) & 0x1ff;

	for (i = 0; i < 4; i++) {
		irqs[i] = mmio_read_32(RK3368_GICD_BASE +
				       GICD_ISPENDR +
				       (1 + i) * 4);
		INFO("irq%d: 0x%x\n", i, irqs[i]);
	}

	for (i = 0; i < 4; i++) {
		if (irqs[i]) {
			irq = 32 * (i + 1) + fls(irqs[i]) - 1;
			break;
		}
	}

	for (i = 0; i < 4; i++) {
		if (irq_gpio & (1 << i)) {
			val = mmio_read_32(GPIO_BASE[i] + GPIO_INT_STATUS);
			bank = (fls(val) - 1) / 8;
			pin = (fls(val) - 1) % 8;
			tf_printf("irq:%d, gpio%d_%s%d\n",
				  irq, i, str_bank[bank], pin);
			break;
		}
	}
}

static void dbg_pmu_sleep_enter_info(uint32_t cfg)
{
	INFO("\nsleep enter:");
#if 0
	INFO("CRU enter\n");
	regs_dump(CRU_BASE, 0x0, 0x400);
#endif
	INFO("enter: cfg=0x%x, sleeptimes:%d\n", cfg, rksleep_data.sleep_cnt++);

	if (cfg & SLP_SFT_32K_EXT)
		INFO("slp_sft_32k_ext\n");
	else
		INFO("slp_sft_32k_int\n");

	if (cfg & SLP_PMU_PMUALIVE_32K)
		INFO("pmu_pmualive_32k\n");

	if (cfg & SLP_PMU_PLLS_PWRDN)
		INFO("pmu_pll_pwrdn\n");

	if (cfg & SLP_SFT_PLLS_DEEP)
		INFO("sft_pll_deep\n");

	if (cfg & SLP_PMU_DIS_OSC)
		INFO("pmu_dis_osc\n");

	if (cfg & SLP_SFT_PD_NBSCUS)
		INFO("SLP_SFT_PD_NBSCUS\n");

	if (cfg & SLP_SFT_PD_PERI)
		INFO("slp_sft_pd_peri\n");
}

static void dbg_pmu_sleep_mode_tst_info(uint32_t cfg)
{
	if (cfg & SLP_SFT_PD_PERI)
		return;

	INFO("ddr flag: 0x%x\n", psram_sleep_cfg->ddr_flag);

	INFO("(%x %x %x)\n",
	     mmio_read_32(PMU_BASE + PMU_PWRMD_CORE),
	     mmio_read_32(PMU_BASE + PMU_PWRMD_COM),
	     mmio_read_32(PMU_BASE + PMU_PWRDN_ST));

	INFO("stable=%d, wakeuprst=%d, osccnt=%d\n"
	     "ddrio=%d, pllrst=%d, plllock=%d\n",
	     mmio_read_32(PMU_BASE + PMU_STABLE_CNT),
	     mmio_read_32(PMU_BASE + PMU_WKUPRST_CNT),
	     mmio_read_32(PMU_BASE + PMU_OSC_CNT),
	     mmio_read_32(PMU_BASE + PMU_DDRIO_PWR_CNT),
	     mmio_read_32(PMU_BASE + PMU_PLLRST_CNT),
	     mmio_read_32(PMU_BASE + PMU_PLLLOCK_CNT));
	/*
	 * debug setting: core timeout autowakeup
	 *
	 */
#if 0
	regs_updata_bit_set(PMU_BASE + RK3368_PMU_WKUP_CFG2,
			    pmu_wkupcfg2_timeout);
	mmio_write_32(PMU_BASE + PMU_TIMEOUT_CNT, SLP_USE_TST_TIMEOUT_CNT);
	INFO("%s: wkup_cfg2=0x%x\n",
	     __func__,
	     mmio_read_32(PMU_BASE + RK3368_PMU_WKUP_CFG2));
	INFO("%s: pmu_timeout_cnt=0x%x\n",
	     __func__,
	     mmio_read_32(PMU_BASE + PMU_TIMEOUT_CNT));
#endif
	/*
	 * config pmu_debug pin
	 *
	 */
#if 0
	pin_set_fun(0x0, 0xa, 0x3, 0x2);
	pin_set_fun(0x0, 0xb, 0x0, 0x3);
	pin_set_fun(0x0, 0xd, 0x4, 0x3);
	pin_set_fun(0x0, 0xd, 0x5, 0x3);
	pin_set_fun(0x0, 0xd, 0x6, 0x3);
	pin_set_fun(0x0, 0xd, 0x7, 0x3);
#endif

#if 0
	{
		int i;

		for (i = 0; i < CRU_CLKGATES_CON_CNT; i++)
			mmio_write_32(CRU_BASE + CRU_CLKGATES_CON(i),
				      0xffff0000);
		for (i = 0; i < CRU_CLKGATES_CON_CNT; i++)
			tf_printf("gate_con_%d:0x%x\n",
				  i,
				  mmio_read_32(CRU_BASE + CRU_CLKGATES_CON(i)));
	}
#endif

	/*
	 * dump reg log
	 *
	 */
	if (0) {
		regs_dump(PMU_BASE, 0x0, 0x7c);
		regs_dump(CRU_BASE, 0x0, 0x5c);
		regs_dump(CRU_BASE, 0x100, 0x1dc);
		regs_dump(CRU_BASE, 0x200, 0x260);
	}
}

void dbg_pmu_resume_info(void)
{
	INFO("sleep exit\n");
#if 0
	INFO("CRU EXIT\n");
	regs_dump(CRU_BASE, 0x0, 0x400);
#endif
}

#if 0
#define DEBUG_SLEEP_MODE_CFG 1
static uint32_t prt_get_sys_sleep_mode_cfg(void)
{
	uint32_t slp_cfg = 0;

#if DEBUG_SLEEP_MODE_CFG
	/* slp_cfg |= SLP_ARMOFF; */
	slp_cfg |= SLP_ARMOFF_LOGPD;
	/* slp_cfg |= SLP_ARMOFF_LOGOFF; */

	if (slp_cfg & SLP_WFI) {
		slp_cfg |= SLP_SFT_PLLS_DEEP
			| SLP_SFT_PD_NBSCUS
			;
	} else if (slp_cfg & SLP_ARMOFF) {
		slp_cfg |= SLP_PMU_PLLS_PWRDN
			| SLP_PMU_PMUALIVE_32K
			| SLP_SFT_PLLS_DEEP
			| SLP_PMU_DIS_OSC
			| SLP_SFT_PD_NBSCUS
			/* | SLP_SFT_PD_PERI */
				;
	} else if (slp_cfg & SLP_ARMOFF_LOGPD) {
		slp_cfg |= 0 | SLP_PMU_PLLS_PWRDN
			| SLP_PMU_PMUALIVE_32K
			| SLP_SFT_PLLS_DEEP
			| SLP_PMU_DIS_OSC
			| SLP_SFT_PD_NBSCUS
			/* | SLP_SFT_PD_PERI */
				;
	} else if (slp_cfg & SLP_ARMOFF_LOGOFF) {
		slp_cfg |= SLP_PMU_PLLS_PWRDN
			| SLP_PMU_PMUALIVE_32K
			| SLP_SFT_PLLS_DEEP
			| SLP_PMU_DIS_OSC
			;
	}

	/*
	 * check dis osc condition
	 */
	if (slp_cfg & SLP_PMU_DIS_OSC) {
		if ((slp_cfg & SLP_PMU_DIS_OSC_CDT) != SLP_PMU_DIS_OSC_CDT) {
			slp_cfg &= ~SLP_PMU_DIS_OSC;
			WARN("%s: dis osc condition is not enough\n", __func__);
		}
	}
#else /* default config */
	slp_cfg = SLP_ARMOFF
		| SLP_PMU_PLLS_PWRDN
		| SLP_SFT_PD_NBSCUS
		;
#endif

	return slp_cfg;
}
#endif
#endif
