
#define RK322XH_SUSPEND_DEBUG	0
int skip_suspend;

/*******************************************************************************
 * code for dbg
 ******************************************************************************/

#if RK322XH_SUSPEND_DEBUG
static inline void print_hex(uint32_t val)
{
	int i;
	unsigned char tmp;

	putchar('0');
	putchar('x');
	for (i = 0; i < 8; val <<= 4, ++i) {
		tmp = val & 0xf0000000) >> 28;
		if (tmp < 10)
			putchar('0' + tmp);
		else
			putchar('a' + tmp - 10);
	}
}

static void regs_dump(uint32_t base, uint32_t start_offset, uint32_t end_offset)
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
}
#endif

#if 0
static void sys_slp_config(void)
{
	uint32_t slp_mode_cfg = 0;

	/* pmu_debug set */
	ddr_data.debug_iomux_save = mmio_read_32(GRF_BASE + GRF_GPIO2A_IOMUX);
	if (ddr_data.pmu_debug_enable) {
		mmio_write_32(GRF_BASE + GRF_GPIO2A_IOMUX,
			      BITS_WITH_WMASK(2, 3, 0) |
			      BITS_WITH_WMASK(2, 3, 2) |
			      BITS_WITH_WMASK(2, 3, 4) |
			      BITS_WITH_WMASK(2, 3, 6));
	}

	/* sleep wakeup set */
	ddr_data.pmu_wakeup_conf0 = mmio_read_32(PMU_BASE + PMU_WAKEUP_CFG0);
	mmio_write_32(PMU_BASE + PMU_WAKEUP_CFG0,  WAKEUP_INT_CLUSTER_EN);

	/* sleep mode set */
	ddr_data.pmu_pwrmd_com = mmio_read_32(PMU_BASE + PMU_PWRMD_COM);
	slp_mode_cfg = BIT(pmu_mode_en) |
			BIT(global_int_disable_cfg) |
			BIT(cpu0_pd_en) |
			BIT(wait_wakeup_begin_cfg)
			;
	mmio_write_32(PMU_BASE + PMU_PWRMD_COM, slp_mode_cfg);

	/* boot address set */
	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(1),
		      (PMUSRAM_BASE >> CPU_BOOT_ADDR_ALIGN) |
		      CPU_BOOT_ADDR_WMASK);
}

static void sys_slp_unconfig(void)
{
	/* set boot address */
	mmio_write_32(SGRF_BASE + SGRF_SOC_CON(1),
		      (cpu_warm_boot_addr >> CPU_BOOT_ADDR_ALIGN) |
		      CPU_BOOT_ADDR_WMASK);

	/* sleep mode */
	mmio_write_32(PMU_BASE + PMU_PWRMD_COM, ddr_data.pmu_pwrmd_com);

	/* sleep wakeup */
	mmio_write_32(PMU_BASE + PMU_WAKEUP_CFG0, ddr_data.pmu_wakeup_conf0);

	/* pmu_debug set */
	if (ddr_data.pmu_debug_enable)
		mmio_write_32(GRF_BASE + GRF_GPIO2A_IOMUX,
			      ddr_data.debug_iomux_save |
			      BITS_WMSK(0xffff, 0));
}
#endif

static inline void dbg_clks_gating_suspend(uint32_t *ungt_msk)
{
#if RK322XH_SUSPEND_DEBUG
		INFO("%d  ", i);
		INFO("save: 0x%x ",
		     mmio_read_32(CRU_BASE + CRU_CLKGATE_CON(i)));
		INFO("msk: 0x%x ", ungt_msk[i]);
#endif
		//mmio_write_32(CRU_BASE + CRU_CLKGATE_CON(i), ((~ungt_msk[i]) << 16) | 0xffff);
		//CLK_UNGATING(ungt_msk[i], CRU_CLKGATE_CON(i));
		/* mmio_write_32(CRU_BASE + CRU_CLKGATE_CON(i), 0xffff0000); */
#if RK322XH_SUSPEND_DEBUG
		INFO("gating: 0x%x\n", mmio_read_32(CRU_BASE + CRU_CLKGATE_CON(i)));
#endif
}

static void dbg_pm_plls_suspend(void)
{
#if RK322XH_SUSPEND_DEBUG
	int i, j;

	for (i = 0; i < MAX_PLL; ++i) {
		tf_printf("pll%d:", i);
		for (j = 0; j < CRU_PLL_CON_NUMS; ++j)
			tf_printf("0x%x ",
				  mmio_read_32(CRU_BASE + PLL_CONS(i, j)));

		tf_printf("\n");
	}
#endif
}

static inline void dbg_pm_plls_resume(void)
{
#if RK322XH_SUSPEND_DEBUG
	int i, j;

	for (i = 0; i < MAX_PLL; ++i) {
		tf_printf("pll%d:", i);

		for (j = 0; j < CRU_PLL_CON_NUMS; ++j)
			tf_printf("0x%x ",
				  mmio_read_32(CRU_BASE + PLL_CONS(i, j)));

		tf_printf("\n");
	}
#endif
}

#define GICD_ISPEND		0x200
#define GPIO_NUMS		4
#define GPIO_INTEN		0x30
#define GPIO_INT_STATUS		0x40

#define RK_GPIO_VIRT(i)		(GPIO0_BASE + (i) * (GPIO1_BASE- GPIO0_BASE))

#define DUMP_GPIO_INTEN(ID) \
	do { \
		uint32_t en = mmio_read_32(RK_GPIO_VIRT(ID) + GPIO_INTEN); \
		if (en) { \
			tf_printf("GPIO%d_INTEN: 0x%x\n", ID, en); \
		} \
	} while (0)

static inline int fls(int x)
{
	int ret;

	__asm volatile("clz\t%0, %1" : "=r" (ret) : "r" (x));

	ret = 64 - ret;
	return ret;
}

static void dbg_report_irq(void)
{
	uint32_t irq[4], val, i;
	uint32_t irq_gpio;
	uint32_t pin, gpio_st;
	static const char *bank[] = {"a", "b", "c", "d"};

	/* gpio0~3, irq: 83, 84, 85, 86 */
	val = mmio_read_32((GIC400_BASE + 0x1000) + GICD_ISPEND + 8);
	irq_gpio = (val >> 19) & 0x0f;

	for (i = 0; i < ARRAY_SIZE(irq); i++)
		irq[i] = mmio_read_32((GIC400_BASE + 0x1000) +
				       GICD_ISPEND + (i + 1) * 4);

	for (i = 0; i < ARRAY_SIZE(irq); i++) {
		for (; irq[i]; ) {
			tf_printf(" IRQ: %d", 32 * (i + 1) + fls(irq[i]) - 1);
			irq[i] &= ~BIT(fls(irq[i]) - 1);
		}
	}

	for (i = 0; i < GPIO_NUMS; i++) {
		if (irq_gpio & (1 << i)) {
			gpio_st = mmio_read_32(RK_GPIO_VIRT(i) +
					       GPIO_INT_STATUS);
			for (; gpio_st; ) {
				pin = fls(gpio_st) - 1;
				tf_printf("  gpio%d_%s%d",
					  i, bank[pin / 8], pin % 8);
				gpio_st &= ~BIT(pin);
			}
		}
	}

	putchar('\n');
}

static void dbg_rk322x_irq_prepare(void)
{
	DUMP_GPIO_INTEN(0);
	DUMP_GPIO_INTEN(1);
	DUMP_GPIO_INTEN(2);
	DUMP_GPIO_INTEN(3);
	dbg_report_irq();
}

static void dbg_rk322x_irq_finish(void)
{
	dbg_report_irq();
}

void bl31_plat_runtime_setup(void)
{
	/* do nothing */
}

#if 0
static __sramfunc void dbg_ddr_idle_req(void)
{
	uint32_t status1;
	return;
	mmio_write_32(GRF_BASE + GRF_SOC_CON(5),  BIT_WITH_WMSK(3));
	dsb();
	do {
		status1 = mmio_read_32(GRF_BASE + GRF_SOC_STATUS(1));
		status1 &= (BIT(3) | BIT(13));
	} while (status1 != (BIT(3) | BIT(13)));
}
#endif

