/*
 * T19XX chip specific  clocks setup
 *
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <asm/mach/map.h>

#include <mach/hardware.h>
//#include <mach/clock.h>

#include <mach/irqs.h>
//#include <mach/psc.h>
//#include <mach/mux.h>

#include <mach/clock.h>



/*
 * Device specific clocks
 */
 #ifdef CONFIG_T19XX_IC
#define T19XX_HREF_FREQ		192000000	//HCLK
#define T19XX_PREF_FREQ		96000000	//HCLK
#else
#define T19XX_REF_FREQ		36000000
#endif


static void t18xx_clk_set(void __iomem *reg, struct clk *clk, int enable)
{
		unsigned int ctrlbit = clk->ctlbit;
		u32 con;
		con = __raw_readl(reg);
	
		if (enable)
			con |= ctrlbit;
		else
			con &= ~ctrlbit;
	
		__raw_writel(con, reg);
	
}


static void t18xx_pclk_ctrl(struct clk *clk, int enable)
{
	//printk("t18xx clk enable entered\n");
	//return t18xx_clk_set(S3C_PCLK_GATE, clk, enable);
}

static void t18xx_hclk_ctrl(struct clk *clk, int enable)
{
	//return t18xx_clk_set(S3C_HCLK_GATE, clk, enable);
}

static void t18xx_sclk_ctrl(struct clk *clk, int enable)
{
	//return t18xx_clk_set(S3C_SCLK_GATE, clk, enable);
}


/*all peripheral except lcd & audio*/
static struct pll_data pll1_data = {
	.num       = 1,
	.phys_off = 0,
};

/* lcd & audio*/
static struct pll_data pll2_data = {
	.num       = 2,
	.phys_off = 0,
};

static struct clk hclk_clk = {
	.name = "hclk_clk",
	.rate = T19XX_HREF_FREQ,
};
static struct clk pclk_clk = {
	.name = "pclk_clk",
	.rate = T19XX_PREF_FREQ,
};

static struct clk pll1_clk = {
	.name = "pll1",
	.parent = &hclk_clk,
	.pll_data = &pll1_data,
};


static struct clk pll2_clk = {
	.name = "pll2",
	.parent = &pclk_clk,
	.pll_data = &pll2_data,
};


static struct clk vicp_clk = {
	.name = "vicp",
	.parent = &pll2_clk,
	.usecount = 1,			/* REVISIT how to disable? */
};

static struct clk vpss_master_clk = {
	.name = "vpss_master",
	.parent = &pll2_clk,
};

static struct clk vpss_slave_clk = {
	.name = "vpss_slave",
	.parent = &pll2_clk,
};

static struct clk uart0_clk = {
	.name = "uart0",
	.parent = &pll1_clk,
};

static struct clk uart1_clk = {
	.name = "uart1",
	.parent = &pll1_clk,
};


static struct clk mac_clk = {
	.name = "mac",
	.parent = &pll1_clk,
};

static struct clk ide_clk = {
	.name = "ide",
	.parent = &pll1_clk,
};

static struct clk asp_clk = {
	.name = "asp0",
	.parent = &pll1_clk,
	.ctlbit = BIT(1),
	.enable = t18xx_pclk_ctrl,
};

static struct clk nand_clk = {
	.name = "nand",
	.parent = &pll1_clk,
	.ctlbit = BIT(1),
	.enable = t18xx_pclk_ctrl,
};


static struct clk mmcsd_clk = {
	.name = "mmcsd0",
	.parent = &pll1_clk,
	.div 	= 2,
	.ctlbit = BIT(1),
	.enable = t18xx_pclk_ctrl,
};
static struct clk mmcsd_clk1 = {
	.name = "mmcsd1",
	.parent = &pll1_clk,
	.div	= 2,
	.ctlbit = BIT(1),
	.enable = t18xx_pclk_ctrl,
};

static struct clk spi_clk = {
	.name = "spi",
	.parent = &pll1_clk,
	.div	= 2,
	.ctlbit = BIT(1),
	.enable = t18xx_pclk_ctrl,
};

static struct clk irda_clk = {
	.name = "irda",
	.parent = &pll1_clk,
	.div	= 4,
	.ctlbit = BIT(1),
	.enable = t18xx_pclk_ctrl,
};


static struct clk gpio_clk = {
	.name = "gpio",
	.parent = &pll1_clk,
};

static struct clk usb_clk = {
	.name = "usb",
	.parent = &pll1_clk,
};


static struct clk aemif_clk = {
	.name = "aemif",
	.parent = &pll1_clk,
	.ctlbit = BIT(1),
};

static struct clk pwm0_clk = {
	.name = "pwm0",
	.parent = &pll1_clk,
};

static struct clk pwm1_clk = {
	.name = "pwm1",
	.parent = &pll1_clk,
};

static struct clk pwm2_clk = {
	.name = "pwm2",
	.parent = &pll1_clk,
};

static struct clk timer0_clk = {
	.name = "timer0",
	.parent = &pll1_clk,
};

static struct clk timer1_clk = {
	.name = "timer1",
	.parent = &pll1_clk,
};

static struct clk timer2_clk = {
	.name = "timer2",
	.parent = &pll1_clk,
	.usecount = 1,              /* REVISIT: why cant' this be disabled? */
};

static struct clk watchdog_clk = {
	.name = "watchdog",
	.parent = &pll1_clk,
	.ctlbit = BIT(1),
	.enable = t18xx_pclk_ctrl,
};



struct t18xx_clk t18xx_clks[] = {
	CLK(NULL, "hclk_clk", &hclk_clk),
	CLK(NULL, "pclk_clk", &pclk_clk),
	CLK(NULL, "pll1", &pll1_clk),
	CLK(NULL, "pll2", &pll2_clk),
	CLK(NULL, "spi", &spi_clk),
	CLK(NULL, "irda", &irda_clk),
	CLK("watchdog", NULL, &watchdog_clk),
	CLK("freechip_nand.0", NULL, &nand_clk),
	CLK("freechip_mmc.0", NULL, &mmcsd_clk),
	CLK("freechip_mmc.1", NULL, &mmcsd_clk1),
	CLK(NULL, "hclk", &hclk_clk),
	#if 0
	CLK(NULL, "vicp", &vicp_clk),
	CLK(NULL, "vpss_master", &vpss_master_clk),
	CLK(NULL, "vpss_slave", &vpss_slave_clk),
	CLK(NULL, "uart0", &uart0_clk),
	CLK(NULL, "uart1", &uart1_clk),
	CLK(NULL, "uart2", &uart2_clk),
	CLK("mac", NULL, &emac_clk),

	CLK(NULL, "aemif", &aemif_clk),
	CLK(NULL, "pwm0", &pwm0_clk),
	CLK(NULL, "pwm1", &pwm1_clk),
	CLK(NULL, "pwm2", &pwm2_clk),
	CLK(NULL, "timer0", &timer0_clk),
	CLK(NULL, "timer1", &timer1_clk),
	#endif
	CLK(NULL, NULL, NULL),
};



