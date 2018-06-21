
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>           /* everything... */
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/kernel.h>       /* printk() */
#include <linux/slab.h>         /* kmalloc() */
#include <asm/uaccess.h>        /* copy_*_user */
#include <mach/hardware.h>
#include <mach/clock_t18xx.h>


static u32 ahb_clk;
static  u32 apb_clk;


#define MHZ	(1000000)
#define CPU_CLK	(36 * MHZ)
#define CLK_MHZ(x)	(x/MHZ)


#ifdef CONFIG_FPGA_DEBUG
void t18xx_clock_init(void)
{
	u32 cpu_clk;
	u32 ratio=1;
	cpu_clk = CPU_CLK;

	ahb_clk = cpu_clk/ratio;
	apb_clk = ahb_clk/ratio;

	printk("T18xx fpga run @%d MHz Sdram  run @%d MHz \n", CLK_MHZ(cpu_clk), CLK_MHZ(ahb_clk));

}


#else
#define PLL0_INPUT	6
#define PLL1_INPUT	3
#define PLL2_INPUT	3


void t18xx_clock_init(void)
{
#if 0
	u32 cpu_clk,video_clk, audio_clk;
	u32 ratio=2;
	int pll0_mul, pll1_mul, pll2_mul;
	int pll0_div, pll1_div, pll2_div;

	pll0_mul = PLL0DIV & 0xff;
	pll1_mul = PLL1DIV & 0xff;
	pll2_mul = PLL2DIV & 0xff;

	pll0_div = 1<<((PLL_DIVS >> 26)& 0x3);
	pll1_div = 1<<((PLL_DIVS >> 14)& 0x3);
	pll2_div = 1<<((PLL_DIVS >> 20)& 0x3);
	printk("pll0_mul =%d, pll0_div=%d\n", pll0_mul, pll0_div);
	printk("pll1_mul =%d, pll1_div=%d\n", pll1_mul, pll1_div);
	printk("pll2_mul =%d, pll2_div=%d\n", pll2_mul, pll2_div);
	/*
	*/
	cpu_clk = PLL0_INPUT * pll0_mul /pll0_div *MHZ ;
	video_clk = PLL1_INPUT * pll1_mul /pll1_div*MHZ;
	audio_clk = PLL2_INPUT * pll2_mul /pll2_div*MHZ;

	ahb_clk = cpu_clk/ratio;
	apb_clk = ahb_clk/ratio;
	printk("T18xx PLL0 (CPU) clk =%dMhz\n",CLK_MHZ(cpu_clk));
	printk("T18xx PLL1 (Vidoe) clk =%dMhz\n",CLK_MHZ(video_clk));
	printk("T18xx PLL2 (Audio) clk =%dMhz\n",CLK_MHZ(audio_clk));
	printk("T18xx run @%d MHz Sdram @%d MHz APH=%d MHz\n", CLK_MHZ(cpu_clk), CLK_MHZ(ahb_clk), CLK_MHZ(apb_clk));
#else

	u32 cpu_clk;
	u32 ratio=2;
	cpu_clk = 384*MHZ;

	ahb_clk = cpu_clk/ratio;
	apb_clk = ahb_clk/ratio;

	printk("T19xx fpga  run @%d MHz Sdram  run @%d MHz \n", CLK_MHZ(cpu_clk), CLK_MHZ(ahb_clk));


#endif


}

#endif


u32 t18xx_get_clks(clk_enum id)
{
	u32 div;
	switch (id){
		case SDMMC_CLK:
			div = 1;
			return apb_clk/div;
		case SPI_CLK:
			div = (SspCfgCount +1 )<<1;
			return apb_clk/div;
		case IIS_CLK:
			return apb_clk;
		case TIMER0_CLK:
			return apb_clk;
		default:
			printk("unknown id=%d clock, pls check!!!\n", (id));
			break;
		}

	return -1;

}
EXPORT_SYMBOL(t18xx_get_clks);


