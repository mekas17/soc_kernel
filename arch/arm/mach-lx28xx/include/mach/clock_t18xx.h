/*
 * T18xx clock definitions
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_CLOCK_T19XX_H
#define __ARCH_ARM_CLOCK_T19XX_H

#include <linux/types.h>

	
#if 0
//#define UART_CLK	(apb_clk)
#define UART_CLK	18000000 //FIXME, (uart_clk= CPU_CLK/2)
#define WDOG_CLK	(apb_clk)
#define TIME_CLK	(apb_clk)
#define IIS_CLK	(apb_clk)
#define SDRAM_CLK	(ahb_clk)
#define CALC_PCLKS(in, out)	(in/(out<<1) -1)

#endif
typedef enum {
	SDMMC_CLK=0,
	SPI_CLK,
	IIS_CLK,
	TIMER0_CLK,
	TIMER1_CLK,
	TIMER2_CLK,
	

}clk_enum;

void t18xx_clock_init(void);
u32 t18xx_get_clks(clk_enum id);


#endif


