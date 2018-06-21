/*
 * TI DaVinci clock definitions
 *
 * Copyright (C) 2006-2007 Texas Instruments.
 * Copyright (C) 2008-2009 Deep Root Systems, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_CLOCK_H
#define __ARCH_ARM_CLOCK_H

#include <linux/list.h>
#include <asm/clkdev.h>

#define DAVINCI_PLL1_BASE 0x01c40800
#define DAVINCI_PLL2_BASE 0x01c40c00
#define MAX_PLL 2

/* PLL/Reset register offsets */
#define PLLCTL          0x100
#define PLLCTL_PLLEN    BIT(0)
#define PLLCTL_CLKMODE  BIT(8)

#define PLLM		0x110
#define PLLM_PLLM_MASK  0xff

#define PREDIV          0x114
#define PLLDIV1         0x118
#define PLLDIV2         0x11c
#define PLLDIV3         0x120
#define POSTDIV         0x128
#define BPDIV           0x12c
#define PLLCMD		0x138
#define PLLSTAT		0x13c
#define PLLALNCTL	0x140
#define PLLDCHANGE	0x144
#define PLLCKEN		0x148
#define PLLCKSTAT	0x14c
#define PLLSYSTAT	0x150
#define PLLDIV4         0x160
#define PLLDIV5         0x164
#define PLLDIV6         0x168
#define PLLDIV7         0x16c
#define PLLDIV8         0x170
#define PLLDIV9         0x174
#define PLLDIV_EN       BIT(15)
#define PLLDIV_RATIO_MASK 0x1f

#define CLK_CFG                 BIT(1)


struct pll_data {
	u32 phys_off;
	void __iomem *base;
	u32 num;
	u32 input_rate;
};
#define PLL_HAS_PREDIV          0x01
#define PLL_HAS_POSTDIV         0x02

struct clk {
	struct list_head	node;
	struct module		*owner;
	const char		*name;
	unsigned long		rate;
	u8			usecount;
	u8			flags;
	u8 			rsv0;
	u8 			rsv1;
	u32			ctlbit;
	u32 			div;
	struct clk              *parent;
	struct pll_data         *pll_data;
	u32                     div_reg;
	void (*enable)(struct clk *, int enable);
};


struct t18xx_clk {
	struct clk_lookup lk;
};

#define CLK(dev, con, ck) 		\
	{				\
		.lk = {			\
			.dev_id = dev,	\
			.con_id = con,	\
			.clk = ck,	\
		},			\
	}


extern struct t18xx_clk t18xx_clks[];

int t18xx_clk_init(struct t18xx_clk *clocks);

extern int clk_register(struct clk *clk);
extern void clk_unregister(struct clk *clk);


#endif

