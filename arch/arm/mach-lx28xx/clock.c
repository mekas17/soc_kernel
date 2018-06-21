/*
 * Clock and PLL control for T18xx devices
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <mach/hardware.h>

#include <mach/clock.h>

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clockfw_lock);



static void __clk_enable(struct clk *clk)
{
	if (clk->parent)
		__clk_enable(clk->parent);
	if ((clk->usecount++ == 0) && (clk->flags & CLK_CFG))
		(clk->enable)(clk, 1);
}

static void __clk_disable(struct clk *clk)
{
	if (WARN_ON(clk->usecount == 0))
		return;
	if ((--clk->usecount == 0) && (clk->flags & CLK_CFG ))
		(clk->enable)(clk, 0);
	if (clk->parent)
		__clk_disable(clk->parent);
}

int clk_enable(struct clk *clk)
{
	unsigned long flags;

	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	spin_lock_irqsave(&clockfw_lock, flags);
	__clk_enable(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);

	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	if (clk == NULL || IS_ERR(clk))
		return;

	spin_lock_irqsave(&clockfw_lock, flags);
	__clk_disable(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	return clk->rate;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	/* changing the clk rate is not supported */
	return -EINVAL;
}
EXPORT_SYMBOL(clk_set_rate);

int clk_register(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	if (WARN(clk->parent && !clk->parent->rate,
			"CLK: %s parent %s has no rate!\n",
			clk->name, clk->parent->name))
		return -EINVAL;

	mutex_lock(&clocks_mutex);
	list_add_tail(&clk->node, &clocks);
	mutex_unlock(&clocks_mutex);

	/* If rate is already set, use it */
	if (clk->rate)
		return 0;

	/* Otherwise, default to parent rate */
	if (clk->parent){
		clk->rate = clk->div?(clk->parent->rate/clk->div):clk->parent->rate;
		}

	return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	mutex_lock(&clocks_mutex);
	list_del(&clk->node);
	mutex_unlock(&clocks_mutex);
}

#if 0
static void clk_sysclk_recalc(struct clk *clk)
{
	u32 v, plldiv;
	struct pll_data *pll;

	/* If this is the PLL base clock, no more calculations needed */
	if (clk->pll_data)
		return;

	if (WARN_ON(!clk->parent))
		return;

	clk->rate = clk->parent->rate;

	/* Otherwise, the parent must be a PLL */
	if (WARN_ON(!clk->parent->pll_data))
		return;

	pll = clk->parent->pll_data;

	/* If pre-PLL, source clock is before the multiplier and divider(s) */
	if (clk->flags & PRE_PLL)
		clk->rate = pll->input_rate;

	if (!clk->div_reg)
		return;

	v = __raw_readl(pll->base + clk->div_reg);
	if (v & PLLDIV_EN) {
		plldiv = (v & PLLDIV_RATIO_MASK) + 1;
		if (plldiv)
			clk->rate /= plldiv;
	}
}
#endif

static void __init clk_pll_init(struct clk *clk)
{



}

int __init t18xx_clk_init(struct t18xx_clk *clocks)
  {
	struct t18xx_clk *c;
	struct clk *clk;

	for (c = clocks; c->lk.clk; c++) {
		clk = c->lk.clk;

		if (clk->pll_data)
			clk_pll_init(clk);

		if (clk->ctlbit)
			clk->flags |= CLK_CFG;

		clkdev_add(&c->lk);
		clk_register(clk);

		/* Turn on clocks that Linux doesn't otherwise manage 
		if (clk->flags & ALWAYS_ENABLED)
			clk_enable(clk);
			*/
	}

	return 0;
}

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static void *t18xx_ck_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *t18xx_ck_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void t18xx_ck_stop(struct seq_file *m, void *v)
{
}

#define CLKNAME_MAX	10		/* longest clock name */
#define NEST_DELTA	2
#define NEST_MAX	4

static void
dump_clock(struct seq_file *s, unsigned nest, struct clk *parent)
{
	char		*state;
	char		buf[CLKNAME_MAX + NEST_DELTA * NEST_MAX];
	struct clk	*clk;
	unsigned	i;

	state = "";

	/* <nest spaces> name <pad to end> */
	memset(buf, ' ', sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = 0;
	i = strlen(parent->name);
	memcpy(buf + nest, parent->name,
			min(i, (unsigned)(sizeof(buf) - 1 - nest)));

	seq_printf(s, "%s users=%2d %-3s %9ld Hz\n",
		   buf, parent->usecount, state, clk_get_rate(parent));
	/* REVISIT show device associations too */

	/* cost is now small, but not linear... */
	list_for_each_entry(clk, &clocks, node) {
		if (clk->parent == parent)
			dump_clock(s, nest + NEST_DELTA, clk);
	}
}

static int t18xx_ck_show(struct seq_file *m, void *v)
{
	/* Show clock tree; we know the main oscillator is first.
	 * We trust nonzero usecounts equate to PSC enables...
	 */
	mutex_lock(&clocks_mutex);
	if (!list_empty(&clocks))
		dump_clock(m, 0, list_first_entry(&clocks, struct clk, node));
	mutex_unlock(&clocks_mutex);

	return 0;
}

static const struct seq_operations t18xx_ck_op = {
	.start	= t18xx_ck_start,
	.next	= t18xx_ck_next,
	.stop	= t18xx_ck_stop,
	.show	= t18xx_ck_show
};

static int t18xx_ck_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &t18xx_ck_op);
}

static const struct file_operations proc_t18xx_ck_operations = {
	.open		= t18xx_ck_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init t18xx_ck_proc_init(void)
{
	proc_create("t18xx_clocks", 0, NULL, &proc_t18xx_ck_operations);
	return 0;

}
__initcall(t18xx_ck_proc_init);
#endif /* CONFIG_DEBUG_PROC_FS */

