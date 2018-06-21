/*
 *  Board-specific MMC configuration
 */

#ifndef _FREECHIP_MMC_H
#define _FREECHIP_MMC_H

#include <linux/types.h>
#include <linux/mmc/host.h>

struct lx28xx_mmc_config {
	/* get_cd()/get_wp() may sleep */
	int	(*get_cd)(int module);
	int	(*get_ro)(int module);
	/* wires == 0 is equivalent to wires == 4 (4-bit parallel) */
	u8	wires;

	u32     max_freq;

	/* any additional host capabilities: OR'd in to mmc->f_caps */
	u32     caps;

};



#endif

