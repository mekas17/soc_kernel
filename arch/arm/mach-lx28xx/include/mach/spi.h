/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ARCH_ARM_FREECHIP_SPI_H
#define __ARCH_ARM_FREECHIP_SPI_H

#define SPI_INTERN_CS	0xFF

/* resource flags for IORESOURCE_DMA resources */
#define IORESOURCE_DMA_RX_CHAN		0x01
#define IORESOURCE_DMA_TX_CHAN		0x02

enum {
	SPI_VERSION_1, /* For DM355/DM365/DM6467*/
	SPI_VERSION_2, /* For DA8xx */
};

struct lx28xx_spi_platform_data {
	u8	version;
	u16	num_chipselect;
	u32	wdelay;
	u32	odd_parity;
	u32	parity_enable;
	u32	wait_enable;
	u32	timer_disable;
	u32	clk_internal;
	u32	cs_hold;
	u32	intr_level;
	u32	poll_mode;
	u32	use_dma;
	u8	c2tdelay;
	u8	t2cdelay;
};

#endif	/* __ARCH_ARM_FREECHIP_SPI_H */

