/*
 *  linux/arch/arm/mach-lx28xx/arch.c
 *
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/memory.h>
	 
#include <linux/i2c.h>

#include <linux/etherdevice.h>
#include <linux/mtd/mtd.h>
//#include <linux/mtd/nand.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/io.h>
#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/videodev2.h>
	 
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>	 
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
//#include <asm/mach/flash.h>
#include <mach/clock.h>
#include <mach/map.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/nand.h>
#include <mach/spi.h>
#include <mach/mmc.h>
#include <linux/dma/dw.h>
//#include <mach/clock_lx28xx.h>
#include <mach/gmac_data.h>


#include <linux/device.h>
#include <linux/resource.h>
#include <linux/serial.h>
#include <linux/serial_lx28xx.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/mmc/mmc.h>

#define INT_OFFSET 		32


/********************************UART Controller*********************************/

static struct resource lx28xx_uart_resources[] = {
    [0] = {
        .start      = (u32)UART0_BASE,
        .end        = UART0_BASE + 0xfff,
        .flags      = IORESOURCE_MEM,
    },
    [1] = {
        .start      = (LX28XX_UART0_INT+INT_OFFSET),
        .end        = (LX28XX_UART0_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
    },
};

static u64 uart_dmamask = DMA_BIT_MASK(32);

struct lx28xx_port lx28xx_ports[] = {
     {
        .port   = {
            .type       = PORT_LX28XX,
            .iotype     = UPIO_MEM,
            .membase    = (void __iomem *)UART0_BASE,
            .mapbase    = UART0_BASE,
            .irq        = (LX28XX_UART0_INT+INT_OFFSET),
            .uartclk    = UART_CLK,
            .fifosize   = 16,
            .flags      = UPF_BOOT_AUTOCONF,
            .line       = 0,
        },
    },
};

static struct platform_device lx28xx_uart_device = {
    .name       = "lx28xx-uart",
    .id     = -1,
    .dev = {
        .dma_mask       = &uart_dmamask,
        .coherent_dma_mask  = DMA_BIT_MASK(32),
        .platform_data = lx28xx_ports,
    },
    .num_resources  = ARRAY_SIZE(lx28xx_uart_resources),
    .resource   = lx28xx_uart_resources,
};

static struct resource lx28xx_suart_resources[] = {
    [0] = {
        .start      = (u32)SUART1_BASE,
        .end        = SUART1_BASE + SMAP_SIZE,
        .flags      = IORESOURCE_MEM,
    },
    [1] = {
        .start      = (LX28XX_SUART1_INT+INT_OFFSET),
        .end        = (LX28XX_SUART1_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
    },
    [2] = {
        .start      = (u32)SUART2_BASE,
        .end        = SUART2_BASE + SMAP_SIZE,
        .flags      = IORESOURCE_MEM,
    },
    [3] = {
        .start      = (LX28XX_SUART2_INT+INT_OFFSET),
        .end        = (LX28XX_SUART2_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
    },
     [4] = {
        .start      = (u32)SUART3_BASE,
        .end        = SUART3_BASE + SMAP_SIZE,
        .flags      = IORESOURCE_MEM,
    },
    [5] = {
        .start      = (LX28XX_SUART3_INT+INT_OFFSET),
        .end        = (LX28XX_SUART3_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
    },
    [6] = {
        .start      = (u32)SUART4_BASE,
        .end        = SUART4_BASE + SMAP_SIZE,
        .flags      = IORESOURCE_MEM,
    },
    [7] = {
        .start      = (LX28XX_SUART4_INT+INT_OFFSET),
        .end        = (LX28XX_SUART4_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
    },
     [8] = {
        .start      = (u32)SUART5_BASE,
        .end        = SUART5_BASE + SMAP_SIZE,
        .flags      = IORESOURCE_MEM,
    },
    [9] = {
        .start      = (LX28XX_SUART5_INT+INT_OFFSET),
        .end        = (LX28XX_SUART5_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
    },
    [10] = {
        .start      = (u32)SUART6_BASE,
        .end        = SUART6_BASE + SMAP_SIZE,
        .flags      = IORESOURCE_MEM,
    },
    [11] = {
        .start      = (LX28XX_SUART6_INT+INT_OFFSET),
        .end        = (LX28XX_SUART6_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
    },
     [12] = {
        .start      = (u32)SUART7_BASE,
        .end        = SUART7_BASE + SMAP_SIZE,
        .flags      = IORESOURCE_MEM,
    },
    [13] = {
        .start      = (LX28XX_SUART7_INT+INT_OFFSET),
        .end        = (LX28XX_SUART7_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
    },
    [14] = {
        .start      = (u32)SUART8_BASE,
        .end        = SUART8_BASE + SMAP_SIZE,
        .flags      = IORESOURCE_MEM,
    },
    [15] = {
        .start      = (LX28XX_SUART8_INT+INT_OFFSET),
        .end        = (LX28XX_SUART8_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
    },
     [16] = {
        .start      = (u32)SUART9_BASE,
        .end        = SUART9_BASE + SMAP_SIZE,
        .flags      = IORESOURCE_MEM,
    },
    [17] = {
        .start      = (LX28XX_SUART9_INT+INT_OFFSET),
        .end        = (LX28XX_SUART9_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
    },
      
};

struct inno_uart_port lx28xx_s_ports[] = {
     {
        .port   = {
            .type       = PORT_INNO,
            .iotype     = UPIO_MEM,
            .membase    =  (unsigned char __iomem*)SUART1_BASE,
            .mapbase    = SUART1_BASE,
            .mapsize    = SMAP_SIZE,
            .irq        = (LX28XX_SUART1_INT+INT_OFFSET),
            .uartclk    = SUART_CLK,
            .fifosize   = 64,
            .flags      = UPF_BOOT_AUTOCONF,
            .line       = 0,
        },
        .baud  = 9600,
    },
    {
        .port   = {
            .type       = PORT_INNO,
            .iotype     = UPIO_MEM,
            .membase    =  (unsigned char __iomem*)SUART2_BASE,
            .mapbase    = SUART2_BASE,
            .mapsize    = SMAP_SIZE,
            .irq        = (LX28XX_SUART2_INT+INT_OFFSET),
            .uartclk    = SUART_CLK,
            .fifosize   = 64,
            .flags      = UPF_BOOT_AUTOCONF,
            .line       = 1,
        },
         .baud  = 9600,
    },
    {
        .port   = {
            .type       = PORT_INNO,
            .iotype     = UPIO_MEM,
            .membase    =  (unsigned char __iomem*)SUART3_BASE,
            .mapbase    = SUART3_BASE,
            .mapsize    = SMAP_SIZE,
            .irq        = (LX28XX_SUART3_INT+INT_OFFSET),
            .uartclk    = SUART_CLK,
            .fifosize   = 64,
            .flags      = UPF_BOOT_AUTOCONF,
            .line       = 2,
        },
         .baud  = 9600,
    },
    {
        .port   = {
            .type       = PORT_INNO,
            .iotype     = UPIO_MEM,
            .membase    =  (unsigned char __iomem*)SUART4_BASE,
            .mapbase    = SUART4_BASE,
            .mapsize    = SMAP_SIZE,
            .irq        = (LX28XX_SUART4_INT+INT_OFFSET),
            .uartclk    = SUART_CLK,
            .fifosize   = 64,
            .flags      = UPF_BOOT_AUTOCONF,
            .line       = 3,
        },
         .baud  = 9600,
    },
    {
        .port   = {
            .type       = PORT_INNO,
            .iotype     = UPIO_MEM,
            .membase    =  (unsigned char __iomem*)SUART5_BASE,
            .mapbase    = SUART5_BASE,
            .mapsize    = SMAP_SIZE,
            .irq        = (LX28XX_SUART5_INT+INT_OFFSET),
            .uartclk    = SUART_CLK,
            .fifosize   = 64,
            .flags      = UPF_BOOT_AUTOCONF,
            .line       = 4,
        },
         .baud  = 9600,
    },
    {
        .port   = {
            .type       = PORT_INNO,
            .iotype     = UPIO_MEM,
            .membase    =  (unsigned char __iomem*)SUART6_BASE,
            .mapbase    = SUART6_BASE,
            .mapsize    = SMAP_SIZE,
            .irq        = (LX28XX_SUART6_INT+INT_OFFSET),
            .uartclk    = SUART_CLK,
            .fifosize   = 64,
            .flags      = UPF_BOOT_AUTOCONF,
            .line       = 5,
        },
         .baud  = 9600,
    },
    {
        .port   = {
            .type       = PORT_INNO,
            .iotype     = UPIO_MEM,
            .membase    =  (unsigned char __iomem*)SUART7_BASE,
            .mapbase    = SUART7_BASE,
            .mapsize    = SMAP_SIZE,
            .irq        = (LX28XX_SUART7_INT+INT_OFFSET),
            .uartclk    = SUART_CLK,
            .fifosize   = 64,
            .flags      = UPF_BOOT_AUTOCONF,
            .line       = 6,
        },
         .baud  = 9600,
    },
    {
        .port   = {
            .type       = PORT_INNO,
            .iotype     = UPIO_MEM,
            .membase    =  (unsigned char __iomem*)SUART8_BASE,
            .mapbase    = SUART8_BASE,
            .mapsize    = SMAP_SIZE,
            .irq        = (LX28XX_SUART8_INT+INT_OFFSET),
            .uartclk    = SUART_CLK,
            .fifosize   = 64,
            .flags      = UPF_BOOT_AUTOCONF,
            .line       = 7,
        },
         .baud  = 9600,
    },
    {
        .port   = {
            .type       = PORT_INNO,
            .iotype     = UPIO_MEM,
            .membase    =  (unsigned char __iomem*)SUART9_BASE,
            .mapbase    = SUART9_BASE,
            .mapsize    = SMAP_SIZE,
            .irq        = (LX28XX_SUART9_INT+INT_OFFSET),
            .uartclk    = SUART_CLK,
            .fifosize   = 64,
            .flags      = UPF_BOOT_AUTOCONF,
            .line       = 8,
        },
         .baud  = 115200,
    }, 
};
EXPORT_SYMBOL(lx28xx_s_ports);

static struct platform_device lx28xx_suart_device = {
    .name       = "inno-uart",
    .id     = -1,
    .dev = {
        //.dma_mask       = &uart_dmamask,
        //.coherent_dma_mask  = DMA_BIT_MASK(32),
        .platform_data = lx28xx_s_ports,
    },
    .num_resources  = ARRAY_SIZE(lx28xx_suart_resources),
    .resource   = lx28xx_suart_resources,
};


static struct platform_device lx28xx_watchdog_device = {
    .name       = "watchdog",
    .id     = -1,
};
#define PWM_FAN_BASE    0x602f0000
static struct resource lx28xx_pwm_resource[] = {
        {
            .start = (u32)PWM_FAN_BASE,
            .end   = PWM_FAN_BASE + SZ_64K -1,
            .flags  = IORESOURCE_MEM,
        },
        {
            .start = LX28XX_PWM_INT + INT_OFFSET,
            .end   = LX28XX_PWM_INT + INT_OFFSET,
            .flags  = IORESOURCE_IRQ,
        },

};

static struct platform_device lx28xx_pwm_device = {
        .name = "pwmdrv_inno",
        .id   = 0,
        .num_resources = ARRAY_SIZE(lx28xx_pwm_resource),
        .resource = lx28xx_pwm_resource,
        
};




/********************************AHB DMA Controller*********************************/

struct dw_dma_platform_data lx28xx_dma_info={
	.nr_channels =8,

};


static struct resource dma_resources[] = { 
	{
        .start      = LX28XX_DMAC_BASE,
        .end        = LX28XX_DMAC_BASE + 0xffff,
        .flags      = IORESOURCE_MEM,
	},
	{
        .start      = (LX28XX_DMA_INT+INT_OFFSET),
        .end        = (LX28XX_DMA_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
	},

};



static struct platform_device lx28xx_dma_device = {
	.name			= "dmac",
	.id			= 0,
	.dev.platform_data	= &lx28xx_dma_info,
	.num_resources		= ARRAY_SIZE(dma_resources),
	.resource		= dma_resources,
};


/********************************Mac Controller*********************************/

static u64 mac_dmamask = DMA_BIT_MASK(32);

//#ifdef CONFIG_DWMAC_LX28XX
#if 1
#if 1
static struct resource gmac_resources[] = { 
	{
        .start      = MAC_BASE,
        .end        = MAC_BASE + 0xffff,
        .flags      = IORESOURCE_MEM,
	},
	{
        .start      = (LX28XX_MAC_INT+INT_OFFSET),
        .end        = (LX28XX_MAC_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
        .name 		= "macirq",
	},

};
#endif
#if 0
static struct resource gmac_resources[] = { 
	{
        .start      = MAC2_BASE,
        .end        = MAC2_BASE + 0xffff,
        .flags      = IORESOURCE_MEM,
	},
	{
        .start      = (LX28XX_MAC2_INT+INT_OFFSET),
        .end        = (LX28XX_MAC2_INT+INT_OFFSET),
        .flags      = IORESOURCE_IRQ,
        .name 		= "macirq",
	},

};
#endif
static struct plat_mdio_bus_data dmp_mdio_platform_bus_data = {
	.phy_mask = 0,
};

static struct clk dmp_gmac_clock = {
	.rate = 240000000, 
};

static struct stmmac_dma_cfg dmp_dma_cfg = {
	.pbl = 8,
};

static struct plat_gmac_data dmp_gmac_platform_data=
{
	//.phy_bus_name = "lx28xxmac",
	.interface = PHY_INTERFACE_MODE_RGMII,
	.bus_id= 0,
	.phy_addr = 0,
	//.pbl = 16,
	.has_gmac = 1,
	.maxmtu = 1500, 
	.clk_csr = 0, 
	.rx_queues_to_use = 1, 
	.tx_queues_to_use = 1, 
	.stmmac_clk = &dmp_gmac_clock, 
	.dma_cfg = &dmp_dma_cfg, 
	.mdio_bus_data = (struct stmmac_mdio_bus_data *)&dmp_mdio_platform_bus_data,
	//.mac_port_sel_speed = 100,
};

static struct platform_device dmp_gmac_device = {
	.name			= "lx28xxmaceth",
	.id			= 0,
    .dev = {
        .dma_mask       = &mac_dmamask,
        .coherent_dma_mask  = DMA_BIT_MASK(32),
        .platform_data = &dmp_gmac_platform_data,
    },
	.num_resources		= ARRAY_SIZE(gmac_resources),
	.resource		= gmac_resources,
	
    
};



#else


static struct platform_device lx28xx_mac_device = {
	.name			= "lx28xx_mac",
	.id			= 0,
    .dev = {
        .dma_mask       = &mac_dmamask,
        .coherent_dma_mask  = DMA_BIT_MASK(32),
    },
};
#endif




/********************************Nand Controller*********************************/
#if 1
struct mtd_partition lx28xx_evm_nandflash_partition[] = {
	{
		.name		= "spl",
		.offset		= 0,
		.size		= SZ_128K, //SZ_256K + SZ_128K,
		.mask_flags	= 0,
	},
	{
		.name		= "bootA",
		.offset		= MTDPART_OFS_APPEND,
		.size 		= SZ_512K,
		.mask_flags	= 0,
	},
	{
		.name		= "bootB",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_512K,
		.mask_flags	= 0,
	},
	{
		.name		= "emv",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_1M - SZ_128K,
		.mask_flags	= 0,
	},
	{
		.name		= "kernelA",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M + SZ_2M + SZ_1M,
		.mask_flags	= 0,
	},
	{
		.name		= "kernelB",
		.offset		= MTDPART_OFS_APPEND,
		.size 		= SZ_4M + SZ_2M + SZ_1M,
		.mask_flags	= 0,
	},
	{
		.name		= "rootfsA",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_64M + SZ_32M,
		.mask_flags	= 0,
	},
	{
		.name		= "rootfsB",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_64M + SZ_32M,
		.mask_flags	= 0,
	},
	{
		.name		= "config",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_8M,
		.mask_flags	= 0,
	},
	{
		.name		= "log",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_8M + SZ_2M,
		.mask_flags	= 0,
	},
	{
		.name		= "bbt",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_8M - SZ_2M,
		.mask_flags	= 0,
	},
	{
		.name		= "reserevd",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
 
};
#else
struct mtd_partition lx28xx_evm_nandflash_partition[] = {
	{
		.name		= "boot",
		.offset		= 0,
		.size		= SZ_2M, //SZ_256K + SZ_128K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* Kernel */
	{
		.name		= "core",
		.offset		= MTDPART_OFS_APPEND,
		//.size		= SZ_4M,
		.size 		= SZ_8M,
		.mask_flags	= 0,
	},
	/* File system  */
	{
		.name		= "base",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_64M,
		.mask_flags	= 0,
	},
	{
		.name		= "guia",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};
#endif
static struct freechip_nand_pdata lx28xx_evm_nandflash_data = {
	.parts		= lx28xx_evm_nandflash_partition,
	.nr_parts	= ARRAY_SIZE(lx28xx_evm_nandflash_partition),
	.ecc_mode	= NAND_ECC_NONE,
	.options	= NAND_SKIP_BBTSCAN, //NAND_USE_FLASH_BBT,
};

static struct resource lx28xx_evm_nandflash_resource[] = {
	{
		.start		= LX28XX_VA_NAND,
		.end		= LX28XX_VA_NAND + SZ_1M - 1,
		.flags		= IORESOURCE_MEM,
	}
};

static struct platform_device lx28xx_evm_nandflash_device = {
	.name		= "lx28xx_nand",
	.id		= 0,
	.dev		= {
		.platform_data	= &lx28xx_evm_nandflash_data,
	},
	.num_resources	= ARRAY_SIZE(lx28xx_evm_nandflash_resource),
	.resource	= lx28xx_evm_nandflash_resource,
};

/********************************SPI device specific data******************************/


struct mtd_partition lx28xx_evm_dataflash_partition[] = {
	{
		.name		= "boot",
		.offset		= 0,
		.size		= SZ_256K, //SZ_128K, //SZ_256K + SZ_128K,
		.mask_flags	= 0,	/* force read-only */
	},
	/* Kernel */
	{
		.name		= "core",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_1M, //SZ_2M,
		.mask_flags	= 0,
	},
	/* File system  */
	{
		.name		= "base",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	},
	#if 0
	{
		.name		= "guia",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
	#endif
};



static struct flash_platform_data lx28xx_dataflash = {
	
	.parts	= lx28xx_evm_dataflash_partition,
	.nr_parts	= ARRAY_SIZE(lx28xx_evm_dataflash_partition),
	.type	="mx25l3205d", // "mx25l1605d",

};

static struct spi_board_info lx28xx_evm_spi_info[] __initconst = {
	{
		.modalias	= "spi_dataflash",
		.platform_data	= &lx28xx_dataflash,
		.max_speed_hz	= 2 * 1000 * 1000,	/* at 3v3 */
		.bus_num	= 0,
		.chip_select	= 0,
		.mode		= SPI_MODE_0,
	},
};

#if 1
/********************************SSP controller*******************************************/

static u64 lx28xx_evm_dma_mask = DMA_BIT_MASK(32);

static struct lx28xx_spi_platform_data lx28xx_evm_spi_pdata = {
	.version 	= SPI_VERSION_1,
	.num_chipselect = 2,
	.clk_internal	= 1,
	.intr_level	= 0,
	.poll_mode	= 1,	/* 0 -> interrupt mode 1-> polling mode */
	.use_dma	= 0,	/* when 1, value in poll_mode is ignored */
	.c2tdelay	= 8,
	.t2cdelay	= 8,
};

static struct resource lx28xx_evm_spi_resources[] = {
	{
		.start		= SPI_BASE,
		.end		= SPI_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start = (LX28XX_SPI_INT + INT_OFFSET),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMA_RQ_SPIRX,
		.flags = IORESOURCE_DMA | IORESOURCE_DMA_RX_CHAN,
	},
	{
		.start = DMA_RQ_SPITX,
		.flags = IORESOURCE_DMA | IORESOURCE_DMA_TX_CHAN,
	},
};

static struct platform_device lx28xx_evm_spi_device = {
	.name = "lx28xx-spi",
	.id = 0,
	.dev = {
		.dma_mask = &lx28xx_evm_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &lx28xx_evm_spi_pdata,
	},
	.num_resources = ARRAY_SIZE(lx28xx_evm_spi_resources),
	.resource = lx28xx_evm_spi_resources,
};

static inline void ssp_platform_init(void)
{

}


void __init lx28xx_init_spi(struct spi_board_info *info, unsigned len)
{

	ssp_platform_init();
	spi_register_board_info(info, len);

	platform_device_register(&lx28xx_evm_spi_device);
}

 #endif
 /********************************SD/MMC controller*******************************************/


static struct lx28xx_mmc_config lx28xx_mmc_config = {
	.caps		= MMC_CAP_4_BIT_DATA,
	.max_freq	= 50000000,
	.wires		= 4,
};

static u64 mmcsd0_dma_mask = DMA_BIT_MASK(32);

static struct resource mmcsd0_resources[] = {
	{
		.start = LX28XX_SDMMC0_BASE,
		.end   = LX28XX_SDMMC0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	/* IRQs:  MMC/SD, then SDIO */
	{
		//.start = LX28XX_UART6_SDMMC_INT,
		.start = (LX28XX_SDMMC_INT+INT_OFFSET),
		.flags = IORESOURCE_IRQ,
	},
	/* DMA channels: RX/TX*/
	{
		.start = DMA_RQ_SDMMCRX,
		.flags = IORESOURCE_DMA,
	},
};

static struct platform_device lx28xx_mmcsd0_device = {
	.name = "mmc",
	.id = 0,
	.dev = {
		.dma_mask = &mmcsd0_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		//.platform_data = &lx28xx_mmcsd0_pdata,
	},
	.num_resources = ARRAY_SIZE(mmcsd0_resources),
	.resource = mmcsd0_resources,
};

static u64 mmcsd1_dma_mask = DMA_BIT_MASK(32);

static struct resource mmcsd1_resources[] = {
	{
		.start = LX28XX_SDMMC1_BASE,
		.end   = LX28XX_SDMMC1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	/* IRQs:  MMC/SD, then SDIO */
	{
		.start = (LX28XX_SDIO_INT+INT_OFFSET),
		.flags = IORESOURCE_IRQ,
	}, 
	/* DMA channels: RX/TX*/
	{
		.start = DMA_RQ_SDIORX,
		.flags = IORESOURCE_DMA,
	},
};

static struct platform_device lx28xx_mmcsd1_device = {
	.name = "mmc",
	.id = 1,
	.dev = {
		.dma_mask = &mmcsd1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources = ARRAY_SIZE(mmcsd1_resources),
	.resource = mmcsd1_resources,
};



void __init lx28xx_setup_mmc(int module, struct lx28xx_mmc_config *config)
{
	struct platform_device	*pdev = NULL;

	
	/* REVISIT: should this be in board-init code? */
	//board specific  code in here
	//SDMMCCFGCOUNT=0; //for mmc freq

	
	switch (module){
		case 0:
			pdev = &lx28xx_mmcsd0_device;
			break;
		case 1:
			pdev = &lx28xx_mmcsd1_device;
			break;

		}
			
	if (WARN_ON(!pdev))
		return;

	pdev->dev.platform_data = config;
	platform_device_register(pdev);
}


/********************************IIS controller*******************************************/
static u64  iis_dma_mask = DMA_BIT_MASK(32);

static struct resource iis_resources[] = {
	{
		.start = LX28XX_I2S_BASE,
		.end   = LX28XX_I2S_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
	},
	/* IRQs */
	{
		.start = (LX28XX_I2S_INT+INT_OFFSET),
		.flags = IORESOURCE_IRQ,
	},
	/* DMA handshaking num*/
	
};



static struct platform_device lx28xx_iis_device = {
	.name = "freechip_i2s",
	.id = 0,
	.dev = {
		.dma_mask = &iis_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources = ARRAY_SIZE(iis_resources),
	.resource = iis_resources,
};
#if 0
/********************************T18xx RTC******************************************/

static struct resource rtc_resources[] = {
	{
		.start = LX28XX_RTC_BASE,
		.end   = LX28XX_RTC_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
	},
	/* IRQs */
	{
		.start = LX28XX_RTC_INT,
		.flags = IORESOURCE_IRQ,
	},
	
};

static struct platform_device lx28xx_rtc_device = {
	.name = "freechip_rtc",
	.id = 0,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};


/********************************T18xx Irda*****************************************

static struct resource irda_resources[] = {
	{
		.start = LX28XX_IRDA_BASE,
		.end   = LX28XX_IRDA_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
	},
	/* IRQs */
	{
		.start = LX28XX_IRDA_INT,
		.flags = IORESOURCE_IRQ,
	},
	
};

static struct platform_device lx28xx_irda_device = {
	.name = "freechip_irda",
	.id = 0,
	.num_resources = ARRAY_SIZE(irda_resources),
	.resource = irda_resources,
};

*/

#endif

static struct platform_device *lx28xx_platform_devices[] __initdata = {
	//&lx28xx_dma_device,
//#ifdef CONFIG_DWMAC_LX28XX
#if 1
	&dmp_gmac_device,
#else
	&lx28xx_mac_device,
#endif
	&lx28xx_uart_device,
//	&lx28xx_watchdog_device,
	&lx28xx_evm_nandflash_device,
//	&lx28xx_iis_device,
//	&lx28xx_rtc_device,
//	&lx28xx_irda_device,
	&lx28xx_suart_device,

    &lx28xx_pwm_device,
};






extern void lx28xx_init_time(void);

extern void __init lx28xx_init_irq(void);

static struct map_desc lx28xx_io_desc[] __initdata = {
	//IODESC_ENTRY(IRQ),
	IODESC_ENTRY(APB_SYS),
	//IODESC_ENTRY(NAND),
	IODESC_ENTRY(AHB_SYS),
	IODESC_ENTRY(SRAM),
	IODESC_ENTRY(PERIPH),
    //IODESC_ENTRY(UART),
};



static __init void lx28xx_evm_init(void)
{
	//t18_mux_init();
	//lx28xx_clk_init(lx28xx_clks);

	//lx28xx_net_rst();

	platform_add_devices(lx28xx_platform_devices,ARRAY_SIZE(lx28xx_platform_devices));

	//lx28xx_init_spi(lx28xx_evm_spi_info, ARRAY_SIZE(lx28xx_evm_spi_info));

	//lx28xx_setup_mmc(0, &lx28xx_mmc_config); // sdmmc
	lx28xx_setup_mmc(1, &lx28xx_mmc_config); // sdio

}

static void lx28xx_watchdog_reset(void)
{
    *((volatile unsigned int *)LX28XX_WDT_BASE) = 0x3;
}

void lx28xx_restart(enum reboot_mode mode, const char *cmd)
{
	//lx28xx_watchdog_reset(); need to fix
	lx28xx_watchdog_reset();
}

void __init lx28xx_init_late(void)
{

}



static void __init lx28xx_map_io(void)
{
	iotable_init(lx28xx_io_desc, ARRAY_SIZE(lx28xx_io_desc));
}

extern struct smp_operations dmp_smp_ops;

#ifdef CONFIG_MMU
MACHINE_START(LX28XX_EVM, "Logmicro Soc")
	/*MAINTAINER("jjdeng <hndeng06@gmail.com>")*/
	.atag_offset	  = 0x100,
	.map_io 	  = lx28xx_map_io,
	.init_irq	  = lx28xx_init_irq,
	.init_time	= lx28xx_init_time,
	.init_machine = lx28xx_evm_init,
	.init_late	= lx28xx_init_late,
	.restart	= lx28xx_restart,
#ifdef CONFIG_SMP
	.smp 		= &dmp_smp_ops,
#endif
MACHINE_END
#else


MACHINE_START(LX28XX_EVM, "Xiangjing Soc")
	/*MAINTAINER("jjdeng <jjdeng@xiangjing.com>")*/
	.phys_io	  = T18_PA_UART,
	//.io_pg_offst  = (((u32)LX28XX_VA_UART) >> 18) & 0xfffc,//(u32)(LX28XX_VA_UART >> 18) & 0xfffc,
	.boot_params  = (LX28XX_DDR_BASE + 0x100),
	//.map_io 	  = lx28xx_map_io,
	.init_irq	  = ark_dmp_init_irq,
	.timer		  = &lx28xx_timer,
	.init_machine = lx28xx_evm_init,

MACHINE_END

#endif





