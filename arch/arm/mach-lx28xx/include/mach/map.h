#ifndef __ASM_ARCH_LX28XX_MAP_H
#define __ASM_ARCH_LX28XX_MAP_H




#define LX28XX_ADDR_BASE	(0xF4000000)

#ifndef __ASSEMBLY__
#define LX28XX_ADDR(x)	((void __iomem __force *)LX28XX_ADDR_BASE + (x))
#else
#define LX28XX_ADDR(x)	(LX28XX_ADDR_BASE + (x))
#endif



/* interrupt controller is the first thing we put in, to make
 * the assembly code for the irq detection easier
 */



#define LX28_PA_APB_SYS	   (0x60000000)
#define LX28XX_SZ_APB_SYS	   SZ_1M

/* memory controller registers 
#define LX28_PA_AHB_MEM 	(0xf0050000)
#define LX28XX_SZ_AHB_MEM  SZ_1M
*/
#define LX28_PA_AHB_SYS	   (0x70000000)
#define LX28XX_SZ_AHB_SYS	   SZ_2M


#define LX28_PA_AHB_JPEG	(0xf0400000)
#define LX28XX_SZ_AHB_JPEG  SZ_1M

//#define LX28_PA_NAND	(0x50000000)
//#define LX28XX_SZ_NAND  SZ_1M

#define LX28_PA_SRAM 		(0x400000)
#define LX28XX_SZ_SRAM 		SZ_1M

//#define LX28_PA_PERIPH 		(0x71210000)
//#define LX28XX_SZ_PERIPH 	SZ_32K
#define LX28_PA_PERIPH 		(0x71200000)
#define LX28XX_SZ_PERIPH 	SZ_1M

/* UARTs */
#define LX28_PA_UART	   (0x60070000)
#define LX28XX_SZ_UART	   SZ_1M
#define LX28XX_UART_OFFSET	   (0x1000)

#define LX28XX_VA_UARTx(uart) (LX28XX_VA_UART + ((uart * LX28XX_UART_OFFSET)))


/* Standard size definitions for peripheral blocks. */

#define LX28XX_SZ_IIS		SZ_1M
#define LX28XX_SZ_ADC		SZ_1M
#define LX28XX_SZ_SPI		SZ_1M
#define LX28XX_SZ_SDI		SZ_1M
#define LX28XX_SZ_NAND		SZ_1M



#ifdef CONFIG_MMU
/*APB0*/
#define LX28XX_VA_APB_SYS	LX28XX_ADDR(0x00000000)	/* system control */
/*AHBs*/
#define LX28XX_VA_AHB_SYS		LX28XX_ADDR(0x00100000)	/* memory control */
#define LX28XX_VA_AHB_JPEG	LX28XX_ADDR(0x00200000)	/* watchdog */
//#define LX28XX_VA_NAND		LX28XX_ADDR(0x00300000)	/* system control */
#define LX28XX_VA_MAE0 		(LX28XX_VA_AHB_SYS + 0x50000) /* MAE0 */
#define LX28XX_VA_NAND 		(LX28XX_VA_AHB_SYS + 0x90000) /* NFC */
//#define LX28XX_VA_UART_IO		LX28XX_ADDR(0x00400000)/* UART0 */

#define LX28XX_VA_UART		(LX28XX_VA_APB_SYS + 0x70000)/* UART0 */

#define LX28XX_VA_SRAM 		LX28XX_ADDR(0x00400000) /* SRAM */

#define LX28XX_VA_PERIPH 	LX28XX_ADDR(0x00500000) /* Core Peripherial*/
#else
#define LX28XX_VA_APB_SYS	LX28_PA_APB_SYS	/* system control */
/*AHBs*/
#define LX28XX_VA_AHB_SYS		LX28_PA_AHB_SYS	/* memory control */
#define LX28XX_VA_AHB_JPEG	LX28_PA_AHB_JPEG	/* watchdog */
#define LX28XX_VA_NAND		LX28_PA_NAND	/* system control */

#define LX28XX_VA_UART		LX28_PA_UART	/* UART0 */

#define LX28XX_VA_SRAM  	LX28_PA_SRAM    /* SRAM */

#define LX28XX_VA_PERIPH 	LX28_PA_PERIPH
#endif



/*
 * I/O mapping
 */
	



#define IO_ADDRESS(x)	

#define IODESC_ENTRY(x) { (unsigned long)LX28XX_VA_##x, __phys_to_pfn(LX28_PA_##x), LX28XX_SZ_##x, MT_DEVICE }



#endif /* __ASM_PLAT_LX28XX_MAP_H */


