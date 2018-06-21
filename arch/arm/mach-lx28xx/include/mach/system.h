/*
 * linux/include/asm-arm/arch-arkdmp/system.h
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <asm/io.h>
#include <mach/hardware.h>

/*regs offset*/

#define WDTCR			 0x0
#define WDTPSR		 0x4
#define WDTLDR		 0x8
#define WDTVLR		 0xc
#define WDTISR     0x10
#define WDTRVR     0x14
#define WDTTMR     0x80
#define WDTTCR      0x84


static inline void arch_reset(char mode, const char *cmd)
{
	void * wdt_base=WDT_BASE;
	
	__raw_writel(0, wdt_base + WDTLDR);
	__raw_writel(WDTCR_INTEN |WDTCR_RSTEN |WDTCR_WDTEN, wdt_base + WDTCR);

}

static inline void arch_idle(void)
{
	cpu_do_idle() ;
}

#endif
