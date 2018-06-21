/*
 * linux/include/asm-arm/arch-armdmp/memory.h
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#include <asm/sizes.h>

/*
 * Physical DRAM offset.
 */
#define T19XX_DDR_BASE	0x80000000

#define PHYS_OFFSET T19XX_DDR_BASE

#ifdef CONFIG_MMU
#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt(x)	__phys_to_virt(x)
#else
#define __virt_to_bus(x)	(x)
#define __bus_to_virt(x)	(x)

#endif

#define __pfn_to_bus(x) __pfn_to_phys(x)
#define __bus_to_pfn(x)	__phys_to_pfn(x)


#define __uncache_addr(x)	(__virt_to_phys(x) + MEM_UNCACHE_BASE_ADDR - MEM_BASE_PHY_ADDR)
#endif
