/*
 * Debugging macro for lx28xx Soc
 *
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/* Modifications
 * Jan 2009	Chaithrika U S	Added senduart, busyuart, waituart
 *				macros, based on debug-8250.S file
 *				but using 32-bit accesses required for
 *                              some davinci devices.
 */

#include<mach/map.h>

       .macro	addruart, rp, rv, tmp
		mov	\rv, #LX28XX_VA_APB_SYS	@ virtual base
	    orr \rv, \rv, #0x00070000
	// physical addr
		mov	\rp, #0x60000000	@ physical base address
	    orr \rp, \rp, #0x00070000
        .endm
        

        .macro  senduart,rd,rx
        strb    \rd, [\rx]
        .endm

        .macro  busyuart,rd,rx

        .endm

        .macro  waituart,rd,rx
1001:       ldr    \rd, [\rx, #0x18]
	and  \rd,\rd, #0x20
        teq \rd, #0x20
        beq 1001b
        .endm


