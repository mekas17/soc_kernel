/*
 * arch/arm/mach-t18xx/include/mach/entry-macro.S
 *
 * Low-level IRQ helper macros for T19XX-based platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
*/

/* We have a problem that the INTOFFSET register does not always
 * show one interrupt. Occasionally we get two interrupts through
 * the prioritiser, and this causes the INTOFFSET register to show
 * what looks like the logical-or of the two interrupt numbers.
 *
 * Thanks to Klaus, Shannon, et al for helping to debug this problem
*/

#define IRQPENDR		(0x4)
#define IRQISPR		(0x3c)
//#define IVEC_ADDR	(0x78)
#define IVEC_ADDR		(0x1b4) // new icu vector address

#include <mach/map.h>
#include <asm/irq.h>

	.macro  get_irqnr_preamble, base, tmp
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
#if 0 //FIXME, if dont' use the IVEC_ADDR register, IRQ 31 will occur, WHY? 
		mov	\base, #T19XX_VA_IRQ
		
		ldr \irqstat, [ \base, #IRQISPR ]
		mov \tmp, \irqstat


		@@ the number specified is not a valid irq, so try
		 @@ and work it out for ourselves
		
		 mov \irqnr, #0 	 @@ start here
		
		 @@ work out which irq (if any) we got
		
		 movs	 \tmp, \irqstat, lsl#16
		 addeq	 \irqnr, \irqnr, #16
		 moveq	 \irqstat, \irqstat, lsr#16
		 tst \irqstat, #0xff
		 addeq	 \irqnr, \irqnr, #8
		 moveq	 \irqstat, \irqstat, lsr#8
		 tst \irqstat, #0xf
		 addeq	 \irqnr, \irqnr, #4
		 moveq	 \irqstat, \irqstat, lsr#4
		 tst \irqstat, #0x3
		 addeq	 \irqnr, \irqnr, #2
		 moveq	 \irqstat, \irqstat, lsr#2
		 tst \irqstat, #0x1
		 addeq	 \irqnr, \irqnr, #1


		teq \tmp, #0
#elif 1
#ifdef CONFIG_MMU
	mov \base, #T19XX_VA_IRQ
#else
	ldr 	\base, =T19XX_VA_IRQ
#endif
  @ get vector for new icu
	ldr	\irqnr, [ \base, #IVEC_ADDR ]
	adr	r0, str_mark
	bl 	printascii
str_mark: .asciz "\ninterrupt occured.\n"
#if 0
	ldr	\irqnr, [ \base, #IVEC_ADDR ]
	mov	\irqnr, \irqnr, LSR #2
	teq  \irqnr, #0
	beq 1f
	ldr \irqstat, [ \base, #IRQISPR ]
	b 2f
1:
#if 1
	ldr r6, =0xd0409000
	ldr r7, [r6]
	and r7,r7, #(1<< 12)
	str r7, [r6 ]
	ldr r7, [r6, #0x4]
	and r7,r7, #(1<< 12)
	str r7, [r6, #0x4]
#endif	
	ldr \irqstat, [ \base, #IRQISPR ]
2:
	teq \irqstat, #(1 << 14)
#endif
	

#endif
	.endm

		/* currently don't need an disable_fiq macro */

		.macro	disable_fiq
		.endm
