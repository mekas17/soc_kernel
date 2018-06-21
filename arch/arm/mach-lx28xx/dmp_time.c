/*
 *  linux/arch/armnommu/mach-atmel/time.c
 *
 *  Copyright (C) SAMSUNG ELECTRONICS 
 *                      Hyok S. Choi <hyok.choi@samsung.com>
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
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/clockchips.h>
//#include <asm/system.h>
//#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
                                                                                                                                           
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/clock_t18xx.h>
#include <asm/setup.h>


//#define RTC

#ifdef RTC
#define YEAR_VALUE	(BcdToValue(rRTCYEAR_reg & 0xffff))
#define MON_VALUE	(BcdToValue(rRTCMON_reg & 0x1f))
#define DATE_VALUE	(BcdToValue(rRTCDATE_reg & 0x3f))
#define HOUR_VALUE	(BcdToValue(rRTCHOUR_reg & 0x3f))
#define MIN_VALUE	(BcdToValue(rRTCMIN_reg & 0x7f))
#define SEC_VALUE	(BcdToValue(rRTCSEC_reg & 0x7f))

static unsigned int  ValueToBcd(unsigned int value)
{
	unsigned int j = 0, bcd = 0;

	for ( j = 0; j < 16; j ++)
	{
		if ( value == 0 )
			break;
		bcd += (value % 10)<<(j<<2);
		value = value/10;
	}
	return bcd;
}

static unsigned int BcdToValue(unsigned int bcd)
{
	unsigned int i, value = 0, temp = 1;

	for (i = 0; i < 8; i ++)
	{
		if (bcd == 0)
			break;
		value += (bcd & 0x0f) * temp;
		temp *= 10;
		bcd >>= 4;
	}
	return value;
}

static unsigned long ark_dmp_rtc_get_time(void)
{
	unsigned long temp = 0;
	unsigned long sec = mktime(YEAR_VALUE, MON_VALUE, DATE_VALUE, HOUR_VALUE, MIN_VALUE, SEC_VALUE);

	while(sec != temp )
	{
		temp = sec;
		sec = mktime(YEAR_VALUE, MON_VALUE, DATE_VALUE, HOUR_VALUE, MIN_VALUE, SEC_VALUE);
	}
	return sec;
}

static int ark_dmp_rtc_set_time(void)
{
	unsigned long sec;
	unsigned long year, mon, date, hour, min;

	sec = xtime.tv_sec;

	year = 1970;
	while(sec >= mktime(year++, 1, 1, 0, 0, 0));
	year -= 2;
	rRTCYEAR_reg = ValueToBcd(year);

	for ( mon = 2; mon < 13; mon ++ )
		if (sec < mktime(year, mon, 1, 0, 0, 0))
			break;
	mon --;
	rRTCMON_reg = ValueToBcd(mon);

	sec -= mktime(year, mon, 1, 0, 0, 0);
	rRTCSEC_reg = ValueToBcd(sec % 60);

	min = sec/60;
	rRTCMIN_reg = ValueToBcd(min % 60);

	hour = min/60;
	rRTCHOUR_reg = ValueToBcd(hour % 24);

	date = hour/24 + 1;
	rRTCDATE_reg = ValueToBcd(date);
	return 0;
}

extern int (*set_rtc)(void);

#endif

#ifndef CONFIG_GENERIC_CLOCKEVENTS

static unsigned long timer_startval;

/***
 * Returns microsecond  since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 * IRQs are disabled before entering here from do_gettimeofday()
 */

static unsigned long ark_dmp_gettimeoffset (void)
{
	unsigned long tdone;
	unsigned long usec;

	/* work out how many ticks have gone since last timer interrupt */

	tdone = timer_startval - rTCNT0;

	/* check to see if there is an interrupt pending */
	if (rICPEND & (1 << LX28XX_TIMER0_INT)) {
		/* re-read the timer, and try and fix up for the missed
		 * interrupt */
		tdone = 2 * timer_startval - rTCNT0;
	}

	usec = (tdone * (1000000/HZ)) / timer_startval;
	return usec;
}


/*
 * IRQ handler for the timer
 */
static irqreturn_t
ark_dmp_timer_interrupt(int irq, void *dev_id)
{
	//early_print("timer_tick\n");
	timer_tick();

	return IRQ_HANDLED;
}

static struct irqaction ark_dmp_timer_irq = {
	.name		= "timer0",
	.flags		= IRQF_TIMER | IRQF_IRQPOLL, //IRQF_TIMER,
	.handler	= ark_dmp_timer_interrupt
};

void dump_irq_info(void)
{
	printk("rICSET =%x\n", rICSET);
	printk("rICMODE =%x\n", rICMODE);
	printk("rICMASK =%x\n", rICMASK);
	printk("rICLEVEL =%x\n", rICLEVEL);
}

/*
 * Set up timer interrupt, and return the current time in seconds.
 *
 * Currently we only use timer4, as it is the only timer which has no
 * other function that can be exploited externally
 */

#ifdef CONFIG_FPGA_DEBUG

#else
void __init lx28xx_init_time (void)
{
	volatile int prieod,i ;

	//lx28xx_clock_init();
	rTCTL0 = 0x0;
	
	for(i=0; i< 200; i++);
	rTPRS0 = 120-1;
	rTMOD0 = (36000000/36)/HZ;//HZ;

	rTCTL0 = 0x07 | (1 << 6); //bit 6 --> start bit

	rTSTART0 = 1;

	timer_startval = rTMOD0;
	
	setup_irq(LX28XX_TIMER0_INT, &ark_dmp_timer_irq);
	early_print("setup_irq time0 end rICMASK=%x\n", rICMASK);
	
	//dump_irq_info();

}
#endif

#else  // CONFIG_GENERIC_CLOCKEVENTS

//#define LX28XX_TIMER_CLOCK       120000000
#define LX28XX_TIMER_CLOCK       60000000
#define LX28XX_TIMER_PRESCALER   12
#define LX28XX_TIMER_RATE        (LX28XX_TIMER_CLOCK/LX28XX_TIMER_PRESCALER)
#if 0
static void lx28xx_set_mode(enum clock_event_mode mode,
			    struct clock_event_device *evt)
{
	if (mode == CLOCK_EVT_FEAT_PERIODIC) 
    {
		/* Disable timer */
        rTCTL0 = 0x07;
        rTMOD0 = LX28XX_TIMER_RATE/HZ;
        
		/* Re-enable timer */
        rTCTL0 = 0x07 | (1 << 6); //bit 6 --> start bit
        rTSTART0 = 1;
	}
}
#endif
static int lx28xx_set_next_event(unsigned long cycles,
				 struct clock_event_device *evt)

{
	/* Disable timer */
    rTCTL0 = 0x07;

	/* Both registers need to count down */
    rTMOD0 = cycles;

    /* Re-enable timer */
    rTCTL0 = 0x07 | (1 << 6); //bit 6 --> start bit

	rTSTART0 = 1;
		
	return 0;
}

static struct clock_event_device clockevent_lx28xx = {
	.name		= "lx28xx_timer",
	.rating		= 300, /* Reasonably fast and accurate clock event */
	.features	= CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.set_next_event	= lx28xx_set_next_event,
	//.set_mode	= lx28xx_set_mode,
};

/*
 * IRQ handler for the timer.
 */
static irqreturn_t lx28xx_timer_interrupt(int irq, void *dev_id)
{
	//early_print("lx28xx_timer_interrupt\n");
	rTITCLR = bTINTT0;

	struct clock_event_device *evt = &clockevent_lx28xx;

	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction lx28xx_timer_irq = {
	.name		= "lx28xx_tick",
	.flags		= (IRQF_TIMER | IRQF_TRIGGER_RISING),
	.handler	= lx28xx_timer_interrupt,
};

static void lx28xx_timer_setup(void)
{
	/* Disable timer */
    rTCTL0 = 0x07;
	rTPRS0 = LX28XX_TIMER_PRESCALER-1;
    rTMOD0 = LX28XX_TIMER_RATE/HZ;
    
	/* Re-enable timer */
    rTCTL0 = 0x07 | (1 << 6); //bit 6 --> start bit

	rTSTART0 = 1;

	rTITEN = 1;
	rGINTREN= 1;

	clockevents_config_and_register(&clockevent_lx28xx,
				LX28XX_TIMER_RATE, 100,
				0xFFFFFFFFU);

    rTCTL1 = 0x07;
    rTPRS1 = LX28XX_TIMER_PRESCALER-1;
    rTMOD1 = LX28XX_TIMER_RATE/HZ;
    
	/* Re-enable timer */
    rTCTL1 = 0x07 | (1 << 5) | (1 << 6); //bit 6 --> start bit
    rTSTART1 = 1;
}

void __init lx28xx_init_time(void)
{

	early_print("lx28xx_init_time\n");
	lx28xx_timer_setup();

	/* Enable timer interrupts */
	setup_irq((LX28XX_TIMER0_INT+32), &lx28xx_timer_irq);
#if 0
	while (1){
		early_print("rICPEND_31_0: %d\n", rICPEND_31_0);
	}
#endif
}


#endif // CONFIG_GENERIC_CLOCKEVENTS






