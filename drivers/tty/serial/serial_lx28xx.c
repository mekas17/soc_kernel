/*
 * UART driver for LX28XX SoCs
 *
 * Author:   jjdeng@gmail.com 
 * Ported to 2.6 kernel by EmbeddedAlley
 * Reworked by Vitaly Wool <vitalywool@gmail.com>
 *
 * Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 * Copyright (C) 2000 Deep Blue Solutions Ltd.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of
 * any kind, whether express or implied.
 *
 * NOTES: uart fifo depth is 16byte, In addition, the uart modules don't support tx empty interrupt
 * So we first write the tx data before enable the tx interrupt. the tx interrupt only occur when tx
 * fifo cnt is less than the 1/x fifo depth.
 * 
 
 */


#if defined(CONFIG_SERIAL_LX28XX_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_lx28xx.h>
#include <mach/hardware.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/setup.h>


//#define UART_DEBUG

#ifdef UART_DEBUG
#define	UART_printk(format, args...)	\
	printk("UART %s line %d: " format, __func__, __LINE__, ##args)
#else
#define	UART_printk(foramt, args...)
#endif


/* We'll be using StrongARM sa1100 serial port major/minor */
#define SERIAL_LX28XX_NAME	"ttyS"
#define SERIAL_LX28XX_MAJOR	204
#define MINOR_START		5

#define NR_PORTS		2

#define LX28XX_ISR_PASS_LIMIT	256

#define RXSTAT_DUMMY_READ (0x10000000)




/*
 * This is the size of our serial port register set.
 */
#define UART_PORT_SIZE	0x1000

/*
 * This determines how often we check the modem status signals
 * for any change.  They generally aren't connected to an IRQ
 * so we have to poll them.  We also check immediately before
 * filling the TX fifo incase CTS has been dropped.
 */
#define MCTRL_TIMEOUT	(250*HZ/1000)

static void lx28xx_tx_chars(struct lx28xx_port *sport);

extern struct lx28xx_port lx28xx_ports[];

static inline int serial_in(struct lx28xx_port *sport, int offset)
{
	return (__raw_readl(sport->port.membase + offset));
}

static inline void serial_out(struct lx28xx_port *sport, int offset, int value)
{
	__raw_writel(value, sport->port.membase + offset);
}

void dump_uart_reg(void)
{
	printk("rUARTIMSC=%x\n", __raw_readl(UART0_BASE + rUARTIMSC));
	printk("rUARTRIS=%x\n", __raw_readl(UART0_BASE + rUARTRIS));
	printk("rUARTMIS=%x\n", __raw_readl(UART0_BASE + rUARTMIS));

}

/*
 * Handle any change of modem status signal since we were last called.
 */
static void lx28xx_mctrl_check(struct lx28xx_port *sport)
{
#if 0
	unsigned int status, changed;
	status = sport->port.ops->get_mctrl(&sport->port);
	changed = status ^ sport->old_status;

	if (changed == 0)
		return;

	sport->old_status = status;

	if (changed & TIOCM_RI)
		sport->port.icount.rng++;
	if (changed & TIOCM_DSR)
		sport->port.icount.dsr++;
	if (changed & TIOCM_CAR)
		uart_handle_dcd_change(&sport->port, status & TIOCM_CAR);
	if (changed & TIOCM_CTS)
		uart_handle_cts_change(&sport->port, status & TIOCM_CTS);

	wake_up_interruptible(&sport->port.state->port.delta_msr_wait);
#endif
}

/*
 * This is our per-port timeout handler, for checking the
 * modem status signals.
 */
static void lx28xx_timeout(unsigned long data)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)data;
	unsigned long flags;

	if (sport->port.state) {
		spin_lock_irqsave(&sport->port.lock, flags);
		lx28xx_mctrl_check(sport);
		spin_unlock_irqrestore(&sport->port.lock, flags);

		mod_timer(&sport->timer, jiffies + MCTRL_TIMEOUT);
	}
}

/*
 * interrupts disabled on entry
 */
static void lx28xx_stop_tx(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	u32 ien;
	//UART_printk("%s: entered\n", __func__);

	/* Disable TX intr */
	ien = serial_in(sport, rUARTIMSC);
	serial_out(sport, rUARTIMSC, ien & ~LX28XX_UART_INT_ALLTX);
	
	/* Clear all pending TX intr */
	serial_out(sport, rUARTICR, LX28XX_UART_INT_ALLTX);
}

/*
 * 
 //because of hw interrupt problem, we don't use interrupt
 
 */
static void lx28xx_start_tx(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	u32 ien;
	//printk("%s: entered\n", __func__);

	/* Clear all pending TX intr  */
	serial_out(sport, rUARTICR, LX28XX_UART_INT_ALLTX);

	/* Enable TX intr */
	ien = serial_in(sport, rUARTIMSC);
	serial_out(sport, rUARTIMSC, ien | LX28XX_UART_INT_ALLTX);

    lx28xx_tx_chars(sport);
}



/*
 * Interrupts enabled
 */
static void lx28xx_stop_rx(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	u32 ien;

	/* Disable RX intr */
	ien = serial_in(sport, rUARTIMSC);
	serial_out(sport, rUARTIMSC, ien & ~LX28XX_UART_INT_ALLRX);

	/* Clear all pending RX intr */
	serial_out(sport, rUARTICR, LX28XX_UART_INT_ALLRX);
}

/*
 * Set the modem control timer to fire immediately.
 */
static void lx28xx_enable_ms(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;

	mod_timer(&sport->timer, jiffies);
}

static void lx28xx_rx_chars(struct lx28xx_port *sport)
{
	struct tty_struct *tty = sport->port.state->port.tty;
	unsigned int status, ch, flg;
	unsigned int err_status;

	status = (serial_in(sport, rUARTFR)) ;//|(serial_in(sport, LX28XX_ISTAT));
	err_status = serial_in(sport, rUARTRSR);
	while (!(status & LX28XX_UART_FIFO_RXFE)) {

		ch = serial_in(sport, rUARTDR) & 0xff;
		sport->port.icount.rx++;

		flg = TTY_NORMAL;
		

		/*
		 * note that the error handling code is
		 * out of the main execution path
		 */
		#if 1 // FIXME
		if (err_status & ((LX28XX_UARTECR_FE |
					LX28XX_UARTECR_PE |
					LX28XX_UARTECR_BE) |
				  (LX28XX_UARTECR_OE))) {
			if (err_status & (LX28XX_UARTECR_BE)) {
				err_status &= ~((LX28XX_UARTECR_BE) |
					(LX28XX_UARTECR_PE));
				sport->port.icount.brk++;
				if (uart_handle_break(&sport->port))
					goto ignore_char;
			} else if (err_status & (LX28XX_UARTECR_PE))
				sport->port.icount.parity++;
			else if (err_status & (LX28XX_UARTECR_FE))
				sport->port.icount.frame++;
			if (err_status & (LX28XX_UARTECR_OE))
				sport->port.icount.overrun++;

			err_status &= sport->port.read_status_mask;

			if (err_status & (LX28XX_UARTECR_PE))
				flg = TTY_PARITY;
			else if (err_status & (LX28XX_UARTECR_FE))
				flg = TTY_FRAME;

		#ifdef SUPPORT_SYSRQ
			sport->port.sysrq = 0;
		#endif
		serial_out(sport, rUARTRSR, err_status);
		}
		#endif

		if (uart_handle_sysrq_char(&sport->port, ch))
			goto ignore_char;

		uart_insert_char(&sport->port, err_status,
				(LX28XX_UARTECR_OE), ch, flg);
	//	printk("!!rx ch=%c, err_status=%x,flg=%x\n", ch, err_status, flg);
		//printk("!!rx ch=%c\n", ch);
	ignore_char:

		status =  serial_in(sport, rUARTFR);
		err_status = serial_in(sport, rUARTRSR);
	}
	tty_flip_buffer_push(tty->port);
}

static void lx28xx_tx_chars(struct lx28xx_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;
	int count =0;
	if (sport->port.x_char) {
		serial_out(sport, rUARTDR, sport->port.x_char);
		sport->port.icount.tx++;
		sport->port.x_char = 0;
		return;
	}

	/*
	 * Check the modem control lines before
	 * transmitting anything.
	 */
	lx28xx_mctrl_check(sport);

	if (uart_circ_empty(xmit) || uart_tx_stopped(&sport->port)) {
		lx28xx_stop_tx(&sport->port);
		return;
	}

	/*
	 * TX while bytes available
	 */
	while (!(serial_in(sport, rUARTFR) & LX28XX_UART_FIFO_TXFF)) {
		//spin_lock_irqsave(&sport->port.lock, flags);
		serial_out(sport, rUARTDR, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
		count++;
		//spin_unlock_irqrestore(&sport->port.lock, flags);
		if (uart_circ_empty(xmit))
			break;
	}
	UART_printk("tx count =%d\n", count);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		lx28xx_stop_tx(&sport->port);
}

static irqreturn_t lx28xx_int(int irq, void *dev_id)
{
	struct lx28xx_port *sport = dev_id;
	unsigned int status;

	spin_lock(&sport->port.lock);
	/* Get the interrupts */
	status  = serial_in(sport, LX28XX_ISTAT) ;//& serial_in(sport, rUARTIMSC);
	UART_printk("uart int status=%x\n", status);

	/* Byte or break signal received */
	if (status & (LX28XX_UART_INT_RX | DMP_UARTIMSC_RTIE)){
		//UART_printk("begain lx28xx_rx_chars status =0x%x\n", status);
		lx28xx_rx_chars(sport);
    }

	/* TX holding register empty - transmit a byte */
	if (status & (LX28XX_UART_INT_TX)){
		//printk("\n lx28xx_tx_chars status =0x%x\n", status);
		lx28xx_tx_chars(sport);
	}

	/* Clear the ISTAT register */
	serial_out(sport, rUARTICR, status );

	spin_unlock(&sport->port.lock);
	return IRQ_HANDLED;
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int lx28xx_tx_empty(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;

	return serial_in(sport, rUARTFR) & LX28XX_UART_FIFO_TXFE? TIOCSER_TEMT : 0;
}

static unsigned int lx28xx_get_mctrl(struct uart_port *port)
{

	return TIOCM_DSR;
}

static void lx28xx_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
#if	0	/* FIXME */
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	unsigned int msr;
#endif
}

/*
 * Interrupts always disabled.
 */
static void lx28xx_break_ctl(struct uart_port *port, int break_state)
{
}

static int lx28xx_startup(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	int retval;
	UART_printk("%s: entered\n", __func__);

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(sport->port.irq, lx28xx_int, 0,
			     "lx28xx-uart", sport);
	if (retval)
		return retval;

	/*
	 * Finally, clear and disenable interrupts*/
	

	serial_out(sport, rUARTICR, -1);

	serial_out(sport, rUARTIMSC, 0x50);

	

	/* 
	 * Enable modem status interrupts
	 
	spin_lock_irq(&sport->port.lock);
	lx28xx_enable_ms(&sport->port);
	spin_unlock_irq(&sport->port.lock);
	*/

	return 0;
}

static void lx28xx_shutdown(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	int lcr;

	/*
	 * Stop our timer.
	 */
	del_timer_sync(&sport->timer);

	/*
	 * Disable all interrupts
	 */
	serial_out(sport, rUARTIMSC, 0);

	/*
	 * Reset the Tx and Rx FIFOS, disable the break condition
	 */
	lcr = serial_in(sport, rUARTLCR_H);
	lcr &= ~LX28XX_UART_LCR_TXBREAK;
	//lcr |= LX28XX_UART_LCR_TX_RST | LX28XX_UART_LCR_RX_RST;
	serial_out(sport, rUARTLCR_H, lcr);

	/*
	 * Clear all interrupts
	 */
	serial_out(sport, rUARTICR, LX28XX_UART_INT_ALLRX |
			     LX28XX_UART_INT_ALLTX);

	/*
	 * Free the interrupt
	 */
	free_irq(sport->port.irq, sport);
}

static void
lx28xx_set_termios(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	unsigned long flags;
	unsigned int lcr_fcr, old_ien, baud, quot;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;

	/*
	 * We don't support modem control lines.
	 */
	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	
#if 1
	/*
	 * We only support CS7 and CS8.
	 */
	while ((termios->c_cflag & CSIZE) != CS7 &&
	       (termios->c_cflag & CSIZE) != CS8) {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= old_csize;
		old_csize = CS8;
	}

	if ((termios->c_cflag & CSIZE) == CS8)
		lcr_fcr = LX28XX_UART_LCR_8BIT;
	else
		lcr_fcr = 0;

	if (termios->c_cflag & CSTOPB)
		lcr_fcr |= LX28XX_UART_LCR_2STOPB;
	if (termios->c_cflag & PARENB) {
		lcr_fcr |= LX28XX_UART_LCR_PAREN;
		if (!(termios->c_cflag & PARODD))
			lcr_fcr |= LX28XX_UART_LCR_PAREVN;
	}

	lcr_fcr |= LX28XX_UART_LCR_ENABLE_FIFO;
	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, 115200*8);

	if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST)
		quot = port->custom_divisor;
	else
		quot =  (port->uartclk * 64 + 8 * baud)/(16*baud);



	spin_lock_irqsave(&sport->port.lock, flags);

	sport->port.read_status_mask = (LX28XX_UARTECR_OE);
	if (termios->c_iflag & INPCK)
		sport->port.read_status_mask |= (LX28XX_UARTECR_FE) |
			(LX28XX_UARTECR_PE);


	/*
	 * Characters to ignore
	 */
	sport->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		sport->port.ignore_status_mask |=LX28XX_UARTECR_OE;
	if (termios->c_iflag & IGNBRK) {
		sport->port.ignore_status_mask |=(LX28XX_UARTECR_OE);
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			sport->port.ignore_status_mask |=LX28XX_UARTECR_FE;
				
	}
	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		sport->port.ignore_status_mask |= RXSTAT_DUMMY_READ;

	del_timer_sync(&sport->timer);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * disable interrupts and drain transmitter
	 */
	old_ien = serial_in(sport, rUARTIMSC);
	serial_out(sport, rUARTIMSC, 0);

	#if 0
	while (!(serial_in(sport, rUARTFR) & LX28XX_UART_FIFO_TXFE))
		barrier();

	/* then, disable everything */
	serial_out(sport, rUARTIMSC, 0);
	#endif
	/* set the parity, stop bits and data size */
	serial_out(sport, rUARTLCR_H, lcr_fcr);

	/* Reset the Rx and Tx FIFOs too
	lcr_fcr |= LX28XX_UART_LCR_TX_RST;
	lcr_fcr |= LX28XX_UART_LCR_RX_RST; */
	serial_out(sport, rUARTIFLS, 0x12); // 1/2 fifo depth


	/* set the baud rate */
	//quot -= 1;
	//FIXME serial bard rate set should be enable
	/*
	serial_out(sport, rUARTIBRD, quot >> 6);
	serial_out(sport, rUARTFBRD, quot & 0x3f);
	*/
	serial_out(sport, rUARTICR, -1);

	serial_out(sport, rUARTIMSC, old_ien);


	spin_unlock_irqrestore(&sport->port.lock, flags);
	/*printk("rUARTLCR_H=%x, rUARTIMSC=%x,rUARTCR=%x\n", \
		serial_in(sport,rUARTLCR_H),serial_in(sport,rUARTIMSC), serial_in(sport, rUARTCR));*/
	//UART_printk("sport->port.ignore_status_mask= %x\n", sport->port.ignore_status_mask);
	
#endif

}

static const char *lx28xx_type(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;

	return sport->port.type == PORT_LX28XX ? "LX28XX" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void lx28xx_release_port(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;

	release_mem_region(sport->port.mapbase, UART_PORT_SIZE);
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int lx28xx_request_port(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	return request_mem_region(sport->port.mapbase, UART_PORT_SIZE,
			"lx28xx-uart") != NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void lx28xx_config_port(struct uart_port *port, int flags)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;

	if (flags & UART_CONFIG_TYPE &&
	    lx28xx_request_port(&sport->port) == 0)
		sport->port.type = PORT_LX28XX;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_LX28XX and PORT_UNKNOWN
 */
static int
lx28xx_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_LX28XX)
		ret = -EINVAL;
	if (sport->port.irq != ser->irq)
		ret = -EINVAL;
	UART_printk("ret =%x\n", ret);	
	return ret;
}

static struct uart_ops lx28xx_pops = {
	.tx_empty	= lx28xx_tx_empty,
	.set_mctrl	= lx28xx_set_mctrl,
	.get_mctrl	= lx28xx_get_mctrl,
	.stop_tx	= lx28xx_stop_tx,
	.start_tx	= lx28xx_start_tx,
	.stop_rx	= lx28xx_stop_rx,
	.enable_ms	= lx28xx_enable_ms,
	.break_ctl	= lx28xx_break_ctl,
	.startup	= lx28xx_startup,
	.shutdown	= lx28xx_shutdown,
	.set_termios	= lx28xx_set_termios,
	.type		= lx28xx_type,
	.release_port	= lx28xx_release_port,
	.request_port	= lx28xx_request_port,
	.config_port	= lx28xx_config_port,
	.verify_port	= lx28xx_verify_port,
};


void lx28xx_tx_write(struct uart_port *port)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	struct circ_buf *xmit = &sport->port.state->xmit;
	int count =0;
	if (sport->port.x_char) {
		serial_out(sport, rUARTDR, sport->port.x_char);
		sport->port.icount.tx++;
		sport->port.x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit)) {
		lx28xx_stop_tx(&sport->port);
		return;
	}
	/*
	 * TX while bytes available
	 */
	while (!(serial_in(sport, rUARTFR) &LX28XX_UART_FIFO_TXFF)) {
		//spin_lock_irqsave(&sport->port.lock, flags);
		serial_out(sport, rUARTDR, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
		count++;
		//spin_unlock_irqrestore(&sport->port.lock, flags);
		if (uart_circ_empty(xmit))
			break;
	}


	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		lx28xx_stop_tx(&sport->port);
}


/*
 * Setup the LX28XX serial ports.
 *
 * Note also that we support "console=ttySx" where "x" is either 0 or 1.
 */
static void __init lx28xx_init_ports(void)
{
	static int first = 1;
	int i;

	if (!first)
		return;
	first = 0;

	for (i = 0; i < NR_PORTS; i++) {
		init_timer(&lx28xx_ports[i].timer);
		lx28xx_ports[i].timer.function = lx28xx_timeout;
		lx28xx_ports[i].timer.data     = (unsigned long)&lx28xx_ports[i];
		lx28xx_ports[i].port.ops = &lx28xx_pops;
	}
}

#ifdef CONFIG_SERIAL_LX28XX_CONSOLE

static void lx28xx_console_putchar(struct uart_port *port, int ch)
{
	struct lx28xx_port *sport = (struct lx28xx_port *)port;
	int status; 

	do {
		/* Wait for UART_TX register to empty */
		status = serial_in(sport, rUARTFR);
	} while (status & LX28XX_UART_FIFO_TXFF);
	serial_out(sport, rUARTDR, ch); 
}

/*
 * Interrupts are disabled on entering
 */static void
lx28xx_console_write(struct console *co, const char *s, unsigned int count)
{
	struct lx28xx_port *sport = &lx28xx_ports[co->index];
	unsigned int old_ien, status;
	
	//uart_console_write(&sport->port, s, count, lx28xx_console_putchar);
#if 1
	/*
	 *	First, save IEN and then disable interrupts
	 */
	old_ien = serial_in(sport, rUARTIMSC);
	serial_out(sport, rUARTIMSC, 0);

	uart_console_write(&sport->port, s, count, lx28xx_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore IEN
	 */
	do {
		/* Wait for UART_TX register to empty */
		status = serial_in(sport, rUARTFR);
	} while (status & LX28XX_UART_FIFO_TXFF);	 

	/* Clear TX  interrupt 
	serial_out(sport, rUARTICR, LX28XX_UART_INT_TX | LX28XX_UART_INT_ALLRX);
	*/

	serial_out(sport, rUARTIMSC, old_ien);//old_ien);
#endif


}

static int __init
lx28xx_console_setup(struct console *co, char *options)
{
	struct lx28xx_port *sport;
	int baud = 38400;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	 UART_printk("co->index=%d\n", co->index);
	if (co->index == -1 || co->index >= NR_PORTS)
		co->index = 0;
	sport = &lx28xx_ports[co->index];

	if (options){
		uart_parse_options(options, &baud, &parity, &bits, &flow);
		
		UART_printk("baud=%d,parity=%c,bits=%d,flow=%c\n", baud, parity, bits,flow);
		uart_set_options(&sport->port, co, baud, parity, bits, flow);
		}
	
	return 0;
}

static struct uart_driver lx28xx_reg;
static struct console lx28xx_console = {
	.name		= SERIAL_LX28XX_NAME,
	.write		= lx28xx_console_write,
	.device		= uart_console_device,
	.setup		= lx28xx_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &lx28xx_reg,
};

static int __init lx28xx_rs_console_init(void)
{
	//add_preferred_console(SERIAL_LX28XX_NAME, 0, NULL);
	
	lx28xx_init_ports();
	register_console(&lx28xx_console);
	return 0;
}
console_initcall(lx28xx_rs_console_init);

#define LX28XX_CONSOLE	&lx28xx_console
#else
#define LX28XX_CONSOLE	NULL
#endif

static struct uart_driver lx28xx_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= SERIAL_LX28XX_NAME,
	.dev_name		= SERIAL_LX28XX_NAME,
#if 0
	.major			= SERIAL_LX28XX_MAJOR,
	.minor			= MINOR_START,
#else 
	.major			= TTY_MAJOR,
	.minor			= 64,
#endif
	.nr			= NR_PORTS,
	.cons			= LX28XX_CONSOLE,
};

static int lx28xx_serial_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct lx28xx_port *sport = platform_get_drvdata(pdev);

	return uart_suspend_port(&lx28xx_reg, &sport->port);
}

static int lx28xx_serial_resume(struct platform_device *pdev)
{
	struct lx28xx_port *sport = platform_get_drvdata(pdev);

	return uart_resume_port(&lx28xx_reg, &sport->port);
}

static int lx28xx_serial_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	int i;

	for (i = 0; i < pdev->num_resources; i++, res++) {
		if (!(res->flags & IORESOURCE_MEM))
			continue;

		for (i = 0; i < NR_PORTS; i++) {
			if (lx28xx_ports[i].port.mapbase != res->start)
				continue;

			lx28xx_ports[i].port.dev = &pdev->dev;
			uart_add_one_port(&lx28xx_reg, &lx28xx_ports[i].port);
			platform_set_drvdata(pdev, &lx28xx_ports[i]);
			break;
		}
	}

	return 0;
}

static int lx28xx_serial_remove(struct platform_device *pdev)
{
	struct lx28xx_port *sport = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (sport)
		uart_remove_one_port(&lx28xx_reg, &sport->port);

	return 0;
}

static struct platform_driver lx28xx_serial_driver = {
	.driver		= {
		.name	= "lx28xx-uart",
		.owner	= THIS_MODULE,
	},
	.probe		= lx28xx_serial_probe,
	.remove		= lx28xx_serial_remove,
	.suspend	= lx28xx_serial_suspend,
	.resume		= lx28xx_serial_resume,
};

static int __init lx28xx_serial_init(void)
{
	int ret;

	printk(KERN_INFO "Serial: LX28XX driver\n");

	lx28xx_init_ports();

	ret = uart_register_driver(&lx28xx_reg);
	if (ret == 0) {
		ret = platform_driver_register(&lx28xx_serial_driver);
		if (ret)
			uart_unregister_driver(&lx28xx_reg);
	}
	return ret;
}

static void __exit lx28xx_serial_exit(void)
{
	platform_driver_unregister(&lx28xx_serial_driver);
	uart_unregister_driver(&lx28xx_reg);
}

module_init(lx28xx_serial_init);
module_exit(lx28xx_serial_exit);

MODULE_AUTHOR("jjdeng@gmail.com");
MODULE_DESCRIPTION("LX28XX SoCs serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(SERIAL_LX28XX_MAJOR);
MODULE_ALIAS("platform:lx28xx-uart");


