/*
 *  hndeng06@gmail.com .
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

#ifndef _LINUX_SERIAL_LX28XX_H
#define _LINUX_SERIAL_LX28XX_H

#include <linux/serial_core.h>
#include <linux/device.h>

#define LX28XX_NR_PORTS	2

struct lx28xx_port {
	struct uart_port	port;
	struct timer_list	timer;
	unsigned int		old_status;
};

struct inno_uart_port {
	struct uart_port port;
	unsigned long baud;
};



/*
 * Access routines for the DMP UARTs
 */
#define	rUARTDR		0x00
#define	rUARTRSR	0x04
#define	rUARTFR		0x18
#define	rUARTILPR	0x20
#define	rUARTIBRD	0x24
#define	rUARTFBRD	0x28
#define	rUARTLCR_H	0x2C
#define	rUARTCR		0x30
#define	rUARTIFLS	0x34
#define	rUARTIMSC	0x38
#define	rUARTRIS	0x3C
#define	rUARTMIS	0x40
#define	rUARTICR	0x44
#define	rUARTDMACR	0x48

#define	IO_READ(x)		(*(volatile unsigned int *)(x))
#define	IO_WRITE(x, c)	*(volatile unsigned int *)(x) = c

#define UART_GET_INT_STATUS(p)	IO_READ((p)->uart_base + rUARTRIS)
#define UART_GET_FR(p)		IO_READ((p)->uart_base + rUARTFR)
#define UART_GET_CHAR(p)	IO_READ((p)->uart_base + rUARTDR)
#define UART_PUT_CHAR(p, c)	IO_WRITE((p)->uart_base + rUARTDR, (c))
#define UART_GET_RSR(p)		IO_READ((p)->uart_base + rUARTRSR)
#define UART_GET_CR(p)		IO_READ((p)->uart_base + rUARTCR)
#define UART_PUT_CR(p,c)	IO_WRITE((p)->uart_base + rUARTCR, (c))
#define UART_GET_IBRD(p)	IO_READ((p)->uart_base + rUARTIBRD)
#define UART_PUT_IBRD(p,c)	IO_WRITE((p)->uart_base + rUARTIBRD, (c))
#define UART_GET_FBRD(p)	IO_READ((p)->uart_base + rUARTFBRD)
#define UART_PUT_FBRD(p,c)	IO_WRITE((p)->uart_base + rUARTFBRD, (c))
#define UART_GET_LCRH(p)	IO_READ((p)->uart_base + rUARTLCR_H)
#define UART_PUT_LCRH(p,c)	IO_WRITE((p)->uart_base + rUARTLCR_H, (c))
#define UART_GET_IFLS(p)	IO_READ((p)->uart_base + rUARTIFLS)
#define UART_PUT_IFLS(p,c)	IO_WRITE((p)->uart_base + rUARTIFLS, (c))
#define UART_GET_IMSC(p)	IO_READ((p)->uart_base + rUARTIMSC)
#define UART_PUT_IMSC(p,c)	IO_WRITE((p)->uart_base + rUARTIMSC, (c))
#define UART_GET_ICR(p)		IO_READ((p)->uart_base + rUARTICR)
#define UART_PUT_ICR(p,c)	IO_WRITE((p)->uart_base + rUARTICR, (c))

#define DMP_UARTIMSC_RIE	0x10	/* Enable receive interrupt */
#define DMP_UARTIMSC_TIE	0x20	/* Enable transmit interrupt */
#define DMP_UARTIMSC_RTIE	0x40
#define DMP_UARTIMSC_MIS    0x0f

#define DMP_UARTRSR_OE      0x08
#define DMP_UARTRSR_BE      0x04
#define DMP_UARTRSR_PE      0x02
#define DMP_UARTRSR_FE      0x01

#define DMP_UARTFR_TXFE     0x80
#define DMP_UARTFR_RXFF     0x40
#define DMP_UARTFR_TXFF     0x20
#define DMP_UARTFR_RXFE     0x10
#define DMP_UARTFR_BUSY     0x08
#define DMP_UARTFR_DCD		0x04
#define DMP_UARTFR_DSR		0x02
#define DMP_UARTFR_CTS		0x01
#define DMP_UARTFR_TMSK     (DMP_UARTFR_TXFF + DMP_UARTFR_BUSY)
 
#define DMP_UARTCR_TIE                 0x100
#define DMP_UARTCR_RIE                 0x200
#define DMP_UARTCR_UARTEN              0x01
 
#define DMP_UARTLCR_H_WLEN_8           0x60
#define DMP_UARTLCR_H_WLEN_7           0x40
#define DMP_UARTLCR_H_WLEN_6           0x20
#define DMP_UARTLCR_H_WLEN_5           0x00
#define DMP_UARTLCR_H_FEN              0x10
#define DMP_UARTLCR_H_STP2             0x08
#define DMP_UARTLCR_H_EPS              0x04
#define DMP_UARTLCR_H_PEN              0x02
#define DMP_UARTLCR_H_BRK              0x01


#define DMP_UARTRIS_RTIS               DMP_UARTIMSC_RTIE
#define DMP_UARTRIS_TIS                DMP_UARTIMSC_TIE
#define DMP_UARTRIS_RIS                DMP_UARTIMSC_RIE
#define DMP_UARTRIS_MIS                0x0f


#define UART_RX_DATA(s)		(((s) & DMP_UARTFR_RXFE) == 0)
#define UART_TX_READY(s)	(((s) & DMP_UARTFR_TXFF) == 0)
#define UART_TX_EMPTY(p)	((UART_GET_FR(p) & DMP_UARTFR_TMSK) == 0)

#define DMP_UARTRSR_ANY			(DMP_UARTRSR_OE|DMP_UARTRSR_BE|DMP_UARTRSR_PE|DMP_UARTRSR_FE)
#define DMP_UARTFR_MODEM_ANY	(DMP_UARTFR_DCD|DMP_UARTFR_DSR|DMP_UARTFR_CTS)




/* register offsets */
#define LX28XX_LCR		0
#define LX28XX_MCR		0x004
#define LX28XX_BAUD		0x008
#define LX28XX_CFG		0x00c
#define LX28XX_FIFO		0x028
#define LX28XX_ISTAT		rUARTRIS
#define LX28XX_IEN		0xfe4
#define LX28XX_ICLR		0xfe8
#define LX28XX_ISET		0xfec
#define LX28XX_PD		0xff4
#define LX28XX_MID		0xffc

#define LX28XX_UART_LCR_TXBREAK	(1<<0)
#define LX28XX_UART_LCR_PAREVN		(1<<2)
#define LX28XX_UART_LCR_PAREN		(1<< 1)
#define LX28XX_UART_LCR_2STOPB		(1<< 3)
#define LX28XX_UART_LCR_8BIT		(3<<5)
#define LX28XX_UART_LCR_7BIT		(2<<5)
#define LX28XX_UART_LCR_TX_RST		0x00000000
#define LX28XX_UART_LCR_RX_RST		0x00000000
#define LX28XX_UART_LCR_RX_NEXT	0x00010000
#define LX28XX_UART_LCR_ENABLE_FIFO	(1<< 4) /* Enable the FIFO */


#define LX28XX_UART_MCR_SCR		0xFF000000
#define LX28XX_UART_MCR_DCD		0x00800000
#define LX28XX_UART_MCR_CTS		0x00100000
#define LX28XX_UART_MCR_LOOP		0x00000010
#define LX28XX_UART_MCR_RTS		0x00000002
#define LX28XX_UART_MCR_DTR		0x00000001

#define LX28XX_UART_INT_TX		(1<< 5)
#define LX28XX_UART_INT_EMPTY		0x00000040
#define LX28XX_UART_INT_RCVTO		0x00000020
#define LX28XX_UART_INT_RX		(1<< 4)
#define LX28XX_UART_INT_RXOVRN		0x00000008
#define LX28XX_UART_INT_FRERR		0x00000004
#define LX28XX_UART_INT_BREAK		(1<< 9)
#define LX28XX_UART_INT_PARITY		0x00000001
#define LX28XX_UART_INT_ALLRX		(1<< 4 | 1 << 6)
#define LX28XX_UART_INT_ALLTX		(1<< 5)




#define LX28XX_UART_FIFO_TXFIFO	0x001F0000
#define LX28XX_UART_FIFO_TXFIFO_STA	(1<<5)
#define LX28XX_UART_FIFO_RXBRK		0x00008000
#define LX28XX_UART_FIFO_RXPAR		0x00002000
#define LX28XX_UART_FIFO_RXFIFO	0x00001F00
#define LX28XX_UART_FIFO_RBRTHR	0x000000FF

#define LX28XX_UART_FIFO_TXFF		(1<<5)
#define LX28XX_UART_FIFO_TXFE		(1<<7)
#define LX28XX_UART_FIFO_RXFF		(1<<6)
#define LX28XX_UART_FIFO_RXFE		(1<<4)

/*UART  UARTRSR/UARTECR*/

#define LX28XX_UARTECR_FE		(1<<0)
#define LX28XX_UARTECR_PE		(1<<1)
#define LX28XX_UARTECR_BE		(1<<2)
#define LX28XX_UARTECR_OE	(1<<3)

unsigned int get_tx_status(void);


#endif


