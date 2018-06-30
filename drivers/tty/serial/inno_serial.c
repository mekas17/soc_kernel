#if defined(CONFIG_SERIAL_INNO_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <mach/hardware.h>

#define R_IE       (0x04)  //rw must LCR[7]=0
#define R_LC       (0x0c)
#define R_DLF      (0xc0)
#define R_BR       (0x00)  //read must LCR[7]=0
#define R_TH       (0x00)  //write must LCR[7]=0
#define R_BAUDL	   (0x00)  //must LCR[7]=1
#define R_BAUDH	   (0x04)  //must LCR[7]=1
#define R_FC	   (0x08)  
#define R_II	   (0x08) 
#define R_CP	   (0xF4) 
#define R_STS	   (0x7c)
#define R_LS	   (0x14)
#define R_US	   (0x7C)
#define R_RXDATA	R_BR
#define R_TXDATA	R_TH
//#define APB_SYS_BASE       		(0x60000000)
#define PAD7CTL_OFFSET          (0x44*4)
#define PADMODE22_OFFSET        (0x5f*4)
#define PADMODE21_OFFSET        (0x5e*4)

//////////////////////////////////////////////////////////////////////
#define SUART_TX_FIFO_DEPTH         64
#define SUART_RX_FIFO_DEPTH         64

// IER
#define SUART_PTIME_INTEN           (1 << 7) // programmable THRE interrupt
#define SUART_EDSSI_INTEN           (1 << 3) // modern status interrupt
#define SUART_ELSI_INTEN            (1 << 2) // receiver line status interrupt
#define SUART_ETBEI_INTEN           (1 << 1) // transmit holding register empty interrupt
#define SUART_ERBFI_INTEN           (1 << 0) // received data available interrupt

// IIR
#define SUART_FIFOSE_DIS            (0x0 << 6)
#define SUART_FIFOSE_ENB            (0x3 << 6)
#define IS_SUART_FIFO_SEL(FIFO)     ((FIFO == SUART_FIFOSE_DIS) || \
                                    (FIFO == SUART_FIFOSE_ENB))

#define SUART_INTID_MODEM_STA       (0x0) // assert by asserting IER[3] and according events
                                      // de-assert by reading of the MSR
#define SUART_INTID_NOPENDING       (0x1)
#define SUART_INTID_THRE            (0x2)
#define SUART_INTID_RXDA            (0x4)
#define SUART_INTID_RXL_STA         (0x6)
#define SUART_INTID_BUSY            (0x7) // assert by writing to the LCR while bus busy
                                      // de-assert by reading of the USR 
#define SUART_INTID_CHA_TMO         (0xc)
#define IS_SUART_INTID(INTID)       ((INTID == SUART_INTID_MODEM_STA) || \
                                     (INTID == SUART_INTID_NOPENDING) || \
                                     (INTID == SUART_INTID_THRE) || \
                                     (INTID == SUART_INTID_RXDA) || \
                                     (INTID == SUART_INTID_RXL_STA) || \
                                     (INTID == SUART_INTID_BUSY) || \
                                     (INTID == SUART_INTID_CHA_TMO))

// FCR
#define SUART_RCVR_1CHAR            (0 << 6)
#define SUART_RCVR_QFIFO            (1 << 6)
#define SUART_RCVR_HFIFO            (2 << 6)
#define SUART_RCVR_FFIFOM2          (3 << 6)
#define IS_SUART_RCVR(RCVR)         ((RCVR == SUART_RCVR_1CHAR) || \
                                     (RCVR == SUART_RCVR_QFIFO) || \
                                     (RCVR == SUART_RCVR_HFIFO) || \
                                     (RCVR == SUART_RCVR_FFIFOM2))

#define SUART_TET_EMPTY             (0 << 4)
#define SUART_TET_2CHAR             (1 << 4)
#define SUART_TET_QFIFO             (2 << 4)
#define SUART_TET_HFIFO             (3 << 4)
#define IS_UART_TET(TET)            ((TET == SUART_TET_EMPTY) || \
                                     (TET == SUART_TET_2CHAR) || \
                                     (TET == SUART_TET_QFIFO) || \
                                     (TET == SUART_TET_HFIFO))

#define SUART_XFIFOR                (1 << 2)
#define SUART_RFIFOR                (1 << 1)
#define SUART_FIFOE                 (1 << 0)

// LCR
#define SUART_DLAB                  (1 << 7) // divisor latch access bit
#define SUART_BCB                   (1 << 6) // break control bit
#define SUART_SPB                   (1 << 5) // stick parity bit
#define SUART_EPSB                  (1 << 4) // even parity select bit
#define SUART_PENB                  (1 << 3) // parity enable bit
#define SUART_STOP_1B               (0 << 2) // 1 stop bit
#define SUART_STOPB                 (1 << 2) // 1.5 or 2 stop bit according to DLS
#define SUART_DLS_5B                (0 << 0)
#define SUART_DLS_6B                (1 << 0)
#define SUART_DLS_7B                (2 << 0)
#define SUART_DLS_8B                (3 << 0)

// MCR
#define SUART_AFCM_DIS              (0 << 5)
#define SUART_AFEM_ENB              (1 << 5)

#define SUART_LBM_DIS               (0 << 4)
#define SUART_LBM_ENB               (1 << 4)

#define SUART_OUT2_DIS              (0 << 3)
#define SUART_OUT2_ENB              (1 << 3)

#define SUART_OUT1_DIS              (0 << 2)
#define SUART_OUT1_ENB              (1 << 2)

#define SUART_RTS_DIS               (0 << 1)
#define SUART_RTS_ENB               (1 << 1)

#define SUART_DTR_DIS               (0 << 0)
#define SUART_DTR_ENB               (1 << 0)

// LSR
#define SUART_RFNE                  (0 << 7)
#define SUART_RFE                   (1 << 7) // receiver fifo error bit

#define SUART_TEMT                  (1 << 6) // transmitter empty bit
#define SUART_THRE                  (1 << 5) // transmit holding register empty bit
#define SUART_BIB                   (1 << 4) // break interrupt bit
#define SUART_FEB                   (1 << 3) // framing error bit
#define SUART_PEB                   (1 << 2) // parity error bit
#define SUART_OEB                   (1 << 1) // overrun error bit
#define SUART_DRB                   (1 << 0) // data ready bit

// MSR
#define SUART_DCDB                  (1 << 7) // data carrier detect
#define SUART_RIB                   (1 << 6) // ring indicator
#define SUART_DSRB                  (1 << 5) // data set ready
#define SUART_CTSB                  (1 << 4) // clear to send
#define SUART_DDCDB                 (1 << 3) // delta data carrier detect
#define SUART_TERIB                 (1 << 2) // trailing edge of ring indicator
#define SUART_DDSRB                 (1 << 1) // delta data set ready
#define SUART_DCTSB                 (1 << 0) // delta clear to send

// SCR
#define SUART_SCR_MSK               (0xff)

// TFR
#define SUART_TFR_MSK               (0xff)

/*USR :has no this register*/
/* 
#define SUART_RFFB                  (1 << 4) // receive fifo full
#define SUART_RFNEB                 (1 << 3) // receive fifo not empty
#define SUART_TFEB                  (1 << 2) // transmit fifo empty
#define SUART_TFNFB                 (1 << 1) // transmit fifo not full
#define SUART_BUSYB                 (1 << 0) // uart busy
*/

// HTX
#define SUART_HALT_TX_DIS           (0 << 0)
#define SUART_HALT_TX_ENB           (1 << 0)

// TCR
#define SUART_XFER_FD_MODE          (0 << 3)
#define SUART_XFER_SCHD_MODE        (1 << 3)
#define SUART_XFER_HCHD_MODE        (2 << 3)
#define IS_UART_XFER_MODE(mode)     ((mode == SUART_XFER_FD_MODE) || \
                                     (mode == SUART_XFER_SCHD_MODE) || \
                                     (mode == SUART_XFER_HCHD_MODE))

#define SUART_DE_POL_H              (1 << 2)
#define SUART_DE_POL_L              (0 << 2)

#define SUART_RE_POL_H              (1 << 1)
#define SUART_RE_POL_L              (0 << 1)

#define SUART_RS485_EN              (1 << 0)
#define SUART_RS232_EN              (0 << 0)

// DE_EN
#define SUART_DE_DEASSERT           (0)
#define SUART_DE_ASSERT             (1)

// RE_EN
#define SUART_RE_DEASSERT           (0)
#define SUART_RE_ASSERT             (1)

// DET
#define SUART_DE_DASST_POS          16
#define SUART_DE_DASST_MSK          (0xff)
#define SUART_DE_ASST_POS           0
#define SUART_DE_ASST_MSK           (0xff)

// DLF
#define SUART_DLF_POS               0
#define SUART_DLF_MSK               (0x3f)

// TMO
#define SUART_TMO_CFG_EN            (1 << 15)
#define SUART_TMO_CFG_POS           0
#define SUART_TMO_CFG_MSK           (0x3ff)


/*************************************
 * inno UART Hardware Specs
 ************************************/
#define INNO_UART_TX_FIFO_SIZE  64


/*
 * UART Register set (this is not a Standards Compliant IP)
 * Also each reg is Word aligned, but only 8 bits wide
 */
#define R_ID0	0
#define R_ID1	4
#define R_ID2	8
#define R_ID3	12


/* Bits for UART Status Reg (R/W) */
#define RXIENB  0x04	/* Receive Interrupt Enable */
#define TXIENB  0x80	/* Transmit Interrupt Enable */

#define RXFULL  0x08	/* Receive FIFO full */
#define RXFULL1 0x10	/* Receive FIFO has space for 1 char (tot space=4) */

#define RXFERR  0x01	/* Frame Error: Stop Bit not detected */
#define RXOERR  0x02	/* OverFlow Err: Char recv but RXFULL still set */

/* Uart bit fiddling helpers: lowest level */
#define RBASE(port, reg)      (port->membase + reg)
#define UART_REG_SET(u, r, v) writel((v), RBASE(u, r))
#define UART_REG_GET(u, r)    readl(RBASE(u, r))

#define UART_REG_OR(u, r, v)  UART_REG_SET(u, r, UART_REG_GET(u, r) | (v))
#define UART_REG_CLR(u, r, v) UART_REG_SET(u, r, UART_REG_GET(u, r) & ~(v))

/* Uart bit fiddling helpers: API level */
#define UART_SET_DATA(uart, val)   UART_REG_SET(uart, R_TXDATA, val)
#define UART_GET_DATA(uart)        UART_REG_GET(uart, R_RXDATA)

#define UART_SET_BAUDH(uart, val)  UART_REG_SET(uart, R_BAUDH, val)
#define UART_SET_BAUDL(uart, val)  UART_REG_SET(uart, R_BAUDL, val)

//#define UART_CLR_STATUS(uart, val) UART_REG_CLR(uart, R_II, val)
#define UART_GET_STATUS(uart)      UART_REG_GET(uart, R_II)

#define UART_ALL_IRQ_DISABLE(uart) UART_REG_CLR(uart, R_IE, RXIENB|TXIENB)
#define UART_RX_IRQ_DISABLE(uart)  UART_REG_CLR(uart, R_IE, RXIENB)
#define UART_TX_IRQ_DISABLE(uart)  UART_REG_CLR(uart, R_IE, TXIENB)

//#define UART_ALL_IRQ_ENABLE(uart)  UART_REG_OR(uart, R_IE, RXIENB|TXIENB)
#define UART_ALL_IRQ_ENABLE(uart)  UART_REG_OR(uart, R_IE, RXIENB|TXIENB|SUART_ETBEI_INTEN|SUART_ERBFI_INTEN )

#define UART_RX_IRQ_ENABLE(uart)   UART_REG_OR(uart, R_IE, RXIENB)
#define UART_TX_IRQ_ENABLE(uart)   UART_REG_OR(uart, R_IE, TXIENB)

#define INNO_SERIAL_DEV_NAME	"ttyINNO"
#define SET_BIT(x,y)          ((x) |= (1<<(y)))


struct inno_uart_port {
	struct uart_port port;
	unsigned long baud;
};

extern struct inno_uart_port lx28xx_s_ports[];

#define to_inno_port(uport)  container_of(uport, struct inno_uart_port, port)

#ifdef CONFIG_SERIAL_INNO_CONSOLE
static struct console inno_console;
#endif

#define DRIVER_NAME	"inno-uart"

static struct uart_driver inno_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= INNO_SERIAL_DEV_NAME,
	.major		= 0,
	.minor		= 0,
	.nr		= CONFIG_SERIAL_INNO_NR_PORTS,
#ifdef CONFIG_SERIAL_INNO_CONSOLE
	.cons		= &inno_console,
#endif
};

static void inno_set_rt_mux(void)
{
    PAD7CTL = 22;
    PADMODE22 = 0x207;//RX_TX
}

static void inno_set_vid_pwm(void)
{
    PAD5CTL = 23;
    PADMODE23 = 0x200;//VID_PWM0-7
    //PADMODE23 = 0xfff;//VID_PWM0-7
}



static void inno_get_rt_mux(void)
{
    printk(KERN_EMERG"PADMODE22:0x%x\n",PADMODE22);
    printk(KERN_EMERG"PAD7CTL:0x%x\n",PAD7CTL);
}


static void inno_serial_stop_rx(struct uart_port *port)
{
	UART_RX_IRQ_DISABLE(port);
}

static void inno_serial_stop_tx(struct uart_port *port)
{
	while (!(UART_GET_STATUS(port) & SUART_INTID_THRE))
		cpu_relax();

	UART_TX_IRQ_DISABLE(port);
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int inno_serial_tx_empty(struct uart_port *port)
{
	unsigned int stat;

	stat = UART_GET_STATUS(port);
	if (stat & SUART_INTID_THRE)
		return TIOCSER_TEMT;

	return 0;
}

static void inno_serial_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned char ch;

    //print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, xmit->buf, CIRC_CNT(xmit->head,xmit->tail,UART_XMIT_SIZE));
	if (!uart_circ_empty(xmit)) {
		ch = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		//while (!(((UART_GET_STATUS(port))& 0xf) == SUART_INTID_THRE))
		while (!(UART_REG_GET(port, R_LS)& (SUART_THRE | SUART_TEMT)))
        {
           cpu_relax();
           #if  0   //add by lzl 20180529
           printk(KERN_EMERG"---@lzl@---func:%s;line:%d;IIR:0x%x\n",__func__,__LINE__,UART_REG_GET(port, R_II));
           printk(KERN_EMERG"---@lzl@---func:%s;line:%d;IER:0x%x\n",__func__,__LINE__,UART_REG_GET(port, R_IE));
           printk(KERN_EMERG"---@lzl@---func:%s;line:%d;LCR:0x%x\n",__func__,__LINE__,UART_REG_GET(port, R_LC));
           printk(KERN_EMERG"---@lzl@---func:%s;line:%d;LSR:0x%x\n",__func__,__LINE__,UART_REG_GET(port, R_LS));
           printk(KERN_EMERG"---@lzl@---func:%s;line:%d;CPR:0x%x\n",__func__,__LINE__,UART_REG_GET(port, R_CP));
           printk(KERN_EMERG"---@lzl@---func:%s;line:%d;USR:0x%x\n",__func__,__LINE__,UART_REG_GET(port, R_US));
           #endif
        }    
		UART_SET_DATA(port, ch);
        //printk(KERN_EMERG"---@lzl@---send char:%c;LCR:0x%x\n",ch,UART_REG_GET(port, R_LC));
	}

	/*
	 * If num chars in xmit buffer are too few, ask tty layer for more.
	 * By Hard ISR to schedule processing in software interrupt part
	 */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
    UART_TX_IRQ_ENABLE(port); 	
}

/*
 * port is locked and interrupts are disabled
 * uart_start( ) calls us under the port spinlock irqsave
 */
static void inno_serial_start_tx(struct uart_port *port)
{
	inno_serial_tx_chars(port);
}

static void inno_serial_rx_chars(struct uart_port *port, unsigned int status)
{
	unsigned int ch, flg = 0;
  
	while (((status = UART_GET_STATUS(port))& 0xf ) ==  SUART_INTID_RXDA)
    {
        flg = TTY_NORMAL;
		ch = UART_GET_DATA(port);
		port->icount.rx++;

        #if 0  //add by lzl 20180530
        printk(KERN_EMERG"read ch:%c\n",ch);
        printk(KERN_EMERG"IIR:0x%x\n",UART_REG_GET(port,R_II));
        #else
        //mdelay(10);
        //UART_REG_GET(port,R_II);
        #endif
        
		if (!(uart_handle_sysrq_char(port, ch))) 
        {
            uart_insert_char(port, status, RXOERR, ch, flg);
        }      
			
		spin_unlock(&port->lock);
		tty_flip_buffer_push(&port->state->port);
		spin_lock(&port->lock);
	}
}

static void inno_receiver_line_status(struct uart_port *port)
{
    unsigned int tmp, flg = 0;
    
    tmp = UART_REG_GET(port,R_LS);
    if (tmp & SUART_RFE) //bit 7
    {
        printk(KERN_EMERG"func:%s;line:%d;Receiver FIFO Error:LSR:0x%x;\n",__func__,__LINE__,tmp);
    }
    if (tmp & SUART_BIB)  //bit4
    {
        port->icount.brk++;
        flg = TTY_BREAK;
        printk(KERN_EMERG"func:%s;line:%d;Break Interrupt  Error:LSR:0x%x;\n",__func__,__LINE__,tmp);
    }
    if (tmp & SUART_RFE) //bit3
    {
        port->icount.frame++;
		flg = TTY_FRAME;
       printk(KERN_EMERG"func:%s;line:%d;Framing Error:LSR:0x%x;\n",__func__,__LINE__,tmp);
    }
    if (tmp & SUART_PEB) //bit2
    {
        port->icount.parity++;
        flg = TTY_PARITY;
        printk(KERN_EMERG"func:%s;line:%d;Parity Error:LSR:0x%x;\n",__func__,__LINE__,tmp);
    }
    if (tmp & SUART_PEB) //bit1
    {
        port->icount.overrun++;
        flg = TTY_OVERRUN;
        printk(KERN_EMERG"func:%s;line:%d;Parity Error:LSR:0x%x;\n",__func__,__LINE__,tmp);
    }

}
static irqreturn_t inno_serial_isr(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned int status;

	status = UART_GET_STATUS(port);
    //printk(KERN_EMERG"---@lzl@---func:%s;line:%d;IIR:0x%x\n",__func__,__LINE__,status);
    if ((status & 0xf )== SUART_INTID_RXDA){

		spin_lock(&port->lock);
		inno_serial_rx_chars(port, status);
		spin_unlock(&port->lock);
	}

	if ((status & 0xf ) == SUART_INTID_THRE || (!(UART_REG_GET(port, R_LS)& SUART_THRE))) {

		UART_TX_IRQ_DISABLE(port);

		spin_lock(&port->lock);

		if (!uart_tx_stopped(port))
			inno_serial_tx_chars(port);

		spin_unlock(&port->lock);
	}
    
    if ((status & 0xf ) ==  SUART_INTID_RXL_STA) 
    {
        spin_lock(&port->lock);
        inno_receiver_line_status(port);
        spin_unlock(&port->lock);
    }
	if ((status & 0xf ) == SUART_INTID_CHA_TMO)
    {
        spin_lock(&port->lock);

        UART_REG_CLR(port,R_LC,SUART_DLAB);//set LCR[7] =0
        printk(KERN_EMERG"timeout errors:IIR:0x%x;LSR:0x%x;after read:IIR:0x%x\n",status,UART_REG_GET(port,R_LS),UART_REG_GET(port,R_II));
		//ch = UART_GET_DATA(port);
		//port->icount.rx++;
        spin_unlock(&port->lock);
    }      

	return IRQ_HANDLED;
}

static unsigned int inno_serial_get_mctrl(struct uart_port *port)
{
	/*
	 * Pretend we have a Modem status reg and following bits are
	 *  always set, to satify the serial core state machine
	 *  (DSR) Data Set Ready
	 *  (CTS) Clear To Send
	 *  (CAR) Carrier Detect
	 */
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void inno_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* MCR not present */
}

static void inno_serial_break_ctl(struct uart_port *port, int break_state)
{
	/* inno UART doesn't support sending Break signal */
}

static int inno_serial_startup(struct uart_port *port)
{
    unsigned int vid_pwm ,index;
    static unsigned int first_mux = 0;

    index = port->line ;
    if (index >= 0 && index <= 7)
    {
        if (!first_mux) 
            inno_set_vid_pwm();
        unsigned char * base = NULL;
        base = (volatile unsigned char *)ioremap(0X602f0000,0xffff);
        if (!base) 
        {
            printk(KERN_EMERG"error:func:%s;line:%d\n",__func__,__LINE__);
        }
        vid_pwm = *((volatile unsigned int *)(base + 0x10));
        vid_pwm = SET_BIT(vid_pwm,index);
        vid_pwm = SET_BIT(vid_pwm,(index + 16));
        *((volatile unsigned int *)(base + 0x10)) = vid_pwm;
        iounmap(base);
        first_mux = 1;
    }
    else if (port->line == 8)
    {
        inno_set_rt_mux();
    }
   
    UART_REG_CLR(port,R_LC,SUART_DLAB);//LCR[7] =0
	UART_ALL_IRQ_DISABLE(port);
	if (request_irq(port->irq, inno_serial_isr, 0, "inno uart rx-tx", port)) {
		printk(KERN_EMERG"Unable to attach inno UART intr\n");
		return -EBUSY;
	}
	UART_ALL_IRQ_ENABLE(port);
	return 0;
}

/* This is not really needed */
static void inno_serial_shutdown(struct uart_port *port)
{
	free_irq(port->irq, port);
}

static void
inno_serial_set_termios(struct uart_port *port, struct ktermios *new,
		       struct ktermios *old)
{
	struct inno_uart_port *uart = to_inno_port(port);
	unsigned int baud, uartl, uarth, hw_val,quot,lcr,fcr;
	unsigned long flags;

	baud = uart_get_baud_rate(port, new, old, 0, 921600);
	hw_val = port->uartclk / (uart->baud * 16);
	uartl = hw_val & 0xFF;
	uarth = (hw_val >> 8) & 0xFF;

	spin_lock_irqsave(&port->lock, flags);

    UART_REG_CLR(port,R_LC,SUART_DLAB);//LCR[7] =0
	UART_ALL_IRQ_DISABLE(port);

    quot = uart_get_divisor(port, baud);
    //printk(KERN_EMERG"---@lzl@---the quot is %d;baud:%d\n",quot,baud);
    if (baud > port->uartclk/16)
		quot = DIV_ROUND_CLOSEST(port->uartclk * 8, baud);
	else
		quot = DIV_ROUND_CLOSEST(port->uartclk * 4, baud);
     //printk(KERN_EMERG"---@lzl@---the quot is %d;baud:%d\n",quot,baud);

     UART_REG_OR(port,R_LC,SUART_DLAB);//LCR[7] =1
     UART_SET_BAUDL(port, uartl);
     UART_SET_BAUDH(port, uarth);
     //printk(KERN_EMERG"---@lzl@--- baudh:%d;baudl:%d\n",UART_REG_GET(port, R_BAUDH),UART_REG_GET(port, R_BAUDL));
     UART_REG_SET(port,R_DLF,quot);

     UART_REG_CLR(port,R_LC,SUART_DLAB);//LCR[7] =0

	//UART_RX_IRQ_ENABLE(port);

	new->c_cflag &= ~(CMSPAR|CRTSCTS|CSIZE);
	new->c_cflag |= CS8;

    switch (new->c_cflag & CSIZE) {   //data bits
	case CS5:
		lcr = SUART_DLS_5B;
		break;
	case CS6:
		lcr = SUART_DLS_6B;
		break;
	case CS7:
		lcr = SUART_DLS_7B;
		break;
	default: // CS8
		lcr = SUART_DLS_8B;
		break;
	}
	if (new->c_cflag & CSTOPB)//should add switch case 
		lcr |= SUART_STOP_1B;//1 bit stop
	if (new->c_cflag & PARENB) {
		lcr |= SUART_PENB;  //enable  parity 
		if (!(new->c_cflag & PARODD))
			lcr |= (0 << 4);//odd parity
	}
    
    UART_REG_SET(port,R_LC,lcr);

    /*set fifo control register */
    //fcr = (SUART_RCVR_QFIFO | SUART_TET_QFIFO | SUART_XFIFOR | SUART_RFIFOR | SUART_FIFOE );
    fcr = SUART_FIFOE;
    UART_REG_SET(port,R_FC,fcr);

	if (old)
		tty_termios_copy_hw(new, old);

	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(new))
		tty_termios_encode_baud_rate(new, baud, baud);

	uart_update_timeout(port, new->c_cflag, baud);

    UART_ALL_IRQ_ENABLE(port);//enable rx and tx
    //printk(KERN_EMERG"---@lzl@---func:%s;line:%d;IER:0x%x\n",__func__,__LINE__,UART_REG_GET(port, R_IE));
    //printk(KERN_EMERG"---@lzl@---func:%s;line:%d;IIR:0x%x\n",__func__,__LINE__,UART_REG_GET(port, R_II));
	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *inno_serial_type(struct uart_port *port)
{
	return port->type == PORT_INNO ? DRIVER_NAME : NULL;
}

static void inno_serial_release_port(struct uart_port *port)
{
}

static int inno_serial_request_port(struct uart_port *port)
{
	return 0;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 */
static int
inno_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (port->type != PORT_UNKNOWN && ser->type != PORT_INNO)
		return -EINVAL;

	return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void inno_serial_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_INNO;
}

#ifdef CONFIG_CONSOLE_POLL

static void inno_serial_poll_putchar(struct uart_port *port, unsigned char chr)
{
	while (!(UART_GET_STATUS(port) & SUART_INTID_THRE))
		cpu_relax();

	UART_SET_DATA(port, chr);
}

static int inno_serial_poll_getchar(struct uart_port *port)
{
	unsigned char chr;

	while (!(UART_GET_STATUS(port) & SUART_INTID_CHA_TMO))
		cpu_relax();

    UART_REG_CLR(port,R_LC,SUART_DLAB);//set LCR[7] =0
	chr = UART_GET_DATA(port);
	return chr;
}
#endif

static struct uart_ops inno_serial_pops = {
	.tx_empty	= inno_serial_tx_empty,
	.set_mctrl	= inno_serial_set_mctrl,
	.get_mctrl	= inno_serial_get_mctrl,
	.stop_tx	= inno_serial_stop_tx,
	.start_tx	= inno_serial_start_tx,
	.stop_rx	= inno_serial_stop_rx,
	.break_ctl	= inno_serial_break_ctl,
	.startup	= inno_serial_startup,
	.shutdown	= inno_serial_shutdown,
	.set_termios	= inno_serial_set_termios,
	.type		= inno_serial_type,
	.release_port	= inno_serial_release_port,
	.request_port	= inno_serial_request_port,
	.config_port	= inno_serial_config_port,
	.verify_port	= inno_serial_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char = inno_serial_poll_putchar,
	.poll_get_char = inno_serial_poll_getchar,
#endif
};

#ifdef CONFIG_SERIAL_inno_CONSOLE

static int inno_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= CONFIG_SERIAL_inno_NR_PORTS)
		return -ENODEV;

	/*
	 * The uart port backing the console (e.g. ttyinno1) might not have been
	 * init yet. If so, defer the console setup to after the port.
	 */
	port = &inno_uart_ports[co->index].port;
	if (!port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	/*
	 * Serial core will call port->ops->set_termios( )
	 * which will set the baud reg
	 */
	return uart_set_options(port, co, baud, parity, bits, flow);
}

static void inno_serial_console_putchar(struct uart_port *port, int ch)
{
	while (!(UART_GET_STATUS(port) & SUART_INTID_THRE))
		cpu_relax();

	UART_SET_DATA(port, (unsigned char)ch);
}

/*
 * Interrupts are disabled on entering
 */
static void inno_serial_console_write(struct console *co, const char *s,
				     unsigned int count)
{
	struct uart_port *port = &inno_uart_ports[co->index].port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	uart_console_write(port, s, count, inno_serial_console_putchar);
	spin_unlock_irqrestore(&port->lock, flags);
}

static struct console inno_console = {
	.name	= INNO_SERIAL_DEV_NAME,
	.write	= inno_serial_console_write,
	.device	= uart_console_device,
	.setup	= inno_serial_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &inno_uart_driver
};

static __init void inno_early_serial_write(struct console *con, const char *s,
					  unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, inno_serial_console_putchar);
}

static int __init inno_early_console_setup(struct earlycon_device *dev,
					  const char *opt)
{
	struct uart_port *port = &dev->port;
	unsigned int l, h, hw_val;

	if (!dev->port.membase)
		return -ENODEV;

	hw_val = port->uartclk / (dev->baud * 4) - 1;
	l = hw_val & 0xFF;
	h = (hw_val >> 8) & 0xFF;

	UART_SET_BAUDL(port, l);
	UART_SET_BAUDH(port, h);

	dev->con->write = inno_early_serial_write;
	return 0;
}
EARLYCON_DECLARE(inno_uart, inno_early_console_setup);
OF_EARLYCON_DECLARE(inno_uart, "snps,inno-uart", inno_early_console_setup);

#endif	/* CONFIG_SERIAL_inno_CONSOLE */

static int inno_serial_probe(struct platform_device *pdev)
{
    struct resource *res = pdev->resource;
	int i;

	for (i = 0; i < pdev->num_resources; i++, res++) {
		if (!(res->flags & IORESOURCE_MEM))
			continue;

		for (i = 0; i < CONFIG_SERIAL_INNO_NR_PORTS; i++) {
			if (lx28xx_s_ports[i].port.mapbase != res->start)
				continue;

			lx28xx_s_ports[i].port.dev = &pdev->dev;
            lx28xx_s_ports[i].port.membase =(unsigned char __iomem*)ioremap((unsigned long)lx28xx_s_ports[i].port.mapbase ,lx28xx_s_ports[i].port.mapsize);
            if (!lx28xx_s_ports[i].port.membase) 
            {
                printk(KERN_EMERG"func:%s;line:%d;port:%d ioremap failed !\n",__func__,__LINE__,i);
            }
            printk(KERN_EMERG"inno_uart%d->membase :0x%x\n",i,lx28xx_s_ports[i].port.membase);
			uart_add_one_port(&inno_uart_driver, &lx28xx_s_ports[i].port);
			platform_set_drvdata(pdev, &lx28xx_s_ports[i]);
			break;
		}
	}
	return 0;   
}

static int inno_serial_remove(struct platform_device *pdev)
{
    int i;
    
	for (i = 0; i < CONFIG_SERIAL_INNO_NR_PORTS; i++) {
		uart_remove_one_port(&inno_uart_driver, &lx28xx_s_ports[i].port);
	}
	return 0;
}

static struct platform_driver inno_platform_driver = {
	.probe = inno_serial_probe,
	.remove = inno_serial_remove,
	.driver = {
		.name = DRIVER_NAME,
		//.of_match_table  = inno_uart_dt_ids,
	 },
};

static void __init inno_init_ports(void)
{
	static int first = 1;
	int i;

	if (!first)
	    return;
	first = 0;

	for (i = 0; i < CONFIG_SERIAL_INNO_NR_PORTS; i++) 
    {
		lx28xx_s_ports[i].port.ops = &inno_serial_pops;
	}
}


static int __init inno_serial_init(void)
{
	int ret;
    
    inno_init_ports();
    
	ret = uart_register_driver(&inno_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&inno_platform_driver);
	if (ret)
		uart_unregister_driver(&inno_uart_driver);

	return ret;
}

static void __exit inno_serial_exit(void)
{
	platform_driver_unregister(&inno_platform_driver);
	uart_unregister_driver(&inno_uart_driver);
}

module_init(inno_serial_init);
module_exit(inno_serial_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Inno Fireware Team ");
MODULE_DESCRIPTION("Inno serial driver");

