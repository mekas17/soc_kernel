/*
 * freechip_mmc.c - Xiangjing LX28XX MMC/SD/SDIO driver
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 *	Freechip mmc/sd fifo depth is 32 len,width is 32 bits, so the fifo total size is 32*32/8 =128 bytes.
 * NOTES: when the rx/tx fifo level is reach/less  the fifo_rx/tx_watermark, it will not produce the 
 * MMC Interrupt only reflect in SD_RINTSTS regs (only set the RXDR/TXDR).
 *  SD_FIFOTH Register ---Burst size of multiple transaction; should be programmed same as 
	DW-DMA controller multiple-transaction-size SRC/DEST_MSIZE
 *
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/card.h>

#include <mach/mmc.h>
#include <mach/hardware.h>

#include <linux/dmaengine.h>
#ifdef CONFIG_MMC_DMA
#include <linux/dw_dmac.h>
#endif
#include <mach/clock_t18xx.h>


//for debug convienient
//#define CONFIG_MMC_DMA
//#undef CONFIG_MMC_DMA


//#define DEBUG_MMC
#ifdef DEBUG_MMC
#define debug_mmc(fmt,args...)	printk(fmt ,##args)
#else
#define debug_mmc(fmt,args...)
#endif 

/*
 * Register Definitions
 */

#define	 	SD_CTRL		( 0X00)
#define            SD_PWREN    	( 0X04)
#define            SD_CLKDIV    	( 0X08)
#define            SD_CLKSRC    	( 0X0C)
#define            SD_CLKENA    	( 0X010)
#define            SD_TMOUT    	( 0X014)
#define            SD_CTYPE    	( 0X018)
#define            SD_BLKSIZ    	( 0X01C)
#define            SD_BYTCNT    	( 0X020)
#define            SD_INTMASK    ( 0X024)
#define            SD_CMDARG    	( 0X028)
#define            SD_CMD    	( 0X02C)
#define            SD_RESP0    	( 0X030)
#define            SD_RESP1    	( 0X034)
#define            SD_RESP2    	( 0X038)
#define            SD_RESP3    	( 0X03C)
#define            SD_MINTSTS    ( 0X040)
#define            SD_RINTSTS    	( 0X044)
#define            SD_STAT      	( 0X048)
#define            SD_FIFOTH    	( 0X04C)
#define            SD_CDETECT    ( 0X050)
#define            SD_WRTPRT    	( 0X054)
#define            SD_GPIO    	( 0X058)
#define            SD_TCBCNT    	( 0X05C)
#define            SD_TBBCNT    	( 0X060)
#define            SD_DEBNCE    	( 0X064)
#define            SD_USRID    	( 0X068)
#define            SD_VERID    	( 0X06C)
#define            SD_HCON    	( 0X070)
#define	        SD_FIFO		( 0X100)


/* MMCSD Init clock in Hz in opendrain mode */
#define MMCSD_INIT_CLOCK		400000


/*clk setting definitions*/

#define CMD_CLK_SET	0x80202000
#define CMD_FINISH	0x80000000

/* SD_CTRL definitions*/
#define SD_CTRL_INTEN	(1<< 4)
#define SD_CTRL_DMAEN	(1<< 5)



/*SD_CMD definitions*/

#define SD_CMD_RSPNONE   (0)
#define SD_CMD_RSPEX   (1 << 6)
#define SD_CMD_RSPLONG  (1 << 7)
#define SD_CMD_RSPCRC  (1 << 8)
#define SD_CMD_DATAEX  (1 << 9)
#define SD_CMD_WRITE   (1 << 10)
#define SD_CMD_STREAM  (1 << 11)
#define SD_CMD_AUTO_STOP   	(1 << 12)
#define SD_CMD_WAIT_COMP   	(1 << 13)
#define SD_CMD_STOP_ABORT   (1 << 14)
#define SD_CMD_SEND_INIT   	(1 << 15)
#define SD_CMD_START   	(1 << 31)


/* IRQ bit definitions, SD_INTMASK */
#define SD_INTMASK_RSPERR       BIT(1)	
#define SD_INTMASK_CMDDNE      BIT(2)	
#define SD_INTMASK_DTO        	BIT(3)	
#define SD_INTMASK_TXDR           BIT(4)	
#define SD_INTMASK_RXDR           BIT(5)	
#define SD_INTMASK_RCRC           BIT(6)	
#define SD_INTMASK_DCRC           BIT(7)	
#define SD_INTMASK_RTO          	BIT(8)	
#define SD_INTMASK_DRTO           BIT(9)	
#define SD_INTMASK_HTO          	BIT(10)	
#define SD_INTMASK_FRUN           BIT(11)	
#define SD_INTMASK_HLE         	BIT(12)	
#define SD_INTMASK_SBE         	BIT(13)	
#define SD_INTMASK_ACD         	BIT(14)	
#define SD_INTMASK_EBE         	BIT(15)	


#define SD_INTMASK_SDIO         	BIT(16)	

/* IRQ bit definitions, SD_MINTSTS */
#define SD_MINTSTS_RSPERR       BIT(1)	
#define SD_MINTSTS_CMDDNE      BIT(2)	
#define SD_MINTSTS_DTO        	BIT(3)	
#define SD_MINTSTS_TXDR           BIT(4)	
#define SD_MINTSTS_RXDR           BIT(5)	
#define SD_MINTSTS_RCRC           BIT(6)	
#define SD_MINTSTS_DCRC           BIT(7)	
#define SD_MINTSTS_RTO          	BIT(8)	
#define SD_MINTSTS_DRTO           BIT(9)	
#define SD_MINTSTS_HTO          	BIT(10)	
#define SD_MINTSTS_FRUN           BIT(11)	
#define SD_MINTMASK_SBE         	BIT(13)	
#define SD_MINTSTS_HLE         	BIT(12)	

#define SD_MINTSTS_MASK         (0xffff)

/* IRQ bit definitions, SD_RINTSTS */
#define SD_RINTSTS_RSPERR       BIT(1)	
#define SD_RINTSTS_CMDDNE      BIT(2)	
#define SD_RINTSTS_DTO        	BIT(3)	
#define SD_RINTSTS_TXDR           BIT(4)	
#define SD_RINTSTS_RXDR           BIT(5)	
#define SD_RINTSTS_RCRC           BIT(6)	
#define SD_RINTSTS_DCRC           BIT(7)	
#define SD_RINTSTS_RTO          	BIT(8)	
#define SD_RINTSTS_DRTO           BIT(9)	
#define SD_RINTSTS_HTO          	BIT(10)	
#define SD_RINTSTS_FRUN           BIT(11)	
#define SD_RINTSTS_HLE         	BIT(12)	
#define SD_RINTSTS_SBE         	BIT(13)	
#define SD_RINTSTS_ACD        	BIT(14)	
#define SD_RINTSTS_MASK         (0xffff)

/* SD_STAT definitions  */
#define SD_STAT_FE		BIT(2)//fifo empty
#define SD_STAT_FF		BIT(3)//fifo full
#define SD_STAT_FIFOC_MASK		(0x1fff)
#define FIFO_CNT(host)		((readl(host->base + SD_STAT) >> 17) & SD_STAT_FIFOC_MASK)

#define SD_STAT_BUSY           (1 << 9)


/*
 * One scatterlist dma "segment" is at most MAX_CCNT rw_threshold units,
 * and we handle up to MAX_NR_SG segments.  MMC_BLOCK_BOUNCE kicks in only
 * for drivers with max_hw_segs == 1, making the segments bigger (8KB)
 * than the page or two that's otherwise typical. nr_sg (passed from
 * platform data) == 16 gives at least the same throughput boost, using
 * DMA transfer linkage instead of spending CPU time copying pages.
 */
#define MAX_CCNT	((1 << 16) - 1)

#define MAX_NR_SG	32

#define T18_SDIO_IRQ(deviceId)             \
    (((deviceId) == 0) ? "sdio0" : "sdio1")


#define sd_readl(host, reg)	readl(host->base + reg)
#define sd_writel(host, reg , val)	writel(val, host->base + reg)



static u32 volatile flag_test=0;
static u32 test_buf[4];
unsigned char cmd_stop=0;

static unsigned rw_threshold = 64;
module_param(rw_threshold, uint, S_IRUGO);
MODULE_PARM_DESC(rw_threshold,
		"Read/Write threshold. Default = 64");
#ifdef CONFIG_MMC_DMA
static unsigned __initdata use_dma = 1;
static struct dw_dma_slave mmc_dma;
struct lx28xx_mmc_dma {
	struct dma_chan			*chan;
	struct dma_async_tx_descriptor	*data_desc;
};

#else
static unsigned __initdata use_dma = 0;
#endif
module_param(use_dma, uint, 0);
MODULE_PARM_DESC(use_dma, "Whether to use DMA or not. Default = 1");

struct mmc_freechip_host {
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct mmc_host *mmc;
	struct clk *clk;
	unsigned int mmc_input_clk;
	void __iomem *base;
	struct resource *mem_res;
	int mmc_irq, sdio_irq;
	unsigned char bus_mode;

#define FREECHIP_MMC_DATADIR_NONE	0
#define FREECHIP_MMC_DATADIR_READ	1
#define FREECHIP_MMC_DATADIR_WRITE	2
	unsigned char data_dir;

	/* buffer is used during PIO of one scatterlist segment, and
	 * is updated along with buffer_bytes_left.  bytes_left applies
	 * to all N blocks of the PIO transfer.
	 */
	u8 *buffer;
	u32 buffer_bytes_left;
	u32 bytes_left;
#ifdef CONFIG_MMC_DMA
	struct dw_dma_slave *ds;
	struct lx28xx_mmc_dma dma;
#endif
	bool use_dma;
	bool do_dma;
	u32 sdio_int;


	/* For PIO we walk scatterlists one segment at a time. */
	unsigned int		sg_len;
	struct scatterlist *sg;

	/* for ns in one cycle calculation */
	unsigned ns_in_one_cycle;
};

static void mmc_freechip_cmd_done(struct mmc_freechip_host *host,
				 struct mmc_command *cmd);



/*
 * SD/MMC reset
 */
static inline void sd_controller_reset(struct mmc_freechip_host *host)
{
	sd_writel(host, SD_CTRL, sd_readl(host,SD_CTRL) | 0X1);
	while(sd_readl(host,SD_CTRL)& 0x1)
	{
		udelay(10);
	}
}
static inline void sd_fifo_reset(struct mmc_freechip_host *host)
{
	sd_writel(host, SD_CTRL, sd_readl(host,SD_CTRL) | 0X2);
	while(sd_readl(host,SD_CTRL)& 0x2)
	{
		udelay(10);
	}
}
static inline void sd_dma_reset(struct mmc_freechip_host *host)
{
	sd_writel(host, SD_CTRL, sd_readl(host,SD_CTRL) | 0X4);
	while(sd_readl(host,SD_CTRL)& 0x4)
	{
		udelay(10);
	}
}

static void  init_mmcsd_host(struct mmc_freechip_host *host)
{
	/*SDMMCDELAY reg[0-5]bit is used to select "data in" sample time. assuming clk_out is first rising edge then falling edge. 
	 0-31 --> sample time is at clk_out rising edge + val*0.8ns 
	 32-63 --> sample time is at clk_out falling edge + (val-31)*0.8ns 
	 */
//#define SDMMCDELAY				*(volatile unsigned int *)(LX28XX_VA_APB_SYS+0x2104) 
#define SDMMCDELAY 	        *(volatile unsigned int *)(APB_SYS_BASE+0x001B*4)  //sdmmc_clk_delay_cfg
#define SDIODELAY  	        *(volatile unsigned int *)(APB_SYS_BASE+0x001C*4)
#define SDMMCCFGCOUNT 				*(volatile unsigned int *)(APB_SYS_BASE+0x0012*4)
#define SDIOCFGCOUNT 				*(volatile unsigned int *)(APB_SYS_BASE+0x0013*4)

	u32 temp;
#if 0//def CONFIG_LX28XX_IC
	temp =(SDMMCDELAY & (~(0xffff) )) | (6 |16<< 10) | (0<< 6);
	SDMMCDELAY = temp;	
	printk("SDMMCDELAY =%x\n", SDMMCDELAY);
#else
	temp = SDMMCDELAY ;
	temp = temp & 0xffff0000 ;
	temp =(temp | (18| (1 << 5) |(0 << 15)| (0 << 13)| (0 << 12) | (0 << 6)| (1 << 14)) );
	//temp = 1 << 30 | 64 << 23 ;

//	temp =(SDMMCDELAY & (~(0xffff) ) | (6 | (16 << 10) | (0 << 6)));
	//SDMMCDELAY = temp;	
	//printk("SDMMCDELAY =%x\n", SDMMCDELAY);
	printk("SDIODELAY =%x\n", SDIODELAY);
	printk("SDIOCFGCOUNT =%x\n", SDIOCFGCOUNT);

#endif

	/*sd/mmc reset*/
	sd_controller_reset(host);
	sd_fifo_reset(host);
	sd_dma_reset(host);

	/*set fifo
	if(use_dma){
		sd_writel(host, SD_FIFOTH, 0x10070008);
	}
	else
		sd_writel(host, SD_FIFOTH, 0x10070008);
	*/
	
	sd_writel(host, SD_FIFOTH, 0x300f0010);
	debug_mmc("SD_FIFOTH= %x\n", sd_readl(host,SD_FIFOTH));
	//rw_threshold = 64;

	/*enable global int*/
#ifdef CONFIG_MMC_DMA
	//sd_writel(host, SD_CTRL, (sd_readl(host, SD_CTRL) | SD_CTRL_INTEN|SD_CTRL_DMAEN));
#else
#endif
	sd_writel(host, SD_CTRL, (sd_readl(host, SD_CTRL) | SD_CTRL_INTEN));

	/* mask interrupt disable */
	sd_writel(host, SD_INTMASK, 0x0);
	/*clear int*/
	sd_writel(host, SD_RINTSTS, sd_readl(host, SD_RINTSTS));

	sd_writel(host, SD_PWREN, 0Xf);
	udelay(4);
}


/* PIO only */
static void mmc_freechip_sg_to_buf(struct mmc_freechip_host *host)
{
	host->buffer_bytes_left = sg_dma_len(host->sg);
	host->buffer = sg_virt(host->sg);
	if (host->buffer_bytes_left > host->bytes_left)
		host->buffer_bytes_left = host->bytes_left;
}

static void freechip_fifo_data_trans(struct mmc_freechip_host *host,
					unsigned int n)
{
	u8 *p;
	unsigned int i;
	if (host->buffer_bytes_left == 0) {
		host->sg = sg_next(host->data->sg);
		mmc_freechip_sg_to_buf(host);
	}

	p = host->buffer;
	if (n > host->buffer_bytes_left)
		n = host->buffer_bytes_left;
	host->buffer_bytes_left -= n;
	host->bytes_left -= n;

	/* NOTE:  we never transfer more than rw_threshold bytes
	 * to/from the fifo here; there's no I/O overlap.
	 * This also assumes that access width( i.e. ACCWD) is 4 bytes
	 */
	if (host->data_dir == FREECHIP_MMC_DATADIR_WRITE) {
		for (i = 0; i < (n >> 2); i++) {
			writel(*((u32 *)p), host->base + SD_FIFO);
			p = p + 4;
		}
		if (n & 3) {
			iowrite8_rep(host->base + SD_FIFO, p, (n & 3));
			p = p + (n & 3);
		}
	} else {
		for (i = 0; i < (n >> 2); i++) {
			*((u32 *)p) = readl(host->base + SD_FIFO);
			if(flag_test !=4 )
				test_buf[flag_test++ ]=*((u32 *)p);
			p  = p + 4;
		}
		if (n & 3) {
			ioread8_rep(host->base + SD_FIFO, p, (n & 3));
			p = p + (n & 3);
		}
	}
	host->buffer = p;
}


static void mmc_freechip_start_command(struct mmc_freechip_host *host,
		struct mmc_command *cmd)
{
	u32 cmd_reg = 0;
	u32 im_val;
	static int first=0;

	dev_dbg(mmc_dev(host->mmc), "CMD%d, arg 0x%08x%s\n",
		cmd->opcode, cmd->arg,
		({ char *s;
		switch (mmc_resp_type(cmd)) {
		case MMC_RSP_R1:
			s = ", R1/R5/R6/R7 response";
			break;
		case MMC_RSP_R1B:
			s = ", R1b response";
			break;
		case MMC_RSP_R2:
			s = ", R2 response";
			break;
		case MMC_RSP_R3:
			s = ", R3/R4 response";
			break;
		default:
			s = ", (R? response)";
			break;
		}; s; }));
	host->cmd = cmd;
	
	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_R1B:
		/* There's some spec confusion about when R1B is
		 * allowed, but if the card doesn't issue a BUSY
		 * then it's harmless for us to allow it.
		 */
		cmd_reg |= SD_CMD_RSPNONE;
		/* FALLTHROUGH */
	case MMC_RSP_R1:		/* 48 bits, CRC */
		cmd_reg |= SD_CMD_RSPCRC |SD_CMD_RSPEX ;//
		break;
	case MMC_RSP_R2:		/* 136 bits, CRC */
		cmd_reg |= SD_CMD_RSPCRC |SD_CMD_RSPEX |SD_CMD_RSPLONG;
		break;
	case MMC_RSP_R3:		/* 48 bits, no CRC */
		cmd_reg |= SD_CMD_RSPEX;
		break;
	default:
		cmd_reg |= SD_CMD_RSPNONE;
		break;
	}
	cmd_reg |=SD_CMD_START ;

	
	/* Set command index */
	cmd_reg |= cmd->opcode;
	/* Setting whether the first cmd */
	if (cmd->opcode == MMC_GO_IDLE_STATE){
		if(first == 0)
			cmd_reg |= SD_CMD_SEND_INIT;	
		first =1; 		
		}
	else if (cmd->opcode == MMC_STOP_TRANSMISSION)
		cmd_reg |= SD_CMD_STOP_ABORT  ;	//must disable SD_CMD_WAIT_COMP
		
	/*
	else if ( (cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK)){
		if(host->mmc->card->type != MMC_TYPE_SDIO)
			cmd_reg |= SD_CMD_AUTO_STOP; //SD_CMD_AUTO_STOP ;
		}
	*/

	switch(cmd->opcode){
		case MMC_STOP_TRANSMISSION:
		case MMC_GO_IDLE_STATE:
		case MMC_SEND_STATUS:
			break;

		default:
			cmd_reg |= SD_CMD_WAIT_COMP;
			break;
		}
	/* Setting whether command involves data transfer or not */
	if (cmd->data){
		cmd_reg |= SD_CMD_DATAEX | SD_CMD_WAIT_COMP;// ;
		}

	/* Setting whether stream or block transfer */
	if (cmd->flags & MMC_DATA_QBR) //MMC_DATA_STREAM)
		cmd_reg |= SD_CMD_STREAM;

	/* Setting whether data read or write */
	if (host->data_dir == FREECHIP_MMC_DATADIR_WRITE)
		cmd_reg |= SD_CMD_WRITE;

	//if (host->bus_mode == MMC_BUSMODE_PUSHPULL)
		//cmd_reg |= MMCCMD_PPLEN;
		
	/* Enable DMA transfer triggers */	
	if (host->do_dma)
		sd_writel(host, SD_CTRL, (sd_readl(host, SD_CTRL) | SD_CTRL_DMAEN));
	else
		sd_writel(host, SD_CTRL, (sd_readl(host, SD_CTRL) & (~SD_CTRL_DMAEN)));

	


	/* set Command timeout */
	writel(0xFFFFFFFF, host->base + SD_TMOUT);

	/* Enable interrupt (calculate here, defer until FIFO is stuffed). */
	im_val =  SD_INTMASK_CMDDNE  | SD_INTMASK_RSPERR | SD_INTMASK_RTO |SD_INTMASK_DTO |SD_INTMASK_DRTO |\
	SD_INTMASK_FRUN| SD_INTMASK_HTO| SD_INTMASK_SBE |SD_INTMASK_EBE ;//| SD_INTMASK_DTO |SD_INTMASK_SBE
	if (host->data_dir == FREECHIP_MMC_DATADIR_WRITE) {
		if (!host->do_dma)
			im_val |= SD_INTMASK_TXDR ;
	} else if (host->data_dir == FREECHIP_MMC_DATADIR_READ) {
		if (!host->do_dma)
			im_val |= SD_INTMASK_RXDR  ;
	}

	/*
	 * Before non-DMA WRITE commands the controller needs priming:
	 * FIFO should be populated with 32 bytes i.e. whatever is the FIFO size
	 */
	if (!host->do_dma && (host->data_dir == FREECHIP_MMC_DATADIR_WRITE))
		freechip_fifo_data_trans(host, rw_threshold);
	debug_mmc("host->do_dma=%d\n", host->do_dma);
	debug_mmc("cmd=%d, cmd_reg=0x%x, cmd->arg=0x%x\n", cmd->opcode, cmd_reg, cmd->arg);
	writel(cmd->arg, host->base + SD_CMDARG);
	writel(cmd_reg,  host->base + SD_CMD);
	writel(im_val, host->base + SD_INTMASK);
}

/*----------------------------------------------------------------------*/

/* DMA infrastructure */

static void
mmc_freechip_xfer_done(struct mmc_freechip_host *host, struct mmc_data *data);

#ifdef CONFIG_MMC_DMA

static bool filter(struct dma_chan *chan, void *slave)
{
	struct dw_dma_slave *dws = slave;
	dws->dma_dev = chan->device->dev;
	chan->private = dws;
	
	return true;
}



static void freechip_mmc_stop_dma(struct mmc_freechip_host *host)
{
	struct dma_chan *chan = host->dma.chan;

	chan->device->device_terminate_all(chan);
	//freechip_mmc_dma_cleanup(host);

}


/* This function is called by the DMA driver from tasklet context. */
static void freechip_mmc_dma_complete(void *arg)
{
	struct mmc_freechip_host	*host = arg;
	sd_writel(host , SD_INTMASK, sd_readl(host,SD_INTMASK) | SD_INTMASK_DTO);
	dev_dbg(mmc_dev(host->mmc), "DMA complete\n");
}


static int mmc_freechip_start_dma_transfer(struct mmc_freechip_host *host,
		struct mmc_data *data)
{
	int mask = rw_threshold - 1;
	struct dma_chan 		*chan;
	struct dma_async_tx_descriptor	*desc;
	unsigned int			i;
	enum dma_data_direction 	direction;

	host->sg_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				((data->flags & MMC_DATA_WRITE)
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE));

	/* no individual DMA segment should need a partial FIFO */
	for (i = 0; i < host->sg_len; i++) {
		if (sg_dma_len(data->sg + i) & mask) {
			dma_unmap_sg(mmc_dev(host->mmc),
					data->sg, data->sg_len,
					(data->flags & MMC_DATA_WRITE)
					? DMA_TO_DEVICE
					: DMA_FROM_DEVICE);
			return -1;
		}
	}
	/* If we don't have a channel, we can't do DMA */
	chan = host->dma.chan;
	if (!chan)
		return -ENODEV;

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;


	desc = chan->device->device_prep_slave_sg(chan,
			data->sg, data->sg_len, direction,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		goto unmap_exit;

	host->dma.data_desc = desc;
	desc->callback = freechip_mmc_dma_complete;
	desc->callback_param = host;
	desc->tx_submit(desc);

	/* Go! */
	chan->device->device_issue_pending(chan);
	host->do_dma = 1;

	return 0;
unmap_exit:
	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len, direction);
	return -ENOMEM;

}

#endif
/*----------------------------------------------------------------------*/


static void
mmc_freechip_prepare_data(struct mmc_freechip_host *host, struct mmc_request *req)
{
	int timeout;
	struct mmc_data *data = req->data;

	host->data = data;
	if (data == NULL) {
		host->data_dir = FREECHIP_MMC_DATADIR_NONE;
		writel(0, host->base + SD_BYTCNT);
		writel(0, host->base + SD_BLKSIZ);
		return;
	}

	dev_dbg(mmc_dev(host->mmc), "%s %s, %d blocks of %d bytes\n",
		(data->flags & MMC_DATA_QBR) ? "stream" : "block",
		//(data->flags & MMC_DATA_STREAM) ? "stream" : "block",
		(data->flags & MMC_DATA_WRITE) ? "write" : "read",
		data->blocks, data->blksz);
	dev_dbg(mmc_dev(host->mmc), "  DTO %d cycles + %d ns\n",
		data->timeout_clks, data->timeout_ns);
	timeout = data->timeout_clks +
		(data->timeout_ns / host->ns_in_one_cycle);
	if (timeout > 0xffffff)
		timeout = 0xffffff;

	
	writel(data->blocks * data->blksz, host->base + SD_BYTCNT);
	writel(data->blksz, host->base + SD_BLKSIZ);

	if(data->flags & MMC_DATA_WRITE)
		host->data_dir= FREECHIP_MMC_DATADIR_WRITE;
	else if (data->flags & MMC_DATA_READ)
		host->data_dir= FREECHIP_MMC_DATADIR_READ;
	
	host->buffer = NULL;
	host->bytes_left = data->blocks * data->blksz;

	if (host->mmc->card) {
		if (mmc_card_sdio(host->mmc->card)) {
			if ((data->blksz == 64)) {
				mdelay(5);
			}
		}
	}

	/* For now we try to use DMA whenever we won't need partial FIFO
	 * reads or writes, either for the whole transfer (as tested here)
	 * or for any individual scatterlist segment (tested when we call
	 * start_dma_transfer).
	 *
	 * While we *could* change that, unusual block sizes are rarely
	 * used.  The occasional fallback to PIO should't hurt.
	 */
	 #ifdef CONFIG_MMC_DMA
	if (host->use_dma && (host->bytes_left & (rw_threshold - 1)) == 0
			&& mmc_freechip_start_dma_transfer(host, data) == 0) {
		/* zero this to ensure we take no PIO paths */
		host->bytes_left = 0;
	} else
	#endif
	{
		/* Revert to CPU Copy */
		host->sg_len = data->sg_len;
		host->sg = host->data->sg;
		mmc_freechip_sg_to_buf(host);
	}
}

static void mmc_freechip_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct mmc_freechip_host *host = mmc_priv(mmc);
	unsigned long timeout = jiffies + msecs_to_jiffies(900);
	u32 mmcst1 = 0;
	// This check can be get rid of  
	/* Card may still be sending BUSY after a previous operation,
	 * typically some kind of write.  If so, we can't proceed yet.
	 */
	while (time_before(jiffies, timeout)) {
		mmcst1  = sd_readl(host,  SD_STAT);
		if (!(mmcst1 & SD_STAT_BUSY))
			break;
		cpu_relax();
	}
	if (mmcst1 & SD_STAT_BUSY) {
		printk("still BUSY? bad ... \n");
		req->cmd->error = -ETIMEDOUT;
		mmc_request_done(mmc, req);
		return;
	}

	host->do_dma = 0;
	mmc_freechip_prepare_data(host, req);
	mmc_freechip_start_command(host, req->cmd);
}

static unsigned int calculate_freq_for_card(struct mmc_freechip_host *host,
	unsigned int mmc_req_freq)
{
	unsigned int mmc_pclk = 0, mmc_push_pull_divisor = 0;

	mmc_pclk = host->mmc_input_clk;
	if (mmc_req_freq && mmc_pclk > (2 * mmc_req_freq)){
		if(mmc_req_freq < 1000000){
		mmc_push_pull_divisor = ((unsigned int)mmc_pclk
				/ (2 * mmc_req_freq*4) + 1)  ;
		}else{
		mmc_push_pull_divisor = ((unsigned int)mmc_pclk
				/ (2 * mmc_req_freq) + 1)  ;
			}

		}
	else
		mmc_push_pull_divisor = 1;

	return mmc_push_pull_divisor;
}




static void mmc_freechip_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	unsigned int open_drain_freq = 0, mmc_pclk = 0;
	unsigned int mmc_div = 0;
	struct mmc_freechip_host *host = mmc_priv(mmc);
	//set sd/mmc clk
	unsigned long timeout = jiffies + msecs_to_jiffies(900);
	u32 mmcst1 = 0;

	mmc_pclk = host->mmc_input_clk;
	debug_mmc("bus_width =%d clock %dHz busmode %d powermode %d Vdd %04x\n",
		ios->bus_width, ios->clock, ios->bus_mode, ios->power_mode,
		ios->vdd);
	if (ios->bus_width == MMC_BUS_WIDTH_4) {
		dev_dbg(mmc_dev(host->mmc), "Enabling 4 bit mode\n");
		sd_writel(host, SD_CTYPE , 0x1);
	} else {
		dev_dbg(mmc_dev(host->mmc), "Enabling 1 bit mode\n");
		sd_writel(host, SD_CTYPE , 0x0);
	}


	
	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN) {

		/* Ignoring the init clock value passed for fixing the inter
		 * operability with different cards.
		 */
		open_drain_freq = ((unsigned int)mmc_pclk
				/ (2 * MMCSD_INIT_CLOCK*10))+1 ;
		debug_mmc("open_drain_freq =%x\n", open_drain_freq);

		if (open_drain_freq > 0xFF)
			open_drain_freq = 0xFF;
		//while(readl( host->base + SD_STAT) & SD_STAT_BUSY);
		while (time_before(jiffies, timeout)) {
			mmcst1	= sd_readl(host,  SD_STAT);
			if (!(mmcst1 & SD_STAT_BUSY))
				break;
			cpu_relax();
		}
		if (mmcst1 & SD_STAT_BUSY) {
			dev_err(mmc_dev(host->mmc), "still BUSY? bad ... \n");
			return;
		}

		writel(0x00, host->base + SD_CLKENA);
		writel(CMD_CLK_SET, host->base + SD_CMD);
		while (readl(host->base + SD_CMD) & CMD_FINISH);		
		writel(0x00, host->base + SD_CLKSRC);
		writel(open_drain_freq, host->base + SD_CLKDIV);
		writel(CMD_CLK_SET, host->base + SD_CMD);
		while (readl(host->base + SD_CMD) & CMD_FINISH);
		writel(0xffff, host->base + SD_CLKENA);/*CLOCK ENABLE*/
		writel(CMD_CLK_SET, host->base + SD_CMD);
		while (readl(host->base + SD_CMD) & CMD_FINISH);
		udelay(10);

		/* Convert ns to clock cycles */
		host->ns_in_one_cycle = (1000000) / (MMCSD_INIT_CLOCK/1000);
	}else {
	//sd_writel(host, SD_CTRL , sd_readl(host, SD_CTRL) & (~(1 << 24)));
	mmc_div = calculate_freq_for_card(host, ios->clock);

	if (mmc_div > 0xFF)
		mmc_div = 0xFF;
	printk("mmc_div =%x\n", mmc_div);
	//while(readl( host->base + SD_STAT) & SD_STAT_BUSY);
	while (time_before(jiffies, timeout)) {
		mmcst1	= sd_readl(host,  SD_STAT);
		if (!(mmcst1 & SD_STAT_BUSY))
			break;
		cpu_relax();
	}
	if (mmcst1 & SD_STAT_BUSY) {
		dev_err(mmc_dev(host->mmc), "still BUSY? bad ... \n");
		return;
	}
	writel(0x00, host->base + SD_CLKENA);
	writel(CMD_CLK_SET, host->base + SD_CMD);
	while (readl(host->base + SD_CMD) & CMD_FINISH);		
	writel(0x00, host->base + SD_CLKSRC);
	writel(mmc_div, host->base + SD_CLKDIV);
	writel(CMD_CLK_SET, host->base + SD_CMD);
	while (readl(host->base + SD_CMD) & CMD_FINISH);
	writel(0xffff, host->base + SD_CLKENA);/*CLOCK ENABLE*/
	writel(CMD_CLK_SET, host->base + SD_CMD);
	while (readl(host->base + SD_CMD) & CMD_FINISH);
	udelay(10);

	}
	

}

static void
mmc_freechip_xfer_done(struct mmc_freechip_host *host, struct mmc_data *data)
{

	u32 status;
	

	host->data_dir = FREECHIP_MMC_DATADIR_NONE;

	if (host->do_dma) {
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			     (data->flags & MMC_DATA_WRITE)
			     ? DMA_TO_DEVICE
			     : DMA_FROM_DEVICE);
		host->do_dma = false;
	}

	host->data = NULL;
	host->data_dir = FREECHIP_MMC_DATADIR_NONE;

	if (!data->stop || (host->cmd && host->cmd->error)) {
		mmc_request_done(host->mmc, data->mrq);
		writel(0, host->base + SD_INTMASK);
	} else{
			mmc_freechip_start_command(host, data->stop);
		}
}

static void mmc_freechip_cmd_done(struct mmc_freechip_host *host,
				 struct mmc_command *cmd)
{
	host->cmd = NULL;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = readl(host->base + SD_RESP0);
			cmd->resp[2] = readl(host->base + SD_RESP1);
			cmd->resp[1] = readl(host->base + SD_RESP2);
			cmd->resp[0] = readl(host->base + SD_RESP3);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = readl(host->base + SD_RESP0);
		}
	}

	if (host->data == NULL || cmd->error) {
		if (cmd->error == -ETIMEDOUT)
			cmd->mrq->cmd->retries = 0;
		mmc_request_done(host->mmc, cmd->mrq);
		writel(0, host->base + SD_INTMASK);
	}
}

static void
freechip_abort_data(struct mmc_freechip_host *host, struct mmc_data *data)
{
	/*sd/mmc reset*/
	sd_controller_reset(host);
	sd_fifo_reset(host);
	sd_dma_reset(host);
	#ifdef CONFIG_MMC_DMA
	if(host->do_dma)
		freechip_mmc_stop_dma(host);
	#endif

}

static ssize_t mmc_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mmc_freechip_host * host;
	struct mmc_host * mmc;
	u32 status;
	mmc = container_of(dev, struct mmc_host, class_dev);
	host = mmc_priv(mmc);
	status = sd_readl(host, SD_STAT);
	return sprintf(buf, "0x%x\n", status);
	

}


static ssize_t mmc_status_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_freechip_host * host;
	struct mmc_host * mmc;
	u32 status=0;
	mmc = container_of(dev, struct mmc_host, class_dev);
	host = mmc_priv(mmc);
	status = simple_strtoul(buf,NULL, 16);

	printk("status =0x%x\n", status);
	
	return 0;
	


}



static irqreturn_t mmc_freechip_sdio_irq(int irq, void *dev_id)
{
#if 0
	struct mmc_freechip_host *host = (struct mmc_freechip_host *)dev_id;
	unsigned int status;
	status = readl(host->base + FREECHIP_SDIOIST);
	if (status & SDIOIST_IOINT) {
		dev_dbg(mmc_dev(host->mmc),
				"SDIO interrupt status %x\n", status);
		writel(status | SDIOIST_IOINT,
				host->base + FREECHIP_SDIOIST);
		mmc_signal_sdio_irq(host->mmc);
	}
#endif
	return IRQ_HANDLED;
}
/*Note: must note SD_MINTSTS and SD_RINTSTS it's very important for auto-stop cmd */

static DEVICE_ATTR(mmc_status,S_IWUSR | S_IRUGO, mmc_status_show, mmc_status_store);

static irqreturn_t mmc_freechip_irq(int irq, void *dev_id)
{
	struct mmc_freechip_host *host = (struct mmc_freechip_host *)dev_id;
	unsigned int status, qstatus, raw_st;
	int end_command = 0;
	int end_transfer = 0;
	int cnt=0;
	struct mmc_data *data = host->data;

	if (host->cmd == NULL && host->data == NULL) {
		status = readl(host->base + SD_RINTSTS);
		dev_dbg(mmc_dev(host->mmc),
			"Spurious interrupt 0x%04x\n", status);
		/* Disable the interrupt  */
		writel(0, host->base + SD_INTMASK);
		return IRQ_NONE;
	}
	/*
	raw_st =readl(host->base + SD_RINTSTS);
	if(raw_st & SD_RINTSTS_DCRC)
		printk("CRC Error !!!!!!!!\n");
	*/

	status = readl(host->base + SD_MINTSTS);
	//debug_mmc("irq status=0x%x\n", status);
	writel(status, host->base + SD_RINTSTS);
	
	qstatus = status;

	if (qstatus & SD_RINTSTS_MASK) {
		if (qstatus & SD_RINTSTS_DRTO) {
			/* Read data timeout */
			data->error = -ETIMEDOUT;
			end_transfer = 1;

			dev_err(mmc_dev(host->mmc),
					"read data timeout, status %x\n",
					qstatus);

			freechip_abort_data(host, data);
			goto end_data;
		}

		if (qstatus & (SD_RINTSTS_DCRC)) {
			/* Data CRC error */
			data->error = -EILSEQ;
			end_transfer = 1;

			dev_err(mmc_dev(host->mmc), "data %s %s error\n",
				(qstatus & SD_RINTSTS_DCRC) ? "write" :
				"read",	(data->error == -ETIMEDOUT) ?
				"timeout" : "CRC");

			freechip_abort_data(host, data);
			goto end_data;
		} 

		if (qstatus & (SD_RINTSTS_RTO | SD_RINTSTS_DRTO)) {
			/* Command timeout */
			if (host->cmd) {
				dev_dbg(mmc_dev(host->mmc),
						"CMD%d timeout, status %x\n",
						host->cmd->opcode, qstatus);
				host->cmd->error = -ETIMEDOUT;
				if (data) {
					end_transfer = 1;
					freechip_abort_data(host, data);
				} else
					end_command = 1;
			}
			goto end_cmd;
		}

		if (qstatus & SD_RINTSTS_RCRC) {
			/* Respond CRC error */
			dev_err(mmc_dev(host->mmc), "Respond CRC error\n");
			if (host->cmd) {
				host->cmd->error = -EILSEQ;
				end_command = 1;
			}
			goto end_cmd;
		}
	}


	/* handle FIFO first when using PIO for data.
	 * bytes_left will decrease to zero as I/O progress and status will
	 * read zero over iteration because this controller status
	 * register(MMCST0) reports any status only once and it is cleared
	 * by read. So, it is not unbouned loop even in the case of
	 * non-dma.
	 */
	 #if 1
	while (host->bytes_left && (host->do_dma == 0) && (status & (SD_INTMASK_TXDR | SD_INTMASK_RXDR |SD_RINTSTS_FRUN))) { //|| host->cmd->opcode ==18 
		//debug_mmc("fifo count =%x\n", FIFO_CNT(host));
		freechip_fifo_data_trans(host,  rw_threshold); //<rw_threshold?cnt:rw_threshold
		//debug_mmc("after read fifo count =%x\n", FIFO_CNT(host));
		udelay(4); //must use udelay, or the sd card controller clk will stopped. Why????
		status = readl(host->base + SD_MINTSTS) & 0xfffe;
		if(!status){
			break;
			}
		writel(status , (host->base + SD_RINTSTS));
		//debug_mmc("irq tx rx: status =%x\n", status);
		qstatus |= status;
		
	}
#else
	while (host->bytes_left && (host->do_dma == 0) && (status & (SD_INTMASK_TXDR | SD_INTMASK_RXDR))) { //|| host->cmd->opcode ==18 
		//debug_mmc("fifo count =%x\n", FIFO_CNT(host));
		cnt= rw_threshold;//(status&SD_INTMASK_RXDR)?(FIFO_CNT(host)<<2):((32-FIFO_CNT(host))<<2); //32 -> fifo depth
		freechip_fifo_data_trans(host,  cnt); //<rw_threshold?cnt:rw_threshold
		//debug_mmc("after read fifo count =%x\n", FIFO_CNT(host));
		//udelay(4); //must use udelay, or the sd card controller clk will stopped. Why????
		status = readl(host->base + SD_MINTSTS) & 0xfffe;
		//printk("rd st=%x\n", status);
		writel(status , (host->base + SD_RINTSTS));
		//debug_mmc("irq tx rx: status =%x\n", status);
		qstatus |= status;
		/*
		if(!status | (status & SD_RINTSTS_DTO)){
			//mask tx/rx interrupt
			raw_st =  readl(host->base + SD_INTMASK);
			raw_st &= ~(SD_INTMASK_TXDR | SD_INTMASK_RXDR);
			writel(raw_st, (host->base + SD_INTMASK));
			break;
			}
		*/
	}
	if (host->bytes_left < rw_threshold && host->bytes_left >0)
		{
		freechip_fifo_data_trans(host,  host->bytes_left); //<rw_threshold?cnt:rw_threshold
		}

	

#endif
	

	if (qstatus & (SD_RINTSTS_CMDDNE )) {
		/* End of command phase */
		end_command = (int) host->cmd;
	}
	if ((qstatus & SD_RINTSTS_DTO)) { // || (FIFO_CNT(host) )
		 /* All blocks sent/received, and CRC checks passed */
		 if (data != NULL) {
			 if ((host->do_dma == 0) && (host->bytes_left > 0)) {
				 /* if datasize < rw_threshold
				  * no RX ints are generated
				  */
				  while(host->bytes_left){
				 freechip_fifo_data_trans(host, host->bytes_left);
				 }
			 }
			 end_transfer = 1;
			 data->bytes_xfered = data->blocks * data->blksz;
		 }else {
				dev_err(mmc_dev(host->mmc),
						"DATDNE with no host->data\n");
			}
	}

end_cmd:
	if (end_command)
		mmc_freechip_cmd_done(host, host->cmd);
end_data:
	if (end_transfer)
		mmc_freechip_xfer_done(host, data);

	return IRQ_HANDLED;
}

static int mmc_freechip_get_cd(struct mmc_host *mmc)
{
	struct mmc_freechip_host *host;
	int present;
	//FIXME use the gpio method, the internal detect is not supported.
	host = mmc_priv(mmc);
	present = sd_readl(host, SD_CDETECT);
	printk("sd_readl(host, SD_CDETECT) =%x\n", sd_readl(host, SD_CDETECT));
	return 1;
}

static int mmc_freechip_get_ro(struct mmc_host *mmc)
{
/*
	struct mmc_freechip_host *host;
	int ro;
	//FIXME use the gpio method, the internal wp is not supported.
	
	host = mmc_priv(mmc);
	ro = sd_readl(host, SD_WRTPRT);
	
	return !!(ro);
	*/
	return 0;
}

static void mmc_freechip_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
#if 0
	struct mmc_freechip_host *host = mmc_priv(mmc);
	if (enable) {
		if (!((readl(host->base + SD_INTMASK))
			    & SDIOST0_DAT1_HI)) {
			writel(SDIOIST_IOINT,
					host->base + FREECHIP_SDIOIST);
			mmc_signal_sdio_irq(host->mmc);
		} else {
			host->sdio_int = 1;
			writel(readl(host->base + FREECHIP_SDIOIEN) |
				SDIOIEN_IOINTEN, host->base + FREECHIP_SDIOIEN);
		}
	} else {
		host->sdio_int = 0;
		writel(readl(host->base + FREECHIP_SDIOIEN) & ~SDIOIEN_IOINTEN,
				host->base + FREECHIP_SDIOIEN);
	}

#endif
}
static struct mmc_host_ops mmc_freechip_ops = {
	.request	= mmc_freechip_request,
	.set_ios	= mmc_freechip_set_ios,
	.get_cd		= mmc_freechip_get_cd,
	.get_ro		= mmc_freechip_get_ro,
	.enable_sdio_irq	= mmc_freechip_enable_sdio_irq,
};

/*----------------------------------------------------------------------*/


static  int __init freechip_mmcsd_probe(struct platform_device *pdev)
{
	struct lx28xx_mmc_config *pdata = pdev->dev.platform_data;
	struct mmc_freechip_host *host = NULL;
	struct mmc_host *mmc = NULL;
	struct resource *r, *mem = NULL;
	int ret = 0;
	size_t mem_size;

	/* REVISIT:  when we're fully converted, fail if pdata is NULL */

	ret = -ENODEV;
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		goto out;

	ret = -EBUSY;
	mem_size = resource_size(r);
	mem = request_mem_region(r->start, mem_size, pdev->name);
	if (!mem)
		goto out;

	ret = -ENOMEM;
	mmc = mmc_alloc_host(sizeof(struct mmc_freechip_host), &pdev->dev);
	if (!mmc)
		goto out;

	host = mmc_priv(mmc);
	host->mmc = mmc;	/* Important */

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r)
		goto out;
	host->mmc_irq = r->start;
#if 0
	r = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!r)
		goto out;
	host->sdio_irq = r->start;
#endif


	host->mem_res = mem;
	host->base = (void *)mem->start;
	debug_mmc("host->base =%p\n", host->base);
	if (!host->base)
		goto out;

	ret = -ENXIO;
	debug_mmc("pdev->dev name =%s\n", dev_name(&pdev->dev));
	host->clk = clk_get(&pdev->dev, "MMCSDCLK");
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		goto out;
	}
	clk_enable(host->clk);
	host->mmc_input_clk = 80*1000; //t18xx_get_clks(SDMMC_CLK);//clk_get_rate(host->clk);
	debug_mmc("mmc_input_clk  =%dHz\n", host->mmc_input_clk);

	init_mmcsd_host(host);

	host->use_dma = use_dma;
#ifdef CONFIG_MMC_DMA
	if (host->use_dma ){
	dma_cap_mask_t mask;

	host->ds = &mmc_dma;
	host->ds->reg_width = DW_DMA_SLAVE_WIDTH_32BIT;
	host->ds->tx_reg = host->ds->rx_reg =LX28XX_SDMMC0_PA + SD_FIFO;
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	//FIXME dma channel linked item num should be changed
	host->dma.chan = dma_request_channel(mask, filter, host->ds);
	host->dma.chan->private = host->ds ;
	if (!host->dma.chan){
		dev_notice(&pdev->dev, "DMA not available, using PIO\n");
		host->use_dma = 0;
		}

	}
#endif
	/* REVISIT:  someday, support IRQ-driven card detection.  */
	//mmc->caps |= MMC_CAP_NEEDS_POLL;

	if (!pdata || pdata->wires == 4 || pdata->wires == 0)
		mmc->caps |= MMC_CAP_4_BIT_DATA;


	mmc->ops = &mmc_freechip_ops;
	mmc->f_min = 312500;
	if (pdata && pdata->max_freq)
		mmc->f_max = pdata->max_freq;
	
	mmc->f_max =25000000;
	if (pdata && pdata->caps)
		mmc->caps |= pdata->caps;
	mmc->ocr_avail =MMC_VDD_30_31| MMC_VDD_31_32 |MMC_VDD_32_33 | MMC_VDD_33_34;

#if 1 //def CONFIG_MMC_BLOCK_BOUNCE 
	mmc->max_segs= 1;//MAX_NR_SG;
#else
	mmc->max_segs	= MAX_NR_SG;
#endif

	/* DMA limit per hw segment (one or two MBytes) */
	mmc->max_seg_size	=4096 ;//MAX_CCNT * rw_threshold; //

	/* MMC/SD controller limits for multiblock requests */
	mmc->max_blk_size	= 512; //(1<< 16) -1;  /* BLEN is 16 bits */
	mmc->max_blk_count	= 65535; /* NCNT is 32 bits */
	mmc->max_req_size	= mmc->max_blk_size * mmc->max_blk_count;

	dev_dbg(mmc_dev(host->mmc), "max_hw_segs=%d\n", mmc->max_segs);
	dev_dbg(mmc_dev(host->mmc), "max_blk_size=%d\n", mmc->max_blk_size);
	dev_dbg(mmc_dev(host->mmc), "max_req_size=%d\n", mmc->max_req_size);
	dev_dbg(mmc_dev(host->mmc), "max_seg_size=%d\n", mmc->max_seg_size);

	platform_set_drvdata(pdev, host);

	ret = mmc_add_host(mmc);
	if (ret < 0)
		goto out;

	

	ret = request_irq(host->mmc_irq, mmc_freechip_irq, 0,
		mmc_hostname(mmc), host);
	if (ret)
		goto out;

	if (host->sdio_irq > 0) {
		ret = request_irq(host->sdio_irq,
				mmc_freechip_sdio_irq, 0,
				T18_SDIO_IRQ(pdev->id), host);
		if (ret == 0) {
			mmc->caps |= MMC_CAP_SDIO_IRQ;
			host->sdio_int = 0;
		} else
			goto out;
	}

	rename_region(mem, mmc_hostname(mmc));
	ret =device_create_file(&mmc->class_dev, &dev_attr_mmc_status);
	if(ret < 0)
		goto out;

	dev_info(mmc_dev(host->mmc), "Using %s, %d-bit mode\n",
		host->use_dma ? "DMA" : "PIO",
		(mmc->caps & MMC_CAP_4_BIT_DATA) ? 4 : 1);

	return 0;

out:
	if (host) {
#ifdef CONFIG_MMC_DMA
		dma_release_channel(host->dma.chan);
#endif
		if (host->clk) {
			clk_disable(host->clk);
			clk_put(host->clk);
		}

		if (host->base)
			iounmap(host->base);
	}

	if (mmc)
		mmc_free_host(mmc);

	if (mem)
		release_resource(mem);

	dev_dbg(&pdev->dev, "probe err %d\n", ret);

	return ret;
}

static int __exit freechip_mmcsd_remove(struct platform_device *pdev)
{
	struct mmc_freechip_host *host = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	if (host) {

		//writel((readl(host->base + FREECHIP_MMCCLK) & ~MMCCLK_CLKEN),
				//host->base + FREECHIP_MMCCLK);

		mmc_remove_host(host->mmc);

		free_irq(host->mmc_irq, host);

		if (host->mmc->caps & MMC_CAP_SDIO_IRQ)
			free_irq(host->sdio_irq, host);
#ifdef CONFIG_MMC_DMA

		if (host->use_dma)
			dma_release_channel(host->dma.chan);
#endif
		clk_disable(host->clk);
		clk_put(host->clk);

		release_resource(host->mem_res);

		mmc_free_host(host->mmc);
	}

	return 0;
}

#ifdef CONFIG_PM
static int freechip_mmcsd_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct mmc_freechip_host *host = platform_get_drvdata(pdev);

	return 0;
}

static int freechip_mmcsd_resume(struct platform_device *pdev)
{
	struct mmc_freechip_host *host = platform_get_drvdata(pdev);

	return 0;
}
#else
#define freechip_mmcsd_suspend	NULL
#define freechip_mmcsd_resume	NULL
#endif

static struct platform_driver freechip_mmcsd_driver = {
	.driver		= {
		.name	= "mmc",
		.owner	= THIS_MODULE,
	},
	.probe 		= freechip_mmcsd_probe,
	.remove		= __exit_p(freechip_mmcsd_remove),
	.suspend	= freechip_mmcsd_suspend,
	.resume		= freechip_mmcsd_resume,
};

static int __init freechip_mmcsd_init(void)
{
	return platform_driver_register(&freechip_mmcsd_driver);
}
late_initcall(freechip_mmcsd_init);

static void __exit freechip_mmcsd_exit(void)
{
	platform_driver_unregister(&freechip_mmcsd_driver);
}
module_exit(freechip_mmcsd_exit);

MODULE_AUTHOR("jjdeng @Xiangjing Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MMC/SD driver for LX28XX MMC controller");


