/*
 * drivers/mtd/nand/freechip_nand.c 
 *
 * Nand controller driver for T19XX processors
 *
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
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
//#include <linux/mtd/nand.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>

#include "../mtdcore.h"

#include <mach/nand.h>
#include <mach/hardware.h>

#include <asm/mach-types.h>

static inline int mtd_has_partitions(void) { return 1; }
static inline int mtd_has_cmdlinepart(void) { return 1; }


/*
 * This is a device driver for the NAND flash controller found on the
 * various DaVinci family chips.  It handles up to four SoC chipselects,
 * and some flavors of secondary chipselect (e.g. based on A12) as used
 * with multichip packages.
 *
 * The 1-bit ECC hardware is supported, as well as the newer 4-bit ECC
 * available on chips like the DM355 and OMAP-L137 and needed with the
 * more error-prone MLC NAND chips.
 *
 * This driver assumes EM_WAIT connects all the NAND devices' RDY/nBUSY
 * outputs in a "wire-AND" configuration, with no per-chip signals.
 */
struct freechip_nand_info {
	//struct mtd_info		mtd;
	struct nand_chip	chip;
	//struct nand_ecclayout	ecclayout;

	struct device		*dev;
	struct clk		*clk;
	bool			partitioned;

	bool			is_readmode;

	void __iomem		*base;

	uint32_t		ioaddr;
	uint32_t		current_cs;

	uint32_t		mask_chipsel;

	uint32_t		core_chipsel;
};

static DEFINE_SPINLOCK(freechip_nand_lock);
static bool ecc4_busy;

//#define to_freechip_nand(m) container_of(m, struct freechip_nand_info, mtd)
#define to_freechip_nand(n) container_of(n, struct freechip_nand_info, chip)


static inline unsigned int freechip_nand_readl(struct freechip_nand_info *info,
		int offset)
{
	return __raw_readl(info->base + offset);
}

static inline void freechip_nand_writel(struct freechip_nand_info *info,
		int offset, unsigned long value)
{
	__raw_writel(value, info->base + offset);
}

/*----------------------------------------------------------------------*/

static void nand_freechip_select_chip(struct mtd_info *mtd, int chip)
{
	//struct freechip_nand_info	*info = to_freechip_nand(mtd);

}


/*
 * Access to hardware control lines:  ALE, CLE, secondary chipselect.
 */

static void nand_freechip_hwcontrol(struct mtd_info *mtd, int cmd,
				   unsigned int ctrl)
{
	//struct freechip_nand_info	*info = to_freechip_nand(mtd);
	//uint32_t			addr = info->current_cs;
	//struct nand_chip		*nand = mtd->priv;
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct freechip_nand_info *info = to_freechip_nand(nand);

	/* Did the control lines change? */
	if (ctrl & NAND_CTRL_CHANGE) {
		if (ctrl & NAND_CLE)
			nand->IO_ADDR_W = (void __iomem *)(info->base + NAND_CLE_OFFSET);
		else if (ctrl & NAND_ALE)
			nand->IO_ADDR_W = (void __iomem *)(info->base + NAND_ALE_OFFSET);
		else
			nand->IO_ADDR_W = (void __iomem *)(info->base + NAND_RW_OFFSET);
		if (ctrl & NAND_NCE)
			nand_freechip_select_chip(mtd, *(int *)nand->priv);
		else
			nand_freechip_select_chip(mtd, -1);
	}

	if (cmd != NAND_CMD_NONE) {
		//printk("%x \n", (u8)cmd);
		//udelay(20);
		iowrite8(cmd, nand->IO_ADDR_W);
	}
}


/*----------------------------------------------------------------------*/


#if 0


/*----------------------------------------------------------------------*/

/*
 * 4-bit hardware ECC ... context maintained over entire AEMIF
 *
 * This is a syndrome engine, but we avoid NAND_ECC_HW_SYNDROME
 * since that forces use of a problematic "infix OOB" layout.
 * Among other things, it trashes manufacturer bad block markers.
 * Also, and specific to this hardware, it ECC-protects the "prepad"
 * in the OOB ... while having ECC protection for parts of OOB would
 * seem useful, the current MTD stack sometimes wants to update the
 * OOB without recomputing ECC.
 */

static void nand_freechip_hwctl_4bit(struct mtd_info *mtd, int mode)
{
	struct freechip_nand_info *info = to_freechip_nand(mtd);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&freechip_nand_lock, flags);

	/* Start 4-bit ECC calculation for read/write */
	val = freechip_nand_readl(info, NANDFCR_OFFSET);
	val &= ~(0x03 << 4);
	val |= (info->core_chipsel << 4) | BIT(12);
	freechip_nand_writel(info, NANDFCR_OFFSET, val);

	info->is_readmode = (mode == NAND_ECC_READ);

	spin_unlock_irqrestore(&freechip_nand_lock, flags);
}

/* Read raw ECC code after writing to NAND. */
static void
nand_freechip_readecc_4bit(struct freechip_nand_info *info, u32 code[4])
{
	const u32 mask = 0x03ff03ff;

	code[0] = freechip_nand_readl(info, NAND_4BIT_ECC1_OFFSET) & mask;
	code[1] = freechip_nand_readl(info, NAND_4BIT_ECC2_OFFSET) & mask;
	code[2] = freechip_nand_readl(info, NAND_4BIT_ECC3_OFFSET) & mask;
	code[3] = freechip_nand_readl(info, NAND_4BIT_ECC4_OFFSET) & mask;
}

/* Terminate read ECC; or return ECC (as bytes) of data written to NAND. */
static int nand_freechip_calculate_4bit(struct mtd_info *mtd,
		const u_char *dat, u_char *ecc_code)
{
	struct freechip_nand_info *info = to_freechip_nand(mtd);
	u32 raw_ecc[4], *p;
	unsigned i;

	/* After a read, terminate ECC calculation by a dummy read
	 * of some 4-bit ECC register.  ECC covers everything that
	 * was read; correct() just uses the hardware state, so
	 * ecc_code is not needed.
	 */
	if (info->is_readmode) {
		freechip_nand_readl(info, NAND_4BIT_ECC1_OFFSET);
		return 0;
	}

	/* Pack eight raw 10-bit ecc values into ten bytes, making
	 * two passes which each convert four values (in upper and
	 * lower halves of two 32-bit words) into five bytes.  The
	 * ROM boot loader uses this same packing scheme.
	 */
	nand_freechip_readecc_4bit(info, raw_ecc);
	for (i = 0, p = raw_ecc; i < 2; i++, p += 2) {
		*ecc_code++ =   p[0]        & 0xff;
		*ecc_code++ = ((p[0] >>  8) & 0x03) | ((p[0] >> 14) & 0xfc);
		*ecc_code++ = ((p[0] >> 22) & 0x0f) | ((p[1] <<  4) & 0xf0);
		*ecc_code++ = ((p[1] >>  4) & 0x3f) | ((p[1] >> 10) & 0xc0);
		*ecc_code++ =  (p[1] >> 18) & 0xff;
	}

	return 0;
}

/* Correct up to 4 bits in data we just read, using state left in the
 * hardware plus the ecc_code computed when it was first written.
 */
static int nand_freechip_correct_4bit(struct mtd_info *mtd,
		u_char *data, u_char *ecc_code, u_char *null)
{
	int i;
	struct freechip_nand_info *info = to_freechip_nand(mtd);
	unsigned short ecc10[8];
	unsigned short *ecc16;
	u32 syndrome[4];
	unsigned num_errors, corrected;

	/* All bytes 0xff?  It's an erased page; ignore its ECC. */
	for (i = 0; i < 10; i++) {
		if (ecc_code[i] != 0xff)
			goto compare;
	}
	return 0;

compare:
	/* Unpack ten bytes into eight 10 bit values.  We know we're
	 * little-endian, and use type punning for less shifting/masking.
	 */
	if (WARN_ON(0x01 & (unsigned) ecc_code))
		return -EINVAL;
	ecc16 = (unsigned short *)ecc_code;

	ecc10[0] =  (ecc16[0] >>  0) & 0x3ff;
	ecc10[1] = ((ecc16[0] >> 10) & 0x3f) | ((ecc16[1] << 6) & 0x3c0);
	ecc10[2] =  (ecc16[1] >>  4) & 0x3ff;
	ecc10[3] = ((ecc16[1] >> 14) & 0x3)  | ((ecc16[2] << 2) & 0x3fc);
	ecc10[4] =  (ecc16[2] >>  8)         | ((ecc16[3] << 8) & 0x300);
	ecc10[5] =  (ecc16[3] >>  2) & 0x3ff;
	ecc10[6] = ((ecc16[3] >> 12) & 0xf)  | ((ecc16[4] << 4) & 0x3f0);
	ecc10[7] =  (ecc16[4] >>  6) & 0x3ff;

	/* Tell ECC controller about the expected ECC codes. */
	for (i = 7; i >= 0; i--)
		freechip_nand_writel(info, NAND_4BIT_ECC_LOAD_OFFSET, ecc10[i]);

	/* Allow time for syndrome calculation ... then read it.
	 * A syndrome of all zeroes 0 means no detected errors.
	 */
	freechip_nand_readl(info, NANDFSR_OFFSET);
	nand_freechip_readecc_4bit(info, syndrome);
	if (!(syndrome[0] | syndrome[1] | syndrome[2] | syndrome[3]))
		return 0;

	/*
	 * Clear any previous address calculation by doing a dummy read of an
	 * error address register.
	 */
	freechip_nand_readl(info, NAND_ERR_ADD1_OFFSET);

	/* Start address calculation, and wait for it to complete.
	 * We _could_ start reading more data while this is working,
	 * to speed up the overall page read.
	 */
	freechip_nand_writel(info, NANDFCR_OFFSET,
			freechip_nand_readl(info, NANDFCR_OFFSET) | BIT(13));
	for (;;) {
		u32	fsr = freechip_nand_readl(info, NANDFSR_OFFSET);

		switch ((fsr >> 8) & 0x0f) {
		case 0:		/* no error, should not happen */
			freechip_nand_readl(info, NAND_ERR_ERRVAL1_OFFSET);
			return 0;
		case 1:		/* five or more errors detected */
			freechip_nand_readl(info, NAND_ERR_ERRVAL1_OFFSET);
			return -EIO;
		case 2:		/* error addresses computed */
		case 3:
			num_errors = 1 + ((fsr >> 16) & 0x03);
			goto correct;
		default:	/* still working on it */
			cpu_relax();
			continue;
		}
	}

correct:
	/* correct each error */
	for (i = 0, corrected = 0; i < num_errors; i++) {
		int error_address, error_value;

		if (i > 1) {
			error_address = freechip_nand_readl(info,
						NAND_ERR_ADD2_OFFSET);
			error_value = freechip_nand_readl(info,
						NAND_ERR_ERRVAL2_OFFSET);
		} else {
			error_address = freechip_nand_readl(info,
						NAND_ERR_ADD1_OFFSET);
			error_value = freechip_nand_readl(info,
						NAND_ERR_ERRVAL1_OFFSET);
		}

		if (i & 1) {
			error_address >>= 16;
			error_value >>= 16;
		}
		error_address &= 0x3ff;
		error_address = (512 + 7) - error_address;

		if (error_address < 512) {
			data[error_address] ^= error_value;
			corrected++;
		}
	}

	return corrected;
}
#endif
#define MAX_PROGRAM_SZ      (0x1000)
#define BOOT_ADDR           (0x410000)
#define ECC_STEP            (512 >>2) //words

#define  MAE0_CR_BASE             (MAE0_BASE + (15<<10) + 0x00*4)   
#define  HOG_SOFT_RST_N_REG       (*(volatile unsigned int *)( MAE0_CR_BASE + 0x7D*4))  

#define  ECC_BASE     MAE0_BASE + (8<<10)
#define  BCH_PARAM0   *(volatile unsigned int *)(ECC_BASE+0x00*4)
#define  BCH_PARAM1   *(volatile unsigned int *)(ECC_BASE+0x01*4)
#define  BCH_PARAM2   *(volatile unsigned int *)(ECC_BASE+0x02*4)
#define  BCH_PARAM3   *(volatile unsigned int *)(ECC_BASE+0x03*4)
#define  BCH_PARAM4   *(volatile unsigned int *)(ECC_BASE+0x04*4)
#define  BCH_PARAM5   *(volatile unsigned int *)(ECC_BASE+0x05*4)
#define  BCH_PARAM6   *(volatile unsigned int *)(ECC_BASE+0x06*4)
#define  BCH_PARAM7   *(volatile unsigned int *)(ECC_BASE+0x07*4)
#define  BCH_PARAM8   *(volatile unsigned int *)(ECC_BASE+0x08*4)
#define  BCH_CLR      *(volatile unsigned int *)(ECC_BASE+0x20*4)
#define  BCH_STATUS   *(volatile unsigned int *)(ECC_BASE+0x21*4)

#define  ECC_RAM0_ADDR(x)   *(volatile unsigned int *)(MAE0_BASE+ 0*0x0000 + x )

#define  DATA_BYTE      512
#define  REDUND_BYTE    15
#define  T_PARAM              9
#define  N_PARAM              4213
#define  N_DIV3_PARAM     1404
#define  mPARAM_ODD     1
#define  DMA_MODE       0
#define  TRACETESTVAL   513
#define  REDUND_BITS    117
#define  DMA_OUT_STEP   256

static int t18_nand_ecc_encoder(struct mtd_info *mtd, const u_char *src_byte,
				  u_char *dst_byte)
{
	unsigned int param0, param1, param2, param3, param4, param5, param6, param7;
	int i;
	unsigned int tmp;
	unsigned int SRC_DATA_ADDR = (unsigned int)src_byte;
	unsigned int DST_DATA_ADDR = (unsigned int)dst_byte;
	unsigned int SRC_DATA_SIZE = 512;
	unsigned int DST_DATA_SIZE = 528;
	unsigned int *p32;
	unsigned int bch_ret;
	int repeat_times = 8;

	volatile unsigned int status;

	HOG_SOFT_RST_N_REG = 0xff ;
	HOG_SOFT_RST_N_REG = 0 ;
	HOG_SOFT_RST_N_REG = 0xff ;

	p32 = (unsigned int *)src_byte;

	for (i = 0; i < 128; i++)
		ECC_RAM0_ADDR(i * 4) = p32[i];

	param2 = (mPARAM_ODD << 26) | (DMA_MODE << 27);
	param3 = (TRACETESTVAL << 0) | (REDUND_BITS << 14);
	param4 = DMA_OUT_STEP;
	param5 = SRC_DATA_ADDR;
	param6 = DST_DATA_ADDR;
	param7 = (DST_DATA_SIZE << 13) | SRC_DATA_SIZE;
	
	BCH_PARAM2 = param2;
	BCH_PARAM3 = param3;
	BCH_PARAM4 = param4;
	BCH_PARAM5 = param5;
	BCH_PARAM6 = param6;
	BCH_PARAM7 = param7;

	param1 = (1 << 26) | (T_PARAM << 19) | (REDUND_BYTE << 11) | (512);
	BCH_PARAM1 = param1;

	BCH_PARAM0 = (1 << 1);

	while ((BCH_STATUS & 1) == 0);
 
	p32 = (unsigned int *)dst_byte;
	for (i = 0; i < 132; i++)
		p32[i] = ECC_RAM0_ADDR(i * 4);
   
	BCH_CLR = 1;

	return  0;

}

static int t18_nand_ecc_decoder(struct mtd_info *mtd, u_char *src_byte,
				 u_char *dst_byte)
{
	unsigned int param0, param1, param2, param3, param4, param5, param6, param7;
	int i;
	unsigned int tmp;
	unsigned int SRC_DATA_ADDR = (unsigned int)src_byte;
	unsigned int DST_DATA_ADDR = (unsigned int)dst_byte;
	unsigned int SRC_DATA_SIZE = 528;
	unsigned int DST_DATA_SIZE = 512;
	unsigned int *p32;
	unsigned int bch_ret;
	int repeat_times = 8;

	volatile unsigned int status;

	HOG_SOFT_RST_N_REG = 0xff ;
	HOG_SOFT_RST_N_REG = 0 ;
	HOG_SOFT_RST_N_REG = 0xff ;

	p32 = (unsigned int *)src_byte;

	for (i = 0; i < 132; i++)
		ECC_RAM0_ADDR(i * 4) = p32[i];

	param2 = (mPARAM_ODD << 26) | (DMA_MODE << 27);
	param3 = (TRACETESTVAL << 0) | (REDUND_BITS << 14);
	param4 = DMA_OUT_STEP;
	param5 = SRC_DATA_ADDR;
	param6 = DST_DATA_ADDR;
	param7 = (repeat_times << 26) | (DST_DATA_SIZE << 13) | SRC_DATA_SIZE;
	BCH_PARAM2 = param2;
	BCH_PARAM3 = param3;
	BCH_PARAM4 = param4;
	BCH_PARAM5 = param5;
	BCH_PARAM6 = param6;
	BCH_PARAM7 = param7;

	param1 = (0 << 26) | (T_PARAM << 19) | (REDUND_BYTE << 11) | (512);
	BCH_PARAM1 = param1;

	BCH_PARAM0 = (1 << 0);

	while ((BCH_STATUS & 1) == 0);

	bch_ret = (BCH_STATUS >> 3) & 3;

	p32 = (unsigned int *)dst_byte;
	for (i = 0; i < 128; i++)
		p32[i] = ECC_RAM0_ADDR(i * 4);
   
	BCH_CLR = 1;

	return (bch_ret == 2) ? 1 : 0;
}

static void t18_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
}

static int t18_nand_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
			uint8_t *buf, int oob_required, int page)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	int readlen = eccsize + eccbytes;
	int stat;
	uint8_t *p = buf;
	u32 psrc[528/4];
	u32 pdst[512/4];
	int j, cell;
	u8 *pd;
	u32 eccstat_failed;

	//printk("t18 nand read one page hwecc @page %d\n", page);
	//printk("r@p%d\n", page);
	//printk("eccsize = %d, eccbytes = %d, eccsteps = %d\n", eccsize, eccbytes, eccsteps);
	mdelay(1);
	//udelay(20);

	eccstat_failed = mtd->ecc_stats.failed;
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		//printk("readlen = %d\n", readlen);
		chip->read_buf(mtd, (u8 *)psrc, readlen);
#if 0
		//if ((page > 37825) || (page == 7256)) {
		if (page  > 21377) {
		//////////////////////////////////////
		j = readlen >> 4;
		pd = (u8 *)psrc;
		printk("before decoder: \n");
		while (j--) {
			printk("\t%02x %02x %02x %02x %02x %02x %02x %02x"
				  "  %02x %02x %02x %02x %02x %02x %02x %02x\n",
				   pd[0], pd[1], pd[2], pd[3], pd[4], pd[5], pd[6], pd[7],
				   pd[8], pd[9], pd[10], pd[11], pd[12], pd[13], pd[14],
				   pd[15]);
			pd += 16;
		}
		/////////////////////////////////////
		}
#endif
		stat = t18_nand_ecc_decoder(mtd, (u8 *)psrc, (u8 *)pdst);
		memcpy(p, (u8 *)pdst, eccsize);
#if 0
		//////////////////////////////////////
		j = eccsize >> 4;
		pd = p;
		printk("after decoder: \n");
		while (j--) {
			printk("\t%02x %02x %02x %02x %02x %02x %02x %02x"
				  "  %02x %02x %02x %02x %02x %02x %02x %02x\n",
				   pd[0], pd[1], pd[2], pd[3], pd[4], pd[5], pd[6], pd[7],
				   pd[8], pd[9], pd[10], pd[11], pd[12], pd[13], pd[14],
				   pd[15]);
			pd += 16;
		}
		/////////////////////////////////////
#endif
#if 1 // active ecc flag
#if 1
		// make erased cell ecc correct
		if (stat) {
			//printk("t18 ecc result adjuct\n");
			cell = eccsize / 4;
			while (cell--) {
				if (psrc[cell] != 0xffffffff) {
					printk("%x %d %d\n", psrc[cell], cell, eccsteps);
					goto t18_nand_read_exit;
				}
			}
			stat = 0;
		}
t18_nand_read_exit:
#endif
		if (stat) {// ecc failed
			mtd->ecc_stats.failed++;
#if 0
			//////////////////////////////////////
			j = readlen >> 4;
			pd = (u8 *)psrc;
			printk("before decoder: @%d\n", eccsteps);
			while (j--) {
				printk("\t%02x %02x %02x %02x %02x %02x %02x %02x"
					  "  %02x %02x %02x %02x %02x %02x %02x %02x\n",
					   pd[0], pd[1], pd[2], pd[3], pd[4], pd[5], pd[6], pd[7],
					   pd[8], pd[9], pd[10], pd[11], pd[12], pd[13], pd[14],
					   pd[15]);
				pd += 16;
			}
			/////////////////////////////////////
#endif
#if 0
			//////////////////////////////////////
			j = eccsize >> 4;
			pd = p;
			printk("after decoder: \n");
			while (j--) {
				printk("\t%02x %02x %02x %02x %02x %02x %02x %02x"
					  "  %02x %02x %02x %02x %02x %02x %02x %02x\n",
					   pd[0], pd[1], pd[2], pd[3], pd[4], pd[5], pd[6], pd[7],
					   pd[8], pd[9], pd[10], pd[11], pd[12], pd[13], pd[14],
					   pd[15]);
				pd += 16;
			}
			/////////////////////////////////////
#endif

		}
		//else
		//	mtd->ecc_stats.corrected++;
		mtd->ecc_stats.corrected = 0;
#else
		mtd->ecc_stats.failed = 0;
		mtd->ecc_stats.corrected = 0;
#endif
	}
	//printk("ecc_stats.failed = %d, ecc_status.corrected = %d\n", mtd->ecc_stats.failed, mtd->ecc_stats.corrected);
	if (eccstat_failed != mtd->ecc_stats.failed)
		printk("ecc_stats.failed = %d\n", mtd->ecc_stats.failed);
	return 0;
}

static int t18_nand_read_subpage_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
			uint32_t offs, uint32_t len, uint8_t *buf, int page)
{
	printk("t18_nand_read_subpage_hwecc\n");
}
		

static int t18_nand_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf, int oob_required, int page)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	int writelen = eccsize + eccbytes;
	const uint8_t *p = buf;
	u32 psrc[512/4];
	u32 pdst[528/4];
	u8 *pd;
	int j;
	int cell;

	//printk("t18 nand write page hwecc\n");
	//printk("w@p%d\n", page);
	mdelay(1);
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		memcpy((u8 *)psrc, p, eccsize);
		cell = eccsize / 4;
		while (cell--) {
			if (psrc[cell] != 0xffffffff) {
				goto t18_nand_ecc_do_encoder;
			}
		}
		memcpy((u8 *)pdst, (u8 *)psrc, eccsize);
		pdst[eccsize/4] = 0xffffffff;
		pdst[eccsize/4+1] = 0xffffffff;
		pdst[eccsize/4+2] = 0xffffffff;
		pdst[eccsize/4+3] = 0xffffffff;
		goto t18_nand_ecc_do_write_page;
t18_nand_ecc_do_encoder:
		t18_nand_ecc_encoder(mtd, (u8 *)psrc, (u8 *)pdst);
t18_nand_ecc_do_write_page:
		chip->write_buf(mtd, (u8 *)pdst, writelen);
#if 0
		//if (page > 37825) {
		//if (page > 21377) {
		//if ((page == 5699) || (page == 7262)) {
		if (page == 8896) {
		j = writelen >> 4;
		pd = (u8 *)pdst;
		printk("ENCODED: \n");
		while (j--) {
			printk("\t%02x %02x %02x %02x %02x %02x %02x %02x"
				  "  %02x %02x %02x %02x %02x %02x %02x %02x\n",
				   pd[0], pd[1], pd[2], pd[3], pd[4], pd[5], pd[6], pd[7],
				   pd[8], pd[9], pd[10], pd[11], pd[12], pd[13], pd[14],
				   pd[15]);
			pd += 16;
		}
		}
#endif
	}
	mdelay(1);
	return 0;
}

static int t18_nand_write_subpage_hwecc(struct mtd_info *mtd,
				struct nand_chip *chip, uint32_t offset,
				uint32_t data_len, const uint8_t *buf,
				int oob_required, int page)
{
	return t18_nand_write_page_hwecc(mtd, chip, buf, oob_required, page);
}

static int t18_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
			int page)
{
	printk("t18 nand write oob none\n");
}

/*----------------------------------------------------------------------*/

/*
 * NOTE:  NAND boot requires ALE == EM_A[1], CLE == EM_A[2], so that's
 * how these chips are normally wired.  This translates to both 8 and 16
 * bit busses using ALE == BIT(3) in byte addresses, and CLE == BIT(4).
 *
 * For now we assume that configuration, or any other one which ignores
 * the two LSBs for NAND access ... so we can issue 32-bit reads/writes
 * and have that transparently morphed into multiple NAND operations.
 */
static void nand_freechip_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	//struct nand_chip *chip = mtd->priv;
	struct nand_chip *chip = mtd_to_nand(mtd);
#if 0
	u32 *data = (u32 * )buf;
	int i;

	for (i = 0; i < (len >> 2); i++)
		*data++ = ioread32(chip->IO_ADDR_R);
#endif
#if 1
	if ((0x03 & ((unsigned)buf)) == 0 && (0x03 & len) == 0)
		ioread32_rep(chip->IO_ADDR_R, buf, len >> 2);
	else if ((0x01 & ((unsigned)buf)) == 0 && (0x01 & len) == 0)
		ioread16_rep(chip->IO_ADDR_R, buf, len >> 1);
	else
		ioread8_rep(chip->IO_ADDR_R, buf, len);
#endif
}

static void nand_freechip_write_buf(struct mtd_info *mtd,
		const uint8_t *buf, int len)
{
	//struct nand_chip *chip = mtd->priv;
	struct nand_chip *chip = mtd_to_nand(mtd);
#if 0
	u32 *data = (u32 *)buf;
	int i;

	for (i = 0; i < (len >> 2); i++)
		iowrite32(*data++, chip->IO_ADDR_W);
#endif
#if 1
	if ((0x03 & ((unsigned)buf)) == 0 && (0x03 & len) == 0)
		iowrite32_rep(chip->IO_ADDR_R, buf, len >> 2);
	else if ((0x01 & ((unsigned)buf)) == 0 && (0x01 & len) == 0)
		iowrite16_rep(chip->IO_ADDR_R, buf, len >> 1);
	else
		iowrite8_rep(chip->IO_ADDR_R, buf, len);
#endif
}

static u32 nand_read_long(struct mtd_info *mtd)
{
	//struct nand_chip *chip = mtd->priv;
	struct nand_chip *chip = mtd_to_nand(mtd);
	return readl(chip->IO_ADDR_R);
}

static u8 nand_read_byte(struct mtd_info *mtd)
{
	//struct nand_chip *chip = mtd->priv;
	struct nand_chip *chip = mtd_to_nand(mtd);
	return (u8)readl(chip->IO_ADDR_R);
}


/*
 * Check hardware register for wait status. Returns 1 if device is ready,
 * 0 if it is still busy.
 */
static int nand_freechip_dev_ready(struct mtd_info *mtd)
{
	//struct freechip_nand_info *info = to_freechip_nand(mtd);
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct freechip_nand_info *info = to_freechip_nand(chip);

	//printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	return freechip_nand_readl(info, NAND_STATUS_OFFSET) & BIT(7);
}

static int nand_freechip_block_bad(struct mtd_info *mtd, loff_t ofs)
{
	//printk("lx28xx blcok bad\n");
	return 0;
}

static void __init nand_t18xxevm_flash_init(struct freechip_nand_info *info)
{
	uint32_t regval, a1cr;

	printk("nand flash init...\n");
	regval = 0x2fffff;
	//freechip_nand_writel(info, A1CR_OFFSET, regval);
	freechip_nand_writel(info,NAND_CR_OFFSET, regval);
}

#if 0
/*----------------------------------------------------------------------*/

/* An ECC layout for using 4-bit ECC with small-page flash, storing
 * ten ECC bytes plus the manufacturer's bad block marker byte, and
 * and not overlapping the default BBT markers.
 */
static struct nand_ecclayout hwecc4_small __initconst = {
	.eccbytes = 10,
	.eccpos = { 0, 1, 2, 3, 4,
		/* offset 5 holds the badblock marker */
		6, 7,
		13, 14, 15, },
	.oobfree = {
		{.offset = 8, .length = 5, },
		{.offset = 16, },
	},
};

/* An ECC layout for using 4-bit ECC with large-page (2048bytes) flash,
 * storing ten ECC bytes plus the manufacturer's bad block marker byte,
 * and not overlapping the default BBT markers.
 */
static struct nand_ecclayout hwecc4_2048 __initconst = {
	.eccbytes = 40,
	.eccpos = {
		/* at the end of spare sector */
		24, 25, 26, 27, 28, 29,	30, 31, 32, 33,
		34, 35, 36, 37, 38, 39,	40, 41, 42, 43,
		44, 45, 46, 47, 48, 49, 50, 51, 52, 53,
		54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
		},
	.oobfree = {
		/* 2 bytes at offset 0 hold manufacturer badblock markers */
		{.offset = 2, .length = 22, },
		/* 5 bytes at offset 8 hold BBT markers */
		/* 8 bytes at offset 16 hold JFFS2 clean markers */
	},
};
#endif
static int __init nand_freechip_probe(struct platform_device *pdev)
{
	struct freechip_nand_pdata	*pdata = pdev->dev.platform_data;
	struct freechip_nand_info	*info;
	struct resource			*res1;
	void __iomem			*base;
	int				ret;
	uint32_t			val;
	nand_ecc_modes_t		ecc_mode;

	printk("nand probe...\n");
	/* insist on board-specific configuration */
	if (!pdata)
		return -ENODEV;

	/* which external chipselect will we be managing? */
	if (pdev->id < 0 || pdev->id > 3)
		return -ENODEV;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "unable to allocate memory\n");
		ret = -ENOMEM;
		goto err_nomem;
	}

	platform_set_drvdata(pdev, info);

	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res1 ) {
		dev_err(&pdev->dev, "resource missing\n");
		ret = -EINVAL;
		goto err_nomem;
	}

	//base = ioremap(res1->start, res1->end - res1->start);
	base = res1->start;
	printk("base = 0x%p\n", base);
	if ( !base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -EINVAL;
		goto err_ioremap;
	}
	info->dev		= &pdev->dev;
	info->base		= base;

	info->chip.mtd.priv		= &info->chip;
	info->chip.mtd.name		= dev_name(&pdev->dev);
	info->chip.mtd.owner		= THIS_MODULE;

	info->chip.mtd.dev.parent	= &pdev->dev;

	info->chip.IO_ADDR_R	= base + NAND_RW_OFFSET;
	info->chip.IO_ADDR_W	= base + NAND_RW_OFFSET;
	info->chip.chip_delay	= 0;
	info->chip.select_chip	= nand_freechip_select_chip;

	/* options such as NAND_USE_FLASH_BBT or 16-bit widths */
	info->chip.options	= pdata->options;//NAND_SKIP_BBTSCAN
 
	info->core_chipsel	= pdev->id;
	info->mask_chipsel	= pdata->mask_chipsel;

	info->chip.block_bad = nand_freechip_block_bad;
	/* Set address of hardware control function */
	info->chip.cmd_ctrl	= nand_freechip_hwcontrol;
	//info->chip.dev_ready	= nand_freechip_dev_ready;
	info->chip.chip_delay = 400000;

	/* Speed up buffer I/O */
	info->chip.read_buf     = nand_freechip_read_buf;
	info->chip.write_buf    = nand_freechip_write_buf;

	/*specific nand read */
	info->chip.read_byte 	= nand_read_byte;
	info->chip.read_long    = nand_read_long;
	//info->chip.read_word = nand_read_long;

	/* Use board-specific ECC config */
	ecc_mode		= pdata->ecc_mode;

	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	printk("pdata->ecc_mode = %d\n", pdata->ecc_mode);
	ret = -EINVAL;
	switch (ecc_mode) {
	case NAND_ECC_NONE:
	case NAND_ECC_SOFT:
		pdata->ecc_bits = 0;
		break;
	case NAND_ECC_HW:
#if 0
		if (pdata->ecc_bits == 4) {
			/* No sanity checks:  CPUs must support this,
			 * and the chips may not use NAND_BUSWIDTH_16.
			 */

			/* No sharing 4-bit hardware between chipselects yet */
			spin_lock_irq(&freechip_nand_lock);
			if (ecc4_busy)
				ret = -EBUSY;
			else
				ecc4_busy = true;
			spin_unlock_irq(&freechip_nand_lock);

			if (ret == -EBUSY)
				goto err_ecc;

		#if 0
			info->chip.ecc.calculate = nand_freechip_calculate_4bit;
			info->chip.ecc.correct = nand_freechip_correct_4bit;
			info->chip.ecc.hwctl = nand_freechip_hwctl_4bit;
			info->chip.ecc.bytes = 10;
		} else {
			info->chip.ecc.calculate = nand_freechip_calculate_1bit;
			info->chip.ecc.correct = nand_freechip_correct_1bit;
			info->chip.ecc.hwctl = nand_freechip_hwctl_1bit;
			info->chip.ecc.bytes = 3;
		#endif
		}
#endif
		info->chip.ecc.hwctl = t18_nand_enable_hwecc;
		info->chip.ecc.write_page = t18_nand_write_page_hwecc;
		info->chip.ecc.write_subpage = t18_nand_write_subpage_hwecc;
		info->chip.ecc.read_page = t18_nand_read_page_hwecc;
		info->chip.ecc.read_subpage = t18_nand_read_subpage_hwecc;
		info->chip.ecc.write_oob = t18_nand_write_oob;
		info->chip.ecc.size = 512;
		info->chip.ecc.strength = 9;
		info->chip.ecc.bytes = 16;
		break;
	default:
		ret = -EINVAL;
		goto err_ecc;
	}
	info->chip.ecc.mode = ecc_mode;

#if 0
	info->clk = clk_get(&pdev->dev, "nand");
	if (IS_ERR(info->clk)) {
		ret = PTR_ERR(info->clk);
		dev_dbg(&pdev->dev, "unable to get AEMIF clock, err %d\n", ret);
		goto err_clk;
	}

	ret = clk_enable(info->clk);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "unable to enable AEMIF clock, err %d\n",
			ret);
		goto err_clk_enable;
	}
#endif
	/* EMIF timings should normally be set by the boot loader,
	 * especially after boot-from-NAND.  The *only* reason to
	 * have this special casing for the T18xx EVM is to work
	 * with boot-from-SPI...
	 */
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	nand_t18xxevm_flash_init(info);

	spin_lock_irq(&freechip_nand_lock);

	/* put CSxNAND into NAND mode */
	//val = freechip_nand_readl(info, NAND_CS_OFFSET);
	//val |= (info->core_chipsel);
	val = 0;
	freechip_nand_writel(info, NAND_CS_OFFSET, val);

	spin_unlock_irq(&freechip_nand_lock);

	/* Scan to find existence of the device(s) */
	ret = nand_scan_ident(&info->chip.mtd, pdata->mask_chipsel ? 2 : 1, NULL);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "no NAND chip(s) found\n");
		goto err_scan;
	}

	/* Update ECC layout if needed ... for 1-bit HW ECC, the default
	 * is OK, but it allocates 6 bytes when only 3 are needed (for
	 * each 512 bytes).  For the 4-bit HW ECC, that default is not
	 * usable:  10 bytes are needed, not 6.
	 */
	 #if 0
	if (pdata->ecc_bits == 4) {
		int	chunks = info->mtd.writesize / 512;

		if (!chunks || info->mtd.oobsize < 16) {
			dev_dbg(&pdev->dev, "too small\n");
			ret = -EINVAL;
			goto err_scan;
		}

		/* For small page chips, preserve the manufacturer's
		 * badblock marking data ... and make sure a flash BBT
		 * table marker fits in the free bytes.
		 */
		if (chunks == 1) {
			info->ecclayout = hwecc4_small;
			info->ecclayout.oobfree[1].length =
				info->mtd.oobsize - 16;
			goto syndrome_done;
		}
		if (chunks == 4) {
			info->ecclayout = hwecc4_2048;
			info->chip.ecc.mode = NAND_ECC_HW_OOB_FIRST;
			goto syndrome_done;
		}

		/* 4KiB page chips are not yet supported. The eccpos from
		 * nand_ecclayout cannot hold 80 bytes and change to eccpos[]
		 * breaks userspace ioctl interface with mtd-utils. Once we
		 * resolve this issue, NAND_ECC_HW_OOB_FIRST mode can be used
		 * for the 4KiB page chips.
		 */
		dev_warn(&pdev->dev, "no 4-bit ECC support yet "
				"for 4KiB-page NAND\n");
		ret = -EIO;
		goto err_scan;

syndrome_done:
		info->chip.ecc.layout = &info->ecclayout;
	}
	#endif

	ret = nand_scan_tail(&info->chip.mtd);
	if (ret < 0)
		goto err_scan;

	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	if (mtd_has_partitions()) {
		struct mtd_partitions	mtd_parts;
		int			mtd_parts_nb = 0;
#if 0
		if (mtd_has_cmdlinepart()) {
			static const char *probes[] __initconst =
				{ "cmdlinepart", NULL };

			//mtd_parts_nb = parse_mtd_partitions(&info->mtd, probes,
			//				    &mtd_parts, 0);
			ret = parse_mtd_partitions(&info->chip.mtd, probes, &mtd_parts, 0);
			if (ret == 0)
				mtd_parts_nb = mtd_parts.nr_parts;
		}
#endif
		if (mtd_parts_nb <= 0) {
			mtd_parts.parts = pdata->parts;
			mtd_parts_nb = pdata->nr_parts;
		}

		printk("mtd_parts_nb = %d, pdata->nr_parts = %d\n", mtd_parts_nb, pdata->nr_parts);
		/* Register any partitions */
		if (mtd_parts_nb > 0) {
			ret = add_mtd_partitions(&info->chip.mtd, mtd_parts.parts, mtd_parts_nb);
			if (ret == 0)
				info->partitioned = true;
		}

	} else if (pdata->nr_parts) {
		dev_warn(&pdev->dev, "ignoring %d default partitions on %s\n",
				pdata->nr_parts, info->chip.mtd.name);
	}

	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	/* If there's no partition info, just package the whole chip
	 * as a single MTD device.
	 */
	if (!info->partitioned)
		ret = add_mtd_device(&info->chip.mtd) ? -ENODEV : 0;

	if (ret < 0)
		goto err_scan;

	return 0;

err_scan:
	clk_disable(info->clk);

err_clk_enable:
	clk_put(info->clk);

err_ecc:
err_clk:
err_ioremap:
	if (base)
		iounmap(base);


err_nomem:
	kfree(info);
	return ret;
}

static int __exit nand_freechip_remove(struct platform_device *pdev)
{
	struct freechip_nand_info *info = platform_get_drvdata(pdev);
	int status;

	if (mtd_has_partitions() && info->partitioned)
		status = del_mtd_partitions(&info->chip.mtd);
	else
		status = del_mtd_device(&info->chip.mtd);

	spin_lock_irq(&freechip_nand_lock);
	if (info->chip.ecc.mode == NAND_ECC_HW_SYNDROME)
		ecc4_busy = false;
	spin_unlock_irq(&freechip_nand_lock);

	iounmap(info->base);

	nand_release(&info->chip.mtd);
	clk_disable(info->clk);
	clk_put(info->clk);

	kfree(info);

	return 0;
}

static struct platform_driver nand_freechip_driver = {
	.remove		= __exit_p(nand_freechip_remove),
	.driver		= {
		.name	= "lx28xx_nand",
	},
};
MODULE_ALIAS("platform:freechip_nand");

static int __init nand_freechip_init(void)
{
	return platform_driver_probe(&nand_freechip_driver, nand_freechip_probe);
}
module_init(nand_freechip_init);

static void __exit nand_freechip_exit(void)
{
	platform_driver_unregister(&nand_freechip_driver);
}
module_exit(nand_freechip_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xiangjing Inc.");
MODULE_DESCRIPTION("T19XX Soc NAND flash driver");


