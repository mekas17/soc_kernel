/*
 *  linux/arch/armnommu/mach-atmel/irq.c
 *
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2003 SAMSUNG ELECTRONICS 
 *	      Hyok S. Choi (hyok.choi@samsung.com)
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/map.h>
#include <asm/mach/irq.h>
#include <asm/setup.h>
#include <mach/irqs.h>

void gic_init(unsigned int gic_nr, int irq_start,
		     void __iomem *dist_base, void __iomem *cpu_base);

static uint32_t int_src_map[NR_IRQS];

static void int_source_map(void)
{
    int_src_map[0]  =  0; 
    int_src_map[1]  =  LX28XX_MCAN0_INT        ;
    int_src_map[2]  =  LX28XX_MCAN1_INT        ;
    int_src_map[3]  =  LX28XX_MCAN2_INT        ;
    int_src_map[4]  =  LX28XX_RCRT_INT         ;
    int_src_map[5]  =  LX28XX_I2C1_INT         ;
    int_src_map[6]  =  LX28XX_I2C_INT         ;
    int_src_map[7]  =  LX28XX_SDIO_INT         ;
    int_src_map[8]  =  LX28XX_SDMMC_INT        ;
    int_src_map[9]  =  LX28XX_SPI_INT         ;
    int_src_map[10] =  LX28XX_SPI1_INT         ;
    int_src_map[11] =  LX28XX_SPI2_INT         ;
    int_src_map[12] =  LX28XX_SPI3_INT         ;
    int_src_map[13] =  LX28XX_SPI4_INT         ;
    int_src_map[14] =  LX28XX_SPI5_INT         ;
    int_src_map[15] =  LX28XX_SPI6_INT         ;
    int_src_map[16] =  LX28XX_SPI7_INT         ;
    int_src_map[17] =  LX28XX_SPI8_INT         ;
    int_src_map[18] =  LX28XX_SPI9_INT         ;
    int_src_map[19] =  LX28XX_UART0_INT        ;
    int_src_map[20] =  LX28XX_UART1_INT        ;
    int_src_map[21] =  LX28XX_UART2_INT        ;
    int_src_map[22] =  LX28XX_UART3_INT        ;
    int_src_map[23] =  LX28XX_UART4_INT        ;
    int_src_map[24] =  LX28XX_UART5_INT        ;
    int_src_map[25] =  LX28XX_UART6_INT        ;
    int_src_map[26] =  LX28XX_UART7_INT        ;
    int_src_map[27] =  LX28XX_UART8_INT        ;
    int_src_map[28] =  LX28XX_GPIO_INT         ;
    int_src_map[29] =  LX28XX_TIMER0_INT       ;
    int_src_map[30] =  LX28XX_TIMER1_INT       ;
    int_src_map[31] =  LX28XX_TIMER2_INT       ;
    int_src_map[32] =  LX28XX_TIMER3_INT       ;
    int_src_map[33] =  LX28XX_TIMER4_INT       ;
    int_src_map[34] =  LX28XX_TIMER5_INT       ;
    int_src_map[35] =  LX28XX_TIMER6_INT       ;
    int_src_map[36] =  LX28XX_TIMER7_INT       ;
    int_src_map[37] =  LX28XX_TIMER8_INT       ;
    int_src_map[38] =  LX28XX_TIMER9_INT       ;
    int_src_map[39] =  LX28XX_TIMER10_INT      ;
    int_src_map[40] =  LX28XX_TIMER11_INT      ;
    int_src_map[41] =  LX28XX_TIMER12_INT      ;
    int_src_map[42] =  LX28XX_TIMER13_INT      ;
    int_src_map[43] =  LX28XX_TIMER14_INT      ;
    int_src_map[44] =  LX28XX_TIMER15_INT      ;
    int_src_map[45] =  LX28XX_WDT_INT          ;
    int_src_map[46] =  LX28XX_PWM_INT          ;
    int_src_map[47] =  LX28XX_I2S_INT         ;
    int_src_map[48] =  LX28XX_I2S1_INT         ;
    int_src_map[49] =  LX28XX_I2S2_INT         ;
    int_src_map[50] =  LX28XX_SPDIF0_RX_INT    ;
    int_src_map[51] =  LX28XX_SPDIF0_TX_INT    ;
    int_src_map[52] =  LX28XX_RTC_INT          ;
    int_src_map[53] =  LX28XX_EXT0_INT         ;
    int_src_map[54] =  LX28XX_EXT1_INT         ;
    int_src_map[55] =  LX28XX_EXT2_INT         ;
    int_src_map[56] =  LX28XX_EXT3_INT         ;
    int_src_map[57] =  LX28XX_EXT4_INT         ;
    int_src_map[58] =  LX28XX_EXT5_INT         ;
    int_src_map[59] =  LX28XX_EXT6_INT         ;
    int_src_map[60] =  LX28XX_PLC_CO_CTRL0_INT ;
    int_src_map[61] =  LX28XX_PLC_CO_CTRL1_INT ;
    int_src_map[62] =  LX28XX_ADC_INT          ;
    int_src_map[63] =  LX28XX_ADC2_INT         ;
    int_src_map[64] =  LX28XX_MAC2_INT         ;
    int_src_map[65] =  LX28XX_MAC_INT         ;
    int_src_map[66] =  LX28XX_USB_INT          ;
    int_src_map[67] =  LX28XX_USB2_INT         ;
    int_src_map[68] =  LX28XX_IT656_INT        ;
    int_src_map[69] =  LX28XX_LCD_INT          ;
    int_src_map[70] =  LX28XX_DMA_INT          ;
    int_src_map[71] =  LX28XX_MAE0_INT         ;
    int_src_map[72] =  LX28XX_DMA2_INT         ;
    int_src_map[73] =  LX28XX_SATA_INT         ;
    int_src_map[74] =  LX28XX_PCIE_INT         ;
    int_src_map[75] =  LX28XX_MAE2_INT         ;
    int_src_map[76] =  LX28XX_HDMI_INT         ;
    int_src_map[77] =  LX28XX_HDMI_HPD_INT     ;
    int_src_map[78] =  LX28XX_GPU_IRQPMU_INT   ;
    int_src_map[79] =  LX28XX_GPU_IRQGPMMU_INT ;
    int_src_map[80] =  LX28XX_GPU_IRQGP_INT    ;
    int_src_map[81] =  LX28XX_GPU_IRQPPMMU1_INT;
    int_src_map[82] =  LX28XX_GPU_IRQPP1_INT   ;
    int_src_map[83] =  LX28XX_GPU_IRQPPMMU0_INT;
    int_src_map[84] =  LX28XX_GPU_IRQPP0_INT   ;
    int_src_map[85] =  85;
    int_src_map[86] =  86;
    int_src_map[87] =  87;
    int_src_map[88] =  88;
    int_src_map[89] =  89;
    int_src_map[90] =  90;
    int_src_map[91] =  91;
    int_src_map[92] =  92;
    int_src_map[93] =  93;
    int_src_map[94] = LX28XX_SYS_INTR0_0_INT;
    int_src_map[95] = LX28XX_SYS_INTR0_1_INT;
}

static void
ark_dmp_irq_mask(struct irq_data *d )
{
	//early_print("dmp_irq_mask : %d\n", d->irq);
	//rICMASK |= (1 << d->irq);
	DisableIntNum(d->irq);
}

static  void
ark_dmp_irq_ack(struct irq_data *d )
{
	//early_print("dmp_irq_ack : %d\n", d->irq);
	//rIRQISPC = (1 << d->irq);
	ClearPend(d->irq);
}

static  void
ark_dmp_irq_maskack(struct irq_data *d )
{
	//early_print("dmp_irq_maskack : %d\n", d->irq);
	//rICMASK |= (1 << d->irq);
	//rIRQISPC = (1 << d->irq);
	DisableIntNum(d->irq);
	ClearPend(d->irq);
}


static void
ark_dmp_irq_unmask(struct irq_data *d )
{
	//early_print("irq_unmask: %d\n", d->irq);
	//rICMASK &= (~(1 << d->irq));
	EnableIntNum(d->irq);
}

static struct irq_chip ark_dmp_irq_level_chip = {
	.irq_ack	   = ark_dmp_irq_maskack,
	.irq_mask	   = ark_dmp_irq_mask,
	.irq_unmask	   = ark_dmp_irq_unmask,
	//.irq_mask_ack = ark_dmp_irq_maskack,
};

static struct irq_chip ark_dmp_irq_chip = {
	.irq_ack	   = ark_dmp_irq_ack,
	.irq_mask	   = ark_dmp_irq_mask,
	.irq_unmask	   = ark_dmp_irq_unmask,
	//.irq_mask_ack = ark_dmp_irq_maskack,
};

/* ark_dmp_init_irq
 *
 * Initialise ARK DMP IRQ system
*/

void __init lx28xx_init_irq(void)
{
#if 0
	int i, irq;

#if 0
   	//rMAC1 = 0x8000;
	rICSET = 0x08;
	for ( i = 0; i < 1000; i ++);
	
	rICSET = 0x05; //enabel irq disable fiq
	rICMODE = 0;	//0--> irq type, 1 --> fiq type
	rICMASK = 0xffffffff;

	rICLEVEL = 0;
#endif
	int_source_map();
	EnableGInt;
	early_print("rICGMASK : 0x%x\n", rICGMASK);

	for (irq = 0; irq < NR_IRQS; irq++) 
	{
#if 0
		switch(irq)
		{
		case LX28XX_TIMER0_INT:
		case LX28XX_TIMER1_INT:
			irq_set_chip(irq, &ark_dmp_irq_chip);
			irq_set_handler(irq, handle_edge_irq);
			//set_irq_flags(irq, IRQF_VALID);
			set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
			break;
		default:
			rICLEVEL |= (1 << irq);
			irq_set_chip(irq, &ark_dmp_irq_level_chip);
			irq_set_handler(irq, handle_level_irq);
			set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
			break;
		}			
#endif
		switch(int_src_map[irq])
		{
		case LX28XX_TIMER0_INT:
		case LX28XX_TIMER1_INT:
                        SetIntEdge(int_src_map[irq]);
			early_print("int_src_map[irq] : 0x%x\n", int_src_map[irq]);
			irq_set_chip(int_src_map[irq], &ark_dmp_irq_chip);
			irq_set_handler(int_src_map[irq], handle_edge_irq);
			//set_irq_flags(irq, IRQF_VALID);
			set_irq_flags(int_src_map[irq], IRQF_VALID | IRQF_PROBE);
			break;
		default:
			//rICLEVEL |= (1 << irq);
			SetIntLevel(int_src_map[irq]);
			irq_set_chip(int_src_map[irq], &ark_dmp_irq_level_chip);
			irq_set_handler(int_src_map[irq], handle_level_irq);
			set_irq_flags(int_src_map[irq], IRQF_VALID | IRQF_PROBE);
			break;
		}			
	}
	//early_print("%s line:%d rICLEVEL=%x\n", __FILE__, __LINE__, rICLEVEL);
	//early_print("%s line:%d rICMASK=%x \n", __FILE__, __LINE__, rICMASK);
#else
    gic_init(0, -1, (LX28XX_VA_PERIPH + 0x11000), (LX28XX_VA_PERIPH + 0x12000));
#endif
	
}

