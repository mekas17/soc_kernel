/*
 * Hardware definitions common to LX28XX SOC processors
 * 
 * Author: jjdeng @Xiangjing Inc.
 *
DMAC :  0x7000_0000
MAC  :  0x7001_0000
USB  :  0x7002_0000
SENSOR :0x7003_0000
LCD  :  0x7004_0000
ECC  :  0x7005_0000
HD   :  0x7006_0000
USB_2:  0x7007_0000


 CP_IP1_BASE :   0x4000_0000 
 CP_IP2_BASE :   0x3000_0000 
 NAND_BASE   :   0x5000_0000 

APB_SYS : 0x6000_0000
WDT     : 0x6001_0000
ICU     : 0x6002_0000
TIMER   : 0x6003_0000
GPIO    : 0x6005_0000
PWM     : 0x6006_0000
UART0   : 0x6007_0000
UART1   : 0x6008_0000
UART2   : 0x6009_0000
I2S     : 0x600B_0000
SDMMC   : 0x600C_0000
SDIO    : 0x600D_0000
I2C     : 0x600E_0000
SPI     : 0x600F_0000
RCRT    : 0x6010_0000
UART3   : 0x6011_0000
UART4   : 0x6012_0000
CP_IP3  : 0x601D_0000
CP_IP4  : 0x601E_0000
APB_CFG : 0x601F_0000

dma source :
[0] : uart0
[1] : uart1 
[2] : uart2
[3] : --
[4] : --
[5] : i2s_rx
[6] : i2s_tx
[7] : sdmmc
[8] : sdio
[9] : i2c_tx
[10]: i2c_rx
[11]: spi_rx
[12]: spi_tx
[13]: CP_IP3
[14]: CP_IP4
	

INTR SOURCE
[0] : --
[1] : WDT 
[2] : SPI 
[3] : I2C
[4] : RCRT
[5] : UART0
[6] : UART1
[7] : UART2
[8] : UART3
[9] : UART4
[10]: UART5|UART6
[11]: TIMER0
[12]: TIMER1
[13]: TIMER2
[14]: SDMMC
[15]: SDIO
[16]: GPIO_0
[17]: GPIO_1
[18]: I2S
[19]: PWM
[20]: CP
[21]: AXI_IMC_INTR|DSP
[22]: MAC
[23]: LCD
[24]: SENSOR
[25]: USB
[26]: DMA
[27]: ECC|DEINTERLACE
[28]: HD
[29]: USB2
[30]: --
[31]: --

 *
 */
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include<mach/map.h>


#define IO_BASE			0
#define PCIO_BASE		0

#define UART_CLK	24000000
#define SUART_CLK	60000000  //APB clock


/////////////////////////////////////////////////////
/* memory */
#define MEM_SIZE		(0x2000000)
#define MEM_BASE_PHY_ADDR	0x00000000
#define MEM_UNCACHE_BASE_ADDR	0xf2000000

/////////////////////////////////////////////////////
/* IRAM */

#define IRAM_SIZE				0x40000
//#define IRAM_BASE_PHY_ADDR		0x400000



/////////////////////////////////////////////////////
/* flash */
#define FLASH_SIZE				0x1000000
#define FLASH_BASE_PHY_ADDR		0x14000000

/////////////////////////////////////////////////////
/* bus addr */


#ifdef CONFIG_MMU

#define  AHB_BASE               LX28XX_VA_AHB_SYS
#define  APB0_BASE         		LX28XX_VA_APB_SYS
#define  IRAM_BASE_PHY_ADDR  	LX28XX_VA_SRAM
#define  PERIPH_BASE			LX28XX_VA_PERIPH
#define  MAE0_BASE 				LX28XX_VA_MAE0

#else
#define  AHB_BASE               0x70000000
#define  APB0_BASE         		0x60000000
#define  IRAM_BASE_PHY_ADDR		0x400000

#endif

/////////////////////////////////////////////////////
/* DMA */

#define DMA_BASE				(AHB_BASE + 0x00000)
#define LX28XX_DMAC_BASE				(DMA_BASE)

#define DMACIntStatus           *((volatile unsigned int*)(DMA_BASE + 0x000)) /*interrupt status*/
#define DMACIntTCStatus         *((volatile unsigned int*)(DMA_BASE + 0x004)) /*interrupt TC status*/
#define DMACIntTCClear          *((volatile unsigned int*)(DMA_BASE + 0x008))
#define DMACIntErrorStatus      *((volatile unsigned int*)(DMA_BASE + 0x00c))
#define DMACIntErrClr           *((volatile unsigned int*)(DMA_BASE + 0x010))
#define DMACRawIntTCStatus      *((volatile unsigned int*)(DMA_BASE + 0x014))
#define DMACRawIntErrorStatus   *((volatile unsigned int*)(DMA_BASE + 0x018))
#define DMACEnbldChns           *((volatile unsigned int*)(DMA_BASE + 0x01c))
#define DMACSoftBReq            *((volatile unsigned int*)(DMA_BASE + 0x020))
#define DMACSoftSReq            *((volatile unsigned int*)(DMA_BASE + 0x024))
#define DMACSoftLBReq           *((volatile unsigned int*)(DMA_BASE + 0x028))
#define DMACSoftLSReq           *((volatile unsigned int*)(DMA_BASE + 0x02c))
#define DMACConfiguration       *((volatile unsigned int*)(DMA_BASE + 0x030))
#define DMACSync                *((volatile unsigned int*)(DMA_BASE + 0x034))
#define DMACC0SrcAddr           *((volatile unsigned int*)(DMA_BASE + 0x100))
#define DMACC0DestAddr          *((volatile unsigned int*)(DMA_BASE + 0x104))
#define DMACC0LLI               *((volatile unsigned int*)(DMA_BASE + 0x108))
#define DMACC0Control           *((volatile unsigned int*)(DMA_BASE + 0x10c))
#define DMACC0Configuration     *((volatile unsigned int*)(DMA_BASE + 0x110))
#define DMACC1SrcAddr           *((volatile unsigned int*)(DMA_BASE + 0x120))
#define DMACC1DestAddr          *((volatile unsigned int*)(DMA_BASE + 0x124))
#define DMACC1LLI               *((volatile unsigned int*)(DMA_BASE + 0x128))
#define DMACC1Control           *((volatile unsigned int*)(DMA_BASE + 0x12c))
#define DMACC1Configuration     *((volatile unsigned int*)(DMA_BASE + 0x130))                               
#define DMACC2SrcAddr           *((volatile unsigned int*)(DMA_BASE + 0x140))
#define DMACC2DestAddr          *((volatile unsigned int*)(DMA_BASE + 0x144))
#define DMACC2LLI               *((volatile unsigned int*)(DMA_BASE + 0x148))
#define DMACC2Control           *((volatile unsigned int*)(DMA_BASE + 0x14c))
#define DMACC2Configuration     *((volatile unsigned int*)(DMA_BASE + 0x150))
#define DMACC3SrcAddr           *((volatile unsigned int*)(DMA_BASE + 0x160))
#define DMACC3DestAddr          *((volatile unsigned int*)(DMA_BASE + 0x164))
#define DMACC3LLI               *((volatile unsigned int*)(DMA_BASE + 0x168))
#define DMACC3Control           *((volatile unsigned int*)(DMA_BASE + 0x16c))
#define DMACC3Configuration     *((volatile unsigned int*)(DMA_BASE + 0x170))
#define DMACC4SrcAddr           *((volatile unsigned int*)(DMA_BASE + 0x180))
#define DMACC4DestAddr          *((volatile unsigned int*)(DMA_BASE + 0x184))
#define DMACC4LLI               *((volatile unsigned int*)(DMA_BASE + 0x188))
#define DMACC4Control           *((volatile unsigned int*)(DMA_BASE + 0x18c))
#define DMACC4Configuration     *((volatile unsigned int*)(DMA_BASE + 0x190))
#define DMACC5SrcAddr           *((volatile unsigned int*)(DMA_BASE + 0x1a0))
#define DMACC5DestAddr          *((volatile unsigned int*)(DMA_BASE + 0x1a4))
#define DMACC5LLI               *((volatile unsigned int*)(DMA_BASE + 0x1a8))
#define DMACC5Control           *((volatile unsigned int*)(DMA_BASE + 0x1ac))
#define DMACC5Configuration     *((volatile unsigned int*)(DMA_BASE + 0x1b0))
#define DMACC6SrcAddr           *((volatile unsigned int*)(DMA_BASE + 0x1c0))
#define DMACC6DestAddr          *((volatile unsigned int*)(DMA_BASE + 0x1c4))
#define DMACC6LLI               *((volatile unsigned int*)(DMA_BASE + 0x1c8))
#define DMACC6Control           *((volatile unsigned int*)(DMA_BASE + 0x1cc))
#define DMACC6Configuration     *((volatile unsigned int*)(DMA_BASE + 0x1d0))
#define DMACC7SrcAddr           *((volatile unsigned int*)(DMA_BASE + 0x1e0))
#define DMACC7DestAddr          *((volatile unsigned int*)(DMA_BASE + 0x1e4))
#define DMACC7LLI               *((volatile unsigned int*)(DMA_BASE + 0x1e8))
#define DMACC7Control           *((volatile unsigned int*)(DMA_BASE + 0x1ec))
#define DMACC7Configuration     *((volatile unsigned int*)(DMA_BASE + 0x1f0))                               
#define DMACITOP2               *((volatile unsigned int*)(DMA_BASE + 0x508))
#define DMACITOP3               *((volatile unsigned int*)(DMA_BASE + 0x50c))

/////////////////////////////////////////////////////
/* MAC */
#define MAC_BASE				(AHB_BASE + 0x140000)//(AHB_BASE + 0x140000)
#define MAC2_BASE 				(AHB_BASE + 0x150000)

#define rMAC1					*((volatile unsigned int *)(MAC_BASE + 0x000))    
#define rMAC2					*((volatile unsigned int *)(MAC_BASE + 0x004))    
#define rIPGT					*((volatile unsigned int *)(MAC_BASE + 0x008))    
#define rIPGR					*((volatile unsigned int *)(MAC_BASE + 0x00c))    
#define rCLRT					*((volatile unsigned int *)(MAC_BASE + 0x010))
#define rMAXF					*((volatile unsigned int *)(MAC_BASE + 0x014))
#define rSUPP					*((volatile unsigned int *)(MAC_BASE + 0x018))    
#define rTEST					*((volatile unsigned int *)(MAC_BASE + 0x01c))
#define rMCFG					*((volatile unsigned int *)(MAC_BASE + 0x020))
#define rMCMD					*((volatile unsigned int *)(MAC_BASE + 0x024))
#define rMADR					*((volatile unsigned int *)(MAC_BASE + 0x028))
#define rMWTD					*((volatile unsigned int *)(MAC_BASE + 0x02c))
#define rMRDD					*((volatile unsigned int *)(MAC_BASE + 0x030))    
#define rMIND					*((volatile unsigned int *)(MAC_BASE + 0x034))
#define rMACFIFOCFG0			*((volatile unsigned int *)(MAC_BASE + 0x03c))
#define rMACADDR1				*((volatile unsigned int *)(MAC_BASE + 0x040))    
#define rMACADDR2				*((volatile unsigned int *)(MAC_BASE + 0x044))    
#define rMACADDR3				*((volatile unsigned int *)(MAC_BASE + 0x048))    
#define rMACFIFOCFG1			*((volatile unsigned int *)(MAC_BASE + 0x04c))
#define rMACFIFOCFG2			*((volatile unsigned int *)(MAC_BASE + 0x050))
#define rMACFIFOCFG3			*((volatile unsigned int *)(MAC_BASE + 0x054))
#define rMACFIFOCFG4			*((volatile unsigned int *)(MAC_BASE + 0x058))
#define rMACFIFOCFG5			*((volatile unsigned int *)(MAC_BASE + 0x05c))
#define rMACFIFORAM0			*((volatile unsigned int *)(MAC_BASE + 0x060))
#define rMACFIFORAM1			*((volatile unsigned int *)(MAC_BASE + 0x064))
#define rMACFIFORAM2			*((volatile unsigned int *)(MAC_BASE + 0x068))
#define rMACFIFORAM3			*((volatile unsigned int *)(MAC_BASE + 0x06c))
#define rMACFIFORAM4			*((volatile unsigned int *)(MAC_BASE + 0x070))
#define rMACFIFORAM5			*((volatile unsigned int *)(MAC_BASE + 0x074))
#define rMACFIFORAM6			*((volatile unsigned int *)(MAC_BASE + 0x078))
#define rMACFIFORAM7			*((volatile unsigned int *)(MAC_BASE + 0x07c))
#define rDMATxCtrl				*((volatile unsigned int *)(MAC_BASE + 0x180))    
#define rDMATxDescriptor		*((volatile unsigned int *)(MAC_BASE + 0x184))
#define rDMATxStatus			*((volatile unsigned int *)(MAC_BASE + 0x188))
#define rDMARxCtrl				*((volatile unsigned int *)(MAC_BASE + 0x18c))
#define rDMARxDescriptor		*((volatile unsigned int *)(MAC_BASE + 0x190))
#define rDMARxStatus   			*((volatile unsigned int *)(MAC_BASE + 0x194))
#define rDMAIntrMask   			*((volatile unsigned int *)(MAC_BASE + 0x198))
#define rDMAInterrupt   		*((volatile unsigned int *)(MAC_BASE + 0x19c))


/////////////////////////////////////////////////////
/* USB */
#define USB_BASE				(AHB_BASE + 0x20000)  
                            	
#define USB_FADDR      			(*(volatile unsigned char *)(USB_BASE + 0x000))
#define USB_Power      			(*(volatile unsigned char *)(USB_BASE + 0x001)) 
#define USB_IntrTx     			(*(volatile unsigned char *)(USB_BASE + 0x002))
#define USB_IntrRx     			(*(volatile unsigned char *)(USB_BASE + 0x004))
#define USB_IntrTxEn   			(*(volatile unsigned char *)(USB_BASE + 0x006))
#define USB_IntrRxEn   			(*(volatile unsigned char *)(USB_BASE + 0x008))
#define USB_Intr       			(*(volatile unsigned char *)(USB_BASE + 0x00a))
#define USB_IntrEn     			(*(volatile unsigned char *)(USB_BASE + 0x00b))
#define USB_FRAMEL     			(*(volatile unsigned char *)(USB_BASE + 0x00c))
#define USB_FRAMEH     			(*(volatile unsigned char *)(USB_BASE + 0x00d))
#define USB_INDEX      			(*(volatile unsigned char *)(USB_BASE + 0x00e))
#define USB_TESTMODE   			(*(volatile unsigned char *)(USB_BASE + 0x00f))
#define USB_TXMAXPL    			(*(volatile unsigned char *)(USB_BASE + 0x010))
#define USB_TXMAXPH    			(*(volatile unsigned char *)(USB_BASE + 0x011))
#define USB_TXCSRL     			(*(volatile unsigned char *)(USB_BASE + 0x012)) 
#define USB_CSR0L      			(*(volatile unsigned char *)(USB_BASE + 0x012)) 
#define USB_TXCSRH     			(*(volatile unsigned char *)(USB_BASE + 0x013)) 
#define USB_CSR0H      			(*(volatile unsigned char *)(USB_BASE + 0x013)) 
#define USB_RXMAXPL    			(*(volatile unsigned char *)(USB_BASE + 0x014))
#define USB_RXMAXPH    			(*(volatile unsigned char *)(USB_BASE + 0x015))
#define USB_RXCSRL     			(*(volatile unsigned char *)(USB_BASE + 0x016))
#define USB_RXCSRH     			(*(volatile unsigned char *)(USB_BASE + 0x017))
#define USB_CountL     			(*(volatile unsigned char *)(USB_BASE + 0x018))
#define USB_CountH     			(*(volatile unsigned char *)(USB_BASE + 0x019))
#define USB_TxType     			(*(volatile unsigned char *)(USB_BASE + 0x01a))
#define USB_TxInterval 			(*(volatile unsigned char *)(USB_BASE + 0x01b))
#define USB_NAKLIMIT0  			(*(volatile unsigned char *)(USB_BASE + 0x01b))
                       			                                 
#define USB_RxType     			(*(volatile unsigned char *)(USB_BASE + 0x01c))
#define USB_RxInterval 			(*(volatile unsigned char *)(USB_BASE + 0x01d))
                       			                                  
#define USB_ENDP0_FIFO 			(*(volatile unsigned char *)(USB_BASE + 0x020))         
#define USB_ENDP1_FIFO 			(*(volatile unsigned      *)(USB_BASE + 0x024))
#define USB_ENDP2_FIFO 			(*(volatile unsigned      *)(USB_BASE + 0x028))
#define USB_ENDP3_FIFO 			(*(volatile unsigned      *)(USB_BASE + 0x02c))
#define USB_DEVCTL     			(*(volatile unsigned short*)(USB_BASE + 0x060))
                            	
#define USB_TXFIFOSIZE      	(*(volatile unsigned char *)(USB_BASE + 0x062))
#define USB_RXFIFOSIZE      	(*(volatile unsigned char *)(USB_BASE + 0x063))
#define USB_TXFIFOADDR      	(*(volatile unsigned short *)(USB_BASE + 0x064))
#define USB_RXFIFOADDR      	(*(volatile unsigned short *)(USB_BASE + 0x066))
                       			                                 
#define USB_DMAINT     			(*(volatile unsigned      *)(USB_BASE + 0x200))
                       			                                  
#define USB_DMA1CTL    			(*(volatile unsigned      *)(USB_BASE + 0x204))
#define USB_DMA1ADDR   			(*(volatile unsigned      *)(USB_BASE + 0x208))
#define USB_DMA1CNT    			(*(volatile unsigned      *)(USB_BASE + 0x20c))
                       			                                  
#define USB_DMA2CTL    			(*(volatile unsigned      *)(USB_BASE + 0x214))
#define USB_DMA2ADDR   			(*(volatile unsigned      *)(USB_BASE + 0x218))
#define USB_DMA2CNT    			(*(volatile unsigned      *)(USB_BASE + 0x21c))
                       			                                  
#define USB_DMA3CTL    			(*(volatile unsigned      *)(USB_BASE + 0x234))
#define USB_DMA3ADDR   			(*(volatile unsigned      *)(USB_BASE + 0x238))
#define USB_DMA3CNT    			(*(volatile unsigned      *)(USB_BASE + 0x23c))
                       			                                  
#define USB_DMA4CTL    			(*(volatile unsigned      *)(USB_BASE + 0x244))
#define USB_DMA4ADDR   			(*(volatile unsigned      *)(USB_BASE + 0x248))
#define USB_DMA4CNT    			(*(volatile unsigned      *)(USB_BASE + 0x24c))

#if 0
#define  ECC_BASE            (AHB_BASE + 0x50000) 
#define  AES_BASE   		     (ECC_BASE + 0x1000)
#define  BCH_PARAM0         *(volatile unsigned int *)(ECC_BASE+0x00*4)
#define  BCH_PARAM1         *(volatile unsigned int *)(ECC_BASE+0x01*4)
#define  BCH_PARAM2         *(volatile unsigned int *)(ECC_BASE+0x02*4)
#define  BCH_PARAM3         *(volatile unsigned int *)(ECC_BASE+0x03*4)
#define  BCH_PARAM4         *(volatile unsigned int *)(ECC_BASE+0x04*4)
#define  BCH_PARAM5         *(volatile unsigned int *)(ECC_BASE+0x05*4)
#define  BCH_PARAM6         *(volatile unsigned int *)(ECC_BASE+0x06*4)
#define  BCH_PARAM7         *(volatile unsigned int *)(ECC_BASE+0x07*4)
#define  BCH_PARAM8         *(volatile unsigned int *)(ECC_BASE+0x08*4)
#define  BCH_CLR            *(volatile unsigned int *)(ECC_BASE+0x20*4)
#define  BCH_STATUS         *(volatile unsigned int *)(ECC_BASE+0x21*4)

#define  ECC_RAM0_ADDR(x)   *(volatile unsigned int *)(ECC_BASE+ 1*0x1000 + x )
#define  ECC_RAM1_ADDR(x)   *(volatile unsigned int *)(ECC_BASE+ 2*0x1000 + x )
#define  ECC_RAM2_ADDR(x)   *(volatile unsigned int *)(ECC_BASE+ 3*0x1000 + x )
#define  ECC_RAM3_ADDR(x)   *(volatile unsigned int *)(ECC_BASE+ 4*0x1000 + x )
#define  ECC_RAM4_ADDR(x)   *(volatile unsigned int *)(ECC_BASE+ 5*0x1000 + x )
#define  ECC_RAM5_ADDR(x)   *(volatile unsigned int *)(ECC_BASE+ 6*0x1000 + x )
#define  ECC_RAM6_ADDR(x)   *(volatile unsigned int *)(ECC_BASE+ 7*0x1000 + x )
#define  ECC_RAM7_ADDR(x)   *(volatile unsigned int *)(ECC_BASE+ 8*0x1000 + x )
#define  ECC_RAM8_ADDR(x)   *(volatile unsigned int *)(ECC_BASE+ 9*0x1000 + x )

#define AES_CONFIG				*((volatile unsigned int *)(AES_BASE + 0x00))
#define AES_CONTRL				*((volatile unsigned int *)(AES_BASE + 0x04))
#define AES_STATUS				*((volatile unsigned int *)(AES_BASE + 0x08))
#define AES_DATA0_IN			*((volatile unsigned int *)(AES_BASE + 0x0c))
#define AES_DATA1_IN			*((volatile unsigned int *)(AES_BASE + 0x10))
#define AES_DATA2_IN			*((volatile unsigned int *)(AES_BASE + 0x14))
#define AES_DATA3_IN			*((volatile unsigned int *)(AES_BASE + 0x18))
#define AES_KEY0_IN				*((volatile unsigned int *)(AES_BASE + 0x1c))
#define AES_KEY1_IN				*((volatile unsigned int *)(AES_BASE + 0x20))
#define AES_KEY2_IN				*((volatile unsigned int *)(AES_BASE + 0x24))
#define AES_KEY3_IN				*((volatile unsigned int *)(AES_BASE + 0x28))
#define AES_KEY4_IN				*((volatile unsigned int *)(AES_BASE + 0x2c))
#define AES_KEY5_IN				*((volatile unsigned int *)(AES_BASE + 0x30))
#define AES_KEY6_IN				*((volatile unsigned int *)(AES_BASE + 0x34))
#define AES_KEY7_IN				*((volatile unsigned int *)(AES_BASE + 0x38))
#define AES_IV0_IN				*((volatile unsigned int *)(AES_BASE + 0x3c))
#define AES_IV1_IN				*((volatile unsigned int *)(AES_BASE + 0x40))
#define AES_IV2_IN				*((volatile unsigned int *)(AES_BASE + 0x44))
#define AES_IV3_IN				*((volatile unsigned int *)(AES_BASE + 0x48))
#define AES_DATA0_OUT			*((volatile unsigned int *)(AES_BASE + 0x4c))
#define AES_DATA1_OUT			*((volatile unsigned int *)(AES_BASE + 0x50))
#define AES_DATA2_OUT			*((volatile unsigned int *)(AES_BASE + 0x54))
#define AES_DATA3_OUT			*((volatile unsigned int *)(AES_BASE + 0x58))
#endif
/////////////////////////////////////////////////////
/* SDRAM register */
#define SDRAM_BASE				(AHB_BASE + 0xc0000)

#define rSCONR					*((volatile unsigned int *)(SDRAM_BASE + 0x00))
#define rSTMG0R					*((volatile unsigned int *)(SDRAM_BASE + 0x04))
#define rSTMG1R					*((volatile unsigned int *)(SDRAM_BASE + 0x08))
#define rSCTLR					*((volatile unsigned int *)(SDRAM_BASE + 0x0c))
#define rSREFR					*((volatile unsigned int *)(SDRAM_BASE + 0x10))
#define rSMSKR0					*((volatile unsigned int *)(SDRAM_BASE + 0x54))
#define rSMSKR3					*((volatile unsigned int *)(SDRAM_BASE + 0x60))
#define rSMCTLR					*((volatile unsigned int *)(SDRAM_BASE + 0xa4))
#define rFTiming				*((volatile unsigned int *)(SDRAM_BASE + 0x98))
#define rRegSelFlag				*((volatile unsigned int *)(SDRAM_BASE + 0xffc))


#define LX28XX_VIDA_BASE		0xf0080000
#define LX28XX_VIDB_BASE		0xf0080000
#define LX28XX_VIDC_BASE		0xf0080000
#define LX28XX_VIDD_BASE		0xf0080000

#define ITU_A_BASE              0xf0080000
#define ITU_A_ENABLE              (*(volatile unsigned *)(ITU_A_BASE + 0x00))
#define ITU_A_CTL                 (*(volatile unsigned *)(ITU_A_BASE + 0x04))
#define ITU_A_ADDR01              (*(volatile unsigned *)(ITU_A_BASE + 0x08))
#define ITU_A_ADDR02              (*(volatile unsigned *)(ITU_A_BASE + 0x0c))
#define ITU_A_ADDR03              (*(volatile unsigned *)(ITU_A_BASE + 0x10))
#define ITU_A_ADDR10              (*(volatile unsigned *)(ITU_A_BASE + 0x14))
#define ITU_A_ADDR11              (*(volatile unsigned *)(ITU_A_BASE + 0x18))
#define ITU_A_ADDR12              (*(volatile unsigned *)(ITU_A_BASE + 0x1c))
#define ITU_A_INT_CTL             (*(volatile unsigned *)(ITU_A_BASE + 0x20))
#define ITU_A_STATUS              (*(volatile unsigned *)(ITU_A_BASE + 0x24))
#define ITU_A_INPUT_SIZE          (*(volatile unsigned *)(ITU_A_BASE + 0x28))
#define ITU_A_INPUT_WIDE_OFFSET   (*(volatile unsigned *)(ITU_A_BASE + 0x2c))
#define ITU_A_STORE_CTL1          (*(volatile unsigned *)(ITU_A_BASE + 0x30))
#define ITU_A_GP_CTL              (*(volatile unsigned *)(ITU_A_BASE + 0x34))
#define ITU_A_DVB_PACK_CNT        (*(volatile unsigned *)(ITU_A_BASE + 0x38))
#define ITU_A_H_CROP              (*(volatile unsigned *)(ITU_A_BASE + 0x4C))
#define ITU_A_V_CROP              (*(volatile unsigned *)(ITU_A_BASE + 0x50))
#define ITU_A_DARK_R              (*(volatile unsigned *)(ITU_A_BASE + 0x60))
#define ITU_A_DARK_G              (*(volatile unsigned *)(ITU_A_BASE + 0x64))
#define ITU_A_DARK_B              (*(volatile unsigned *)(ITU_A_BASE + 0x68))
#define ITU_A_AWB_R               (*(volatile unsigned *)(ITU_A_BASE + 0x6C))
#define ITU_A_AWB_G               (*(volatile unsigned *)(ITU_A_BASE + 0x70))
#define ITU_A_AWB_B               (*(volatile unsigned *)(ITU_A_BASE + 0x74))
#define ITU_A_FRAME_SEL           (*(volatile unsigned *)(ITU_A_BASE + 0x78))
#define ITU_A_PID_CMP0            (*(volatile unsigned *)(ITU_A_BASE + 0x7C))
#define ITU_A_PID_CMP1            (*(volatile unsigned *)(ITU_A_BASE + 0x80))
#define ITU_A_PID_CMP2            (*(volatile unsigned *)(ITU_A_BASE + 0x84))
#define ITU_A_PID_CMP3            (*(volatile unsigned *)(ITU_A_BASE + 0x88))
#define ITU_A_PID_CMP4            (*(volatile unsigned *)(ITU_A_BASE + 0x8C))
#define ITU_A_PID_CMP5            (*(volatile unsigned *)(ITU_A_BASE + 0x90))
#define ITU_A_PID_FLAG            (*(volatile unsigned *)(ITU_A_BASE + 0x94))
#define ITU_A_DVB_SIZE            (*(volatile unsigned *)(ITU_A_BASE + 0x98))
#define ITU_A_DMA_OUT_STEP        (*(volatile unsigned *)(ITU_A_BASE + 0x9C))                                                                
#define ITU_A_SOFT_VALUE          (*(volatile unsigned *)(ITU_A_BASE + 0xf0))
#define ITU_A_FINISH              (*(volatile unsigned *)(ITU_A_BASE + 0xfc))

#define ITU_A_GAMMA_RAM(addr)        (*(volatile unsigned *)(ITU_A_BASE+2*1024 + addr*4))



/////////////////////////////////////////////////////
/* NAND Flash */
#define NAND_BASE							(LX28XX_VA_NAND)
                        					
#define NAND_REG_A							(NAND_BASE + 0x00)
#define NAND_CLE_A							(NAND_BASE + 0x04)
#define NAND_ALE_A							(NAND_BASE + 0x08)
#define NFC_TRANS_ADDR						(NAND_BASE + 0x10)
#define NFC_CR                              *((volatile unsigned int  *) (NAND_BASE + 0x00))
#define NFC_CLE                             *((volatile unsigned char *) (NAND_BASE + 0x04))
#define NFC_ALE                             *((volatile unsigned char *) (NAND_BASE + 0x08))
#define NFC_CS                              *((volatile unsigned int  *) (NAND_BASE + 0x0c))
#define NFC_Trans                           *((volatile unsigned int  *) (NAND_BASE + 0x10))

#define RS_GLOBAL_CONTROL                   *((volatile unsigned int  *) (NAND_BASE + 0x14 + 0x6c))
#define RS_ENCODE_CONTROL                   *((volatile unsigned int  *) (NAND_BASE + 0x18 + 0x6c))
#define RS_DECODE_CONTROL                   *((volatile unsigned int  *) (NAND_BASE + 0x1c + 0x6c))
#define RS_STATUS                           *((volatile unsigned int  *) (NAND_BASE + 0x20 + 0x6c))
#define RS_RANDOM_DATA_INPUT_COMMAND        *((volatile unsigned int  *) (NAND_BASE + 0x24 + 0x6c))
#define RS_RANDOM_DATA_OUTPUT_COMMAND1      *((volatile unsigned int  *) (NAND_BASE + 0x28 + 0x6c))
#define RS_RANDOM_DATA_OUTPUT_COMMAND2      *((volatile unsigned int  *) (NAND_BASE + 0x2c + 0x6c))
#define RS_COLUMN_ADDR_1_1_REG              *((volatile unsigned int  *) (NAND_BASE + 0x30 + 0x6c))
#define RS_COLUMN_ADDR_1_2_REG              *((volatile unsigned int  *) (NAND_BASE + 0x34 + 0x6c))
#define RS_COLUMN_ADDR_2_1_REG              *((volatile unsigned int  *) (NAND_BASE + 0x38 + 0x6c))
#define RS_COLUMN_ADDR_2_2_REG              *((volatile unsigned int  *) (NAND_BASE + 0x3c + 0x6c))
#define RS_COLUMN_ADDR_3_1_REG              *((volatile unsigned int  *) (NAND_BASE + 0x40 + 0x6c))
#define RS_COLUMN_ADDR_3_2_REG              *((volatile unsigned int  *) (NAND_BASE + 0x44 + 0x6c))
#define RS_COLUMN_ADDR_4_1_REG              *((volatile unsigned int  *) (NAND_BASE + 0x48 + 0x6c))
#define RS_COLUMN_ADDR_4_2_REG              *((volatile unsigned int  *) (NAND_BASE + 0x4c + 0x6c))
#define RS_PAGE_PROGRAM_COMMAND1            *((volatile unsigned int  *) (NAND_BASE + 0x50 + 0x6c))
#define RS_PAGE_PROGRAM_COMMAND2            *((volatile unsigned int  *) (NAND_BASE + 0x54 + 0x6c))
#define RS_CACHE_PROGRAM_COMMAND1           *((volatile unsigned int  *) (NAND_BASE + 0x58 + 0x6c))
#define RS_CACHE_PROGRAM_COMMAND2           *((volatile unsigned int  *) (NAND_BASE + 0x5c + 0x6c))
#define RS_READ_COMMAND2                    *((volatile unsigned int  *) (NAND_BASE + 0x60 + 0x6c))


 

/////////////////////////////////////////////////////
/* System controller */
#define APB_SYS_BASE   ( APB0_BASE  +  (0<<16)) 
#define SYS_CFG_0				        *(volatile unsigned int *)(APB_SYS_BASE+0x0000*4)
#define SYS_CFG_1                       *(volatile unsigned int *)(APB_SYS_BASE+0x0001*4)
#define SYS_CFG_2 		                *(volatile unsigned int *)(APB_SYS_BASE+0x0002*4)
#define SYS_CFG_3 		                *(volatile unsigned int *)(APB_SYS_BASE+0x0003*4)
#define SDMMC_REQ 		                *(volatile unsigned int *)(APB_SYS_BASE+0x0004*4)
#define ahb_reset_cfg			        *(volatile unsigned int *)(APB_SYS_BASE+0x0005*4)
#define apb_reset_cfg		            *(volatile unsigned int *)(APB_SYS_BASE+0x0006*4)
#define cpu_hclk_sel  			        *(volatile unsigned int *)(APB_SYS_BASE+0x0007*4)
#define cpu_xclk_sel    		        *(volatile unsigned int *)(APB_SYS_BASE+0x0008*4)
#define tvout_lcd_clk_cfg		        *(volatile unsigned int *)(APB_SYS_BASE+0x0009*4)
#define lcd_clk_cfg				        *(volatile unsigned int *)(APB_SYS_BASE+0x000A*4)
#define mac_clk_cfg  	                *(volatile unsigned int *)(APB_SYS_BASE+0x000B*4)
#define audio_adc_nco_cfg               *(volatile unsigned int *)(APB_SYS_BASE+0x000C*4)
#define audio_dac_nco_cfg   	        *(volatile unsigned int *)(APB_SYS_BASE+0x000D*4)
#define fclk1_cfg           	        *(volatile unsigned int *)(APB_SYS_BASE+0x000E*4)
#define fclk2_cfg           	        *(volatile unsigned int *)(APB_SYS_BASE+0x000F*4)
#define fclk3_cfg           	        *(volatile unsigned int *)(APB_SYS_BASE+0x0010*4)
#define SspCfgCount         	        *(volatile unsigned int *)(APB_SYS_BASE+0x0011*4)
#define SdmmcCfgCount       	        *(volatile unsigned int *)(APB_SYS_BASE+0x0012*4)
#define SdioCfgCount        	        *(volatile unsigned int *)(APB_SYS_BASE+0x0013*4)
#define I2sCfgCount         	        *(volatile unsigned int *)(APB_SYS_BASE+0x0014*4)
#define rgb_dac_clk_cfg     	        *(volatile unsigned int *)(APB_SYS_BASE+0x0015*4)
#define ahbclkenable        	        *(volatile unsigned int *)(APB_SYS_BASE+0x0016*4)
#define apbclkenable        	        *(volatile unsigned int *)(APB_SYS_BASE+0x0017*4)
#define pll0_cfg            	        *(volatile unsigned int *)(APB_SYS_BASE+0x0018*4)
#define pll1_cfg            	        *(volatile unsigned int *)(APB_SYS_BASE+0x0019*4)
#define pll2_cfg            	        *(volatile unsigned int *)(APB_SYS_BASE+0x001A*4)
#define sdmmc_clk_delay_cfg 	        *(volatile unsigned int *)(APB_SYS_BASE+0x001B*4)
#define sdio_clk_delay_cfg  	        *(volatile unsigned int *)(APB_SYS_BASE+0x001C*4)
#define pci_clk_cfg         	        *(volatile unsigned int *)(APB_SYS_BASE+0x0021*4)
#define rtc_clksel	                    *(volatile unsigned int *)(APB_SYS_BASE+0x0022*4)
#define ext_param_reg0    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0023*4)
#define ext_param_reg1    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0024*4)
#define ext_param_reg2    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0025*4)
#define ext_param_reg3    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0026*4)
#define ext_param_reg4    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0027*4)
#define ext_param_reg5    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0028*4)
#define ext_param_reg6    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0029*4)
#define ext_param_reg7    	            *(volatile unsigned int *)(APB_SYS_BASE+0x002A*4)
#define ext_param_reg8    	            *(volatile unsigned int *)(APB_SYS_BASE+0x002B*4)
#define ext_param_reg9    	            *(volatile unsigned int *)(APB_SYS_BASE+0x002C*4)
#define ext_param_reg10   	            *(volatile unsigned int *)(APB_SYS_BASE+0x002D*4)
#define ext_param_reg11   	            *(volatile unsigned int *)(APB_SYS_BASE+0x002E*4)
#define ext_param_reg12   	            *(volatile unsigned int *)(APB_SYS_BASE+0x002F*4)
#define ext_param_reg13   	            *(volatile unsigned int *)(APB_SYS_BASE+0x0030*4)
#define ext_param_reg14   	            *(volatile unsigned int *)(APB_SYS_BASE+0x0031*4)
#define ext_param_reg15   	            *(volatile unsigned int *)(APB_SYS_BASE+0x0032*4)
#define ext_param_reg16   	            *(volatile unsigned int *)(APB_SYS_BASE+0x0033*4)
#define ext_param_reg17   	            *(volatile unsigned int *)(APB_SYS_BASE+0x0034*4)
#define ext_param_reg18   	            *(volatile unsigned int *)(APB_SYS_BASE+0x0035*4)
#define ext_param_reg19   	            *(volatile unsigned int *)(APB_SYS_BASE+0x0036*4)
#define PadPull0          	            *(volatile unsigned int *)(APB_SYS_BASE+0x0037*4)
#define PadPull1          	            *(volatile unsigned int *)(APB_SYS_BASE+0x0038*4)
#define PadPull2          	            *(volatile unsigned int *)(APB_SYS_BASE+0x0039*4)
#define PadPull3          	            *(volatile unsigned int *)(APB_SYS_BASE+0x003A*4)
#define PadPull4          	            *(volatile unsigned int *)(APB_SYS_BASE+0x003B*4)
#define PadPull5          	            *(volatile unsigned int *)(APB_SYS_BASE+0x003C*4)
#define PAD0CTL      	            *(volatile unsigned int *)(APB_SYS_BASE+0x003D*4)
#define PAD1CTL      	            *(volatile unsigned int *)(APB_SYS_BASE+0x003E*4)
#define PAD2CTL      	            *(volatile unsigned int *)(APB_SYS_BASE+0x003F*4)
#define PAD3CTL      	            *(volatile unsigned int *)(APB_SYS_BASE+0x0040*4)
#define PAD4CTL      	            *(volatile unsigned int *)(APB_SYS_BASE+0x0041*4)
#define PAD5CTL      	            *(volatile unsigned int *)(APB_SYS_BASE+0x0042*4)
#define PAD6CTL      	            *(volatile unsigned int *)(APB_SYS_BASE+0x0043*4)
#define PAD7CTL      	            *(volatile unsigned int *)(APB_SYS_BASE+0x0044*4)
#define PAD8CTL      	            *(volatile unsigned int *)(APB_SYS_BASE+0x0045*4)
#define PAD9CTL      	            *(volatile unsigned int *)(APB_SYS_BASE+0x0046*4)
#define PAD10CTL     	            *(volatile unsigned int *)(APB_SYS_BASE+0x0047*4)
#define PAD11CTL     	            *(volatile unsigned int *)(APB_SYS_BASE+0x0048*4)
#define PADMODE0     	            *(volatile unsigned int *)(APB_SYS_BASE+0x0049*4)
#define PADMODE1     	            *(volatile unsigned int *)(APB_SYS_BASE+0x004A*4)
#define PADMODE2     	            *(volatile unsigned int *)(APB_SYS_BASE+0x004B*4)
#define PADMODE3     	            *(volatile unsigned int *)(APB_SYS_BASE+0x004C*4)
#define PADMODE4     	            *(volatile unsigned int *)(APB_SYS_BASE+0x004D*4)
#define PADMODE5     	            *(volatile unsigned int *)(APB_SYS_BASE+0x004E*4)
#define PADMODE6     	            *(volatile unsigned int *)(APB_SYS_BASE+0x004F*4)
#define PADMODE7     	            *(volatile unsigned int *)(APB_SYS_BASE+0x0050*4)
#define PADMODE8     	            *(volatile unsigned int *)(APB_SYS_BASE+0x0051*4)
#define PADMODE9     	            *(volatile unsigned int *)(APB_SYS_BASE+0x0052*4)
#define PADMODE10    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0053*4)
#define PADMODE11    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0054*4)
#define PADMODE12    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0055*4)
#define PADMODE13    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0056*4)
#define PADMODE14    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0057*4)
#define PADMODE15    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0058*4)
#define PADMODE16    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0059*4)
#define PADMODE17    	            *(volatile unsigned int *)(APB_SYS_BASE+0x005A*4)
#define PADMODE18    	            *(volatile unsigned int *)(APB_SYS_BASE+0x005B*4)
#define PADMODE19    	            *(volatile unsigned int *)(APB_SYS_BASE+0x005C*4)
#define PADMODE20    	            *(volatile unsigned int *)(APB_SYS_BASE+0x005D*4)
#define PADMODE21    	            *(volatile unsigned int *)(APB_SYS_BASE+0x005E*4)
#define PADMODE22    	            *(volatile unsigned int *)(APB_SYS_BASE+0x005F*4)
#define PADMODE23    	            *(volatile unsigned int *)(APB_SYS_BASE+0x0060*4) 
#define BOOTSEL                         *(volatile unsigned int *)(APB_SYS_BASE+0x0080*4) 

#define WDT_BASE   ( APB0_BASE  +  (1<<16)) 
#define LX28XX_WDT_BASE	WDT_BASE

/////////////////////////////////////////////////////
/* APB ICU register */                                                       
#define ICU_BASE		( APB0_BASE  +  (2<<16)) 
#if 0
#define rICSET			*((volatile unsigned int *)(ICU_BASE + 0x00))
#define rICPEND			*((volatile unsigned int *)(ICU_BASE + 0x04))
#define rICMODE			*((volatile unsigned int *)(ICU_BASE + 0x08))
#define rICMASK			*((volatile unsigned int *)(ICU_BASE + 0x0c))
#define rICLEVEL		*((volatile unsigned int *)(ICU_BASE + 0x10))
#define rIRQISPR		*((volatile unsigned int *)(ICU_BASE + 0x3c))
#define rIRQISPC		*((volatile unsigned int *)(ICU_BASE + 0x40))
#define rIVEC_ADDR      *((volatile unsigned int *)(ICU_BASE + 0x78))

//#define DisableIntNum(num)	do { rICMASK |= (1 << num); }while(0)
//#define EnableIntNum(num)	do { rICMASK &= (~(1 << num)); }while(0) 

#endif
#define rICPRI_MAP_0            *((volatile unsigned int *)(ICU_BASE + 0*4))
#define rICPRI_MAP_1            *((volatile unsigned int *)(ICU_BASE + 1*4))
#define rICPRI_MAP_2            *((volatile unsigned int *)(ICU_BASE + 2*4))
#define rICPRI_MAP_3            *((volatile unsigned int *)(ICU_BASE + 3*4))
#define rICPRI_MAP_4            *((volatile unsigned int *)(ICU_BASE + 4*4))
#define rICPRI_MAP_5            *((volatile unsigned int *)(ICU_BASE + 5*4))
#define rICPRI_MAP_6            *((volatile unsigned int *)(ICU_BASE + 6*4))
#define rICPRI_MAP_7            *((volatile unsigned int *)(ICU_BASE + 7*4))
#define rICPRI_MAP_8            *((volatile unsigned int *)(ICU_BASE + 8*4))
#define rICPRI_MAP_9            *((volatile unsigned int *)(ICU_BASE + 9*4))
#define rICPRI_MAP_10           *((volatile unsigned int *)(ICU_BASE + 10*4))
#define rICPRI_MAP_11           *((volatile unsigned int *)(ICU_BASE + 11*4))
#define rICPRI_MAP_12           *((volatile unsigned int *)(ICU_BASE + 12*4))
#define rICPRI_MAP_13           *((volatile unsigned int *)(ICU_BASE + 13*4))
#define rICPRI_MAP_14           *((volatile unsigned int *)(ICU_BASE + 14*4))
#define rICPRI_MAP_15           *((volatile unsigned int *)(ICU_BASE + 15*4))
#define rICPRI_MAP_16           *((volatile unsigned int *)(ICU_BASE + 16*4))
#define rICPRI_MAP_17           *((volatile unsigned int *)(ICU_BASE + 17*4))
#define rICPRI_MAP_18           *((volatile unsigned int *)(ICU_BASE + 18*4))
#define rICPRI_MAP_19           *((volatile unsigned int *)(ICU_BASE + 19*4))
#define rICPRI_MAP_20           *((volatile unsigned int *)(ICU_BASE + 20*4))
#define rICPRI_MAP_21           *((volatile unsigned int *)(ICU_BASE + 21*4))
#define rICPRI_MAP_22           *((volatile unsigned int *)(ICU_BASE + 22*4))
#define rICPRI_MAP_23           *((volatile unsigned int *)(ICU_BASE + 23*4))
#define rICPRI_MAP_24           *((volatile unsigned int *)(ICU_BASE + 24*4))
#define rICPRI_MAP_25           *((volatile unsigned int *)(ICU_BASE + 25*4))
#define rICPRI_MAP_26           *((volatile unsigned int *)(ICU_BASE + 26*4))
#define rICPRI_MAP_27           *((volatile unsigned int *)(ICU_BASE + 27*4))
#define rICPRI_MAP_28           *((volatile unsigned int *)(ICU_BASE + 28*4))
#define rICPRI_MAP_29           *((volatile unsigned int *)(ICU_BASE + 29*4))
#define rICPRI_MAP_30           *((volatile unsigned int *)(ICU_BASE + 30*4))
#define rICPRI_MAP_31           *((volatile unsigned int *)(ICU_BASE + 31*4))
#define rICPRI_MAP_32           *((volatile unsigned int *)(ICU_BASE + 32*4))
#define rICPRI_MAP_33           *((volatile unsigned int *)(ICU_BASE + 33*4))
#define rICPRI_MAP_34           *((volatile unsigned int *)(ICU_BASE + 34*4))
#define rICPRI_MAP_35           *((volatile unsigned int *)(ICU_BASE + 35*4))
#define rICPRI_MAP_36           *((volatile unsigned int *)(ICU_BASE + 36*4))
#define rICPRI_MAP_37           *((volatile unsigned int *)(ICU_BASE + 37*4))
#define rICPRI_MAP_38           *((volatile unsigned int *)(ICU_BASE + 38*4))
#define rICPRI_MAP_39           *((volatile unsigned int *)(ICU_BASE + 39*4))
#define rICPRI_MAP_40           *((volatile unsigned int *)(ICU_BASE + 40*4))
#define rICPRI_MAP_41           *((volatile unsigned int *)(ICU_BASE + 41*4))
#define rICPRI_MAP_42           *((volatile unsigned int *)(ICU_BASE + 42*4))
#define rICPRI_MAP_43           *((volatile unsigned int *)(ICU_BASE + 43*4))
#define rICPRI_MAP_44           *((volatile unsigned int *)(ICU_BASE + 44*4))
#define rICPRI_MAP_45           *((volatile unsigned int *)(ICU_BASE + 45*4))
#define rICPRI_MAP_46           *((volatile unsigned int *)(ICU_BASE + 46*4))
#define rICPRI_MAP_47           *((volatile unsigned int *)(ICU_BASE + 47*4))
#define rICPRI_MAP_48           *((volatile unsigned int *)(ICU_BASE + 48*4))
#define rICPRI_MAP_49           *((volatile unsigned int *)(ICU_BASE + 49*4))
#define rICPRI_MAP_50           *((volatile unsigned int *)(ICU_BASE + 50*4))
#define rICPRI_MAP_51           *((volatile unsigned int *)(ICU_BASE + 51*4))
#define rICPRI_MAP_52           *((volatile unsigned int *)(ICU_BASE + 52*4))
#define rICPRI_MAP_53           *((volatile unsigned int *)(ICU_BASE + 53*4))
#define rICPRI_MAP_54           *((volatile unsigned int *)(ICU_BASE + 54*4))
#define rICPRI_MAP_55           *((volatile unsigned int *)(ICU_BASE + 55*4))
#define rICPRI_MAP_56           *((volatile unsigned int *)(ICU_BASE + 56*4))
#define rICPRI_MAP_57           *((volatile unsigned int *)(ICU_BASE + 57*4))
#define rICPRI_MAP_58           *((volatile unsigned int *)(ICU_BASE + 58*4))
#define rICPRI_MAP_59           *((volatile unsigned int *)(ICU_BASE + 59*4))
#define rICPRI_MAP_60           *((volatile unsigned int *)(ICU_BASE + 60*4))
#define rICPRI_MAP_61           *((volatile unsigned int *)(ICU_BASE + 61*4))
#define rICPRI_MAP_62           *((volatile unsigned int *)(ICU_BASE + 62*4))
#define rICPRI_MAP_63           *((volatile unsigned int *)(ICU_BASE + 63*4))
#define rICPRI_MAP_64           *((volatile unsigned int *)(ICU_BASE + 64*4))
#define rICPRI_MAP_65           *((volatile unsigned int *)(ICU_BASE + 65*4))
#define rICPRI_MAP_66           *((volatile unsigned int *)(ICU_BASE + 66*4))
#define rICPRI_MAP_67           *((volatile unsigned int *)(ICU_BASE + 67*4))
#define rICPRI_MAP_68           *((volatile unsigned int *)(ICU_BASE + 68*4))
#define rICPRI_MAP_69           *((volatile unsigned int *)(ICU_BASE + 69*4))
#define rICPRI_MAP_70           *((volatile unsigned int *)(ICU_BASE + 70*4))
#define rICPRI_MAP_71           *((volatile unsigned int *)(ICU_BASE + 71*4))
#define rICPRI_MAP_72           *((volatile unsigned int *)(ICU_BASE + 72*4))
#define rICPRI_MAP_73           *((volatile unsigned int *)(ICU_BASE + 73*4))
#define rICPRI_MAP_74           *((volatile unsigned int *)(ICU_BASE + 74*4))
#define rICPRI_MAP_75           *((volatile unsigned int *)(ICU_BASE + 75*4))
#define rICPRI_MAP_76           *((volatile unsigned int *)(ICU_BASE + 76*4))
#define rICPRI_MAP_77           *((volatile unsigned int *)(ICU_BASE + 77*4))
#define rICPRI_MAP_78           *((volatile unsigned int *)(ICU_BASE + 78*4))
#define rICPRI_MAP_79           *((volatile unsigned int *)(ICU_BASE + 79*4))
#define rICPRI_MAP_80           *((volatile unsigned int *)(ICU_BASE + 80*4))
#define rICPRI_MAP_81           *((volatile unsigned int *)(ICU_BASE + 81*4))
#define rICPRI_MAP_82           *((volatile unsigned int *)(ICU_BASE + 82*4))
#define rICPRI_MAP_83           *((volatile unsigned int *)(ICU_BASE + 83*4))
#define rICPRI_MAP_84           *((volatile unsigned int *)(ICU_BASE + 84*4))
#define rICPRI_MAP_85           *((volatile unsigned int *)(ICU_BASE + 85*4))
#define rICPRI_MAP_86           *((volatile unsigned int *)(ICU_BASE + 86*4))
#define rICPRI_MAP_87           *((volatile unsigned int *)(ICU_BASE + 87*4))
#define rICPRI_MAP_88           *((volatile unsigned int *)(ICU_BASE + 88*4))
#define rICPRI_MAP_89           *((volatile unsigned int *)(ICU_BASE + 89*4))
#define rICPRI_MAP_90           *((volatile unsigned int *)(ICU_BASE + 90*4))
#define rICPRI_MAP_91           *((volatile unsigned int *)(ICU_BASE + 91*4))
#define rICPRI_MAP_92           *((volatile unsigned int *)(ICU_BASE + 92*4))
#define rICPRI_MAP_93           *((volatile unsigned int *)(ICU_BASE + 93*4))
#define rICPRI_MAP_94           *((volatile unsigned int *)(ICU_BASE + 94*4))
#define rICPRI_MAP_95           *((volatile unsigned int *)(ICU_BASE + 95*4))

#define rICGMASK                *((volatile unsigned int *)(ICU_BASE + 96*4))
#define rICPOL_31_0             *((volatile unsigned int *)(ICU_BASE + 97*4))
#define rICPOL_63_32            *((volatile unsigned int *)(ICU_BASE + 98*4))
#define rICPOL_95_64            *((volatile unsigned int *)(ICU_BASE + 99*4))
#define rICLEVEL_31_0           *((volatile unsigned int *)(ICU_BASE + 100*4))
#define rICLEVEL_63_32          *((volatile unsigned int *)(ICU_BASE + 101*4))
#define rICLEVEL_95_64          *((volatile unsigned int *)(ICU_BASE + 102*4))
#define rICMASK_31_0            *((volatile unsigned int *)(ICU_BASE + 103*4))
#define rICMASK_63_32           *((volatile unsigned int *)(ICU_BASE + 104*4))
#define rICMASK_95_64           *((volatile unsigned int *)(ICU_BASE + 105*4))
#define rICCLRPEND_31_0         *((volatile unsigned int *)(ICU_BASE + 106*4))
#define rICCLRPEND_63_32        *((volatile unsigned int *)(ICU_BASE + 107*4))
#define rICCLRPEND_95_64        *((volatile unsigned int *)(ICU_BASE + 108*4))
#define rICVECTOR               *((volatile unsigned int *)(ICU_BASE + 109*4))
#define rICPEND_31_0            *((volatile unsigned int *)(ICU_BASE + 110*4))
#define rICPEND_63_32           *((volatile unsigned int *)(ICU_BASE + 111*4))
#define rICPEND_95_64           *((volatile unsigned int *)(ICU_BASE + 112*4))

#define EnableGInt \
    do { \
        rICGMASK = 0; \
    } while (0)

#define DisableGInt \
    do { \
        rICGMASK = 1; \
    } while (0)

#define EnableIntNum(num) \
    do { \
        if (num > 63) \
            rICMASK_95_64 &= ~(1 << (num - 64)); \
        else if (num > 31) \
            rICMASK_63_32 &= ~(1 << (num - 32)); \
        else \
            rICMASK_31_0 &= ~(1 << num); \
    } while (0)

#define DisableIntNum(num) \
    do { \
        if (num > 63) \
            rICMASK_95_64 |= 1 << (num - 64); \
        else if (num > 31) \
            rICMASK_63_32 |= 1 << (num - 32); \
        else \
            rICMASK_31_0 |= 1 << num; \
    } while (0)

#define SetIntPolLow(num) \
    do { \
        if (num > 63) \
            rICPOL_95_64 |= 1 << (num - 64); \
        else if (num > 31) \
            rICPOL_63_32 |= 1 << (num - 32); \
        else \
            rICPOL_31_0 |= 1 << num; \
    } while (0)

#define SetIntPolHigh(num) \
    do { \
        if (num > 63) \
            rICPOL_95_64 &= ~(1 << (num - 64)); \
        else if (num > 31) \
            rICPOL_63_32 &= ~(1 << (num - 32)); \
        else \
            rICPOL_31_0 &= ~(1 << num); \
    } while (0)

#define SetIntEdge(num) \
    do { \
        if (num > 63) \
            rICLEVEL_95_64 |= 1 << (num - 64); \
        else if (num > 31) \
            rICLEVEL_63_32 |= 1 << (num - 32); \
        else \
            rICLEVEL_31_0 |= 1 << num; \
    } while (0)

#define SetIntLevel(num) \
    do { \
        if (num > 63) \
            rICLEVEL_95_64 &= ~(1 << (num - 64)); \
        else if (num > 31) \
            rICLEVEL_63_32 &= ~(1 << (num - 32)); \
        else \
            rICLEVEL_31_0 &= ~(1 << num); \
    } while (0)

#define ClearPend(num) \
    do { \
        if (num > 63) { \
            rICCLRPEND_95_64 |= 1 << (num - 64); \
            rICCLRPEND_95_64 &= ~(1 << (num - 64)); \
        } \
        else if (num > 31) { \
            rICCLRPEND_63_32 |= 1 << (num - 32); \
            rICCLRPEND_63_32 &= ~(1 << (num -32)); \
        } \
        else { \
            rICCLRPEND_31_0 |= 1 << num; \
            rICCLRPEND_31_0 &= ~(1 << num); \
        } \
    } while (0)

/////////////////////////////////////////////////////
/* Timer */
#define TIMER_BASE		( APB0_BASE  +  (3<<16)) 
#if 0
#define rTCTL0          *((volatile unsigned int *)(TIMER_BASE + 0x00))
#define rTPRS0          *((volatile unsigned int *)(TIMER_BASE + 0x20))
#define rTMOD0          *((volatile unsigned int *)(TIMER_BASE + 0x40))
#define rTCNT0          *((volatile unsigned int *)(TIMER_BASE + 0x60))
#define rTNPE0          *((volatile unsigned int *)(TIMER_BASE + 0x80))
#define rTNNE0          *((volatile unsigned int *)(TIMER_BASE + 0xa0))

#define rTCTL1          *((volatile unsigned int *)(TIMER_BASE + 0x04))
#define rTPRS1          *((volatile unsigned int *)(TIMER_BASE + 0x24))
#define rTMOD1          *((volatile unsigned int *)(TIMER_BASE + 0x44))
#define rTCNT1          *((volatile unsigned int *)(TIMER_BASE + 0x64))
#define rTNPE1          *((volatile unsigned int *)(TIMER_BASE + 0x84))
#define rTNNE1          *((volatile unsigned int *)(TIMER_BASE + 0xa4))

#define rTCTL2          *((volatile unsigned int *)(TIMER_BASE + 0x08))
#define rTPRS2          *((volatile unsigned int *)(TIMER_BASE + 0x28))
#define rTMOD2          *((volatile unsigned int *)(TIMER_BASE + 0x48))
#define rTCNT2          *((volatile unsigned int *)(TIMER_BASE + 0x68))
#define rTNPE2          *((volatile unsigned int *)(TIMER_BASE + 0x88))
#define rTNNE2          *((volatile unsigned int *)(TIMER_BASE + 0xa8))

#define rTCTL3          *((volatile unsigned int *)(TIMER_BASE + 0x0c))
#define rTPRS3          *((volatile unsigned int *)(TIMER_BASE + 0x2c))
#define rTMOD3          *((volatile unsigned int *)(TIMER_BASE + 0x4c))
#define rTCNT3          *((volatile unsigned int *)(TIMER_BASE + 0x6c))
#define rTNPE3          *((volatile unsigned int *)(TIMER_BASE + 0x8c))
#define rTNNE3          *((volatile unsigned int *)(TIMER_BASE + 0xac))

#define rTCTL4          *((volatile unsigned int *)(TIMER_BASE + 0x10))
#define rTPRS4          *((volatile unsigned int *)(TIMER_BASE + 0x30))
#define rTMOD4          *((volatile unsigned int *)(TIMER_BASE + 0x50))
#define rTCNT4          *((volatile unsigned int *)(TIMER_BASE + 0x70))
#define rTNPE4          *((volatile unsigned int *)(TIMER_BASE + 0x90))
#define rTNNE4          *((volatile unsigned int *)(TIMER_BASE + 0xb0))

#define rTCTL5          *((volatile unsigned int *)(TIMER_BASE + 0x14))
#define rTPRS5          *((volatile unsigned int *)(TIMER_BASE + 0x34))
#define rTMOD5          *((volatile unsigned int *)(TIMER_BASE + 0x54))
#define rTCNT5          *((volatile unsigned int *)(TIMER_BASE + 0x74))
#define rTNPE5          *((volatile unsigned int *)(TIMER_BASE + 0x94))
#define rTNNE5          *((volatile unsigned int *)(TIMER_BASE + 0xb4))

#define rTCTL6          *((volatile unsigned int *)(TIMER_BASE + 0x18))
#define rTPRS6          *((volatile unsigned int *)(TIMER_BASE + 0x38))
#define rTMOD6          *((volatile unsigned int *)(TIMER_BASE + 0x58))
#define rTCNT6          *((volatile unsigned int *)(TIMER_BASE + 0x78))
#define rTNPE6          *((volatile unsigned int *)(TIMER_BASE + 0x98))
#define rTNNE6          *((volatile unsigned int *)(TIMER_BASE + 0xb8))

#define rTCTL7          *((volatile unsigned int *)(TIMER_BASE + 0x1c))
#define rTPRS7          *((volatile unsigned int *)(TIMER_BASE + 0x3c))
#define rTMOD7          *((volatile unsigned int *)(TIMER_BASE + 0x5c))
#define rTCNT7          *((volatile unsigned int *)(TIMER_BASE + 0x7c))
#define rTNPE7          *((volatile unsigned int *)(TIMER_BASE + 0x9c))
#define rTNNE7          *((volatile unsigned int *)(TIMER_BASE + 0xbc))

#define rTITEN          *((volatile unsigned int *)(TIMER_BASE + 0xf4))
#define rTITCLR         *((volatile unsigned int *)(TIMER_BASE + 0xf8))
#define rTITSTA         *((volatile unsigned int *)(TIMER_BASE + 0xfc))
#else 
/***********************************************************************/
#define rTCTL0          *((volatile unsigned int *)(TIMER_BASE + 0x000))
#define rTPRS0          *((volatile unsigned int *)(TIMER_BASE + 0x004))
#define rTMOD0          *((volatile unsigned int *)(TIMER_BASE + 0x008))
#define rTCNT0          *((volatile unsigned int *)(TIMER_BASE + 0x00C))
#define rTNPE0          *((volatile unsigned int *)(TIMER_BASE + 0x010))
#define rTNNE0          *((volatile unsigned int *)(TIMER_BASE + 0x014))
#define rTSTART0        *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL1          *((volatile unsigned int *)(TIMER_BASE + 0x020))
#define rTPRS1          *((volatile unsigned int *)(TIMER_BASE + 0x024))
#define rTMOD1          *((volatile unsigned int *)(TIMER_BASE + 0x028))
#define rTCNT1          *((volatile unsigned int *)(TIMER_BASE + 0x02C))
#define rTNPE1          *((volatile unsigned int *)(TIMER_BASE + 0x030))
#define rTNNE1          *((volatile unsigned int *)(TIMER_BASE + 0x034))
#define rTSTART1        *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL2          *((volatile unsigned int *)(TIMER_BASE + 0x040))
#define rTPRS2          *((volatile unsigned int *)(TIMER_BASE + 0x044))
#define rTMOD2          *((volatile unsigned int *)(TIMER_BASE + 0x048))
#define rTCNT2          *((volatile unsigned int *)(TIMER_BASE + 0x04C))
#define rTNPE2          *((volatile unsigned int *)(TIMER_BASE + 0x050))
#define rTNNE2          *((volatile unsigned int *)(TIMER_BASE + 0x054))
#define rTSTART2        *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL3          *((volatile unsigned int *)(TIMER_BASE + 0x060))
#define rTPRS3          *((volatile unsigned int *)(TIMER_BASE + 0x064))
#define rTMOD3          *((volatile unsigned int *)(TIMER_BASE + 0x068))
#define rTCNT3          *((volatile unsigned int *)(TIMER_BASE + 0x06C))
#define rTNPE3          *((volatile unsigned int *)(TIMER_BASE + 0x070))
#define rTNNE3          *((volatile unsigned int *)(TIMER_BASE + 0x074))
#define rTSTART3        *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL4          *((volatile unsigned int *)(TIMER_BASE + 0x080))
#define rTPRS4          *((volatile unsigned int *)(TIMER_BASE + 0x084))
#define rTMOD4          *((volatile unsigned int *)(TIMER_BASE + 0x088))
#define rTCNT4          *((volatile unsigned int *)(TIMER_BASE + 0x08C))
#define rTNPE4          *((volatile unsigned int *)(TIMER_BASE + 0x090))
#define rTNNE4          *((volatile unsigned int *)(TIMER_BASE + 0x094))
#define rTSTART4        *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL5          *((volatile unsigned int *)(TIMER_BASE + 0x0A0))
#define rTPRS5          *((volatile unsigned int *)(TIMER_BASE + 0x0A4))
#define rTMOD5          *((volatile unsigned int *)(TIMER_BASE + 0x0A8))
#define rTCNT5          *((volatile unsigned int *)(TIMER_BASE + 0x0AC))
#define rTNPE5          *((volatile unsigned int *)(TIMER_BASE + 0x0B0))
#define rTNNE5          *((volatile unsigned int *)(TIMER_BASE + 0x0B4))
#define rTSTART5        *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL6          *((volatile unsigned int *)(TIMER_BASE + 0x0C0))
#define rTPRS6          *((volatile unsigned int *)(TIMER_BASE + 0x0C4))
#define rTMOD6          *((volatile unsigned int *)(TIMER_BASE + 0x0C8))
#define rTCNT6          *((volatile unsigned int *)(TIMER_BASE + 0x0CC))
#define rTNPE6          *((volatile unsigned int *)(TIMER_BASE + 0x0D0))
#define rTNNE6          *((volatile unsigned int *)(TIMER_BASE + 0x0D4))
#define rTSTART6        *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL7          *((volatile unsigned int *)(TIMER_BASE + 0x0E0))
#define rTPRS7          *((volatile unsigned int *)(TIMER_BASE + 0x0E4))
#define rTMOD7          *((volatile unsigned int *)(TIMER_BASE + 0x0E8))
#define rTCNT7          *((volatile unsigned int *)(TIMER_BASE + 0x0EC))
#define rTNPE7          *((volatile unsigned int *)(TIMER_BASE + 0x0F0))
#define rTNNE7          *((volatile unsigned int *)(TIMER_BASE + 0x0F4))
#define rTSTART7        *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL8          *((volatile unsigned int *)(TIMER_BASE + 0x100))
#define rTPRS8          *((volatile unsigned int *)(TIMER_BASE + 0x104))
#define rTMOD8          *((volatile unsigned int *)(TIMER_BASE + 0x108))
#define rTCNT8          *((volatile unsigned int *)(TIMER_BASE + 0x10C))
#define rTNPE8          *((volatile unsigned int *)(TIMER_BASE + 0x110))
#define rTNNE8          *((volatile unsigned int *)(TIMER_BASE + 0x114))
#define rTSTART8        *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL9          *((volatile unsigned int *)(TIMER_BASE + 0x120))
#define rTPRS9          *((volatile unsigned int *)(TIMER_BASE + 0x124))
#define rTMOD9          *((volatile unsigned int *)(TIMER_BASE + 0x128))
#define rTCNT9          *((volatile unsigned int *)(TIMER_BASE + 0x12C))
#define rTNPE9          *((volatile unsigned int *)(TIMER_BASE + 0x130))
#define rTNNE9          *((volatile unsigned int *)(TIMER_BASE + 0x134))
#define rTSTART9        *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL10         *((volatile unsigned int *)(TIMER_BASE + 0x140))
#define rTPRS10         *((volatile unsigned int *)(TIMER_BASE + 0x144))
#define rTMOD10         *((volatile unsigned int *)(TIMER_BASE + 0x148))
#define rTCNT10         *((volatile unsigned int *)(TIMER_BASE + 0x14C))
#define rTNPE10         *((volatile unsigned int *)(TIMER_BASE + 0x150))
#define rTNNE10         *((volatile unsigned int *)(TIMER_BASE + 0x154))
#define rTSTART10       *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL11         *((volatile unsigned int *)(TIMER_BASE + 0x160))
#define rTPRS11         *((volatile unsigned int *)(TIMER_BASE + 0x164))
#define rTMOD11         *((volatile unsigned int *)(TIMER_BASE + 0x168))
#define rTCNT11         *((volatile unsigned int *)(TIMER_BASE + 0x16C))
#define rTNPE11         *((volatile unsigned int *)(TIMER_BASE + 0x170))
#define rTNNE11         *((volatile unsigned int *)(TIMER_BASE + 0x174))
#define rTSTART11       *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL12         *((volatile unsigned int *)(TIMER_BASE + 0x180))
#define rTPRS12         *((volatile unsigned int *)(TIMER_BASE + 0x184))
#define rTMOD12         *((volatile unsigned int *)(TIMER_BASE + 0x188))
#define rTCNT12         *((volatile unsigned int *)(TIMER_BASE + 0x18C))
#define rTNPE12         *((volatile unsigned int *)(TIMER_BASE + 0x190))
#define rTNNE12         *((volatile unsigned int *)(TIMER_BASE + 0x194))
#define rTSTART12       *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL13         *((volatile unsigned int *)(TIMER_BASE + 0x1A0))
#define rTPRS13         *((volatile unsigned int *)(TIMER_BASE + 0x1A4))
#define rTMOD13         *((volatile unsigned int *)(TIMER_BASE + 0x1A8))
#define rTCNT13         *((volatile unsigned int *)(TIMER_BASE + 0x1AC))
#define rTNPE13         *((volatile unsigned int *)(TIMER_BASE + 0x1B0))
#define rTNNE13         *((volatile unsigned int *)(TIMER_BASE + 0x1B4))
#define rTSTART13       *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL14         *((volatile unsigned int *)(TIMER_BASE + 0x1C0))
#define rTPRS14         *((volatile unsigned int *)(TIMER_BASE + 0x1C4))
#define rTMOD14         *((volatile unsigned int *)(TIMER_BASE + 0x1C8))
#define rTCNT14         *((volatile unsigned int *)(TIMER_BASE + 0x1CC))
#define rTNPE14         *((volatile unsigned int *)(TIMER_BASE + 0x1D0))
#define rTNNE14         *((volatile unsigned int *)(TIMER_BASE + 0x1D4))
#define rTSTART14       *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rTCTL15         *((volatile unsigned int *)(TIMER_BASE + 0x1E0))
#define rTPRS15         *((volatile unsigned int *)(TIMER_BASE + 0x1E4))
#define rTMOD15         *((volatile unsigned int *)(TIMER_BASE + 0x1E8))
#define rTCNT15         *((volatile unsigned int *)(TIMER_BASE + 0x1EC))
#define rTNPE15         *((volatile unsigned int *)(TIMER_BASE + 0x1F0))
#define rTNNE15         *((volatile unsigned int *)(TIMER_BASE + 0x1F4))
#define rTSTART15       *((volatile unsigned int *)(TIMER_BASE + 0x01C))

#define rGINTREN        *((volatile unsigned int *)(TIMER_BASE + 0x3E4))

#define rTITEN          *((volatile unsigned int *)(TIMER_BASE + 0x3F4))
#define rTITCLR         *((volatile unsigned int *)(TIMER_BASE + 0x3F8))
#define rTITSTA         *((volatile unsigned int *)(TIMER_BASE + 0x3Fc))

#define TMR_ENB_BIT     0x01 
#define TMR_PM_BIT      0x02 
#define TMR_INTEN_BIT   0x04 

#define TMR0_BIT        0x0001 
#define TMR1_BIT        0x0002 
#define TMR2_BIT        0x0004 
#define TMR3_BIT        0x0008 
#define TMR4_BIT        0x0010 
#define TMR5_BIT        0x0020 
#define TMR6_BIT        0x0040 
#define TMR7_BIT        0x0080 
#define TMR8_BIT        0x0100 
#define TMR9_BIT        0x0200 
#define TMR10_BIT       0x0400 
#define TMR11_BIT       0x0800 
#define TMR12_BIT       0x1000 
#define TMR13_BIT       0x2000 
#define TMR14_BIT       0x4000 
#define TMR15_BIT       0x8000 

#define bTINTT0         (unsigned int)(1 << 0)
#define bTINTT1         (unsigned int)(1 << 1)
#define bTINTT2         (unsigned int)(1 << 2)
#define bTINTT3         (unsigned int)(1 << 3)
#define bTINTT4         (unsigned int)(1 << 4)
#define bTINTT5         (unsigned int)(1 << 5)
#define bTINTT6         (unsigned int)(1 << 6)
#define bTINTT7         (unsigned int)(1 << 7)
#define bTINTT8         (unsigned int)(1 << 8)
#define bTINTT9         (unsigned int)(1 << 9)
#define bTINTT10        (unsigned int)(1 << 10)
#define bTINTT11        (unsigned int)(1 << 11)
#define bTINTT12        (unsigned int)(1 << 12)
#define bTINTT13        (unsigned int)(1 << 13)
#define bTINTT14        (unsigned int)(1 << 14)
#define bTINTT15        (unsigned int)(1 << 15)
#define bTINTT0PE       (unsigned int)(1 << 16)
#define bTINTT1PE       (unsigned int)(1 << 17)
#define bTINTT2PE       (unsigned int)(1 << 18)
#define bTINTT3PE       (unsigned int)(1 << 19)
#define bTINTT4PE       (unsigned int)(1 << 20)
#define bTINTT5PE       (unsigned int)(1 << 21)
#define bTINTT6PE       (unsigned int)(1 << 22)
#define bTINTT7PE       (unsigned int)(1 << 23)
#define bTINTT0NE       (unsigned int)(1 << 24)
#define bTINTT1NE       (unsigned int)(1 << 25)
#define bTINTT2NE       (unsigned int)(1 << 26)
#define bTINTT3NE       (unsigned int)(1 << 27)
#define bTINTT4NE       (unsigned int)(1 << 28)
#define bTINTT5NE       (unsigned int)(1 << 29)
#define bTINTT6NE       (unsigned int)(1 << 30)
#define bTINTT7NE       (unsigned int)(1 << 31)

#endif



/////////////////////////////////////////////////////                                
/* RTC */
#define RTC_BASE		( APB0_BASE  +  (4<<16)) 
#define LX28XX_RTC_BASE	RTC_BASE
                        		
#define rRTCCTL	   				*((volatile unsigned int *)(RTC_BASE + 0x00)) /*control register*/           
#define rALMCFG	   				*((volatile unsigned int *)(RTC_BASE + 0x04)) /*alarm config register*/      
#define rALMSEC	   				*((volatile unsigned int *)(RTC_BASE + 0x08)) /*alarm second register*/      
#define rALMMIN	   				*((volatile unsigned int *)(RTC_BASE + 0x0c)) /*alarm minute register*/      
#define rALMHOUR   				*((volatile unsigned int *)(RTC_BASE + 0x10)) /*alarm hour register*/        
#define rALMDATE				*((volatile unsigned int *)(RTC_BASE + 0x14)) /*alarm date register*/        
#define rALMDAY	   				*((volatile unsigned int *)(RTC_BASE + 0x18)) /*alarm day register*/         
#define rALMMON	   				*((volatile unsigned int *)(RTC_BASE + 0x1c)) /*alarm month register*/       
#define rALMYEAR				*((volatile unsigned int *)(RTC_BASE + 0x20)) /*alarm year register*/        
#define rRTCSEC_reg				*((volatile unsigned int *)(RTC_BASE + 0x24)) /*RTC second register*/        
#define rRTCMIN_reg				*((volatile unsigned int *)(RTC_BASE + 0x28)) /*RTC minute register*/        
#define rRTCHOUR_reg			*((volatile unsigned int *)(RTC_BASE + 0x2c)) /*RTC hour register*/          
#define rRTCDATE_reg			*((volatile unsigned int *)(RTC_BASE + 0x30)) /*RTC date register*/          
#define rRTCDAY_reg				*((volatile unsigned int *)(RTC_BASE + 0x34)) /*RTC day register*/           
#define rRTCMON_reg				*((volatile unsigned int *)(RTC_BASE + 0x38)) /*RTC month register*/         
#define rRTCYEAR_reg			*((volatile unsigned int *)(RTC_BASE + 0x3c)) /*RTC year register*/          
#define rRTCIM   				*((volatile unsigned int *)(RTC_BASE + 0x40)) /*RTC interrupt register*/     
#define rRTCSTA  				*((volatile unsigned int *)(RTC_BASE + 0x44)) /*RTC status register*/        
#define rSECCNT  				*((volatile unsigned int *)(RTC_BASE + 0x48)) /*RTC second counter register*/
#define rPWRONI  				*((volatile unsigned int *)(RTC_BASE + 0x4c)) /*power on interrupt register*/



/////////////////////////////////////////////////////
/* GPIO */           
#define GPIO_BASE		( APB0_BASE  +  (5<<16)) 
#if 0                            	
#define GPIO_1_0_DATAOUT    	*((volatile unsigned int *)(GPIO_BASE + 0x00))
#define GPIO_1_0_DATADIR    	*((volatile unsigned int *)(GPIO_BASE + 0x04))
#define GPIO_1_0_DATAIN     	*((volatile unsigned int *)(GPIO_BASE + 0xb0))
                            	
#define GPIO_3_2_DATAOUT    	*((volatile unsigned int *)(GPIO_BASE + 0x0C))
#define GPIO_3_2_DATADIR    	*((volatile unsigned int *)(GPIO_BASE + 0x10))
#define GPIO_3_2_DATAIN     	*((volatile unsigned int *)(GPIO_BASE + 0xb4))
                            	
#define GPIO_5_4_DATAOUT    	*((volatile unsigned int *)(GPIO_BASE + 0x18))
#define GPIO_5_4_DATADIR    	*((volatile unsigned int *)(GPIO_BASE + 0x1C))
#define GPIO_5_4_DATAIN     	*((volatile unsigned int *)(GPIO_BASE + 0xb8))
                            	
#define GPIO_7_6_DATAOUT    	*((volatile unsigned int *)(GPIO_BASE + 0x24))
#define GPIO_7_6_DATADIR    	*((volatile unsigned int *)(GPIO_BASE + 0x28))
#define GPIO_7_6_DATAIN     	*((volatile unsigned int *)(GPIO_BASE + 0xbC))
                            	
#define GPIO_9_8_DATAOUT    	*((volatile unsigned int *)(GPIO_BASE + 0x100))
#define GPIO_9_8_DATADIR    	*((volatile unsigned int *)(GPIO_BASE + 0x104))
#define GPIO_9_8_DATAIN     	*((volatile unsigned int *)(GPIO_BASE + 0x1b0))
                            	
#define GPIO_11_10_DATAOUT  	*((volatile unsigned int *)(GPIO_BASE + 0x10C))
#define GPIO_11_10_DATADIR  	*((volatile unsigned int *)(GPIO_BASE + 0x110))
#define GPIO_11_10_DATAIN   	*((volatile unsigned int *)(GPIO_BASE + 0x1b4))
                            	
#define GPIO_13_12_DATAOUT  	*((volatile unsigned int *)(GPIO_BASE + 0x118))
#define GPIO_13_12_DATADIR  	*((volatile unsigned int *)(GPIO_BASE + 0x11C))
#define GPIO_13_12_DATAIN   	*((volatile unsigned int *)(GPIO_BASE + 0x1b8))
                            	
#define GPIO_15_14_DATAOUT  	*((volatile unsigned int *)(GPIO_BASE + 0x124))
#define GPIO_15_14_DATADIR  	*((volatile unsigned int *)(GPIO_BASE + 0x128))
#define GPIO_15_14_DATAIN   	*((volatile unsigned int *)(GPIO_BASE + 0x1bC))
                            	
#define GPIO_17_16_DATAOUT  	*((volatile unsigned int *)(GPIO_BASE + 0x200))
#define GPIO_17_16_DATADIR  	*((volatile unsigned int *)(GPIO_BASE + 0x204))
#define GPIO_17_16_DATAIN   	*((volatile unsigned int *)(GPIO_BASE + 0x2b0))
                            	
#define GPIO_19_18_DATAOUT  	*((volatile unsigned int *)(GPIO_BASE + 0x20C))
#define GPIO_19_18_DATADIR  	*((volatile unsigned int *)(GPIO_BASE + 0x210))
#define GPIO_19_18_DATAIN   	*((volatile unsigned int *)(GPIO_BASE + 0x2b4))
                            	
#define GPIO_21_20_DATAOUT  	*((volatile unsigned int *)(GPIO_BASE + 0x218))
#define GPIO_21_20_DATADIR  	*((volatile unsigned int *)(GPIO_BASE + 0x21C))
#define GPIO_21_20_DATAIN   	*((volatile unsigned int *)(GPIO_BASE + 0x2b8))
                            	
#define GPIO_23_22_DATAOUT  	*((volatile unsigned int *)(GPIO_BASE + 0x224))
#define GPIO_23_22_DATADIR  	*((volatile unsigned int *)(GPIO_BASE + 0x228))
#define GPIO_23_22_DATAIN   	*((volatile unsigned int *)(GPIO_BASE + 0x2bC))
                            	
#define GPIO_25_24_DATAOUT  	*((volatile unsigned int *)(GPIO_BASE + 0x300))
#define GPIO_25_24_DATADIR  	*((volatile unsigned int *)(GPIO_BASE + 0x304))
#define GPIO_25_24_DATAIN   	*((volatile unsigned int *)(GPIO_BASE + 0x3b0))
#endif

#define  PWM_BASE		( APB0_BASE  +  (6<<16)) 

/////////////////////////////////////////////////////
/* UART */ 
#define UART0_BASE				( APB0_BASE  +  (0x7<<16))   //(0x60070000 ) //

#if  0   //add by lzl 20180528
#define SUART1_BASE				( APB0_BASE  +  (0x8<<16))  
#define SUART2_BASE				( APB0_BASE  +  (0x9<<16))  
#define SUART3_BASE				( APB0_BASE  +  (0x11<<16))  
#define SUART4_BASE				( APB0_BASE  +  (0x12<<16))  
#define SUART5_BASE				( APB0_BASE  +  (0x19<<16))  
#define SUART6_BASE				( APB0_BASE  +  (0x1a<<16))  
#define SUART7_BASE				( APB0_BASE  +  (0x24<<16))  
#define SUART8_BASE				( APB0_BASE  +  (0x25<<16))  
#define SUART9_BASE				( APB0_BASE  +  (0x30<<16))
#else
#define SUART1_BASE				( 0x60000000  +  (0x8<<16))  
#define SUART2_BASE				( 0x60000000  +  (0x9<<16))  
#define SUART3_BASE				( 0x60000000  +  (0x11<<16))  
#define SUART4_BASE				( 0x60000000  +  (0x12<<16))  
#define SUART5_BASE				( 0x60000000  +  (0x19<<16))  
#define SUART6_BASE				( 0x60000000  +  (0x1a<<16))  
#define SUART7_BASE				( 0x60000000  +  (0x24<<16))  
#define SUART8_BASE				( 0x60000000  +  (0x25<<16))  
#define SUART9_BASE				( 0x60000000  +  (0x30<<16))
#endif
#define SMAP_SIZE                (0xffff)


                        		
#define	rUARTDR0				*((volatile unsigned int *)(UART0_BASE + 0x00))
#define	rUARTRSR0				*((volatile unsigned int *)(UART0_BASE + 0x04))
#define	rUARTFR0				*((volatile unsigned int *)(UART0_BASE + 0x18))
#define	rUARTILPR0				*((volatile unsigned int *)(UART0_BASE + 0x20))
#define	rUARTIBRD0				*((volatile unsigned int *)(UART0_BASE + 0x24))
#define	rUARTFBRD0				*((volatile unsigned int *)(UART0_BASE + 0x28))
#define	rUARTLCR_H0				*((volatile unsigned int *)(UART0_BASE + 0x2C))
#define	rUARTCR0				*((volatile unsigned int *)(UART0_BASE + 0x30))
#define	rUARTIFLS0				*((volatile unsigned int *)(UART0_BASE + 0x34))
#define	rUARTIMSC0				*((volatile unsigned int *)(UART0_BASE + 0x38))
#define	rUARTRIS0				*((volatile unsigned int *)(UART0_BASE + 0x3C))
#define	rUARTMIS0				*((volatile unsigned int *)(UART0_BASE + 0x40))
#define	rUARTICR0				*((volatile unsigned int *)(UART0_BASE + 0x44))
#define	rUARTDMACR0				*((volatile unsigned int *)(UART0_BASE + 0x48))
                        			                                               
#define	rUARTDR1				*((volatile unsigned int *)(UART1_BASE + 0x00))
#define	rUARTRSR1				*((volatile unsigned int *)(UART1_BASE + 0x04))
#define	rUARTFR1				*((volatile unsigned int *)(UART1_BASE + 0x18))
#define	rUARTILPR1				*((volatile unsigned int *)(UART1_BASE + 0x20))
#define	rUARTIBRD1				*((volatile unsigned int *)(UART1_BASE + 0x24))
#define	rUARTFBRD1				*((volatile unsigned int *)(UART1_BASE + 0x28))
#define	rUARTLCR_H1				*((volatile unsigned int *)(UART1_BASE + 0x2C))
#define	rUARTCR1				*((volatile unsigned int *)(UART1_BASE + 0x30))
#define	rUARTIFLS1				*((volatile unsigned int *)(UART1_BASE + 0x34))
#define	rUARTIMSC1				*((volatile unsigned int *)(UART1_BASE + 0x38))
#define	rUARTRIS1				*((volatile unsigned int *)(UART1_BASE + 0x3C))
#define	rUARTMIS1				*((volatile unsigned int *)(UART1_BASE + 0x40))
#define	rUARTICR1				*((volatile unsigned int *)(UART1_BASE + 0x44))
#define	rUARTDMACR1				*((volatile unsigned int *)(UART1_BASE + 0x48))




/////////////////////////////////////////////////////
/* I2S */
#define I2S_BASE				( APB0_BASE  +  (11<<16))  
                    			
#define rIER					*(volatile unsigned int *)(I2S_BASE+0x00)
#define rIRER					*(volatile unsigned int *)(I2S_BASE+0x04)
#define rITER					*(volatile unsigned int *)(I2S_BASE+0x08)
#define rCER					*(volatile unsigned int *)(I2S_BASE+0x0c)
#define rCCR					*(volatile unsigned int *)(I2S_BASE+0x10)
#define rRXFFR					*(volatile unsigned int *)(I2S_BASE+0x14)
#define rTXFFR					*(volatile unsigned int *)(I2S_BASE+0x18)
#define rLRBR0					*(volatile unsigned int *)(I2S_BASE+0x20)
#define rLTHR0					*(volatile unsigned int *)(I2S_BASE+0x20)
#define rRRBR0					*(volatile unsigned int *)(I2S_BASE+0x24)
#define rRTHR0					*(volatile unsigned int *)(I2S_BASE+0x24)
#define rRER0					*(volatile unsigned int *)(I2S_BASE+0x28)
#define rTER0					*(volatile unsigned int *)(I2S_BASE+0x2c)
#define rRCR0					*(volatile unsigned int *)(I2S_BASE+0x30)
#define rTCR0					*(volatile unsigned int *)(I2S_BASE+0x34)
#define rISR0					*(volatile unsigned int *)(I2S_BASE+0x38)
#define rIMR0					*(volatile unsigned int *)(I2S_BASE+0x3c)
#define rROR0					*(volatile unsigned int *)(I2S_BASE+0x40)
#define rTOR0					*(volatile unsigned int *)(I2S_BASE+0x44)
#define rRFCR0					*(volatile unsigned int *)(I2S_BASE+0x48)
#define rTFCR0					*(volatile unsigned int *)(I2S_BASE+0x4c)
#define rRFF0					*(volatile unsigned int *)(I2S_BASE+0x50)
#define rTFF0					*(volatile unsigned int *)(I2S_BASE+0x54)
#define rRXDMA					*(volatile unsigned int *)(I2S_BASE+0x1c0)
#define rRRXDMA					*(volatile unsigned int *)(I2S_BASE+0x1c4)
#define rTXDMA					*(volatile unsigned int *)(I2S_BASE+0x1c8)
#define rRTXDMA					*(volatile unsigned int *)(I2S_BASE+0x1cc)




#define SDMMC_BASE				( APB0_BASE  +  (12<<16))  

#define SDIO_BASE				( APB0_BASE  +  (13<<16))  

#define I2C_BASE				( APB0_BASE  +  (14<<16))  

#define SPI_BASE				( APB0_BASE  +  (15<<16))  
/*#define SSPCR0					*(volatile unsigned int *)(SPI_BASE+0x00)
#define SSPCR1					*(volatile unsigned int *)(SPI_BASE+0x04)
#define SSPDR					*(volatile unsigned int *)(SPI_BASE+0x08)
#define SSPSR					*(volatile unsigned int *)(SPI_BASE+0x0c)
#define SSPCPSR					*(volatile unsigned int *)(SPI_BASE+0x10)
#define SSPIMSC					*(volatile unsigned int *)(SPI_BASE+0x14)
#define SSPRIS					*(volatile unsigned int *)(SPI_BASE+0x18)
#define SSPMIS					*(volatile unsigned int *)(SPI_BASE+0x1c)
#define SSPICR					*(volatile unsigned int *)(SPI_BASE+0x20)
#define SSPDMACR				*(volatile unsigned int *)(SPI_BASE+0x24)*/
#if 0
#define SSICTRLR0				*(volatile unsigned int *)(SPI_BASE+0x00)
#define SSICTRLR1				*(volatile unsigned int *)(SPI_BASE+0x04)
#define SSIENR					*(volatile unsigned int *)(SPI_BASE+0x08)
#define SSIMWCR					*(volatile unsigned int *)(SPI_BASE+0x0c)
#define SSISER					*(volatile unsigned int *)(SPI_BASE+0x10)
#define SSIBAUDR				*(volatile unsigned int *)(SPI_BASE+0x14)
#define SSITXFTLR				*(volatile unsigned int *)(SPI_BASE+0x18)
#define SSIRXFTLR				*(volatile unsigned int *)(SPI_BASE+0x1c)
#define SSITXFLR				*(volatile unsigned int *)(SPI_BASE+0x20)
#define SSIRXFLR				*(volatile unsigned int *)(SPI_BASE+0x24)
#define SSISR					*(volatile unsigned int *)(SPI_BASE+0x28)
#define SSIIMR					*(volatile unsigned int *)(SPI_BASE+0x2c)
#define SSIISR					*(volatile unsigned int *)(SPI_BASE+0x30)
#define SSIRISR					*(volatile unsigned int *)(SPI_BASE+0x34)
#define SSITXOICR				*(volatile unsigned int *)(SPI_BASE+0x38)
#define SSIRXOICR				*(volatile unsigned int *)(SPI_BASE+0x3c)
#define SSIRXUICR				*(volatile unsigned int *)(SPI_BASE+0x40)
#define SSIMSTICR				*(volatile unsigned int *)(SPI_BASE+0x44)
#define SSICIR					*(volatile unsigned int *)(SPI_BASE+0x48)
#define SSIDMACR				*(volatile unsigned int *)(SPI_BASE+0x4c)
#define SSIDMATDLR				*(volatile unsigned int *)(SPI_BASE+0x50)
#define SSIDMARDLR				*(volatile unsigned int *)(SPI_BASE+0x54)
#define SSIIDR					*(volatile unsigned int *)(SPI_BASE+0x58)
#define SSIVERSION				*(volatile unsigned int *)(SPI_BASE+0x5c)
#define SSIDR					*(volatile unsigned int *)(SPI_BASE+0x60)
#define SSIRXDLY				*(volatile unsigned int *)(SPI_BASE+0xfc)
#endif

#define rSPI_CONTROLREG             (*(volatile unsigned int *)(SPI_BASE + 0x08))
#define rSPI_CONFIGREG              (*(volatile unsigned int *)(SPI_BASE + 0x0C))
#define rSPI_INTREG                 (*(volatile unsigned int *)(SPI_BASE + 0x10))
#define rSPI_DMAREG                 (*(volatile unsigned int *)(SPI_BASE + 0x14))
#define rSPI_STATUSREG              (*(volatile unsigned int *)(SPI_BASE + 0x18))
#define rSPI_PERIODREG              (*(volatile unsigned int *)(SPI_BASE + 0x1C))
#define rSPI_TESTREG                (*(volatile unsigned int *)(SPI_BASE + 0x20))
#define rSPI_MSGREG                 (*(volatile unsigned int *)(SPI_BASE + 0x40))
#define rSPI_RXDATA                 (*(volatile unsigned int *)(SPI_BASE + 0x50))
#define rSPI_TXDATA                 (*(volatile unsigned int *)(SPI_BASE + 0x460))
#define rSPI_TXFIFO                   (SPI_BASE + 0x460)
#define rSPI_RXFIFO                   (SPI_BASE + 0x50) 


#define  RCRT_BASE				( APB0_BASE  +  (16<<16))  

#define  UART3_BASE				( APB0_BASE  +  (17<<16))  

#define  UART4_BASE				( APB0_BASE  +  (18<<16))  

#define  UART5_BASE				( APB0_BASE  +  (19<<16))  

#define  UART6_BASE				( APB0_BASE  +  (20<<16))  

#define  ANA_CFG0_BASE			( APB0_BASE  +  (31<<16) + (0<<12) )  
#define  ANA_CFG1_BASE			( APB0_BASE  +  (31<<16) + (1<<12) )  
#define  ANA_CFG2_BASE			( APB0_BASE  +  (31<<16) + (2<<12) )  
#define  ANA_CFG3_BASE			( APB0_BASE  +  (31<<16) + (3<<12) )  
#define  ANA_CFG4_BASE			( APB0_BASE  +  (31<<16) + (4<<12) )  
#define  ANA_CFG5_BASE			( APB0_BASE  +  (31<<16) + (5<<12) )  
#define  ANA_CFG6_BASE			( APB0_BASE  +  (31<<16) + (6<<12) )  
#define  ANA_CFG7_BASE			( APB0_BASE  +  (31<<16) + (7<<12) )  




/*WDTCR*/
#define WDTCR_CLKSEL	(1<<3)
#define WDTCR_INTEN	(1<<2)
#define WDTCR_RSTEN	(1<<1)
#define WDTCR_WDTEN	(1<<0)

/********************************GPIO *************************/
#define LX28XX_GPIO_PA		(0x60000000 + (5<<16))
#define LX28XX_GPIO_BASE		(APB0_BASE + (5<<16))

/*Gpio regs offset*/
#define GPIO_1_0_DATAOUT    	0x00
#define GPIO_1_0_DATADIR    	0x04
#define GPIO_1_0_DATAIN     	0xb0
                            	
#define GPIO_3_2_DATAOUT    	0x0C
#define GPIO_3_2_DATADIR    	0x10
#define GPIO_3_2_DATAIN     	0xb4
                            	
#define GPIO_5_4_DATAOUT    	0x18
#define GPIO_5_4_DATADIR    	0x1C
#define GPIO_5_4_DATAIN     	0xb8
                            	
#define GPIO_7_6_DATAOUT    	0x24
#define GPIO_7_6_DATADIR    	0x28
#define GPIO_7_6_DATAIN     	0xbC
                            	
#define GPIO_9_8_DATAOUT    	0x100
#define GPIO_9_8_DATADIR    	0x104
#define GPIO_9_8_DATAIN     	0x1b0
                            	
#define GPIO_11_10_DATAOUT  	0x10C
#define GPIO_11_10_DATADIR  	0x110
#define GPIO_11_10_DATAIN   	0x1b4
                            	
#define GPIO_13_12_DATAOUT  	0x118
#define GPIO_13_12_DATADIR  	0x11C
#define GPIO_13_12_DATAIN   	0x1b8
                            	
#define GPIO_15_14_DATAOUT  	0x124
#define GPIO_15_14_DATADIR  	0x128
#define GPIO_15_14_DATAIN   	0x1bC
                            	
#define GPIO_17_16_DATAOUT  	0x200
#define GPIO_17_16_DATADIR  	0x204
#define GPIO_17_16_DATAIN   	0x2b0
                            	
#define GPIO_19_18_DATAOUT  	0x20C
#define GPIO_19_18_DATADIR  	0x210
#define GPIO_19_18_DATAIN   	0x2b4
                            	
#define GPIO_21_20_DATAOUT  	0x218
#define GPIO_21_20_DATADIR  	0x21C
#define GPIO_21_20_DATAIN   	0x2b8
                            	
#define GPIO_23_22_DATAOUT  	0x224
#define GPIO_23_22_DATADIR  	0x228
#define GPIO_23_22_DATAIN   	0x2bC
                            	
#define GPIO_25_24_DATAOUT  	0x300
#define GPIO_25_24_DATADIR  	0x304
#define GPIO_25_24_DATAIN   	0x3b0

#define gpio_writel(off, val)		__raw_writel(val, off +LX28XX_GPIO_BASE )
#define gpio_readl(off)		__raw_readl(off +LX28XX_GPIO_BASE )


/* APB1 base 0xe0410000*/

/**************************** I2S *********************************************/

#define LX28XX_I2S_PA		(( APB0_BASE  +  (11<<16)))
#define LX28XX_I2S_BASE		(LX28XX_VA_APB_SYS + 0x3000)

/**************************** IRDA*********************************************/

#define LX28XX_IRDA_PA		(0xe0415000)
#define LX28XX_IRDA_BASE		(LX28XX_VA_APB_SYS + 0x5000)

/**************************** SD/MMC *********************************************/

#define LX28XX_SDMMC0_PA	(( APB0_BASE  +  (12<<16)))
#define LX28XX_SDMMC0_BASE	(LX28XX_SDMMC0_PA)


#define LX28XX_SDMMC1_PA	(( APB0_BASE  +  (13<<16)))
//#define LX28XX_SDMMC1_BASE	(LX28XX_VA_APB_SYS + 0x7000)
#define LX28XX_SDMMC1_BASE	(LX28XX_SDMMC1_PA)



/**************************** SSP(SPI) *********************************************/
#define LX28XX_SPI_PA		(( APB0_BASE  +  (15<<16)))
#define LX28XX_SPI_BASE		(LX28XX_VA_APB_SYS + 0xf000)



#endif /* __ASM_ARCH_HARDWARE_H */

