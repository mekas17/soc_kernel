/*
 * linux/include/asm-arm/arch-arkdmp/irqs.h
 */


/* interrupt num */
// interrupt number
#if 0
#define LX28XX_WDT_INT           1
#define LX28XX_SPI_INT           2
#define LX28XX_I2C_INT           3
#define LX28XX_RCRT_I2C1_INT     4
#define LX28XX_UART0_INT         5
#define LX28XX_UART1_INT         6
#define LX28XX_UART2_INT         7
#define LX28XX_UART3_INT         8
#define LX28XX_UART4_INT         9
#define LX28XX_UART5_INT         10
#define LX28XX_TIMER0_INT        11
#define LX28XX_TIMER1_INT        12
#define LX28XX_SPI2_INT          13
#define LX28XX_UART6_SDMMC_INT   14
#define LX28XX_SDIO_INT          15
#define LX28XX_GPIO_INT          16
#define LX28XX_MCAN_INT          17
#define LX28XX_I2S_INT           18
#define LX28XX_I2S1_STX0_INT     19
#define LX28XX_I2S2_SRX0_INT     20
#define LX28XX_PCIE_INT          21
#define LX28XX_MAC_INT           22
#define LX28XX_LCD_INT           23
#define LX28XX_ITU565_INT        24
#define LX28XX_USB_INT           25
#define LX28XX_DMA_INT           26
#define LX28XX_MAE0_INT          27
#define LX28XX_MAE2_INT          28
#define LX28XX_SATA_INT          29
#define LX28XX_USB2_INT          31

#define NR_IRQS		32

#else
#define LX28XX_MCAN0_INT                 1  // 
#define LX28XX_MCAN1_INT                 2  // 
#define LX28XX_MCAN2_INT                 3  // 
#define LX28XX_RCRT_INT                  4  // 
#define LX28XX_I2C1_INT                  5  // 
#define LX28XX_I2C_INT                   6  // 
#define LX28XX_SDIO_INT                  7  // 
#define LX28XX_SDMMC_INT                 8  // 
#define LX28XX_SPI_INT                   9  // 
#define LX28XX_SPI1_INT                  10 // 
#define LX28XX_SPI2_INT                  11 // 
#define LX28XX_SPI3_INT                  12 // 
#define LX28XX_SPI4_INT                  13 // 
#define LX28XX_SPI5_INT                  14 // 
#define LX28XX_SPI6_INT                  15 // 
#define LX28XX_SPI7_INT                  16 // 
#define LX28XX_SPI8_INT                  17 // 
#define LX28XX_SPI9_INT                  18 // 
#define LX28XX_UART0_INT                 19 // 
#define LX28XX_SUART1_INT                 20 // 
#define LX28XX_SUART2_INT                 21 // 
#define LX28XX_SUART3_INT                 22 // 
#define LX28XX_SUART4_INT                 23 // 
#define LX28XX_SUART5_INT                 24 // 
#define LX28XX_SUART6_INT                 25 // 
#define LX28XX_SUART7_INT                 26 // 
#define LX28XX_SUART8_INT                 27 // 
#define LX28XX_GPIO_INT                  28 // 
#define LX28XX_TIMER0_INT                29 // 
#define LX28XX_TIMER1_INT                30 // 
#define LX28XX_TIMER2_INT                31 // 
#define LX28XX_TIMER3_INT                32 // 
#define LX28XX_TIMER4_INT                33 // 
#define LX28XX_TIMER5_INT                34 // 
#define LX28XX_TIMER6_INT                35 // 
#define LX28XX_TIMER7_INT                36 // 
#define LX28XX_TIMER8_INT                37 // 
#define LX28XX_TIMER9_INT                38 // 
#define LX28XX_TIMER10_INT               39 // 
#define LX28XX_TIMER11_INT               40 // 
#define LX28XX_TIMER12_INT               41 // 
#define LX28XX_TIMER13_INT               42 // 
#define LX28XX_TIMER14_INT               43 // 
#define LX28XX_TIMER15_INT               44 // 
#define LX28XX_WDT_INT                   45 // 
#define LX28XX_PWM_INT                   46 // 
#define LX28XX_I2S_INT                   47 // 
#define LX28XX_I2S1_INT                  48 // 
#define LX28XX_I2S2_INT                  49 // 
#define LX28XX_SPDIF0_RX_INT             50 // 
#define LX28XX_SPDIF0_TX_INT             51 // 
#define LX28XX_RTC_INT                   52 // 
#define LX28XX_EXT0_INT                  53 // 
#define LX28XX_EXT1_INT                  54 // 
#define LX28XX_EXT2_INT                  55 // 
#define LX28XX_EXT3_INT                  56 // 
#define LX28XX_EXT4_INT                  57 // 
#define LX28XX_EXT5_INT                  58 //
#if  0   //add by lzl 20180524
#define LX28XX_EXT6_INT                  59 //
#else
#define LX28XX_SUART9_INT                  59 //
#endif

#define LX28XX_PLC_CO_CTRL0_INT          60 // 
#define LX28XX_PLC_CO_CTRL1_INT          61 // 
#define LX28XX_ADC_INT                   62 // 
#define LX28XX_ADC2_INT                  63 // 
#define LX28XX_MAC2_INT                  64 // 
#define LX28XX_MAC_INT                   65 // 
#define LX28XX_USB_INT                   66 // 
#define LX28XX_USB2_INT                  67 // 
#define LX28XX_IT656_INT                 68 // 
#define LX28XX_LCD_INT                   69 // 
#define LX28XX_DMA_INT                   70 // 
#define LX28XX_MAE0_INT                  71 // 
#define LX28XX_DMA2_INT                  72 // 
#define LX28XX_SATA_INT                  73 // 
#define LX28XX_PCIE_INT                  74 // 
#define LX28XX_MAE2_INT                  75 // 
#define LX28XX_HDMI_INT                  76 // 
#define LX28XX_HDMI_HPD_INT              77 // 
#define LX28XX_GPU_IRQPMU_INT            78 // 
#define LX28XX_GPU_IRQGPMMU_INT          79 // 
#define LX28XX_GPU_IRQGP_INT             80 // 
#define LX28XX_GPU_IRQPPMMU1_INT         81 // 
#define LX28XX_GPU_IRQPP1_INT            82 // 
#define LX28XX_GPU_IRQPPMMU0_INT         83 // 
#define LX28XX_GPU_IRQPP0_INT            84 // 

#define LX28XX_SYS_INTR0_0_INT           94 // 
#define LX28XX_SYS_INTR0_1_INT           95 //
      
      
#define NR_IRQS		(96 + 32)

#endif
