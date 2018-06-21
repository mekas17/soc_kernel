/*
 * Xiangjing T19XX chip specific setup
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>

#include <mach/irqs.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>	 
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/map.h>
#include <mach/hardware.h>
#include <mach/video.h>

#include <media/tvp514x.h>
#include <mach/clock.h>


#define SENSOR_CMOS		"geke-sensor"
#define TVP5147_CH1		"tvp514x-1"



struct i2c_gpio_platform_data i2c_config={
	.sda_pin = 0, //3 --> 0
	.scl_pin	= 11,
	.scl_is_open_drain = 1,

	//.scl_is_output_only =1;

};



static struct platform_device i2c_device = {
	.name		= "i2c-gpio",
	.id		= 1,
	.dev		= {
			.platform_data= &i2c_config,
	},
};


static u64 vpif_dma_mask = DMA_BIT_MASK(32);


static struct resource vpif_capture_resource[] = {
	{//ch 0
		.start = T19XX_VIDA_BASE,
		.end   = T19XX_VIDA_BASE + SZ_4K-1,
		.flags = IORESOURCE_MEM,
	},

	{//ch 1
	.start = T19XX_VIDB_BASE,
	.end   = T19XX_VIDB_BASE + SZ_4K-1,
	.flags = IORESOURCE_MEM,
	},

	{//ch 2
	.start = T19XX_VIDC_BASE,
	.end   = T19XX_VIDC_BASE + SZ_4K-1,
	.flags = IORESOURCE_MEM,
	},

	{//ch 3
		.start = T19XX_VIDD_BASE,
		.end   = T19XX_VIDD_BASE + SZ_4K-1,
		.flags = IORESOURCE_MEM,
	},
	//irq
	{
		.start = T19XX_CAP_INT,
		.end   = T19XX_CAP_INT,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device vpif_capture_dev = {
	.name		= "vpif_capture",
	.id		= -1,
	.dev		= {
			.dma_mask 		= &vpif_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource	= vpif_capture_resource,
	.num_resources	= ARRAY_SIZE(vpif_capture_resource),
};


static struct resource vpif_display_resource[] = {
	{
		.start = T19XX_LCD_INT,
		.end   = T19XX_LCD_INT,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device vpif_display_dev = {
	.name		= "vpif_display",
	.id		= -1,
	.dev		= {
			.dma_mask 		= &vpif_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource	= vpif_display_resource,
	.num_resources	= ARRAY_SIZE(vpif_display_resource),
};



static struct tvp514x_platform_data tvp5146_pdata = {
	.clk_polarity = 0,
	.hs_polarity = 1,
	.vs_polarity = 1
};

#define TVP514X_STD_ALL (V4L2_STD_NTSC | V4L2_STD_PAL)
#ifdef CONFIG_OV_SENSOR

static struct vpif_subdev_info vpif_capture_sdev_info[] = {
	{
		.name	= SENSOR_CMOS,

		.board_info = {
			I2C_BOARD_INFO("ov", 0x21),
			//.platform_data = &tvp5146_pdata,
		},		//.input = INPUT_CVBS_VI2B,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
		.can_route = 1,
		.vpif_if = {
			.if_type = VPIF_IF_YUV,
			.hd_pol = 0,
			.vd_pol = 1, //for ov
			.fid_pol = 0,
		},
	},
};

#else
static struct vpif_subdev_info vpif_capture_sdev_info[] = {
	{
		.name	= SENSOR_CMOS,

		.board_info = {
			I2C_BOARD_INFO("aptina", 0x5d),
			//.platform_data = &tvp5146_pdata,
		},		//.input = INPUT_CVBS_VI2B,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
		.can_route = 1,
		.vpif_if = {
			.if_type = VPIF_IF_YUV,
			.hd_pol = 0,
			.vd_pol = 0,
			.fid_pol = 0,
		},
	},
};
#endif


static const struct vpif_input ch0_inputs[] = {
	{
		.input = {
			.index = 0,
			.name = "Camera",
			.type = V4L2_INPUT_TYPE_CAMERA,
			.std = TVP514X_STD_ALL,
		},
		.subdev_name = SENSOR_CMOS,
	},
};

static const struct vpif_input ch1_inputs[] = {
       {
		.input = {
			.index = 0,
			.name = "S-Video",
			.type = V4L2_INPUT_TYPE_CAMERA,
			.std = TVP514X_STD_ALL,
		},
		.subdev_name = TVP5147_CH1,
	},
};

static struct vpif_capture_config t18xx_vpif_capture_cfg = {
	.subdev_info = vpif_capture_sdev_info,
	.subdev_count = ARRAY_SIZE(vpif_capture_sdev_info),
	.card_name	= "T18xx EVM",
	.chan_config[0] = {
		.inputs = ch0_inputs,
		.input_count = ARRAY_SIZE(ch0_inputs),
	},
#if 0
	.chan_config[1] = {
		.inputs = ch1_inputs,
		.input_count = ARRAY_SIZE(ch1_inputs),
	},
#endif
};




void t18xx_setup_vpif(struct vpif_display_config *display_config,
		       struct vpif_capture_config *capture_config)
{
	unsigned int value;
	//vid mux ctl in here

	vpif_display_dev.dev.platform_data = display_config;
	vpif_capture_dev.dev.platform_data = capture_config;
	//platform_device_register(&vpif_dev);
	platform_device_register(&vpif_display_dev);
	platform_device_register(&vpif_capture_dev);
}


static int __init t18xx_init_devices(void)
{
	platform_device_register(&i2c_device);

	t18xx_setup_vpif(NULL, &t18xx_vpif_capture_cfg);

	//platform_device_register(&i2c_device);
	return 0;
}
postcore_initcall(t18xx_init_devices);
