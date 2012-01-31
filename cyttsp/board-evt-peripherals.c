
/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom2.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/interrupt.h>
#include <linux/switch.h>
#include <linux/usb/android_composite.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/common.h>
#include <plat/usb.h>
#include <plat/control.h>
#include <plat/mux.h>
#include <plat/dmtimer.h>

#include <mach/board-encore.h>

#include "mmc-twl4030.h"
#include "mux.h"
#include "twl4030.h"
#include <linux/delay.h>
#include <linux/input/cyttsp.h>
char expansionboard_name[] = CY_I2C_NAME;

#define CY_GEN3		3

#define CY_SPI_VKEY_NAME "virtualkeys.cyttsp-spi" /* must match SPI name */
#define CY_I2C_VKEY_NAME "virtualkeys.cyttsp-i2c" /* must match I2C name */
#define CY_MAXX			600//1000
#define CY_MAXY			(1024)//1000	/* leave room for vkeys */
#define CY_VK_SZ_X		60
#define CY_VK_SZ_Y		80
#define CY_VK_CNTR_X1		(CY_VK_SZ_X*0)+(CY_VK_SZ_X/2)
#define CY_VK_CNTR_X2		(CY_VK_SZ_X*1)+(CY_VK_SZ_X/2)
#define CY_VK_CNTR_X3		(CY_VK_SZ_X*2)+(CY_VK_SZ_X/2)
#define CY_VK_CNTR_X4		(CY_VK_SZ_X*3)+(CY_VK_SZ_X/2)
#define CY_VK_CNTR_Y1		CY_MAXY+(CY_VK_SZ_Y/2)
#define CY_VK_CNTR_Y2		CY_MAXY+(CY_VK_SZ_Y/2)
#define CY_VK_CNTR_Y3		CY_MAXY+(CY_VK_SZ_Y/2)
#define CY_VK_CNTR_Y4		CY_MAXY+(CY_VK_SZ_Y/2)

#define CY_VK1_POS		":95:770:190:60"
#define CY_VK2_POS 		":285:770:190:60"
#define CY_VK3_POS		":475:770:190:60"
#define CY_VK4_POS 		":665:770:190:60"

#include <linux/kxtf9.h>
#include <linux/max17042.h>

#define KXTF9_DEVICE_ID                 "kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS         0x0F
#define KXTF9_GPIO_FOR_PWR              34
#define KXTF9_GPIO_FOR_IRQ              113

#define CYTTSP_I2C_SLAVEADDRESS 34
#define OMAP_CYTTSP_GPIO        99
#define OMAP_CYTTSP_RESET_GPIO 46
#define LCD_EN_GPIO                     36

#define MAX17042_GPIO_FOR_IRQ			100

/*virtual key support */
static ssize_t cyttsp_vkeys_show(struct kobject *kobj,
                        struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) CY_VK1_POS
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) CY_VK2_POS
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) CY_VK3_POS
	":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) CY_VK4_POS
	"\n");
}

static struct kobj_attribute cyttsp_vkeys_attr = {
        .attr = {
                .mode = S_IRUGO,
        },
        .show = &cyttsp_vkeys_show,
};

static struct attribute *cyttsp_properties_attrs[] = {
        &cyttsp_vkeys_attr.attr,
        NULL
};

static struct attribute_group cyttsp_properties_attr_group = {
        .attrs = cyttsp_properties_attrs,
};

static int cyttsp_vkey_init(const char *name)
{
	struct kobject *properties_kobj;
	int rc;
	char buf[160];

	printk("%s: init virtual keys\n", __func__);

	cyttsp_vkeys_attr.attr.name = name;
	properties_kobj = kobject_create_and_add("board_properties", NULL);
	rc = 0;
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
			&cyttsp_properties_attr_group);

	if (!properties_kobj)
		printk("%s: "
			"setup cyttsp virtual keys fail nobj \n",
			__func__);
	if (rc)
		printk("%s: "
			"setup cyttsp virtual keys fail rc=%d \n",
			__func__, rc);

	if (!properties_kobj || rc)
		printk("%s: failed to create board_properties\n",
			__func__);
	else {
		printk("%s: "
			"setup cyttsp virtual keys ok name=%s\n",
			__func__, cyttsp_vkeys_attr.attr.name);
		cyttsp_vkeys_show(properties_kobj, &cyttsp_vkeys_attr, buf);
		printk("%s: " "%s""\n", __func__, buf);
	}
	return rc;
}

#define CY_I2C_IRQ_GPIO	OMAP_CYTTSP_GPIO
#define CY_I2C_ADR	CYTTSP_I2C_SLAVEADDRESS

static int cyttsp_i2c_init(void)
{
	int ret;

	ret = cyttsp_vkey_init(CY_I2C_VKEY_NAME);

	if (gpio_request(OMAP_CYTTSP_RESET_GPIO, "tma340_reset") < 0) {
		printk(KERN_ERR "can't get tma340 xreset GPIO\n");
		return -1;
	}

	if (gpio_request(OMAP_CYTTSP_GPIO, "cyttsp_touch") < 0) {
		printk(KERN_ERR "can't get cyttsp interrupt GPIO\n");
		return -1;
	}

	gpio_direction_input(OMAP_CYTTSP_GPIO);
	omap_set_gpio_debounce(OMAP_CYTTSP_GPIO, 0);

	mdelay(100);
	printk("Reseting TMA340\n");
	gpio_direction_output(OMAP_CYTTSP_RESET_GPIO, 0);
	mdelay(10);
	gpio_direction_output(OMAP_CYTTSP_RESET_GPIO, 1);
	mdelay(100);

	return 0;
}

static int cyttsp_i2c_wakeup(void)
{
	return 0;
}

static struct cyttsp_platform_data cyttsp_i2c_platform_data = {
	.init = cyttsp_i2c_init,
	.maxx = CY_MAXX,
	.maxy = CY_MAXY,
	.use_hndshk = 0 /*CY_SEND_HNDSHK*/,
	/* change act_intrvl to customize the Active power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.name = CY_I2C_NAME,
	.irq_gpio = CY_I2C_IRQ_GPIO,
};

extern void evt_lcd_panel_init(void);

int has_3G_support(void)
{
	return( (system_rev & BOARD_FEATURE_3G) != 0 );
}

int has_1GHz_support(void)
{
	return( (system_rev & BOARD_FEATURE_1GHz) != 0 );
}

static void kxtf9_dev_init(void)
{
	printk("board-3621_evt1a.c: kxtf9_dev_init ...\n");

	if (gpio_request(KXTF9_GPIO_FOR_IRQ, "kxtf9_irq") < 0) {
		printk(KERN_ERR "Can't get GPIO for kxtf9 IRQ\n");
		return;
	}

	printk("board-3621_evt1a.c: kxtf9_dev_init > Init kxtf9 irq pin %d !\n", KXTF9_GPIO_FOR_IRQ);
	gpio_direction_input(KXTF9_GPIO_FOR_IRQ);
	omap_set_gpio_debounce(KXTF9_GPIO_FOR_IRQ, 0);
}


struct kxtf9_platform_data kxtf9_platform_data_here = {
	.min_interval   = 1,
	.poll_interval  = 1000,

	.g_range        = KXTF9_G_8G,
	.shift_adj      = SHIFT_ADJ_2G,

	// Map the axes from the sensor to the device.

	//. SETTINGS FOR THE EVT1A:
	.axis_map_x     = 1,
	.axis_map_y     = 0,
	.axis_map_z     = 2,
	.negate_x       = 1,
	.negate_y       = 0,
	.negate_z       = 0,

	.data_odr_init          = ODR12_5F,
	.ctrl_reg1_init         = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init          = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
	.int_ctrl_init          = KXTF9_IEN,
	.tilt_timer_init        = 0x03,
	.engine_odr_init        = OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init         = 0x16,
	.wuf_thresh_init        = 0x28,
	.tdt_timer_init         = 0x78,
	.tdt_h_thresh_init      = 0xFF,
	.tdt_l_thresh_init      = 0x14,
	.tdt_tap_timer_init     = 0x53,
	.tdt_total_timer_init   = 0x24,
	.tdt_latency_timer_init = 0x10,
	.tdt_window_timer_init  = 0xA0,

	.gpio = KXTF9_GPIO_FOR_IRQ,
};

static struct regulator_consumer_supply encore_lcd_tp_supply[] = {
	{ .supply = "vtp" },
	{ .supply = "vlcd" },
};

static struct regulator_init_data encore_lcd_tp_vinit = {
    .constraints = {
        .min_uV = 3300000,
        .max_uV = 3300000,
        .valid_modes_mask = REGULATOR_MODE_NORMAL,
        .valid_ops_mask = REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies = 2,
    .consumer_supplies = encore_lcd_tp_supply,
};

static struct fixed_voltage_config encore_lcd_touch_reg_data = {
    .supply_name = "vdd_lcdtp",
    .microvolts = 3300000,
    .gpio = LCD_EN_GPIO,
    .enable_high = 1,
    .enabled_at_boot = 0,
    .init_data = &encore_lcd_tp_vinit,
};

static struct platform_device encore_lcd_touch_regulator_device = {
    .name   = "reg-fixed-voltage",
    .id     = -1,
    .dev    = {
        .platform_data = &encore_lcd_touch_reg_data,
    },
};

static int board_keymap[] = {
	KEY(0, 0, KEY_HOME),
	KEY(1, 0, KEY_VOLUMEUP),
	KEY(2, 0, KEY_VOLUMEDOWN),
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data evt_kp_twl4030_data = {
	.keymap_data	= &board_map_data,
	.rows		= 8,
	.cols		= 8,
	.rep		= 1,
};

static struct gpio_keys_button evt_gpio_buttons[] = {
	{
		.code			= KEY_POWER,
		.gpio			= 14,
		.desc			= "POWER",
		.active_low		= 0,
		.wakeup			= 1,
	},
	{
		.code			= KEY_HOME,
		.gpio			= 48,
		.desc			= "HOME",
		.active_low		= 1,
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data evt_gpio_key_info = {
	.buttons	= evt_gpio_buttons,
	.nbuttons	= ARRAY_SIZE(evt_gpio_buttons),
};

static struct platform_device evt_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &evt_gpio_key_info,
	},
};

static struct __initdata twl4030_power_data evt_t2scripts_data;

static struct regulator_consumer_supply evt_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply evt_vsim_supply = {
	.supply		= "vmmc_aux",
};


/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data evt_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &evt_vmmc1_supply,
};


/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data evt_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &evt_vsim_supply,
};

/*--------------------------------------------------------------------------*/
#ifdef CONFIG_CHARGER_MAX8903
static struct platform_device max8903_charger_device = {
	.name           = "max8903_charger",
	.id             = -1,
};
#endif


#ifdef CONFIG_ENCORE_MODEM_MGR
struct platform_device encore_modem_mgr_device = {
	.name = "encore_modem_mgr",
	.id   = -1,
};
#endif


/*--------------------------------------------------------------------------*/

static struct platform_device *evt_board_devices[] __initdata = {	
	&encore_lcd_touch_regulator_device,
	&evt_keys_gpio,
#ifdef CONFIG_CHARGER_MAX8903
	&max8903_charger_device,
#endif
};

static struct twl4030_hsmmc_info mmc[] __initdata = {
	{
		.name		= "external",
		.mmc		= 1,
		.wires		= 4,
		.cd_active_high	= true,
		.gpio_wp	= -EINVAL,
		.power_saving	= true,
	},
	{
		.name		= "internal",
		.mmc		= 2,
		.wires		= 8,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
	},
	{
		.mmc		= 3,
		.wires		= 4,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{}      /* Terminator */
};

static int evt_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	/* The controller that is connected to the 128x device
	 * should have the card detect gpio disabled. This is
	 * achieved by initializing it with a negative value
	 */
	mmc[CONFIG_TIWLAN_MMC_CONTROLLER - 1].gpio_cd = -EINVAL;
#endif

	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	evt_vmmc1_supply.dev = mmc[0].dev;
	evt_vsim_supply.dev = mmc[0].dev;

	return 0;
}


static struct twl4030_usb_data evt_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_gpio_platform_data evt_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= evt_twl_gpio_setup,
};

static struct twl4030_madc_platform_data evt_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data evt_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &evt_madc_data,
	.usb		= &evt_usb_data,
	.gpio		= &evt_gpio_data,
	.keypad		= &evt_kp_twl4030_data,
	.power		= &evt_t2scripts_data,
	.vmmc1		= &evt_vmmc1,
	.vsim		= &evt_vsim,
};

#ifdef CONFIG_BATTERY_MAX17042
struct max17042_platform_data max17042_platform_data_here = {

        //fill in device specific data here
        //load stored parameters from Rom Tokens? 
        //.val_FullCAP =
        //.val_Cycles =
        //.val_FullCAPNom =
        //.val_SOCempty =
        //.val_Iavg_empty =
        //.val_RCOMP0 =
        //.val_TempCo=
        //.val_k_empty0 =
        //.val_dQacc =
        //.val_dPacc =

        .gpio = MAX17042_GPIO_FOR_IRQ,
};
#endif

static struct i2c_board_info __initdata evt_i2c_boardinfo[] = {
#ifdef CONFIG_BATTERY_MAX17042
        {
                I2C_BOARD_INFO(MAX17042_DEVICE_ID, MAX17042_I2C_SLAVE_ADDRESS),
                .platform_data = &max17042_platform_data_here,
                .irq = OMAP_GPIO_IRQ(MAX17042_GPIO_FOR_IRQ),
        },
#endif  /*CONFIG_BATTERY_MAX17042*/
	{
		I2C_BOARD_INFO("tps65921", 0x48),
		.flags		= I2C_CLIENT_WAKE,
		.irq		= INT_34XX_SYS_NIRQ,
		.platform_data	= &evt_twldata,
	},
	{
		I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtf9_platform_data_here,
		.irq = OMAP_GPIO_IRQ(KXTF9_GPIO_FOR_IRQ),
	},
};

#define AUDIO_CODEC_IRQ_GPIO             59
#define AIC3100_NAME			"tlv320dac3100"
#define AIC3100_I2CSLAVEADDRESS		0x18

#if defined(CONFIG_SND_SOC_TLV320DAC3100) || defined(CONFIG_SND_SOC_TLV320DAC3100)
#define AUDIO_CODEC_POWER_ENABLE_GPIO    103
#define AUDIO_CODEC_RESET_GPIO           37

static void audio_dac_3100_dev_init(void)
{
        printk("board-3621_evt1a.c: audio_dac_3100_dev_init ...\n");
        if (gpio_request(AUDIO_CODEC_RESET_GPIO, "AUDIO_CODEC_RESET_GPIO") < 0) {
                printk(KERN_ERR "can't get AUDIO_CODEC_RESET_GPIO \n");
                return;
        }

        printk("board-3621_evt1a.c: audio_dac_3100_dev_init > set AUDIO_CODEC_RESET_GPIO to output Low!\n");
        gpio_direction_output(AUDIO_CODEC_RESET_GPIO, 0);
	gpio_set_value(AUDIO_CODEC_RESET_GPIO, 0);

        printk("board-3621_evt1a.c: audio_dac_3100_dev_init ...\n");
        if (gpio_request(AUDIO_CODEC_POWER_ENABLE_GPIO, "AUDIO DAC3100 POWER ENABLE") < 0) {
                printk(KERN_ERR "can't get AUDIO_CODEC_POWER_ENABLE_GPIO \n");
                return;
        }

        printk("board-3621_evt1a.c: audio_dac_3100_dev_init > set AUDIO_CODEC_POWER_ENABLE_GPIO to output and value high!\n");
        gpio_direction_output(AUDIO_CODEC_POWER_ENABLE_GPIO, 0);
	gpio_set_value(AUDIO_CODEC_POWER_ENABLE_GPIO, 1);

	/* 1 msec delay needed after PLL power-up */
        mdelay (1);

        printk("board-3621_evt1a.c: audio_dac_3100_dev_init > set AUDIO_CODEC_RESET_GPIO to output and value high!\n");
	gpio_set_value(AUDIO_CODEC_RESET_GPIO, 1);
}
#endif
#ifdef CONFIG_BATTERY_MAX17042
static void max17042_dev_init(void)
{
        printk("board-3621_evt1a.c: max17042_dev_init ...\n");

        if (gpio_request(MAX17042_GPIO_FOR_IRQ, "max17042_irq") < 0) {
                printk(KERN_ERR "Can't get GPIO for max17042 IRQ\n");
                return;
        }

        printk("board-3621_evt1a.c: max17042_dev_init > Init max17042 irq pin %d !\n", MAX17042_GPIO_FOR_IRQ);
        gpio_direction_input(MAX17042_GPIO_FOR_IRQ);
        omap_set_gpio_debounce(MAX17042_GPIO_FOR_IRQ, 0);
        printk("max17042 GPIO pin read %d\n", gpio_get_value(MAX17042_GPIO_FOR_IRQ));
}
#endif

static struct i2c_board_info __initdata evt_i2c_bus2_info[] = {
	{
		I2C_BOARD_INFO(AIC3100_NAME,  AIC3100_I2CSLAVEADDRESS),
                .irq = OMAP_GPIO_IRQ(AUDIO_CODEC_IRQ_GPIO),
	},
	{
		I2C_BOARD_INFO(CY_I2C_NAME, CYTTSP_I2C_SLAVEADDRESS),
		.platform_data = &cyttsp_i2c_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_CYTTSP_GPIO),
	},
};

static int __init omap_i2c_init(void)
{
	/* Disable OMAP 3630 internal pull-ups for I2Ci */
	if (cpu_is_omap3630()) {

		u32 prog_io;

		prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
		/* Program (bit 19)=1 to disable internal pull-up on I2C1 */
		prog_io |= OMAP3630_PRG_I2C1_PULLUPRESX;
		/* Program (bit 0)=1 to disable internal pull-up on I2C2 */
		prog_io |= OMAP3630_PRG_I2C2_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO2);
		/* Program (bit 7)=1 to disable internal pull-up on I2C3 */
		prog_io |= OMAP3630_PRG_I2C3_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO2);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
		/* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
		prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
	}
	omap_register_i2c_bus(1, 100, NULL, evt_i2c_boardinfo,
			ARRAY_SIZE(evt_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, evt_i2c_bus2_info,
			ARRAY_SIZE(evt_i2c_bus2_info));
	return 0;
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode			= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 100,
};

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.vendor = "B&N     ",
	.product = "Ebook Disk      ",
	.release = 0x0101,
	.nluns = 2,
};

static struct platform_device usb_mass_storage_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &mass_storage_pdata,
		},
};
#endif

static struct twl4030_ins sleep_on_seq[] = {

	/* Turn off HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_OFF), 2},
	/* Turn OFF VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_OFF), 2},
	/* Turn OFF VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_OFF), 2},
	/* Turn OFF VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_OFF), 2},
        /* Turn OFF REGEN */
   //     {MSG_SINGULAR(DEV_GRP_P1, 0x15, RES_STATE_OFF), 2},
};

static struct twl4030_script sleep_on_script = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_p12_seq[] = {
	/* Turn on HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 2},
	/* Turn ON VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_ACTIVE), 2},
        /* Turn ON REGEN */
        {MSG_SINGULAR(DEV_GRP_P1, 0x15, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p12_script = {
	.script = wakeup_p12_seq,
	.size   = ARRAY_SIZE(wakeup_p12_seq),
	.flags  = TWL4030_WAKEUP12_SCRIPT,
};

static struct twl4030_ins wakeup_p3_seq[] = {
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p3_script = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

static struct twl4030_ins wrst_seq[] = {
/*
 * Reset twl4030.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_WRST), 0x60},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] = {
	&sleep_on_script,
	&wakeup_p12_script,
	&wakeup_p3_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3, .type = -1,
		.type2 = -1 },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
	{ 0, 0},
};

static struct twl4030_power_data boxer_t2scripts_data = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static struct ehci_hcd_omap_platform_data ehci_pdata = {
    .port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
    .port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
    .port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

    .phy_reset = 0,

    .reset_gpio_port[0] = -EINVAL,
    .reset_gpio_port[1] = -EINVAL,
    .reset_gpio_port[2] = -EINVAL,
};

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
    .start  = ENCORE_RAM_CONSOLE_START,
    .end    = (ENCORE_RAM_CONSOLE_START + ENCORE_RAM_CONSOLE_SIZE - 1),
    .flags  = IORESOURCE_MEM,
};

static struct platform_device ram_console_device = {
    .name           = "ram_console",
    .id             = 0,
    .num_resources  = 1,
    .resource       = &ram_console_resource,
};

static inline void ramconsole_init(void)
{
    platform_device_register(&ram_console_device);
}
#else
static inline void ramconsole_init(void) {}
#endif /* CONFIG_ANDROID_RAM_CONSOLE */

void __init evt_peripherals_init(void)
{
#if 0
	/* Use scripts from TI L25.i3 FroYo */
	twl4030_get_scripts(&evt_t2scripts_data);
#else
	/* Use custom Encore scripts */
	evt_t2scripts_data = boxer_t2scripts_data;
#endif
    ramconsole_init();
    omap_i2c_init();
	platform_add_devices(evt_board_devices,
			ARRAY_SIZE(evt_board_devices));

	if (has_3G_support()) {
    		usb_ehci_init(&ehci_pdata);
	}

#ifdef CONFIG_ENCORE_MODEM_MGR
	if (has_3G_support()) {
		platform_device_register(&encore_modem_mgr_device);
	}
#endif /* CONFIG_ENCORE_MODEM_MGR */

#if defined(CONFIG_SND_SOC_TLV320DAC3100) || defined(CONFIG_SND_SOC_TLV320DAC3100)
	audio_dac_3100_dev_init();
#endif

	/* NOTE: Please deselect CONFIG_MACH_OMAP_USE_UART3 in order
	 * to init only UART1 and UART2, all in the name of saving some
	 * power.
	 */
	omap_serial_init();
	evt_lcd_panel_init();
	kxtf9_dev_init();
#ifdef CONFIG_BATTERY_MAX17042
        max17042_dev_init();
#endif
	usb_musb_init(&musb_board_data);
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
	/*
	 * Strangely enough, Android gadget platform data is registered in
	 * the MUSB driver. We can only register the mass-storage platform
	 * data here.
	 */
	platform_device_register(&usb_mass_storage_device);
#endif
}
