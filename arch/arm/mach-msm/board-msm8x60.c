/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2010, Hewlett-Packard Development Company, L.P. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/bahama.h>
#include <linux/input/pmic8058-keypad.h>
#include <linux/pmic8058-pwrkey.h>
#include <linux/pmic8058-vibrator.h>
#include <linux/leds.h>
#include <linux/pmic8058-othc.h>
#include <linux/mfd/pmic8901.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <linux/bootmem.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/leds-pmic8058.h>
#include <linux/pmic8058-xoadc.h>
#include <linux/m_adc.h>
#include <linux/m_adcproc.h>
#include <linux/leds-lm8502.h>
#include <linux/mfd/marimba.h>
#include <linux/msm-charger.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
//guoye:[20100914] st lsm303dlh g/a/o sensor
#include <linux/i2c/lsm303dlh.h>
//guoye
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/input/tdisc_shinetsu.h>
#include <linux/input/cy8c_ts.h>
#ifdef CONFIG_TOUCHSCREEN_CYTTSP_I2C
#include <linux/cyttsp.h>
#endif
// yegw for HP cypress touch 
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_HP_I2C
#include <linux/cyttsp_hp.h>
#endif
// end
#include <linux/gpio_keys.h>
#include <linux/i2c/isa1200.h>
#include <linux/dma-mapping.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/dma.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/irqs.h>
#include <mach/msm_spi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_iomap.h>
#include <asm/mach/mmc.h>
#include <mach/msm_battery.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_xo.h>
#include <mach/msm_bus_board.h>
#include <linux/i2c/isl9519.h>
#include <mach/msm_touchpad.h>
#include <linux/hp_sensors.h>
#include <linux/i2c/atmel_maxtouch.h>
#ifdef CONFIG_INPUT_ISL29023
#include <linux/isl29023.h>
#endif

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <mach/sdio_al.h>

#include "devices.h"
#include "devices-msm8x60.h"
#include "devices-topaz.h"
#include "devices-opal.h"
#include "cpuidle.h"
#include "pm.h"
#include "rpm.h"
#include "spm.h"
#include "rpm_log.h"
#include "timer.h"
#include "saw-regulator.h"
#include "socinfo.h"
#include "rpm-regulator.h"
#include "gpiomux.h"
#include "gpiomux-8x60.h"

#include "hss.h"
#ifdef CONFIG_CHARGER_MAX8903
#include <linux/max8903_charger.h>
#include <linux/power_supply.h>
#endif

#ifdef CONFIG_A6
#include <linux/a6_sbw_interface.h>
#include <linux/a6.h>
#include <mach/gpio.h>
#endif

#ifdef CONFIG_HRES_COUNTER
#include <linux/hres_counter.h>
#endif

#ifdef CONFIG_MFD_WM8958
#include <linux/mfd/wm8994/pdata.h>
#endif

#ifdef CONFIG_SENSORS_MPU3050
#include <linux/mpu.h>
#endif

#include <linux/boardid.h>

//HP zhanghong:Oct 27 11:12 CST 2010, begin
#ifdef CONFIG_INPUT_CYPRESS_CY8C20236A
#include <linux/i2c/cy8c20236a.h>
#endif
//End

// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
// #define WORKAROUND_RPM_REGULATOR
// end
//HP wanqin 
#if defined(CONFIG_BT) && defined(CONFIG_BT_MSM_SLEEP)
#include <net/bluetooth/bluesleep.h>
#endif

//wanqin
#ifdef CONFIG_NFC
#include <linux/i2c/nfc.h>
#endif
#define TOPAZ_CLOCK_FIXUP 1
#if TOPAZ_CLOCK_FIXUP
#include "clock-8x60.h"
#endif

/* HP SamLin 20110118, start for boradcom gps module 4751 driver */
#if defined (CONFIG_BRCM4751)
#include <linux/brcm4751.h>
#endif
/* End */

#define MSM_SHARED_RAM_PHYS 0x40000000

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_BASE			NR_MSM_GPIOS
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_GPIO_BASE)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)		(sys_gpio - PM8058_GPIO_BASE)
#define PM8058_MPP_BASE			(PM8058_GPIO_BASE + PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_MPP_BASE)
#define PM8058_MPP_SYS_TO_PM(sys_gpio)		(sys_gpio - PM8058_MPP_BASE)
#define PM8058_IRQ_BASE				(NR_MSM_IRQS + NR_GPIO_IRQS)

#define PM8901_GPIO_BASE			(PM8058_GPIO_BASE + \
						PM8058_GPIOS + PM8058_MPPS)
#define PM8901_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8901_GPIO_BASE)
#define PM8901_GPIO_SYS_TO_PM(sys_gpio)		(sys_gpio - PM901_GPIO_BASE)
#define PM8901_IRQ_BASE				(PM8058_IRQ_BASE + \
						NR_PMIC8058_IRQS)

enum {
	GPIO_EXPANDER_IRQ_BASE  = PM8901_IRQ_BASE + NR_PMIC8901_IRQS,
	GPIO_EXPANDER_GPIO_BASE = PM8901_GPIO_BASE + PM8901_MPPS,
	/* CORE expander */
	GPIO_CORE_EXPANDER_BASE = GPIO_EXPANDER_GPIO_BASE,
	GPIO_CLASS_D1_EN        = GPIO_CORE_EXPANDER_BASE,
	GPIO_WLAN_DEEP_SLEEP_N,
	GPIO_LVDS_SHUTDOWN_N,
	GPIO_DISP_RESX_N        = GPIO_LVDS_SHUTDOWN_N,
	GPIO_MS_SYS_RESET_N,
	GPIO_CAP_TS_RESOUT_N,
	GPIO_CAP_GAUGE_BI_TOUT,
	GPIO_ETHERNET_PME,
	GPIO_EXT_GPS_LNA_EN,
	GPIO_MSM_WAKES_BT,
	GPIO_ETHERNET_RESET_N,
	GPIO_HEADSET_DET_N,
	GPIO_USB_UICC_EN,
	GPIO_BACKLIGHT_EN,
	GPIO_EXT_CAMIF_PWR_EN,
	GPIO_BATT_GAUGE_INT_N,
	GPIO_BATT_GAUGE_EN,
	/* DOCKING expander */
	GPIO_DOCKING_EXPANDER_BASE = GPIO_EXPANDER_GPIO_BASE + 16,
	GPIO_MIPI_DSI_RST_N        = GPIO_DOCKING_EXPANDER_BASE,
	GPIO_AUX_JTAG_DET_N,
	GPIO_DONGLE_DET_N,
	GPIO_SVIDEO_LOAD_DET,
	GPIO_SVID_AMP_SHUTDOWN1_N,
	GPIO_SVID_AMP_SHUTDOWN0_N,
	GPIO_SDC_WP,
	GPIO_IRDA_PWDN,
	GPIO_IRDA_RESET_N,
	GPIO_DONGLE_GPIO0,
	GPIO_DONGLE_GPIO1,
	GPIO_DONGLE_GPIO2,
	GPIO_DONGLE_GPIO3,
	GPIO_DONGLE_PWR_EN,
	GPIO_EMMC_RESET_N,
	GPIO_TP_EXP2_IO15,
	/* SURF expander */
	GPIO_SURF_EXPANDER_BASE = GPIO_EXPANDER_GPIO_BASE + (16 * 2),
	GPIO_SD_CARD_DET_1      = GPIO_SURF_EXPANDER_BASE,
	GPIO_SD_CARD_DET_2,
	GPIO_SD_CARD_DET_4,
	GPIO_SD_CARD_DET_5,
	GPIO_UIM3_RST,
	GPIO_SURF_EXPANDER_IO5,
	GPIO_SURF_EXPANDER_IO6,
	GPIO_ADC_I2C_EN,
	GPIO_SURF_EXPANDER_IO8,
	GPIO_SURF_EXPANDER_IO9,
	GPIO_SURF_EXPANDER_IO10,
	GPIO_SURF_EXPANDER_IO11,
	GPIO_SURF_EXPANDER_IO12,
	GPIO_SURF_EXPANDER_IO13,
	GPIO_SURF_EXPANDER_IO14,
	GPIO_SURF_EXPANDER_IO15,
	/* LEFT KB IO expander */
	GPIO_LEFT_KB_EXPANDER_BASE = GPIO_EXPANDER_GPIO_BASE + (16 * 3),
	GPIO_LEFT_LED_1            = GPIO_LEFT_KB_EXPANDER_BASE,
	GPIO_LEFT_LED_2,
	GPIO_LEFT_LED_3,
	GPIO_LEFT_LED_WLAN,
	GPIO_JOYSTICK_EN,
	GPIO_CAP_TS_SLEEP,
	GPIO_LEFT_KB_IO6,
	GPIO_LEFT_LED_5,
	/* RIGHT KB IO expander */
	GPIO_RIGHT_KB_EXPANDER_BASE = GPIO_EXPANDER_GPIO_BASE + (16 * 3) + 8,
	GPIO_RIGHT_LED_1            = GPIO_RIGHT_KB_EXPANDER_BASE,
	GPIO_RIGHT_LED_2,
	GPIO_RIGHT_LED_3,
	GPIO_RIGHT_LED_BT,
	GPIO_WEB_CAMIF_STANDBY,
	GPIO_COMPASS_RST_N,
	GPIO_WEB_CAMIF_RESET_N,
	GPIO_RIGHT_LED_5,
	GPIO_R_ALTIMETER_RESET_N,
	/* FLUID S IO expander */
	GPIO_SOUTH_EXPANDER_BASE,
	GPIO_MIC2_ANCR_SEL = GPIO_SOUTH_EXPANDER_BASE,
	GPIO_MIC1_ANCL_SEL,
	GPIO_HS_MIC4_SEL,
	GPIO_FML_MIC3_SEL,
	GPIO_FMR_MIC5_SEL,
	GPIO_TS_SLEEP,
	GPIO_HAP_SHIFT_LVL_OE,
	GPIO_HS_SW_DIR,
	/* FLUID N IO expander */
	GPIO_NORTH_EXPANDER_BASE,
	GPIO_EPM_3_3V_EN = GPIO_NORTH_EXPANDER_BASE,
	GPIO_EPM_5V_BOOST_EN,
	GPIO_AUX_CAM_2P7_EN,
	GPIO_LED_FLASH_EN,
	GPIO_LED1_GREEN_N,
	GPIO_LED2_RED_N,
	GPIO_FRONT_CAM_RESET_N,
	GPIO_EPM_LVLSFT_EN,
	GPIO_N_ALTIMETER_RESET_N,
};

/*
 * The UI_INTx_N lines are pmic gpio lines which connect i2c
 * gpio expanders to the pm8058.
 */
#define UI_INT1_N 25
#define UI_INT2_N 34
#define UI_INT3_N 14


enum topaz_board_types {
	TOPAZ_PROTO = 0,
	TOPAZ_PROTO2,
	TOPAZ_EVT1,
	TOPAZ_EVT2,
	TOPAZ_EVT3,
	TOPAZ_DVT,
	TOPAZ_PVT,
};
enum topaz3g_board_types {
	TOPAZ3G_PROTO = 0,
	TOPAZ3G_EVT1,
	TOPAZ3G_EVT2,
	TOPAZ3G_EVT3,
	TOPAZ3G_DVT,
	TOPAZ3G_PVT,
};
static bool board_is_topaz_3g_flag = false;
static bool board_is_topaz_3g (void)
{
	return board_is_topaz_3g_flag;
}
static bool board_is_topaz_wifi_flag = true;
static bool board_is_topaz_wifi (void)
{
	return board_is_topaz_wifi_flag;
}
enum opal_board_types {
	OPAL_PROTO = 0,
	OPAL_PROTO2,
	OPAL_EVT1,
	OPAL_EVT2,
	OPAL_EVT3,
	OPAL_DVT,
	OPAL_PVT,
};
enum opal3g_board_types {
	OPAL3G_PROTO = 0,
	OPAL3G_PROTO2,
	OPAL3G_EVT1,
	OPAL3G_EVT2,
	OPAL3G_EVT3,
	OPAL3G_DVT,
	OPAL3G_PVT,
};

static bool board_is_opal_3g_flag = false;
static bool board_is_opal_3g (void)
{
	return board_is_opal_3g_flag;
}
static bool board_is_opal_wifi_flag = false;
static bool board_is_opal_wifi (void)
{
	return board_is_opal_wifi_flag;
}

static u32 board_type = TOPAZ_PROTO;

int config_gpio_tlmm_table(uint32_t *table, int len)
{
	int n, rc = 0;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
	return rc;
}

static struct msm_spm_platform_data msm_spm_data_v1[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x0F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0x94,
		.retention_vlevel = 0x81,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x94,
		.collapse_mid_vlevel = 0x8C,

		.vctl_timeout_us = 50,
	},

	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x0F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x13,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0x94,
		.retention_vlevel = 0x81,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x94,
		.collapse_mid_vlevel = 0x8C,

		.vctl_timeout_us = 50,
	},
};

static struct msm_spm_platform_data msm_spm_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x1F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0xA0,
		.retention_vlevel = 0x8D,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0xA0,
		.collapse_mid_vlevel = 0x98,

		.vctl_timeout_us = 50,
	},

	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x1F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x13,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0xA0,
		.retention_vlevel = 0x8D,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0xA0,
		.collapse_mid_vlevel = 0x98,

		.vctl_timeout_us = 50,
	},
};

static struct msm_acpu_clock_platform_data msm8x60_acpu_clock_data = {
};

static struct regulator_consumer_supply saw_s0_supply =
	REGULATOR_SUPPLY("8901_s0", NULL);
static struct regulator_consumer_supply saw_s1_supply =
	REGULATOR_SUPPLY("8901_s1", NULL);

static struct regulator_init_data saw_s0_init_data = {
		.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 840000,
			.max_uV = 1200000,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &saw_s0_supply,
};

static struct regulator_init_data saw_s1_init_data = {
		.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 840000,
			.max_uV = 1200000,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &saw_s1_supply,
};

static struct platform_device msm_device_saw_s0 = {
	.name          = "saw-regulator",
	.id            = SAW_VREG_ID_S0,
	.dev           = {
		.platform_data = &saw_s0_init_data,
	},
};

static struct platform_device msm_device_saw_s1 = {
	.name          = "saw-regulator",
	.id            = SAW_VREG_ID_S1,
	.dev           = {
		.platform_data = &saw_s1_init_data,
	},
};

/*
 * The smc91x configuration varies depending on platform.
 * The resources data structure is filled in at runtime.
 */
static struct resource smc91x_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name          = "smc91x",
	.id            = 0,
	.num_resources = ARRAY_SIZE(smc91x_resources),
	.resource      = smc91x_resources,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
		.start = 0x1b800000,
		.end   = 0x1b8000ff
	},
	[1] = {
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity = SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type     = SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags        = SMSC911X_USE_16BIT,
	.has_reset_gpio= 1,
	.reset_gpio   = GPIO_ETHERNET_RESET_N
};

static struct platform_device smsc911x_device = {
	.name          = "smsc911x",
	.id            = 0,
	.num_resources = ARRAY_SIZE(smsc911x_resources),
	.resource      = smsc911x_resources,
	.dev           = {
		.platform_data = &smsc911x_config
	}
};

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0x18500000

#define ADM_CHANNEL_CE_0_IN	DMOV_CE_CHAN_IN
#define ADM_CHANNEL_CE_0_OUT	DMOV_CE_CHAN_OUT

#define ADM_CRCI_0_IN		DMOV_CE_CRCI_IN
#define ADM_CRCI_0_OUT		DMOV_CE_CRCI_OUT
#define ADM_CRCI_0_HASH		DMOV_CE_CRCI_HASH

static struct resource qce_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = ADM_CHANNEL_CE_0_IN,
		.end = ADM_CHANNEL_CE_0_OUT,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = ADM_CRCI_0_IN,
		.end = ADM_CRCI_0_IN,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = ADM_CRCI_0_OUT,
		.end = ADM_CRCI_0_OUT,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = ADM_CRCI_0_HASH,
		.end = ADM_CRCI_0_HASH,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qce_resources),
	.resource	= qce_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qce_resources),
	.resource	= qce_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};
#endif

#if defined(CONFIG_HAPTIC_ISA1200) || \
		defined(CONFIG_HAPTIC_ISA1200_MODULE)

static const char *vregs_isa1200_name[] = {
	"8058_s3",
	"8901_l4",
};

static const int vregs_isa1200_val[] = {
	1800000,/* uV */
	2600000,
};
static struct regulator *vregs_isa1200[ARRAY_SIZE(vregs_isa1200_name)];

static int isa1200_power(int vreg_on)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++) {
		rc = vreg_on ? regulator_enable(vregs_isa1200[i]) :
			regulator_disable(vregs_isa1200[i]);
		if (rc < 0) {
			pr_err("%s: vreg %s %s failed (%d)\n",
				__func__, vregs_isa1200_name[i],
				vreg_on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
	}
	return 0;

vreg_fail:
	while (i) {
		int ret;
		ret = !vreg_on ? regulator_enable(vregs_isa1200[i]) :
			regulator_disable(vregs_isa1200[i]);
		if (ret < 0) {
			pr_err("%s: vreg %s %s failed (%d) in err path\n",
				__func__, vregs_isa1200_name[i],
				!vreg_on ? "enable" : "disable", ret);
		}
		i--;
	}

	return rc;
}

static int isa1200_dev_setup(bool enable)
{
	int i, rc;

	if (enable == true) {
		for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++) {
			vregs_isa1200[i] = regulator_get(NULL,
						vregs_isa1200_name[i]);
			if (IS_ERR(vregs_isa1200[i])) {
				pr_err("%s: regulator get of %s failed (%ld)\n",
					__func__, vregs_isa1200_name[i],
					PTR_ERR(vregs_isa1200[i]));
				rc = PTR_ERR(vregs_isa1200[i]);
				goto vreg_get_fail;
			}
			rc = regulator_set_voltage(vregs_isa1200[i],
				vregs_isa1200_val[i], vregs_isa1200_val[i]);
			if (rc) {
				pr_err("%s: regulator_set_voltage(%s) failed\n",
					__func__, vregs_isa1200_name[i]);
				goto vreg_get_fail;
			}
		}

		rc = gpio_request(GPIO_HAP_SHIFT_LVL_OE, "haptics_shft_lvl_oe");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
					__func__, GPIO_HAP_SHIFT_LVL_OE, rc);
			goto vreg_get_fail;
		}

		rc = gpio_direction_output(GPIO_HAP_SHIFT_LVL_OE, 1);
		if (rc) {
			pr_err("%s: Unable to set direction\n", __func__);;
			goto free_gpio;
		}
	} else {
		gpio_set_value(GPIO_HAP_SHIFT_LVL_OE, 0);
		gpio_free(GPIO_HAP_SHIFT_LVL_OE);

		for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++)
			regulator_put(vregs_isa1200[i]);
	}

	return 0;
free_gpio:
	gpio_free(GPIO_HAP_SHIFT_LVL_OE);
vreg_get_fail:
	while (i)
		regulator_put(vregs_isa1200[--i]);
	return rc;
}

#define PMIC_GPIO_HAP_ENABLE   18  /* PMIC GPIO Number 19 */
static struct isa1200_platform_data isa1200_1_pdata = {
	.name = "vibrator",
	.power_on = isa1200_power,
	.dev_setup = isa1200_dev_setup,
	/*gpio to enable haptic*/
	.hap_en_gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE),
	.max_timeout = 15000,
	.mode_ctrl = PWM_GEN_MODE,
	.pwm_fd = {
		.pwm_div = 256,
	},
	.is_erm = false,
	.smart_en = true,
	.ext_clk_en = true,
	.chip_en = 1,
};

static struct i2c_board_info msm_isa1200_board_info[] = {
	{
		I2C_BOARD_INFO("isa1200_1", 0x90>>1),
		.platform_data = &isa1200_1_pdata,
	},
};
#endif

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR * 2] = {
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
		.supported = 1,
		.suspend_enabled = 0,
		.idle_enabled = 0,
		.latency = 4000,
		.residency = 13000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
		.supported = 1,
		.suspend_enabled = 0,
		.idle_enabled = 0,
		.latency = 500,
		.residency = 6000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 2,
		.residency = 0,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
		.supported = 1,
		.suspend_enabled = 0,
		.idle_enabled = 0,
		.latency = 600,
		.residency = 7200,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
		.supported = 1,
		.suspend_enabled = 0,
		.idle_enabled = 0,
		.latency = 500,
		.residency = 6000,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

static struct msm_cpuidle_state msm_cstates[] __initdata = {
	{0, 0, "C0", "WFI",
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT},

	{0, 1, "C1", "STANDALONE_POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE},

	{0, 2, "C2", "POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE},

	{1, 0, "C0", "WFI",
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT},

	{1, 1, "C1", "STANDALONE_POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE},
};

#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)

#define ISP1763_INT_GPIO		172
#define ISP1763_RST_GPIO		152
#define ISP1763_ID				-1

/* Important fields to initialize for Non-PCI based driver*/

/* The base physical memory address assigned for the ISP176x */
#define ISP176x_MEM_BASE 0x1D000000 //base address

/* The memory range assigned for the ISP176x 24k*/
#define ISP176x_MEM_RANGE 0x6000

/* The IRQ number assigned to the ISP176x */
#define ISP176x_IRQ_NUM 	MSM_GPIO_TO_INT(ISP1763_INT_GPIO)
static struct resource isp1763_resources[] = {
	[0] = {
		.flags	= IORESOURCE_MEM,
		.start	= ISP176x_MEM_BASE,
		.end	= (ISP176x_MEM_BASE + ISP176x_MEM_RANGE - 1),		/* 24KB */
	},
	[1] = {
		.flags	= IORESOURCE_IRQ,
		.start	= ISP176x_IRQ_NUM,
		.end	= ISP176x_IRQ_NUM,
	},
};
static void __init msm8x60_cfg_isp1763(void)
{
	isp1763_resources[1].start = gpio_to_irq(ISP1763_INT_GPIO);
	isp1763_resources[1].end = gpio_to_irq(ISP1763_INT_GPIO);
}

static uint32_t msmebi2_tlmm_cfgs[]=
{
	//control
	GPIO_CFG(172, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),   // Interrupt (GPIO(INTERRUPT)
	GPIO_CFG(152, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),   // ISP1763A.RESET_N (GPIO152)
	GPIO_CFG(133, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_CS3_N (GPIO133)
	GPIO_CFG(151, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_OE_N (GPIO151)
	GPIO_CFG(157, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),	 // EBI2_WE_N (GPIO157)

	//Address
	GPIO_CFG(38, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_ADDR_8 (GPIO38)
	GPIO_CFG(123, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_ADDR_7 (GPIO123)
	GPIO_CFG(124, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_ADDR_6 (GPIO124)
	GPIO_CFG(125, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_ADDR_5 (GPIO125)
	GPIO_CFG(126, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_ADDR_4 (GPIO126)
	GPIO_CFG(127, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_ADDR_3 (GPIO127)
	GPIO_CFG(128, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_ADDR_2 (GPIO128)
	GPIO_CFG(129, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_ADDR_1 (GPIO129)
	GPIO_CFG(130, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),   // EBI2_ADDR_0 (GPIO130)

	//Data
	GPIO_CFG(135, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_15 (GPIO135)
	GPIO_CFG(136, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_14 (GPIO136)
	GPIO_CFG(137, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_13 (GPIO137)
	GPIO_CFG(138, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_12 (GPIO138)
	GPIO_CFG(139, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_11 (GPIO139)
	GPIO_CFG(140, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_10 (GPIO140)
	GPIO_CFG(141, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_9 (GPIO141)
	GPIO_CFG(142, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_8 (GPIO142)
	GPIO_CFG(143, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_7 (GPIO143)
	GPIO_CFG(144, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_6 (GPIO144)
	GPIO_CFG(145, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_5 (GPIO145)
	GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_4 (GPIO146)
	GPIO_CFG(147, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_3 (GPIO147)
	GPIO_CFG(148, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_2 (GPIO148)
	GPIO_CFG(149, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_1 (GPIO149)
	GPIO_CFG(150, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),    // EBI2_AD_0 (GPIO150)
};

static int VDD_3V3_EN_3G=0;
#define GPIO_3G_DISABLE_N	171
static int isp1763_setup_gpio(int enable)
{
	int status = 0;

	if (enable) {



		config_gpio_tlmm_table(msmebi2_tlmm_cfgs, ARRAY_SIZE(msmebi2_tlmm_cfgs));
		status = gpio_request(ISP1763_INT_GPIO, "isp1763_usb");
		if (status) {
			pr_err("%s:Failed to request GPIO %d\n",
						__func__, ISP1763_INT_GPIO);
			return status;
		}
		status = gpio_direction_input(ISP1763_INT_GPIO);
		if (status) {
			pr_err("%s:Failed to configure GPIO %d\n",
					__func__, ISP1763_INT_GPIO);
			goto gpio_free_int;
		}
		status = gpio_request(ISP1763_RST_GPIO, "isp1763_usb");
		if (status) {
			pr_err("%s:Failed to request GPIO %d\n",
						__func__, ISP1763_RST_GPIO);
			goto gpio_free_int;
		}
		status = gpio_direction_output(ISP1763_RST_GPIO, 1);
		if (status) {
			pr_err("%s:Failed to configure GPIO %d\n",
					__func__, ISP1763_RST_GPIO);
			goto gpio_free_rst;
		}
		pr_debug("\nISP GPIO configuration done\n");
		return status;
	}

gpio_free_rst:
	gpio_free(ISP1763_RST_GPIO);
gpio_free_int:
	gpio_free(ISP1763_INT_GPIO);

	return status;
}
static struct isp1763_platform_data isp1763_pdata = {
	.reset_gpio	= ISP1763_RST_GPIO,
	.setup_gpio	= isp1763_setup_gpio
};

static struct platform_device isp1763_device = {
	.name          = "isp1763_usb",
	.num_resources = ARRAY_SIZE(isp1763_resources),
	.resource      = isp1763_resources,
	.dev           = {
		.platform_data = &isp1763_pdata
	}
};

#endif

#if defined(CONFIG_USB_PEHCI_HCD)||defined(CONFIG_USB_PEHCI_HCD_MODULE)
#define GPIO_3G_WAKE_N		38
#define GPIO_3G_UIM_CD_N	61

/*HP SamLin@20101216, Begin: Modify function static int isp1763_modem_enable(int on).
  This function only is charge of modem gpio initialize in board level, and should not 
  take responsiblity for enabling/disabling external modem, which will be implemented 
  in exmdm (external modem) driver.
*/
static int isp1763_modem_gpio_init(void)
{
	int rc;

	if( board_is_topaz_3g() || board_is_opal_3g()) {
		if ((board_is_topaz_3g() && board_type == TOPAZ3G_PROTO) || (board_is_opal_3g() && board_type == OPAL3G_PROTO))
			VDD_3V3_EN_3G=158;
		else
			VDD_3V3_EN_3G=82;
		gpio_tlmm_config(GPIO_CFG(VDD_3V3_EN_3G, 0,GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
		rc = gpio_request(VDD_3V3_EN_3G, "VDD_3V3_EN");
		if (rc < 0) {
			printk(KERN_ERR "%s: VDD_3V3_EN gpio %d request failed\n", __func__, VDD_3V3_EN_3G);
		}
		else
		{
			printk(KERN_ERR "%s: VDD_3V3_EN gpio %d statu: %d\n", __func__, VDD_3V3_EN_3G, gpio_get_value(VDD_3V3_EN_3G));		
		}	
		gpio_direction_output(VDD_3V3_EN_3G, 0);

		rc = gpio_request(GPIO_3G_DISABLE_N, "3G_DISABLE_N");
		if (rc < 0) {
			printk(KERN_ERR "%s: GPIO_3G_DISABLE_N gpio %d request failed\n", __func__, GPIO_3G_DISABLE_N);
		}
		else
		{
			printk(KERN_ERR "%s: GPIO_3G_DISABLE_N gpio %d status: %d\n", __func__, GPIO_3G_DISABLE_N, gpio_get_value(GPIO_3G_DISABLE_N));
		}	
		gpio_direction_output(GPIO_3G_DISABLE_N, 0);

		gpio_tlmm_config(GPIO_CFG(GPIO_3G_WAKE_N, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		rc = gpio_request(GPIO_3G_WAKE_N, "3G_WAKE");
		if (rc < 0) {
			printk(KERN_ERR "%s: GPIO_3G_WAKE_N gpio %d request failed\n", __func__, GPIO_3G_WAKE_N);
		}
		gpio_direction_input(GPIO_3G_WAKE_N);

		gpio_tlmm_config(GPIO_CFG(GPIO_3G_UIM_CD_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		rc = gpio_request(GPIO_3G_UIM_CD_N, "UIM_CD");
		if (rc < 0) {
			printk(KERN_ERR "%s: GPIO_3G_UIM_CD_N gpio %d request failed\n", __func__, GPIO_3G_UIM_CD_N);
		}
		gpio_direction_input(GPIO_3G_UIM_CD_N);

		rc = gpio_request(169, "ISP1763A_DACK");
		if (rc < 0) {
			printk(KERN_ERR "%s: ISP1763A_DACK gpio %d request failed\n", __func__, 169);
		}

		rc = gpio_request(29, "ISP1763A_DREQ");
		if (rc < 0) {
			printk(KERN_ERR "%s: ISP1763A_DREQ gpio %d request failed\n", __func__, 29);
		}

		mdelay(300);
		//Free the GPIOs which will be controlled by exmdm (external modem) driver.
		//gpio_free(GPIO_3G_DISABLE_N);
		//gpio_free(VDD_3V3_EN_3G);

		gpio_set_value(VDD_3V3_EN_3G, 1);
		mdelay(25);
		gpio_set_value(GPIO_3G_DISABLE_N, 1);
		gpio_free(GPIO_3G_DISABLE_N);
		gpio_free(VDD_3V3_EN_3G);
	}

	return 0;
}
#endif
/*HP SamLin@20101216, End: Modify function static int isp1763_modem_enable(int on)*/


#ifdef CONFIG_CHARGER_MAX8903
void msm_hsusb_chg_connected(enum chg_type chg_type)
{
	printk("MAX8903_CHARGER: %s : chg_type = %d\n", __func__, chg_type);
	if (chg_type == USB_CHG_TYPE__SDP) {
		max8903_charger_connected(1, POWER_SUPPLY_TYPE_USB);
	} else {
		if ((chg_type == USB_CHG_TYPE__WALLCHARGER) || (chg_type == USB_CHG_TYPE__CARKIT)) {
			max8903_charger_connected(1, POWER_SUPPLY_TYPE_MAINS);
		} else {
			max8903_charger_connected(0, POWER_SUPPLY_TYPE_BATTERY);
		}
	}

}

void msm_hsusb_chg_vbus_draw(unsigned mA)
{
	max8903_charger_draw_current(mA);
}

#define CHG_D_ISET1_GPIO 34
#define CHG_D_ISET2_GPIO 30
static uint32_t max8903_charger_gpio_table_topaz[] = {
	GPIO_CFG(42,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DCM */
	GPIO_CFG(133, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* IUSB */
	GPIO_CFG(140, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* DOK_N */
	GPIO_CFG(41,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CEN */
	GPIO_CFG(33,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* USUS */
	GPIO_CFG(35,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* FLT_N */
	GPIO_CFG(36,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* CHG_N */

	GPIO_CFG(CHG_D_ISET1_GPIO,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(CHG_D_ISET2_GPIO,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),

};
static uint32_t max8903_charger_gpio_table[] = {
	GPIO_CFG(42,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DCM */
	GPIO_CFG(134, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* IUSB */
	GPIO_CFG(86, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* DOK_N */
	GPIO_CFG(41,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CEN */
	GPIO_CFG(33,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* USUS */
	GPIO_CFG(35,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* FLT_N */
	GPIO_CFG(36,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* CHG_N */

	GPIO_CFG(CHG_D_ISET1_GPIO,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(CHG_D_ISET2_GPIO,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),

};

static int max8903_charger_board_init(struct platform_device *pdev)
{
	int ret = 0;

	if (board_is_topaz_wifi()) {
		config_gpio_tlmm_table(max8903_charger_gpio_table_topaz, ARRAY_SIZE(max8903_charger_gpio_table_topaz));
	} else {
		config_gpio_tlmm_table(max8903_charger_gpio_table, ARRAY_SIZE(max8903_charger_gpio_table));
	}
	ret = gpio_request(CHG_D_ISET1_GPIO, "CHG_D_ISET1_GPIO");
	if (ret < 0) {
		pr_err("MAX8903_CHARGER : %s : failed to request %d\n", __func__, CHG_D_ISET1_GPIO);
		return ret;
	}
	ret = gpio_direction_output(CHG_D_ISET1_GPIO, gpio_get_value_cansleep(CHG_D_ISET1_GPIO));
	if (ret < 0) {
		gpio_free(CHG_D_ISET1_GPIO);
		pr_err("MAX8903_CHARGER : %s : failed to set %d\n", __func__, CHG_D_ISET1_GPIO);
		return ret;
	}
	gpio_export(CHG_D_ISET1_GPIO, true);
	gpio_export_link(&pdev->dev, "CHG_D_ISET1_GPIO", CHG_D_ISET1_GPIO);

	ret = gpio_request(CHG_D_ISET2_GPIO, "CHG_D_ISET2_GPIO");
	if (ret < 0) {
		gpio_free(CHG_D_ISET1_GPIO);
		pr_err("MAX8903_CHARGER : %s : failed to request %d\n", __func__, CHG_D_ISET2_GPIO);
		return ret;
	}
	ret = gpio_direction_output(CHG_D_ISET2_GPIO, gpio_get_value_cansleep(CHG_D_ISET2_GPIO));
	if (ret < 0) {
		gpio_free(CHG_D_ISET1_GPIO);
		gpio_free(CHG_D_ISET2_GPIO);
		pr_err("MAX8903_CHARGER : %s : failed to request %d\n", __func__, CHG_D_ISET2_GPIO);
		return ret;
	}
	gpio_export(CHG_D_ISET2_GPIO, true);
	gpio_export_link(&pdev->dev, "CHG_D_ISET2_GPIO", CHG_D_ISET2_GPIO);

	return ret;

}
void max8903_charger_config_DC_current(unsigned mA)
{
	if ((board_is_topaz_wifi() && (board_type >= TOPAZ_EVT1)) || board_is_topaz_3g() || board_is_opal_3g() || board_is_opal_wifi()) {
		if (mA <= 500) {
			//do nothing
		} else {
			if (mA <= 750) {
				gpio_set_value_cansleep(CHG_D_ISET1_GPIO, 0);
				gpio_set_value_cansleep(CHG_D_ISET2_GPIO, 0);
			} else {
				if (mA <= 900) {
					gpio_set_value_cansleep(CHG_D_ISET1_GPIO, 0);
					gpio_set_value_cansleep(CHG_D_ISET2_GPIO, 1);
				} else  {
					if (mA <= 1400) {
						gpio_set_value_cansleep(CHG_D_ISET1_GPIO, 1);
						gpio_set_value_cansleep(CHG_D_ISET2_GPIO, 1);
					} else {
						gpio_set_value_cansleep(CHG_D_ISET1_GPIO, 1);
						gpio_set_value_cansleep(CHG_D_ISET2_GPIO, 0);
					}
				}
			}
		}
	} else {
		if (mA <= 500) {
			//do nothing
		} else {
			if (mA <= 900) {
				gpio_set_value_cansleep(CHG_D_ISET1_GPIO, 0);
				gpio_set_value_cansleep(CHG_D_ISET2_GPIO, 0);
			} else {
				if (mA <= 1000) {
					gpio_set_value_cansleep(CHG_D_ISET1_GPIO, 0);
					gpio_set_value_cansleep(CHG_D_ISET2_GPIO, 1);
				} else  {
					if (mA <= 1500) {
						gpio_set_value_cansleep(CHG_D_ISET1_GPIO, 1);
						gpio_set_value_cansleep(CHG_D_ISET2_GPIO, 0);
					} else {
						gpio_set_value_cansleep(CHG_D_ISET1_GPIO, 1);
						gpio_set_value_cansleep(CHG_D_ISET2_GPIO, 1);
					}
				}
			}
		}
	}
}

static struct max8903_charger_platfom_data max8903_charger_pdata_topaz = {
	.DCM_pin   = 42,
	.DCM_pin_polarity = 1,
	.IUSB_pin  = 133,
	.IUSB_pin_polarity = 0,
	.DOK_N_pin = 140,
	.CEN_N_pin = 41,
	.CEN_N_pin_polarity = 1,
	.USUS_pin  = 33,
	.USUS_pin_polarity = 0,
	.FLT_N_pin = 35,
	.UOK_N_pin = -1,
	.CHG_N_pin = 36,
	.avail_charge_sources = AC_CHG | USB_CHG,
	.board_init = max8903_charger_board_init,
	.config_DC_current = max8903_charger_config_DC_current,

};

static struct max8903_charger_platfom_data max8903_charger_pdata = {
	.DCM_pin   = 42,
	.DCM_pin_polarity = 1,
	.IUSB_pin  = 134,
	.IUSB_pin_polarity = 0,
	.DOK_N_pin = 86,
	.CEN_N_pin = 41,
	.CEN_N_pin_polarity = 1,
	.USUS_pin  = 33,
	.USUS_pin_polarity = 0,
	.FLT_N_pin = 35,
	.UOK_N_pin = -1,
	.CHG_N_pin = 36,
	.avail_charge_sources = AC_CHG | USB_CHG,
	.board_init = max8903_charger_board_init,
	.config_DC_current = max8903_charger_config_DC_current,

};

static struct platform_device max8903_charger_device = {
	.name               = "max8903-charger",
	.id                 = -1,
};

#endif /* CONFIG_CHARGER_MAX8903 */

#ifdef CONFIG_HRES_COUNTER
static int msm_hres_timer_init(void** timer)
{
	return 0;
}

static int msm_hres_timer_release(void* timer)
{
	return 0;
}

#ifdef CONFIG_PM
static int msm_hres_timer_suspend(void *timer)
{
	return 0;
}

static int msm_hres_timer_resume(void *timer)
{
	return 0;
}
#else
#define msm_hres_timer_suspend    NULL
#define msm_hres_timer_resume     NULL
#endif

extern u32 msm_dgt_read_count(void);
static u32 msm_hres_timer_read(void* timer)
{
	return msm_dgt_read_count();
}

extern u32 msm_dgt_convert_usec(u32);
static u32 msm_hres_timer_convert(u32 count)
{
	// Count is in 24.576Mhz terms
	// convert it to uSec
	// return (count * 400) / 2457;
	return msm_dgt_convert_usec(count);
}

static struct hres_counter_platform_data msm_hres_counter_platform_data = {
	.init_hres_timer = msm_hres_timer_init,
	.release_hres_timer = msm_hres_timer_release,
	.suspend_hres_timer = msm_hres_timer_suspend,
	.resume_hres_timer = msm_hres_timer_resume,
	.read_hres_timer = msm_hres_timer_read,
	.convert_hres_timer = msm_hres_timer_convert,
};

static struct platform_device hres_counter_device = {
	.name = "hres_counter",
	.id   = -1,
	.dev  = {
		.platform_data  = &msm_hres_counter_platform_data,
	}
};
#endif

#if defined(CONFIG_A6)
#define A6_INTERNAL_WAKE 1

uint16_t a6_0_set_sbwtck(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	if (board_is_topaz_wifi()) {
		gpio_set_value(157, 1);
	} else {
		if ((board_is_topaz_3g() && (board_type > TOPAZ3G_DVT)) || (board_is_opal_3g() && (board_type > OPAL3G_EVT1))) {
			gpio_set_value(156, 1);
		} else {
			gpio_set_value(68, 1);
		}
	}
	return 0;
}
uint16_t a6_1_set_sbwtck(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(115, 1);
	return 0;
}

uint16_t a6_0_clr_sbwtck(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	if (board_is_topaz_wifi()) {
		gpio_set_value(157, 0);
	} else {
		if ((board_is_topaz_3g() && (board_type > TOPAZ3G_DVT)) || (board_is_opal_3g() && (board_type > OPAL3G_EVT1))) {
			gpio_set_value(156, 0);
		} else {
			gpio_set_value(68, 0);
		}
	}
	return 0;
}

uint16_t a6_1_clr_sbwtck(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(115, 0);
	return 0;
}

uint16_t a6_0_set_sbwtdio(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	if (board_is_topaz_wifi()) {
		gpio_set_value(158, 1);
	} else {
		gpio_set_value(170, 1);
	}
	return 0;
}
uint16_t a6_1_set_sbwtdio(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(116, 1);
	return 0;
}

uint16_t a6_0_clr_sbwtdio(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	if (board_is_topaz_wifi()) {
		gpio_set_value(158, 0);
	} else {
		gpio_set_value(170, 0);
	}
	return 0;
}
uint16_t a6_1_clr_sbwtdio(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(116, 0);
	return 0;
}

uint16_t a6_0_set_in_sbwtdio(void)
{
	if (board_is_topaz_wifi()) {
		gpio_direction_input(158);
	} else {
		gpio_direction_input(170);
	}

	return 0;
}

uint16_t a6_1_set_in_sbwtdio(void)
{
	gpio_direction_input(116);

	return 0;
}

uint16_t a6_0_set_out_sbwtdio(void)
{
	if (board_is_topaz_wifi()) {
		gpio_direction_output(158, 0);
	} else {
		gpio_direction_output(170, 0);
	}

	return 0;
}

uint16_t a6_1_set_out_sbwtdio(void)
{
	gpio_direction_output(116, 0);

	return 0;
}

uint16_t a6_0_get_sbwtdio(void)
{
	if (board_is_topaz_wifi()) {
		return gpio_get_value(158);
	} else {
		return gpio_get_value(170);
	}
}

uint16_t a6_1_get_sbwtdio(void)
{
	return gpio_get_value(116);
}

uint16_t a6_0_set_sbwakeup(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(155, 1);
	udelay(5);
	return 0;
}

uint16_t a6_1_set_sbwakeup(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	if (board_is_topaz_wifi()) {
		gpio_set_value(141, 1);
	} else {
		gpio_set_value(78, 1);
	}
	udelay(5);
	return 0;
}

uint16_t a6_0_clr_sbwakeup(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(155, 0);
	return 0;
}
uint16_t a6_1_clr_sbwakeup(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	if (board_is_topaz_wifi()) {
		gpio_set_value(141, 0);
	} else {
		gpio_set_value(78, 0);
	}
	return 0;
}

/****/
/* per-target functions */
/****/
// delay in usecs
void a6_delay_impl(uint32_t delay_us)
{
	if ((delay_us >> 10) <= MAX_UDELAY_MS) {
		udelay(delay_us);
	}
	else {
		mdelay(delay_us >> 10);
	}
}

struct a6_wake_interface_data {
	int	pwm_channel;
	int	pwm_period;
	int	pwm_duty;
	struct pwm_device *pwm_data;
};

static struct a6_sbw_interface sbw_ops_impl_0;
static struct a6_sbw_interface sbw_ops_impl_1;
static struct a6_wake_ops a6_wake_ops_impl_0;
static struct a6_wake_ops a6_wake_ops_impl_1;


static uint32_t a6_0_sbw_init_gpio_config[] = {
	GPIO_CFG(68, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(155, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* the pull-up config below is for input mode of A6_TDIO. A6 does not always drive
	   this and it floats in input mode unless explicitly pulled-up by host. */
	GPIO_CFG(170, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static uint32_t a6_0_sbw_init_gpio_config_topaz3g_6thbuild[] = {
	GPIO_CFG(156, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(155, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* the pull-up config below is for input mode of A6_TDIO. A6 does not always drive
	   this and it floats in input mode unless explicitly pulled-up by host. */
	GPIO_CFG(170, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static uint32_t a6_1_sbw_init_gpio_config[] = {
	GPIO_CFG(115, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(78, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* the pull-up config below is for input mode of A6_TDIO. A6 does not always drive
	   this and it floats in input mode unless explicitly pulled-up by host. */
	GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static uint32_t a6_0_sbw_deinit_gpio_config[] = {
	GPIO_CFG(68, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(155, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(170, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t a6_0_sbw_deinit_gpio_config_topaz3g_6thbuild[] = {
	GPIO_CFG(156, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(155, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(170, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t a6_1_sbw_deinit_gpio_config[] = {
	GPIO_CFG(115, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(78, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(116, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t a6_0_sbw_init_gpio_config_topaz[] = {
	GPIO_CFG(157, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(155, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* the pull-up config below is for input mode of A6_TDIO. A6 does not always drive
	   this and it floats in input mode unless explicitly pulled-up by host. */
	GPIO_CFG(158, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static uint32_t a6_1_sbw_init_gpio_config_topaz[] = {
	GPIO_CFG(115, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(141, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* the pull-up config below is for input mode of A6_TDIO. A6 does not always drive
	   this and it floats in input mode unless explicitly pulled-up by host. */
	GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static uint32_t a6_0_sbw_deinit_gpio_config_topaz[] = {
	GPIO_CFG(157, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(155, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(158, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t a6_1_sbw_deinit_gpio_config_topaz[] = {
	GPIO_CFG(115, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(141, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(116, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static int a6_sbw_init_imp(struct a6_platform_data* plat_data)
{
	int rc = 0;

	rc = config_gpio_tlmm_table((uint32_t *)plat_data->sbw_init_gpio_config, plat_data->sbw_init_gpio_config_size);
	if (rc < 0) {
		printk(KERN_ERR "%s: failed to configure A6 SBW gpios.\n", __func__);
	}

	return rc;
}

static int a6_sbw_deinit_imp(struct a6_platform_data* plat_data)
{
	int rc = 0;

	rc = config_gpio_tlmm_table((uint32_t *)plat_data->sbw_deinit_gpio_config, plat_data->sbw_deinit_gpio_config_size);
	if (rc < 0) {
		printk(KERN_ERR "%s: failed to de-configure A6 SBW gpios.\n", __func__);
	}

	return rc;
}

static struct a6_platform_data a6_0_platform_data = {
	.dev_name		= A6_DEVICE_0,
	.sbw_wkup_gpio		= 155,
	.sbw_ops		= &sbw_ops_impl_0,
	.wake_ops		= &a6_wake_ops_impl_0,
	.sbw_init		= a6_sbw_init_imp,
	.sbw_deinit		= a6_sbw_deinit_imp,
};

static struct a6_platform_data a6_1_platform_data = {
	.dev_name		= A6_DEVICE_1,
	.sbw_tck_gpio	= 115,
	.sbw_tdio_gpio	= 116,
	.sbw_ops		= &sbw_ops_impl_1,
	.wake_ops		= &a6_wake_ops_impl_1,
	.sbw_init		= a6_sbw_init_imp,
	.sbw_deinit		= a6_sbw_deinit_imp,
};

static uint32_t a6_0_config_data[] = {
		GPIO_CFG(156, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static uint32_t a6_1_config_data[] = {
		GPIO_CFG(132, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};


static uint32_t a6_0_canwakeup_config_data[] = {
		GPIO_CFG(37, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static uint32_t a6_1_canwakeup_config_data[] = {
		GPIO_CFG(94, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static struct i2c_board_info a6_i2c_board_info_0 = {
	I2C_BOARD_INFO( A6_DEVICE_0, (0x62>>1)),
	.platform_data = NULL,
};

static struct i2c_board_info a6_i2c_board_info_1 = {
	I2C_BOARD_INFO( A6_DEVICE_1, (0x64>>1)),
	.platform_data = NULL,
};

#ifdef A6_INTERNAL_WAKE
 struct a6_internal_wake_interface_data {
	int wake_enable: 1;
	// currently restricted to 3 bits: x = [0 - 7] and period = 2^^x
	int wake_period: 9;
	int wake_gpio;
};
static struct a6_internal_wake_interface_data a6_0_wi_data =
{
	.wake_enable = 1,
	.wake_period = 16,
};

static struct a6_internal_wake_interface_data a6_1_wi_data =
{
	.wake_enable = 1,
	.wake_period = 16,
};

static int32_t a6_force_wake(void* data)
{
	struct a6_internal_wake_interface_data *pdata = (struct a6_internal_wake_interface_data *)data;

	pr_debug("%s(%d) : A6 force wakeup!\n", __func__, __LINE__);
	gpio_set_value(pdata->wake_gpio, 1);
	udelay(1);
	gpio_set_value(pdata->wake_gpio, 0);
	udelay(100);
	gpio_set_value(pdata->wake_gpio, 1);

	msleep(30);

	return 0;
}

static int a6_force_sleep(void* data)
{
	struct a6_internal_wake_interface_data *pdata = (struct a6_internal_wake_interface_data *)data;

	pr_debug("%s(%d) : A6 force sleep!\n", __func__, __LINE__);
	gpio_set_value(pdata->wake_gpio, 0);

	return 0;
}

static int a6_internal_wake_enable_state(void* data)
{
	return (((struct a6_internal_wake_interface_data *)data)->wake_enable);
}

static int a6_internal_wake_period(void* data)
{
	return (((struct a6_internal_wake_interface_data *)data)->wake_period);
}
#endif // #ifdef A6_INTERNAL_WAKE

static void __init init_a6(void)
{
	uint32_t* config_data;
	int32_t config_data_size;

	printk(KERN_ERR "Registering a6_0 device.\n");

	if ((board_is_topaz_wifi() && (board_type > TOPAZ_EVT3)) ||
		(board_is_topaz_3g() && (board_type > TOPAZ3G_DVT)) ||
		(board_is_opal_3g() && (board_type > OPAL3G_EVT1))) {
		a6_0_platform_data.pwr_gpio = 37;
		a6_1_platform_data.pwr_gpio = 94;
	} else {
		a6_0_platform_data.pwr_gpio = 156;
		a6_1_platform_data.pwr_gpio= 132;
	}

	if (board_is_topaz_wifi()) {
		a6_0_platform_data.sbw_tck_gpio = 157;
		a6_0_platform_data.sbw_tdio_gpio = 158;
		a6_0_platform_data.sbw_init_gpio_config = a6_0_sbw_init_gpio_config_topaz;
		a6_0_platform_data.sbw_init_gpio_config_size = ARRAY_SIZE(a6_0_sbw_init_gpio_config_topaz);
		a6_0_platform_data.sbw_deinit_gpio_config = a6_0_sbw_deinit_gpio_config_topaz;
		a6_0_platform_data.sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_0_sbw_deinit_gpio_config_topaz);
		
		a6_1_platform_data.sbw_wkup_gpio = 141;
		a6_1_platform_data.sbw_init_gpio_config = a6_1_sbw_init_gpio_config_topaz;
		a6_1_platform_data.sbw_init_gpio_config_size = ARRAY_SIZE(a6_1_sbw_init_gpio_config_topaz);
		a6_1_platform_data.sbw_deinit_gpio_config = a6_1_sbw_deinit_gpio_config_topaz;
		a6_1_platform_data.sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_1_sbw_deinit_gpio_config_topaz);
		
	} else {
		if ((board_is_topaz_3g() && (board_type > TOPAZ3G_DVT)) || (board_is_opal_3g() && (board_type > OPAL3G_EVT1))) {
			a6_0_platform_data.sbw_tck_gpio = 156;
		} else {
			a6_0_platform_data.sbw_tck_gpio = 68;
		}
		a6_0_platform_data.sbw_tdio_gpio = 170;
		if ((board_is_topaz_3g() && (board_type > TOPAZ3G_DVT)) || (board_is_opal_3g() && (board_type > OPAL3G_EVT1))) {
			a6_0_platform_data.sbw_init_gpio_config = a6_0_sbw_init_gpio_config_topaz3g_6thbuild;
			a6_0_platform_data.sbw_init_gpio_config_size = ARRAY_SIZE(a6_0_sbw_init_gpio_config_topaz3g_6thbuild);
			a6_0_platform_data.sbw_deinit_gpio_config = a6_0_sbw_deinit_gpio_config_topaz3g_6thbuild;
			a6_0_platform_data.sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_0_sbw_deinit_gpio_config_topaz3g_6thbuild);
		} else {
			a6_0_platform_data.sbw_init_gpio_config = a6_0_sbw_init_gpio_config;
			a6_0_platform_data.sbw_init_gpio_config_size = ARRAY_SIZE(a6_0_sbw_init_gpio_config);
			a6_0_platform_data.sbw_deinit_gpio_config = a6_0_sbw_deinit_gpio_config;
			a6_0_platform_data.sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_0_sbw_deinit_gpio_config);
		}
		a6_1_platform_data.sbw_wkup_gpio = 78;
		a6_1_platform_data.sbw_init_gpio_config = a6_1_sbw_init_gpio_config;
		a6_1_platform_data.sbw_init_gpio_config_size = ARRAY_SIZE(a6_1_sbw_init_gpio_config);
		a6_1_platform_data.sbw_deinit_gpio_config = a6_1_sbw_deinit_gpio_config;
		a6_1_platform_data.sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_1_sbw_deinit_gpio_config);
	}

	a6_i2c_board_info_0.platform_data = &a6_0_platform_data;
	i2c_register_board_info(MSM_GSBI8_QUP_I2C_BUS_ID, &a6_i2c_board_info_0, 1);
	
	a6_i2c_board_info_1.platform_data = &a6_1_platform_data;
	i2c_register_board_info(MSM_GSBI8_QUP_I2C_BUS_ID, &a6_i2c_board_info_1, 1);

	if (board_is_topaz_wifi()) {
		config_data = a6_0_sbw_init_gpio_config_topaz;
		config_data_size = ARRAY_SIZE(a6_0_sbw_init_gpio_config_topaz);
	} else {
		if ((board_is_topaz_3g() && (board_type > TOPAZ3G_DVT)) || (board_is_opal_3g() && (board_type > OPAL3G_EVT1))) {
			config_data = a6_0_sbw_init_gpio_config_topaz3g_6thbuild;
			config_data_size = ARRAY_SIZE(a6_0_sbw_init_gpio_config_topaz3g_6thbuild);
		} else {
			config_data = a6_0_sbw_init_gpio_config;
			config_data_size = ARRAY_SIZE(a6_0_sbw_init_gpio_config);
		}
	}
	config_gpio_tlmm_table(config_data, config_data_size);

	if (board_is_topaz_wifi()) {
		config_data = a6_1_sbw_init_gpio_config_topaz;
		config_data_size = ARRAY_SIZE(a6_1_sbw_init_gpio_config_topaz);
	} else {
		config_data = a6_1_sbw_init_gpio_config;
		config_data_size = ARRAY_SIZE(a6_1_sbw_init_gpio_config);
	}
	config_gpio_tlmm_table(config_data, config_data_size);

	/* no change for dvt boards */
	if ((board_is_topaz_wifi() && (board_type > TOPAZ_EVT3)) ||
		(board_is_topaz_3g() && (board_type > TOPAZ3G_DVT)) ||
		(board_is_opal_3g() && (board_type > OPAL3G_EVT1))) {
		config_data = a6_0_canwakeup_config_data;
		config_data_size = ARRAY_SIZE(a6_0_canwakeup_config_data);
		config_gpio_tlmm_table(config_data, config_data_size);

		config_data = a6_1_canwakeup_config_data;
		config_data_size = ARRAY_SIZE(a6_1_canwakeup_config_data);
		config_gpio_tlmm_table(config_data, config_data_size);
	} else {
		config_data = a6_0_config_data;
		config_data_size = ARRAY_SIZE(a6_0_config_data);
		config_gpio_tlmm_table(config_data, config_data_size);

		config_data = a6_1_config_data;
		config_data_size = ARRAY_SIZE(a6_1_config_data);
		config_gpio_tlmm_table(config_data, config_data_size);
	}

	a6_0_platform_data.wake_ops = NULL;
	a6_1_platform_data.wake_ops = NULL;
#if A6_INTERNAL_WAKE
	((struct a6_platform_data*)a6_i2c_board_info_0.platform_data)->wake_ops = &a6_wake_ops_impl_0;
	a6_0_wi_data.wake_gpio = ((struct a6_platform_data *)a6_i2c_board_info_0.platform_data)->sbw_wkup_gpio;
	a6_wake_ops_impl_0.data = &a6_0_wi_data;
	a6_wake_ops_impl_0.enable_periodic_wake = NULL;
	a6_wake_ops_impl_0.disable_periodic_wake = NULL;
	a6_wake_ops_impl_0.internal_wake_enable_state = &a6_internal_wake_enable_state;
	a6_wake_ops_impl_0.internal_wake_period = &a6_internal_wake_period;
	a6_wake_ops_impl_0.force_wake = &a6_force_wake;
	a6_wake_ops_impl_0.force_sleep = &a6_force_sleep;

	((struct a6_platform_data*)a6_i2c_board_info_1.platform_data)->wake_ops = &a6_wake_ops_impl_1;
	a6_1_wi_data.wake_gpio = ((struct a6_platform_data *)a6_i2c_board_info_1.platform_data)->sbw_wkup_gpio;
	a6_wake_ops_impl_1.data = &a6_1_wi_data;
	a6_wake_ops_impl_1.enable_periodic_wake = NULL;
	a6_wake_ops_impl_1.disable_periodic_wake = NULL;
	a6_wake_ops_impl_1.internal_wake_enable_state = &a6_internal_wake_enable_state;
	a6_wake_ops_impl_1.internal_wake_period = &a6_internal_wake_period;
	a6_wake_ops_impl_1.force_wake = &a6_force_wake;
	a6_wake_ops_impl_1.force_sleep = &a6_force_sleep;
#else
	((struct a6_platform_data*)a6_i2c_board_info_0.platform_data)->wake_ops = NULL;
	((struct a6_platform_data*)a6_i2c_board_info_1.platform_data)->wake_ops = NULL;

#endif

	sbw_ops_impl_0.a6_per_device_interface.SetSBWTCK = &a6_0_set_sbwtck;
	sbw_ops_impl_0.a6_per_device_interface.ClrSBWTCK = &a6_0_clr_sbwtck;
	sbw_ops_impl_0.a6_per_device_interface.SetSBWTDIO = &a6_0_set_sbwtdio;
	sbw_ops_impl_0.a6_per_device_interface.ClrSBWTDIO = &a6_0_clr_sbwtdio;
	sbw_ops_impl_0.a6_per_device_interface.SetInSBWTDIO = &a6_0_set_in_sbwtdio;
	sbw_ops_impl_0.a6_per_device_interface.SetOutSBWTDIO = &a6_0_set_out_sbwtdio;
	sbw_ops_impl_0.a6_per_device_interface.GetSBWTDIO = &a6_0_get_sbwtdio;
	sbw_ops_impl_0.a6_per_device_interface.SetSBWAKEUP = &a6_0_set_sbwakeup;
	sbw_ops_impl_0.a6_per_device_interface.ClrSBWAKEUP = &a6_0_clr_sbwakeup;
	sbw_ops_impl_0.a6_per_target_interface.delay = a6_delay_impl;

	sbw_ops_impl_1.a6_per_device_interface.SetSBWTCK = &a6_1_set_sbwtck;
	sbw_ops_impl_1.a6_per_device_interface.ClrSBWTCK = &a6_1_clr_sbwtck;
	sbw_ops_impl_1.a6_per_device_interface.SetSBWTDIO = &a6_1_set_sbwtdio;
	sbw_ops_impl_1.a6_per_device_interface.ClrSBWTDIO = &a6_1_clr_sbwtdio;
	sbw_ops_impl_1.a6_per_device_interface.SetInSBWTDIO = &a6_1_set_in_sbwtdio;
	sbw_ops_impl_1.a6_per_device_interface.SetOutSBWTDIO = &a6_1_set_out_sbwtdio;
	sbw_ops_impl_1.a6_per_device_interface.GetSBWTDIO = &a6_1_get_sbwtdio;
	sbw_ops_impl_1.a6_per_device_interface.SetSBWAKEUP = &a6_1_set_sbwakeup;
	sbw_ops_impl_1.a6_per_device_interface.ClrSBWAKEUP = &a6_1_clr_sbwakeup;
	sbw_ops_impl_1.a6_per_target_interface.delay = a6_delay_impl;
}
#endif // CONFIG_A6

#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_MSM)
static struct regulator *ldo6_3p3;
static struct regulator *ldo7_1p8;
static struct regulator *vdd_cx;
#define PMICID_INT		PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 36)
notify_vbus_state notify_vbus_state_func_ptr;

#ifdef CONFIG_USB_EHCI_MSM
#define USB_PMIC_ID_DET_DELAY	msecs_to_jiffies(100)
struct delayed_work pmic_id_det;
static void pmic_id_detect(struct work_struct *w)
{
	int val = gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS(36));
	pr_info("%s(): gpio_read_value = %d\n", __func__, val);

	if (notify_vbus_state_func_ptr)
		(*notify_vbus_state_func_ptr) (val);
}

static irqreturn_t pmic_id_on_irq(int irq, void *data)
{
	/*
	 * Spurious interrupts are observed on pmic gpio line
	 * even though there is no state change on USB ID. Schedule the
	 * work to to allow debounce on gpio
	 */
	schedule_delayed_work(&pmic_id_det, USB_PMIC_ID_DET_DELAY);

	return IRQ_HANDLED;
}

static int msm_hsusb_pmic_id_notif_init(void (*callback)(int online), int init)
{
	unsigned ret = -ENODEV;

	if (!callback)
		return -EINVAL;

	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 2) {
		pr_debug("%s: USB_ID pin is not routed to PMIC"
					"on V1 surf/ffa\n", __func__);
		return -ENOTSUPP;
	}

	if (machine_is_msm8x60_ffa()) {
		pr_debug("%s: USB_ID is not routed to PMIC"
			"on V2 ffa\n", __func__);
		return -ENOTSUPP;
	}

	if (init) {
		notify_vbus_state_func_ptr = callback;
		ret = pm8901_mpp_config_digital_out(1,
			PM8901_MPP_DIG_LEVEL_L5, 1);
		if (ret) {
			pr_err("%s: MPP2 configuration failed\n", __func__);
			return -ENODEV;
		}
		INIT_DELAYED_WORK(&pmic_id_det, pmic_id_detect);
		ret = request_threaded_irq(PMICID_INT, NULL, pmic_id_on_irq,
			(IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING),
						"msm_otg_id", NULL);
		if (ret) {
			pm8901_mpp_config_digital_out(1,
					PM8901_MPP_DIG_LEVEL_L5, 0);
			pr_err("%s:pmic_usb_id interrupt registration failed",
					__func__);
			return ret;
		}
	} else {
		free_irq(PMICID_INT, 0);
		cancel_delayed_work_sync(&pmic_id_det);
		notify_vbus_state_func_ptr = NULL;
		ret = pm8901_mpp_config_digital_out(1,
			PM8901_MPP_DIG_LEVEL_L5, 0);
		if (ret) {
			pr_err("%s:MPP2 configuration failed\n", __func__);
			return -ENODEV;
		}
	}
	return 0;
}
#endif

#define USB_PHY_SUSPEND_MIN_VDD_DIG_VOL		750000
#define USB_PHY_OPERATIONAL_MIN_VDD_DIG_VOL	1000000
#define USB_PHY_MAX_VDD_DIG_VOL			1320000
static int msm_hsusb_init_vddcx(int init)
{
	int ret = 0;

	if (init) {
		vdd_cx = regulator_get(NULL, "8058_s1");
		if (IS_ERR(vdd_cx)) {
			return PTR_ERR(vdd_cx);
		}

		ret = regulator_set_voltage(vdd_cx,
				USB_PHY_OPERATIONAL_MIN_VDD_DIG_VOL,
				USB_PHY_MAX_VDD_DIG_VOL);
		if (ret) {
			pr_err("%s: unable to set the voltage for regulator"
				"vdd_cx\n", __func__);
			regulator_put(vdd_cx);
			return ret;
		}

		ret = regulator_enable(vdd_cx);
		if (ret) {
			pr_err("%s: unable to enable regulator"
				"vdd_cx\n", __func__);
			regulator_put(vdd_cx);
		}
	} else {
		ret = regulator_disable(vdd_cx);
		if (ret) {
			pr_err("%s: Unable to disable the regulator:"
				"vdd_cx\n", __func__);
			return ret;
		}

		regulator_put(vdd_cx);
	}

	return ret;
}

static int msm_hsusb_config_vddcx(int high)
{
	int max_vol = USB_PHY_MAX_VDD_DIG_VOL;
	int min_vol;
	int ret;

	if (high)
		min_vol = USB_PHY_OPERATIONAL_MIN_VDD_DIG_VOL;
	else
		min_vol = USB_PHY_SUSPEND_MIN_VDD_DIG_VOL;

	ret = regulator_set_voltage(vdd_cx, min_vol, max_vol);
	if (ret) {
		pr_err("%s: unable to set the voltage for regulator"
			"vdd_cx\n", __func__);
		return ret;
	}

	pr_debug("%s: min_vol:%d max_vol:%d\n", __func__, min_vol, max_vol);

	return ret;
}

static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		ldo6_3p3 = regulator_get(NULL, "8058_l6");
		if (IS_ERR(ldo6_3p3))
			return PTR_ERR(ldo6_3p3);

		ldo7_1p8 = regulator_get(NULL, "8058_l7");
		if (IS_ERR(ldo7_1p8)) {
			regulator_put(ldo6_3p3);
			return PTR_ERR(ldo7_1p8);
		}

		regulator_set_voltage(ldo7_1p8, 1800000, 1800000);
		if (machine_is_msm8x60_topaz() || machine_is_msm8x60_opal())
			regulator_set_voltage(ldo6_3p3, 3300000, 3300000);
		else
			regulator_set_voltage(ldo6_3p3, 3050000, 3050000);
	} else {
		regulator_put(ldo6_3p3);
		regulator_put(ldo7_1p8);
	}
	return 0;
}

static int msm_hsusb_ldo_enable(int on)
{
	static int ldo_status;
	int ret = 0;

	if (!ldo7_1p8 || IS_ERR(ldo7_1p8)) {
		pr_err("%s: ldo7_1p8 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (!ldo6_3p3 || IS_ERR(ldo6_3p3)) {
		pr_err("%s: ldo6_3p3 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (ldo_status == on)
		return 0;

	ldo_status = on;

	if (on) {
		ret = regulator_enable(ldo7_1p8);
		if (ret) {
			pr_err("%s: Unable to enable the regulator:"
				"ldo7_1p8\n", __func__);
			ldo_status = !on;
			return ret;
		}
		ret = regulator_enable(ldo6_3p3);
		if (ret) {
			pr_err("%s: Unable to enable the regulator:"
				"ldo6_3p3\n", __func__);
			regulator_disable(ldo7_1p8);
			ldo_status = !on;
			return ret;
		}
	} else {
		/* calling regulator_disable when its already disabled might
		 * * print WARN_ON. Trying to avoid it by regulator_is_enable
		 * * */
		if (regulator_is_enabled(ldo6_3p3)) {
			ret = regulator_disable(ldo6_3p3);
			if (ret) {
				pr_err("%s: Unable to disable the regulator:"
					"ldo6_3p3\n", __func__);
				ldo_status = !on;
				return ret;
			}
		}

		if (regulator_is_enabled(ldo7_1p8)) {
			ret = regulator_disable(ldo7_1p8);
			if (ret) {
				pr_err("%s: Unable to enable the regulator:"
					" ldo7_1p8\n", __func__);
				ldo_status = !on;
				return ret;
			}
		}

	}

	pr_debug("reg (%s)\n", on ? "ENABLED" : "DISABLED");
	return 0;
 }
#endif
#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	static struct regulator *votg_5v_switch;
	static struct regulator *ext_5v_reg;
	static int vbus_is_on;

	/* If VBUS is already on (or off), do nothing. */
	if (on == vbus_is_on)
		return;

	if (!votg_5v_switch) {
		votg_5v_switch = regulator_get(NULL, "8901_usb_otg");
		if (IS_ERR(votg_5v_switch)) {
			pr_err("%s: unable to get votg_5v_switch\n", __func__);
			return;
		}
	}
	if (!ext_5v_reg) {
		ext_5v_reg = regulator_get(NULL, "8901_mpp0");
		if (IS_ERR(ext_5v_reg)) {
			pr_err("%s: unable to get ext_5v_reg\n", __func__);
			return;
		}
	}
	if (on) {
		if (regulator_enable(ext_5v_reg)) {
			pr_err("%s: Unable to enable the regulator:"
					" ext_5v_reg\n", __func__);
			return;
		}
		if (regulator_enable(votg_5v_switch)) {
			pr_err("%s: Unable to enable the regulator:"
					" votg_5v_switch\n", __func__);
			return;
		}
	} else {
		if (regulator_disable(votg_5v_switch))
			pr_err("%s: Unable to enable the regulator:"
				" votg_5v_switch\n", __func__);
		if (regulator_disable(ext_5v_reg))
			pr_err("%s: Unable to enable the regulator:"
				" ext_5v_reg\n", __func__);
	}

	vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
	.power_budget	= 500,
};

static struct msm_usb_host_platform_data msm_usb_host_pdata_topaz = {
        .phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
        .power_budget   = 390,
};

#endif

#ifdef CONFIG_BATTERY_MSM8X60
static int msm_hsusb_pmic_vbus_notif_init(void (*callback)(int online),
								int init)
{
	int ret = -ENOTSUPP;

	/* ID and VBUS lines are connected to pmic on 8660.V2.SURF,
	 * hence, irrespective of either peripheral only mode or
	 * OTG (host and peripheral) modes, can depend on pmic for
	 * vbus notifications
	 */
	if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2)
			&& (machine_is_msm8x60_surf())) {
		if (init)
			ret = msm_charger_register_vbus_sn(callback);
		else {
			msm_charger_unregister_vbus_sn(callback);
			ret = 0;
		}
	} else {
#if !defined(CONFIG_USB_EHCI_MSM)
	if (init)
		ret = msm_charger_register_vbus_sn(callback);
	else {
		msm_charger_unregister_vbus_sn(callback);
		ret = 0;
	}
#endif
	}
	return ret;
}
#endif

#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_MSM)
static struct msm_otg_platform_data msm_otg_pdata = {
	/* if usb link is in sps there is no need for
	 * usb pclk as dayatona fabric clock will be
	 * used instead
	 */
	.usb_in_sps = 1,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.se1_gating		 = SE1_GATING_DISABLE,
#ifdef CONFIG_USB_EHCI_MSM
	.pmic_id_notif_init = msm_hsusb_pmic_id_notif_init,
#endif
#ifdef CONFIG_USB_EHCI_MSM
	.vbus_power = msm_hsusb_vbus_power,
#endif
#ifdef CONFIG_BATTERY_MSM8X60
	.pmic_vbus_notif_init	= msm_hsusb_pmic_vbus_notif_init,
#endif
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.config_vddcx            = msm_hsusb_config_vddcx,
	.init_vddcx              = msm_hsusb_init_vddcx,
#ifdef CONFIG_BATTERY_MSM8X60
	.chg_vbus_draw = msm_charger_vbus_draw,
#endif
#if defined(CONFIG_CHARGER_MAX8903)
	.chg_connected	 = msm_hsusb_chg_connected,
	.chg_vbus_draw	 = msm_hsusb_chg_vbus_draw,
#endif
};
#endif


#ifdef CONFIG_USB_ANDROID
static char *usb_functions_default[] = {
	"diag",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_default_adb[] = {
	"diag",
	"adb",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
	"adb",
#ifdef CONFIG_USB_F_SERIAL
	"modem",
	"nmea",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif
	"usb_mass_storage",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x9026,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions	= usb_functions_default,
	},
	{
		.product_id	= 0x9025,
		.num_functions	= ARRAY_SIZE(usb_functions_default_adb),
		.functions	= usb_functions_default_adb,
	},
	{
		.product_id	= 0xf00e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x9024,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
};
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Qualcomm Incorporated",
	.product        = "Mass storage",
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x05C6,
	.vendorDescr	= "Qualcomm Incorporated",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x05C6,
	.product_id	= 0x9026,
	.version	= 0x0100,
	.product_name		= "Qualcomm HSUSB Device",
	.manufacturer_name	= "Qualcomm Incorporated",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	//android_usb_pdata.serial_number = serialno;  //Use default serialno instead of serialno from cmdline.
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0x05300000,
		.end	= 0x05300000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
	.name = "msm_vpe",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_vpe_resources),
	.resource = msm_vpe_resources,
};
#endif

#ifdef CONFIG_MSM_CAMERA
#define VFE_CAMIF_TIMER1_GPIO 29
#define VFE_CAMIF_TIMER2_GPIO 30
#define VFE_CAMIF_TIMER3_GPIO_INT 31

int msm_cam_gpio_tbl[] = {
	32,/*CAMIF_MCLK*/
	47,/*CAMIF_I2C_DATA*/
	48,/*CAMIF_I2C_CLK*/
#if defined(CONFIG_MACH_MSM8X60_TOPAZ)|| defined(CONFIG_MACH_MSM8X60_OPAL)	
	107,/*STANDBY*/
#else
	105,/*STANDBY*/
#endif
};

enum msm_cam_stat{
	MSM_CAM_OFF,
	MSM_CAM_ON,
};

static int config_gpio_table(enum msm_cam_stat stat)
{
	int rc = 0, i = 0;
	if (stat == MSM_CAM_ON) {
		for (i = 0; i < ARRAY_SIZE(msm_cam_gpio_tbl); i++) {
			rc = gpio_request(msm_cam_gpio_tbl[i], "CAM_GPIO");
			if (unlikely(rc < 0)) {
				pr_err("%s not able to get gpio\n", __func__);
				for (i--; i >= 0; i--)
					gpio_free(msm_cam_gpio_tbl[i]);
				break;
			}
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(msm_cam_gpio_tbl); i++)
			gpio_free(msm_cam_gpio_tbl[i]);
	}
	return rc;
}

static int config_camera_on_gpios(void)
{
	int rc = 0;

	rc = config_gpio_table(MSM_CAM_ON);
	if (rc < 0) {
		printk(KERN_ERR "%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	rc = gpio_request(GPIO_EXT_CAMIF_PWR_EN, "CAM_EN");
	if (rc < 0) {
		config_gpio_table(MSM_CAM_OFF);
		printk(KERN_ERR "%s: CAMSENSOR gpio %d request"
			"failed\n", __func__, GPIO_EXT_CAMIF_PWR_EN);
		return rc;
	}
	gpio_direction_output(GPIO_EXT_CAMIF_PWR_EN, 0);
	mdelay(20);
	gpio_set_value_cansleep(GPIO_EXT_CAMIF_PWR_EN, 1);
	return rc;
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(MSM_CAM_OFF);

	gpio_set_value_cansleep(GPIO_EXT_CAMIF_PWR_EN, 0);
	gpio_free(GPIO_EXT_CAMIF_PWR_EN);
}

//As 1.3M camera and 5M camera will share the same pin, so add switch to avoid multi-access
static bool HP_CAM_CONFIGURED=false;

#define DONOT_USE_GPIOMUX
#ifdef DONOT_USE_GPIOMUX 
static uint32_t camera_vx6953_off_gpio_table[] = {
	GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), //I2C_DATA the same with surf
	GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), //I2C_CLK the same with surf
	//Change MCLK driving current from 2ma to 8ma to get better MIPI signal.
	GPIO_CFG(32, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), //CAM_MCLK, the same with surf
};

static uint32_t camera_vx6953_on_gpio_table[] = {
	GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),  //I2C_DATA the same with surf
	GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),  //I2C_CLK the same with surf
	//Change MCLK driving current from 2ma to 8ma to get better MIPI signal.
	GPIO_CFG(32, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),  //CAM_MCLK, the same with surf
};
#endif


//As VX6953 share the same pins as well as sources with MT9M113, except that
//HARD_STANDBY pin as well as CAM_RESET pin has difference
static int config_camera_on_gpios_vx6953(void)
{    
	int rc = 0;
	pr_info("Bob %s:++\n", __func__);
	if(false==HP_CAM_CONFIGURED)
		{
			#ifdef DONOT_USE_GPIOMUX 
			rc=config_gpio_tlmm_table(camera_vx6953_on_gpio_table,
							 ARRAY_SIZE(camera_vx6953_on_gpio_table));
			#else	
			rc = config_gpio_table(MSM_CAM_ON);
			#endif
			if (rc < 0) {
				printk(KERN_ERR "%s: CAMSENSOR gpio table request"
				"failed\n", __func__);
				return rc;
			 }
		  HP_CAM_CONFIGURED=true;
		}
	if( board_is_opal_wifi() || board_is_opal_3g())
	  {
			struct pm8058_gpio_cfg {
				int 			   gpio;
				struct pm8058_gpio cfg;
			};
			struct pm8058_gpio_cfg camera_pwdn_cfg={
				/* PM8058 GPIO_09 5M PWDN */
				8,
				{
					.direction	= PM_GPIO_DIR_OUT,
					.output_value	= 1,
					.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
					.pull		= PM_GPIO_PULL_NO,
					.out_strength	= PM_GPIO_STRENGTH_HIGH,
					.function	= PM_GPIO_FUNC_NORMAL,
					.vin_sel	= PM_GPIO_VIN_S3, /*1.8v*/
					.inv_int_pol	= 0,
				}
			};
			rc = pm8058_gpio_config(camera_pwdn_cfg.gpio,
					&camera_pwdn_cfg.cfg);
			if (rc < 0) {
				pr_err("%s pmic gpio 8 config failed 1\n",
						__func__);
				return rc;
			}
		
	  }
	else
	  {
	    //Reserved for further projects
	  }
	return 0;
}

static void config_camera_off_gpios_vx6953(void)
{
 	int rc=0;
 
  pr_debug("Bob %s:++\n", __func__);
	
	if( board_is_opal_wifi() || board_is_opal_3g())
	  {

		struct pm8058_gpio_cfg {
		int                gpio;
		struct pm8058_gpio cfg;
		};
		struct pm8058_gpio_cfg camera_pwdn_cfg={
			/* PM8058 GPIO_09  Opal 5M Camera PWDN*/
			8,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_S3, /*1.8v*/
				.inv_int_pol	= 0,
			}
		};

		rc = pm8058_gpio_config(camera_pwdn_cfg.gpio,
				  &camera_pwdn_cfg.cfg);
		if (rc < 0) {
			pr_err("%s pmic gpio config failed\n",
				  __func__);
			return;
		}
	  }
	else
	  {
          //Reserved for further projects
	  }
   if(true==HP_CAM_CONFIGURED)
	 {
		 #ifdef DONOT_USE_GPIOMUX
		  config_gpio_tlmm_table(camera_vx6953_off_gpio_table,
			  ARRAY_SIZE(camera_vx6953_off_gpio_table));
		 #else
		  config_gpio_table(MSM_CAM_OFF);
		 #endif
		 HP_CAM_CONFIGURED=false;
   	}

}


static int config_camera_on_gpios_web_cam(void)
{
	int rc = 0;
	rc = config_gpio_table(MSM_CAM_ON);
	if (rc < 0) {
		printk(KERN_ERR "%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	rc = gpio_request(GPIO_WEB_CAMIF_STANDBY, "CAM_EN");
	if (rc < 0) {
		config_gpio_table(MSM_CAM_OFF);
		pr_err(KERN_ERR "%s: CAMSENSOR gpio %d request"
			"failed\n", __func__, GPIO_WEB_CAMIF_STANDBY);
		return rc;
	}
	gpio_direction_output(GPIO_WEB_CAMIF_STANDBY, 0);
	return rc;
}

static void config_camera_off_gpios_web_cam(void)
{
	config_gpio_table(MSM_CAM_OFF);

	gpio_set_value_cansleep(GPIO_WEB_CAMIF_STANDBY, 1);
	gpio_free(GPIO_WEB_CAMIF_STANDBY);
}

#ifdef DONOT_USE_GPIOMUX 
static uint32_t camera_mt9m113_off_gpio_table[] = {
	GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), //I2C_DATA the same with surf
	GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), //I2C_CLK the same with surf
	//Change MCLK driving current from 2ma to 8ma to get better MIPI signal.
	GPIO_CFG(32, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), //CAM_MCLK, the same with surf

};

static uint32_t camera_mt9m113_on_gpio_table[] = {
	GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),  //I2C_DATA the same with surf
	GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),  //I2C_CLK the same with surf
	//Change MCLK driving current from 2ma to 8ma to get better MIPI signal.
	GPIO_CFG(32, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),  //CAM_MCLK, the same with surf
};
#endif

static int config_camera_on_gpios_web_cam_mt9m113(void)
{
	int rc = 0;
	pr_debug("%s: Bob camera_on_gpios_mt9m113++\n", __func__);
	//Move camera reset pin related operations here.  Bob Zhu 11-09
	//Move LDO related operations back to msm_io_8x60 and match QCT default design
	if(false==HP_CAM_CONFIGURED)
		{
          #ifdef DONOT_USE_GPIOMUX 
			rc=config_gpio_tlmm_table(camera_mt9m113_on_gpio_table,
						     ARRAY_SIZE(camera_mt9m113_on_gpio_table));
		  #else	
			rc = config_gpio_table(MSM_CAM_ON);
		  #endif
			if (rc < 0) {
				printk(KERN_ERR "%s: CAMSENSOR gpio table request"
						"failed\n", __func__);
				return rc;
			 }
		  HP_CAM_CONFIGURED=true;	
		}

	if( board_is_topaz_wifi())
	{
	    /*GPIO106 Topaz_WIFI 1.3M camera reset pin*/
		rc = gpio_request(106, "CAM_RESET");
		if (rc < 0) {
			printk(KERN_ERR "%s: CAMSENSOR RESET gpio 106 request"
						"failed\n", __func__);
			return rc;
		}
		gpio_direction_output(106, 1);
		mdelay(20);
		gpio_direction_output(106, 0);
		mdelay(20);
		gpio_direction_output(106, 1);
		mdelay(20);
	}
	else if( board_is_topaz_3g()||board_is_opal_wifi() || board_is_opal_3g() )
	{
		struct pm8058_gpio_cfg {
			int                gpio;
			struct pm8058_gpio cfg;
		};
		struct pm8058_gpio_cfg camera_reset_cfg={
			/* PM8058 GPIO_08 TOPAZ_3G&&Opal 1.3M Camera Reset Pin */
			7,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_S3, /*1.8v*/
				.inv_int_pol	= 0,
			}
		};
		rc = pm8058_gpio_config(camera_reset_cfg.gpio,
				&camera_reset_cfg.cfg);
		if (rc < 0) {
			pr_err("%s pmic gpio 8 config failed 1\n",
					__func__);
			return rc;
		}
		mdelay(20);

		camera_reset_cfg.cfg.output_value=0;

		rc = pm8058_gpio_config(camera_reset_cfg.gpio,
				&camera_reset_cfg.cfg);
		if (rc < 0) {
			pr_err("%s pmic gpio 8 config failed 2\n",
				__func__);
			return rc;
		}

		mdelay(20);

		camera_reset_cfg.cfg.output_value=1;

		rc = pm8058_gpio_config(camera_reset_cfg.gpio,
					&camera_reset_cfg.cfg);
		if (rc < 0) {
			pr_err("%s pmic gpio 8 config failed 3\n",
					__func__);
			return rc;
		}
		mdelay(20);
	}
	else
	{
		//Reserved for further projects.
	}

	pr_debug("--- %s\n", __func__);
	return 0;
}

static void config_camera_off_gpios_web_cam_mt9m113(void)
{

	int rc=0;

	pr_debug("+++ %s\n", __func__);
	//Move camera reset pin related operations here.  Bob Zhu 11-09

	if( board_is_topaz_wifi() )
	{

		gpio_direction_output(106, 1);
		mdelay(20);
		gpio_free(106);
	}
	else if( board_is_topaz_3g()||board_is_opal_wifi() || board_is_opal_3g())
	{
		struct pm8058_gpio_cfg {
		int                gpio;
		struct pm8058_gpio cfg;
		};
		struct pm8058_gpio_cfg camera_reset_cfg={
			/* PM8058 GPIO_08 TOPAZ_3G &&Opal 1.3M Camera Reset Pin*/
			7,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_S3, /*1.8v*/
				.inv_int_pol	= 0,
			}
		};

		rc = pm8058_gpio_config(camera_reset_cfg.gpio,
				  &camera_reset_cfg.cfg);
		if (rc < 0) {
			pr_err("%s pmic gpio config failed\n",
				  __func__);
			return;
		}
	}
	else
	{
		//Reserved for further projects.
	}
	if(true==HP_CAM_CONFIGURED)
		{
			#ifdef DONOT_USE_GPIOMUX
				config_gpio_tlmm_table(camera_mt9m113_off_gpio_table,
					ARRAY_SIZE(camera_mt9m113_off_gpio_table));
			#else
				config_gpio_table(MSM_CAM_OFF);
			#endif
			HP_CAM_CONFIGURED=false;
		}
	pr_debug("--- %s\n", __func__);
}

struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.csiphy = 0x04800000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_0_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
};

struct msm_camera_device_platform_data msm_camera_device_data_vx6953 = {
	.camera_gpio_on  = config_camera_on_gpios_vx6953,
	.camera_gpio_off = config_camera_off_gpios_vx6953,
	.ioext.csiphy = 0x04800000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_0_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
};


struct msm_camera_device_platform_data msm_camera_device_data_web_cam = {
	.camera_gpio_on  = config_camera_on_gpios_web_cam,
	.camera_gpio_off = config_camera_off_gpios_web_cam,
	.ioext.csiphy = 0x04900000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_1_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
};

//Bob
//Temporarily keep unchanged for these settings, need further check.
struct msm_camera_device_platform_data msm_camera_device_data_web_cam_mt9m113 = {
	.camera_gpio_on  = config_camera_on_gpios_web_cam_mt9m113,
	.camera_gpio_off = config_camera_off_gpios_web_cam_mt9m113,
	.ioext.csiphy = 0x04900000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_1_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
};

struct resource msm_camera_resources[] = {
	{
		.start	= 0x04500000,
		.end	= 0x04500000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VFE_IRQ,
		.end	= VFE_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};
#ifdef CONFIG_MSM_CAMERA_FLASH
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
	._fsrc.pmic_src.num_of_src = 2,
	._fsrc.pmic_src.low_current  = 100,
	._fsrc.pmic_src.high_current = 300,
	._fsrc.pmic_src.led_src_1 = PMIC8058_ID_FLASH_LED_0,
	._fsrc.pmic_src.led_src_2 = PMIC8058_ID_FLASH_LED_1,
	//These settings are for pm8058 flash, just keep them to avoid compile errors
	._fsrc.pmic_src.pmic_set_current =NULL,//pm8058_set_flash_led_current,
};
static struct msm_camera_sensor_strobe_flash_data strobe_flash_xenon = {
	.flash_trigger = VFE_CAMIF_TIMER1_GPIO,
	.flash_charge = VFE_CAMIF_TIMER2_GPIO,
	.flash_charge_done = VFE_CAMIF_TIMER3_GPIO_INT,
	.flash_recharge_duration = 50000,
	.irq = MSM_GPIO_TO_INT(VFE_CAMIF_TIMER3_GPIO_INT),
};
#endif
#ifdef CONFIG_IMX074
static struct msm_camera_sensor_flash_data flash_imx074 = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_imx074_data = {
	.sensor_name	= "imx074",
	.sensor_reset	= 106,
	.sensor_pwd		= 85,
	.vcm_pwd		= 1,
	.vcm_enable		= 0,
	.pdata			= &msm_camera_device_data,
	.resource		= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_imx074,
	.strobe_flash_data	= &strobe_flash_xenon,
	.csi_if			= 1
};
struct platform_device msm_camera_sensor_imx074 = {
	.name	= "msm_camera_imx074",
	.dev	= {
		.platform_data = &msm_camera_sensor_imx074_data,
	},
};
#endif
#ifdef CONFIG_WEBCAM_OV7692
static struct msm_camera_sensor_flash_data flash_ov7692 = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src
};
static struct msm_camera_sensor_info msm_camera_sensor_ov7692_data = {
	.sensor_name	= "ov7692",
	.sensor_reset	= 106,
	.sensor_pwd		= 85,
	.vcm_pwd		= 1,
	.vcm_enable		= 0,
	.pdata			= &msm_camera_device_data_web_cam,
	.resource		= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_ov7692,
	.csi_if			= 1
};
struct platform_device msm_camera_sensor_webcam = {
	.name	= "msm_camera_ov7692",
	.dev	= {
		.platform_data = &msm_camera_sensor_ov7692_data,
	},
};
#endif

#ifdef CONFIG_WEBCAM_MT9M113
static struct msm_camera_sensor_flash_data msm_flash_none = {
	   .flash_type = MSM_CAMERA_FLASH_NONE,
	   .flash_src  = NULL
};
static struct msm_camera_sensor_info msm_camera_sensor_mt9m113_data = {
	.sensor_name	= "mt9m113",
	.sensor_reset	= 106,
	.sensor_pwd		= 107,
	.vcm_pwd		= 1,
	.vcm_enable		= 0,
	.pdata			= &msm_camera_device_data_web_cam_mt9m113,
	.resource		= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &msm_flash_none,
	.csi_if			= 1
};
struct platform_device msm_camera_sensor_webcam_mt9m113 = {
	.name	= "msm_camera_mt9m113",
	.dev	= {
		.platform_data = &msm_camera_sensor_mt9m113_data,
	},
};
#endif

#ifdef CONFIG_VX6953
//Enable flash 
static struct msm_camera_sensor_flash_data flash_vx6953 = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_vx6953_data = {
	.sensor_name	= "vx6953",
	.sensor_reset	= NONE, //No reset pin for VX6953
	.sensor_pwd		= 9,    //Use PM8058 GPIO9 as camera standby pin
	.vcm_pwd		= NONE, //No dedicated VCM_POWER pin
	.vcm_enable		= 0,    //Temporarily keep it unenable
	.pdata			= &msm_camera_device_data_vx6953,
	.resource		= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_vx6953,
	.strobe_flash_data	= &strobe_flash_xenon, //Enable flash.
	.csi_if			= 1
};
struct platform_device msm_camera_sensor_vx6953 = {
	.name	= "msm_camera_vx6953",
	.dev	= {
		.platform_data = &msm_camera_sensor_vx6953_data,
	},
};
#endif


static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
	#ifdef CONFIG_IMX074
	{
		I2C_BOARD_INFO("imx074", 0x1A),
	},
	#endif
	#ifdef CONFIG_WEBCAM_OV7692
	{
		I2C_BOARD_INFO("ov7692", 0x78),
	},
	#endif
	//Bob
	#ifdef CONFIG_WEBCAM_MT9M113
	{
		I2C_BOARD_INFO("mt9m113", 0x78),
	},
	#endif
	#ifdef CONFIG_VX6953
	{
		I2C_BOARD_INFO("vx6953", 0x20),
	},
	#endif
};
#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0x04600000,
		.end    = 0x04600000 + SZ_1M - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_I2C_QUP
static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
}

static struct msm_i2c_platform_data msm_gsbi3_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi4_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi7_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi8_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi9_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

// Yegw 2010-8-20 config gsbi10 as i2c function
static struct msm_i2c_platform_data msm_gsbi10_qup_i2c_pdata = {
	.clk_freq = 300000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};
// yegw end

static struct msm_i2c_platform_data msm_gsbi12_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.use_gsbi_shared_mode = 1,
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};
#endif

#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
static struct msm_spi_platform_data msm_gsbi1_qup_spi_pdata = {
	.max_clock_speed = 24000000,
	.clk_name = "gsbi_qup_clk",
	.pclk_name = "gsbi_pclk",
};
//HP_Effie, QUP5_SPI for lcd interface, Start
static struct msm_spi_platform_data msm_gsbi5_qup_spi_pdata = {
	.max_clock_speed = 5400000,
	.clk_name = "gsbi_qup_clk",
	.pclk_name = "gsbi_pclk",
};
//HP_Effie, QUP5_SPI for lcd interface, End
#endif

#ifdef CONFIG_I2C_SSBI
/* PMIC SSBI */
static struct msm_ssbi_platform_data msm_ssbi1_pdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
};

/* PMIC SSBI */
static struct msm_ssbi_platform_data msm_ssbi2_pdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
};

/* CODEC/TSSC SSBI */
static struct msm_ssbi_platform_data msm_ssbi3_pdata = {
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

#ifdef CONFIG_BATTERY_MSM
/* Use basic value for fake MSM battery */
static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.avail_chg_sources = AC_CHG,
};

static struct platform_device msm_batt_device = {
	.name              = "msm-battery",
	.id                = -1,
	.dev.platform_data = &msm_psy_batt_data,
};
#endif

#ifdef CONFIG_SENSORS_MPU3050
#define MPU3050_GPIO_IRQ 125
#define MPU3050_GPIO_FSYNC 119

static void mpu3050_init(void);
static void mpu3050_init_complete(void);

static struct mpu3050_platform_data_ext mpu3050_data = {
	.dev_init   = mpu3050_init,
	.dev_init_complete = mpu3050_init_complete,
	.int_config  = 0x10,
//	.orientation = {  1,  0,  0,
//					  0,  1,  0,
//					  0,  0,  1 },
	.level_shifter = 0,
	.accel = {
	#ifdef CONFIG_SENSORS_ACCELEROMETER_NONE
		.get_slave_descr = NULL,
	#endif
	#ifdef CONFIG_SENSORS_KXTF9
		.get_slave_descr = kxtf9_get_slave_descr,
		.adapt_num   = 2,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = 0x0F,
		.orientation = {  0, -1,  0,
						 -1,  0,  0,
						  0,  0, -1 },
	#endif
	#ifdef CONFIG_SENSORS_LIS331DLH
		.get_slave_descr = lis331dlh_get_slave_descr,
		.adapt_num   = 2,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = 0x0F,
		.orientation = {  0, -1,  0,
						 -1,  0,  0,
						  0,  0, -1 },
	#endif
	#ifdef CONFIG_SENSORS_LSM303DLHA
		.get_slave_descr = lsm303dlha_get_slave_descr,
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = 0x18,
//		.orientation = {  0,  1,  0,
//						  -1,  0,  0,
//						  0,  0,  1 },
	#endif
	},
	.compass = {
	#ifdef CONFIG_SENSORS_LSM303DLHM
		.get_slave_descr = NULL,
		.get_slave_descr = lsm303dlhm_get_slave_descr,
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x1E,
//		.orientation = {  0,  1,  0, 
//		                 -1,  0,  0, 
//		                  0,  0,  1 },
	#endif
	#ifdef CONFIG_SENSORS_COMPASS_NONE
		.get_slave_descr = NULL,
	#endif
	#ifdef CONFIG_SENSORS_AMI304
		.get_slave_descr = ami304_get_slave_descr,
		.adapt_num   = 2,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x0E,
		.orientation = {  0, -1,  0,
						 -1,  0,  0,
						  0,  0, -1 },
	#endif
	},
};

static void init_gyro_data(void)
{
	unsigned char gyro1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1}; 
	unsigned char gyro2[9] = {-1, 0, 0, 0, 1, 0, 0, 0,-1}; 
	unsigned char gyro3[9] = { 0, 1, 0, 1, 0, 0, 0, 0,-1}; 

	unsigned char acc1[9] =  { 1, 0, 0, 0, 1, 0, 0, 0, 1}; 
	unsigned char acc2[9] =  {-1, 0, 0, 0, 1, 0, 0, 0,-1}; 
	unsigned char acc3[9] =  { 0, 1, 0, 1, 0, 0, 0, 0,-1}; 

	unsigned char mag1[9] =  { 1, 0, 0, 0, 1, 0, 0, 0, 1}; 
	unsigned char mag2[9] =  {-1, 0, 0, 0, 1, 0, 0, 0,-1}; 
	unsigned char mag3[9] =  { 0, 1, 0, 1, 0, 0, 0, 0,-1}; 

	int i;
	
	if( board_is_topaz_wifi())
	{
		for(i = 0; i < 9; i++)
		{
			mpu3050_data.orientation[i] = gyro1[i];			
			mpu3050_data.accel.orientation[i] = acc1[i];			
			mpu3050_data.compass.orientation[i] = mag1[i];			
		}
	}
	else if( board_is_topaz_3g())
	{
		for(i = 0; i < 9; i++)
		{
			mpu3050_data.orientation[i] = gyro2[i];			
			mpu3050_data.accel.orientation[i] = acc2[i];			
			mpu3050_data.compass.orientation[i] = mag2[i];			
		}
	}
	else if( board_is_opal_wifi() || board_is_opal_3g())
	{
		for(i = 0; i < 9; i++)
		{
			mpu3050_data.orientation[i] = gyro3[i];			
			mpu3050_data.accel.orientation[i] = acc3[i];			
			mpu3050_data.compass.orientation[i] = mag3[i];			
		}
	}
}

static void mpu3050_init(void)
{
//	struct regulator *votg_lvs3_1v8, *votg_l15;
	struct regulator *votg_lvs3_1v8;
	int rc;

	pr_err("%s\n", __func__);

	init_gyro_data();  //set different data for Topaz wifi/3G version

	// HP Wade: Based on chip spec, should turn on VDD(8058_l15) first, then turn on VLOGIC(8901_lvs3)
/*	votg_l15 = regulator_get(NULL, "8058_l15");
	if (IS_ERR(votg_l15))
	{
		pr_err("%s: unable to get votg_l15\n", __func__);
		return ;
	}

	if(regulator_set_voltage(votg_l15, 2850000, 2850000))
	{
		pr_err("%s: Unable to set regulator voltage:"
					" votg_l15\n", __func__);
		return ;
	}
	// VDD_LVDS_3.3V ENABLE
	if (regulator_enable(votg_l15))
	{
		pr_err("%s: Unable to enable the regulator:"
					" votg_l15\n", __func__);
		return ;
	}
*/
	// VDD_LVS3_1.8V
	votg_lvs3_1v8	 = regulator_get(NULL, "8901_lvs3");
	if (IS_ERR(votg_lvs3_1v8))
	{
		pr_err("%s: unable to get LVS3_1.8V\n", __func__);
		return ;
	}

	if (regulator_enable(votg_lvs3_1v8))
	{
		pr_err("%s: Unable to enable the regulator: LVS3_1.8V\n", __func__);
		return ;
	}

	/* configure topaz MPU3050 int pin */
	rc = gpio_tlmm_config(GPIO_CFG(MPU3050_GPIO_IRQ,   0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s: unable to configure int pin\n", __func__);

	/* configure topaz MPU3050 fsync pin */
	rc = gpio_tlmm_config(GPIO_CFG(MPU3050_GPIO_FSYNC, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s: unable to configure FSYNC pin\n", __func__);

	rc = gpio_request(MPU3050_GPIO_FSYNC, "MPU3050_FSYNC");
	if (rc) {
		pr_err("%s: MPU3050_GPIO_FSYNC request failed\n", __func__);
		return;
	}
	gpio_direction_output(MPU3050_GPIO_FSYNC, 0);
}

static void mpu3050_init_complete(void)
{
	// should move adding gsensor device to here
}

static struct i2c_board_info __initdata mpu3050_boardinfo[] = {
	{
		I2C_BOARD_INFO("mpu3050", 0x68),
//		.irq = MSM_GPIO_TO_INT(MPU3050_GPIO_IRQ),
		.platform_data = &mpu3050_data,
	},
};
#endif

//guoye:[20100914] ST lsm303dlh g/a/o sensor
#if defined(CONFIG_INPUT_LSM303_ACCEL) || defined(CONFIG_INPUT_LSM303_MAGNE)
int lsm303dlc_plt_power(int on)
{
	struct regulator *votg_l15;

	/* VDD_LVDS_3.3V*/
	votg_l15 = regulator_get(NULL, "8058_l15");
	if (IS_ERR(votg_l15))
	{
		pr_err("%s: unable to get votg_l15\n", __func__);
		return -1;
	}

	if (on)
	{
		if(regulator_set_voltage(votg_l15, 2850000, 2850000))
		{
			pr_err("%s: Unable to set regulator voltage:"
						" votg_l15\n", __func__);
			return -1;
		}
		/* VDD_LVDS_3.3V ENABLE*/
		if (regulator_enable(votg_l15))
		{
			pr_err("%s: Unable to enable the regulator:"
						" votg_l15\n", __func__);
			return -1;
		}
	}
	else
	{

//		regulator_disable(votg_l15);
//		regulator_put(votg_l15);
	}

	mdelay(20);
	return 0;
}

int lsm303dlc_plt_power_on(void)
{
	printk("guoye: %s \n",__func__);
	return lsm303dlc_plt_power(1);
}

int lsm303dlc_plt_power_off(void)
{
	printk("guoye: %s \n",__func__);
	return lsm303dlc_plt_power(0);
}
#endif

#ifdef CONFIG_INPUT_LSM303_ACCEL
//[HPP]guoye-For interrupt mode
#define LSM303DLC_IRQ_GPIO_TOPAZ_WIFI_PROTO	124
#define LSM303DLC_IRQ_GPIO_TOPAZ_3G_PROTO	46

//[HPP]guoye
#if 0
static int lsm303dlc_plt_gpio_config(void)
{
	int rc = -1;
	int gpio;

	/* configure touchscreen irq gpio */

	if( board_is_topaz_3g() )
		gpio = LSM303DLC_IRQ_GPIO_TOPAZ_3G_PROTO;
	else
		gpio = LSM303DLC_IRQ_GPIO_TOPAZ_WIFI_PROTO;
		
	rc = gpio_tlmm_config(GPIO_CFG(gpio, 0, GPIO_CFG_INPUT,                GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 0);
	if (rc)
	{
		pr_err("%s: unable to configure gpio %d\n",            __func__, gpio);
		goto fail_irq;
	}

	rc = gpio_request(gpio, "gsensor_irq_gpio");
	if (rc)
	{
		pr_err("%s: unable to request gpio %d\n",            __func__, gpio);
		goto fail_irq;
	}

	rc = gpio_direction_input(gpio);
	if (rc)
	{
		pr_err("%s: FAILED: gpio_direction_input(%d) rc=%d\n", __func__, gpio, rc);
		goto fail_irq;
	}

	return MSM_GPIO_TO_INT(gpio);

fail_irq:
	return rc;
}
#endif

//[HPP]guoye-get axis map dynamically with different board
static void lsm303dlc_plt_get_axis_map(int* map)
{
	if( board_is_topaz_3g() )
	{
	/*
		Y					x  z
    |         |  /
    |         | /    
    |         |/    		.
    |         o----------->y
    O--------->X
	 /
  /
 Z	
 */
		map[0] = 1;//axis_map_x
		map[1] = 0;//axis_map_y
		map[2] = 2;//axis_map_z
		map[3] = 0;//negate_x
		map[4] = 0;//negate_y
		map[5] = 1;//negate_z
	}
	else if( board_is_topaz_wifi() )
	{
	/* 
		Y					  				
    |         o----------->y    		
    |        /|					.
    |				/ |
    | 		 z  x
    O--------->X
	 /
  /
 Z
 */
		map[0] = 1;//axis_map_x
		map[1] = 0;//axis_map_y
		map[2] = 2;//axis_map_z
		map[3] = 0;//negate_x
		map[4] = 1;//negate_y
		map[5] = 0;//negate_z
	}
	else if( board_is_opal_3g() || board_is_opal_wifi() )
	{
	/*
		Y					y  z
    |       . |  /
    |         | /    
    |         |/
    | x<------o
    O--------->X
	 /
  /
 Z	
 */
		map[0] = 0;//axis_map_x
		map[1] = 1;//axis_map_y
		map[2] = 2;//axis_map_z
		map[3] = 1;//negate_x
		map[4] = 0;//negate_y
		map[5] = 1;//negate_z
	}
	
}
//[HPP]guoye

struct lsm303dlh_acc_platform_data lsm303dlc_acc_plt_dat = {
	.poll_interval = 50,
	.min_interval = 10,
	.g_range = LSM303DLH_G_2G,
	//.axis_map_x = 1,
	//.axis_map_y = 0,
	//.axis_map_z = 2,
	//.negate_x = 0,
	//.negate_y = 1,
	//.negate_z = 0,
	//[HPP]guoye-For interrupt mode
	//.gpioirq = LSM303DLC_IRQ_GPIO_TOPAZ_WIFI_PROTO,
	//[HPP]guoye
	.power_on = lsm303dlc_plt_power_on,
	.power_off = lsm303dlc_plt_power_off,
	//[HPP]guoye--since acc interrupt will not be used in any future h/w build
	//.gpio_config = lsm303dlc_plt_gpio_config,
	.gpio_config = 0,
	//[HPP]guoye-get axis map dynamically with different board	
	.get_axis_map = lsm303dlc_plt_get_axis_map,
	//[HPP]guoye
};
#endif

#ifdef CONFIG_INPUT_LSM303_MAGNE
struct lsm303dlh_mag_platform_data lsm303dlc_mag_plt_dat = {
	.poll_interval = 50,
	.min_interval = 10,
	.h_range = LSM303DLH_H_8_1G,
	//.axis_map_x = 1,
	//.axis_map_y = 0,
	//.axis_map_z = 2,
	//.negate_x = 0,
	//.negate_y = 1,
	//.negate_z = 0,
	.power_on = lsm303dlc_plt_power_on,
	.power_off = lsm303dlc_plt_power_off,
	//[HPP]guoye-get axis map dynamically with different board
	.get_axis_map = lsm303dlc_plt_get_axis_map,
	//[HPP]guoye
};
#endif

#if defined(CONFIG_INPUT_LSM303_ACCEL) || \
		defined(CONFIG_INPUT_LSM303_MAGNE)
static struct i2c_board_info lsm303_i2c_devs[] __initdata = {
#ifdef CONFIG_INPUT_LSM303_ACCEL
	{
		I2C_BOARD_INFO(HP_GSENSOR_NAME, 0x18),
		.platform_data = &lsm303dlc_acc_plt_dat,
		//[HPP]guoye-For interrupt mode
		//.irq = MSM_GPIO_TO_INT(LSM303DLC_IRQ_GPIO_TOPAZ_WIFI_PROTO),
		//[HPP]guoye
	},
#endif
#ifdef CONFIG_INPUT_LSM303_MAGNE
	{
		I2C_BOARD_INFO(HP_MAGNETIC_NAME, 0x1E),
		.platform_data = &lsm303dlc_mag_plt_dat,
	},
#endif
};
#endif
//guoye

#if defined(CONFIG_FB_MSM_LCDC_LG_XGA) || defined(CONFIG_FB_MSM_LCDC_HITACHI_XGA)
static struct i2c_board_info lcd_panel_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("lcdxpanel", 0x50),
	},
};
#endif

//HP zhanghong:Oct 27 10:35 CST 2010, begin
#ifdef CONFIG_INPUT_CYPRESS_CY8C20236A
static int get_ps_gpio_pin_configuration(u8 type)
{
	int pin = -1;

	if(type == GPIO_INT_TYPE)
	{
		if(board_is_topaz_wifi())
		{
			pin = 136;// maybe here is a problem, reset_pin = 40?
		}
		else
			pin = 39;
	}

	if(type == GPIO_RESET_TYPE)
	{
		if(board_is_topaz_wifi())
		{
			pin = 36;
		}
		else
			pin = 85;
	}
	return pin;
}

struct  cypress_cy8c20236a_platform_data cypress_cy8c20236a_pdata = {
	//.power = cypress_cy8c20236a_power,
	.reset_pin = get_ps_gpio_pin_configuration,
	.p_out = get_ps_gpio_pin_configuration,
};
static struct i2c_board_info cy8c20236a_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("cy8c20236a", 0x22), //i2c slave addree can be changed ;
		.platform_data = &cypress_cy8c20236a_pdata,
	},
};
#endif

//wanqin nfc
#ifdef CONFIG_NFC

static int nfc_chip_reset(void)
{

	printk("nfc reset\n");
	
	if( (board_is_opal_wifi() && (board_type >= OPAL_EVT1)) || (board_is_opal_3g() && (board_type >= OPAL3G_EVT1)))
	{
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(PM8058_NFC_WAKEUP-1), 1);
		udelay(2);
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(PM8058_NFC_WAKEUP-1), 0);
		return 0;
	}
	else
		return -1;
}

static int nfc_irq_get(void)
{
	printk("nfc_irq_get\n");
	
	if( (board_is_opal_wifi() && (board_type >= OPAL_EVT1)) || (board_is_opal_3g() && (board_type >= OPAL3G_EVT1)) )
		return gpio_to_irq(PM8058_GPIO_PM_TO_SYS(PM8058_NFC_IRQOUT-1));
	else
		return -1;
}

static int nfc_gpio_cfg(void)
{
	int rc = -1;
	
	printk("nfc_gpio_cfg\n");
	
	if( (board_is_opal_wifi() && (board_type >= OPAL_EVT1)) || (board_is_opal_3g() && (board_type >= OPAL3G_EVT1)) )
	{
		rc = gpio_request(PM8058_GPIO_PM_TO_SYS(PM8058_NFC_WAKEUP-1), "nfc_wake");
		if (rc < 0)
		{
			printk("NFC reset fail\n");
			return rc;
		}
		
		rc = gpio_request(PM8058_GPIO_PM_TO_SYS(PM8058_NFC_IRQOUT-1), "nfc_int");
		if (rc < 0)
		{
			printk("NFC IRQ fail\n");
			return rc;
		}
		
		gpio_direction_input(PM8058_GPIO_PM_TO_SYS(PM8058_NFC_IRQOUT-1));
			
		return 0;
	  }
	  
	  return rc;
}
struct nfc_platform_data nfc_gpio_ctrl = {
	.nfc_reset = nfc_chip_reset,
	.nfc_irq = nfc_irq_get,
	.nfc_iocfg = nfc_gpio_cfg,
};

static struct i2c_board_info nfc_i2c_info[] = {
	{
		I2C_BOARD_INFO("opennfc", 0x5E),
		.platform_data = &nfc_gpio_ctrl,
	}
};
#endif
#ifdef CONFIG_FB_MSM_LCDC_DSUB
/* VGA = 1440 x 900 x 4(bpp) x 2(pages)
   prim = 1024 x 600 x 4(bpp) x 2(pages)
   This is the difference. */
#define MSM_FB_DSUB_PMEM_ADDER (0x9E3400-0x4B0000)
#else
#define MSM_FB_DSUB_PMEM_ADDER (0)
#endif
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
/* prim = 1024 x 600 x 4(bpp) x 2(pages)
 * hdmi = 1920 x 1080 x 2(bpp) x 1(page)
 * Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(0x4B0000 + 0x3F4800 + MSM_FB_DSUB_PMEM_ADDER, 4096)
#elif defined(CONFIG_FB_MSM_TVOUT)
/* prim = 1024 x 600 x 4(bpp) x 2(pages)
 * tvout = 720 x 576 x 2(bpp) x 2(pages)
 * Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(0x4B0000 + 0x195000 + MSM_FB_DSUB_PMEM_ADDER, 4096)
#else /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

//HP_Effie, Changed for Topaz/Opal Panel display, Begin
#if defined(CONFIG_FB_MSM_LCDC_LG_XGA) || defined(CONFIG_FB_MSM_LCDC_HITACHI_XGA)
#define MSM_FB_SIZE 0x600000
#else
#define MSM_FB_SIZE roundup(0x4B0000 + MSM_FB_DSUB_PMEM_ADDER, 4096)
#endif /* CONFIG_FB_MSM_LCDC_LG_XGA,CONFIG_FB_MSM_LCDC_HITACHI_XGA_PANLE */
//HP_Effie, Changed for Topaz/OpalPanel display, End

#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#define MSM_PMEM_SF_SIZE 0x4000000 /* 64 Mbytes */
#define MSM_GPU_PHYS_SIZE 0x4cf000 /* 4.8 Mbytes */
		/* 8* (512KB + 68384 + 32KB) + 36KB = 5040384 = 0x4ce900 */

#define MSM_PMEM_KERNEL_EBI1_SIZE  0x600000
#define MSM_PMEM_ADSP_SIZE         0x2000000
#define MSM_PMEM_AUDIO_SIZE        0x219000

#define MSM_SMI_BASE          0x38000000
/* Kernel SMI PMEM Region for video core, used for Firmware */
/* and encoder,decoder scratch buffers */
/* Kernel SMI PMEM Region Should always precede the user space */
/* SMI PMEM Region, as the video core will use offset address */
/* from the Firmware base */
#define PMEM_KERNEL_SMI_BASE  (MSM_SMI_BASE)
#define PMEM_KERNEL_SMI_SIZE  0x300000
/* User space SMI PMEM Region for video core*/
/* used for encoder, decoder input & output buffers  */
#define MSM_PMEM_SMIPOOL_BASE (PMEM_KERNEL_SMI_BASE + PMEM_KERNEL_SMI_SIZE)
#define MSM_PMEM_SMIPOOL_SIZE 0x3D00000

/*HP_Hover[20101027] SMI RAM for framebuffer*/
#define MSM_FB_IN_SMI 0
#define MSM_FB_BASE	(MSM_PMEM_SMIPOOL_BASE + MSM_PMEM_SMIPOOL_SIZE)

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned gpu_phys_size = MSM_GPU_PHYS_SIZE;
static int __init gpu_phys_size_setup(char *p)
{
	gpu_phys_size = memparse(p, NULL);
	return 0;
}
early_param("gpu_phys_size", gpu_phys_size_setup);

#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
static unsigned pmem_kernel_ebi1_size = MSM_PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;

static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;

static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	if (!strcmp(name, "lcdc_samsung_wsvga"))
		return 0;
	pr_warning("%s: not supported '%s'", __func__, name);
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource          = msm_fb_resources,
	.dev.platform_data = &msm_fb_pdata,
};

#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};
#endif

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
	.name = PMEM_KERNEL_SMI_DATA_NAME,
	/* defaults to bitmap don't edit */
	.cached = 0,
};

static struct platform_device android_pmem_kernel_smi_device = {
	.name = "android_pmem",
	.id = 6,
	.dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {.platform_data = &android_pmem_pdata},
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_smipool_pdata = {
	.name = "pmem_smipool",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};
static struct platform_device android_pmem_smipool_device = {
	.name = "android_pmem",
	.id = 7,
	.dev = { .platform_data = &android_pmem_smipool_pdata },
};

#endif

#define GPIO_DONGLE_PWR_EN 258
void setup_display_power(void);
#ifdef CONFIG_FB_MSM_LCDC_SAMSUNG_WSVGA
static int lcdc_vga_enabled;
static int vga_enable_request(int enable)
{
	if (enable)
		lcdc_vga_enabled = 1;
	else
		lcdc_vga_enabled = 0;
	setup_display_power();

	return 0;
}
#endif

#define GPIO_BACKLIGHT_PWM0 0
#define GPIO_BACKLIGHT_PWM1 1

static int pmic_backlight_gpio[2]
	= { GPIO_BACKLIGHT_PWM0, GPIO_BACKLIGHT_PWM1 };

#ifdef CONFIG_FB_MSM_LCDC_SAMSUNG_WSVGA
static struct msm_panel_common_pdata lcdc_samsung_panel_data = {
	.gpio_num = pmic_backlight_gpio, /* two LPG CHANNELS for backlight */
	.vga_switch = vga_enable_request,
};

static struct platform_device lcdc_samsung_panel_device = {
	.name = "lcdc_samsung_wsvga",
	.id = 0,
	.dev = {
		.platform_data = &lcdc_samsung_panel_data,
	}
};
#endif

//HP_Effie,Added for Topaz LG Panel display, Begin
#if defined(CONFIG_FB_MSM_LCDC_LG_XGA) || defined(CONFIG_FB_MSM_LCDC_HITACHI_XGA)
/* PMIC8058 GPIO offset starts at 0 */
#define GPIO_BACKLIGHT_EN  PM8058_GPIO_PM_TO_SYS(25-1)
static bool delay_bl_power_up = true;

int set_pmic_backlight(int bl_level)
{
	if(delay_bl_power_up)
	{
		delay_bl_power_up = false;
		msleep(200);
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, 1);
		pr_info("[%s] backlight enabled\n",__func__);
	}
	return 0;
}

static struct msm_panel_common_pdata lcdc_common_panel_data = {
	.gpio_num = pmic_backlight_gpio, /* two LPG CHANNELS for backlight */
	.pmic_backlight = set_pmic_backlight,
};
#endif

#ifdef CONFIG_FB_MSM_LCDC_LG_XGA
static struct platform_device lcdc_lg_panel_device = {
	.name = "lcdc_lg_xga",
	.id = 0,
	.dev = {
		.platform_data = &lcdc_common_panel_data,
	}
};
#endif
//HP_Effie,Added for Topaz LG Panel display, End

#ifdef CONFIG_FB_MSM_LCDC_HITACHI_XGA
static struct platform_device lcdc_hitachi_panel_device = {
	.name = "lcdc_hitachi_xga",
	.id = 0,
	.dev = {
		.platform_data = &lcdc_common_panel_data,
	}
};
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct resource hdmi_msm_resources[] = {
	{
		.name  = "hdmi_msm_qfprom_addr",
		.start = 0x00700000,
		.end   = 0x007060FF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_hdmi_addr",
		.start = 0x04A00000,
		.end   = 0x04A00FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_irq",
		.start = HDMI_IRQ,
		.end   = HDMI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static int hdmi_enable_5v(int on);
static int hdmi_core_power(int on);
static int hdmi_cec_power(int on);

static struct msm_hdmi_platform_data hdmi_msm_data = {
	.irq = HDMI_IRQ,
	.enable_5v = hdmi_enable_5v,
	.core_power = hdmi_core_power,
	.cec_power = hdmi_cec_power,
};

static struct platform_device hdmi_msm_device = {
	.name = "hdmi_msm",
	.id = 0,
	.num_resources = ARRAY_SIZE(hdmi_msm_resources),
	.resource = hdmi_msm_resources,
	.dev.platform_data = &hdmi_msm_data,
};
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

#ifdef CONFIG_FB_MSM_MIPI_DSI
static struct platform_device mipi_dsi_toshiba_panel_device = {
	.name = "mipi_toshiba",
	.id = 0,
};
#endif

static void __init msm8x60_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = MSM_FB_SIZE;
	#if MSM_FB_IN_SMI
	addr = (void *) MSM_FB_BASE;
	msm_fb_resources[0].start = (unsigned long)addr;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("using %lu bytes of SMI at %lx physical for fb\n",
			size, (unsigned long)addr);
	#else
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
	#endif

#ifdef CONFIG_MSM_KGSL
	size = gpu_phys_size;
	if (size) {
		addr = alloc_bootmem(size);
		msm_device_kgsl.resource[1].start = __pa(addr);
		msm_device_kgsl.resource[1].end =
			msm_device_kgsl.resource[1].start + size - 1;
		pr_info("allocating %lu bytes at %p (%lx physical) for "
			"KGSL\n", size, addr, __pa(addr));
	}
#endif

#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}
#endif

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	size = PMEM_KERNEL_SMI_SIZE;
	if (size) {
		android_pmem_kernel_smi_pdata.start = PMEM_KERNEL_SMI_BASE;
		android_pmem_kernel_smi_pdata.size = size;
		pr_info("allocating %lu bytes at %lx physical for kernel"
			" smi pmem arena\n", size,
			(unsigned long) PMEM_KERNEL_SMI_BASE);
	}
#endif

#ifdef CONFIG_ANDROID_PMEM
	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = MSM_PMEM_SMIPOOL_SIZE;
	if (size) {
		android_pmem_smipool_pdata.start = MSM_PMEM_SMIPOOL_BASE;
		android_pmem_smipool_pdata.size = size;
		pr_info("allocating %lu bytes at %lx physical for user"
			" smi  pmem arena\n", size,
			(unsigned long) MSM_PMEM_SMIPOOL_BASE);
	}

	size = MSM_PMEM_AUDIO_SIZE;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_audio_pdata.start = __pa(addr);
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for audio "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}
#endif
}

#if defined(CONFIG_TOUCHSCREEN_CYTTSP_I2C) || \
		defined(CONFIG_TOUCHSCREEN_CYTTSP_I2C_MODULE)
/*virtual key support */
static ssize_t tma300_vkeys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":60:875:90:90"
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":180:875:90:90"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":300:875:90:90"
	":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":420:875:90:90"
	"\n");
}

static struct kobj_attribute tma300_vkeys_attr = {
	.attr = {
		.mode = S_IRUGO,
	},
	.show = &tma300_vkeys_show,
};

static struct attribute *tma300_properties_attrs[] = {
	&tma300_vkeys_attr.attr,
	NULL
};

static struct attribute_group tma300_properties_attr_group = {
	.attrs = tma300_properties_attrs,
};

static struct kobject *properties_kobj;

#define FLUID_CYTTSP_TS_GPIO_IRQ	61
static int cyttsp_fluid_platform_init(struct i2c_client *client)
{
	int rc = -EINVAL;
	struct regulator *pm8058_l5;

	pm8058_l5 = regulator_get(NULL, "8058_l5");
	if (IS_ERR(pm8058_l5)) {
		pr_err("%s: regulator get of 8058_l5 failed (%ld)\n",
			__func__, PTR_ERR(pm8058_l5));
		rc = PTR_ERR(pm8058_l5);
		return rc;
	}
	rc = regulator_set_voltage(pm8058_l5, 2850000, 2850000);
	if (rc) {
		pr_err("%s: regulator_set_voltage of 8058_l5 failed(%d)\n",
			__func__, rc);
		goto reg_put;
	}

	rc = regulator_enable(pm8058_l5);
	if (rc) {
		pr_err("%s: regulator_enable of 8058_l5 failed(%d)\n",
			__func__, rc);
		goto reg_put;
	}

	/* check this device active by reading first byte/register */
	rc = i2c_smbus_read_byte_data(client, 0x01);
	if (rc < 0) {
		pr_err("%s: i2c sanity check failed\n", __func__);
		goto reg_disable;
	}

	/* configure touchscreen interrupt gpio */
	rc = gpio_request(FLUID_CYTTSP_TS_GPIO_IRQ, "cyttsp_irq_gpio");
	if (rc) {
		pr_err("%s: unable to request gpio %d\n",
			__func__, FLUID_CYTTSP_TS_GPIO_IRQ);
		goto reg_disable;
	}

	/* virtual keys */
	tma300_vkeys_attr.attr.name = "virtualkeys.cyttsp-i2c";
	properties_kobj = kobject_create_and_add("board_properties",
				NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
			&tma300_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("%s: failed to create board_properties\n",
				__func__);

	return CY_OK;

reg_disable:
	regulator_disable(pm8058_l5);
reg_put:
	regulator_put(pm8058_l5);
	return rc;
}

static int cyttsp_fluid_platform_resume(struct i2c_client *client)
{
	/* add any special code to strobe a wakeup pin or chip reset */
	msleep(10);

	return CY_OK;
}

static struct cyttsp_platform_data cyttsp_fluid_pdata = {
	.panel_maxx = 539,
	.panel_maxy = 994,
	.disp_minx = 30,
	.disp_maxx = 509,
	.disp_miny = 60,
	.disp_maxy = 859,
	.flags = 0x04,
	.gen = CY_GEN3,	/* or */
	.use_st = CY_USE_ST,
	.use_mt = CY_USE_MT,
	.use_hndshk = CY_SEND_HNDSHK,
	.use_trk_id = CY_USE_TRACKING_ID,
	.use_sleep = CY_USE_SLEEP,
	.use_gestures = CY_USE_GESTURES,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP1 | CY_GEST_GRP2 |
				CY_GEST_GRP3 | CY_GEST_GRP4 |
				CY_ACT_DIST,
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
	.resume = cyttsp_fluid_platform_resume,
	.init = cyttsp_fluid_platform_init,
};

static struct i2c_board_info cyttsp_fluid_info[] __initdata = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, 0x24),
		.platform_data = &cyttsp_fluid_pdata,
#ifndef CY_USE_TIMER
		.irq = MSM_GPIO_TO_INT(FLUID_CYTTSP_TS_GPIO_IRQ),
#endif /* CY_USE_TIMER */
	},
};
#endif

static struct regulator *vreg_tmg200;

#define TS_PEN_IRQ_GPIO 61
static int tmg200_power(int vreg_on)
{
	int rc = -EINVAL;

	if (!vreg_tmg200) {
		printk(KERN_ERR "%s: regulator 8058_s3 not found (%d)\n",
			__func__, rc);
		return rc;
	}

	rc = vreg_on ? regulator_enable(vreg_tmg200) :
		  regulator_disable(vreg_tmg200);
	if (rc < 0)
		printk(KERN_ERR "%s: vreg 8058_s3 %s failed (%d)\n",
				__func__, vreg_on ? "enable" : "disable", rc);
	return rc;
}

static int tmg200_dev_setup(bool enable)
{
	int rc;

	if (enable) {
		vreg_tmg200 = regulator_get(NULL, "8058_s3");
		if (IS_ERR(vreg_tmg200)) {
			pr_err("%s: regulator get of 8058_s3 failed (%ld)\n",
				__func__, PTR_ERR(vreg_tmg200));
			rc = PTR_ERR(vreg_tmg200);
			return rc;
		}

		rc = regulator_set_voltage(vreg_tmg200, 1800000, 1800000);
		if (rc) {
			pr_err("%s: regulator_set_voltage() = %d\n",
				__func__, rc);
			goto reg_put;
		}
	} else {
		/* put voltage sources */
		regulator_put(vreg_tmg200);
	}
	return 0;
reg_put:
	regulator_put(vreg_tmg200);
	return rc;
}

static struct cy8c_ts_platform_data cy8ctmg200_pdata = {
	.ts_name = "msm_tmg200_ts",
	.dis_min_x = 0,
	.dis_max_x = 1023,
	.dis_min_y = 0,
	.dis_max_y = 599,
	.min_tid = 1,
	.max_tid = 255,
	.min_touch = 0,
	.max_touch = 255,
	.min_width = 0,
	.max_width = 255,
	.power_on = tmg200_power,
	.dev_setup = tmg200_dev_setup,
	.nfingers = 2,
	.irq_gpio = TS_PEN_IRQ_GPIO,
	.resout_gpio = GPIO_CAP_TS_RESOUT_N,
};

static struct i2c_board_info cy8ctmg200_board_info[] = {
	{
		I2C_BOARD_INFO("cy8ctmg200", 0x2),
		.platform_data = &cy8ctmg200_pdata,
	}
};

// yegw 2010-8-13 add for xMT1386 touch
// change for TOPAZ_proto_MB_AUG_18
static int  MXT1386_TS_PEN_IRQ_GPIO = 123;  // defuaut 123 topaz wifi version  45 for topaz 3G version
static int  MXT1386_TS_PEN_RESET_GPIO = 70;   //

static u8 get_touch_gpio_pin(void)
{
	if(board_is_topaz_wifi())
	{
		MXT1386_TS_PEN_IRQ_GPIO = 123;//
	}
	else if (board_is_topaz_3g())
	{
		MXT1386_TS_PEN_IRQ_GPIO = 45;
	}
    else if (board_is_opal_3g() ||
        board_is_opal_wifi() )
    {
        MXT1386_TS_PEN_IRQ_GPIO = 45;
    }

	return MXT1386_TS_PEN_IRQ_GPIO;

}


static void touchscreen_gpio_release(void)
{
	gpio_free(MXT1386_TS_PEN_RESET_GPIO);
	gpio_free(MXT1386_TS_PEN_IRQ_GPIO);
}


static void touch_poweron(bool on)
{

#if 0
	// power on
	struct regulator *votg_l10;

	/* VDD_LVDS_3.3V*/
	votg_l10 = regulator_get(NULL, "8058_l10");
	if (IS_ERR(votg_l10))
	{
		pr_err("%s: unable to get votg_l10\n", __func__);
		return;
	}

	if(on)
	{
		if(regulator_set_voltage(votg_l10, 3050000, 3050000))
		{
			pr_err("%s: Unable to set regulator voltage:"
					" votg_l10\n", __func__);
			return;
		}
		/* VDD_LVDS_3.3V ENABLE*/
		if (regulator_enable(votg_l10))
		{
			pr_err("%s: Unable to enable the regulator:"
					" votg_l10\n", __func__);
		   return;
		}
	}
	else
	{
		regulator_disable(votg_l10);
		regulator_put(votg_l10);
	}
#endif
	return;
}


static void init_touch_hw(void)
{
	int rc;
	int irq_pin = get_touch_gpio_pin();
	int reset_pin = MXT1386_TS_PEN_RESET_GPIO;
	unsigned irq_cfg =
		GPIO_CFG(irq_pin,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

	rc = gpio_request(reset_pin, "touch_reset");
	if (rc)
	{
	  pr_err("gpio_request failed on pin %d (rc=%d)\n",
					 reset_pin, rc);
	   goto err_gpioconfig;
	}


	/* pull reset high */
	gpio_set_value(reset_pin, 1);
	mdelay(100);

	// power on
	touch_poweron(true);

	// power reset
	gpio_direction_output(reset_pin, 0);
	mdelay(100);
	gpio_direction_output(reset_pin, 1);
	mdelay(100);


	rc = gpio_request(irq_pin, "msm_touchpad_irq");
	if (rc) {
		pr_err("gpio_request failed on pin %d (rc=%d)\n",
			irq_pin, rc);
		goto err_gpioconfig;
	}

	rc = gpio_tlmm_config(irq_cfg, 0);
	if (rc) {
		pr_err("gpio_tlmm_config failed on pin %d (rc=%d)\n",
					irq_pin, rc);
		goto err_gpioconfig;
	}

	rc = gpio_direction_input(irq_pin);
	if (rc) {
		pr_err("gpio_direction_input failed on pin %d (rc=%d)\n",
					irq_pin, rc);
		goto err_gpioconfig;
	}


	return ;

err_gpioconfig:
	touchscreen_gpio_release();
	return ;

}


static void exit_touch_hw(void)
{
	// keep power
	touchscreen_gpio_release();
}

/*
 * Reads the CHANGELINE state; interrupt is valid if the changeline
 * is low.
 */
static u8 read_chg(void)
{
	return gpio_get_value(MXT1386_TS_PEN_IRQ_GPIO);
}

static u8 valid_interrupt(void)
{
	return !read_chg();
}

static struct mxt_platform_data msm_touchscreen_data = {
	/* Maximum number of simultaneous touches to report. */
	.numtouch         = 10,
	.init_platform_hw = NULL,
	.exit_platform_hw = &exit_touch_hw,
	.max_x            = 1024,
	.max_y            = 768,
	.valid_interrupt  = &valid_interrupt,
	.read_chg         = &read_chg,
	.irq_gpio         = &get_touch_gpio_pin,
};



static struct i2c_board_info xMT1386_board_info[] = {
	{
		I2C_BOARD_INFO("maXTouch", 0x4C),
		.platform_data = &msm_touchscreen_data,
	}
};

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_HP_I2C
#define CY_HP_I2C_ADR  0x67
#define CY_HP_USE_MT
#define CY_HP_MAXX 4000
#define CY_HP_MAXY 3000

static int cyttsp_i2c_wakeup(void)
{
    return 0;
}

static struct cyttsp_hp_platform_data cypress_i2c_ttsp_platform_data = {
    .wakeup = cyttsp_i2c_wakeup,
    .init = NULL,
#ifdef CY_HP_USE_MT
    .mt_sync = input_mt_sync,
#endif
    .maxx = CY_HP_MAXX,
    .maxy = CY_HP_MAXY,
    .flags = 0x06,
    .gen = CY_GEN4,
    .use_mt = 1,
    .use_trk_id = 0,
    .use_hndshk = 0,
    .use_timer = 0,
    .use_sleep = 1,
    .use_gestures = 0,
    .use_load_file = 0,
    .use_force_fw_update = 0,
    .use_virtual_keys = 0,
    /* activate up to 4 groups
     * and set active distance
     */
    .gest_set = CY_GEST_GRP_NONE | CY_ACT_DIST,
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
    .irq_gpio = &get_touch_gpio_pin,
};
static struct i2c_board_info cypress_cytcb5s_touch_info[] = {
    {
        I2C_BOARD_INFO(CY_I2C_NAME, CY_HP_I2C_ADR),
        //.irq = MSM_GPIO_TO_INT(CY_I2C_IRQ_GPIO),
        .platform_data = &cypress_i2c_ttsp_platform_data,
    }
};
#endif

#ifdef CONFIG_LEDS_LM8502
static struct lm8502_led_list lm8502_ledlist0[] = {
    [0] = {
        .type = LED_WHITE,
        .id = LED_ID_D1,
    },
    [1] = {
        .type = LED_WHITE,
        .id = LED_ID_D2,
    },
};

static struct lm8502_led_config lm8502_leds[] = {
    [0] = {
        .cdev.name = "button-backlight",
        .led_list = lm8502_ledlist0,
        .nleds = ARRAY_SIZE(lm8502_ledlist0),
        .hw_group = LED_HW_GRP_1,
        .max_brightness = LED_FULL,
        .default_max_current = 0x02, // 12.5mA
        .default_brightness = LED_OFF,
    },
};

static struct lm8502_platform_data lm8502_flash_data = {
    //leds
    .num_leds = ARRAY_SIZE(lm8502_leds),
    .leds	= lm8502_leds,
    
    //vibrator
    .max_timeout_ms = 15000,
    .level_pwm = 0,

    //flash or torch
    .flash_default_duration = 512,    // 512ms
    .flash_default_current = 600,   // 600mA
    .torch_default_current = 150,   // 150mA
    
    //others
    .power_mode = LM8502_MISC_POWER_SAVE_ON,
};

static struct i2c_board_info lm8502_board_info[] = {
    {
        I2C_BOARD_INFO(LM8502_I2C_DEVICE, LM8502_I2C_ADDR),
        .platform_data = &lm8502_flash_data,
    }
};
static void __init lm8502_init(void)
{
    int rc;

    /* LM8502 LIGHTING_EN */
    rc = gpio_tlmm_config(GPIO_CFG(121, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (rc)
        pr_err("%s: unable to configure gpio 121\n", __func__);

    rc = gpio_request(121, "LIGHTING_EN");
    if (rc) {
        pr_err("%s: LIGHTING_EN gpio121 request failed\n", __func__);
        return;
    }
    gpio_direction_output(121, 0);
    gpio_set_value(121, 1);
}
#endif

static void __init boardid_init(void)
{
	int rc=0;
	int emu,device0,device1,device2,d0,d1,d2,d3;

	/*******************************
	Topaz Board ID GPIO pins:
	GPIO95  - ID_EMU
	GPIO96  - ID_Device0(ID_TOPAZ)
	GPIO101 - ID_Device1
	GPIO79  - ID_Device2
	GPIO97  - ID_D0
	GPIO98  - ID_D1
	GPIO99  - ID_D2
	GPIO100 - ID_D3
	*******************************/
	rc += gpio_request(95, "ID_EMU");
	rc += gpio_request(96, "ID_Device0");
	rc += gpio_request(101, "ID_Device1");
	rc += gpio_request(79, "ID_Device2");
	rc += gpio_request(97, "ID_D0");
	rc += gpio_request(98, "ID_D1");
	rc += gpio_request(99, "ID_D2");
	rc += gpio_request(100, "ID_D3");
	if (rc) {
		pr_err("%s: board id pins (gpio95-101,gpio79) request failed\n", __func__);
		return;
	}

	gpio_direction_input(95);
	gpio_direction_input(96);
	gpio_direction_input(101);
	gpio_direction_input(79);
	gpio_direction_input(97);
	gpio_direction_input(98);
	gpio_direction_input(99);
	gpio_direction_input(100);


	emu = gpio_get_value(95);
	boardid_info.emu = emu;

	device0 = gpio_get_value(96);
	device1 = gpio_get_value(101);
	device2 = gpio_get_value(79);
	boardid_info.product = (device2 << 2) | (device1 << 1) | (device0 << 0);
	
	d0 = gpio_get_value(97);
	d1 = gpio_get_value(98);
	d2 = gpio_get_value(99);
	boardid_info.hwbuild = (d2 << 2) | (d1 << 1) | (d0 << 0);

	d3 = gpio_get_value(100);
	boardid_info.sku = d3;

	//printk("%s:boardid_info.emu = %d\n", __func__, boardid_info.emu);
	//printk("%s:boardid_info.product = %d\n", __func__, boardid_info.product);
	//printk("%s:boardid_info.hwbuild = %d\n", __func__, boardid_info.hwbuild);
	//printk("%s:boardid_info.sku = %d\n", __func__, boardid_info.sku);

}

#ifdef CONFIG_INPUT_ISL29023
//HP zhanghong:Oct 15 18:04 CST 2010,begin
#if 0
#define ISL29023_GPIO_LGS_INT_N PM8058_GPIO_PM_TO_SYS(34)
#else
#define ISL29023_GPIO_LGS_INT_N_TOPAZ_WIFI_PROTO 126
#define ISL29023_GPIO_LGS_INT_N_TOPAZ_3G_PROTO   83
#define ISL29023_INT_GPIO_CFG_TOPAZ_WIFI_PROTO  GPIO_CFG(ISL29023_GPIO_LGS_INT_N_TOPAZ_WIFI_PROTO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define ISL29023_INT_GPIO_CFG_TOPAZ_3G_PROTO    GPIO_CFG(ISL29023_GPIO_LGS_INT_N_TOPAZ_3G_PROTO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
static unsigned int isl29023_gpio_sts = 0;
#endif
//End

static int intersil_isl29023_power(int on)
{
	static struct regulator * votg_L14A_2V8;
	votg_L14A_2V8 = regulator_get(NULL, "8058_l14");
	regulator_set_voltage(votg_L14A_2V8, 2850000, 2850000);

	if (on) {
		if (IS_ERR(votg_L14A_2V8)) {
			pr_err("%s: unable to get 8058_l14\n", __func__);
			return -EINVAL;
		}
		if (regulator_enable(votg_L14A_2V8)) {
			pr_err("%s: Unable to enable the regulator:"
					" 8058_l14\n", __func__);
			return -EINVAL;
		}
		pr_err("%s:  8058_l14 power on ok\n", __func__);
		mdelay(1);
	}
	else{
		if (regulator_disable(votg_L14A_2V8)) {
			pr_err("%s: Unable to disable the regulator:"
					" 8058_l14\n", __func__);
			return -EINVAL;
		}
		regulator_put(votg_L14A_2V8);
		pr_err("%s:  8058_l14 power off\n", __func__);
		mdelay(1);
	}
	return 0;
}

//HP zhanghong: Oct 15 18:09 CST 2010,begin
static int intersil_isl29023_gpio_config(int on)
{
	int rc=0;
	unsigned int gpio_irq,flags;
	printk(KERN_INFO"%s\n",__func__);

	//HP zhanghong: Dec 21 13:25 CST 2010, begin
	//No ALS_INT pin for OPAL
  if(board_is_opal_3g() || board_is_opal_wifi())
    {
      //do nothing
			return 0;
    }
	//No ALS_INT pin since topaz 3G EVT
	if(board_is_topaz_3g() && (board_type > TOPAZ3G_PROTO))
		{
			//do nothing
			return 0;
		}
	//End

	if(board_is_topaz_wifi())
		{
			gpio_irq = ISL29023_GPIO_LGS_INT_N_TOPAZ_WIFI_PROTO;
			flags = ISL29023_INT_GPIO_CFG_TOPAZ_WIFI_PROTO;
		}
	else{
			gpio_irq = ISL29023_GPIO_LGS_INT_N_TOPAZ_3G_PROTO;
			flags = ISL29023_INT_GPIO_CFG_TOPAZ_3G_PROTO;
		}
		
	
	if(!isl29023_gpio_sts ^ !!on)
	{
		pr_err("%s: have configurated the gpio (%d)",__func__,
					gpio_irq);
		goto out;
	}

	if(on)
	{
		rc = gpio_request(gpio_irq,"isl29023_int");
		if(rc < 0)			
			{
				pr_err("%s: gpio_request(%d)"
						"failed", __func__,
						gpio_irq);
				goto out;
			}
		rc = gpio_tlmm_config(flags,GPIO_CFG_ENABLE);
		if(rc)
			{
				pr_err("%s: unable to configure gpio %d\n",
						__func__, gpio_irq);
				goto free_out;
			}
		isl29023_gpio_sts = 1;
	}
	else
		{
			isl29023_gpio_sts = 0;
			goto free_out;
		}

	return rc;

free_out:
	gpio_free(gpio_irq);
out:
	return rc;
}
//End

static struct intersil_isl29023_platform_data intersil_isl29023_pdata = {
	.power = intersil_isl29023_power,
	//HP zhanghong:Oct 15 18:08 CST 2010,begin
	.configure_int_pin = intersil_isl29023_gpio_config,
	//End
	//.p_out = ISL29023_GPIO_LGS_INT_N,
};


static struct i2c_board_info intersil_isl29023[] __initdata = {

	{ I2C_BOARD_INFO("isl29023", 0x44),
		.platform_data = &intersil_isl29023_pdata,
		//.irq = MSM_GPIO_TO_INT(ISL29023_GPIO_LGS_INT_N),
	},
};

/*
static struct platform_device intersil_isl29023_device = {
	.name          = INTERSIL_ISL29023,
	.id            = -1,
	.dev           = {
		.platform_data = &intersil_isl29023_pdata,
	},
};
*/
#endif

#ifdef CONFIG_SERIAL_MSM_HS
static int configure_uart_gpios(int on)
{
	int ret = 0, i;
	int uart_gpios[] = {53, 54, 55, 56};
	for (i = 0; i < ARRAY_SIZE(uart_gpios); i++) {
		if (on) {
			ret = msm_gpiomux_get(uart_gpios[i]);
			if (unlikely(ret))
				break;
		} else {
			ret = msm_gpiomux_put(uart_gpios[i]);
			if (unlikely(ret))
				return ret;
		}
	}
	if (ret)
		for (; i >= 0; i--)
			msm_gpiomux_put(uart_gpios[i]);
	return ret;
}
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
		.inject_rx_on_wakeup = 1,
		.rx_to_inject = 0xFD,
		.gpio_config = configure_uart_gpios,
};
#endif

static struct gpio_keys_button topaz_wifi_gpio_keys_buttons[] = {
	{
		.code           = KEY_HOME,
		.gpio           = PM8058_GPIO_PM_TO_SYS(0),
		.desc           = "Home",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0
	},
	{
		.code           = KEY_BACK,
		.gpio           = PM8058_GPIO_PM_TO_SYS(1),
		.desc           = "Back",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0
	},
	{
		.code           = KEY_MENU,
		.gpio           = PM8058_GPIO_PM_TO_SYS(2),
		.desc           = "Menu",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0
	},
	{
		.code           = KEY_VOLUMEUP,
		.gpio           = 103,
		.desc           = "VolUp",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0
	},
	{
		.code           = KEY_VOLUMEDOWN,
		.gpio           = 104,
		.desc           = "VolDn",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0
	},
	{
		.code           = KEY_BACK,
		.gpio           = 40,
		.desc           = "Back",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0
	},
};

static struct gpio_keys_button topaz_3g_gpio_keys_buttons[] = {
	{
		.code           = KEY_VOLUMEUP,
		.gpio           = PM8058_GPIO_PM_TO_SYS(5),
		.desc           = "VolUp",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0
	},
	{
		.code           = KEY_VOLUMEDOWN,
		.gpio           = PM8058_GPIO_PM_TO_SYS(6),
		.desc           = "VolDn",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0
	},
	{
		.code           = KEY_BACK,
		.gpio           = 40,
		.desc           = "Back",
		.active_low     = 1,
		.type		= EV_KEY,
		.wakeup		= 0
	},
};

static struct gpio_keys_platform_data topaz_3g_gpio_keys_data = {
	.buttons        = topaz_3g_gpio_keys_buttons,
	.nbuttons       = ARRAY_SIZE(topaz_3g_gpio_keys_buttons),
	.rep		= 0,
};

static struct gpio_keys_platform_data topaz_wifi_gpio_keys_data = {
	.buttons        = topaz_wifi_gpio_keys_buttons,
	.nbuttons       = ARRAY_SIZE(topaz_wifi_gpio_keys_buttons),
	.rep		= 0,
};


static struct platform_device msm_gpio_keys = {
	.name           = "gpio-keys",
	.id             = -1,
/*
	//This guy will be valued when msm8x60_init.
	.dev            = {
		.platform_data  = &gpio_keys_data,
	},
*/
};

#ifdef CONFIG_HP_HEADSET

static struct msm_headset_platform_data hs_data = {
		.hs_name = "handset",
		.gpio_detection_irq = 67,
		.gpio_hookkey_irq  = 57,
		.gpio_func_sel = 58,
};

static struct platform_device hs_device = {
		.name   = "topaz-headset",
		.id     = -1,
		.dev    = {
				.platform_data = &hs_data,
		},
};
#endif

#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)

static struct gpio_led gpio_exp_leds_config[] = {
	{
		.name = "left_led1:green",
		.gpio = GPIO_LEFT_LED_1,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "left_led2:red",
		.gpio = GPIO_LEFT_LED_2,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "left_led3:green",
		.gpio = GPIO_LEFT_LED_3,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "wlan_led:orange",
		.gpio = GPIO_LEFT_LED_WLAN,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "left_led5:green",
		.gpio = GPIO_LEFT_LED_5,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "right_led1:green",
		.gpio = GPIO_RIGHT_LED_1,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "right_led2:red",
		.gpio = GPIO_RIGHT_LED_2,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "right_led3:green",
		.gpio = GPIO_RIGHT_LED_3,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "bt_led:blue",
		.gpio = GPIO_RIGHT_LED_BT,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "right_led5:green",
		.gpio = GPIO_RIGHT_LED_5,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data gpio_leds_pdata = {
	.num_leds = ARRAY_SIZE(gpio_exp_leds_config),
	.leds = gpio_exp_leds_config,
};

static struct platform_device gpio_leds = {
	.name          = "leds-gpio",
	.id            = -1,
	.dev           = {
		.platform_data = &gpio_leds_pdata,
	},
};

static struct gpio_led fluid_gpio_leds[] = {
	{
		.name			= "dual_led:green",
		.gpio			= GPIO_LED1_GREEN_N,
		.default_state		= LEDS_GPIO_DEFSTATE_OFF,
		.active_low		= 1,
		.retain_state_suspended = 0,
	},
	{
		.name			= "dual_led:red",
		.gpio			= GPIO_LED2_RED_N,
		.default_state		= LEDS_GPIO_DEFSTATE_OFF,
		.active_low		= 1,
		.retain_state_suspended = 0,
	},
};

static struct gpio_led_platform_data gpio_led_pdata = {
	.leds		= fluid_gpio_leds,
	.num_leds	= ARRAY_SIZE(fluid_gpio_leds),
};

static struct platform_device fluid_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_pdata,
	},
};

#endif

#if defined(CONFIG_AR6003) || defined(CONFIG_AR6003_MODULE)
// the suffix MODULE will be added to your config name if you select module(m) in kernelconfig!
#include <linux/wlan_plat.h>

static int ar6003_wifi_power(int on);
static int ar6003_wifi_reset(int on);
static int ar6003_wifi_set_carddetect(int on);

/*WLAN*/
// for Topaz Wifi only
#define AR6003_GPIO_WL_HOST_WAKE		93		// input
// 2011.2.28 HP Henry: remove it for h/w change #define	AR6003_GPIO_WL_IRQ			94		// input
#define AR6003_GPIO_HOST_WAKE_WL		137		// output
#define AR6003_GPIO_WLAN_RST_N		135		// output
// for Topaz 3G
#define AR6003_GPIO_WL_HOST_WAKE_TOPAZ_3G	93		// input
// 2011.2.28 HP Henry: remove it for h/w change #define	AR6003_GPIO_WL_IRQ_TOPAZ_3G			94		// input
#define AR6003_GPIO_HOST_WAKE_WL_TOPAZ_3G	80		// output
#define AR6003_GPIO_WLAN_RST_N_TOPAZ_3G		28		// output
// for Opal 3G
#define AR6003_GPIO_WL_HOST_WAKE_OPAL_3G	93		// input
// 2011.2.28 HP Henry: remove it for h/w change #define	AR6003_GPIO_WL_IRQ_OPAL_3G			94		// input
#define AR6003_GPIO_HOST_WAKE_WL_OPAL_3G	80		// output
#define AR6003_GPIO_WLAN_RST_N_OPAL_3G		28		// output

static int ar6003_wifi_cd = 0; /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static unsigned ar6003_gpio_wl_host_wake = 0;
// 2011.2.28 HP Henry: remove it for h/w change static unsigned ar6003_gpio_wl_irq = 0;
static unsigned ar6003_gpio_host_wake_wl = 0;
static unsigned ar6003_gpio_wlan_rst_n = 0;

static bool ar6003_wifi_init_gpio_ctrl_pins(void)
{
	bool ret = false;

	if (!ar6003_gpio_wl_host_wake ||
// 2011.2.28 HP Henry: remove it for h/w change		!ar6003_gpio_wl_irq ||	
		!ar6003_gpio_host_wake_wl ||
		!ar6003_gpio_wlan_rst_n)
	{
		if (board_is_topaz_wifi())
		{
			// AR6003_GPIO_WL_HOST_WAKE
			ar6003_gpio_wl_host_wake = AR6003_GPIO_WL_HOST_WAKE;
			// AR6003_GPIO_WL_IRQ
// 2011.2.28 HP Henry: remove it for h/w change			ar6003_gpio_wl_irq = AR6003_GPIO_WL_IRQ;
			// AR6003_GPIO_HOST_WAKE_WL
			ar6003_gpio_host_wake_wl = AR6003_GPIO_HOST_WAKE_WL;
			// AR6003_GPIO_WLAN_RST_N
			ar6003_gpio_wlan_rst_n = AR6003_GPIO_WLAN_RST_N;

			ret = true;
		}
		else if (board_is_topaz_3g())
		{
			// AR6003_GPIO_WL_HOST_WAKE
			ar6003_gpio_wl_host_wake = AR6003_GPIO_WL_HOST_WAKE_TOPAZ_3G;
			// AR6003_GPIO_WL_IRQ
// 2011.2.28 HP Henry: remove it for h/w change			ar6003_gpio_wl_irq = AR6003_GPIO_WL_IRQ_TOPAZ_3G;
			// AR6003_GPIO_HOST_WAKE_WL
			ar6003_gpio_host_wake_wl = AR6003_GPIO_HOST_WAKE_WL_TOPAZ_3G;
			// AR6003_GPIO_WLAN_RST_N
			ar6003_gpio_wlan_rst_n = AR6003_GPIO_WLAN_RST_N_TOPAZ_3G;

			ret = true;
		}
		else if (board_is_opal_3g() ||
			board_is_opal_wifi() )
		{
			// AR6003_GPIO_WL_HOST_WAKE
			ar6003_gpio_wl_host_wake = AR6003_GPIO_WL_HOST_WAKE_OPAL_3G;
			// AR6003_GPIO_WL_IRQ
// 2011.2.28 HP Henry: remove it for h/w change			ar6003_gpio_wl_irq = AR6003_GPIO_WL_IRQ_OPAL_3G;
			// AR6003_GPIO_HOST_WAKE_WL
			ar6003_gpio_host_wake_wl = AR6003_GPIO_HOST_WAKE_WL_OPAL_3G;
			// AR6003_GPIO_WLAN_RST_N
			ar6003_gpio_wlan_rst_n = AR6003_GPIO_WLAN_RST_N_OPAL_3G;

			ret = true;
		}
		else
		{
			// empty
			pr_err("wlan: cannot config GPIO ctrl-pins\n");
		}
	}

	return ret;
}

static int ar6003_wifi_set_carddetect(int val)
{
	ar6003_wifi_cd = val;

	printk("Board-msm8x60.c wifi_set_carddetect(%d)\n", val);

	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warning("%s: Nobody to notify\n", __func__);

	return 0;
}

static int ar6003_wifi_status_register(
			void (*callback)(int card_present, void *dev_id),
			void *dev_id)
{
	printk("Board-msm8x60.c wifi_status_register\n");

	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}


static unsigned int ar6003_wifi_status(struct device *dev)
{
	printk("Board-msm8x60.c wifi_status(%d)\n", ar6003_wifi_cd);

	return ar6003_wifi_cd;
}

static int ar6003_wifi_power(int on)
{
	static struct regulator * votg_L1B_3V3;
	static struct regulator * votg_L3B_3V3;
	static struct regulator * votg_L19A_1V8;
	static int wifi_is_on = 0;
//	static int rc = -EINVAL; /* remember if the gpio_requests succeeded */
	static int wifi_power_is_on = 0;	// h/w suggestion: keep all power on for power consumption

	ar6003_wifi_init_gpio_ctrl_pins();

	// set power supply
	if (on)
	{
		// Open WLAN power
		// VREG_L1B_3V3	==> "8901_l1"   3.3V
		// VREG_L19A_1V8 ==> "8058_l19" 1.8V

		// A. set level

		// B. enable power

		//3.3V -> 1.8V -> wait (Tb 5) -> CHIP_PWD

		if (wifi_power_is_on == 0)
		{
			printk("Board-msm8x60.c wifi_power(%d) 8901_l3 3.3V \n", on);

			votg_L3B_3V3 = regulator_get(NULL, "8901_l3");
			if (IS_ERR(votg_L3B_3V3)) {
				pr_err("%s: unable to get 8901_l3\n", __func__);
				return -EINVAL;
			}
			if (regulator_set_voltage(votg_L3B_3V3, 3300000, 3300000))
			{
				pr_err("%s: unable to set voltage for 8901_l3\n", __func__);
				return -EINVAL;
			}
			if (regulator_enable(votg_L3B_3V3)) {
				pr_err("%s: Unable to enable the regulator:"
					" 8901_l3\n", __func__);
				return -EINVAL;
			}

			printk("Board-msm8x60.c wifi_power(%d) 8901_l1 3.3V\n", on);

			votg_L1B_3V3 = regulator_get(NULL, "8901_l1");
			if (IS_ERR(votg_L1B_3V3)) {
				pr_err("%s: unable to get 8901_l1\n", __func__);
				return -EINVAL;
			}
			if (regulator_set_voltage(votg_L1B_3V3, 3300000, 3300000))
			{
				pr_err("%s: unable to set voltage for 8901_l3\n", __func__);
				return -EINVAL;
			}
			if (regulator_enable(votg_L1B_3V3)) {
				pr_err("%s: Unable to enable the regulator:"
					" 8901_l1\n", __func__);
				return -EINVAL;
			}

			printk("Board-msm8x60.c wifi_power(%d) 8058_l19 1.8V\n", on);

			votg_L19A_1V8 = regulator_get(NULL, "8058_l19");
			if (IS_ERR(votg_L19A_1V8)) {
				pr_err("%s: unable to get 8058_l19\n", __func__);
				return -EINVAL;
			}
			if (regulator_set_voltage(votg_L19A_1V8, 1800000, 1800000))
			{
				pr_err("%s: unable to set voltage for 8901_l3\n", __func__);
				return -EINVAL;
			}
			if (regulator_enable(votg_L19A_1V8)) {
				pr_err("%s: Unable to enable the regulator:"
					" 8058_l19\n", __func__);
				return -EINVAL;
			}
		}
		else
		{
			printk("Board-msm8x60.c wifi_power has enabled\n");
		}
		wifi_power_is_on = 1;

		printk("Board-msm8x60.c wifi_power(%d) CHIP_PWD\n", on);

		//michael - reset wifi
		mdelay(5);
	}

	// set gpio
	if (on)
	{
		if (wifi_is_on == 0)
		{
			printk("%s: set GPIO_WLAN_RST_N to high\n", __func__);
			/*
			rc = gpio_request(AR6003_GPIO_WLAN_RST_N, "WLAN_RST_N");
			if (rc)
			{
			pr_err("%s:  gpio %d request failed\n",	__func__, AR6003_GPIO_WLAN_RST_N);
			gpio_free(AR6003_GPIO_WLAN_RST_N);
			return -EINVAL;
			}

			gpio_direction_output(AR6003_GPIO_WLAN_RST_N, 0);

			mdelay(5);

			gpio_direction_output(AR6003_GPIO_WLAN_RST_N, 1);
			*/
			//gpio_set_value(AR6003_GPIO_WLAN_RST_N, 0);
			gpio_set_value(ar6003_gpio_wlan_rst_n, 0);

			mdelay(5);

			//gpio_set_value(AR6003_GPIO_WLAN_RST_N, 1);
			gpio_set_value(ar6003_gpio_wlan_rst_n, 1);

		}		

		wifi_is_on = 1;
		printk("***WLAN enable power\n");

	}
	else
	{
		//CHIP_PWD -> wait (Tc 5)

		if (wifi_is_on == 1)
		{
			printk("%s: set GPIO_WLAN_RST_N to low\n", __func__);

			//		gpio_direction_output(AR6003_GPIO_WLAN_RST_N, 0);

			//gpio_set_value(AR6003_GPIO_WLAN_RST_N, 0);
			gpio_set_value(ar6003_gpio_wlan_rst_n, 0);

			mdelay(5);

			//		gpio_free(AR6003_GPIO_WLAN_RST_N);
		}

		wifi_is_on = 0;
		printk("***WLAN disable power\n");
	}

	return 0;
}

static int ar6003_wifi_reset_state;

static int ar6003_wifi_reset(int on)
{
	printk("%s: do nothing\n", __func__);
	ar6003_wifi_reset_state = on;
	return 0;
}

// 2011.2.28 HP Henry: remove it for h/w change
/*
static struct resource ar6003_wifi_resources[] = {
	[0] = {
		.name		= "ar6003_wlan_irq",
		.start		= MSM_GPIO_TO_INT(AR6003_GPIO_WL_IRQ),
		.end		= MSM_GPIO_TO_INT(AR6003_GPIO_WL_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};
*/

int wifi_get_board_type(char ** pPlatform, u32* pBoardType)
{
	if (board_is_topaz_wifi_flag)
		*pPlatform ="topaz_wifi";
	else if (board_is_topaz_3g_flag)
		*pPlatform = "topaz_3g";
	else if (board_is_opal_3g_flag)
		*pPlatform = "opal_3g";
	else if (board_is_opal_wifi_flag) 
		*pPlatform = "opal_wifi";

	*pBoardType = board_type;
	
	return 0;
}
EXPORT_SYMBOL(wifi_get_board_type);

static struct wifi_platform_data ar6003_wifi_control = {
	.set_power      = ar6003_wifi_power,
	.set_reset      = ar6003_wifi_reset,
	.set_carddetect = ar6003_wifi_set_carddetect,
};

static struct platform_device ar6003_wifi_device = {
	.name           = "wlan_ar6000_pm_dev",
	.id             = 1,
// 2011.2.28 HP Henry: remove it for h/w change	.num_resources  = ARRAY_SIZE(ar6003_wifi_resources),
// 2011.2.28 HP Henry: remove it for h/w change	.resource       = ar6003_wifi_resources,
	.dev            = {
		.platform_data = &ar6003_wifi_control,
	},
};

#if 0
/* 2010.8.25 HP Henry: move "ar6003_wifi_device" to "ar6003_wifi_device".
static int __init ar6003_wifi_init(void)
{
	int ret;

	if (!machine_is_msm8x60_topaz() && !machine_is_msm8x60_opal())
		return 0;

	ret = platform_device_register(&ar6003_wifi_device);
	return ret;
}

late_initcall(ar6003_wifi_init);
*/
#endif

#endif // CONFIG_AR6003

#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)

static struct msm_rpm_log_platform_data msm_rpm_log_pdata = {
	.phys_addr_base = 0x00106000,
	.reg_offsets = {
		[MSM_RPM_LOG_PAGE_INDICES] = 0x00000C80,
		[MSM_RPM_LOG_PAGE_BUFFER]  = 0x00000CA0,
	},
	.phys_size = SZ_8K,
	.log_len = 4096,		  /* log's buffer length in bytes */
	.log_len_mask = (4096 >> 2) - 1,  /* length mask in units of u32 */
};

static struct platform_device msm_rpm_log_device = {
	.name	= "msm_rpm_log",
	.id	= -1,
	.dev	= {
		.platform_data = &msm_rpm_log_pdata,
	},
};
#endif

#ifdef CONFIG_BATTERY_MSM8X60
static struct msm_charger_platform_data msm_charger_data = {
	.safety_time = 180,
	.update_time = 1,
	.max_voltage = 4200,
	.min_voltage = 3200,
	.resume_voltage = 4100,
};

static struct platform_device msm_charger_device = {
	.name = "msm-charger",
	.id = -1,
	.dev = {
		.platform_data = &msm_charger_data,
	}
};
#endif

static struct regulator_consumer_supply rpm_vreg_supply[RPM_VREG_ID_MAX] = {
	[RPM_VREG_ID_PM8058_L0]  = REGULATOR_SUPPLY("8058_l0", NULL),
	[RPM_VREG_ID_PM8058_L1]  = REGULATOR_SUPPLY("8058_l1", NULL),
	[RPM_VREG_ID_PM8058_L2]  = REGULATOR_SUPPLY("8058_l2", NULL),
	[RPM_VREG_ID_PM8058_L3]  = REGULATOR_SUPPLY("8058_l3", NULL),
	[RPM_VREG_ID_PM8058_L4]  = REGULATOR_SUPPLY("8058_l4", NULL),
	[RPM_VREG_ID_PM8058_L5]  = REGULATOR_SUPPLY("8058_l5", NULL),
	[RPM_VREG_ID_PM8058_L6]  = REGULATOR_SUPPLY("8058_l6", NULL),
	[RPM_VREG_ID_PM8058_L7]  = REGULATOR_SUPPLY("8058_l7", NULL),
	[RPM_VREG_ID_PM8058_L8]  = REGULATOR_SUPPLY("8058_l8", NULL),
	[RPM_VREG_ID_PM8058_L9]  = REGULATOR_SUPPLY("8058_l9", NULL),
	[RPM_VREG_ID_PM8058_L10] = REGULATOR_SUPPLY("8058_l10", NULL),
	[RPM_VREG_ID_PM8058_L11] = REGULATOR_SUPPLY("8058_l11", NULL),
	[RPM_VREG_ID_PM8058_L12] = REGULATOR_SUPPLY("8058_l12", NULL),
	[RPM_VREG_ID_PM8058_L13] = REGULATOR_SUPPLY("8058_l13", NULL),
	[RPM_VREG_ID_PM8058_L14] = REGULATOR_SUPPLY("8058_l14", NULL),
	[RPM_VREG_ID_PM8058_L15] = REGULATOR_SUPPLY("8058_l15", NULL),
	[RPM_VREG_ID_PM8058_L16] = REGULATOR_SUPPLY("8058_l16", NULL),
	[RPM_VREG_ID_PM8058_L17] = REGULATOR_SUPPLY("8058_l17", NULL),
	[RPM_VREG_ID_PM8058_L18] = REGULATOR_SUPPLY("8058_l18", NULL),
	[RPM_VREG_ID_PM8058_L19] = REGULATOR_SUPPLY("8058_l19", NULL),
	[RPM_VREG_ID_PM8058_L20] = REGULATOR_SUPPLY("8058_l20", NULL),
	[RPM_VREG_ID_PM8058_L21] = REGULATOR_SUPPLY("8058_l21", NULL),
	[RPM_VREG_ID_PM8058_L22] = REGULATOR_SUPPLY("8058_l22", NULL),
	[RPM_VREG_ID_PM8058_L23] = REGULATOR_SUPPLY("8058_l23", NULL),
	[RPM_VREG_ID_PM8058_L24] = REGULATOR_SUPPLY("8058_l24", NULL),
	[RPM_VREG_ID_PM8058_L25] = REGULATOR_SUPPLY("8058_l25", NULL),

	[RPM_VREG_ID_PM8058_S0] = REGULATOR_SUPPLY("8058_s0", NULL),
	[RPM_VREG_ID_PM8058_S1] = REGULATOR_SUPPLY("8058_s1", NULL),
	[RPM_VREG_ID_PM8058_S2] = REGULATOR_SUPPLY("8058_s2", NULL),
	[RPM_VREG_ID_PM8058_S3] = REGULATOR_SUPPLY("8058_s3", NULL),
	[RPM_VREG_ID_PM8058_S4] = REGULATOR_SUPPLY("8058_s4", NULL),

	[RPM_VREG_ID_PM8058_LVS0] = REGULATOR_SUPPLY("8058_lvs0", NULL),
	[RPM_VREG_ID_PM8058_LVS1] = REGULATOR_SUPPLY("8058_lvs1", NULL),

	[RPM_VREG_ID_PM8058_NCP] = REGULATOR_SUPPLY("8058_ncp", NULL),

	[RPM_VREG_ID_PM8901_L0]  = REGULATOR_SUPPLY("8901_l0",  NULL),
	[RPM_VREG_ID_PM8901_L1]  = REGULATOR_SUPPLY("8901_l1",  NULL),
	[RPM_VREG_ID_PM8901_L2]  = REGULATOR_SUPPLY("8901_l2",  NULL),
	[RPM_VREG_ID_PM8901_L3]  = REGULATOR_SUPPLY("8901_l3",  NULL),
	[RPM_VREG_ID_PM8901_L4]  = REGULATOR_SUPPLY("8901_l4",  NULL),
	[RPM_VREG_ID_PM8901_L5]  = REGULATOR_SUPPLY("8901_l5",  NULL),
	[RPM_VREG_ID_PM8901_L6]  = REGULATOR_SUPPLY("8901_l6",  NULL),

	[RPM_VREG_ID_PM8901_S2] = REGULATOR_SUPPLY("8901_s2", NULL),
	[RPM_VREG_ID_PM8901_S3] = REGULATOR_SUPPLY("8901_s3", NULL),
	[RPM_VREG_ID_PM8901_S4] = REGULATOR_SUPPLY("8901_s4", NULL),

	[RPM_VREG_ID_PM8901_LVS0] = REGULATOR_SUPPLY("8901_lvs0", NULL),
	[RPM_VREG_ID_PM8901_LVS1] = REGULATOR_SUPPLY("8901_lvs1", NULL),
	[RPM_VREG_ID_PM8901_LVS2] = REGULATOR_SUPPLY("8901_lvs2", NULL),
	[RPM_VREG_ID_PM8901_LVS3] = REGULATOR_SUPPLY("8901_lvs3", NULL),
	[RPM_VREG_ID_PM8901_MVS0] = REGULATOR_SUPPLY("8901_mvs0", NULL),
};

#define RPM_VREG_INIT(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV, \
			  _default_uV, _peak_uA, _avg_uA, _pull_down, _pin_ctrl, \
		      _freq, _pin_fn, _rpm_mode, _state, _sleep_selectable, \
		      _always_on) \
	[_id] = { \
		.init_data = { \
			.constraints = { \
				.valid_modes_mask = _modes, \
				.valid_ops_mask = _ops, \
				.min_uV = _min_uV, \
				.max_uV = _max_uV, \
				.input_uV = _min_uV, \
				.apply_uV = _apply_uV, \
				.always_on = _always_on, \
			}, \
			.num_consumer_supplies = 1, \
			.consumer_supplies = &rpm_vreg_supply[_id], \
		}, \
		.default_uV = _default_uV, \
		.peak_uA = _peak_uA, \
		.avg_uA = _avg_uA, \
		.pull_down_enable = _pull_down, \
		.pin_ctrl = _pin_ctrl, \
		.freq = _freq, \
		.pin_fn = _pin_fn, \
		.mode = _rpm_mode, \
		.state = _state, \
		.sleep_selectable = _sleep_selectable, \
	}

/*
 * Passing this voltage to the RPM will vote for HPM.  Use it as a default in
 * case consumers do not specify their current requirements via
 * regulator_set_optimum_mode.
 */
#define RPM_HPM_UV	(51000)

#define RPM_VREG_INIT_LDO(_id, _always_on, _pd, _sleep_selectable, _min_uV, \
			  _max_uV, _pin_ctrl) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
			  REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
			  REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
			  REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
			  REGULATOR_CHANGE_DRMS, 0, _min_uV, RPM_HPM_UV, \
			  RPM_HPM_UV, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
			  RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_SMPS(_id, _always_on, _pd, _sleep_selectable, _min_uV, \
			   _max_uV, _pin_ctrl, _freq) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
			  REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
			  REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
			  REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
			  REGULATOR_CHANGE_DRMS, 0, _min_uV, RPM_HPM_UV, \
			  RPM_HPM_UV, _pd, _pin_ctrl, _freq, \
			  RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_VS(_id, _always_on, _pd, _sleep_selectable, _pin_ctrl) \
	RPM_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE, \
			  REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE, 0, 0, \
			  RPM_HPM_UV, RPM_HPM_UV, _pd, _pin_ctrl, \
			  RPM_VREG_FREQ_NONE, RPM_VREG_PIN_FN_ENABLE, \
		      RPM_VREG_MODE_NONE, RPM_VREG_STATE_OFF, \
		      _sleep_selectable, _always_on)

#define RPM_VREG_INIT_NCP(_id, _always_on, _pd, _sleep_selectable, _min_uV, \
			  _max_uV, _pin_ctrl) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL, \
			  REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS, 0, \
			  _min_uV, RPM_HPM_UV, RPM_HPM_UV, _pd, _pin_ctrl, \
			  RPM_VREG_FREQ_NONE, RPM_VREG_PIN_FN_ENABLE, \
		      RPM_VREG_MODE_NONE, RPM_VREG_STATE_OFF, \
		      _sleep_selectable, _always_on)

static struct rpm_vreg_pdata rpm_vreg_init_pdata[RPM_VREG_ID_MAX] = {
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L0,  1, 1, 0, 1200000, 1200000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L1,  0, 1, 0, 1200000, 1200000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L2,  0, 1, 0, 1800000, 2600000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L3,  0, 1, 0, 1800000, 1800000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L4,  0, 1, 0, 2850000, 2850000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L5,  1, 1, 0, 2850000, 2850000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L6,  0, 1, 0, 3000000, 3600000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L7,  0, 1, 0, 1800000, 1800000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L8,  0, 1, 0, 2900000, 2900000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L9,  0, 1, 0, 1800000, 1800000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L10, 0, 1, 0, 2600000, 2600000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L11, 0, 1, 0, 1500000, 1500000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L12, 0, 1, 0, 1800000, 1800000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L13, 0, 1, 0, 2050000, 2050000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L14, 1, 0, 0, 2850000, 2850000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L15, 0, 1, 0, 2850000, 2850000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L16, 1, 1, 1, 1800000, 1800000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L17, 0, 1, 0, 2600000, 2600000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L18, 0, 1, 1, 2200000, 2200000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L19, 0, 1, 0, 2500000, 2500000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L20, 0, 1, 0, 1800000, 1800000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L21, 1, 1, 0, 1200000, 1200000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L22, 0, 1, 0, 1200000, 1200000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L23, 0, 1, 0, 1200000, 1200000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L24, 0, 1, 0, 1200000, 1200000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8058_L25, 0, 1, 0, 1200000, 1200000, 0),

	RPM_VREG_INIT_SMPS(RPM_VREG_ID_PM8058_S0, 0, 1, 1,  500000, 1200000, 0,
		RPM_VREG_FREQ_1p75),
	RPM_VREG_INIT_SMPS(RPM_VREG_ID_PM8058_S1, 0, 1, 1,  500000, 1200000, 0,
		RPM_VREG_FREQ_1p75),
	RPM_VREG_INIT_SMPS(RPM_VREG_ID_PM8058_S2, 1, 1, 0, 1200000, 1400000,
		RPM_VREG_PIN_CTRL_A0, RPM_VREG_FREQ_1p75),
	RPM_VREG_INIT_SMPS(RPM_VREG_ID_PM8058_S3, 1, 1, 0, 1800000, 1800000, 0,
		RPM_VREG_FREQ_1p75),
	RPM_VREG_INIT_SMPS(RPM_VREG_ID_PM8058_S4, 1, 1, 0, 2200000, 2200000, 0,
		RPM_VREG_FREQ_1p75),

	RPM_VREG_INIT_VS(RPM_VREG_ID_PM8058_LVS0, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(RPM_VREG_ID_PM8058_LVS1, 0, 1, 0,		     0),

	RPM_VREG_INIT_NCP(RPM_VREG_ID_PM8058_NCP, 0, 1, 0, 1800000, 1800000, 0),

	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8901_L0,  0, 1, 0, 1200000, 1200000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8901_L1,  0, 1, 0, 3300000, 3300000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8901_L2,  0, 1, 0, 2850000, 3300000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8901_L3,  0, 1, 0, 3300000, 3300000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8901_L4,  0, 1, 0, 2600000, 2600000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8901_L5,  1, 1, 0, 2850000, 2850000, 0),
	RPM_VREG_INIT_LDO(RPM_VREG_ID_PM8901_L6,  0, 1, 0, 2200000, 2200000, 0),

	RPM_VREG_INIT_SMPS(RPM_VREG_ID_PM8901_S2, 0, 1, 0, 1300000, 1300000, 0,
		RPM_VREG_FREQ_1p75),
	RPM_VREG_INIT_SMPS(RPM_VREG_ID_PM8901_S3, 0, 1, 0, 1100000, 1100000, 0,
		RPM_VREG_FREQ_1p75),
	RPM_VREG_INIT_SMPS(RPM_VREG_ID_PM8901_S4, 0, 1, 0, 1225000, 1225000,
		RPM_VREG_PIN_CTRL_A0, RPM_VREG_FREQ_1p75),

	RPM_VREG_INIT_VS(RPM_VREG_ID_PM8901_LVS0, 1, 1, 0,		     0),
	RPM_VREG_INIT_VS(RPM_VREG_ID_PM8901_LVS1, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(RPM_VREG_ID_PM8901_LVS2, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(RPM_VREG_ID_PM8901_LVS3, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(RPM_VREG_ID_PM8901_MVS0, 0, 1, 0,		     0),
};

/* HP Wade: Customize rpm_vreg_init_pdata based on out platform*/
#define RPM_VREG_INIT_CUST(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV, \
			  _default_uV, _peak_uA, _avg_uA, _pull_down, _pin_ctrl, \
		      _freq, _pin_fn, _rpm_mode, _state, _sleep_selectable, \
		      _always_on) \
	{_id, { \
		.init_data = { \
			.constraints = { \
				.valid_modes_mask = _modes, \
				.valid_ops_mask = _ops, \
				.min_uV = _min_uV, \
				.max_uV = _max_uV, \
				.input_uV = _min_uV, \
				.apply_uV = _apply_uV, \
				.always_on = _always_on, \
			}, \
			.num_consumer_supplies = 1, \
			.consumer_supplies = &rpm_vreg_supply[_id], \
		}, \
		.default_uV = _default_uV, \
		.peak_uA = _peak_uA, \
		.avg_uA = _avg_uA, \
		.pull_down_enable = _pull_down, \
		.pin_ctrl = _pin_ctrl, \
		.freq = _freq, \
		.pin_fn = _pin_fn, \
		.mode = _rpm_mode, \
		.state = _state, \
		.sleep_selectable = _sleep_selectable, \
	}}

#define RPM_VREG_INIT_LDO_CUST(_id, _always_on, _pd, _sleep_selectable, _min_uV, \
			  _max_uV, _pin_ctrl) \
	RPM_VREG_INIT_CUST(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
			  REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
			  REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
			  REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
			  REGULATOR_CHANGE_DRMS, 0, _min_uV, RPM_HPM_UV, \
			  RPM_HPM_UV, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
			  RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
			  RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_VS_CUST(_id, _always_on, _pd, _sleep_selectable, _pin_ctrl) \
	RPM_VREG_INIT_CUST(_id, 0, 0, REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE, \
			  REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE, 0, 0, \
			  RPM_HPM_UV, RPM_HPM_UV, _pd, _pin_ctrl, \
			  RPM_VREG_FREQ_NONE, RPM_VREG_PIN_FN_ENABLE, \
			  RPM_VREG_MODE_NONE, RPM_VREG_STATE_OFF, \
		      _sleep_selectable, _always_on)

static struct rpm_vreg_customized_pdata rpm_vreg_init_customized_pdata__topaz[] = {
	RPM_VREG_INIT_LDO_CUST(RPM_VREG_ID_PM8058_L10, 1, 1, 0, 3050000, 3050000, 0),//// Wade: PM8058_VREG_INIT_LDO_ALWAYS_ON(PM8058_VREG_ID_L10, 3050000, 3050000),
	RPM_VREG_INIT_LDO_CUST(RPM_VREG_ID_PM8058_L11, 0, 1, 0, 2850000, 2850000, 0),//// Wade: PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L11, 2850000, 2850000),
	RPM_VREG_INIT_LDO_CUST(RPM_VREG_ID_PM8058_L15, 0, 1, 0, 2850000, 2850000, 0),//// Wade: PM8058_VREG_INIT_LDO_ALWAYS_ON(PM8058_VREG_ID_L15, 2850000, 2850000),  
	RPM_VREG_INIT_LDO_CUST(RPM_VREG_ID_PM8058_L19, 0, 1, 0, 1800000, 1800000, 0),//// Wade: PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L19, 1800000, 1800000), //wifi 2.5 -> 1.8
	//RPM_VREG_INIT_VS_CUST(RPM_VREG_ID_PM8901_LVS3, 1, 1, 0,			  0), //// Wade: for Gyro because of timing issue
};
// End

#define RPM_VREG(_id) \
	[_id] = { \
		.name = "rpm-regulator", \
		.id = _id, \
		.dev = { \
			.platform_data = &rpm_vreg_init_pdata[_id], \
		}, \
	}

static struct platform_device rpm_vreg_device[RPM_VREG_ID_MAX] = {
	RPM_VREG(RPM_VREG_ID_PM8058_L0),
	RPM_VREG(RPM_VREG_ID_PM8058_L1),
	RPM_VREG(RPM_VREG_ID_PM8058_L2),
	RPM_VREG(RPM_VREG_ID_PM8058_L3),
	RPM_VREG(RPM_VREG_ID_PM8058_L4),
	RPM_VREG(RPM_VREG_ID_PM8058_L5),
	RPM_VREG(RPM_VREG_ID_PM8058_L6),
	RPM_VREG(RPM_VREG_ID_PM8058_L7),
	RPM_VREG(RPM_VREG_ID_PM8058_L8),
	RPM_VREG(RPM_VREG_ID_PM8058_L9),
	RPM_VREG(RPM_VREG_ID_PM8058_L10),
	RPM_VREG(RPM_VREG_ID_PM8058_L11),
	RPM_VREG(RPM_VREG_ID_PM8058_L12),
	RPM_VREG(RPM_VREG_ID_PM8058_L13),
	RPM_VREG(RPM_VREG_ID_PM8058_L14),
	RPM_VREG(RPM_VREG_ID_PM8058_L15),
	RPM_VREG(RPM_VREG_ID_PM8058_L16),
	RPM_VREG(RPM_VREG_ID_PM8058_L17),
	RPM_VREG(RPM_VREG_ID_PM8058_L18),
	RPM_VREG(RPM_VREG_ID_PM8058_L19),
	RPM_VREG(RPM_VREG_ID_PM8058_L20),
	RPM_VREG(RPM_VREG_ID_PM8058_L21),
	RPM_VREG(RPM_VREG_ID_PM8058_L22),
	RPM_VREG(RPM_VREG_ID_PM8058_L23),
	RPM_VREG(RPM_VREG_ID_PM8058_L24),
	RPM_VREG(RPM_VREG_ID_PM8058_L25),
	RPM_VREG(RPM_VREG_ID_PM8058_S0),
	RPM_VREG(RPM_VREG_ID_PM8058_S1),
	RPM_VREG(RPM_VREG_ID_PM8058_S2),
	RPM_VREG(RPM_VREG_ID_PM8058_S3),
	RPM_VREG(RPM_VREG_ID_PM8058_S4),
	RPM_VREG(RPM_VREG_ID_PM8058_LVS0),
	RPM_VREG(RPM_VREG_ID_PM8058_LVS1),
	RPM_VREG(RPM_VREG_ID_PM8058_NCP),
	RPM_VREG(RPM_VREG_ID_PM8901_L0),
	RPM_VREG(RPM_VREG_ID_PM8901_L1),
	RPM_VREG(RPM_VREG_ID_PM8901_L2),
	RPM_VREG(RPM_VREG_ID_PM8901_L3),
	RPM_VREG(RPM_VREG_ID_PM8901_L4),
	RPM_VREG(RPM_VREG_ID_PM8901_L5),
	RPM_VREG(RPM_VREG_ID_PM8901_L6),
	RPM_VREG(RPM_VREG_ID_PM8901_S2),
	RPM_VREG(RPM_VREG_ID_PM8901_S3),
	RPM_VREG(RPM_VREG_ID_PM8901_S4),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS0),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS1),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS2),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS3),
	RPM_VREG(RPM_VREG_ID_PM8901_MVS0),
};

static struct platform_device *early_regulators[] __initdata = {
	&msm_device_saw_s0,
	&msm_device_saw_s1,
#ifdef CONFIG_PMIC8058
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S1],
#endif
};

static struct platform_device *early_devices[] __initdata = {
#ifdef CONFIG_MSM_BUS_SCALING
	&msm_bus_apps_fabric,
	&msm_bus_sys_fabric,
	&msm_bus_mm_fabric,
	&msm_bus_sys_fpb,
	&msm_bus_cpss_fpb,
#endif
};

#if (defined(CONFIG_BAHAMA_CORE)) && \
	(defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE)) 

static int bluetooth_power(int);
static struct platform_device msm_bt_power_device = {
	.name	 = "bt_power",
	.id	 = -1,
	.dev	 = {
		.platform_data = &bluetooth_power,
	},
};
#endif

#ifdef CONFIG_BT
static int bluetooth_csr_power(int);
static struct platform_device msm_bt_csr_power_device = {
	.name	 = "bt_power",
	.id	 = -1,
	.dev	 = {
		.platform_data = &bluetooth_csr_power,
	},
};
#endif

//wanqin
#if defined(CONFIG_BT) && defined(CONFIG_BT_MSM_SLEEP)

static int bluesleep_get_host_wake_io(void);
static int bluesleep_get_ext_wake_io(void);
static int bluesleep_get_host_wake_irq(void);
static void bluesleep_set_io(int io, int io_status);
static int bluesleep_get_io(int io);

static struct bluesleepmethods csr_bluesleep_io_ctrl = {
  .host_wake = bluesleep_get_host_wake_io,
  .ext_wake = bluesleep_get_ext_wake_io,
  .host_wake_irq = bluesleep_get_host_wake_irq,
  .set_io = bluesleep_set_io, 
  .get_io = bluesleep_get_io,   
  .wake = 1,
};

static struct platform_device msm_bluesleep_device = {		
			.name = "bluesleep",		
			.id = -1,	
			.dev = {
               .platform_data = &csr_bluesleep_io_ctrl,
			},	
};
#endif

static struct platform_device *rumi_sim_devices[] __initdata = {
	&smc91x_device,
	&msm_device_uart_dm12,
#ifdef CONFIG_I2C_QUP
	&msm_gsbi3_qup_i2c_device,
	&msm_gsbi4_qup_i2c_device,
	&msm_gsbi7_qup_i2c_device,
	&msm_gsbi8_qup_i2c_device,
	&msm_gsbi9_qup_i2c_device,
	&msm_gsbi12_qup_i2c_device,
#endif
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	&msm_gsbi1_qup_spi_device,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi1,
	&msm_device_ssbi2,
	&msm_device_ssbi3,
#endif
#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
	&android_pmem_kernel_ebi1_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&android_pmem_smipool_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_fb_device,
	&msm_device_kgsl,
#ifdef CONFIG_FB_MSM_LCDC_SAMSUNG_WSVGA
	&lcdc_samsung_panel_device,
#endif
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	&hdmi_msm_device,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_IMX074
	&msm_camera_sensor_imx074,
#endif
#ifdef CONFIG_WEBCAM_OV7692
	&msm_camera_sensor_webcam,
#endif
#endif
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
	&msm_device_vidc,
};

#ifdef CONFIG_SENSORS_M_ADC
static struct resource resources_adc[] = {
	{
		.start = PM8058_ADC_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_ADC_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
};

static struct adc_access_fn xoadc_fn = {
	pm8058_xoadc_select_chan_and_start_conv,
	pm8058_xoadc_read_adc_code,
	pm8058_xoadc_get_properties,
	pm8058_xoadc_slot_request,
	pm8058_xoadc_restore_slot,
	pm8058_xoadc_calibrate,
};

static struct msm_adc_channels msm_adc_channels_data[] = {
	{"vbatt", CHANNEL_ADC_VBATT, 0, &xoadc_fn, CHAN_PATH_TYPE2,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE3, scale_default},
	{"vcoin", CHANNEL_ADC_VCOIN, 0, &xoadc_fn, CHAN_PATH_TYPE1,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE2, scale_default},
	{"vcharger_channel", CHANNEL_ADC_VCHG, 0, &xoadc_fn, CHAN_PATH_TYPE3,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE4, scale_default},
	{"charger_current_monitor", CHANNEL_ADC_CHG_MONITOR, 0, &xoadc_fn,
		CHAN_PATH_TYPE4,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1, scale_default},
	{"vph_pwr", CHANNEL_ADC_VPH_PWR, 0, &xoadc_fn, CHAN_PATH_TYPE5,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE3, scale_default},
	{"usb_vbus", CHANNEL_ADC_USB_VBUS, 0, &xoadc_fn, CHAN_PATH_TYPE11,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE3, scale_default},
	{"pmic_therm", CHANNEL_ADC_DIE_TEMP, 0, &xoadc_fn, CHAN_PATH_TYPE12,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1, scale_pmic_therm},
	{"pmic_therm_4K", CHANNEL_ADC_DIE_TEMP_4K, 0, &xoadc_fn,
		CHAN_PATH_TYPE12,
		ADC_CONFIG_TYPE1, ADC_CALIB_CONFIG_TYPE7, scale_pmic_therm},
	{"xo_therm", CHANNEL_ADC_XOTHERM, 0, &xoadc_fn, CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE5, tdkntcgtherm},
	{"xo_therm_4K", CHANNEL_ADC_XOTHERM_4K, 0, &xoadc_fn,
		CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE1, ADC_CALIB_CONFIG_TYPE6, tdkntcgtherm},
	{"hdset_detect", CHANNEL_ADC_HDSET, 0, &xoadc_fn, CHAN_PATH_TYPE6,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1, scale_default},
	{"chg_batt_amon", CHANNEL_ADC_BATT_AMON, 0, &xoadc_fn, CHAN_PATH_TYPE10,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1,
		scale_xtern_chgr_cur},
	{"msm_therm", CHANNEL_ADC_MSM_THERM, 0, &xoadc_fn, CHAN_PATH_TYPE8,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE2, scale_msm_therm},
	{"batt_therm", CHANNEL_ADC_BATT_THERM, 0, &xoadc_fn, CHAN_PATH_TYPE7,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE2, scale_batt_therm},
	{"batt_id", CHANNEL_ADC_BATT_ID, 0, &xoadc_fn, CHAN_PATH_TYPE9,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE2, scale_default},
};

static struct msm_adc_platform_data msm_adc_pdata = {
	.channel = msm_adc_channels_data,
	.num_chan_supported = ARRAY_SIZE(msm_adc_channels_data),
};

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};

static struct regulator *xoadc_1;
static struct regulator *xoadc_2;

static unsigned int pmic8058_xoadc_setup_power(void)
{
	int rc;

	xoadc_1 = regulator_get(NULL, "8058_s4");
	if (IS_ERR(xoadc_1)) {
		pr_err("%s: Unable to get 8058_s4\n", __func__);
		rc = PTR_ERR(xoadc_1);
		return rc;
	}

	rc = regulator_set_voltage(xoadc_1, 2200000, 2200000);
	if (rc) {
		pr_err("%s: vreg_set_level failed\n", __func__);
		goto fail_vreg_xoadc;
	}

	rc = regulator_enable(xoadc_1);
	if (rc) {
		pr_err("%s: vreg_enable failed\n", __func__);
		goto fail_vreg_xoadc;
	}

	xoadc_2 = regulator_get(NULL, "8058_l18");
	if (IS_ERR(xoadc_2)) {
		pr_err("%s: Unable to get 8058_l18\n", __func__);
		rc = PTR_ERR(xoadc_2);
		goto fail_vreg_xoadc;
	}

	rc = regulator_set_voltage(xoadc_2, 2200000, 2200000);
	if (rc) {
		pr_err("%s: vreg_set_level failed\n", __func__);
		goto fail_vreg_xoadc2;
	}

	rc = regulator_enable(xoadc_2);
	if (rc) {
		pr_err("%s: vreg_enable failed\n", __func__);
		goto fail_vreg_xoadc2;
	}

	return rc;

fail_vreg_xoadc2:
	regulator_put(xoadc_2);
fail_vreg_xoadc:
	regulator_put(xoadc_1);
	return rc;
}

static void pmic8058_xoadc_shutdown_power(void)
{
	int rc;

	rc = regulator_disable(xoadc_1);
	if (rc)
		pr_err("%s: Disable regulator 8058_s4 failed\n", __func__);

	regulator_put(xoadc_1);

	rc = regulator_disable(xoadc_2);
	if (rc)
		pr_err("%s: Disable regulator 8058_l18 failed\n", __func__);

	regulator_put(xoadc_2);
}

static void pmic8058_xoadc_mpp_config(void)
{
	int rc;

	rc = pm8901_mpp_config_digital_out(XOADC_MPP_4,
			PM8901_MPP_DIG_LEVEL_S4, PM_MPP_DOUT_CTL_LOW);
	if (rc)
		pr_err("%s: Config mpp4 on pmic 8901 failed\n", __func__);

	rc = pm8058_mpp_config_analog_input(XOADC_MPP_3,
			PM_MPP_AIN_AMUX_CH5, PM_MPP_AOUT_CTL_DISABLE);
	if (rc)
		pr_err("%s: Config mpp3 on pmic 8058 failed\n", __func__);

	rc = pm8058_mpp_config_analog_input(XOADC_MPP_5,
			PM_MPP_AIN_AMUX_CH9, PM_MPP_AOUT_CTL_DISABLE);
	if (rc)
		pr_err("%s: Config mpp5 on pmic 8058 failed\n", __func__);

	rc = pm8058_mpp_config_analog_input(XOADC_MPP_7,
			PM_MPP_AIN_AMUX_CH6, PM_MPP_AOUT_CTL_DISABLE);
	if (rc)
		pr_err("%s: Config mpp7 on pmic 8058 failed\n", __func__);

	rc = pm8058_mpp_config_analog_input(XOADC_MPP_8,
			PM_MPP_AIN_AMUX_CH8, PM_MPP_AOUT_CTL_DISABLE);
	if (rc)
		pr_err("%s: Config mpp8 on pmic 8058 failed\n", __func__);

	rc = pm8058_mpp_config_analog_input(XOADC_MPP_10,
			PM_MPP_AIN_AMUX_CH7, PM_MPP_AOUT_CTL_DISABLE);
	if (rc)
		pr_err("%s: Config mpp10 on pmic 8058 failed\n", __func__);
}

/* usec. For this ADC,
 * this time represents clk rate @ txco w/ 1024 decimation ratio.
 * Each channel has different configuration, thus at the time of starting
 * the conversion, xoadc will return actual conversion time
 * */
static struct adc_properties pm8058_xoadc_data = {
	.adc_reference          = 2200, /* milli-voltage for this adc */
	.bitresolution         = 15,
	.bipolar                = 0,
	.conversiontime         = 54,
};

static struct xoadc_platform_data xoadc_pdata = {
	.xoadc_prop = &pm8058_xoadc_data,
	.xoadc_setup = pmic8058_xoadc_setup_power,
	.xoadc_shutdown = pmic8058_xoadc_shutdown_power,
	.xoadc_mpp_config = pmic8058_xoadc_mpp_config,
	.xoadc_num = XOADC_PMIC_0,
};
#endif


#ifdef CONFIG_MSM_SDIO_AL

static unsigned mdm2ap_status = 77;

static int configure_mdm2ap_status(int on)
{
	int ret = 0;
	if (on)
		ret = msm_gpiomux_get(mdm2ap_status);
	else
		ret = msm_gpiomux_put(mdm2ap_status);

	if (ret)
		pr_err("%s: mdm2ap_status config failed, on = %d\n", __func__,
		       on);

	return ret;
}


static int get_mdm2ap_status(void)
{
	return gpio_get_value(mdm2ap_status);
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
};

struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};

#endif /* CONFIG_MSM_SDIO_AL */

/* HP SamLin 20110118, start for boradcom gps module 4751 driver */
#if defined (CONFIG_BRCM4751)
/*
 * Configure gps brcm 4751 GPIOs
 * regpu gpio ~ pm8058 gpio4
 * reset gpio ~ pm8058 gpio5
 */
#define PM8058_GPIO4   3
#define PM8058_GPIO5   4
#define BRCM4751_REGPU_GPIO   PM8058_GPIO4
#define BRCM4751_RESET_GPIO   PM8058_GPIO5

static int brcm4751_config(bool on, struct brcm4751_cfg *cfg)
{
	int ret = 0;
	int regpu_sys_gpio;
	int reset_sys_gpio;

	int regpu_gpio = BRCM4751_REGPU_GPIO;
	int reset_gpio = BRCM4751_RESET_GPIO;

	struct pm8058_gpio regpu_gpio_cfg = {
		.direction     = PM_GPIO_DIR_OUT,
		.output_value  = 0,
		.output_buffer = PM_GPIO_OUT_BUF_CMOS,
		.pull 	       = PM_GPIO_PULL_NO,
		.out_strength  = PM_GPIO_STRENGTH_HIGH,
		.function      = PM_GPIO_FUNC_NORMAL,
		.vin_sel       = PM_GPIO_VIN_S3,        /* 1.8v */
		.inv_int_pol   = 0,
	 };

	struct pm8058_gpio reset_gpio_cfg = {
		.direction     = PM_GPIO_DIR_OUT,
		.output_value  = 1,
		.output_buffer = PM_GPIO_OUT_BUF_CMOS,
		.pull          = PM_GPIO_PULL_NO,
		.out_strength  = PM_GPIO_STRENGTH_HIGH,
		.function      = PM_GPIO_FUNC_NORMAL,
		.vin_sel       = PM_GPIO_VIN_S3,        /* 1.8v */
		.inv_int_pol   = 0,
	 };

	/* Configure BRCM4751 REGPU and RESET gpios */
	regpu_sys_gpio = PM8058_GPIO_PM_TO_SYS(regpu_gpio);
	reset_sys_gpio = PM8058_GPIO_PM_TO_SYS(reset_gpio);
	
	if (on) {
		ret = pm8058_gpio_config(regpu_gpio, &regpu_gpio_cfg);
		if (ret < 0) {
			printk(KERN_ERR "%s: pmic8058 gpio %d config failed\n",
				__func__, regpu_gpio);
			return ret;
		}
		ret = gpio_request(regpu_sys_gpio, "brcm4751-regpu");
		if (ret < 0) {
			printk(KERN_ERR "%s: brcm4751 regpu pin request failed\n",
				__func__);
			return ret;
		}
		gpio_direction_output(regpu_sys_gpio, regpu_gpio_cfg.output_value);
	
		ret = pm8058_gpio_config(reset_gpio, &reset_gpio_cfg);
		if (ret < 0) {
			printk(KERN_ERR "%s: pmic8058 gpio %d config failed\n",
				__func__, reset_gpio);
			return ret;
		}
		ret = gpio_request(reset_sys_gpio, "brcm4751-reset");
		gpio_direction_output(reset_sys_gpio, reset_gpio_cfg.output_value);
		if (ret < 0) {
			printk(KERN_ERR "%s: brcm4751 reset pin request failed\n",
				__func__);
			return ret;
		}
	} else {
		gpio_free(regpu_sys_gpio);
		gpio_free(reset_sys_gpio);
	}
	
	if (cfg) {
		/* Return REGPU and RESET sysfs gpios */
		cfg->regpu_sys_gpio = regpu_sys_gpio;
		cfg->reset_sys_gpio = reset_sys_gpio;
	}
	
	return ret;
}

static struct brcm4751_platform_data brcm4751_pdata = {
	.config = brcm4751_config,
};

struct platform_device msm_device_brcm4751 = {
	.name = "brcm4751",
	.id = -1,
	.dev = {
		.platform_data = &brcm4751_pdata,
	},
};
#endif
/* End */

static struct platform_device *surf_devices[] __initdata = {
	&msm_device_smd,
	&smsc911x_device,
	&msm_device_uart_dm12,
#ifdef CONFIG_I2C_QUP
	&msm_gsbi3_qup_i2c_device,
	&msm_gsbi4_qup_i2c_device,
	&msm_gsbi7_qup_i2c_device,
	&msm_gsbi8_qup_i2c_device,
	&msm_gsbi9_qup_i2c_device,
	&msm_gsbi12_qup_i2c_device,
#endif
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	&msm_gsbi1_qup_spi_device,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi1,
	&msm_device_ssbi2,
	&msm_device_ssbi3,
#endif
#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)
	&isp1763_device,
#endif
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_HCD)
	&msm_device_otg,
#endif
#ifdef CONFIG_USB_GADGET_MSM_72K
	&msm_device_gadget_peripheral,
#endif
#ifdef CONFIG_USB_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
#ifdef CONFIG_USB_ANDROID_DIAG
	&usb_diag_device,
#endif
	&android_usb_device,
#endif
#ifdef CONFIG_BATTERY_MSM
	&msm_batt_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
	&android_pmem_kernel_ebi1_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&android_pmem_smipool_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_fb_device,
	&msm_device_kgsl,
#ifdef CONFIG_FB_MSM_LCDC_SAMSUNG_WSVGA
	&lcdc_samsung_panel_device,
#endif
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	&hdmi_msm_device,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#ifdef CONFIG_FB_MSM_MIPI_DSI
	&mipi_dsi_toshiba_panel_device,
#endif
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_IMX074
	&msm_camera_sensor_imx074,
#endif
#ifdef CONFIG_WEBCAM_OV7692
	&msm_camera_sensor_webcam,
#endif
#endif
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif

#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)
	&msm_rpm_log_device,
#endif
#ifdef CONFIG_BATTERY_MSM8X60
	&msm_charger_device,
#endif
	&msm_device_vidc,
#if (defined(CONFIG_BAHAMA_CORE)) && \
	(defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
	&msm_bt_power_device,
#endif
#ifdef CONFIG_SENSORS_M_ADC
	&msm_adc_device,
#endif
#ifdef CONFIG_PMIC8058
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L7],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L8],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L9],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L10],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L11],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L12],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L13],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L14],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L15],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L16],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L17],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L18],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L19],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L20],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L21],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L22],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L23],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L24],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L25],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S4],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_NCP],
#endif
#ifdef CONFIG_PMIC8901
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L0],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L1],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS0],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_MVS0],
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif

#ifdef CONFIG_MSM_SDIO_AL
	&msm_device_sdio_al,
#endif
};

static struct platform_device *topaz_devices[] __initdata = {
	&msm_device_smd,
	&msm_device_uart_dm12,
#ifdef CONFIG_I2C_QUP
	&msm_gsbi3_qup_i2c_device,
	&msm_gsbi4_qup_i2c_device,
	&msm_gsbi7_qup_i2c_device,
	&msm_gsbi8_qup_i2c_device,
	// Yegw 2010-8-20 config gsbi10 as i2c function
	&msm_gsbi10_qup_i2c_device,
	// Yegw End
	&msm_gsbi9_qup_i2c_device,
#endif
//HP_Effie, QUP5_SPI for lcd inteface, Start
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	&msm_gsbi5_qup_spi_device,
#endif
//HP_Effie, QUP5_SPI for lcd interface, End
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
    &msm_device_uart_dm10,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi1,
	&msm_device_ssbi2,
	&msm_device_ssbi3,
#endif
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_HCD)
	&msm_device_otg,
#endif
#ifdef CONFIG_USB_GADGET_MSM_72K
	&msm_device_gadget_peripheral,
#endif
#ifdef CONFIG_USB_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
#ifdef CONFIG_USB_ANDROID_DIAG
	&usb_diag_device,
#endif
	&android_usb_device,
#endif
#ifdef CONFIG_BATTERY_MSM
	&msm_batt_device,
#endif
#ifdef CONFIG_CHARGER_MAX8903
	&max8903_charger_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
	&android_pmem_kernel_ebi1_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&android_pmem_smipool_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_fb_device,
	&msm_device_kgsl,
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	&hdmi_msm_device,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_IMX074
	&msm_camera_sensor_imx074,
#endif
#ifdef CONFIG_WEBCAM_OV7692
	&msm_camera_sensor_webcam,
#endif
//Bob
#ifdef CONFIG_WEBCAM_MT9M113
	&msm_camera_sensor_webcam_mt9m113,
#endif
#ifdef CONFIG_VX6953
	&msm_camera_sensor_vx6953,
#endif

#endif
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif

#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)
	&msm_rpm_log_device,
#endif
	&msm_device_vidc,
#if (defined(CONFIG_BAHAMA_CORE)) && \
	(defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
	&msm_bt_power_device,
#endif
#ifdef CONFIG_SENSORS_M_ADC
	&msm_adc_device,
#endif
//wanqin
#ifdef CONFIG_BT
	&msm_bt_csr_power_device,
#ifdef	CONFIG_BT_MSM_SLEEP
	&msm_bluesleep_device,
#endif	
#endif

#if defined(CONFIG_AR6003) || defined(CONFIG_AR6003_MODULE)
	&ar6003_wifi_device,
#endif
	//Zhs<20100914>[VDD5V]Already keep always on in SBL, it doesn't need control anymore.
	#ifdef USE_REGULATOR_VDD5V
	&topaz_fixed_reg_device[0],
	#endif
	&msm_gpio_keys,
#ifdef CONFIG_HP_HEADSET
	&hs_device,
#endif
#ifdef CONFIG_HRES_COUNTER
	&hres_counter_device,
#endif
#ifdef CONFIG_PMIC8058
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L7],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L8],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L9],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L10],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L11],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L12],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L13],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L14],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L15],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L16],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L17],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L18],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L19],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L20],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L21],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L22],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L23],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L24],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L25],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S4],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS0],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_NCP],
#endif
#ifdef CONFIG_PMIC8901
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L0],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L1],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L2],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L3],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS0],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS2],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS3],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8901_MVS0],
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif
};

static struct platform_device *opal_devices[] __initdata = {
	&msm_device_smd,
	&msm_device_uart_dm12,
#ifdef CONFIG_I2C_QUP
	&msm_gsbi3_qup_i2c_device,
	&msm_gsbi4_qup_i2c_device,
	&msm_gsbi7_qup_i2c_device,
	&msm_gsbi8_qup_i2c_device,
	// Yegw 2010-8-20 config gsbi10 as i2c function
	&msm_gsbi10_qup_i2c_device,
	// Yegw End
	&msm_gsbi9_qup_i2c_device,

#endif
//HP_Effie, QUP5_SPI for lcd inteface, Start
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	&msm_gsbi5_qup_spi_device,
#endif
//HP_Effie, QUP5_SPI for lcd interface, End
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi1,
	&msm_device_ssbi2,
	&msm_device_ssbi3,
#endif
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_HCD)
	&msm_device_otg,
#endif
#ifdef CONFIG_USB_GADGET_MSM_72K
	&msm_device_gadget_peripheral,
#endif
#ifdef CONFIG_USB_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
#ifdef CONFIG_USB_ANDROID_DIAG
	&usb_diag_device,
#endif
	&android_usb_device,
#endif
#ifdef CONFIG_BATTERY_MSM
	&msm_batt_device,
#endif
#ifdef CONFIG_CHARGER_MAX8903
	&max8903_charger_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
	&android_pmem_kernel_ebi1_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&android_pmem_smipool_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_fb_device,
	&msm_device_kgsl,
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	&hdmi_msm_device,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_IMX074
	&msm_camera_sensor_imx074,
#endif
#ifdef CONFIG_WEBCAM_OV7692
	&msm_camera_sensor_webcam,
#endif
//Bob
#ifdef CONFIG_WEBCAM_MT9M113
	&msm_camera_sensor_webcam_mt9m113,
#endif
#ifdef CONFIG_VX6953
	&msm_camera_sensor_vx6953,
#endif

#endif
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif

#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)
	&msm_rpm_log_device,
#endif
	&msm_device_vidc,
#if (defined(CONFIG_BAHAMA_CORE)) && \
	(defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
	&msm_bt_power_device,
#endif
#ifdef CONFIG_SENSORS_M_ADC
	&msm_adc_device,
#endif
//wanqin
#ifdef CONFIG_BT
	&msm_bt_csr_power_device,
#ifdef	CONFIG_BT_MSM_SLEEP
	&msm_bluesleep_device,
#endif	
#endif

#if defined(CONFIG_AR6003) || defined(CONFIG_AR6003_MODULE)
	&ar6003_wifi_device,
#endif
	//Zhs<20100914>[VDD5V]Already keep always on in SBL, it doesn't need control anymore.
	#ifdef USE_REGULATOR_VDD5V
	&opal_fixed_reg_device[0],
	#endif
	&msm_gpio_keys,
#ifdef CONFIG_HP_HEADSET
	&hs_device,
#endif
#ifdef CONFIG_HRES_COUNTER
	&hres_counter_device,
#endif
#ifdef CONFIG_PMIC8058
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L7],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L8],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L9],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L10],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L11],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L12],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L13],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L14],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L15],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L16],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L17],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L18],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L19],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L20],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L21],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L22],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L23],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L24],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L25],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S4],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS0],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_NCP],
#endif
#ifdef CONFIG_PMIC8901
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L0],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L1],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L2],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L3],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS0],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS2],
	// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifndef WORKAROUND_RPM_REGULATOR
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS3],
	#endif
	// end
	&rpm_vreg_device[RPM_VREG_ID_PM8901_MVS0],
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif

};

#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)
enum {
	SX150X_CORE,
	SX150X_DOCKING,
	SX150X_SURF,
	SX150X_LEFT_FHA,
	SX150X_RIGHT_FHA,
	SX150X_SOUTH,
	SX150X_NORTH,
	SX150X_CORE_FLUID,
};

static struct sx150x_platform_data sx150x_data[] __initdata = {
	[SX150X_CORE] = {
		.gpio_base         = GPIO_CORE_EXPANDER_BASE,
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0x0408,
		.io_pulldn_ena     = 0x4060,
		.io_open_drain_ena = 0x000c,
		.io_polarity       = 0,
		.irq_summary       = -1, /* see fixup_i2c_configs() */
		.irq_base          = GPIO_EXPANDER_IRQ_BASE,
	},
	[SX150X_DOCKING] = {
		.gpio_base         = GPIO_DOCKING_EXPANDER_BASE,
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0x5e06,
		.io_pulldn_ena     = 0x81b8,
		.io_open_drain_ena = 0,
		.io_polarity       = 0,
		.irq_summary       = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
							 UI_INT2_N),
		.irq_base          = GPIO_EXPANDER_IRQ_BASE +
					 GPIO_DOCKING_EXPANDER_BASE -
					 GPIO_EXPANDER_GPIO_BASE,
	},
	[SX150X_SURF] = {
		.gpio_base         = GPIO_SURF_EXPANDER_BASE,
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0,
		.io_pulldn_ena     = 0,
		.io_open_drain_ena = 0,
		.io_polarity       = 0,
		.irq_summary       = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
							 UI_INT1_N),
		.irq_base          = GPIO_EXPANDER_IRQ_BASE +
					 GPIO_SURF_EXPANDER_BASE -
					 GPIO_EXPANDER_GPIO_BASE,
	},
	[SX150X_LEFT_FHA] = {
		.gpio_base         = GPIO_LEFT_KB_EXPANDER_BASE,
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0,
		.io_pulldn_ena     = 0x40,
		.io_open_drain_ena = 0,
		.io_polarity       = 0,
		.irq_summary       = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
							 UI_INT3_N),
		.irq_base          = GPIO_EXPANDER_IRQ_BASE +
					 GPIO_LEFT_KB_EXPANDER_BASE -
					 GPIO_EXPANDER_GPIO_BASE,
	},
	[SX150X_RIGHT_FHA] = {
		.gpio_base         = GPIO_RIGHT_KB_EXPANDER_BASE,
		.oscio_is_gpo      = true,
		.io_pullup_ena     = 0,
		.io_pulldn_ena     = 0,
		.io_open_drain_ena = 0,
		.io_polarity       = 0,
		.irq_summary       = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
							 UI_INT3_N),
		.irq_base          = GPIO_EXPANDER_IRQ_BASE +
					 GPIO_RIGHT_KB_EXPANDER_BASE -
					 GPIO_EXPANDER_GPIO_BASE,
	},
	[SX150X_SOUTH] = {
		.gpio_base    = GPIO_SOUTH_EXPANDER_BASE,
		.irq_base     = GPIO_EXPANDER_IRQ_BASE +
				GPIO_SOUTH_EXPANDER_BASE -
				GPIO_EXPANDER_GPIO_BASE,
		.irq_summary  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, UI_INT3_N),
	},
	[SX150X_NORTH] = {
		.gpio_base    = GPIO_NORTH_EXPANDER_BASE,
		.irq_base     = GPIO_EXPANDER_IRQ_BASE +
				GPIO_NORTH_EXPANDER_BASE -
				GPIO_EXPANDER_GPIO_BASE,
		.irq_summary  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, UI_INT3_N),
		.oscio_is_gpo = true,
		.io_open_drain_ena = 0x30,
	},
	[SX150X_CORE_FLUID] = {
		.gpio_base         = GPIO_CORE_EXPANDER_BASE,
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0x0408,
		.io_pulldn_ena     = 0x4060,
		.io_open_drain_ena = 0x0008,
		.io_polarity       = 0,
		.irq_summary       = -1, /* see fixup_i2c_configs() */
		.irq_base          = GPIO_EXPANDER_IRQ_BASE,
	},
};

/* sx150x_low_power_cfg
 *
 * This data and init function are used to put unused gpio-expander output
 * lines into their low-power states at boot. The init
 * function must be deferred until a later init stage because the i2c
 * gpio expander drivers do not probe until after they are registered
 * (see register_i2c_devices) and the work-queues for those registrations
 * are processed.  Because these lines are unused, there is no risk of
 * competing with a device driver for the gpio.
 *
 * gpio lines whose low-power states are input are naturally in their low-
 * power configurations once probed, see the platform data structures above.
 */
struct sx150x_low_power_cfg {
	unsigned gpio;
	unsigned val;
};

static struct sx150x_low_power_cfg
common_sx150x_lp_cfgs[] __initdata = {
	{GPIO_WLAN_DEEP_SLEEP_N, 0},
	{GPIO_EXT_GPS_LNA_EN,    0},
	{GPIO_MSM_WAKES_BT,      0},
	{GPIO_USB_UICC_EN,       0},
	{GPIO_BATT_GAUGE_EN,     0},
};

static struct sx150x_low_power_cfg
surf_ffa_sx150x_lp_cfgs[] __initdata = {
	{GPIO_MIPI_DSI_RST_N,      0},
	{GPIO_DONGLE_PWR_EN,       0},
	{GPIO_CAP_TS_SLEEP,        1},
	{GPIO_COMPASS_RST_N,       0},
	{GPIO_WEB_CAMIF_RESET_N,   0},
	{GPIO_R_ALTIMETER_RESET_N, 0},
};

static void __init
cfg_gpio_low_power(struct sx150x_low_power_cfg *cfgs, unsigned nelems)
{
	unsigned n;
	int rc;

	for (n = 0; n < nelems; ++n) {
		rc = gpio_request(cfgs[n].gpio, NULL);
		if (!rc) {
			rc = gpio_direction_output(cfgs[n].gpio, cfgs[n].val);
			gpio_free(cfgs[n].gpio);
		}

		if (rc) {
			printk(KERN_NOTICE "%s: failed to sleep gpio %d: %d\n",
				   __func__, cfgs[n].gpio, rc);
		}
	}
}

static int __init cfg_sx150xs_low_power(void)
{
	cfg_gpio_low_power(common_sx150x_lp_cfgs,
		ARRAY_SIZE(common_sx150x_lp_cfgs));
	if (!machine_is_msm8x60_fluid())
		cfg_gpio_low_power(surf_ffa_sx150x_lp_cfgs,
			ARRAY_SIZE(surf_ffa_sx150x_lp_cfgs));
	return 0;
}
module_init(cfg_sx150xs_low_power);

#ifdef CONFIG_I2C
static struct i2c_board_info core_expander_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
		.platform_data = &sx150x_data[SX150X_CORE]
	},
};

static struct i2c_board_info docking_expander_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3f),
		.platform_data = &sx150x_data[SX150X_DOCKING]
	},
};

static struct i2c_board_info surf_expanders_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x70),
		.platform_data = &sx150x_data[SX150X_SURF]
	}
};

static struct i2c_board_info fha_expanders_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1508q", 0x21),
		.platform_data = &sx150x_data[SX150X_LEFT_FHA]
	},
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data = &sx150x_data[SX150X_RIGHT_FHA]
	}
};

static struct i2c_board_info fluid_expanders_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1508q", 0x23),
		.platform_data = &sx150x_data[SX150X_SOUTH]
	},
	{
		I2C_BOARD_INFO("sx1508q", 0x20),
		.platform_data = &sx150x_data[SX150X_NORTH]
	}
};

static struct i2c_board_info fluid_core_expander_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
		.platform_data = &sx150x_data[SX150X_CORE_FLUID]
	},
};
#endif
#endif

#ifdef CONFIG_ISL9519_CHARGER
#define ISL_VALID_MPP 10
#define ISL_VALID_MPP_2 11
static int isl_detection_setup(void)
{
	int ret = 0;

	ret = pm8058_mpp_config_digital_in(ISL_VALID_MPP,
					   PM8058_MPP_DIG_LEVEL_S3,
					   PM_MPP_DIN_TO_INT);
	ret |=  pm8058_mpp_config_bi_dir(ISL_VALID_MPP_2,
					   PM8058_MPP_DIG_LEVEL_S3,
					   PM_MPP_BI_PULLUP_10KOHM
					   );
	return ret;
}

static struct isl_platform_data isl_data __initdata = {
	.chgcurrent = 700,
	.valid_n_gpio = PM8058_MPP_PM_TO_SYS(10),
	.chg_detection_config = isl_detection_setup,
	.max_system_voltage = 4200,
	.min_system_voltage = 3200,
	.term_current = 120,
};

static struct i2c_board_info isl_charger_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("isl9519q", 0x9),
		.irq = PM8058_CBLPWR_IRQ(PM8058_IRQ_BASE),
		.platform_data = &isl_data,
	},
};
#endif

#ifdef CONFIG_PMIC8058
#define PMIC_GPIO_SDC3_DET 22

static int pm8058_gpios_init(void)
{
	int i;
	int rc;
	struct pm8058_gpio_cfg {
		int                gpio;
		struct pm8058_gpio cfg;
	};

	struct pm8058_gpio_cfg gpio_cfgs[] = {
		{ /* FFA ethernet */
			6,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_DN,
				.vin_sel        = 2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
		{
			PMIC_GPIO_SDC3_DET - 1,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_30,
				.vin_sel        = 2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
#endif
		{ /* core&surf gpio expander */
			UI_INT1_N,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{ /* docking gpio expander */
			UI_INT2_N,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{ /* FHA/keypad gpio expanders */
			UI_INT3_N,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{ /* TouchDisc Interrupt */
			5,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_1P5,
				.vin_sel        = 2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			}
		},
		{ /* Timpani Reset */
			20,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_DN,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= 2,
				.inv_int_pol	= 0,
			}
		},
		{ /* PMIC ID interrupt */
			36,
			{
				.direction	= PM_GPIO_DIR_IN,
				.pull		= PM_GPIO_PULL_UP_1P5,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= 2,
				.inv_int_pol	= 0,
			}
		},
	};

#if defined(CONFIG_HAPTIC_ISA1200) || \
		defined(CONFIG_HAPTIC_ISA1200_MODULE)

	struct pm8058_gpio_cfg en_hap_gpio_cfg = {
			PMIC_GPIO_HAP_ENABLE,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.out_strength   = PM_GPIO_STRENGTH_HIGH,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
				.vin_sel        = 2,
				.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
				.output_value   = 0,
			}

	};

	if (machine_is_msm8x60_fluid()) {
		rc = pm8058_gpio_config(en_hap_gpio_cfg.gpio,
				&en_hap_gpio_cfg.cfg);
		if (rc < 0) {
			pr_err("%s pmic haptics gpio config failed\n",
							__func__);
			return rc;
		}
	}
#endif

#if defined(CONFIG_PMIC8058_OTHC) || defined(CONFIG_PMIC8058_OTHC_MODULE)
	struct pm8058_gpio_cfg line_in_gpio_cfg = {
			18,
			{
				.direction	= PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_1P5,
				.vin_sel        = 2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			}
	};
	/* Line_in only for 8660 ffa & surf */
	if (machine_is_msm8x60_ffa() || machine_is_msm8x60_surf()) {
		rc = pm8058_gpio_config(line_in_gpio_cfg.gpio,
				&line_in_gpio_cfg.cfg);
		if (rc < 0) {
			pr_err("%s pmic line_in gpio config failed\n",
							__func__);
			return rc;
		}
	}
#endif

	for (i = 0; i < ARRAY_SIZE(gpio_cfgs); ++i) {
		rc = pm8058_gpio_config(gpio_cfgs[i].gpio,
				&gpio_cfgs[i].cfg);
		if (rc < 0) {
			pr_err("%s pmic gpio config failed\n",
				__func__);
			return rc;
		}
	}

	return 0;
}

static int pm8058_gpios_init_topaz(void)
{
	int i;
	int rc;
	struct pm8058_gpio_cfg {
		int                gpio;
		struct pm8058_gpio cfg;
	};
//Levi101102 add for pmic gpio config.
	struct pm8058_gpio_cfg topaz_wifi_gpio_cfgs[] = {
		{
			0,
			{
				.direction	= PM_GPIO_DIR_IN,
				.pull		= PM_GPIO_PULL_UP_1P5,
				.vin_sel	= PM_GPIO_VIN_S3,
				.function	= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol	= 0,
			},
		},
		{
			1,
			{
				.direction	= PM_GPIO_DIR_IN,
				.pull		= PM_GPIO_PULL_UP_1P5,
				.vin_sel	= PM_GPIO_VIN_S3,
				.function	= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol	= 0,
			},
		},
		{
			2,
			{
				.direction	= PM_GPIO_DIR_IN,
				.pull		= PM_GPIO_PULL_UP_1P5,
				.vin_sel	= PM_GPIO_VIN_S3,
				.function	= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol	= 0,
			},
		},
	};

	struct pm8058_gpio_cfg topaz_3g_gpio_cfgs[] = {
		{
			5,
			{
				.direction	= PM_GPIO_DIR_IN,
				.pull		= PM_GPIO_PULL_UP_1P5,
				.vin_sel	= PM_GPIO_VIN_S3,
				.function	= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol	= 0,
			},
		},
		{
			6,
			{
				.direction	= PM_GPIO_DIR_IN,
				.pull		= PM_GPIO_PULL_UP_1P5,
				.vin_sel	= PM_GPIO_VIN_S3,
				.function	= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol	= 0,
			},
		},
	};
	struct pm8058_gpio_cfg opal_3g_gpio_cfgs[] = {
		{
			5,
			{
				.direction	= PM_GPIO_DIR_IN,
				.pull		= PM_GPIO_PULL_UP_1P5,
				.vin_sel	= PM_GPIO_VIN_S3,
				.function	= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol	= 0,
			},
		},
		{
			6,
			{
				.direction	= PM_GPIO_DIR_IN,
				.pull		= PM_GPIO_PULL_UP_1P5,
				.vin_sel	= PM_GPIO_VIN_S3,
				.function	= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol	= 0,

			},
		},
//wanqin nfc
#ifdef CONFIG_NFC
		{
			14,
			{
				.direction = PM_GPIO_DIR_IN,
				.pull =  PM_GPIO_PULL_NO,
				.vin_sel = PM_GPIO_VIN_S3,
				.function = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol = 0,		
			},
		},
		{
			15,
			{
				.direction = PM_GPIO_DIR_OUT,
				.output_value = 0,
				.pull = PM_GPIO_PULL_NO,
				.vin_sel = PM_GPIO_VIN_S3,
				.function = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol = 0,
			},
		},
#endif
	};
//Levi101102 add for pmic keys. end
	struct pm8058_gpio_cfg gpio_cfgs[] = {
		{
			/*LCD BL PWM, PMIC GPIO24 in schematic*/
			23,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_VPH,
				.function       = PM_GPIO_FUNC_2,
				.inv_int_pol    = 0,
			},
		},
		{
			/*LCD BL ENABLE, PMIC GPIO25 in schematic*/
			24,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_VPH,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
				.output_value	= 1,
			},
		},
		{
			/*UIM CLK, PMIC GPIO29 in schematic*/
			28,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_1,
				.inv_int_pol    = 0,
			},
		},
		{
			/*UIM CLK, PMIC GPIO30 in schematic*/
			29,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_1,
				.inv_int_pol    = 0,
			},
		},
		{
			/*UIM Reset, PMIC GPIO32 in schematic*/
			31,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{
			/*Proximity reset, PMIC GPIO36 in schematic*/
			35,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{
			/*USB ID, PMIC GPIO37 in schematic*/
			36,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{
			/*SSBI PMIC CLK, PMIC GPIO39 in schematic*/
			38,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_1,
				.inv_int_pol    = 0,
			},
		},
		{
			/*Proximity interrupt, PMIC GPIO40 in schematic*/
			39,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},

	};

	for (i = 0; i < ARRAY_SIZE(gpio_cfgs); ++i) {
		rc = pm8058_gpio_config(gpio_cfgs[i].gpio,
				&gpio_cfgs[i].cfg);
		if (rc < 0) {
			pr_err("%s pmic gpio config failed\n",
				__func__);
			return rc;
		}
	}
	if(board_is_topaz_3g()){
		for (i = 0; i < ARRAY_SIZE(topaz_3g_gpio_cfgs); ++i) {
			rc = pm8058_gpio_config(topaz_3g_gpio_cfgs[i].gpio,
					&topaz_3g_gpio_cfgs[i].cfg);
			if (rc < 0) {
				pr_err("%s topaz 3G pmic gpio config failed\n",
					__func__);
				return rc;
			}
		}
	}else if(board_is_topaz_wifi()){
		for (i = 0; i < ARRAY_SIZE(topaz_wifi_gpio_cfgs); ++i) {
			rc = pm8058_gpio_config(topaz_wifi_gpio_cfgs[i].gpio,
					&topaz_wifi_gpio_cfgs[i].cfg);
			if (rc < 0) {
				pr_err("%s topaz wifi pmic gpio config failed\n",
					__func__);
				return rc;
			}
		}
	}else if(board_is_opal_3g()|| board_is_opal_wifi()){
		for (i = 0; i < ARRAY_SIZE(opal_3g_gpio_cfgs); ++i) {
			rc = pm8058_gpio_config(opal_3g_gpio_cfgs[i].gpio,
					&opal_3g_gpio_cfgs[i].cfg);
			if (rc < 0) {
				pr_err("%s opal 3G pmic gpio config failed\n",
					__func__);
				return rc;
			}
		}
	}

	return 0;
}

static const unsigned int ffa_keymap[] = {
	KEY(0, 0, KEY_FN_F1),	 /* LS - PUSH1 */
	KEY(0, 1, KEY_UP),	 /* NAV - UP */
	KEY(0, 2, KEY_LEFT),	 /* NAV - LEFT */
	KEY(0, 3, KEY_VOLUMEUP), /* Shuttle SW_UP */

	KEY(1, 0, KEY_FN_F2), 	 /* LS - PUSH2 */
	KEY(1, 1, KEY_RIGHT),    /* NAV - RIGHT */
	KEY(1, 2, KEY_DOWN),     /* NAV - DOWN */
	KEY(1, 3, KEY_VOLUMEDOWN),

	KEY(2, 3, KEY_ENTER),     /* SW_PUSH key */

	KEY(4, 0, KEY_CAMERA_FOCUS), /* RS - PUSH1 */
	KEY(4, 1, KEY_UP),	  /* USER_UP */
	KEY(4, 2, KEY_LEFT),	  /* USER_LEFT */
	KEY(4, 3, KEY_HOME),	  /* Right switch: MIC Bd */
	KEY(4, 4, KEY_FN_F3),	  /* Reserved MIC */

	KEY(5, 0, KEY_CAMERA_SNAPSHOT), /* RS - PUSH2 */
	KEY(5, 1, KEY_RIGHT),	  /* USER_RIGHT */
	KEY(5, 2, KEY_DOWN),	  /* USER_DOWN */
	KEY(5, 3, KEY_BACK),	  /* Left switch: MIC */
	KEY(5, 4, KEY_MENU),	  /* Center switch: MIC */
};

static struct resource resources_keypad[] = {
	{
		.start	= PM8058_KEYPAD_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_KEYPAD_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_KEYSTUCK_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_KEYSTUCK_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct matrix_keymap_data ffa_keymap_data = {
	.keymap_size	= ARRAY_SIZE(ffa_keymap),
	.keymap		= ffa_keymap,
};

static struct pmic8058_keypad_data ffa_keypad_data = {
	.input_name		= "ffa-keypad",
	.input_phys_device	= "ffa-keypad/input0",
	.num_rows		= 6,
	.num_cols		= 5,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.debounce_ms		= {8, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns            = 91500,
	.wakeup			= 1,
	.keymap_data		= &ffa_keymap_data,
};

static const unsigned int fluid_keymap[] = {
	KEY(0, 0, KEY_FN_F1),	 /* LS - PUSH1 */
	KEY(0, 1, KEY_UP),	 /* NAV - UP */
	KEY(0, 2, KEY_LEFT),	 /* NAV - LEFT */
	KEY(0, 3, KEY_VOLUMEDOWN), /* Shuttle SW_UP */

	KEY(1, 0, KEY_FN_F2),	 /* LS - PUSH2 */
	KEY(1, 1, KEY_RIGHT),    /* NAV - RIGHT */
	KEY(1, 2, KEY_DOWN),     /* NAV - DOWN */
	KEY(1, 3, KEY_VOLUMEUP),

	KEY(2, 3, KEY_ENTER),     /* SW_PUSH key */

	KEY(4, 0, KEY_CAMERA_FOCUS), /* RS - PUSH1 */
	KEY(4, 1, KEY_UP),	  /* USER_UP */
	KEY(4, 2, KEY_LEFT),	  /* USER_LEFT */
	KEY(4, 3, KEY_HOME),	  /* Right switch: MIC Bd */
	KEY(4, 4, KEY_FN_F3),	  /* Reserved MIC */

	KEY(5, 0, KEY_CAMERA_SNAPSHOT), /* RS - PUSH2 */
	KEY(5, 1, KEY_RIGHT),	  /* USER_RIGHT */
	KEY(5, 2, KEY_DOWN),	  /* USER_DOWN */
	KEY(5, 3, KEY_BACK),	  /* Left switch: MIC */
	KEY(5, 4, KEY_MENU),	  /* Center switch: MIC */
};

static struct matrix_keymap_data fluid_keymap_data = {
	.keymap_size	= ARRAY_SIZE(fluid_keymap),
	.keymap		= fluid_keymap,
};

static struct pmic8058_keypad_data fluid_keypad_data = {
	.input_name		= "fluid-keypad",
	.input_phys_device	= "fluid-keypad/input0",
	.num_rows		= 6,
	.num_cols		= 5,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.debounce_ms		= {8, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns            = 91500,
	.wakeup			= 1,
	.keymap_data		= &fluid_keymap_data,
};

static struct resource resources_pwrkey[] = {
	{
		.start	= PM8058_PWRKEY_REL_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_PWRKEY_REL_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_PWRKEY_PRESS_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_PWRKEY_PRESS_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct pmic8058_pwrkey_pdata pwrkey_pdata = {
	.pull_up		= 1,
	.kpd_trigger_delay_us   = 970,
	.wakeup			= 1,
	.pwrkey_time_ms		= 500,
};

static struct pmic8058_pwrkey_pdata pwrkey_pdata_topaz = {
	.pull_up		= 1,
	.kpd_trigger_delay_us   = 970,
	.wakeup			= 1,
	//.pwrkey_time_ms		= 500,
};

static struct pmic8058_vibrator_pdata pmic_vib_pdata = {
	.initial_vibrate_ms  = 500,
	.level_mV = 3000,
	.max_timeout_ms = 15000,
};

#if defined(CONFIG_PMIC8058_OTHC) || defined(CONFIG_PMIC8058_OTHC_MODULE)
#define PM8058_OTHC_CNTR_BASE0	0xA0
#define PM8058_OTHC_CNTR_BASE1	0x134
#define PM8058_OTHC_CNTR_BASE2	0x137
#define PM8058_LINE_IN_DET_GPIO	PM8058_GPIO_PM_TO_SYS(18)

static struct othc_accessory_info othc_accessories[]  = {
	{
		.accessory = OTHC_ANC_HEADPHONE,
		.detect_flags = OTHC_MICBIAS_DETECT | OTHC_GPIO_DETECT |
							OTHC_SWITCH_DETECT,
		.gpio = PM8058_LINE_IN_DET_GPIO,
		.active_low = 1,
		.key_code = SW_HEADPHONE_INSERT,
		.enabled = true,
	},
	{
		.accessory = OTHC_ANC_HEADSET,
		.detect_flags = OTHC_MICBIAS_DETECT | OTHC_GPIO_DETECT,
		.gpio = PM8058_LINE_IN_DET_GPIO,
		.active_low = 1,
		.key_code = SW_HEADPHONE_INSERT,
		.enabled = true,
	},
	{
		.accessory = OTHC_HEADPHONE,
		.detect_flags = OTHC_MICBIAS_DETECT | OTHC_SWITCH_DETECT,
		.key_code = SW_HEADPHONE_INSERT,
		.enabled = true,
	},
	{
		.accessory = OTHC_MICROPHONE,
		.detect_flags = OTHC_GPIO_DETECT,
		.gpio = PM8058_LINE_IN_DET_GPIO,
		.active_low = 1,
		.key_code = SW_MICROPHONE_INSERT,
		.enabled = true,
	},
	{
		.accessory = OTHC_HEADSET,
		.detect_flags = OTHC_MICBIAS_DETECT,
		.key_code = SW_HEADPHONE_INSERT,
		.enabled = true,
	},
};

static struct othc_switch_info switch_info[] = {
	{
		.min_adc_threshold = 0,
		.max_adc_threshold = 100,
		.key_code = KEY_PLAYPAUSE,
	},
	{
		.min_adc_threshold = 100,
		.max_adc_threshold = 200,
		.key_code = KEY_REWIND,
	},
	{
		.min_adc_threshold = 200,
		.max_adc_threshold = 500,
		.key_code = KEY_FASTFORWARD,
	},
};

static struct othc_n_switch_config switch_config = {
	.voltage_settling_time_ms = 0,
	.num_adc_samples = 3,
	.adc_channel = CHANNEL_ADC_HDSET,
	.switch_info = switch_info,
	.num_keys = ARRAY_SIZE(switch_info),
};

static struct hsed_bias_config hsed_bias_config = {
	/* HSED mic bias config info */
	.othc_headset = OTHC_HEADSET_NO,
	.othc_lowcurr_thresh_uA = 100,
	.othc_highcurr_thresh_uA = 600,
	.othc_hyst_prediv_us = 7800,
	.othc_period_clkdiv_us = 62500,
	.othc_hyst_clk_us = 121000,
	.othc_period_clk_us = 312500,
	.othc_wakeup = 1,
};

static struct othc_hsed_config hsed_config_1 = {
	.hsed_bias_config = &hsed_bias_config,
	.detection_delay_ms = 200,
	/* Switch info */
	.switch_debounce_ms = 1000,
	.othc_support_n_switch = false,
	.switch_config = &switch_config,
	/* Accessory info */
	.accessories_support = true,
	.accessories = othc_accessories,
	.othc_num_accessories = ARRAY_SIZE(othc_accessories),
};

/* MIC_BIAS0 is configured as normal MIC BIAS */
static struct pmic8058_othc_config_pdata othc_config_pdata_0 = {
	.micbias_select = OTHC_MICBIAS_0,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
};

/* MIC_BIAS1 is configured as HSED_BIAS for OTHC */
static struct pmic8058_othc_config_pdata othc_config_pdata_1 = {
	.micbias_select = OTHC_MICBIAS_1,
	.micbias_capability = OTHC_MICBIAS_HSED,
	.micbias_enable = OTHC_SIGNAL_PWM_TCXO,
	.hsed_config = &hsed_config_1,
	.hsed_name = "8660_handset",
};

/* MIC_BIAS2 is configured as normal MIC BIAS */
static struct pmic8058_othc_config_pdata othc_config_pdata_2 = {
	.micbias_select = OTHC_MICBIAS_2,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
};

static struct resource resources_othc_0[] = {
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE0,
		.end   = PM8058_OTHC_CNTR_BASE0,
		.flags = IORESOURCE_IO,
	},
};

static struct resource resources_othc_1[] = {
	{
		.start = PM8058_SW_1_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_SW_1_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = PM8058_IR_1_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_IR_1_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE1,
		.end   = PM8058_OTHC_CNTR_BASE1,
		.flags = IORESOURCE_IO,
	},
};

static struct resource resources_othc_2[] = {
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE2,
		.end   = PM8058_OTHC_CNTR_BASE2,
		.flags = IORESOURCE_IO,
	},
};

static void __init msm8x60_init_pm8058_othc(void)
{
	int i;

	/* 3-switch headset supported only by V2 FFA and FLUID */
	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2 ||
					machine_is_msm8x60_fluid())
		hsed_config_1.othc_support_n_switch = true;

	for (i = 0; i < ARRAY_SIZE(othc_accessories); i++) {
		if (machine_is_msm8x60_fluid()) {
			switch (othc_accessories[i].accessory) {
			case OTHC_ANC_HEADPHONE:
			case OTHC_ANC_HEADSET:
				othc_accessories[i].gpio = GPIO_HEADSET_DET_N;
				break;
			case OTHC_MICROPHONE:
				othc_accessories[i].enabled = false;
				break;
			}
		}
	}
}
#endif

static struct resource resources_pm8058_charger[] = {
	{	.name = "CHGVAL",
		.start = PM8058_CHGVAL_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGVAL_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{	.name = "CHGINVAL",
		.start = PM8058_CHGINVAL_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGINVAL_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHGILIM",
		.start = PM8058_CHGILIM_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGILIM_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "VCP",
		.start = PM8058_VCP_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_VCP_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
		{
		.name = "ATC_DONE",
		.start = PM8058_ATC_DONE_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_ATC_DONE_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "ATCFAIL",
		.start = PM8058_ATCFAIL_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_ATCFAIL_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "AUTO_CHGDONE",
		 .start = PM8058_AUTO_CHGDONE_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_AUTO_CHGDONE_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "AUTO_CHGFAIL",
		.start = PM8058_AUTO_CHGFAIL_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_AUTO_CHGFAIL_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHGSTATE",
		.start = PM8058_CHGSTATE_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGSTATE_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "FASTCHG",
		.start = PM8058_FASTCHG_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_FASTCHG_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHG_END",
		 .start = PM8058_CHG_END_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_CHG_END_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "BATTTEMP",
		.start = PM8058_BATTTEMP_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_BATTTEMP_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHGHOT",
		.start = PM8058_CHGHOT_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGHOT_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHGTLIMIT",
		.start = PM8058_CHGTLIMIT_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_CHGTLIMIT_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "CHG_GONE",
		 .start = PM8058_CHG_GONE_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_CHG_GONE_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "VCPMAJOR",
		 .start = PM8058_VCPMAJOR_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_VCPMAJOR_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "VBATDET",
		 .start = PM8058_VBATDET_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_VBATDET_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "BATFET",
		 .start = PM8058_BATFET_IRQ(PM8058_IRQ_BASE),
		 .end = PM8058_BATFET_IRQ(PM8058_IRQ_BASE),
		 .flags = IORESOURCE_IRQ,
	},
	{
		.name = "BATT_REPLACE",
		.start = PM8058_BATT_REPLACE_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_BATT_REPLACE_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "BATTCONNECT",
		.start = PM8058_BATTCONNECT_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_BATTCONNECT_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "VBATDET_LOW",
		.start = PM8058_VBATDET_LOW_IRQ(PM8058_IRQ_BASE),
		.end = PM8058_VBATDET_LOW_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
};

static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm8058_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_VPH,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};

	int rc = -EINVAL;
	int id, mode, max_mA;

	id = mode = max_mA = 0;
	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8058_gpio_config(id - 1, &pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8058_gpio_config(%d): rc=%d\n",
					__func__, id, rc);
		}
		break;

#ifdef CONFIG_LEDS_PMIC8058
	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 300;
		break;

	case 7:
		id = PM_PWM_LED_FLASH1;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 300;
		break;
#endif

	default:
		break;
	}

#ifdef CONFIG_LEDS_PMIC8058
	if (ch >= 6 && ch <= 7) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
				   __func__, ch, rc);
	}
#endif
	return rc;

}

static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config		= pm8058_pwm_config,
};

#define PM8058_GPIO_INT           88

static struct pm8058_gpio_platform_data pm8058_gpio_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 0),
	.init		= pm8058_gpios_init,
};

static struct pm8058_gpio_platform_data pm8058_gpio_data_topaz = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 0),
	.init		= pm8058_gpios_init_topaz,
};

static struct pm8058_gpio_platform_data pm8058_mpp_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS),
	.irq_base	= PM8058_MPP_IRQ(PM8058_IRQ_BASE, 0),
};

// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
#ifdef WORKAROUND_RPM_REGULATOR
static struct regulator_consumer_supply pm8058_vreg_supply[PM8058_VREG_MAX] = {
	[PM8058_VREG_ID_L10] = REGULATOR_SUPPLY("8058_l10", NULL),
	[PM8058_VREG_ID_L11] = REGULATOR_SUPPLY("8058_l11", NULL),
	[PM8058_VREG_ID_L14] = REGULATOR_SUPPLY("8058_l14", NULL),
	[PM8058_VREG_ID_L15] = REGULATOR_SUPPLY("8058_l15", NULL),
	[PM8058_VREG_ID_L19] = REGULATOR_SUPPLY("8058_l19", NULL),
	[PM8058_VREG_ID_LVS0] = REGULATOR_SUPPLY("8058_lvs0", NULL),
};

#define PM8058_VREG_INIT(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV) \
	[_id] = { \
		.constraints = { \
			.valid_modes_mask = _modes, \
			.valid_ops_mask = _ops, \
			.min_uV = _min_uV, \
			.max_uV = _max_uV, \
			.apply_uV = _apply_uV, \
		}, \
		.num_consumer_supplies = 1, \
		.consumer_supplies = &pm8058_vreg_supply[_id], \
	}
#define PM8058_VREG_INIT_ALWAYS_ON(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV) \
	[_id] = { \
		.constraints = { \
			.valid_modes_mask = _modes, \
			.valid_ops_mask = _ops, \
			.min_uV = _min_uV, \
			.max_uV = _max_uV, \
			.apply_uV = _apply_uV, \
			.always_on = 1, \
		}, \
		.num_consumer_supplies = 1, \
		.consumer_supplies = &pm8058_vreg_supply[_id], \
	}

#define PM8058_VREG_INIT_LDO(_id, _min_uV, _max_uV) \
	PM8058_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1)

#define PM8058_VREG_INIT_LDO_ALWAYS_ON(_id, _min_uV, _max_uV) \
	PM8058_VREG_INIT_ALWAYS_ON(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1)

#define PM8058_VREG_INIT_SMPS(_id, _min_uV, _max_uV) \
	PM8058_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1)

#define PM8058_VREG_INIT_LVS(_id) \
	PM8058_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_STATUS, 0)

#define PM8058_VREG_INIT_NCP(_id, _min_uV, _max_uV) \
	PM8058_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS, 1)

static struct regulator_init_data pm8058_vreg_init[PM8058_VREG_MAX] = {
	PM8058_VREG_INIT_LDO_ALWAYS_ON(PM8058_VREG_ID_L10, 3050000, 3050000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L11, 2850000, 2850000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L14, 2850000, 2850000),
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L15, 2850000, 2850000), //In current stage, keep the L15 always on //not always on, gyro driver will set it on, and no one will set to low
	PM8058_VREG_INIT_LDO(PM8058_VREG_ID_L19, 1800000, 1800000),
	PM8058_VREG_INIT_LVS(PM8058_VREG_ID_LVS0),
};

#define PM8058_VREG(_id) { \
	.name = "pm8058-regulator", \
	.id = _id, \
	.platform_data = &pm8058_vreg_init[_id], \
	.data_size = sizeof(pm8058_vreg_init[_id]), \
}

#endif
// end

static struct resource resources_rtc[] = {
	{
		.start  = PM8058_RTC_IRQ(PM8058_IRQ_BASE),
		.end    = PM8058_RTC_IRQ(PM8058_IRQ_BASE),
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = PM8058_RTC_ALARM_IRQ(PM8058_IRQ_BASE),
		.end    = PM8058_RTC_ALARM_IRQ(PM8058_IRQ_BASE),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct pmic8058_led pmic8058_flash_leds[] = {
	[0] = {
		.name		= "camera:flash0",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_0,
	},
	[1] = {
		.name		= "camera:flash1",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_1,
	},
};

static struct pmic8058_leds_platform_data pm8058_flash_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_flash_leds),
	.leds	= pmic8058_flash_leds,
};

static struct pmic8058_led pmic8058_fluid_flash_leds[] = {
	[0] = {
		.name		= "led:drv0",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_0,
	},/* 300 mA flash led0 drv sink */
	[1] = {
		.name		= "led:drv1",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_1,
	},/* 300 mA flash led1 sink */
	[2] = {
		.name		= "led:drv2",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_0,
	},/* 40 mA led0 sink */
	[3] = {
		.name		= "keypad:drv",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},/* 300 mA keypad drv sink */
};

static struct pmic8058_leds_platform_data pm8058_fluid_flash_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_fluid_flash_leds),
	.leds	= pmic8058_fluid_flash_leds,
};

static struct resource resources_temp_alarm[] = {
	{
		.start  = PM8058_TEMP_ALARM_IRQ(PM8058_IRQ_BASE),
		.end    = PM8058_TEMP_ALARM_IRQ(PM8058_IRQ_BASE),
		.flags  = IORESOURCE_IRQ,
	},
};

#define PM8058_SUBDEV_KPD 0
#define PM8058_SUBDEV_LED 1

static struct mfd_cell pm8058_subdevs[] = {
	{
		.name = "pm8058-keypad",
		.id		= -1,
		.num_resources	= ARRAY_SIZE(resources_keypad),
		.resources	= resources_keypad,
	},
	{	.name = "pm8058-led",
		.id		= -1,
	},
	{	.name = "pm8058-gpio",
		.id		= -1,
		.platform_data	= &pm8058_gpio_data,
		.data_size	= sizeof(pm8058_gpio_data),
	},
	{	.name = "pm8058-mpp",
		.id		= -1,
		.platform_data	= &pm8058_mpp_data,
		.data_size	= sizeof(pm8058_mpp_data),
	},
	{	.name = "pm8058-pwrkey",
		.id	= -1,
		.resources = resources_pwrkey,
		.num_resources = ARRAY_SIZE(resources_pwrkey),
		.platform_data = &pwrkey_pdata,
		.data_size = sizeof(pwrkey_pdata),
	},
	{
		.name = "pm8058-vib",
		.id = -1,
		.platform_data = &pmic_vib_pdata,
		.data_size     = sizeof(pmic_vib_pdata),
	},
	{
		.name = "pm8058-pwm",
		.id = -1,
		.platform_data = &pm8058_pwm_data,
		.data_size = sizeof(pm8058_pwm_data),
	},
#ifdef CONFIG_SENSORS_M_ADC
	{
		.name = "pm8058-xoadc",
		.id = -1,
		.num_resources = ARRAY_SIZE(resources_adc),
		.resources = resources_adc,
		.platform_data = &xoadc_pdata,
		.data_size = sizeof(xoadc_pdata),
	},
#endif
#if defined(CONFIG_PMIC8058_OTHC) || defined(CONFIG_PMIC8058_OTHC_MODULE)
	{
		.name = "pm8058-othc",
		.id = 0,
		.platform_data = &othc_config_pdata_0,
		.data_size = sizeof(othc_config_pdata_0),
		.num_resources = ARRAY_SIZE(resources_othc_0),
		.resources = resources_othc_0,
	},
	{
		/* OTHC1 module has headset/switch dection */
		.name = "pm8058-othc",
		.id = 1,
		.num_resources = ARRAY_SIZE(resources_othc_1),
		.resources = resources_othc_1,
		.platform_data = &othc_config_pdata_1,
		.data_size = sizeof(othc_config_pdata_1),
	},
	{
		.name = "pm8058-othc",
		.id = 2,
		.platform_data = &othc_config_pdata_2,
		.data_size = sizeof(othc_config_pdata_2),
		.num_resources = ARRAY_SIZE(resources_othc_2),
		.resources = resources_othc_2,
	},
#endif
	{
		.name = "pm8058-rtc",
		.id = -1,
		.num_resources  = ARRAY_SIZE(resources_rtc),
		.resources      = resources_rtc,
	},
	{
		.name = "pm8058-charger",
		.id = -1,
		.num_resources = ARRAY_SIZE(resources_pm8058_charger),
		.resources = resources_pm8058_charger,
	},
	{
		.name = "pm8058-tm",
		.id = -1,
		.num_resources  = ARRAY_SIZE(resources_temp_alarm),
		.resources      = resources_temp_alarm,
	},
	{	.name = "pm8058-upl",
		.id		= -1,
	},
};

static struct mfd_cell pm8058_subdevs_topaz[] = {
	{	.name = "pm8058-gpio",
		.id		= -1,
		.platform_data	= &pm8058_gpio_data_topaz,
		.data_size	= sizeof(pm8058_gpio_data_topaz),
	},
	{	.name = "pm8058-mpp",
		.id		= -1,
		.platform_data	= &pm8058_mpp_data,
		.data_size	= sizeof(pm8058_mpp_data),
	},
	{	.name = "pm8058-pwrkey",
		.id	= -1,
		.resources = resources_pwrkey,
		.num_resources = ARRAY_SIZE(resources_pwrkey),
		.platform_data = &pwrkey_pdata_topaz,
		.data_size = sizeof(pwrkey_pdata_topaz),
	},
	{
		.name = "pm8058-pwm",
		.id = -1,
		.platform_data = &pm8058_pwm_data,
		.data_size = sizeof(pm8058_pwm_data),
	},
#ifdef CONFIG_SENSORS_M_ADC
	{
		.name = "pm8058-xoadc",
		.id = -1,
		.num_resources = ARRAY_SIZE(resources_adc),
		.resources = resources_adc,
		.platform_data = &xoadc_pdata,
		.data_size = sizeof(xoadc_pdata),
	},
#endif
#if defined(CONFIG_PMIC8058_OTHC) || defined(CONFIG_PMIC8058_OTHC_MODULE)
	{
		.name = "pm8058-othc",
		.id = 0,
		.platform_data = &othc_config_pdata_0,
		.data_size = sizeof(othc_config_pdata_0),
		.num_resources = ARRAY_SIZE(resources_othc_0),
		.resources = resources_othc_0,
	},
	{
		/* OTHC1 module has headset/switch dection */
		.name = "pm8058-othc",
		.id = 1,
		.num_resources = ARRAY_SIZE(resources_othc_1),
		.resources = resources_othc_1,
		.platform_data = &othc_config_pdata_1,
		.data_size = sizeof(othc_config_pdata_1),
	},
	{
		.name = "pm8058-othc",
		.id = 2,
		.platform_data = &othc_config_pdata_2,
		.data_size = sizeof(othc_config_pdata_2),
		.num_resources = ARRAY_SIZE(resources_othc_2),
		.resources = resources_othc_2,
	},
#endif
	{
		.name = "pm8058-rtc",
		.id = -1,
		.num_resources  = ARRAY_SIZE(resources_rtc),
		.resources      = resources_rtc,
	},
	{
		.name = "pm8058-tm",
		.id = -1,
		.num_resources  = ARRAY_SIZE(resources_temp_alarm),
		.resources      = resources_temp_alarm,
	},
// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifdef WORKAROUND_RPM_REGULATOR
	PM8058_VREG(PM8058_VREG_ID_L10),
	PM8058_VREG(PM8058_VREG_ID_L11),
	PM8058_VREG(PM8058_VREG_ID_L14),
	PM8058_VREG(PM8058_VREG_ID_L15),
	PM8058_VREG(PM8058_VREG_ID_L19),
	PM8058_VREG(PM8058_VREG_ID_LVS0),
	#endif
// end		
	{	.name = "pm8058-upl",
		.id		= -1,
	},
};

static struct pm8058_platform_data pm8058_platform_data = {
	.irq_base = PM8058_IRQ_BASE,

	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
};

static struct pm8058_platform_data pm8058_platform_data_topaz = {
	.irq_base = PM8058_IRQ_BASE,

	.num_subdevs = ARRAY_SIZE(pm8058_subdevs_topaz),
	.sub_devices = pm8058_subdevs_topaz,
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_INT),
		.platform_data = &pm8058_platform_data,
	},
};

static struct i2c_board_info pm8058_boardinfo_topaz[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_INT),
		.platform_data = &pm8058_platform_data_topaz,
	},
};
#endif /* CONFIG_PMIC8058 */

#if defined(CONFIG_TOUCHDISC_VTD518_SHINETSU) || \
		defined(CONFIG_TOUCHDISC_VTD518_SHINETSU_MODULE)
#define TDISC_I2C_SLAVE_ADDR	0x67
#define PMIC_GPIO_TDISC		PM8058_GPIO_PM_TO_SYS(5)
#define TDISC_INT		PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 5)

static const char *vregs_tdisc_name[] = {
	"8058_l5",
	"8058_s3",
};

static const int vregs_tdisc_val[] = {
	2850000,/* uV */
	1800000,
};
static struct regulator *vregs_tdisc[ARRAY_SIZE(vregs_tdisc_name)];

static int tdisc_shinetsu_setup(void)
{
	int rc, i;

	rc = gpio_request(PMIC_GPIO_TDISC, "tdisc_interrupt");
	if (rc) {
		pr_err("%s: gpio_request failed for PMIC_GPIO_TDISC\n",
								__func__);
		return rc;
	}

	rc = gpio_request(GPIO_JOYSTICK_EN, "tdisc_oe");
	if (rc) {
		pr_err("%s: gpio_request failed for GPIO_JOYSTICK_EN\n",
							__func__);
		goto fail_gpio_oe;
	}

	rc = gpio_direction_output(GPIO_JOYSTICK_EN, 1);
	if (rc) {
		pr_err("%s: gpio_direction_output failed for GPIO_JOYSTICK_EN\n",
								__func__);
		gpio_free(GPIO_JOYSTICK_EN);
		goto fail_gpio_oe;
	}

	for (i = 0; i < ARRAY_SIZE(vregs_tdisc_name); i++) {
		vregs_tdisc[i] = regulator_get(NULL, vregs_tdisc_name[i]);
		if (IS_ERR(vregs_tdisc[i])) {
			printk(KERN_ERR "%s: regulator get %s failed (%ld)\n",
				__func__, vregs_tdisc_name[i],
				PTR_ERR(vregs_tdisc[i]));
			rc = PTR_ERR(vregs_tdisc[i]);
			goto vreg_get_fail;
		}

		rc = regulator_set_voltage(vregs_tdisc[i],
				vregs_tdisc_val[i], vregs_tdisc_val[i]);
		if (rc) {
			printk(KERN_ERR "%s: regulator_set_voltage() = %d\n",
				__func__, rc);
			goto vreg_set_voltage_fail;
		}
	}

	return rc;
vreg_set_voltage_fail:
	i++;
vreg_get_fail:
	while (i)
		regulator_put(vregs_tdisc[--i]);
fail_gpio_oe:
	gpio_free(PMIC_GPIO_TDISC);
	return rc;
}

static void tdisc_shinetsu_release(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vregs_tdisc_name); i++)
		regulator_put(vregs_tdisc[i]);

	gpio_free(PMIC_GPIO_TDISC);
	gpio_free(GPIO_JOYSTICK_EN);
}

static int tdisc_shinetsu_enable(void)
{
	int i, rc = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(vregs_tdisc_name); i++) {
		rc = regulator_enable(vregs_tdisc[i]);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg %s enable failed (%d)\n",
				__func__, vregs_tdisc_name[i], rc);
			goto vreg_fail;
		}
	}

	/* Enable the OE (output enable) gpio */
	gpio_set_value_cansleep(GPIO_JOYSTICK_EN, 1);
	/* voltage and gpio stabilization delay */
	msleep(50);

	return 0;
vreg_fail:
	while (i)
		regulator_disable(vregs_tdisc[--i]);
	return rc;
}

static int tdisc_shinetsu_disable(void)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(vregs_tdisc_name); i++) {
		rc = regulator_disable(vregs_tdisc[i]);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg %s disable failed (%d)\n",
				__func__, vregs_tdisc_name[i], rc);
			goto tdisc_reg_fail;
		}
	}

	/* Disable the OE (output enable) gpio */
	gpio_set_value_cansleep(GPIO_JOYSTICK_EN, 0);

	return 0;

tdisc_reg_fail:
	while (i)
		regulator_enable(vregs_tdisc[--i]);
	return rc;
}

static struct tdisc_abs_values tdisc_abs = {
	.x_max = 32,
	.y_max = 32,
	.x_min = -32,
	.y_min = -32,
	.pressure_max = 32,
	.pressure_min = 0,
};

static struct tdisc_platform_data tdisc_data = {
	.tdisc_setup = tdisc_shinetsu_setup,
	.tdisc_release = tdisc_shinetsu_release,
	.tdisc_enable = tdisc_shinetsu_enable,
	.tdisc_disable = tdisc_shinetsu_disable,
	.tdisc_wakeup  = 0,
	.tdisc_gpio = PMIC_GPIO_TDISC,
	.tdisc_report_keys = true,
	.tdisc_report_relative = true,
	.tdisc_report_absolute = false,
	.tdisc_report_wheel = false,
	.tdisc_reverse_x = false,
	.tdisc_reverse_y = true,
	.tdisc_abs  = &tdisc_abs,
};

static struct i2c_board_info msm_i2c_gsbi3_tdisc_info[] = {
	{
		I2C_BOARD_INFO("vtd518", TDISC_I2C_SLAVE_ADDR),
		.irq =  TDISC_INT,
		.platform_data = &tdisc_data,
	},
};
#endif


#ifdef CONFIG_MFD_WM8958

#define WM8994_LDO1_ENABLE 66
#define WM8994_LDO2_ENABLE 108


#if 0
static int wm8994_ldo_power(int enable)
{
		int ret = 0;
		if (enable)
		{
				/* Power up the WM8994 LDOs */
				pr_err("%s: Power up the WM8994 LDOs\n", __func__);
				ret = gpio_request(WM8994_LDO1_ENABLE, "wm8994-ldo1");
				if (ret != 0){
						pr_err("Failed to get GPIO for WM8994 LDO1: %d\n", ret);
						return ret;
				}
				gpio_direction_output(WM8994_LDO1_ENABLE, 1);
				gpio_set_value_cansleep(WM8994_LDO1_ENABLE,1);
				ret = gpio_request(WM8994_LDO2_ENABLE, "wm8994-ldo2");
				if (ret != 0){
						pr_err("Failed to get GPIO for WM8994 LDO2: %d\n", ret);
						return ret;
				}
				gpio_direction_output(WM8994_LDO2_ENABLE, 1);
				gpio_set_value_cansleep(WM8994_LDO2_ENABLE,1);
		}else
		{
				pr_err("%s: Power down the WM8994 LDOs\n", __func__);
				gpio_direction_output(WM8994_LDO1_ENABLE, 0);
				gpio_direction_output(WM8994_LDO2_ENABLE, 0);
				gpio_free(WM8994_LDO2_ENABLE);
				gpio_free(WM8994_LDO1_ENABLE);
		}
		return ret;
}

#else
static int wm8994_ldo_power(int enable)
{

		gpio_tlmm_config(GPIO_CFG(66, 0, GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(108, 0, GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (enable)
		{
				pr_err("%s: Power up the WM8994 LDOs\n", __func__);
				gpio_set_value_cansleep(66,1);
				gpio_set_value_cansleep(108,1);
		}else{
				pr_err("%s: Power down the WM8994 LDOs\n", __func__);
				gpio_set_value_cansleep(66,0);
				gpio_set_value_cansleep(108,0);
		}
		return 0;

}

#endif

static struct regulator *vreg_wm8958;
static unsigned int msm_wm8958_setup_power(void)
{
		int rc=0;

		pr_err("%s: codec power setup\n", __func__);
		vreg_wm8958 = regulator_get(NULL, "8058_s3");
		if (IS_ERR(vreg_wm8958)) {
				pr_err("%s: Unable to get 8058_s3\n", __func__);
				return -ENODEV;
		}

		if(regulator_set_voltage(vreg_wm8958, 1800000, 1800000))
		{
				pr_err("%s: Unable to set regulator voltage:"
								" votg_8058_s3\n", __func__);
		}
		rc = regulator_enable(vreg_wm8958);

		if (rc) {
				pr_err("%s: Enable regulator 8058_s3 failed\n", __func__);

		}
		wm8994_ldo_power(1);
		mdelay(30);
		return rc;
}

static void msm_wm8958_shutdown_power(void)
{
		int rc;
		pr_err("%s: codec power shutdown\n", __func__);
		wm8994_ldo_power(0);
		rc = regulator_disable(vreg_wm8958);
		if (rc)
				pr_err("%s: Disable regulator 8058_s3 failed\n", __func__);

		regulator_put(vreg_wm8958);
}

static int msm_wm8958_get_boardtype(void)
{
	if (board_is_topaz_3g()||board_is_topaz_wifi())
		return 0;
	 if(board_is_opal_3g() || board_is_opal_wifi())
		return 1;
	return -1;
}

static struct wm8994_pdata wm8958_pdata = {

		.gpio_defaults[0] = 0x1,       // gpio1 output as PA switch
		.gpio_defaults[5] = 0x2005,  // gpio6 output as IRQ to identify headset mic or key status change.
		.wm8994_setup = msm_wm8958_setup_power,
		.wm8994_shutdown = msm_wm8958_shutdown_power,
		.wm8994_get_boardtype = msm_wm8958_get_boardtype,
};



#define WM8958_I2C_SLAVE_ADDR	0x1a

static struct i2c_board_info msm_i2c_gsbi7_wm8958_info[] = {
	{
		I2C_BOARD_INFO("wm8958", WM8958_I2C_SLAVE_ADDR),
		.platform_data = &wm8958_pdata,
	},
};
#endif

#ifdef CONFIG_TIMPANI_CODEC
#define PM_GPIO_CDC_RST_N 20
#define GPIO_CDC_RST_N PM8058_GPIO_PM_TO_SYS(PM_GPIO_CDC_RST_N)

static struct regulator *vreg_timpani_1;
static struct regulator *vreg_timpani_2;

static unsigned int msm_timpani_setup_power(void)
{
	int rc;

	vreg_timpani_1 = regulator_get(NULL, "8058_l0");
	if (IS_ERR(vreg_timpani_1)) {
		pr_err("%s: Unable to get 8058_l0\n", __func__);
		return -ENODEV;
	}

	vreg_timpani_2 = regulator_get(NULL, "8058_s3");
	if (IS_ERR(vreg_timpani_2)) {
		pr_err("%s: Unable to get 8058_s3\n", __func__);
		regulator_put(vreg_timpani_1);
		return -ENODEV;
	}

	rc = regulator_set_voltage(vreg_timpani_1, 1200000, 1200000);
	if (rc) {
		pr_err("%s: unable to set L0 voltage to 1.2V\n", __func__);
		goto fail;
	}

	rc = regulator_set_voltage(vreg_timpani_2, 1800000, 1800000);
	if (rc) {
		pr_err("%s: unable to set S3 voltage to 1.8V\n", __func__);
		goto fail;
	}

	rc = regulator_enable(vreg_timpani_1);
	if (rc) {
		pr_err("%s: Enable regulator 8058_l0 failed\n", __func__);
		goto fail;
	}

	rc = regulator_enable(vreg_timpani_2);
	if (rc) {
		pr_err("%s: Enable regulator 8058_s3 failed\n", __func__);
		regulator_disable(vreg_timpani_1);
		goto fail;
	}

	rc = gpio_request(GPIO_CDC_RST_N, "CDC_RST_N");
	if (rc) {
		pr_err("%s: GPIO Request %d failed\n", __func__,
			GPIO_CDC_RST_N);
		regulator_disable(vreg_timpani_1);
		regulator_disable(vreg_timpani_2);
		goto fail;
	} else {
		gpio_direction_output(GPIO_CDC_RST_N, 1);
		usleep_range(1000, 1050);
		gpio_direction_output(GPIO_CDC_RST_N, 0);
		usleep_range(1000, 1050);
		gpio_direction_output(GPIO_CDC_RST_N, 1);
		gpio_free(GPIO_CDC_RST_N);
	}
	return rc;

fail:
	regulator_put(vreg_timpani_1);
	regulator_put(vreg_timpani_2);
	return rc;
}

static void msm_timpani_shutdown_power(void)
{
	int rc;

	rc = regulator_disable(vreg_timpani_1);
	if (rc)
		pr_err("%s: Disable regulator 8058_l0 failed\n", __func__);

	regulator_put(vreg_timpani_1);

	rc = regulator_disable(vreg_timpani_2);
	if (rc)
		pr_err("%s: Disable regulator 8058_s3 failed\n", __func__);

	regulator_put(vreg_timpani_2);
}

/* Power analog function of codec */
static struct regulator *vreg_timpani_cdc_apwr;
static int msm_timpani_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_timpani_cdc_apwr) {

		vreg_timpani_cdc_apwr = regulator_get(NULL, "8058_s4");

		if (IS_ERR(vreg_timpani_cdc_apwr)) {
			pr_err("%s: vreg_get failed (%ld)\n",
			__func__, PTR_ERR(vreg_timpani_cdc_apwr));
			rc = PTR_ERR(vreg_timpani_cdc_apwr);
			return rc;
		}
	}

	if (vreg_on) {

		rc = regulator_set_voltage(vreg_timpani_cdc_apwr,
				2200000, 2200000);
		if (rc) {
			pr_err("%s: unable to set 8058_s4 voltage to 2.2 V\n",
					__func__);
			goto vreg_fail;
		}

		rc = regulator_enable(vreg_timpani_cdc_apwr);
		if (rc) {
			pr_err("%s: vreg_enable failed %d\n", __func__, rc);
			goto vreg_fail;
		}
	} else {
		rc = regulator_disable(vreg_timpani_cdc_apwr);
		if (rc) {
			pr_err("%s: vreg_disable failed %d\n",
			__func__, rc);
			goto vreg_fail;
		}
	}

	return 0;

vreg_fail:
	regulator_put(vreg_timpani_cdc_apwr);
	vreg_timpani_cdc_apwr = NULL;
	return rc;
}

static struct marimba_codec_platform_data timpani_codec_pdata = {
	.marimba_codec_power =  msm_timpani_codec_power,
};

#define TIMPANI_SLAVE_ID_CDC_ADDR		0X77
#define TIMPANI_SLAVE_ID_QMEMBIST_ADDR		0X66

static struct marimba_platform_data timpani_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	= TIMPANI_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = TIMPANI_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_timpani_setup_power,
	.marimba_shutdown = msm_timpani_shutdown_power,
	.codec = &timpani_codec_pdata,
};

#define TIMPANI_I2C_SLAVE_ADDR	0xD

static struct i2c_board_info msm_i2c_gsbi7_timpani_info[] = {
	{
		I2C_BOARD_INFO("timpani", TIMPANI_I2C_SLAVE_ADDR),
		.platform_data = &timpani_pdata,
	},
};
#endif

#ifdef CONFIG_PMIC8901

#define PM8901_GPIO_INT           91

static struct pm8901_gpio_platform_data pm8901_mpp_data = {
	.gpio_base	= PM8901_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8901_MPP_IRQ(PM8901_IRQ_BASE, 0),
};

static struct resource pm8901_temp_alarm[] = {
	{
		.start = PM8901_TEMP_ALARM_IRQ(PM8901_IRQ_BASE),
		.end = PM8901_TEMP_ALARM_IRQ(PM8901_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = PM8901_TEMP_HI_ALARM_IRQ(PM8901_IRQ_BASE),
		.end = PM8901_TEMP_HI_ALARM_IRQ(PM8901_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
};

static struct regulator_consumer_supply pm8901_vreg_supply[PM8901_VREG_MAX] = {
	[PM8901_VREG_ID_MPP0] =     REGULATOR_SUPPLY("8901_mpp0",     NULL),
// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifdef WORKAROUND_RPM_REGULATOR
	[PM8901_VREG_ID_L1]  = REGULATOR_SUPPLY("8901_l1",  NULL),
	[PM8901_VREG_ID_L3]  = REGULATOR_SUPPLY("8901_l3",  NULL),
	[PM8901_VREG_ID_LVS3]     = REGULATOR_SUPPLY("8901_lvs3",     NULL),
	#endif
// end	
	[PM8901_VREG_ID_USB_OTG]  = REGULATOR_SUPPLY("8901_usb_otg",  NULL),
	[PM8901_VREG_ID_HDMI_MVS] = REGULATOR_SUPPLY("8901_hdmi_mvs", NULL),
};

#define PM8901_VREG_INIT(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV, \
			 _always_on, _active_high) \
	[_id] = { \
		.init_data = { \
			.constraints = { \
				.valid_modes_mask = _modes, \
				.valid_ops_mask = _ops, \
				.min_uV = _min_uV, \
				.max_uV = _max_uV, \
				.input_uV = _min_uV, \
				.apply_uV = _apply_uV, \
				.always_on = _always_on, \
			}, \
			.num_consumer_supplies = 1, \
			.consumer_supplies = &pm8901_vreg_supply[_id], \
		}, \
		.active_high = _active_high, \
	}

// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
#ifdef WORKAROUND_RPM_REGULATOR
#if 0
#define PM8901_VREG_INIT_VS_ALWAYSON_HP(_id) \
	PM8901_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_STATUS, 0, 1, 0)
#endif

#define PM8901_VREG_INIT_LDO(_id, _min_uV, _max_uV) \
	PM8901_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1, 0, 0)
#endif
// end

#define PM8901_VREG_INIT_MPP(_id, _active_high) \
	PM8901_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_STATUS, 0, 0, _active_high)

#define PM8901_VREG_INIT_VS(_id) \
	PM8901_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_STATUS, 0, 0, 0)

static struct pm8901_vreg_pdata pm8901_vreg_init_pdata[PM8901_VREG_MAX] = {
	PM8901_VREG_INIT_MPP(PM8901_VREG_ID_MPP0, 1),
// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifdef WORKAROUND_RPM_REGULATOR
	PM8901_VREG_INIT_LDO(PM8901_VREG_ID_L1, 3300000, 3300000),
	PM8901_VREG_INIT_LDO(PM8901_VREG_ID_L3, 3300000, 3300000),
//	PM8901_VREG_INIT_VS_ALWAYSON_HP(PM8901_VREG_ID_LVS3),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_LVS3),
	#endif
// end	
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_USB_OTG),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_HDMI_MVS),
};

#define PM8901_VREG(_id) { \
	.name = "pm8901-regulator", \
	.id = _id, \
	.platform_data = &pm8901_vreg_init_pdata[_id], \
	.data_size = sizeof(pm8901_vreg_init_pdata[_id]), \
}

static struct mfd_cell pm8901_subdevs[] = {
	{	.name = "pm8901-mpp",
		.id		= -1,
		.platform_data	= &pm8901_mpp_data,
		.data_size	= sizeof(pm8901_mpp_data),
	},
	{	.name = "pm8901-tm",
		.id		= -1,
		.num_resources  = ARRAY_SIZE(pm8901_temp_alarm),
		.resources      = pm8901_temp_alarm,
	},
	PM8901_VREG(PM8901_VREG_ID_MPP0),
// HP Wade Wang: Workaround because there are too many issue in rpm regulator driver
	#ifdef WORKAROUND_RPM_REGULATOR
	PM8901_VREG(PM8901_VREG_ID_L1),
	PM8901_VREG(PM8901_VREG_ID_L3),
	PM8901_VREG(PM8901_VREG_ID_LVS3),
	#endif
// end	
	PM8901_VREG(PM8901_VREG_ID_USB_OTG),
	PM8901_VREG(PM8901_VREG_ID_HDMI_MVS),
};

static struct pm8901_platform_data pm8901_platform_data = {
	.irq_base = PM8901_IRQ_BASE,
	.num_subdevs = ARRAY_SIZE(pm8901_subdevs),
	.sub_devices = pm8901_subdevs,
};

static struct i2c_board_info pm8901_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8901-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PM8901_GPIO_INT),
		.platform_data = &pm8901_platform_data,
	},
};

#endif /* CONFIG_PMIC8901 */

#if defined(CONFIG_BAHAMA_CORE) && (defined(CONFIG_GPIO_SX150X) \
	|| defined(CONFIG_GPIO_SX150X_MODULE))

static struct regulator *vreg_bahama;

static int msm_bahama_setup_power(struct device *dev)
{
	int rc = 0;
	const char *msm_bahama_regulator = "8058_s3";

	vreg_bahama = regulator_get(dev, msm_bahama_regulator);
	if (IS_ERR(vreg_bahama)) {
		rc = PTR_ERR(vreg_bahama);
		dev_err(dev, "%s: regulator_get %s = %d\n", __func__,
			msm_bahama_regulator, rc);
	}

	if (!rc)
		rc = regulator_set_voltage(vreg_bahama, 1800000, 1800000);
	else {
		dev_err(dev, "%s: regulator_set_voltage %s = %d\n", __func__,
			msm_bahama_regulator, rc);
		goto unget;
	}

	if (!rc)
		rc = regulator_enable(vreg_bahama);
	else {
		dev_err(dev, "%s: regulator_enable %s = %d\n", __func__,
			msm_bahama_regulator, rc);
		goto unget;
	}

	if (!rc)
		rc = gpio_request(GPIO_MS_SYS_RESET_N, "bahama sys_rst_n");
	else {
		dev_err(dev, "%s: gpio_request %d = %d\n", __func__,
			GPIO_MS_SYS_RESET_N, rc);
		goto unenable;
	}

	if (!rc)
		rc = gpio_direction_output(GPIO_MS_SYS_RESET_N, 1);
	else {
		dev_err(dev, "%s: gpio_direction_output %d = %d\n", __func__,
			GPIO_MS_SYS_RESET_N, rc);
		goto unrequest;
	}

	return rc;

unrequest:
	gpio_free(GPIO_MS_SYS_RESET_N);
unenable:
	regulator_disable(vreg_bahama);
unget:
	regulator_put(vreg_bahama);
	return rc;
};

static void msm_bahama_shutdown_power(struct device *dev)
{
	gpio_set_value(GPIO_MS_SYS_RESET_N, 0);

	gpio_free(GPIO_MS_SYS_RESET_N);

	regulator_disable(vreg_bahama);

	regulator_put(vreg_bahama);
};

static struct bahama_platform_data bahama_pdata = {
	.bahama_setup = msm_bahama_setup_power,
	.bahama_shutdown = msm_bahama_shutdown_power,
};

static struct i2c_board_info msm_i2c_gsbi7_bahama_info[] = {
	{
		I2C_BOARD_INFO("bahama", 0xc),
		.platform_data = &bahama_pdata,
	}
};
#endif /* CONFIG_BAHAMA_CORE */

#ifdef CONFIG_I2C
#define I2C_SURF 1
#define I2C_FFA  (1 << 1)
#define I2C_RUMI (1 << 2)
#define I2C_SIM  (1 << 3)
#define I2C_FLUID (1 << 4)
#define I2C_TOPAZ (1 << 7)
#define I2C_OPAL (1 << 6)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

static struct i2c_registry msm8x60_i2c_devices[] __initdata = {
#ifdef CONFIG_PMIC8058
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_SSBI1_I2C_BUS_ID,
		pm8058_boardinfo,
		ARRAY_SIZE(pm8058_boardinfo),
	},
	{
		I2C_TOPAZ | I2C_OPAL,
		MSM_SSBI1_I2C_BUS_ID,
		pm8058_boardinfo_topaz,
		ARRAY_SIZE(pm8058_boardinfo_topaz),
	},
#endif
#ifdef CONFIG_PMIC8901
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_SSBI2_I2C_BUS_ID,
		pm8901_boardinfo,
		ARRAY_SIZE(pm8901_boardinfo),
	},
	{
		I2C_TOPAZ | I2C_OPAL,
		MSM_SSBI2_I2C_BUS_ID,
		pm8901_boardinfo,
		ARRAY_SIZE(pm8901_boardinfo),
	},
#endif
#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI8_QUP_I2C_BUS_ID,
		core_expander_i2c_info,
		ARRAY_SIZE(core_expander_i2c_info),
	},
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI8_QUP_I2C_BUS_ID,
		docking_expander_i2c_info,
		ARRAY_SIZE(docking_expander_i2c_info),
	},
	{
		I2C_SURF,
		MSM_GSBI8_QUP_I2C_BUS_ID,
		surf_expanders_i2c_info,
		ARRAY_SIZE(surf_expanders_i2c_info),
	},
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		fha_expanders_i2c_info,
		ARRAY_SIZE(fha_expanders_i2c_info),
	},
	{
		I2C_FLUID,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		fluid_expanders_i2c_info,
		ARRAY_SIZE(fluid_expanders_i2c_info),
	},
	{
		I2C_FLUID,
		MSM_GSBI8_QUP_I2C_BUS_ID,
		fluid_core_expander_i2c_info,
		ARRAY_SIZE(fluid_core_expander_i2c_info),
	},
#endif
#if defined(CONFIG_TOUCHDISC_VTD518_SHINETSU) || \
		defined(CONFIG_TOUCHDISC_VTD518_SHINETSU_MODULE)
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		msm_i2c_gsbi3_tdisc_info,
		ARRAY_SIZE(msm_i2c_gsbi3_tdisc_info),
	},
#endif
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		cy8ctmg200_board_info,
		ARRAY_SIZE(cy8ctmg200_board_info),
	},
#if defined(CONFIG_TOUCHSCREEN_CYTTSP_I2C) || \
		defined(CONFIG_TOUCHSCREEN_CYTTSP_I2C_MODULE)
	{
		I2C_FLUID,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		cyttsp_fluid_info,
		ARRAY_SIZE(cyttsp_fluid_info),
	},
#endif
	{
		I2C_TOPAZ | I2C_OPAL,
		MSM_GSBI10_QUP_I2C_BUS_ID,   // yegw use GSBI10 as touch i2c
		xMT1386_board_info,
		ARRAY_SIZE(xMT1386_board_info),
	},
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_HP_I2C	
	{
		I2C_TOPAZ | I2C_OPAL,
		MSM_GSBI10_QUP_I2C_BUS_ID,   // yegw use GSBI10 as touch i2c
		cypress_cytcb5s_touch_info,
		ARRAY_SIZE(cypress_cytcb5s_touch_info),
	},
#endif
#ifdef  CONFIG_LEDS_LM8502
	{
		I2C_TOPAZ | I2C_OPAL,
		MSM_GSBI8_QUP_I2C_BUS_ID,
		lm8502_board_info,
		ARRAY_SIZE(lm8502_board_info),
	},
#endif
#ifdef CONFIG_MSM_CAMERA
	{
		I2C_SURF | I2C_FFA | I2C_FLUID | I2C_TOPAZ | I2C_OPAL,
		MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo,
		ARRAY_SIZE(msm_camera_boardinfo),
	},
#endif
#ifdef CONFIG_MFD_WM8958
{
		I2C_TOPAZ | I2C_OPAL,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_wm8958_info,
		ARRAY_SIZE(msm_i2c_gsbi7_wm8958_info),
	},
#endif
#ifdef CONFIG_TIMPANI_CODEC
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_timpani_info,
		ARRAY_SIZE(msm_i2c_gsbi7_timpani_info),
	},
#endif
#if defined(CONFIG_BAHAMA_CORE) && (defined(CONFIG_GPIO_SX150X) \
	|| defined(CONFIG_GPIO_SX150X_MODULE))
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_bahama_info,
		ARRAY_SIZE(msm_i2c_gsbi7_bahama_info),
	},
#endif /* CONFIG_BAHAMA_CORE */
#ifdef CONFIG_ISL9519_CHARGER
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI8_QUP_I2C_BUS_ID,
		isl_charger_i2c_info,
		ARRAY_SIZE(isl_charger_i2c_info),
	},
#endif
#if defined(CONFIG_HAPTIC_ISA1200) || \
		defined(CONFIG_HAPTIC_ISA1200_MODULE)
	{
		I2C_FLUID,
		MSM_GSBI8_QUP_I2C_BUS_ID,
		msm_isa1200_board_info,
		ARRAY_SIZE(msm_isa1200_board_info),
	},
#endif
#ifdef CONFIG_SENSORS_MPU3050
	{
		I2C_TOPAZ | I2C_OPAL,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		mpu3050_boardinfo,
		ARRAY_SIZE(mpu3050_boardinfo),
	},
#endif
#ifdef CONFIG_INPUT_ISL29023
	{
		I2C_TOPAZ | I2C_OPAL,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		intersil_isl29023,
		ARRAY_SIZE(intersil_isl29023),
	},
#endif
//guoye:[20100914] st lsm303dlh g/a/o sensor
#if defined(CONFIG_INPUT_LSM303_ACCEL) || \
		defined(CONFIG_INPUT_LSM303_MAGNE)
	{
		I2C_TOPAZ | I2C_OPAL,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		lsm303_i2c_devs,
		ARRAY_SIZE(lsm303_i2c_devs),
	},
#endif
//guoye
#if defined(CONFIG_FB_MSM_LCDC_LG_XGA) || defined(CONFIG_FB_MSM_LCDC_HITACHI_XGA)
{
	I2C_TOPAZ | I2C_OPAL,
	MSM_GSBI3_QUP_I2C_BUS_ID,
	lcd_panel_i2c_info,
	ARRAY_SIZE(lcd_panel_i2c_info),
},
#endif
//HP zhanghong: Oct 27 11:34 CST 2010,begin
#ifdef CONFIG_INPUT_CYPRESS_CY8C20236A
	{
		I2C_TOPAZ | I2C_OPAL,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		cy8c20236a_i2c_info,
		ARRAY_SIZE(cy8c20236a_i2c_info),
	},
#endif
//wanqin nfc
#ifdef CONFIG_NFC
	{
		I2C_TOPAZ | I2C_OPAL,
		MSM_GSBI3_QUP_I2C_BUS_ID,
		nfc_i2c_info,
		ARRAY_SIZE(nfc_i2c_info),
	},
#endif
};
#endif /* CONFIG_I2C */

static void fixup_i2c_configs(void)
{
#ifdef CONFIG_I2C
#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)
	if (machine_is_msm8x60_surf() || machine_is_msm8x60_charm_surf())
		sx150x_data[SX150X_CORE].irq_summary =
			PM8058_GPIO_IRQ(PM8058_IRQ_BASE, UI_INT2_N);
	else if (machine_is_msm8x60_ffa() || machine_is_msm8x60_charm_ffa())
		sx150x_data[SX150X_CORE].irq_summary =
			PM8058_GPIO_IRQ(PM8058_IRQ_BASE, UI_INT1_N);
	else if (machine_is_msm8x60_fluid())
		sx150x_data[SX150X_CORE_FLUID].irq_summary =
			PM8058_GPIO_IRQ(PM8058_IRQ_BASE, UI_INT1_N);
#endif

	/*
	 * Set PMIC 8901 MPP0 active_high to 0 for surf and charm_surf. This
	 * implies that the regulator connected to MPP0 is enabled when
	 * MPP0 is low.
	 */
	// HP Wade: 8901 MPP0 is not used on topaz and opal 
	if (machine_is_msm8x60_topaz() || machine_is_msm8x60_opal())
		pm8901_vreg_init_pdata[PM8901_VREG_ID_MPP0].active_high = 0;
	else 
	// End
	if (machine_is_msm8x60_surf() || machine_is_msm8x60_charm_surf())
		pm8901_vreg_init_pdata[PM8901_VREG_ID_MPP0].active_high = 0;
	else
		pm8901_vreg_init_pdata[PM8901_VREG_ID_MPP0].active_high = 1;
#endif
}

static void register_i2c_devices(void)
{
#ifdef CONFIG_I2C
	u8 mach_mask = 0;
	int i;

	/* Build the matching 'supported_machs' bitmask */
	if (machine_is_msm8x60_surf() || machine_is_msm8x60_charm_surf())
		mach_mask = I2C_SURF;
	else if (machine_is_msm8x60_ffa() || machine_is_msm8x60_charm_ffa())
		mach_mask = I2C_FFA;
	else if (machine_is_msm8x60_rumi3())
		mach_mask = I2C_RUMI;
	else if (machine_is_msm8x60_sim())
		mach_mask = I2C_SIM;
	else if (machine_is_msm8x60_fluid())
		mach_mask = I2C_FLUID;
	else if (machine_is_msm8x60_topaz())
		mach_mask = I2C_TOPAZ;
	else if (machine_is_msm8x60_opal())
		mach_mask = I2C_OPAL;
	else
		pr_err("unmatched machine ID in register_i2c_devices\n");

	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(msm8x60_i2c_devices); ++i) {
		if (msm8x60_i2c_devices[i].machs & mach_mask) {
			if(!strcmp(msm8x60_i2c_devices[i].info->type,"cy8c20236a")) {
				if ((board_is_topaz_3g() && (board_type > TOPAZ3G_DVT))
					||(board_is_opal_3g() && (board_type > OPAL3G_EVT1))) {
					msm8x60_i2c_devices[i].bus = MSM_GSBI9_QUP_I2C_BUS_ID;
				}
			}
			i2c_register_board_info(msm8x60_i2c_devices[i].bus,
						msm8x60_i2c_devices[i].info,
						msm8x60_i2c_devices[i].len);
		}
	}
#endif
}

//HP_Effie, QUP_SPI support , Start
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
#ifdef CONFIG_FB_MSM_LCDC_HITACHI_XGA
static struct spi_board_info lcdc_hitachi_spi_board_info[] __initdata = {
	{
		.modalias	= "lcdc_hitachi_tx18d42vm",
		.mode		= SPI_MODE_1,
		.bus_num	= MSM_GSBI5_QUP_SPI_BUS_ID,
		.chip_select	= 0,
		.max_speed_hz	= 5400000,
	}
};
#endif

static void register_spi_devices(void)
{
#ifdef CONFIG_FB_MSM_LCDC_HITACHI_XGA
	spi_register_board_info(lcdc_hitachi_spi_board_info,
			ARRAY_SIZE(lcdc_hitachi_spi_board_info));
#endif
}
#endif
//HP_Effie, QUP_SPI support , End

static void register_lcdc_panel(void)
{
	if(board_is_opal_3g() || board_is_opal_wifi())
	{
		if(board_is_opal_3g() && board_type == OPAL3G_PROTO)
		{
			#ifdef CONFIG_FB_MSM_LCDC_LG_XGA
			platform_device_register(&lcdc_lg_panel_device);
			#endif
		}
		else
		{
			#ifdef CONFIG_FB_MSM_LCDC_HITACHI_XGA
			platform_device_register(&lcdc_hitachi_panel_device);
			#endif
		}
	}
	else if(board_is_topaz_3g() || board_is_topaz_wifi())
	{ 
		#ifdef CONFIG_FB_MSM_LCDC_LG_XGA
		platform_device_register(&lcdc_lg_panel_device);
		#endif
	}
}

static void __init msm8x60_init_uart12dm(void)
{
#if !defined(CONFIG_USB_PEHCI_HCD) && !defined(CONFIG_USB_PEHCI_HCD_MODULE)
	/* 0x1D000000 now belongs to EBI2:CS3 i.e. USB ISP Controller */
	void *fpga_mem = ioremap_nocache(0x1D000000, SZ_4K);
	/* Advanced mode */
	writew(0xFFFF, fpga_mem + 0x15C);
	/* FPGA_UART_SEL */
	writew(0, fpga_mem + 0x172);
	/* FPGA_GPIO_CONFIG_117 */
	writew(1, fpga_mem + 0xEA);
	/* FPGA_GPIO_CONFIG_118 */
	writew(1, fpga_mem + 0xEC);
	dmb();
	iounmap(fpga_mem);
#endif
}

static void __init msm8x60_init_buses(void)
{
#ifdef CONFIG_I2C_QUP
	void *gsbi_mem = ioremap_nocache(0x19C00000, 4);
	/* Setting protocol code to 0x60 for dual UART/I2C in GSBI12 */
	writel(0x6 << 4, gsbi_mem);
	iounmap(gsbi_mem);
	msm_gsbi3_qup_i2c_device.dev.platform_data = &msm_gsbi3_qup_i2c_pdata;
	msm_gsbi4_qup_i2c_device.dev.platform_data = &msm_gsbi4_qup_i2c_pdata;
	msm_gsbi7_qup_i2c_device.dev.platform_data = &msm_gsbi7_qup_i2c_pdata;
	msm_gsbi8_qup_i2c_device.dev.platform_data = &msm_gsbi8_qup_i2c_pdata;
	msm_gsbi9_qup_i2c_device.dev.platform_data = &msm_gsbi9_qup_i2c_pdata;
	// Yegw 2010-8-20 config gsbi10 as i2c function
	msm_gsbi10_qup_i2c_device.dev.platform_data = &msm_gsbi10_qup_i2c_pdata;
	// Yegw End
	msm_gsbi12_qup_i2c_device.dev.platform_data = &msm_gsbi12_qup_i2c_pdata;
#endif
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	msm_gsbi1_qup_spi_device.dev.platform_data = &msm_gsbi1_qup_spi_pdata;
	msm_gsbi5_qup_spi_device.dev.platform_data = &msm_gsbi5_qup_spi_pdata;
#endif
#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi1.dev.platform_data = &msm_ssbi1_pdata;
	msm_device_ssbi2.dev.platform_data = &msm_ssbi2_pdata;
	msm_device_ssbi3.dev.platform_data = &msm_ssbi3_pdata;
#endif
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_HCD)
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(54); /* GSBI6(2) */
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
#ifdef CONFIG_MSM_BUS_SCALING

	/* RPM calls are only enabled on V2 */
	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2) {
		msm_bus_apps_fabric_pdata.rpm_enabled = 1;
		msm_bus_sys_fabric_pdata.rpm_enabled = 1;
		msm_bus_mm_fabric_pdata.rpm_enabled = 1;
		msm_bus_sys_fpb_pdata.rpm_enabled = 1;
		msm_bus_cpss_fpb_pdata.rpm_enabled = 1;
	}

	msm_bus_apps_fabric.dev.platform_data = &msm_bus_apps_fabric_pdata;
	msm_bus_sys_fabric.dev.platform_data = &msm_bus_sys_fabric_pdata;
	msm_bus_mm_fabric.dev.platform_data = &msm_bus_mm_fabric_pdata;
	msm_bus_sys_fpb.dev.platform_data = &msm_bus_sys_fpb_pdata;
	msm_bus_cpss_fpb.dev.platform_data = &msm_bus_cpss_fpb_pdata;
#endif
}

static void __init msm8x60_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_msm8x60_io();
	msm8x60_allocate_memory_regions();
}

/*
 * Most segments of the EBI2 bus are disabled by default.
 */
static void __init msm8x60_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(0x1a100000, sizeof(uint32_t));
	if (ebi2_cfg_ptr != 0) {
		ebi2_cfg = readl(ebi2_cfg_ptr);

		if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa() || machine_is_msm8x60_topaz() || machine_is_msm8x60_opal() ||
			machine_is_msm8x60_fluid())
			ebi2_cfg |= (1 << 4) | (1 << 5); /* CS2, CS3 */
		else if (machine_is_msm8x60_sim())
			ebi2_cfg |= (1 << 4); /* CS2 */
		else if (machine_is_msm8x60_rumi3())
			ebi2_cfg |= (1 << 5); /* CS3 */

		writel(ebi2_cfg, ebi2_cfg_ptr);
		iounmap(ebi2_cfg_ptr);
	}

	if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa() || machine_is_msm8x60_topaz() || machine_is_msm8x60_opal()||
			machine_is_msm8x60_fluid()) {
		ebi2_cfg_ptr = ioremap_nocache(0x1a110000, SZ_4K);
		if (ebi2_cfg_ptr != 0) {
			/* EBI2_XMEM_CFG:PWRSAVE_MODE off */
			writel(0UL, ebi2_cfg_ptr);

			/* CS2: Delay 9 cycles (140ns@64MHz) between SMSC
			 * LAN9221 Ethernet controller reads and writes.
			 * The lowest 4 bits are the read delay, the next
			 * 4 are the write delay. */
			writel(0x031F1C99, ebi2_cfg_ptr + 0x10);
#if defined(CONFIG_USB_PEHCI_HCD)|| defined(CONFIG_USB_PEHCI_HCD_MODULE)
// HP Wade Wang: 
			if (board_is_topaz_wifi())
			{
				/* EBI2 CS3 muxed address/data,
				 * two cyc addr enable */
				writel(0xA3030020, ebi2_cfg_ptr + 0x34);
			}
			else
// End
			{
			/*
			 * RECOVERY=5, HOLD_WR=1
			 * INIT_LATENCY_WR=1, INIT_LATENCY_RD=1
			 * WAIT_WR=1, WAIT_RD=2
			 */
			writel(0x51010112, ebi2_cfg_ptr + 0x14);
			/*
			 * HOLD_RD=1
			 * ADV_OE_RECOVERY=0, ADDR_HOLD_ENA=1
			 */
			writel(0x01000020, ebi2_cfg_ptr + 0x34);
			}
#else
			/* EBI2 CS3 muxed address/data,
			 * two cyc addr enable */
			writel(0xA3030020, ebi2_cfg_ptr + 0x34);

#endif
			iounmap(ebi2_cfg_ptr);
		}
	}
}

static void __init msm8x60_configure_smc91x(void)
{
	if (machine_is_msm8x60_sim()) {

		smc91x_resources[0].start = 0x1b800300;
		smc91x_resources[0].end   = 0x1b8003ff;

		smc91x_resources[1].start = (NR_MSM_IRQS + 40);
		smc91x_resources[1].end   = (NR_MSM_IRQS + 40);

	} else if (machine_is_msm8x60_rumi3()) {

		smc91x_resources[0].start = 0x1d000300;
		smc91x_resources[0].end   = 0x1d0003ff;

		smc91x_resources[1].start = TLMM_SCSS_DIR_CONN_IRQ_0;
		smc91x_resources[1].end   = TLMM_SCSS_DIR_CONN_IRQ_0;
	}
}

struct msm8x60_tlmm_cfg_struct {
	unsigned gpio;
	u32      flags;
};

static uint32_t topazwifi_tlmm_cfgs[] = {
    /* GSBI10 GSBI uart */
	GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
// yegw 2010-9-7 config gpio to GSBI function
	/* GSBI10 QUP I2C */
	GPIO_CFG(72, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(73, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
// yegw End
#ifdef CONFIG_MFD_WM8958
	/* mic detect gpio  */
	GPIO_CFG(57, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* jack detect gpio */
	GPIO_CFG(67, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* GSBI9 QUP I2C */
	GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* WM8994 LDO ENABLE */
	GPIO_CFG(66,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(108, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif
	GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(44, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
//michael - configure wlan necessary GPIOs at one shot
#if defined(CONFIG_AR6003) || defined(CONFIG_AR6003_MODULE)
	/* AR6003_GPIO_HOST_WAKE_WL */
	GPIO_CFG(AR6003_GPIO_HOST_WAKE_WL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* AR6003_GPIO_WL_IRQ */
// 2011.2.28 HP Henry: remove it for h/w change	GPIO_CFG(AR6003_GPIO_WL_IRQ,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* AR6003_GPIO_WL_HOST_WAKE */
	GPIO_CFG(AR6003_GPIO_WL_HOST_WAKE,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	/* AR6003_GPIO_WLAN_RST_N */
	GPIO_CFG(AR6003_GPIO_WLAN_RST_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
#endif
	/* 5V Bias Enable */
	GPIO_CFG(102, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	/*board id pins*/
	GPIO_CFG(95, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(96, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(97, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(98, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(99, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(100, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
// HP Wade: For Key
	GPIO_CFG(40, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(103, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(104, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
// End
};

static uint32_t topaz3g_tlmm_cfgs[] = {
	/* GSBI10 GSBI uart */
	GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	// yegw 2010-9-7 config gpio to GSBI function
	/* GSBI10 QUP I2C */
	GPIO_CFG(72, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(73, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	// yegw End
#ifdef CONFIG_MFD_WM8958
	/* mic detect gpio  */
	GPIO_CFG(57, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* jack detect gpio */
	GPIO_CFG(67, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* GSBI9 QUP I2C */
	GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* WM8994 LDO ENABLE */
	GPIO_CFG(66,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(108, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif
	GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(44, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
//michael - configure wlan necessary GPIOs at one shot
#if defined(CONFIG_AR6003) || defined(CONFIG_AR6003_MODULE)
	/* AR6003_GPIO_HOST_WAKE_WL */
	GPIO_CFG(AR6003_GPIO_HOST_WAKE_WL_TOPAZ_3G, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* AR6003_GPIO_WL_IRQ */
// 2011.2.28 HP Henry: remove it for h/w change		GPIO_CFG(AR6003_GPIO_WL_IRQ_TOPAZ_3G,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* AR6003_GPIO_WL_HOST_WAKE */
	GPIO_CFG(AR6003_GPIO_WL_HOST_WAKE_TOPAZ_3G,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	/* AR6003_GPIO_WLAN_RST_N */
	GPIO_CFG(AR6003_GPIO_WLAN_RST_N_TOPAZ_3G, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
#endif
	/* 5V Bias Enable */
	GPIO_CFG(102, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	/*board id pins*/
	GPIO_CFG(95, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(96, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(97, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(98, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(99, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(100, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
// HP Wade: For Key
	GPIO_CFG(40, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
// End
	//HP zhanghong:Dec 21 11:56 CST 2010, begin
	//the two gpios are for proximity sensor firmware download
	GPIO_CFG(83, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	//End
	//HP_Effie, QUP5_SPI for lcd, Start
	GPIO_CFG(49, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	//HP_Effie, QUP5_SPI for lcd, End
};

static uint32_t opal_tlmm_cfgs[] = {
	/* GSBI10 GSBI uart */
	GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	// yegw 2010-9-7 config gpio to GSBI function
	/* GSBI10 QUP I2C */
	GPIO_CFG(72, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(73, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	// yegw End
#ifdef CONFIG_MFD_WM8958
	/* mic detect gpio  */
	GPIO_CFG(57, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* jack detect gpio */
	GPIO_CFG(67, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* GSBI9 QUP I2C */
	GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	/* WM8994 LDO ENABLE */
	GPIO_CFG(66,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(108, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif
	GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(44, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
//michael - configure wlan necessary GPIOs at one shot
#if defined(CONFIG_AR6003) || defined(CONFIG_AR6003_MODULE)
	/* AR6003_GPIO_HOST_WAKE_WL */
	GPIO_CFG(AR6003_GPIO_HOST_WAKE_WL_OPAL_3G, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* AR6003_GPIO_WL_IRQ */
// 2011.2.28 HP Henry: remove it for h/w change		GPIO_CFG(AR6003_GPIO_WL_IRQ_OPAL_3G,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* AR6003_GPIO_WL_HOST_WAKE */
	GPIO_CFG(AR6003_GPIO_WL_HOST_WAKE_OPAL_3G,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	/* AR6003_GPIO_WLAN_RST_N */
	GPIO_CFG(AR6003_GPIO_WLAN_RST_N_OPAL_3G, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
#endif
	/* 5V Bias Enable */
	GPIO_CFG(102, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	/*board id pins*/
	GPIO_CFG(95, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(96, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(97, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(98, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(99, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(100, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
// HP Wade: For Key
	GPIO_CFG(40, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
// End
	//HP zhanghong:Dec 21 11:56 CST 2010, begin
	//the two gpios are for proximity sensor firmware download
	GPIO_CFG(83, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	//End	
	//HP_Effie, QUP5_SPI for lcd, Start
	GPIO_CFG(49, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	//HP_Effie, QUP5_SPI for lcd, End
};

/* HP SamLin 20110104, start for boradcom gps module 4751 uart port (gsbi 11) */
static uint32_t brcm4751_uartdm_tlmm_cfgs[] = {
	GPIO_CFG(103, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(104, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(105, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(106, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};
/* End */
static uint32_t gsbi9_i2c_cfgs[] = {
	GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(69, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};
static void __init msm8x60_init_tlmm(void)
{
	unsigned n;
	if (machine_is_msm8x60_rumi3())
		msm_gpio_install_direct_irq(0, 0);
	else if(machine_is_msm8x60_topaz() || machine_is_msm8x60_opal()){
		if (board_is_topaz_wifi()) {
			for (n = 0; n < ARRAY_SIZE(topazwifi_tlmm_cfgs); ++n)
				gpio_tlmm_config(topazwifi_tlmm_cfgs[n], 0);
		} else if (board_is_topaz_3g() || board_is_opal_3g() || board_is_opal_wifi()) {
			for (n = 0; n < ARRAY_SIZE(topaz3g_tlmm_cfgs); ++n)
				gpio_tlmm_config(topaz3g_tlmm_cfgs[n], 0);
			/* HP SamLin 20110104, start for boradcom gps module 4751 uart port (gsbi 11) */
			if (board_is_opal_3g() || board_is_opal_wifi()) {
				for (n = 0; n < ARRAY_SIZE(brcm4751_uartdm_tlmm_cfgs); ++n)
					gpio_tlmm_config(brcm4751_uartdm_tlmm_cfgs[n], 0);
			}
			/* End */
			if ((board_is_topaz_3g() && (board_type > TOPAZ3G_DVT))
				||(board_is_opal_3g() && (board_type > OPAL3G_EVT1)))
			{
				for(n =0; n < ARRAY_SIZE(gsbi9_i2c_cfgs); ++n)
					gpio_tlmm_config(gsbi9_i2c_cfgs[n], 0);
			}
		} else if (machine_is_msm8x60_opal()) {
			for (n = 0; n < ARRAY_SIZE(opal_tlmm_cfgs); ++n)
				gpio_tlmm_config(opal_tlmm_cfgs[n], 0);
		} else {
			for (n = 0; n < ARRAY_SIZE(topazwifi_tlmm_cfgs); ++n)
				gpio_tlmm_config(topazwifi_tlmm_cfgs[n], 0);
		}
	}
}

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC5_SUPPORT))
struct msm_sdcc_gpio {
	/* maximum 10 GPIOs per SDCC controller */
	s16 no;
	/* name of this GPIO */
	const char *name;
};

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct msm_sdcc_gpio sdc1_gpio_cfg[] = {
	{159, "sdc1_dat_0"},
	{160, "sdc1_dat_1"},
	{161, "sdc1_dat_2"},
	{162, "sdc1_dat_3"},
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	{163, "sdc1_dat_4"},
	{164, "sdc1_dat_5"},
	{165, "sdc1_dat_6"},
	{166, "sdc1_dat_7"},
#endif
	{167, "sdc1_clk"},
	{168, "sdc1_cmd"}
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct msm_sdcc_gpio sdc2_gpio_cfg[] = {
	{143, "sdc2_dat_0"},
	{144, "sdc2_dat_1"},
	{145, "sdc2_dat_2"},
	{146, "sdc2_dat_3"},
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{147, "sdc2_dat_4"},
	{148, "sdc2_dat_5"},
	{149, "sdc2_dat_6"},
	{150, "sdc2_dat_7"},
#endif
	{151, "sdc2_cmd"},
	{152, "sdc2_clk"}
};
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
static struct msm_sdcc_gpio sdc5_gpio_cfg[] = {
	{95, "sdc5_cmd"},
	{96, "sdc5_dat_3"},
	{97, "sdc5_clk"},
	{98, "sdc5_dat_2"},
	{99, "sdc5_dat_1"},
	{100, "sdc5_dat_0"}
};
#endif

struct msm_sdcc_pad_pull_cfg {
	enum msm_tlmm_pull_tgt pull;
	u32 pull_val;
};

struct msm_sdcc_pad_drv_cfg {
	enum msm_tlmm_hdrive_tgt drv;
	u32 drv_val;
};

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct msm_sdcc_pad_drv_cfg sdc3_pad_on_drv_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_8MA}
};

static struct msm_sdcc_pad_pull_cfg sdc3_pad_on_pull_cfg[] = {
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_sdcc_pad_drv_cfg sdc3_pad_off_drv_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_2MA}
};

static struct msm_sdcc_pad_pull_cfg sdc3_pad_off_pull_cfg[] = {
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_DOWN},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_DOWN}
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct msm_sdcc_pad_drv_cfg sdc4_pad_on_drv_cfg[] = {
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_8MA}
};

static struct msm_sdcc_pad_pull_cfg sdc4_pad_on_pull_cfg[] = {
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_sdcc_pad_drv_cfg sdc4_pad_off_drv_cfg[] = {
	//michael - align with sdc4_pad_on_pull_cfg to prevent power drop
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_8MA}
};

static struct msm_sdcc_pad_pull_cfg sdc4_pad_off_pull_cfg[] = {
	//michael - align with sdc4_pad_off_pull_cfg to prevent power drop
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_UP}
};
#endif

struct msm_sdcc_pin_cfg {
	/*
	 * = 1 if controller pins are using gpios
	 * = 0 if controller has dedicated MSM pins
	 */
	u8 is_gpio;
	u8 cfg_sts;
	u8 gpio_data_size;
	struct msm_sdcc_gpio *gpio_data;
	struct msm_sdcc_pad_drv_cfg *pad_drv_on_data;
	struct msm_sdcc_pad_drv_cfg *pad_drv_off_data;
	struct msm_sdcc_pad_pull_cfg *pad_pull_on_data;
	struct msm_sdcc_pad_pull_cfg *pad_pull_off_data;
	u8 pad_drv_data_size;
	u8 pad_pull_data_size;
};


static struct msm_sdcc_pin_cfg sdcc_pin_cfg_data[5] = {
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	[0] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc1_gpio_cfg),
		.gpio_data = sdc1_gpio_cfg
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	[1] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc2_gpio_cfg),
		.gpio_data = sdc2_gpio_cfg
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	[2] = {
		.is_gpio = 0,
		.pad_drv_on_data = sdc3_pad_on_drv_cfg,
		.pad_drv_off_data = sdc3_pad_off_drv_cfg,
		.pad_pull_on_data = sdc3_pad_on_pull_cfg,
		.pad_pull_off_data = sdc3_pad_off_pull_cfg,
		.pad_drv_data_size = ARRAY_SIZE(sdc3_pad_on_drv_cfg),
		.pad_pull_data_size = ARRAY_SIZE(sdc3_pad_on_pull_cfg)
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	[3] = {
		.is_gpio = 0,
		.pad_drv_on_data = sdc4_pad_on_drv_cfg,
		.pad_drv_off_data = sdc4_pad_off_drv_cfg,
		.pad_pull_on_data = sdc4_pad_on_pull_cfg,
		.pad_pull_off_data = sdc4_pad_off_pull_cfg,
		.pad_drv_data_size = ARRAY_SIZE(sdc4_pad_on_drv_cfg),
		.pad_pull_data_size = ARRAY_SIZE(sdc4_pad_on_pull_cfg)
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	[4] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc5_gpio_cfg),
		.gpio_data = sdc5_gpio_cfg
	}
#endif
};

static int msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct msm_sdcc_pin_cfg *curr;
	int n;

	curr = &sdcc_pin_cfg_data[dev_id - 1];
	if (!curr->gpio_data)
		goto out;

	for (n = 0; n < curr->gpio_data_size; n++) {
		if (enable) {
			rc = gpio_request(curr->gpio_data[n].no,
				curr->gpio_data[n].name);
			if (rc) {
				pr_err("%s: gpio_request(%d, %s)"
					"failed", __func__,
					curr->gpio_data[n].no,
					curr->gpio_data[n].name);
				goto free_gpios;
			}
			/* set direction as output for all GPIOs */
			rc = gpio_direction_output(
				curr->gpio_data[n].no, 1);
			if (rc) {
				pr_err("%s: gpio_direction_output"
					"(%d, 1) failed\n", __func__,
					curr->gpio_data[n].no);
				goto free_gpios;
			}
		} else {
			/*
			 * now free this GPIO which will put GPIO
			 * in low power mode and will also put GPIO
			 * in input mode
			 */
			gpio_free(curr->gpio_data[n].no);
		}
	}
	curr->cfg_sts = enable;
	goto out;

free_gpios:
	for (; n >= 0; n--)
		gpio_free(curr->gpio_data[n].no);
out:
	return rc;
}

static int msm_sdcc_setup_pad(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct msm_sdcc_pin_cfg *curr;
	int n;

	curr = &sdcc_pin_cfg_data[dev_id - 1];
	if (!curr->pad_drv_on_data || !curr->pad_pull_on_data)
		goto out;

	if (enable) {
		/*
		 * set up the normal driver strength and
		 * pull config for pads
		 */
		for (n = 0; n < curr->pad_drv_data_size; n++)
			msm_tlmm_set_hdrive(curr->pad_drv_on_data[n].drv,
				curr->pad_drv_on_data[n].drv_val);
		for (n = 0; n < curr->pad_pull_data_size; n++)
			msm_tlmm_set_pull(curr->pad_pull_on_data[n].pull,
				curr->pad_pull_on_data[n].pull_val);
	} else {
		/* set the low power config for pads */
		for (n = 0; n < curr->pad_drv_data_size; n++)
			msm_tlmm_set_hdrive(
				curr->pad_drv_off_data[n].drv,
				curr->pad_drv_off_data[n].drv_val);
		for (n = 0; n < curr->pad_pull_data_size; n++)
			msm_tlmm_set_pull(
				curr->pad_pull_off_data[n].pull,
				curr->pad_pull_off_data[n].pull_val);
	}
	curr->cfg_sts = enable;
out:
	return rc;
}

struct sdcc_reg {
	/* VDD/VCC/VCCQ regulator name on PMIC8058/PMIC8089*/
	const char *reg_name;
	/*
	 * is set voltage supported for this regulator?
	 * 0 = not supported, 1 = supported
	 */
	unsigned char set_voltage_sup;
	/* voltage level to be set */
	unsigned int level;
	/* VDD/VCC/VCCQ voltage regulator handle */
	struct regulator *reg;
};
/* all 5 SDCC controllers requires VDD/VCC voltage  */
static struct sdcc_reg sdcc_vdd_reg_data[5];
/* only SDCC1 requires VCCQ voltage */
static struct sdcc_reg sdcc_vccq_reg_data[1];

struct sdcc_reg_data {
	struct sdcc_reg *vdd_data; /* keeps VDD/VCC regulator info */
	struct sdcc_reg *vccq_data; /* keeps VCCQ regulator info */
	unsigned char sts; /* regulator enable/disable status */
};
/* msm8x60 have 5 SDCC controllers */
static struct sdcc_reg_data sdcc_vreg_data[5];

/* this init function should be called only once for each SDCC */
static int msm_sdcc_vreg_init(int dev_id, unsigned char init)
{
	int rc = 0;
	struct sdcc_reg *curr_vdd_reg;
	struct sdcc_reg *curr_vccq_reg;
	struct sdcc_reg_data *curr;

	curr = &sdcc_vreg_data[dev_id - 1];
	curr_vdd_reg = curr->vdd_data;
	curr_vccq_reg = curr->vccq_data;

	if (init) {
		/*
		 * get the regulator handle from voltage regulator framework
		 * and then try to set the voltage level for the regulator
		 */
		if (curr_vdd_reg) {
			curr_vdd_reg->reg =
				regulator_get(NULL, curr_vdd_reg->reg_name);
			if (IS_ERR(curr_vdd_reg->reg)) {
				rc = PTR_ERR(curr_vdd_reg->reg);
				pr_err("%s: regulator_get(%s) failed = %d\n",
					__func__, curr_vdd_reg->reg_name, rc);
				goto out;
			}

			if (curr_vdd_reg->set_voltage_sup) {
				rc = regulator_set_voltage(curr_vdd_reg->reg,
					curr_vdd_reg->level,
					curr_vdd_reg->level);
				if (rc) {
					pr_err("%s: regulator_set_voltage(%s)"
						" = %d\n", __func__,
						curr_vdd_reg->reg_name, rc);
					goto vdd_reg_put;
				}
			}
		}

		if (curr_vccq_reg) {
			curr_vccq_reg->reg =
				regulator_get(NULL, curr_vccq_reg->reg_name);
			if (IS_ERR(curr_vccq_reg->reg)) {
				rc = PTR_ERR(curr_vccq_reg->reg);
				pr_err("%s: regulator get of %s failed (%d)\n",
					__func__, curr_vccq_reg->reg_name, rc);
				goto vdd_reg_put;
			}
			if (curr_vccq_reg->set_voltage_sup) {
				rc = regulator_set_voltage(curr_vccq_reg->reg,
					curr_vccq_reg->level,
					curr_vccq_reg->level);
				if (rc) {
					pr_err("%s: regulator_set_voltage()"
						"= %d\n", __func__, rc);
					goto vccq_reg_put;
				}
			}
		}
	} else {
		/* deregister with voltage regulator framework */
		rc = 0;
		goto vccq_reg_put;
	}
	goto out;

vccq_reg_put:
	if (curr_vccq_reg)
		regulator_put(curr_vccq_reg->reg);
vdd_reg_put:
	if (curr_vdd_reg)
		regulator_put(curr_vdd_reg->reg);
out:
	return rc;
}

unsigned int sd_power_en;
static int msm_sdcc_setup_vreg(int dev_id, unsigned char enable)
{
	int rc = 0;
	struct sdcc_reg *curr_vdd_reg;
	struct sdcc_reg *curr_vccq_reg;
	struct sdcc_reg_data *curr;

#if 1
//michael - workaround for wifi power management
#if defined(CONFIG_AR6003) || defined(CONFIG_AR6003_MODULE)
	if (dev_id==4) {
		ar6003_wifi_power(enable);
		goto out;
	}
#endif
#else
if(dev_id == 4)
{
	goto out;
}
#endif

	curr = &sdcc_vreg_data[dev_id - 1];
	curr_vdd_reg = curr->vdd_data;
	curr_vccq_reg = curr->vccq_data;

	/* check if regulators are initialized or not? */
	if ((curr_vdd_reg && !curr_vdd_reg->reg) ||
		(curr_vccq_reg && !curr_vccq_reg->reg)) {
		/* initialize voltage regulators required for this SDCC */
		rc = msm_sdcc_vreg_init(dev_id, 1);
		if (rc) {
			pr_err("%s: regulator init failed = %d\n",
					__func__, rc);
			goto out;
		}
	}

	if (curr->sts == enable)
		goto out;

	if (enable) {
		if (curr_vdd_reg) {
			rc = regulator_enable(curr_vdd_reg->reg);
			if (rc) {
				pr_err("%s: regulator_enable(%s) failed"
					" = %d\n", __func__,
					curr_vdd_reg->reg_name, rc);
				goto out;
			}
		}
		if (curr_vccq_reg) {
			rc = regulator_enable(curr_vccq_reg->reg);
			if (rc) {
				pr_err("%s: regulator_enable(%s) failed"
					" = %d\n", __func__,
					curr_vccq_reg->reg_name, rc);
				goto vdd_reg_disable;
			}
		}
		/*
		 * now we can safely say that all required regulators
		 * are enabled for this SDCC
		 */
		if(dev_id == 3)
			{
				gpio_set_value(sd_power_en,1);			
			}
		curr->sts = enable;
	} else {
		//disable power
		if(dev_id == 3)
			{
				gpio_set_value(sd_power_en,0);				
			}
		if (curr_vdd_reg) {
			rc = regulator_disable(curr_vdd_reg->reg);
			if (rc) {
				pr_err("%s: regulator_disable(%s) = %d\n",
					__func__, curr_vdd_reg->reg_name, rc);
				goto out;
			}
		}

		if (curr_vccq_reg) {
			rc = regulator_disable(curr_vccq_reg->reg);
			if (rc) {
				pr_err("%s: regulator_disable(%s) = %d\n",
					__func__, curr_vccq_reg->reg_name, rc);
				goto out;
			}
		}
		/*
		 * now we can safely say that all required
		 * regulators are disabled for this SDCC
		 */
		curr->sts = enable;
	}
	goto out;

vdd_reg_disable:
	if (curr_vdd_reg)
		regulator_disable(curr_vdd_reg->reg);
out:
	return rc;
}

static u32 msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	u32 rc_pin_cfg = 0;
	u32 rc_vreg_cfg = 0;
	u32 rc = 0;
	struct platform_device *pdev;
	struct msm_sdcc_pin_cfg *curr_pin_cfg;

	pdev = container_of(dv, struct platform_device, dev);

	/* setup gpio/pad */
	curr_pin_cfg = &sdcc_pin_cfg_data[pdev->id - 1];
	if (curr_pin_cfg->cfg_sts == !!vdd)
		goto setup_vreg;

	if (curr_pin_cfg->is_gpio)
		rc_pin_cfg = msm_sdcc_setup_gpio(pdev->id, !!vdd);
	else
		rc_pin_cfg = msm_sdcc_setup_pad(pdev->id, !!vdd);

setup_vreg:
	/* setup voltage regulators */
	rc_vreg_cfg = msm_sdcc_setup_vreg(pdev->id, !!vdd);

	if (rc_pin_cfg || rc_vreg_cfg)
		rc = rc_pin_cfg ? rc_pin_cfg : rc_vreg_cfg;

	return rc;
}
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
#define SD_DETECT_GPIO 37
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static int msm_sdc3_get_wpswitch(struct device *dev)
{
	struct platform_device *pdev;
	int status;
	pdev = container_of(dev, struct platform_device, dev);

	status = gpio_request(GPIO_SDC_WP, "SD_WP_Switch");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n",
					__func__, GPIO_SDC_WP);
	} else {
		status = gpio_direction_input(GPIO_SDC_WP);
		if (!status) {
			status = gpio_get_value_cansleep(GPIO_SDC_WP);
			pr_info("%s: WP Status for Slot %d = %d\n",
				 __func__, pdev->id, status);
	}
		gpio_free(GPIO_SDC_WP);
	}
	return status;
}
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm8x60_sdcc_slot_status(struct device *dev)
{
	int status;

	if(board_is_opal_3g() || board_is_opal_wifi())
		{
			status = gpio_request(SD_DETECT_GPIO, "SD_HW_Detect");
			if (status) {
						pr_err("%s:Failed to request GPIO %d\n", __func__,
								SD_DETECT_GPIO);
			} else {
								status = gpio_direction_input(SD_DETECT_GPIO);
								if (!status)
									status = !(gpio_get_value_cansleep(SD_DETECT_GPIO));
								gpio_free(SD_DETECT_GPIO);
			}
			if((board_type >= OPAL3G_PROTO2))
				{
				   return (unsigned int)status;
				}
		}
	else
		{
			status = gpio_request(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1)
						, "SD_HW_Detect");
			if (status) {
						pr_err("%s:Failed to request GPIO %d\n", __func__,
								PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1));
			} else {
								status = gpio_direction_input(
										PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1));
								if (!status)
									status = !(gpio_get_value_cansleep(
										PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1)));
								gpio_free(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1));
				
					}

		}	
	return (unsigned int) !status;
}
#endif
#endif /* CONFIG_MMC_MSM_SDC3_SUPPORT */
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data msm8x60_sdc1_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data msm8x60_sdc2_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm8x60_sdc3_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch  	= msm_sdc3_get_wpswitch,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm8x60_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
					   PMIC_GPIO_SDC3_DET - 1),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm8x60_sdc4_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#if defined(CONFIG_AR6003) || defined(CONFIG_AR6003_MODULE)
	.status			= ar6003_wifi_status,
	.register_status_notify	= ar6003_wifi_status_register,
#endif
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
//michael - bug from Qualcomm, supposely this dummy52_required will pass to msm_sdcc.c
//          and to set (host->plat->dummy52_required) then able to control CMD52
#ifdef CONFIG_MMC_SDC4_DUMMY52_REQUIRED
		.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
static struct mmc_platform_data msm8x60_sdc5_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
};
#endif

static void __init msm8x60_init_mmc(void)
{
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	/* SDCC1 : eMMC card connected */
	sdcc_vreg_data[0].vdd_data = &sdcc_vdd_reg_data[0];
	sdcc_vreg_data[0].vdd_data->reg_name = "8901_l5";
	sdcc_vreg_data[0].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[0].vdd_data->level = 2850000;
	sdcc_vreg_data[0].vccq_data = &sdcc_vccq_reg_data[0];
	sdcc_vreg_data[0].vccq_data->reg_name = "8901_lvs0";
	sdcc_vreg_data[0].vccq_data->set_voltage_sup = 0;
	msm_add_sdcc(1, &msm8x60_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	/*
	 * MDM SDIO client is connected to SDC2 on charm SURF/FFA
	 * and no card is connected on 8660 SURF/FFA/FLUID.
	 */
	sdcc_vreg_data[1].vdd_data = &sdcc_vdd_reg_data[1];
	sdcc_vreg_data[1].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[1].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[1].vdd_data->level = 1800000;
	sdcc_vreg_data[1].vccq_data = NULL;
	if (machine_is_msm8x60_charm_surf())
		msm8x60_sdc2_data.msmsdcc_fmax = 24000000;
	if (machine_is_msm8x60_charm_surf() || machine_is_msm8x60_charm_ffa())
		msm_add_sdcc(2, &msm8x60_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	/* SDCC3 : External card slot connected */
	sdcc_vreg_data[2].vdd_data = &sdcc_vdd_reg_data[2];
	if(board_is_opal_3g() || board_is_opal_wifi())		
		sdcc_vreg_data[2].vdd_data->reg_name = "8058_l5";
	else
		sdcc_vreg_data[2].vdd_data->reg_name = "8901_l5";
	sdcc_vreg_data[2].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[2].vdd_data->level = 2850000;
	sdcc_vreg_data[2].vccq_data = NULL;
	if (machine_is_msm8x60_fluid())
		msm8x60_sdc3_data.wpswitch = NULL;
	if(board_is_opal_3g() || board_is_opal_wifi())
		{
			msm8x60_sdc3_data.wpswitch = NULL;
			msm8x60_sdc3_data.status_irq = gpio_to_irq(SD_DETECT_GPIO);
			if(board_type > OPAL3G_PROTO)
				{
					sd_power_en = 158;
				}
			else
				{
					sd_power_en = 82 ;
				}
			gpio_tlmm_config(GPIO_CFG(sd_power_en, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			msm_add_sdcc(3, &msm8x60_sdc3_data);
		}
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	/* SDCC4 : WLAN chip is connected */
	sdcc_vreg_data[3].vdd_data = &sdcc_vdd_reg_data[3];
	sdcc_vreg_data[3].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[3].vdd_data->set_voltage_sup = 0;
	sdcc_vreg_data[3].vdd_data->level = 1800000;
	sdcc_vreg_data[3].vccq_data = NULL;
	msm_add_sdcc(4, &msm8x60_sdc4_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	/*
	 * MDM SDIO client is connected to SDC5 on charm SURF/FFA
	 * and no card is connected on 8660 SURF/FFA/FLUID.
	 */
	sdcc_vreg_data[4].vdd_data = &sdcc_vdd_reg_data[4];
	sdcc_vreg_data[4].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[4].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[4].vdd_data->level = 1800000;
	sdcc_vreg_data[4].vccq_data = NULL;
	if (machine_is_msm8x60_charm_surf())
		msm8x60_sdc5_data.msmsdcc_fmax = 24000000;
	if (machine_is_msm8x60_charm_surf() || machine_is_msm8x60_charm_ffa())
		msm_add_sdcc(5, &msm8x60_sdc5_data);
#endif
}

#if !defined(CONFIG_GPIO_SX150X) && !defined(CONFIG_GPIO_SX150X_MODULE)
static inline void display_common_power(int on) {}
#else

static int display_power_on;
void setup_display_power(void)
{
	if (display_power_on)
		if (lcdc_vga_enabled) {
			gpio_set_value_cansleep(GPIO_LVDS_SHUTDOWN_N, 0);
			gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, 0);
			if (machine_is_msm8x60_ffa() ||
			    machine_is_msm8x60_charm_ffa())
				gpio_set_value_cansleep(GPIO_DONGLE_PWR_EN, 1);
		} else {
			gpio_set_value_cansleep(GPIO_LVDS_SHUTDOWN_N, 1);
			gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, 1);
			if (machine_is_msm8x60_ffa() ||
			    machine_is_msm8x60_charm_ffa())
				gpio_set_value_cansleep(GPIO_DONGLE_PWR_EN, 0);
		}
	else {
		if (machine_is_msm8x60_ffa() || machine_is_msm8x60_charm_ffa())
			gpio_set_value_cansleep(GPIO_DONGLE_PWR_EN, 0);
		/* BACKLIGHT */
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, 0);
		/* LVDS */
		gpio_set_value_cansleep(GPIO_LVDS_SHUTDOWN_N, 0);
	}
}

#define _GET_REGULATOR(var, name) do {					\
	if (var == NULL) {						\
		var = regulator_get(NULL, name);			\
		if (IS_ERR(var)) {					\
			pr_err("'%s' regulator not found, rc=%ld\n",	\
				name, PTR_ERR(var));			\
			var = NULL;					\
		}							\
	}								\
} while (0)
static void display_common_power(int on)
{
	int rc;
	static struct regulator *display_reg;

	if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa() ||
	    machine_is_msm8x60_charm_surf() || machine_is_msm8x60_charm_ffa()) {
		if (on) {
			/* LVDS */
			_GET_REGULATOR(display_reg, "8901_l2");
			if (!display_reg)
				return;
			rc = regulator_set_voltage(display_reg,
				3300000, 3300000);
			if (rc)
				goto out;
			rc = regulator_enable(display_reg);
			if (rc)
				goto out;
			rc = gpio_request(GPIO_LVDS_SHUTDOWN_N,
				"LVDS_STDN_OUT_N");
			if (rc) {
				printk(KERN_ERR "%s: LVDS gpio %d request"
					"failed\n", __func__,
					 GPIO_LVDS_SHUTDOWN_N);
				goto out2;
			}

			/* BACKLIGHT */
			rc = gpio_request(GPIO_BACKLIGHT_EN, "BACKLIGHT_EN");
			if (rc) {
				printk(KERN_ERR "%s: BACKLIGHT gpio %d request"
					"failed\n", __func__,
					 GPIO_BACKLIGHT_EN);
				goto out3;
			}

			if (machine_is_msm8x60_ffa() ||
			    machine_is_msm8x60_charm_ffa()) {
				rc = gpio_request(GPIO_DONGLE_PWR_EN,
						  "DONGLE_PWR_EN");
				if (rc) {
					printk(KERN_ERR "%s: DONGLE_PWR_EN gpio"
						   " %d request failed\n", __func__,
						   GPIO_DONGLE_PWR_EN);
					goto out4;
				}
			}

			gpio_direction_output(GPIO_LVDS_SHUTDOWN_N, 0);
			gpio_direction_output(GPIO_BACKLIGHT_EN, 0);
			if (machine_is_msm8x60_ffa() ||
			    machine_is_msm8x60_charm_ffa())
				gpio_direction_output(GPIO_DONGLE_PWR_EN, 0);
			mdelay(20);
			display_power_on = 1;
			setup_display_power();
		} else {
			if (display_power_on) {
				display_power_on = 0;
				setup_display_power();
				mdelay(20);
				if (machine_is_msm8x60_ffa() ||
				    machine_is_msm8x60_charm_ffa())
					gpio_free(GPIO_DONGLE_PWR_EN);
				goto out4;
			}
		}
	}
	return;

out4:
	gpio_free(GPIO_BACKLIGHT_EN);
out3:
	gpio_free(GPIO_LVDS_SHUTDOWN_N);
out2:
	regulator_disable(display_reg);
out:
	regulator_put(display_reg);
	display_reg = NULL;
}
#undef _GET_REGULATOR
#endif

#define LCDC_NUM_GPIO 28
#define LCDC_GPIO_START 0

//HP_Effie, added SAMSUNG Panel macro judgement
#ifdef CONFIG_FB_MSM_LCDC_SAMSUNG_WSVGA
static void lcdc_samsung_panel_power(int on)
{
	int n, ret = 0;

	display_common_power(on);

	for (n = 0; n < LCDC_NUM_GPIO; n++) {
		if (on) {
			ret = gpio_request(LCDC_GPIO_START + n, "LCDC_GPIO");
			if (unlikely(ret)) {
				pr_err("%s not able to get gpio\n", __func__);
				break;
			}
		} else
			gpio_free(LCDC_GPIO_START + n);
	}

	if (ret) {
		for (n--; n >= 0; n--)
			gpio_free(LCDC_GPIO_START + n);
	}
}
#endif

//HP_Effie Added for Topaz LG Panel power, Begin
#if defined(CONFIG_FB_MSM_LCDC_LG_XGA) || defined(CONFIG_FB_MSM_LCDC_HITACHI_XGA)
#define GPIO_LCD_PWR_EN  63
#define GPIO_LVDS_SHDN_N 62
extern int lcdc_gpio_request(bool on);

#define _GET_REGULATOR(var, name)   \
    do {              \
        var = regulator_get(NULL, name);            \
        if (IS_ERR(var)) {                  \
            panic("'%s' regulator not found, rc=%ld\n",    \
                name, IS_ERR(var));         \
        }                           \
    } while (0)

static struct regulator *votg_l10 = NULL;
#ifdef USE_REGULATOR_VDD5V
static struct regulator *votg_vdd5v = NULL;
#endif

static int lcdc_common_panel_power(int on)
{
	int rc;

	/* VDD_LVDS_3.3V*/
	if(!votg_l10)
		_GET_REGULATOR(votg_l10, "8058_l10");

	/* Due to hardware change, it will not use GPIO102 as 5V boost Enable since EVT1*/
	#ifdef USE_REGULATOR_VDD5V
	if (board_is_topaz_wifi() && board_type < TOPAZ_EVT1)
	{
		/* VDD_BACKLIGHT_5.0V*/
		if(!votg_vdd5v)
			_GET_REGULATOR(votg_vdd5v, "vdd50_boost");
	}
	#endif

	if (on)
	{
		/* VDD_LVDS_3.3V ENABLE*/
		rc = regulator_set_voltage(votg_l10, 3050000, 3050000);
		if(rc)
		{
			pr_err("%s: Unable to set regulator voltage:"
					" votg_l10\n", __func__);
			return rc;
		}

		rc = regulator_enable(votg_l10);
		if(rc)
		{
			pr_err("%s: Unable to enable the regulator:"
					" votg_l10\n", __func__);
			return rc;
		}

		/* Due to hardware change, it will not use GPIO102 as 5V boost Enable since EVT1*/
		#ifdef USE_REGULATOR_VDD5V
		if (board_is_topaz_wifi() && board_type < TOPAZ_EVT1)
		{
			/* VDD_BACKLIGHT_5.0V ENABLE*/
			rc = regulator_enable(votg_vdd5v);
			if(rc)
			{
				pr_err("%s: Unable to enable the regulator:"
						" votg_vdd5v\n", __func__);
				return rc;
			}
		}
		#endif

		/* LVDS_SHDN_N*/
		rc = gpio_request(GPIO_LVDS_SHDN_N,"LVDS_SHDN_N");
		if (rc)
		{
			pr_err("%s: LVDS gpio %d request"
						"failed\n", __func__,
						 GPIO_LVDS_SHDN_N);
			return rc;
		}

		/* LCD_PWR_EN */
		rc = gpio_request(GPIO_LCD_PWR_EN, "LCD_PWR_EN");
		if (rc)
		{
			pr_err("%s: LCD Power gpio %d request"
						"failed\n", __func__,
						 GPIO_LCD_PWR_EN);
			gpio_free(GPIO_LVDS_SHDN_N);
			return rc;
		}

		/* BACKLIGHT */
		rc = gpio_request(GPIO_BACKLIGHT_EN, "BACKLIGHT_EN");
		if (rc)
		{
			pr_err("%s: BACKLIGHT gpio %d request"
						"failed\n", __func__,
						 GPIO_BACKLIGHT_EN);
			gpio_free(GPIO_LVDS_SHDN_N);
			gpio_free(GPIO_LCD_PWR_EN);
			return rc;
		}

		gpio_set_value_cansleep(GPIO_LVDS_SHDN_N, 1);
		gpio_set_value_cansleep(GPIO_LCD_PWR_EN, 1);
		mdelay(2);
		// enable backlight later
		delay_bl_power_up = true;
	}
	else
	{
		rc = regulator_disable(votg_l10);
		if(rc)
		{
			pr_err("%s: Unable to disable votg_l10\n",__func__);
			return rc;
		}

		/* Due to hardware change, it will not use GPIO102 as 5V boost Enable since EVT1*/
		#ifdef USE_REGULATOR_VDD5V
		if (board_is_topaz_wifi() && board_type < TOPAZ_EVT1)
		{
			rc = regulator_disable(votg_vdd5v);
			if(rc)
			{
				pr_err("%s: Unable to disable votg_vdd5v\n",__func__);
				return rc;
			}
		}
		#endif

		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, 0);
		mdelay(5);
		gpio_set_value_cansleep(GPIO_LVDS_SHDN_N, 0);
		gpio_set_value_cansleep(GPIO_LCD_PWR_EN, 0);
		mdelay(20);
		gpio_free(GPIO_BACKLIGHT_EN);
		gpio_free(GPIO_LVDS_SHDN_N);
		gpio_free(GPIO_LCD_PWR_EN);
	}

	lcdc_gpio_request(on);
	return 0;
}
#undef _GET_REGULATOR
#endif
//HP_Effie, Added for Topaz LG Panel power, End

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
#define _GET_REGULATOR(var, name) do {				\
		var = regulator_get(NULL, name);			\
		if (IS_ERR(var)) {					\
			pr_err("'%s' regulator not found, rc=%ld\n",	\
				name, IS_ERR(var));			\
			var = NULL;					\
			return -ENODEV;					\
		}							\
	} while (0)

static int hdmi_enable_5v(int on)
{
	static struct regulator *reg_8901_hdmi_mvs;	/* HDMI_5V */
	static struct regulator *reg_8901_mpp0;		/* External 5V */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8901_hdmi_mvs)
		_GET_REGULATOR(reg_8901_hdmi_mvs, "8901_hdmi_mvs");
	if (!reg_8901_mpp0)
		_GET_REGULATOR(reg_8901_mpp0, "8901_mpp0");

	if (on) {
		rc = regulator_enable(reg_8901_mpp0);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"reg_8901_mpp0", rc);
			return rc;
		}
		rc = regulator_enable(reg_8901_hdmi_mvs);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8901_hdmi_mvs", rc);
			return rc;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8901_hdmi_mvs);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8901_hdmi_mvs", rc);
		rc = regulator_disable(reg_8901_mpp0);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"reg_8901_mpp0", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
}

static int hdmi_core_power(int on)
{
	static struct regulator *reg_8058_l16;		/* VDD_HDMI */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8058_l16)
		_GET_REGULATOR(reg_8058_l16, "8058_l16");

	if (on) {
		rc = regulator_set_voltage(reg_8058_l16, 1800000, 1800000);
		if (!rc)
			rc = regulator_enable(reg_8058_l16);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8058_l16", rc);
			return rc;
		}
		rc = gpio_request(170, "HDMI_DDC_CLK");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_DDC_CLK", 170, rc);
			goto error1;
		}
		rc = gpio_request(171, "HDMI_DDC_DATA");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_DDC_DATA", 171, rc);
			goto error2;
		}
		rc = gpio_request(172, "HDMI_HPD");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_HPD", 172, rc);
			goto error3;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		gpio_free(170);
		gpio_free(171);
		gpio_free(172);
		rc = regulator_disable(reg_8058_l16);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8058_l16", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;

error3:
	gpio_free(171);
error2:
	gpio_free(170);
error1:
	regulator_disable(reg_8058_l16);
	return rc;
}

static int hdmi_cec_power(int on)
{
	static struct regulator *reg_8901_l3;		/* HDMI_CEC */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8901_l3)
		_GET_REGULATOR(reg_8901_l3, "8901_l3");

	if (on) {
		rc = regulator_set_voltage(reg_8901_l3, 3300000, 3300000);
		if (!rc)
			rc = regulator_enable(reg_8901_l3);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8901_l3", rc);
			return rc;
		}
		rc = gpio_request(169, "HDMI_CEC_VAR");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_CEC_VAR", 169, rc);
			goto error;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		gpio_free(169);
		rc = regulator_disable(reg_8901_l3);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8901_l3", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
error:
	regulator_disable(reg_8901_l3);
	return rc;
}

#undef _GET_REGULATOR

#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

static int lcdc_panel_power(int on)
{
	int rc=0;
	int flag_on = !!on;
	static int lcdc_power_save_on = 0;

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

#ifdef CONFIG_FB_MSM_LCDC_SAMSUNG_WSVGA
	lcdc_samsung_panel_power(on);
#endif

//HP_Effie,Added for Topaz LG Panel Power, Begin
#if defined(CONFIG_FB_MSM_LCDC_LG_XGA) || defined(CONFIG_FB_MSM_LCDC_HITACHI_XGA)
	rc = lcdc_common_panel_power(on);
#endif
//HP_Effie,Added for Topaz LG Panel Power, End
	return rc;
}
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors mdp_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_MMSS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_APPSS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_smi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_MMSS_SLAVE_SMI,
		.ab = 147460000,
		.ib = 184325000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_APPSS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_ebi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_MMSS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_APPSS_SLAVE_EBI_CH0,
		.ab = 334080000,
		.ib = 417600000,
	},
};
static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_MMSS_SLAVE_SMI,
		.ab = 175110000,
		.ib = 218887500,
	},
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_APPSS_SLAVE_EBI_CH0,
		.ab = 175110000,
		.ib = 218887500,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_MMSS_SLAVE_SMI,
		.ab = 230400000,
		.ib = 288000000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_APPSS_SLAVE_EBI_CH0,
		.ab = 230400000,
		.ib = 288000000,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_MMSS_SLAVE_SMI,
		.ab = 334080000,
		.ib = 417600000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_APPSS_SLAVE_EBI_CH0,
		.ab = 334080000,
		.ib = 417600000,
	},
};
static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_smi_vectors),
		mdp_sd_smi_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_ebi_vectors),
		mdp_sd_ebi_vectors,
	},
	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};
static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
};

#endif
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_MMSS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_APPSS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};
static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_MMSS_SLAVE_SMI,
		.ab = 435456000,
		.ib = 544320000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_APPSS_SLAVE_EBI_CH0,
		.ab = 435456000,
		.ib = 544320000,
	},
};
static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
};
#endif


static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save   = lcdc_panel_power,
};

#ifdef CONFIG_FB_MSM_TVOUT
static struct regulator *reg_8058_l13;

static int atv_dac_power(int on)
{
	int rc = 0;
	#define _GET_REGULATOR(var, name) do {				\
		var = regulator_get(NULL, name);			\
		if (IS_ERR(var)) {					\
			pr_info("'%s' regulator not found, rc=%ld\n",	\
				name, IS_ERR(var));			\
			var = NULL;					\
			return -ENODEV;					\
		}							\
	} while (0)

	if (!reg_8058_l13)
		_GET_REGULATOR(reg_8058_l13, "8058_l13");
	#undef _GET_REGULATOR

	if (on) {
		rc = regulator_set_voltage(reg_8058_l13, 2050000, 2050000);
		if (rc) {
			pr_info("%s: '%s' regulator set voltage failed,\
				rc=%d\n", __func__, "8058_l13", rc);
			return rc;
		}

		rc = regulator_enable(reg_8058_l13);
		if (rc) {
			pr_err("%s: '%s' regulator enable failed,\
				rc=%d\n", __func__, "8058_l13", rc);
			return rc;
		}
	} else {
		rc = regulator_force_disable(reg_8058_l13);
		if (rc)
			pr_warning("%s: '%s' regulator disable failed, rc=%d\n",
				__func__, "8058_l13", rc);
	}
	return rc;

}
#endif

int mdp_core_clk_rate_table[] = {
	59080000,
	128000000,
	160000000,
	200000000,
};
static struct msm_panel_common_pdata mdp_pdata = {
	.mdp_core_clk_rate = 200000000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
};

#ifdef CONFIG_FB_MSM_TVOUT

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors atv_bus_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_MMSS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_APPSS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};
static struct msm_bus_vectors atv_bus_def_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_MMSS_SLAVE_SMI,
		.ab = 236390400,
		.ib = 265939200,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MMSS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_APPSS_SLAVE_EBI_CH0,
		.ab = 236390400,
		.ib = 265939200,
	},
};
static struct msm_bus_paths atv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(atv_bus_init_vectors),
		atv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(atv_bus_def_vectors),
		atv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata atv_bus_scale_pdata = {
	atv_bus_scale_usecases,
	ARRAY_SIZE(atv_bus_scale_usecases),
};
#endif

static struct tvenc_platform_data atv_pdata = {
	.poll		 = 0,
	.pm_vid_en	 = atv_dac_power,
#ifdef CONFIG_MSM_BUS_SCALING
	.bus_scale_table = &atv_bus_scale_pdata,
#endif
};
#endif

static void __init msm_fb_add_devices(void)
{
	if (machine_is_msm8x60_rumi3())
		msm_fb_register_device("mdp", NULL);
	else
		msm_fb_register_device("mdp", &mdp_pdata);

	msm_fb_register_device("lcdc", &lcdc_pdata);
	msm_fb_register_device("mipi_dsi", 0);
#ifdef CONFIG_MSM_BUS_SCALING
	msm_fb_register_device("dtv", &dtv_pdata);
#endif
#ifdef CONFIG_FB_MSM_TVOUT
	msm_fb_register_device("tvenc", &atv_pdata);
	msm_fb_register_device("tvout_device", NULL);
#endif
}

#if (defined(CONFIG_BAHAMA_CORE)) && \
	(defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
static int bahama_bt(int on)
{
	int rc;
	int i;
	struct bahama config = { .mod_id = BAHAMA_SLAVE_ID_BAHAMA };

	struct bahama_config_register {
		u8 reg;
		u8 value;
		u8 mask;
	};

	struct bahama_variant_register {
		const size_t size;
		const struct bahama_config_register *set;
	};

	const struct bahama_config_register *p;

	u8 version;

	const struct bahama_config_register v10_bt_on[] = {
		{ 0xE9, 0x00, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xE4, 0x00, 0xFF },
		{ 0xE5, 0x00, 0x0F },
#ifdef CONFIG_WLAN
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
		{ 0x01, 0x0C, 0x1F },
		{ 0x01, 0x08, 0x1F },
	};

	const struct bahama_config_register v10_bt_off[] = {
		{ 0xE9, 0x00, 0xFF },
	};

	const struct bahama_config_register v20_bt_on[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v20_bt_off[] = {
		{ 0xE9, 0x00, 0xFF },
	};

	const struct bahama_variant_register bt_bahama[2][2] = {
		{
			{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
			{ ARRAY_SIZE(v20_bt_off), v20_bt_off },
		},
		{
			{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
			{ ARRAY_SIZE(v20_bt_on), v20_bt_on },
		}
	};

	on = on ? 1 : 0;

	rc = bahama_read_bit_mask(&config, 0x00,  &version, 1, 0x1F);
	if (rc < 0) {
		dev_err(&msm_bt_power_device.dev,
			"%s: version read failed: %d\n",
			__func__, rc);
		return rc;
	}
	switch (version) {
	case 0x00:
	case 0x08:
	case 0X10:
		version = 0x00;
		break;
	case 0x09:
		version = 0x01;
		break;
	default:
		version = 0xFF;
		dev_err(&msm_bt_power_device.dev,
		 "%s: unsupported version\n", __func__);
		break;
	}

	if ((version >= ARRAY_SIZE(bt_bahama[on])) ||
		(bt_bahama[on][version].size == 0)) {
		dev_err(&msm_bt_power_device.dev,
			"%s: unsupported version\n",
			__func__);
		return -EIO;
	}

	p = bt_bahama[on][version].set;

	dev_info(&msm_bt_power_device.dev,
		"%s: found version %d\n", __func__, version);

	for (i = 0; i < bt_bahama[on][version].size; i++) {
		u8 value = (p+i)->value;
		rc = bahama_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			dev_err(&msm_bt_power_device.dev,
				"%s: reg %d write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		dev_dbg(&msm_bt_power_device.dev,
			"%s: reg 0x%02x write value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	return 0;
}

static const struct {
	char *name;
	int vmin;
	int vmax;
} bt_regs_info[] = {
	{ "8058_s3", 1800000, 1800000 },
	{ "8058_l2", 1800000, 1800000 },
	{ "8058_l8", 2900000, 2900000 },
};

static struct regulator *bt_regs[ARRAY_SIZE(bt_regs_info)];

static int bluetooth_use_regulators(int on)
{
	int i, recover = -1, rc = 0;

	for (i = 0; i < ARRAY_SIZE(bt_regs_info); i++) {
		bt_regs[i] = on ? regulator_get(&msm_bt_power_device.dev,
						bt_regs_info[i].name) :
				(regulator_put(bt_regs[i]), NULL);
		if (IS_ERR(bt_regs[i])) {
			rc = PTR_ERR(bt_regs[i]);
			dev_err(&msm_bt_power_device.dev,
				"regulator %s get failed (%d)\n",
				bt_regs_info[i].name, rc);
			recover = i - 1;
			bt_regs[i] = NULL;
			break;
		}

		if (!on)
			continue;

		rc = regulator_set_voltage(bt_regs[i],
					  bt_regs_info[i].vmin,
					  bt_regs_info[i].vmax);
		if (rc < 0) {
			dev_err(&msm_bt_power_device.dev,
				"regulator %s voltage set (%d)\n",
				bt_regs_info[i].name, rc);
			recover = i;
			break;
		}
	}

	if (on && (recover > -1))
		for (i = recover; i >= 0; i--) {
			regulator_put(bt_regs[i]);
			bt_regs[i] = NULL;
		}

	return rc;
}

static int bluetooth_switch_regulators(int on)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(bt_regs_info); i++) {
		rc = on ? regulator_enable(bt_regs[i]) :
			  regulator_disable(bt_regs[i]);
		if (rc < 0) {
			dev_err(&msm_bt_power_device.dev,
				"regulator %s %s failed (%d)\n",
				bt_regs_info[i].name,
				on ? "enable" : "disable", rc);
			if (on && (i > 0)) {
				while (--i)
					regulator_disable(bt_regs[i]);
				break;
			}
			break;
		}
	}
	return rc;
}

static struct msm_xo_voter *bt_clock;

static int bluetooth_power(int on)
{
	int rc = 0;

	if (on) {

		rc = bluetooth_use_regulators(1);
		if (rc < 0)
			goto out;

		rc = bluetooth_switch_regulators(1);

		if (rc < 0)
			goto fail_put;

		bt_clock = msm_xo_get(MSM_XO_TCXO_D0, "bt_power");

		if (IS_ERR(bt_clock)) {
			pr_err("Couldn't get TCXO_D0 voter\n");
			goto fail_switch;
		}

		rc = msm_xo_mode_vote(bt_clock, MSM_XO_MODE_ON);

		if (rc < 0) {
			pr_err("Failed to vote for TCXO_DO ON\n");
			goto fail_vote;
		}

		rc = bahama_bt(1);

		if (rc < 0)
			goto fail_clock;

		rc = msm_xo_mode_vote(bt_clock, MSM_XO_MODE_PIN_CTRL);

		if (rc < 0) {
			pr_err("Failed to vote for TCXO_DO pin control\n");
			goto fail_vote;
		}
	} else {
		/* check for initial RFKILL block (power off) */
		/* some RFKILL versions/configurations rfkill_register */
		/* calls here for an initial set_block */
		/* avoid calling i2c and regulator before unblock (on) */
		if (platform_get_drvdata(&msm_bt_power_device) == NULL) {
			dev_info(&msm_bt_power_device.dev,
				"%s: initialized OFF/blocked\n", __func__);
			goto out;
		}

		bahama_bt(0);

fail_clock:
		msm_xo_mode_vote(bt_clock, MSM_XO_MODE_OFF);
fail_vote:
		msm_xo_put(bt_clock);
fail_switch:
		bluetooth_switch_regulators(0);
fail_put:
		bluetooth_use_regulators(0);
	}

out:
	if (rc < 0)
		on = 0;
	dev_info(&msm_bt_power_device.dev,
		"Bluetooth power switch: state %d result %d\n", on, rc);

	return rc;
}

#endif /* CONFIG_BAHAMA_CORE, CONFIG_MSM_BT_POWER, CONFIG_MSM_BT_POWER_MODULE */

//wanqin
#ifdef CONFIG_BT

#define UARTDM_BT_TX		53
#define UARTDM_BT_RX		54
#define UARTDM_BT_CTS_N		55
#define UARTDM_BT_RFR_N		56

#define BT_PCM_DOUT		111
#define BT_PCM_IN		112
#define BT_PCM_SYNC		113
#define BT_PCM_CLK		114

/***** BT UART *****/
static uint32_t bt_uart_config_power_off[] = {
	GPIO_CFG(UARTDM_BT_TX, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),  
	GPIO_CFG(UARTDM_BT_RX, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), 
	GPIO_CFG(UARTDM_BT_CTS_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(UARTDM_BT_RFR_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

static uint32_t bt_uart_config_power_on[] = {
	GPIO_CFG(UARTDM_BT_TX, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(UARTDM_BT_RX, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(UARTDM_BT_CTS_N, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(UARTDM_BT_RFR_N, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};
/***** BT PCM *****/  //currently we don't use pcm interface
static uint32_t bt_pcm_config_power_off[] = {
	GPIO_CFG(BT_PCM_DOUT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(BT_PCM_IN, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(BT_PCM_SYNC, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(BT_PCM_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};
static uint32_t bt_pcm_config_power_on[] = {
	GPIO_CFG(BT_PCM_DOUT, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(BT_PCM_IN, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(BT_PCM_SYNC, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(BT_PCM_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

/***** BT Topaz *****/
static uint32_t topaz_bt_config_power_off[] = {
	GPIO_CFG(138, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* BT_RST_N */
	GPIO_CFG(130, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* BT_POWER */
	GPIO_CFG(131,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* BT_WAKE */
	GPIO_CFG(129, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)    /* BT_HOST_WAKE */
};

static uint32_t topaz_bt_config_power_on[] = {
	GPIO_CFG(138, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* BT_RST_N */
	GPIO_CFG(130, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* BT_POWER */
	GPIO_CFG(131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* BT_WAKE */
	GPIO_CFG(129, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)   /* BT_HOST_WAKE */
};

/***** BT Topaz 3G and opel 3G *****/
static uint32_t bt_config_power_off[] = {
	GPIO_CFG(122, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* BT_RST_N */
	GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* BT_POWER */
	GPIO_CFG(131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* BT_WAKE */
	//defined later     /* BT_HOST_WAKE */
};

static uint32_t bt_config_power_on[] = {
	GPIO_CFG(122, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 	/* BT_RST_N */
	GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* BT_POWER */
	GPIO_CFG(131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 	/* BT_WAKE */
	//defined later     /* BT_HOST_WAKE */
};

static int bt_gpio_config(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: bt_gpio_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}

	return rc;
}

static int bluetooth_csr_power(int on)
{
	int rc;

	printk("Board-msm8x60.c bluetooth_csr_power [%d]\n", on);
	if(on){
		//BT on
		rc = bt_gpio_config(bt_uart_config_power_on, ARRAY_SIZE(bt_uart_config_power_on));
		if (rc < 0) {
			printk("bt on uart config fail rc=[%d]\n", rc);
			return rc;
		}

		rc = bt_gpio_config(bt_pcm_config_power_on, ARRAY_SIZE(bt_pcm_config_power_on));
		if (rc < 0) {
			printk("bt on pcm config fail rc=[%d]\n", rc);
			return rc;
		}
		if (board_is_topaz_wifi()) {
			rc = bt_gpio_config(topaz_bt_config_power_on, ARRAY_SIZE(topaz_bt_config_power_on));
			if(rc < 0) {
				printk("Board-msm8x60.c bluetooth_csr_power rc=[%d]\n", rc);
				return rc;
			}

			gpio_set_value(130, 1);		/* BT_POWER */
			mdelay(1);
			gpio_set_value(138, 0);		/* BT_RST_N */
			mdelay(8);
			gpio_set_value(138, 1);
		}else {
			rc = bt_gpio_config(bt_config_power_on, ARRAY_SIZE(bt_config_power_on));
			if (rc < 0) {
				printk("Board-msm8x60.c bluetooth_csr_power rc=[%d]\n", rc);
				return rc;
			}

			if (board_is_topaz_3g() && (board_type == TOPAZ3G_PROTO)){
				printk("topaz 3g proto");
                                //this is not correct pin as BT_HOST_WAKE
				rc = gpio_tlmm_config(GPIO_CFG(82, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  GPIO_CFG_ENABLE);/* BT_HOST_WAKE */
				if (rc < 0) {
					printk("Board-msm8x60.c bluetooth_csr_power rc=[%d]\n", rc);
					return rc;
				}
			}
			else {
				printk("topaz evt & opel");
				rc = gpio_tlmm_config(GPIO_CFG(50, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  GPIO_CFG_ENABLE);/* BT_HOST_WAKE */
				if (rc < 0) {
					printk("Board-msm8x60.c bluetooth_csr_power rc=[%d]\n", rc);
					return rc;
				}
			}

			gpio_set_value(110, 1);        /* BT_POWER */
			mdelay(1);
			gpio_set_value(122, 0);        /* BT_RST_N */
			mdelay(8);
			gpio_set_value(122, 1);
		}

		printk("BT on\n");
		return 0;
	}
	else {
		//BT off
		gpio_set_value(131, 0);     /* BT_WAKE */
		if (board_is_topaz_wifi()) {
			gpio_set_value(138, 1);      /* BT_RST_N */
			gpio_set_value(130, 0);			 /* BT_POWER */

			rc = bt_gpio_config(topaz_bt_config_power_off, ARRAY_SIZE(topaz_bt_config_power_off));
			if(rc < 0) {
				printk("Board-msm8x60.c bluetooth_csr_power rc=[%d]\n", rc);
				return rc;
			}
		} else {
			gpio_set_value(122, 1);     /* BT_RST_N */
			gpio_set_value(110, 0);     /* BT_POWER */

			rc = bt_gpio_config(bt_config_power_off, ARRAY_SIZE(bt_config_power_off));
			if(rc < 0) {
				printk("Board-msm8x60.c bluetooth_csr_power rc=[%d]\n", rc);
				return rc;
			}

			if (board_is_topaz_3g() && (board_type == TOPAZ3G_PROTO)) {
				printk("topaz 3g proto");
				rc = gpio_tlmm_config(GPIO_CFG(82, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  GPIO_CFG_ENABLE);/* BT_HOST_WAKE */
				if (rc < 0) {
					printk("Board-msm8x60.c bluetooth_csr_power rc=[%d]\n", rc);
					return rc;
				}
			}
			else {
				printk("topaz 3g evt & opel");
				rc = gpio_tlmm_config(GPIO_CFG(50, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  GPIO_CFG_ENABLE);/* BT_HOST_WAKE */
				if (rc < 0) {
					printk("Board-msm8x60.c bluetooth_csr_power rc=[%d]\n", rc);
					return rc;
				}
			}
		}

		gpio_set_value(BT_PCM_DOUT, 0);
		gpio_set_value(BT_PCM_SYNC, 0);
		gpio_set_value(BT_PCM_CLK, 0);

		rc = bt_gpio_config(bt_pcm_config_power_off, ARRAY_SIZE(bt_pcm_config_power_off));
		if (rc < 0) {
			printk("bt off pcm config fail rc=[%d]\n", rc);
			return rc;
		}

		gpio_set_value(UARTDM_BT_TX, 1);
		gpio_set_value(UARTDM_BT_RFR_N, 1);
		rc = bt_gpio_config(bt_uart_config_power_off, ARRAY_SIZE(bt_uart_config_power_off));
		if (rc < 0) {
			printk("bt off uart config fail rc=[%d]\n", rc);
			return rc;
		}

		printk("BT off\n");

		return 0;
	}
}

static void __init bt_power_init(void)
{
		if(bluetooth_csr_power(0) < 0)
			return;

		printk("Board-msm8x60.c Bluetooth Power init successfully\n");
}

#endif

#if defined(CONFIG_BT) && defined(CONFIG_BT_MSM_SLEEP)


static int bluesleep_get_host_wake_io(void)
{
   if (board_is_topaz_wifi())
      return 129;
   else {
		if (board_is_topaz_3g() && (board_type == TOPAZ3G_PROTO)) //wanqin remove bt power control for topaz 3g proto
			return 82;
		else 
			return 50;
	} 
}

static int bluesleep_get_ext_wake_io(void)
{
   return 131;
}

static int bluesleep_get_host_wake_irq(void)
{
   if (board_is_topaz_wifi())
      return MSM_GPIO_TO_INT(129);
   else {
		if (board_is_topaz_3g() && (board_type == TOPAZ3G_PROTO))
		{
   			return 82;
   		}
		else
			return MSM_GPIO_TO_INT(50); 
	}
}

static void bluesleep_set_io(int io, int io_status)
{
  gpio_set_value(io, io_status);
}

static int bluesleep_get_io(int io)
{
  return gpio_get_value(io);
}
#endif

static void __init msm8x60_cfg_smsc911x(void)
{
	smsc911x_resources[1].start =
		PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 6);
	smsc911x_resources[1].end =
		PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 6);
}

#ifdef CONFIG_MSM_RPM
static struct msm_rpm_platform_data msm_rpm_data = {
	.reg_base_addrs = {
		[MSM_RPM_PAGE_STATUS] = MSM_RPM_BASE,
		[MSM_RPM_PAGE_CTRL] = MSM_RPM_BASE + 0x400,
		[MSM_RPM_PAGE_REQ] = MSM_RPM_BASE + 0x600,
		[MSM_RPM_PAGE_ACK] = MSM_RPM_BASE + 0xa00,
	},

	.irq_ack = RPM_SCSS_CPU0_GP_HIGH_IRQ,
	.irq_err = RPM_SCSS_CPU0_GP_LOW_IRQ,
	.irq_vmpm = RPM_SCSS_CPU0_GP_MEDIUM_IRQ,
};
#endif

#define DBG_UART_RX_GPIO 117
#define DBG_UART_TX_GPIO 118
#define DBG_UART_SEL_GPIO 58
static void console_uart_ds_init(void)
{
	gpio_request(DBG_UART_RX_GPIO, "DBG_UART_RX");
	gpio_request(DBG_UART_TX_GPIO, "DBG_UART_TX");
	gpio_request(DBG_UART_SEL_GPIO, "DBG_UART_SEL");
}

static int __init boardtype_setup(char *boardtype_str)
{
	board_is_topaz_wifi_flag = false;
	board_is_topaz_3g_flag = false;
	board_is_opal_3g_flag = false;
	board_is_opal_wifi_flag = false;
	if (!strcmp(boardtype_str, "topaz-1stbuild-Wifi")) {
		board_type = TOPAZ_PROTO;
		board_is_topaz_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-2ndbuild-Wifi")) {
		board_type = TOPAZ_PROTO2;
		board_is_topaz_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-3rdbuild-Wifi")) {
		board_type = TOPAZ_EVT1;
		board_is_topaz_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-4thbuild-Wifi")) {
		board_type = TOPAZ_EVT2;
		board_is_topaz_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-5thbuild-Wifi")) {
		board_type = TOPAZ_EVT3;
		board_is_topaz_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-6thbuild-Wifi")) {
		board_type = TOPAZ_DVT;
		board_is_topaz_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-7thbuild-Wifi")) {
		board_type = TOPAZ_PVT;
		board_is_topaz_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-pvt-Wifi")) {
		board_type = TOPAZ_PVT;
		board_is_topaz_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-1stbuild-3G")) {
		board_type = TOPAZ3G_PROTO;
		board_is_topaz_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-2ndbuild-3G")) {
		board_type = TOPAZ3G_EVT1;
		board_is_topaz_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-3rdbuild-3G")) {
		board_type = TOPAZ3G_EVT2;
		board_is_topaz_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-4thbuild-3G")) {
		board_type = TOPAZ3G_EVT3;
		board_is_topaz_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-5thbuild-3G")) {
		board_type = TOPAZ3G_DVT;
		board_is_topaz_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-6thbuild-3G")) {
		board_type = TOPAZ3G_PVT;
		board_is_topaz_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-7thbuild-3G")) {
		board_type = TOPAZ3G_PVT;
		board_is_topaz_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "topaz-pvt-3G")) {
		board_type = TOPAZ3G_PVT;
		board_is_topaz_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-1stbuild-Wifi")) {
		board_type = OPAL_PROTO;
		board_is_opal_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-2ndbuild-Wifi")) {
		board_type = OPAL_PROTO2;
		board_is_opal_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-3rdbuild-Wifi")) {
		board_type = OPAL_EVT1;
		board_is_opal_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-4thbuild-Wifi")) {
		board_type = OPAL_EVT2;
		board_is_opal_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-5thbuild-Wifi")) {
		board_type = OPAL_EVT3;
		board_is_opal_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-6thbuild-Wifi")) {
		board_type = OPAL_DVT;
		board_is_opal_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-7thbuild-Wifi")) {
		board_type = OPAL_PVT;
		board_is_opal_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-pvt-Wifi")) {
		board_type = OPAL_PVT;
		board_is_opal_wifi_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-1stbuild-3G")) {
		board_type = OPAL3G_PROTO;
		board_is_opal_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-2ndbuild-3G")) {
		board_type = OPAL3G_PROTO2;
		board_is_opal_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-3rdbuild-3G")) {
		board_type = OPAL3G_EVT1;
		board_is_opal_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-4thbuild-3G")) {
		board_type = OPAL3G_EVT2;
		board_is_opal_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-5thbuild-3G")) {
		board_type = OPAL3G_EVT3;
		board_is_opal_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-6thbuild-3G")) {
		board_type = OPAL3G_DVT;
		board_is_opal_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-7thbuild-3G")) {
		board_type = OPAL3G_PVT;
		board_is_opal_3g_flag	= true;
	} else if (!strcmp(boardtype_str, "opal-pvt-3G")) {
		board_type = OPAL3G_PVT;
		board_is_opal_3g_flag	= true;
	} else {
		board_type = TOPAZ_PROTO;
		board_is_topaz_wifi_flag = true;
	}
	printk("%s(%d) : str = %s\n", __func__, __LINE__, boardtype_str);

	return 0;
}

__setup("boardtype=", boardtype_setup);


#if TOPAZ_CLOCK_FIXUP
extern void msm_clock_fixup(struct clk *clock_tbl, unsigned num_clocks,
                uint32_t *fixup_clk_ids, uint32_t fixup_clk_num);
uint32_t fixup_clk_ids[] = {
        L_PIXEL_MDP_CLK,
        L_PIXEL_LCDC_CLK,
    };
uint32_t fixup_clk_num = ARRAY_SIZE(fixup_clk_ids);
#endif

struct msm_board_data {
	struct msm_gpiomux_configs *gpiomux_cfgs;
};

static struct msm_board_data msm8x60_rumi3_board_data __initdata = {
	.gpiomux_cfgs = msm8x60_surf_ffa_gpiomux_cfgs,
};

static struct msm_board_data msm8x60_sim_board_data __initdata = {
	.gpiomux_cfgs = msm8x60_surf_ffa_gpiomux_cfgs,
};

static struct msm_board_data msm8x60_surf_board_data __initdata = {
	.gpiomux_cfgs = msm8x60_surf_ffa_gpiomux_cfgs,
};

static struct msm_board_data msm8x60_ffa_board_data __initdata = {
	.gpiomux_cfgs = msm8x60_surf_ffa_gpiomux_cfgs,
};

static struct msm_board_data msm8x60_fluid_board_data __initdata = {
	.gpiomux_cfgs = msm8x60_fluid_gpiomux_cfgs,
};

static struct msm_board_data msm8x60_charm_surf_board_data __initdata = {
	.gpiomux_cfgs = msm8x60_charm_gpiomux_cfgs,
};

static struct msm_board_data msm8x60_charm_ffa_board_data __initdata = {
	.gpiomux_cfgs = msm8x60_charm_gpiomux_cfgs,
};

static struct msm_board_data msm8x60_topaz_board_data __initdata = {
	.gpiomux_cfgs = msm8x60_topaz_gpiomux_cfgs,
};

static void __init msm8x60_init(struct msm_board_data *board_data)
{
	/* HP Wade: Customize rpm_vreg_init_pdata early based on out platform*/
	if(board_is_topaz_wifi() || board_is_topaz_3g() || board_is_opal_3g() || board_is_opal_wifi()){
		int i;
		for (i = 0; i < ARRAY_SIZE(rpm_vreg_init_customized_pdata__topaz); i++)
		{
			memcpy ( &rpm_vreg_init_pdata[rpm_vreg_init_customized_pdata__topaz[i].id], &(rpm_vreg_init_customized_pdata__topaz[i].rpm_vreg_init_pdata), sizeof(struct rpm_vreg_pdata));
		}
	}
	// End

	/*
	 * Initialize RPM first as other drivers and devices may need
	 * it for their initialization.
	 */
#ifdef CONFIG_MSM_RPM
	BUG_ON(msm_rpm_init(&msm_rpm_data));
#endif

	printk("%s: board_type: %d; board_is_topaz_wifi_flag: %d; board_is_topaz_3g_flag: %d\n", __func__, board_type, board_is_topaz_wifi_flag, board_is_topaz_3g_flag);

	if (msm_xo_init())
		pr_err("Failed to initialize XO votes\n");

	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
			   __func__);
	msm8x60_check_2d_hardware();

	/*
	 * Initialize SPM before acpuclock as the latter calls into SPM
	 * driver to set ACPU voltages.
	 */
	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 1)
		msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	else
		msm_spm_init(msm_spm_data_v1, ARRAY_SIZE(msm_spm_data_v1));

	/*
	 * Disable regulator info printing so that regulator registration
	 * messages do not enter the kmsg log.
	 */
	regulator_suppress_info_printing();

	/* Initialize regulators needed for clock_init. */
	platform_add_devices(early_regulators, ARRAY_SIZE(early_regulators));

	msm_clock_init(msm_clocks_8x60, msm_num_clocks_8x60);

#if TOPAZ_CLOCK_FIXUP
	msm_clock_fixup(msm_clocks_8x60, msm_num_clocks_8x60, fixup_clk_ids, fixup_clk_num);
#endif

	/* Buses need to be initialized before early-device registration
	 * to get the platform data for fabrics.
	 */
	msm8x60_init_buses();
	platform_add_devices(early_devices, ARRAY_SIZE(early_devices));
	/* CPU frequency control is not supported on simulated targets. */
	if (!machine_is_msm8x60_rumi3() && !machine_is_msm8x60_sim())
		msm_acpu_clock_init(&msm8x60_acpu_clock_data);

	if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa() ||
		machine_is_msm8x60_fluid() || machine_is_msm8x60_topaz() || machine_is_msm8x60_opal())
		/* No EBI2 on 8660 charm targets */
		msm8x60_init_ebi2();
	msm8x60_init_tlmm();
	msm8x60_init_gpiomux(board_data->gpiomux_cfgs);

	if (machine_is_msm8x60_topaz() || machine_is_msm8x60_opal()) 
		boardid_init();

#ifdef CONFIG_CHARGER_MAX8903
	if (board_is_topaz_wifi()) {
		max8903_charger_device.dev.platform_data = &max8903_charger_pdata_topaz;
		if (board_type > TOPAZ_EVT1) {
			max8903_charger_pdata_topaz.USUS_pin_polarity = 1;
		}
	} else {
		if (board_is_opal_3g() || board_is_opal_wifi() || (board_is_topaz_3g() && (board_type >= TOPAZ3G_EVT1)) ) {
			max8903_charger_pdata.USUS_pin_polarity = 1;
		}
		max8903_charger_device.dev.platform_data = &max8903_charger_pdata;
	}
#endif

	if(board_is_topaz_wifi()){
		msm_gpio_keys.dev.platform_data = &topaz_wifi_gpio_keys_data;
	}else if(board_is_opal_wifi() || board_is_opal_3g() || board_is_topaz_3g()){
		msm_gpio_keys.dev.platform_data = &topaz_3g_gpio_keys_data;
	}
	else
		msm_gpio_keys.dev.platform_data = &topaz_wifi_gpio_keys_data;

	if(!machine_is_msm8x60_topaz() && !machine_is_msm8x60_opal()) {
		msm8x60_init_uart12dm();
	}
	msm8x60_init_mmc();

	// yegw power on touchscreen: workaround because need more time to init cypress touch, will remove this code after cypress touch SWD issue fixed
	if (machine_is_msm8x60_topaz() || machine_is_msm8x60_opal()) 
		init_touch_hw();

#if defined(CONFIG_PMIC8058_OTHC) || defined(CONFIG_PMIC8058_OTHC_MODULE)
	msm8x60_init_pm8058_othc();
#endif

	if (machine_is_msm8x60_fluid()) {
		pm8058_platform_data.sub_devices[PM8058_SUBDEV_KPD].
			platform_data = &fluid_keypad_data;
		pm8058_platform_data.sub_devices[PM8058_SUBDEV_KPD].data_size
			= sizeof(fluid_keypad_data);
	} else if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa()) {
		pm8058_platform_data.sub_devices[PM8058_SUBDEV_KPD].
			platform_data = &ffa_keypad_data;
		pm8058_platform_data.sub_devices[PM8058_SUBDEV_KPD].data_size
			= sizeof(ffa_keypad_data);
	}

	if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa() ||
	    machine_is_msm8x60_fluid() || machine_is_msm8x60_charm_surf() ||
	    machine_is_msm8x60_charm_ffa()) {
		msm8x60_cfg_smsc911x();
		if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 1)
			platform_add_devices(msm_footswitch_devices,
					     msm_num_footswitch_devices);
		platform_add_devices(surf_devices,
					 ARRAY_SIZE(surf_devices));
#ifdef CONFIG_USB_EHCI_MSM
		msm_add_host(0, &msm_usb_host_pdata);
#endif
	}
	else if (machine_is_msm8x60_topaz()) {
		if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 1)
			platform_add_devices(msm_footswitch_devices,
					     msm_num_footswitch_devices);
		platform_add_devices(topaz_devices,
					ARRAY_SIZE(topaz_devices));
#ifdef CONFIG_USB_EHCI_MSM
		msm_add_host(0, &msm_usb_host_pdata_topaz);
#endif
	}
	else if (machine_is_msm8x60_opal()) {
		if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 1)
			platform_add_devices(msm_footswitch_devices,
					     msm_num_footswitch_devices);
		platform_add_devices(opal_devices,
					ARRAY_SIZE(opal_devices));
#ifdef CONFIG_USB_EHCI_MSM
		msm_add_host(0, &msm_usb_host_pdata_topaz);
#endif
	} else {
		msm8x60_configure_smc91x();
		platform_add_devices(rumi_sim_devices,
					 ARRAY_SIZE(rumi_sim_devices));
	}

	if (machine_is_msm8x60_topaz() || machine_is_msm8x60_opal())
	{
		register_lcdc_panel();
	}

#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)
	if (machine_is_msm8x60_surf() || machine_is_msm8x60_ffa())
		msm8x60_cfg_isp1763();
	if( board_is_topaz_3g() || board_is_opal_3g())
	{
		msm8x60_cfg_isp1763();
		isp1763_modem_gpio_init();
		platform_device_register(&isp1763_device);
	}
#endif
	if (!machine_is_msm8x60_sim())
		msm_fb_add_devices();
	fixup_i2c_configs();
	register_i2c_devices();
//HP_Effie, QUP_SPI support , Start   
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)    
	register_spi_devices();
#endif
//HP_Effie, QUP_SPI support , End
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_cpuidle_set_states(msm_cstates, ARRAY_SIZE(msm_cstates),
				msm_pm_data);

#ifdef CONFIG_MSM8X60_AUDIO
	msm_snddev_init();
#endif
#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)
	if (machine_is_msm8x60_fluid())
		platform_device_register(&fluid_leds_gpio);
	else
		platform_device_register(&gpio_leds);
#endif

	if (!machine_is_msm8x60_topaz() && !machine_is_msm8x60_opal()) {
	/* configure pmic leds */
	if (machine_is_msm8x60_fluid()) {
		pm8058_platform_data.sub_devices[PM8058_SUBDEV_LED].
			platform_data = &pm8058_fluid_flash_leds_data;
		pm8058_platform_data.sub_devices[PM8058_SUBDEV_LED].data_size
			= sizeof(pm8058_fluid_flash_leds_data);
	} else {
		pm8058_platform_data.sub_devices[PM8058_SUBDEV_LED].
			platform_data = &pm8058_flash_leds_data;
		pm8058_platform_data.sub_devices[PM8058_SUBDEV_LED].data_size
			= sizeof(pm8058_flash_leds_data);
	}
	}

#ifdef CONFIG_BT
	if (machine_is_msm8x60_topaz() || machine_is_msm8x60_opal()) {
		bt_power_init();
	}
#endif

/* HP SamLin 20110104, start for boradcom gps module 4751 uart port (gsbi 11)*/
#ifdef CONFIG_BRCM4751
	if ( board_is_topaz_wifi() || board_is_opal_3g()) {
		platform_device_register(&msm_device_uart_dm4);
		platform_device_register(&msm_device_brcm4751);
	}
#endif
/* End */

#ifdef CONFIG_LEDS_LM8502
	lm8502_init();
#endif
#ifdef CONFIG_A6
	init_a6();
#endif
	if (machine_is_msm8x60_topaz() || machine_is_msm8x60_opal()) {
		console_uart_ds_init();
	}
}

static void __init msm8x60_rumi3_init(void)
{
	msm8x60_init(&msm8x60_rumi3_board_data);
}

static void __init msm8x60_sim_init(void)
{
	msm8x60_init(&msm8x60_sim_board_data);
}

static void __init msm8x60_surf_init(void)
{
	msm8x60_init(&msm8x60_surf_board_data);
}

static void __init msm8x60_ffa_init(void)
{
	msm8x60_init(&msm8x60_ffa_board_data);
}

static void __init msm8x60_fluid_init(void)
{
	msm8x60_init(&msm8x60_fluid_board_data);
}

static void __init msm8x60_charm_surf_init(void)
{
	msm8x60_init(&msm8x60_charm_surf_board_data);
}

static void __init msm8x60_charm_ffa_init(void)
{
	msm8x60_init(&msm8x60_charm_ffa_board_data);
}

static void __init msm8x60_topaz_init(void)
{
	msm8x60_init(&msm8x60_topaz_board_data);
}

static void __init msm8x60_opal_init(void)
{
	msm8x60_init(&msm8x60_topaz_board_data);
}

MACHINE_START(MSM8X60_RUMI3, "QCT MSM8X60 RUMI3")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_rumi3_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_SIM, "QCT MSM8X60 SIMULATOR")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_sim_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_SURF, "QCT MSM8X60 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_surf_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_FFA, "QCT MSM8X60 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_ffa_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_FLUID, "QCT MSM8X60 FLUID")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_fluid_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_CHARM_SURF, "QCT MSM8X60 CHARM SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_charm_surf_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_CHARM_FFA, "QCT MSM8X60 CHARM FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_charm_ffa_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_TOPAZ, "HP MSM8X60 TOPAZ")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_topaz_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_OPAL, "HP MSM8X60 OPAL")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_opal_init,
	.timer = &msm_timer,
MACHINE_END
