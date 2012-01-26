/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/delay.h>
#include <linux/pwm.h>
#include <linux/i2c.h>
#ifdef CONFIG_PMIC8058_PWM
#include <linux/mfd/pmic8058.h>
#include <linux/pmic8058-pwm.h>
#endif
#include <mach/gpio.h>
#include "msm_fb.h"



#ifdef CONFIG_PMIC8058_PWM
static struct pwm_device *bl_pwm0;

/* for LG panel 300hz was the minimum freq where flickering wasnt
 * observed as the screen was dimmed
 */

#define PWM_FREQ_HZ 300
#define PWM_PERIOD_USEC (USEC_PER_SEC / PWM_FREQ_HZ)
#define PWM_LEVEL 15
#define PWM_DUTY_LEVEL (PWM_PERIOD_USEC / PWM_LEVEL)
#endif

struct lcm_data {
	struct i2c_client *clientp;
};

static int lp097x02_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id);
static int __exit lp097x02_i2c_remove(struct i2c_client *client);

static const struct i2c_device_id lp097x02_i2c_id[] = {
	{"lcdxpanel", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cy8c_ts_id);

static struct i2c_driver lp097x02_i2c_driver = {
	.driver = {
		.name = "lcdxpanel",
		.owner = THIS_MODULE,
	},
	.probe      = lp097x02_i2c_probe,
 	.remove     = __exit_p(lp097x02_i2c_remove),
	.id_table   = lp097x02_i2c_id,

};

static struct msm_panel_common_pdata *lcdc_lg_pdata;


static int lcdc_lg_panel_on(struct platform_device *pdev)
{
	return 0;
}

static int lcdc_lg_panel_off(struct platform_device *pdev)
{
	return 0;
}


static void lcdc_lg_panel_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level;
	int ret;

	bl_level = mfd->bl_level;

	if(lcdc_lg_pdata && lcdc_lg_pdata->pmic_backlight)
		lcdc_lg_pdata->pmic_backlight(bl_level);

#ifdef CONFIG_PMIC8058_PWM
	if (bl_pwm0) 
    {
		ret = pwm_config(bl_pwm0, PWM_DUTY_LEVEL * bl_level,
			PWM_PERIOD_USEC);
		if (ret)
			printk(KERN_ERR "pwm_config on pwm 0 failed %d\n", ret);
        
        ret = pwm_enable(bl_pwm0);
		if (ret)
			printk(KERN_ERR "pwm_enable on pwm 0 failed %d\n", ret);
	}
#endif

}

static int __init lcdc_lg_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		lcdc_lg_pdata = pdev->dev.platform_data;
		return 0;
	}

#ifdef CONFIG_PMIC8058_PWM
    if(lcdc_lg_pdata == NULL)
        return 0;
    
	bl_pwm0 = pwm_request(lcdc_lg_pdata->gpio_num[0], "backlight");
	if (bl_pwm0 == NULL || IS_ERR(bl_pwm0)) 
    {
		pr_err("%s pwm_request() failed\n", __func__);
		bl_pwm0 = NULL;
	}

	printk(KERN_INFO "Lcdc_lg_probe: bl_pwm0=%p LPG_chan0=%d ",
			bl_pwm0, (int)lcdc_lg_pdata->gpio_num[0]
			);
#endif

	msm_fb_add_device(pdev);

	/* register EDID i2c interface driver */
	i2c_add_driver(&lp097x02_i2c_driver);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = lcdc_lg_probe,
	.driver = {
		.name   = "lcdc_lg_xga",
	},
};

static struct msm_fb_panel_data lg_panel_data = {    
 	.on = lcdc_lg_panel_on,
 	.off = lcdc_lg_panel_off,   
	.set_backlight = lcdc_lg_panel_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_lg_xga",
	.id	= 1,
	.dev	= {
		.platform_data = &lg_panel_data,
	}
};

struct lp097x02_register_s{
	char *name;
	u8	addr;
	u8	reg;
};

#define LP097X02_REGISTER(n, a) \
	{ .name = n, .addr = a }

static struct lp097x02_register_s lp097x02_registers[] = {
	LP097X02_REGISTER("name1", 			0x08),
	LP097X02_REGISTER("name2", 			0x09),
	LP097X02_REGISTER("producet_code",	0x0a),
	LP097X02_REGISTER("hex", 			0x0b),
	LP097X02_REGISTER("sn1", 			0x0c),
	LP097X02_REGISTER("sn2", 			0x0d),
	LP097X02_REGISTER("sn3", 			0x0e),
	LP097X02_REGISTER("sn4", 			0x0f),
	LP097X02_REGISTER("week", 			0x10),
	LP097X02_REGISTER("year", 			0x11),
	LP097X02_REGISTER("edid_ver", 		0x12),
	LP097X02_REGISTER("edid_rever", 	0x13),

	LP097X02_REGISTER("pclk_lsb", 		0x36),
	LP097X02_REGISTER("pclk_msb",		0x37),
	LP097X02_REGISTER("hactive_l", 		0x38),
	LP097X02_REGISTER("hblanking_l", 	0x39),
	LP097X02_REGISTER("hactive_h",		0x3A),
	LP097X02_REGISTER("vactive", 		0x3B),
	LP097X02_REGISTER("vblanking", 		0x3C),
	LP097X02_REGISTER("vactive_h",		0x3D),
	LP097X02_REGISTER("hsync_off",		0x3E),
	LP097X02_REGISTER("hsync_pw",		0x3F),
	LP097X02_REGISTER("vsync_off_pw",	0x40),
	LP097X02_REGISTER("hvsync_off_w",	0x41),
	LP097X02_REGISTER("himage_size_mm",	0x42),
	LP097X02_REGISTER("vimage_size_mm",	0x43),
	LP097X02_REGISTER("hvimage_size",	0x44),
	LP097X02_REGISTER("hborder",		0x45),
	LP097X02_REGISTER("vborder",		0x46),
	LP097X02_REGISTER("hvsync_neg",		0x47),
	LP097X02_REGISTER("pclk_lsb2",		0x48),
	LP097X02_REGISTER("pclk_msb2",		0x49),
	LP097X02_REGISTER("hactive_l2",		0x4a),
	LP097X02_REGISTER("hblanking_l2",	0x4B),
	LP097X02_REGISTER("hactive_h2",		0x4C),
	LP097X02_REGISTER("vactive2",		0x4D),
	LP097X02_REGISTER("vblanking2",		0x4E),
	LP097X02_REGISTER("vactive_h2",		0x4F),
	LP097X02_REGISTER("hsync_off2",		0x50),
	LP097X02_REGISTER("hsync_pw2",		0x51),
	LP097X02_REGISTER("vsync_off_pw2",	0x52),
	LP097X02_REGISTER("hvsync_off_w2",	0x53),
	LP097X02_REGISTER("himage_size_mm2",0x54),
	LP097X02_REGISTER("vimage_size_mm2",0x55),
	LP097X02_REGISTER("hvimage_size2",	0x56),
	LP097X02_REGISTER("hborder2",		0x57),
	LP097X02_REGISTER("vborder2",		0x58),
	LP097X02_REGISTER("hvsync_neg2",	0x59),
	LP097X02_REGISTER("check_sum",		0x7F),
};

static int lp097x02_i2c_init(struct lcm_data *lcm_data)
{
	int i, rc;
	u8 data;
	struct lcm_data *ld = lcm_data;

	for(i = 0; i < ARRAY_SIZE(lp097x02_registers); i++)
	{
		struct i2c_msg msgs[] = {
			[0] = {
				.addr   = ld->clientp->addr,
				.flags  = 0,
				.buf    = (void *)&lp097x02_registers[i].addr,
				.len    = 1,
			},
			[1] = {
				.addr   = ld->clientp->addr,
				.flags  = I2C_M_RD,
				.buf    = (void *)&data,
				.len    = 1,
			}
		};

		rc = i2c_transfer(ld->clientp->adapter, msgs, 2);
		if (rc < 0)
		{
			pr_err("%s: i2c_transfer() failed(%d\n)", __func__, rc);
		}
		else
		{
			lp097x02_registers[i].reg = data;
			pr_info("%s: [%s][0x%02x]: 0x%02x\n", __func__, lp097x02_registers[i].name, lp097x02_registers[i].addr, lp097x02_registers[i].reg);
		}
	
	}

	return 0;
}

static int lp097x02_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct lcm_data *ld;
	int ret;

	ld = (struct lcm_data *)kzalloc(sizeof(struct lcm_data), GFP_KERNEL);
	if (NULL == ld)
	{
		pr_err("%s: failed to alloc lcm_data!\n", __func__);
		ret = -ENOMEM;
		goto error;
	}

	ld->clientp = client;
	i2c_set_clientdata(client, ld);

	ret = lp097x02_i2c_init(ld);

error:
	return ret;
}

static int __exit lp097x02_i2c_remove(struct i2c_client *client)
{
	struct lcm_data *ld = i2c_get_clientdata(client);

	kfree(ld);
	return 0;
}

static int __init lcdc_lg_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_lg_xga"))
		return 0;
#endif

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &lg_panel_data.panel_info;
	pinfo->xres = 1024;
	pinfo->yres = 768;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 96000000; // 100030000; 100.03MHz over MSM8660 Rev1 Spec, the clock will bounce back and fourth between96MHZ and 1009.714MHZ
	pinfo->bl_max = 15;
	pinfo->bl_min = 1;

        pinfo->lcdc.h_back_porch = 400;
        pinfo->lcdc.h_front_porch = 272;
        pinfo->lcdc.h_pulse_width = 328;
        pinfo->lcdc.v_back_porch = 6;
        pinfo->lcdc.v_front_porch = 10;
        pinfo->lcdc.v_pulse_width = 7;
        pinfo->lcdc.border_clr = 0;
        pinfo->lcdc.underflow_clr = 0xff;
        pinfo->lcdc.hsync_skew = 0;

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lcdc_lg_panel_init);
