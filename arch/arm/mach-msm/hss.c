/* arch/arm/mach-msm/hss.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/switch.h>
#include <linux/interrupt.h>
#include <asm/mach-types.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include "hss.h"
#include <linux/mfd/msm-adie-codec.h>
#include <linux/mfd/wm8994/registers.h>

#define DRIVER_NAME	"topaz-headset"

enum {
	NO_DEVICE	= 0,
	MSM_HEADSET	= 1,
	MSM_HEADSET_NOMIC = 3,
};

struct msm_handset {
	struct input_dev *ipdev;
	struct switch_dev sdev;
	struct msm_headset_platform_data *hs_pdata;
};

#define DETECTION_CHANGE  1
#define HOOKKEY_CHANGE   2
#define REG_MICD1 0xD0
#define REG_MICD3 0xD2

static struct msm_handset *hs;
static int hs_status;
static int hs_status_change;
static int hk_status_change;
static int hs_jackstatus_change;
//static DEFINE_SPINLOCK(jd_lock);
//static DEFINE_SPINLOCK(hk_lock);
static struct delayed_work hs_work;
static struct delayed_work hk_work;

static void
report_headset_switch(struct input_dev *dev, int key, int value)
{
	struct msm_handset *hs = input_get_drvdata(dev);

	input_report_switch(dev, key, value);
	if(SW_HEADPHONE_INSERT==key)
	{switch_set_state(&hs->sdev, value);}
	else if(SW_HEADPHONE_NO_MIC_INSERT==key)
	{switch_set_state(&hs->sdev, (value?((value)|0x2):value));}
}


/*
//     hs_status
//  value           status
//    1            headset in
//    0            headset out
*/
static void hs_worker( struct work_struct *work)
{
	int value;
	if(DETECTION_CHANGE == hs_status_change){
		value = gpio_get_value(hs->hs_pdata->gpio_detection_irq);
		if(value != hs_status){
			switch (value){
			case 0:
				wm8958_reg_write(REG_MICD1, 0x5900);
				report_headset_switch(hs->ipdev,SW_HEADPHONE_INSERT,0);
				report_headset_switch(hs->ipdev,SW_HEADPHONE_NO_MIC_INSERT,0);
				pr_err(" headset plug out \n");
				break;
			case 1:
				wm8958_reg_write(REG_MICD1, 0x5903);
				break;
			default:break;
			}
			hs_jackstatus_change = 1;
			hs_status = value;  //update hs_status
		}
		hs_status_change = 0;
	}
}


static void hk_worker( struct work_struct *work)
{
	int rc=0;
	if((HOOKKEY_CHANGE == hk_status_change)&&(1==hs_status)){
		rc=wm8958_reg_read(REG_MICD3);
		if ((rc&0x3)!=0x03)
		{
			pr_err("wm8958 mic detect failed\n");
			return;
		}
		if(hs_jackstatus_change)
		{
			if(0x7==rc)  // < 2ohm
			{
				pr_err(" headset no mic insert\n");
				report_headset_switch(hs->ipdev,SW_HEADPHONE_NO_MIC_INSERT,1);
				hs_jackstatus_change = 0;
			}else   // 0x203 <-> 47.6ohm we are using previosly hai+ 2011/2/17
			{
				pr_err(" headset mic insert\n");
				report_headset_switch(hs->ipdev,SW_HEADPHONE_INSERT,1);
				hs_jackstatus_change = 0;
			}

		}
		else
		{
			if(0x7==rc)
			input_report_key(hs->ipdev,KEY_MEDIA,1);
			if(0x203==rc)
			input_report_key(hs->ipdev,KEY_MEDIA,0);
			pr_err(" press/release key =0x%x\n",rc);
		}
		hk_status_change = 0;
	}
}


static irqreturn_t jackdetection_interrupt(int irq, void *dev_id)
{
	//unsigned long flags;
	int value;
	//spin_lock_irqsave(&jd_lock, flags);
	value = gpio_get_value(hs->hs_pdata->gpio_detection_irq);
	hs_status_change = DETECTION_CHANGE;
	schedule_delayed_work(&hs_work,msecs_to_jiffies(5));
	//spin_unlock_irqrestore(&jd_lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t hookkey_interrupt(int irq, void *dev_id)
{
	//unsigned long flags;
	//spin_lock_irqsave(&hk_lock, flags);
	hk_status_change = HOOKKEY_CHANGE;
	schedule_delayed_work(&hk_work,msecs_to_jiffies(500)); // adjust time for Mic Detect Level Measurement hai+ 2010/2/17
	//spin_unlock_irqrestore(&hk_lock, flags);
	return IRQ_HANDLED;
}

static ssize_t msm_headset_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs->sdev)) {
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case MSM_HEADSET:
	case MSM_HEADSET_NOMIC:
		return sprintf(buf, "Headset\n");
	}
	return -EINVAL;
}


static int __devinit hs_probe(struct platform_device *pdev)
{
	int rc = 0;
	int irq1,irq2;
	struct input_dev *ipdev;

	hs = kzalloc(sizeof(struct msm_handset), GFP_KERNEL);
	if (!hs)
		return -ENOMEM;

	hs->sdev.name	= "h2w";
	hs->sdev.print_name = msm_headset_print_name;

	rc = switch_dev_register(&hs->sdev);
	if (rc)
		goto err_switch_dev_register;

	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, hs);

	hs->ipdev = ipdev;

	if (pdev->dev.platform_data)
		hs->hs_pdata = pdev->dev.platform_data;

	if (hs->hs_pdata->hs_name)
		ipdev->name = hs->hs_pdata->hs_name;
	else
		ipdev->name	= DRIVER_NAME;

	ipdev->id.vendor	= 0x0001;
	ipdev->id.product	= 1;
	ipdev->id.version	= 1;

	input_set_capability(ipdev, EV_KEY, KEY_MEDIA);
	input_set_capability(ipdev, EV_SW, SW_HEADPHONE_INSERT);
	input_set_capability(ipdev, EV_SW, SW_HEADPHONE_NO_MIC_INSERT);

	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, hs);

	// pre read headset status
	if (gpio_get_value(hs->hs_pdata->gpio_func_sel))
	{
		dev_err(&ipdev->dev,
				"hs_probe: swtich to debug mode\n");
		rc = -1;
		goto fail_irq;
	}

	INIT_DELAYED_WORK(&hs_work,hs_worker);
	INIT_DELAYED_WORK(&hk_work,hk_worker);

	hs_status = gpio_get_value(hs->hs_pdata->gpio_detection_irq);
	pr_err("%s:  hs->hs_pdata->hs_name = %s , gpio_detection_irq = %d ,  gpio_hookkey_irq = %d ,hs_status = %d \n", __func__, \
			 hs->hs_pdata->hs_name,
			 hs->hs_pdata->gpio_detection_irq,
			 hs->hs_pdata->gpio_hookkey_irq,
			 hs_status	);

	rc = gpio_get_value(hs->hs_pdata->gpio_hookkey_irq);

	rc = wm8958_reg_read(WM8994_AIF1_CLOCKING_1);
	if( 0==rc )
	{wm8958_reg_write(WM8994_AIF1_CLOCKING_1, 1);}

	// pre get headset status
	if(hs_status)
	{
		wm8958_reg_write(REG_MICD1, 0x5903);
		mdelay(100);
		rc=wm8958_reg_read(REG_MICD3);
		if ((rc&0x3)!=0x03)
		{
			pr_err("wm8958 mic detect failed\n");
		}
		if(0x7==rc)
		{
			report_headset_switch(hs->ipdev,SW_HEADPHONE_NO_MIC_INSERT,1);
		}else{
			report_headset_switch(hs->ipdev,SW_HEADPHONE_INSERT,1);
		}
	}


	rc = gpio_tlmm_config(GPIO_CFG(hs->hs_pdata->gpio_detection_irq, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 0);
	rc = gpio_tlmm_config(GPIO_CFG(hs->hs_pdata->gpio_hookkey_irq, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 0);

	rc = gpio_request( hs->hs_pdata->gpio_detection_irq, "jack_detection");
	if (rc) {
		pr_err("%s: unable to request gpio %d\n", __func__, hs->hs_pdata->gpio_detection_irq);
		goto fail_irq;
	}
	rc = gpio_request( hs->hs_pdata->gpio_hookkey_irq, "hookkey_detection");
	if (rc) {
		pr_err("%s: unable to request gpio %d\n", __func__, hs->hs_pdata->gpio_hookkey_irq);
		goto fail_irq;
	}

	rc = gpio_direction_input(hs->hs_pdata->gpio_detection_irq);
	if (rc) {
		pr_err("%s: unable to gpio_direction_input gpio_detection_irq %d\n",__func__, hs->hs_pdata->gpio_detection_irq);
		goto fail_irq;
	}
	rc = gpio_direction_input(hs->hs_pdata->gpio_hookkey_irq);
	if (rc) {
		pr_err("%s: unable to gpio_direction_input gpio_hookkey_irq %d\n",__func__, hs->hs_pdata->gpio_hookkey_irq);
		goto fail_irq;
	}

	irq1 = gpio_to_irq(hs->hs_pdata->gpio_detection_irq);
	if (irq1 < 0) {
		pr_err("gpio-keys: Unable to get irq number"
			  " for GPIO %d, error %d\n",
			  hs->hs_pdata->gpio_detection_irq, irq1);
		gpio_free(hs->hs_pdata->gpio_detection_irq);
		goto fail_irq;
	}

	irq2 = gpio_to_irq(hs->hs_pdata->gpio_hookkey_irq);
	if (irq2 < 0) {
		pr_err("gpio-keys: Unable to get irq number"
			  " for GPIO %d, error %d\n",
			  hs->hs_pdata->gpio_hookkey_irq, irq2);
		gpio_free(hs->hs_pdata->gpio_hookkey_irq);
		goto fail_irq;
	}

	rc = request_irq(irq1, &jackdetection_interrupt,IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING , "Jack-Detection", 0);
	if (rc) {
		pr_err("%s: unable to request irq %d\n", __func__, hs->hs_pdata->gpio_detection_irq);
		goto fail_irq;
	}

	rc = request_irq(irq2, &hookkey_interrupt,IRQF_TRIGGER_HIGH, "Hookkey-Detection", 0);
	if (rc) {
		pr_err("%s: unable to request irq %d\n", __func__, hs->hs_pdata->gpio_hookkey_irq);
		goto fail_irq;
	}

	return 0;

fail_irq:
	input_unregister_device(ipdev);
	ipdev = NULL;
err_reg_input_dev:
	input_free_device(ipdev);
err_alloc_input_dev:
	switch_dev_unregister(&hs->sdev);
err_switch_dev_register:
	kfree(hs);
	hs = NULL;
	return rc;
}

static int __devexit hs_remove(struct platform_device *pdev)
{
	struct msm_handset *hs = platform_get_drvdata(pdev);
	input_unregister_device(hs->ipdev);
	switch_dev_unregister(&hs->sdev);
	kfree(hs);
	return 0;
}

#ifdef CONFIG_PM
static int hs_suspend(struct platform_device *dev,pm_message_t state)
{
	pr_err("hs_suspend\n");
	if (hs)
	{
		enable_irq_wake(gpio_to_irq(hs->hs_pdata->gpio_detection_irq));
		enable_irq_wake(gpio_to_irq(hs->hs_pdata->gpio_hookkey_irq));
	}
	return 0;
}

static int hs_resume(struct platform_device *dev)
{
	int rc = 0;
	pr_err("hs_resume\n");
	if (!hs)
	{
		return 0;
	}
	disable_irq_wake(gpio_to_irq(hs->hs_pdata->gpio_detection_irq));
	disable_irq_wake(gpio_to_irq(hs->hs_pdata->gpio_hookkey_irq));
	 // pre read headset status
	if (gpio_get_value(hs->hs_pdata->gpio_func_sel))
	{
		pr_err("it is debug mode\n");
		return 0;
	}
	hs_status = gpio_get_value(hs->hs_pdata->gpio_detection_irq);
	rc = wm8958_reg_read(WM8994_AIF1_CLOCKING_1);
	if( 0==rc )
	{wm8958_reg_write(WM8994_AIF1_CLOCKING_1, 1);}
	// pre get headset status
	if(hs_status)
	{
		wm8958_reg_write(REG_MICD1, 0x5903);
		mdelay(100);
		rc=wm8958_reg_read(REG_MICD3);
		if ((rc&0x3)!=0x03)
		{
			pr_err("wm8958 mic detect failed\n");
		}
		if(0x7==rc)
		{
			report_headset_switch(hs->ipdev,SW_HEADPHONE_NO_MIC_INSERT,1);
		}else{
			report_headset_switch(hs->ipdev,SW_HEADPHONE_INSERT,1);
		}
	}
	return 0;
}
#else
#define hs_suspend NULL
#define hs_resume  NULL
#endif /* CONFIG_PM */
static struct platform_driver hs_driver = {
	.probe		= hs_probe,
	.remove		= __devexit_p(hs_remove),
	.suspend        = hs_suspend,
	.resume         = hs_resume,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init hs_init(void)
{
	return platform_driver_register(&hs_driver);
}
late_initcall(hs_init);

static void __exit hs_exit(void)
{
	platform_driver_unregister(&hs_driver);
}
module_exit(hs_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:topaz-headset");
