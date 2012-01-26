/*
 * linux/drivers/exmdm/exmdms-steu300.c
 *
 * Copyright 2010 HP Ltd.
 *
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include "exmdms.h"

struct steu300_exmdm_data {
	struct exmdm_classdev	cdev;
	int gpio_wireless_disable;
	bool gpio_wireless_disable_active_low;
	bool registered;
};

static void 
steu300_exmdm_wireless_disable_set(struct exmdm_classdev *cdev, 	int state);
static int 
steu300_exmdm_wireless_disable_get(struct exmdm_classdev *cdev);

static struct steu300_exmdm_data steu300_exmdm = {
		.cdev		= {
			.name		= "steu300",
			.wireless_disable_set = steu300_exmdm_wireless_disable_set,
			.wireless_disable_get = steu300_exmdm_wireless_disable_get,
			.wireless_disable_state = EXMDM_WIRELESS_DISABLE_OFF
		},
		.gpio_wireless_disable = 171,
		.gpio_wireless_disable_active_low = true,
		.registered = false
	};

static void steu300_exmdm_wireless_disable_set(struct exmdm_classdev *cdev,
				int state)
{
  int value;
  struct steu300_exmdm_data *pexmdm = container_of(cdev, struct steu300_exmdm_data, cdev);

  if (pexmdm->gpio_wireless_disable_active_low)
    value = (EXMDM_WIRELESS_DISABLE_OFF == state) ? 1 : 0;
  else
    value = (EXMDM_WIRELESS_DISABLE_OFF == state) ? 0 : 1;

  printk("####samdebug:%s: set wireless disable state as %d, write %d to gpio register\n", __func__, state, value);
  
  gpio_set_value_cansleep(pexmdm->gpio_wireless_disable, value);	
}

static int steu300_exmdm_wireless_disable_get(struct exmdm_classdev *cdev)
{
  int state, value;
  struct steu300_exmdm_data *pexmdm = container_of(cdev, struct steu300_exmdm_data, cdev);

  value = gpio_get_value_cansleep(pexmdm->gpio_wireless_disable);

  if (pexmdm->gpio_wireless_disable_active_low)
    state = value ? EXMDM_WIRELESS_DISABLE_OFF : EXMDM_WIRELESS_DISABLE_ON;
  else 
    state = value ? EXMDM_WIRELESS_DISABLE_ON : EXMDM_WIRELESS_DISABLE_OFF;

  printk("####samdebug:%s: get wireless disable state as %d, read %d from gpio register\n", __func__, state, value);
  
  return state;
}

static int steu300_exmdm_probe(struct platform_device *pdev)
{
	int ret;
	//int value;

  printk("####samdebug:%s\n", __func__);

  ret = gpio_request(steu300_exmdm.gpio_wireless_disable, "steu300_wireless_disable");
  if (ret < 0)
    return ret;

  /*
  if (steu300_exmdm.gpio_wireless_disable_active_low)
    value = EXMDM_WIRELESS_DISABLE_OFF == steu300_exmdm.cdev.wireless_disable_state ? 1 : 0;
  else
    value = EXMDM_WIRELESS_DISABLE_OFF == steu300_exmdm.cdev.wireless_disable_state ? 0 : 1;
  
  ret = gpio_direction_output(steu300_exmdm.gpio_wireless_disable, value);
  if (ret < 0) {
    gpio_free(steu300_exmdm.gpio_wireless_disable);
    return ret;
  }
  */
    
	ret = exmdm_classdev_register(&pdev->dev, &steu300_exmdm.cdev);
 	if (ret < 0) {
	  gpio_free(steu300_exmdm.gpio_wireless_disable);
	  exmdm_classdev_unregister(&steu300_exmdm.cdev);
  }

  return ret;
}

static int steu300_exmdm_remove(struct platform_device *pdev)
{
	exmdm_classdev_unregister(&steu300_exmdm.cdev);
  gpio_free(steu300_exmdm.gpio_wireless_disable);
  
	return 0;
}


static struct platform_driver steu300_exmdm_driver = {
	.probe		= steu300_exmdm_probe,
	.remove		= steu300_exmdm_remove,
	.driver		= {
		.name = "exmdm-steu300",
		.owner = THIS_MODULE,
	},
};

static struct platform_device *steu300_exmdm_device;

int steu300_exmdm_register(void)
{
  printk("####samdebug:%s\n", __func__);

  if (steu300_exmdm.registered)
    return 0;

  steu300_exmdm_device = platform_device_register_simple("exmdm-steu300", -1, NULL, 0);
  if (IS_ERR(steu300_exmdm_device))
    return PTR_ERR(steu300_exmdm_device);

  steu300_exmdm.registered = true;
  return 0;
}

void steu300_exmdm_unregister(void)
{
  printk("####samdebug:%s\n", __func__);

  if (steu300_exmdm.registered) {
    platform_device_unregister(steu300_exmdm_device);
    steu300_exmdm.registered = false;
  }
}

EXPORT_SYMBOL_GPL(steu300_exmdm_register);
EXPORT_SYMBOL_GPL(steu300_exmdm_unregister);
 
static int __init steu300_exmdm_init(void)
{
  printk("####samdebug:%s\n", __func__);
	return platform_driver_register(&steu300_exmdm_driver);
}

static void __exit steu300_exmdm_exit(void)
{
  printk("####samdebug:%s\n", __func__);
	platform_driver_unregister(&steu300_exmdm_driver);
}

module_init(steu300_exmdm_init);
module_exit(steu300_exmdm_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("STE U300 modem control driver");
MODULE_LICENSE("GPL");
