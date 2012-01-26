/*
 * linux/drivers/exmdm/exmdms-sierra8705.c
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

struct sierra8705_exmdm_data {
	struct exmdm_classdev	cdev;
	int gpio_wireless_disable;
	bool gpio_wireless_disable_active_low;
	bool registered;
};

static void 
sierra8705_exmdm_wireless_disable_set(struct exmdm_classdev *cdev, 	int state);
static int 
sierra8705_exmdm_wireless_disable_get(struct exmdm_classdev *cdev);

static struct sierra8705_exmdm_data sierra8705_exmdm = {
		.cdev		= {
			.name		= "sierra8705",
			.wireless_disable_set = sierra8705_exmdm_wireless_disable_set,
			.wireless_disable_get = sierra8705_exmdm_wireless_disable_get,
			.wireless_disable_state = EXMDM_WIRELESS_DISABLE_OFF
		},
		.gpio_wireless_disable = 171,
		.gpio_wireless_disable_active_low = true,
		.registered = false
	};

static void sierra8705_exmdm_wireless_disable_set(struct exmdm_classdev *cdev,
				int state)
{
  int value;
  struct sierra8705_exmdm_data *pexmdm = container_of(cdev, struct sierra8705_exmdm_data, cdev);

  if (pexmdm->gpio_wireless_disable_active_low)
    value = EXMDM_WIRELESS_DISABLE_OFF == state ? 1 : 0;
  else
    value = EXMDM_WIRELESS_DISABLE_OFF == state ? 0 : 1;

  printk("####samdebug:%s: set wireless disable state as %d, write %d to gpio register\n", __func__, state, value);
  
  gpio_set_value_cansleep(pexmdm->gpio_wireless_disable, value);	
}

static int sierra8705_exmdm_wireless_disable_get(struct exmdm_classdev *cdev)
{
  int state, value;
  struct sierra8705_exmdm_data *pexmdm = container_of(cdev, struct sierra8705_exmdm_data, cdev);

  value = gpio_get_value_cansleep(pexmdm->gpio_wireless_disable);

  if (pexmdm->gpio_wireless_disable_active_low)
    state = value ? EXMDM_WIRELESS_DISABLE_OFF : EXMDM_WIRELESS_DISABLE_ON;
  else 
    state = value ? EXMDM_WIRELESS_DISABLE_ON : EXMDM_WIRELESS_DISABLE_OFF;

  printk("####samdebug:%s: get wireless disable state as %d, read %d from gpio register\n", __func__, state, value);
  
  return state;
}

static int sierra8705_exmdm_probe(struct platform_device *pdev)
{
	int ret;
  //int value;
  
  printk("####samdebug:%s\n", __func__);
  ret = gpio_request(sierra8705_exmdm.gpio_wireless_disable, "sierra8705_wireless_disable");
  if (ret < 0)
    return ret;

  /*
  if (sierra8705_exmdm.gpio_wireless_disable_active_low)
    value = EXMDM_WIRELESS_DISABLE_OFF == sierra8705_exmdm.cdev.wireless_disable_state ? 1 : 0;
  else
    value = EXMDM_WIRELESS_DISABLE_OFF == sierra8705_exmdm.cdev.wireless_disable_state ? 0 : 1;
  
  ret = gpio_direction_output(sierra8705_exmdm.gpio_wireless_disable, value);
  if (ret < 0) {
    gpio_free(sierra8705_exmdm.gpio_wireless_disable);
    return ret;
  }
  */
  
	ret = exmdm_classdev_register(&pdev->dev, &sierra8705_exmdm.cdev);
	if (ret < 0) {
	  gpio_free(sierra8705_exmdm.gpio_wireless_disable);
	  exmdm_classdev_unregister(&sierra8705_exmdm.cdev);
  }

  return ret;
}

static int sierra8705_exmdm_remove(struct platform_device *pdev)
{
	exmdm_classdev_unregister(&sierra8705_exmdm.cdev);
  gpio_free(sierra8705_exmdm.gpio_wireless_disable);
  
	return 0;
}


static struct platform_driver sierra8705_exmdm_driver = {
	.probe		= sierra8705_exmdm_probe,
	.remove		= sierra8705_exmdm_remove,
	.driver		= {
		.name = "exmdm-sierra8705",
		.owner = THIS_MODULE,
	},
};

static struct platform_device *sierra8705_exmdm_device;

int sierra8705_exmdm_register(void)
{
  printk("####samdebug:%s\n", __func__);

  if (sierra8705_exmdm.registered)
    return 0;

  sierra8705_exmdm_device = platform_device_register_simple("exmdm-sierra8705", -1, NULL, 0);
  if (IS_ERR(sierra8705_exmdm_device))
    return PTR_ERR(sierra8705_exmdm_device);

  sierra8705_exmdm.registered = true;
  return 0;
}

void sierra8705_exmdm_unregister(void)
{
  printk("####samdebug:%s\n", __func__);

  if (sierra8705_exmdm.registered) {
    platform_device_unregister(sierra8705_exmdm_device);
    sierra8705_exmdm.registered = false;
  }
}

EXPORT_SYMBOL_GPL(sierra8705_exmdm_register);
EXPORT_SYMBOL_GPL(sierra8705_exmdm_unregister);
 
static int __init sierra8705_exmdm_init(void)
{
  printk("####samdebug:%s\n", __func__);
  //sierra8705_exmdm_register();
	return platform_driver_register(&sierra8705_exmdm_driver);
}

static void __exit sierra8705_exmdm_exit(void)
{
  printk("####samdebug:%s\n", __func__);
  //sierra8705_exmdm_unregister();
	platform_driver_unregister(&sierra8705_exmdm_driver);
}

module_init(sierra8705_exmdm_init);
module_exit(sierra8705_exmdm_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("Sierra modem control driver");
MODULE_LICENSE("GPL");
