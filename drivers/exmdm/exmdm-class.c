/*
 * EXMDM Class Core
 *
 * Copyright 2010 HP Ltd.
 *
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include "exmdms.h"

static struct class *exmdm_class;

static void exmdm_wireless_disable_state_update(struct exmdm_classdev *exmdm_cdev) {
  exmdm_cdev->wireless_disable_state = exmdm_cdev->wireless_disable_get(exmdm_cdev);
}

static ssize_t exmdm_wireless_disable_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct exmdm_classdev *exmdm_cdev = dev_get_drvdata(dev);

  exmdm_wireless_disable_state_update(exmdm_cdev);

	return sprintf(buf, "%u\n", exmdm_cdev->wireless_disable_state);
}

static ssize_t exmdm_wireless_disable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct exmdm_classdev *exmdm_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		if (exmdm_cdev->wireless_disable_state != value) {
      exmdm_cdev->wireless_disable_state = value;
      if (exmdm_cdev->wireless_disable_set) 
	      exmdm_cdev->wireless_disable_set(exmdm_cdev, value);
    }
	}

	return ret;
}

static struct device_attribute exmdm_class_attrs[] = {
	__ATTR(wireless_disable, 0644, exmdm_wireless_disable_show, exmdm_wireless_disable_store),
	__ATTR_NULL,
};


/**
 * exmdm_classdev_register - register a new object of exmdm_classdev class.
 * @parent: The device to register.
 * @exmdm_cdev: the exmdm_classdev structure for this device.
 */
int exmdm_classdev_register(struct device *parent, struct exmdm_classdev *exmdm_cdev)
{
	exmdm_cdev->dev = device_create(exmdm_class, parent, 0, exmdm_cdev,
				      "%s", exmdm_cdev->name);

	if (IS_ERR(exmdm_cdev->dev))
		return PTR_ERR(exmdm_cdev->dev);

  exmdm_wireless_disable_state_update(exmdm_cdev);

	printk(KERN_DEBUG "Registered exmdm device: %s\n",
			exmdm_cdev->name);

	return 0;
}

EXPORT_SYMBOL_GPL(exmdm_classdev_register);

/**
 * led_classdev_unregister - unregisters a object of led_properties class.
 * @led_cdev: the led device to unregister
 *
 * Unregisters a previously registered via led_classdev_register object.
 */
void exmdm_classdev_unregister(struct exmdm_classdev *exmdm_cdev)
{
	device_unregister(exmdm_cdev->dev);
}
EXPORT_SYMBOL_GPL(exmdm_classdev_unregister);

static int __init exmdm_init(void)
{
	exmdm_class = class_create(THIS_MODULE, "exmdm");
	if (IS_ERR(exmdm_class))
		return PTR_ERR(exmdm_class);
	//exmdm_class->suspend = exmdm_suspend;
	//exmdm_class->resume = exmdm_suspend;
	exmdm_class->dev_attrs = exmdm_class_attrs;
	return 0;
}

static void __exit exmdm_exit(void)
{
	class_destroy(exmdm_class);
}

subsys_initcall(exmdm_init);
module_exit(exmdm_exit);

MODULE_AUTHOR("Sam Lin");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exmdm Control Class Interface");
