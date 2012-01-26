/*
 * Driver model for leds and led triggers
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LINUX_EXMDM_H_INCLUDED
#define __LINUX_EXMDM_H_INCLUDED

#include <linux/list.h>
#include <linux/spinlock.h>

struct device;

#define  EXMDM_WIRELESS_DISABLE_OFF 0
#define  EXMDM_WIRELESS_DISABLE_ON  1

struct exmdm_classdev {
	const char		*name;
	int wireless_disable_state;

	/* Set external modem power state */
	void		(*wireless_disable_set)(struct exmdm_classdev *exmdm_cdev,
					  int state);
	/* Get external modem power state */
	int (*wireless_disable_get)(struct exmdm_classdev *exmdm_cdev);

	struct device		*dev;
};

extern int exmdm_classdev_register(struct device *parent,
				 struct exmdm_classdev *exmdm_cdev);
extern void exmdm_classdev_unregister(struct exmdm_classdev *exmdm_cdev);


#endif		/* __LINUX_EXMDM_H_INCLUDED */
