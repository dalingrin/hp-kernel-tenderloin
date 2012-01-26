/******************** (C) COPYRIGHT 2010 HP ********************
*
* File Name          : hp_gravitysensor.c
* Authors            : Wade 
* Version            : V 1.0
* Date               : 28/09/2010
* Description        : dummy gravity sensor sensor device driver
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
******************************************************************************

24_5_2010: hp_gravitysensor_get_acceleration_data now converts the saturation
	   value from 0xF000 (coming from the sensor) to 0x8000

******************************************************************************/

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/hp_sensors.h>

char* NAME[]= {HP_GRAVITYSENSORS_NAME, HP_ROTATIONSENSORS_NAME, HP_GYROSENSORS_NAME};

static struct input_dev *input_dev[sizeof(NAME)/sizeof(NAME[0])]={NULL};

static void hp_gravitysensor_input_cleanup(void)
{
	int i;
	for (i=0; i < sizeof(NAME)/sizeof(NAME[0]); i++)
	{
		if (input_dev[i])
		{	
			input_unregister_device(input_dev[i]);
			input_free_device(input_dev[i]);
			input_dev[i]=NULL;
		}	
	}
}


static int hp_gravitysensor_input_init(void)
{
	int err;
	int i;

	for (i=0; i < sizeof(NAME)/sizeof(NAME[0]); i++)
	{
		input_dev[i] = input_allocate_device();
		if (!input_dev[i]) {
			err = -ENOMEM;
			goto err0;
		}
	
		set_bit(EV_ABS, input_dev[i]->evbit);
		input_set_abs_params(input_dev[i], ABS_X, -10240, 10240, 0, 0);
		input_set_abs_params(input_dev[i], ABS_Y, -10240, 10240, 0, 0);
		input_set_abs_params(input_dev[i], ABS_Z, -10240, 10240, 0, 0);
	
		input_dev[i]->name = NAME[i];
	
		err = input_register_device(input_dev[i]);
		if (err) {
			goto err1;
		}
	}
	return 0;

err1:
err0:
	hp_gravitysensor_input_cleanup();
	return err;
}


static int hp_dummy_gravitysensor_init(void)
{
	int err = -1;

	pr_info("%s\n", __func__);

	err = hp_gravitysensor_input_init();

	return err;
}

static int __init hp_gravitysensor_init(void)
{
	printk(KERN_INFO "HP GRAVITY Sensor Driver\n");
	return hp_dummy_gravitysensor_init();
}

static void __exit hp_gravitysensor_exit(void)
{
	hp_gravitysensor_input_cleanup();
	return;
}

module_init(hp_gravitysensor_init);
module_exit(hp_gravitysensor_exit);

MODULE_DESCRIPTION("HP GRAVITY Sensor Driver");
MODULE_AUTHOR("HP");
MODULE_LICENSE("GPL");
