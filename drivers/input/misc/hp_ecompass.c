/******************** (C) COPYRIGHT 2010 HP ********************
*
* File Name          : hp_ecpompass.c
* Authors            : Wade 
* Version            : V 1.0
* Date               : 28/09/2010
* Description        : dummy eCompass sensor device driver
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

24_5_2010: hp_ecompass_get_acceleration_data now converts the saturation
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


#define NAME			HP_ECOMPASS_NAME


struct hp_ecompass_data {
	struct input_dev *input_dev;
	atomic_t enabled;
	int 	flag;
	u8 resume_state[3];
	int poll_interval;
};


/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
static struct hp_ecompass_data *hp_ecompass_misc_data;

static DECLARE_WAIT_QUEUE_HEAD(active_wq);


#if 0
static void hp_ecompass_report_values(struct hp_ecompass_data *ecomapss,
					int *xyz)
{
	printk("guoye: reporting orientation... (xyz[0], xyz[1], xyz[2]) = (%d, %d, %d) \n", xyz[0], xyz[1], xyz[2]);

	input_report_abs(ecomapss->input_dev, ABS_RX, xyz[0]);
	input_report_abs(ecomapss->input_dev, ABS_RY, xyz[1]);
	input_report_abs(ecomapss->input_dev, ABS_RZ, xyz[2]);
	input_sync(ecomapss->input_dev);
}
#endif

static int hp_ecompass_enable(struct hp_ecompass_data *ecomapss)
{
	if (!atomic_cmpxchg(&ecomapss->enabled, 0, 1)) {
		//pr_info("%s\n", __func__);
		ecomapss->flag = 1;
		wake_up_interruptible(&active_wq);
	}

	return 0;
}

static int hp_ecompass_disable(struct hp_ecompass_data *ecomapss)
{
	if (atomic_cmpxchg(&ecomapss->enabled, 1, 0)) {
		//pr_info("%s\n", __func__);
		ecomapss->flag = 1;
		wake_up_interruptible(&active_wq);
	}

	return 0;
}

static int hp_ecompass_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = hp_ecompass_misc_data;

	return 0;
}

static int hp_ecompass_misc_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int interval;
	struct hp_ecompass_data *ecomapss = file->private_data;

	switch (cmd) {
	case HP_ORIENTATIONSENSOR_IOCTL_GET_DELAY:		
		interval = ecomapss->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case HP_ORIENTATIONSENSOR_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 1500)
			return -EINVAL;

		ecomapss->poll_interval = interval;
		break;

	case HP_ORIENTATIONSENSOR_IOCTL_ENABLE:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval > 1)
			return -EINVAL;
		if (interval)
			hp_ecompass_enable(ecomapss);
		else
			hp_ecompass_disable(ecomapss);
		break;

	case HP_ORIENTATIONSENSOR_IOCTL_GET_STATUS:
		ecomapss ->flag = 0;
	case HP_ORIENTATIONSENSOR_IOCTL_GET_ENABLED:
		interval = atomic_read(&ecomapss->enabled);
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EINVAL;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static unsigned int hp_ecompass_misc_poll(struct file* file, poll_table* wait)
{
	unsigned int    mask = 0;
	struct hp_ecompass_data *ecomapss = file->private_data;

	poll_wait(file, &active_wq, wait);

	/* if have something to read  */
	if (ecomapss ->flag)
	{
		mask |= POLLIN;
	}	
	return mask;
}

static const struct file_operations hp_ecompass_misc_fops = {
	.owner = THIS_MODULE,
	.open = hp_ecompass_misc_open,
	.ioctl = hp_ecompass_misc_ioctl,
	.poll = hp_ecompass_misc_poll,
};

static struct miscdevice hp_ecompass_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &hp_ecompass_misc_fops,
};


static int hp_ecompass_input_init(struct hp_ecompass_data *ecomapss)
{
	int err;

	ecomapss->input_dev = input_allocate_device();
	if (!ecomapss->input_dev) {
		err = -ENOMEM;
		goto err0;
	}

	input_set_drvdata(ecomapss->input_dev, ecomapss);

	set_bit(EV_ABS, ecomapss->input_dev->evbit);
	input_set_abs_params(ecomapss->input_dev, ABS_RX, -400, 400, 0, 0);
	input_set_abs_params(ecomapss->input_dev, ABS_RY, -400, 400, 0, 0);
	input_set_abs_params(ecomapss->input_dev, ABS_RZ, -400, 400, 0, 0);

	ecomapss->input_dev->name = NAME;

	err = input_register_device(ecomapss->input_dev);
	if (err) {
		goto err1;
	}

	return 0;

err1:
	input_free_device(ecomapss->input_dev);
err0:
	return err;
}

static void hp_ecompass_input_cleanup(struct hp_ecompass_data *ecomapss)
{
	input_unregister_device(ecomapss->input_dev);
	input_free_device(ecomapss->input_dev);
}

static int hp_dummy_ecompass_init(void)
{
	struct hp_ecompass_data *ecomapss;
	int err = -1;

	//pr_info("%s\n", __func__);

	ecomapss = kzalloc(sizeof(*ecomapss), GFP_KERNEL);
	if (ecomapss == NULL) {
		pr_err("%s: failed to allocate memory for module data\n", __func__);
		err = -ENOMEM;
		goto err0;
	}

	memset(ecomapss->resume_state, 0, ARRAY_SIZE(ecomapss->resume_state));

	ecomapss->resume_state[0] = 0x10;
	ecomapss->resume_state[1] = 0x20;
	ecomapss->resume_state[2] = 0x00;

	/* As default, do not report information */
	atomic_set(&ecomapss->enabled, 0);

	init_waitqueue_head(&active_wq);

	err = hp_ecompass_input_init(ecomapss);
	if (err < 0)
		goto err1;

	hp_ecompass_misc_data = ecomapss;

	err = misc_register(&hp_ecompass_misc_device);
	if (err < 0) {
		goto err2;
	}

	return 0;

err2:
	hp_ecompass_input_cleanup(ecomapss);
err1:
	kfree(ecomapss);
err0:
	return err;
}

static int __init hp_ecompass_init(void)
{
	//printk(KERN_INFO "HP eCompass driver\n");
	return hp_dummy_ecompass_init();
}

static void __exit hp_ecompass_exit(void)
{
	return;
}

module_init(hp_ecompass_init);
module_exit(hp_ecompass_exit);

MODULE_DESCRIPTION("HP eCompass driver ");
MODULE_AUTHOR("HP");
MODULE_LICENSE("GPL");
