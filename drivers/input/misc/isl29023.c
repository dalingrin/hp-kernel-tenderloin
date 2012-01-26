/******************************************************************************
 * isl29023.c - Linux kernel module for Intersil ISL29023 ambient light sensor
 *
 * Copyright 2008-2009 Intersil Inc..
 *
 * DESCRIPTION:
 *	- This is the linux driver for ISL29023 and passed the test under the Linux
 *	Kernel version 2.6.30.4
 *
 * modification history
 * --------------------
 * v1.0   2009/09/22, Shouxian Chen(Simon Chen) create this file

 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/idr.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <asm/io.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/hp_sensors.h>
#include <linux/isl29023.h>
#include "isl29023.h"


//#define ALS_DEBUG

#define ARRAYSIZE(a) (sizeof(a) / sizeof(a[0]))
#define ALS_READ_DELAY   (HZ * 1)

/* Do not scan isl29023 automatic */
static const unsigned short normal_i2c[] = {I2C_CLIENT_END };

/* force i2c addr */
static const unsigned short force[]= {
	ANY_I2C_BUS,ISL29023_ADDR,
	I2C_CLIENT_END};

static const unsigned short * const forces[] = {force, NULL};

/* Insmod parameters */
//I2C_CLIENT_INSMOD_COMMON;

/* data struct for isl29023 device */
struct isl29023_data_t	isl29023_data = {
	.minor = 255, /* 255 indicated that isl29023 is not regsitered as device file*/
	.dev_open_cnt = 0,
	.client = NULL,
	.pwr_status = 0
};

static const int sLuxValues[] = {
    1,
    3,
    30,
    120,
    250,
    750,
    1200,
    2200,
    5000,
    8000,
    1000000,
};

static int isl29023_get_status(struct isl29023_data_t *p_data)
{
	int als_data;

	// printk(KERN_INFO "%s start\n", __func__);
	if (!p_data->enabled)
	{	
		pr_err("[%s] ALS is not enabled\n", __func__);
		return -1;
	}
	
	/* read ALS data */
	als_data = i2c_smbus_read_word_data(p_data->client,0x02);
	if (als_data < 0) 
	{	
		pr_err("[%s] failed to read ALS data\n", __func__);
		return -1;
	}
	#ifdef ALS_DEBUG
	pr_err("##### Wade : %s ALS Read code = %d\n", __func__, als_data);
	#endif

#if 0
		/* get the commamd I reg value */
		ret = ioctl(fd,RD_CMD1,&buf_8);
		if (ret < 0) goto err;

		/* see if interrupt is triggered */
		if (buf_8 & 0x04) 
			printk(KERN_INFO "%s Interrupt is triggered \n", __fuc__);

		/* Set isl29023 as IR once mode */
		buf_8 = buf_8 & (~0xe4);
		buf_8 = buf_8 | 0x40;
		ret = ioctl(fd,WR_CMD1,&buf_8);
		if (ret < 0) goto err;

		/* wait 100ms for adc finished */
		usleep(100);

		/* read IR data */
		ret = ioctl(fd,RD_DATA,&buf_16);
		if (ret < 0) goto err;
		printk(KERN_INFO "%s IR Read code = %d\n\n", __func__, buf_16);
#endif

    return als_data;
}

static int isl29023_report(struct isl29023_data_t *p_data)
{
	int i;
	int rc = isl29023_get_status(p_data);
	
	#ifdef ALS_DEBUG
	pr_err("##### Wade : isl29023_report, value:%d\n", rc);
	#endif
	if (rc>=0)
	{	
		for (i=0; i < ARRAYSIZE(sLuxValues)-1; i++)
		{
			if (rc<sLuxValues[i+1])
			{
			  break;          
			}
		}
		if (i==ARRAYSIZE(sLuxValues)-1)
		{
			i = ARRAYSIZE(sLuxValues)-2;
		}
		if (p_data->pre_status!=sLuxValues[i])	
		{
			rc=p_data->pre_status=sLuxValues[i];
			input_report_abs(p_data->input_dev, ABS_MISC, rc);
			input_sync(p_data->input_dev);
			#ifdef ALS_DEBUG
			pr_err("##### Wade : isl29023_report, input_report_abs:%d\n", rc);            
			#endif
		}
	}	    
	
	return rc;
}

static void isl29023_work_func(struct work_struct *work)
{
	isl29023_report(&isl29023_data);
	schedule_delayed_work(&isl29023_data.polling_work, ALS_READ_DELAY);    
}

static int isl29023_enable_hw(struct isl29023_data_t *p_data)
{
	int rc;
	
	//printk(KERN_INFO "%s \n", __func__);

	/* command I reg : Power down, interrupt persist = 1 */
	rc = i2c_smbus_write_byte_data(p_data->client,0x00,0x0);
	if (rc<0)
	{	
		goto err;
	}	

	/* command II reg : 12 bits ADC, range = 4000 lux */
	rc = i2c_smbus_write_byte_data(p_data->client,0x01,0x5);
	if (rc<0)
	{	
		goto err;
	}	

	/* INT_LT reg : INT_LT reg = 50 */
	rc = i2c_smbus_write_word_data(p_data->client,0x04,50);
	if (rc<0)
	{	
		goto err;
	}	

	/* INT_LT reg : INT_LT reg = 4095 */
	rc = i2c_smbus_write_word_data(p_data->client,0x06,4095);
	if (rc<0)
	{	
		goto err;
	}	

	/* Set isl29023 as ALS once mode */
	//rc = i2c_smbus_write_byte_data(p_data->client,0x00,0x20);	
	/* Set isl29023 as ALS continuous mode */
	rc = i2c_smbus_write_byte_data(p_data->client,0x00,0xA0);
	if (rc<0)
	{	
		goto err;
	}	
	
	/* TEMPORARY HACK: schedule a deferred light sensor read
	* to work around sensor manager race condition
	*/
	#ifdef ALS_DEBUG	
	pr_err("##### Wade : %s: schedule_delayed_work\n", __func__);
	#endif
	schedule_delayed_work(&p_data->polling_work, ALS_READ_DELAY);
err:	
	return rc;

}

static int isl29023_enable(struct isl29023_data_t *p_data)
{
	int rc;
	
	//printk(KERN_INFO "%s \n", __func__);
  //HP zhanghong:Oct 14 14:19 CST 2010, begin
	if(p_data->enabled)
		  return 0;
	//End
	
	rc=isl29023_enable_hw(p_data);    
	if (rc==0) {
		p_data->enabled = 1;
		p_data->pre_status=-1;	
	}
	else
	  printk(KERN_ERR"%s: enable light sensor fail,rc=%d\n",__func__,rc);
	return rc;
}

static int isl29023_disable_hw(struct isl29023_data_t *p_data)
{
	int rc;
	
	//printk(KERN_INFO "%s \n", __func__);
	
	cancel_delayed_work(&p_data->polling_work);
	
	/* power off the isl29023 */
	rc = i2c_smbus_write_byte_data(isl29023_data.client, 0x00, 0x00);
	if (rc < 0)
	{
		pr_err("%s: failed to power off the isl29023\n", __func__);
	}
	
	return 0;

}

static int isl29023_disable(struct isl29023_data_t *p_data)
{
	//printk(KERN_INFO "%s \n", __func__);

	if (p_data->enabled == 0)
		return 0;
	
	p_data->enabled = 0;
	return isl29023_disable_hw(p_data);	
}


static int isl29023_open(struct inode *inode, struct file *filp)
{
	u8 i_minor;

	i_minor = MINOR(inode->i_rdev);

	/* check open file conut */
	spin_lock(&isl29023_data.lock);
//	if ( i_minor != isl29023_data.minor) goto no_dev_err;
	isl29023_data.dev_open_cnt++;
	spin_unlock(&isl29023_data.lock);

	filp->private_data = (void *)&isl29023_data;

	return 0;

//no_dev_err:
//	spin_unlock(&isl29023_data.lock);
//	pr_err("%s: no this device, i_minor:%d isl29023_data.minor:%d\n", __func__, i_minor, isl29023_data.minor);
//	return -ENODEV;
}

static int isl29023_close(struct inode *inode, struct file *filp)
{
	u8 i_minor;
	struct isl29023_data_t *p_data;

	p_data = (struct isl29023_data_t *)filp->private_data;
	i_minor = MINOR(inode->i_rdev);

	spin_lock(&(p_data->lock));
//	if ( i_minor != p_data->minor) goto no_dev_err;
	if (p_data->dev_open_cnt > 0) p_data->dev_open_cnt--;
	spin_unlock(&(p_data->lock));

	/* power off the isl29023 */
	//i2c_smbus_write_byte_data(isl29023_data.client, 0x00, 0x00);

	return 0;

//no_dev_err:
//	spin_unlock(&(p_data->lock));
//	pr_err("%s: no this device, i_minor:%d isl29023_data.minor:%d\n", __func__, i_minor, isl29023_data.minor);
//	return -ENODEV;
}

#define NATIVE_TEST_IOCTL
static int isl29023_ioctl(struct inode *inode, struct file *filp,	unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	#ifdef  NATIVE_TEST_IOCTL
	u8 buf_8;
	u16 buf_16;
	s32	buf_32;
	int ret;
	#endif
	struct isl29023_data_t *p_data;
	u32 val;

	p_data = (struct isl29023_data_t *)filp->private_data;
	if (p_data == NULL) return -EFAULT;

	switch (cmd)
	{
		case HP_LIGHTSENSOR_IOCTL_ENABLE: 
			if (get_user(val, (unsigned long __user *)argp)) {
				return -EFAULT;
			}
			if (val)
				return isl29023_enable(p_data);
			else
				return isl29023_disable(p_data);
			break;
			
		case HP_LIGHTSENSOR_IOCTL_GET_ENABLED: 
			return put_user(p_data->enabled, (unsigned long __user *)arg);		

#ifdef  NATIVE_TEST_IOCTL		
		case WR_CMD1:
			ret = get_user(buf_8, (u8 __user*)argp);
			if (ret) return -EFAULT;
			return i2c_smbus_write_byte_data(p_data->client,0x00,buf_8);

		case WR_CMD2:
			ret = get_user(buf_8, (u8 __user*)argp);
			if (ret) return -EFAULT;
			return i2c_smbus_write_byte_data(p_data->client,0x01,buf_8);

		case WR_INT_LT:
			ret = get_user(buf_16, (u16 __user*)argp);
			if (ret) return -EFAULT;
			return i2c_smbus_write_word_data(p_data->client,0x04,buf_16);

		case WR_INT_HT:
			ret = get_user(buf_16, (u16 __user*)argp);
			if (ret) return -EFAULT;
			return i2c_smbus_write_word_data(p_data->client,0x06,buf_16);

		case RD_CMD1:
			buf_32 = i2c_smbus_read_byte_data(p_data->client,0x00);
			if(buf_32 < 0) return buf_32;
			return put_user((u8)buf_32, (u8 __user*)argp);

		case RD_CMD2:
			buf_32 = i2c_smbus_read_byte_data(p_data->client,0x01);
			if(buf_32 < 0) return buf_32;
			return put_user((u8)buf_32, (u8 __user*)argp);

		case RD_INT_LT:
			buf_32 = i2c_smbus_read_word_data(p_data->client,0x04);
			if(buf_32 < 0) return buf_32;
			return put_user((u16)buf_32, (u16 __user*)argp);

		case RD_INT_HT:
			buf_32 = i2c_smbus_read_word_data(p_data->client,0x06);
			if(buf_32 < 0) return buf_32;
			return put_user((u16)buf_32, (u16 __user*)argp);

		case RD_DATA:
			buf_32 = i2c_smbus_read_word_data(p_data->client,0x02);
			if(buf_32 < 0) return buf_32;
			return put_user((u16)buf_32, (u16 __user*)argp);
#endif
		default: return -EINVAL;
	}

	return 0;
}

static struct file_operations isl29023_fops = {
	.owner = THIS_MODULE,
	.open = isl29023_open,
	.release = isl29023_close,
	.ioctl = isl29023_ioctl
	//.unlocked_ioctl = isl29023_ioctl,	
};

static struct miscdevice isl29023_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = HP_LIGHTSENSORS_NAME,
	.fops = &isl29023_fops
};

/* Return 0 if detection is successful, -ENODEV otherwise */
static int isl29023_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA
				     | I2C_FUNC_SMBUS_READ_BYTE))
		return -ENODEV;

	/* probe that if isl29023 is at the i2 address */
	if (i2c_smbus_xfer(adapter, client->addr, 0,I2C_SMBUS_WRITE,
		0,I2C_SMBUS_QUICK,NULL) < 0)
		return -ENODEV;

	strlcpy(info->type, "isl29023", I2C_NAME_SIZE);
	//printk(KERN_INFO "%s is found at i2c device address %d\n", info->type, client->addr);

	return 0;
}

static int isl29023_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct intersil_isl29023_platform_data *pdata = client->dev.platform_data;
	int rc = -EIO;
    
	if(pdata && pdata->power)
		pdata->power(1);

	if(pdata && pdata->configure_int_pin)
		pdata->configure_int_pin(1);
	/* initial device data struct */
	isl29023_data.minor = 0;
	isl29023_data.client = client;
	isl29023_data.dev_open_cnt = 0;
	//HP zhanghong: Oct 14 13:35 CST 2010, begin
  isl29023_data.enabled = 0;
	//End
	spin_lock_init(&isl29023_data.lock);
       //HP zhanghong:Oct 18 9:38 CST 2010, begin
	isl29023_data.pdata = kmalloc(sizeof(*isl29023_data.pdata), GFP_KERNEL);
	if (isl29023_data.pdata == NULL)
		{
	    pr_err("%s: could not allocate pdata\n", __func__);
		  rc = -ENOMEM;
		  goto done;
		}
	memcpy(isl29023_data.pdata, client->dev.platform_data, sizeof(*isl29023_data.pdata));
	//End

	i2c_set_clientdata(client,&isl29023_data);
	
	isl29023_data.input_dev = input_allocate_device();
	if(!isl29023_data.input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		rc = -ENOMEM;
		//HP zhanghong:Oct 18 10:34 CST 2010, begin
		kfree(isl29023_data.pdata);
		//End
		goto done;
	}
	input_set_drvdata(isl29023_data.input_dev, &isl29023_data);
	isl29023_data.input_dev->name = HP_LIGHTSENSORS_NAME;
	set_bit(EV_ABS, isl29023_data.input_dev->evbit);
	input_set_abs_params(isl29023_data.input_dev, ABS_MISC, 0, 9, 0, 0);
	
	rc = input_register_device(isl29023_data.input_dev);
	if (rc < 0) {
		pr_err( "%s: can not register input device\n", __func__);
		input_free_device(isl29023_data.input_dev);
		isl29023_data.input_dev=NULL;
		goto done;
	}
	rc = misc_register(&isl29023_misc);
	if (rc < 0) {
		pr_err("%s: can not register misc device\n", __func__);
		input_set_drvdata(isl29023_data.input_dev, NULL);
		input_unregister_device(isl29023_data.input_dev);
		input_free_device(isl29023_data.input_dev);
		goto done;
	}
	
	INIT_DELAYED_WORK(&isl29023_data.polling_work, isl29023_work_func);

done:	
	//printk(KERN_INFO "isl29023 device file is registered at MAJOR = %d,MINOR = %d\n", ISL29023_MAJOR, isl29023_data.minor);
	return 0;
}

static int isl29023_remove(struct i2c_client *client)
{
	//printk(KERN_INFO "%s at address %d is removed\n",client->name,client->addr);

	/* clean the isl29023 data struct when isl29023 device remove */
	isl29023_data.minor = 255;
	isl29023_data.client = NULL;
	isl29023_data.dev_open_cnt = 0;
	//HP zhanghong:Oct 18 10:36 CST 2010, begin
  if(isl29023_data.pdata)
	 	kfree(isl29023_data.pdata);
	//End
	return 0;
}

#ifdef CONFIG_PM	/* if define power manager, define suspend and resume function */
static int isl29023_suspend(struct i2c_client *client, pm_message_t mesg)
{
	u32 buf_32;
	struct isl29023_data_t *p_data = i2c_get_clientdata(client);

	if(p_data->enabled == 0)
	   return 0;     
	/* save power set now, and then set isl29023 to power down mode */
	buf_32 = i2c_smbus_read_byte_data(p_data->client,0x00);
	if (buf_32 < 0) return -EIO;

	p_data->pwr_status = (u8)buf_32 & 0xe0;
	//[HPP]guoye-Always return success
	//return i2c_smbus_write_byte_data(client,0x00,(u8)buf_32 & (~0xe0));
	i2c_smbus_write_byte_data(client,0x00,(u8)buf_32 & (~0xe0));
	return 0;
	//[HPP]guoye
}

static int isl29023_resume(struct i2c_client *client)
{
	u32 buf_32;
	u8	buf_8;
	struct isl29023_data_t *p_data = i2c_get_clientdata(client);

  if(p_data->enabled == 0)
	   return 0;
	/* resume the power staus of isl29023 */
	buf_32 = i2c_smbus_read_byte_data(p_data->client,0x00);
	if (buf_32 < 0) return -EIO;

	buf_8 = (buf_32 & (~0xe0)) | p_data->pwr_status;

	//[HPP]guoye-Always return success
	//return i2c_smbus_write_byte_data(client,0x00,buf_8);
	i2c_smbus_write_byte_data(client,0x00,buf_8);
	return 0;
	//[HPP]guoye
}
#else
#define	isl29023_suspend 	NULL
#define isl29023_resume		NULL
#endif		/*ifdef CONFIG_PM end*/

static const struct i2c_device_id isl29023_id[] = {
	{ "isl29023", 0 },
	{ }
};

static struct i2c_driver isl29023_driver = {
	.driver = {
		.name	= "isl29023",
	},
	.probe			= isl29023_probe,
	.remove			= isl29023_remove,
	.id_table		= isl29023_id,
	.detect			= isl29023_detect,
//	.address_data	= &addr_data,
	.suspend		= isl29023_suspend,
	.resume			= isl29023_resume
};

struct i2c_client *isl29023_client;

static int __init isl29023_init(void)
{
	int ret;

	/* register the i2c driver for isl29023 */
	ret = i2c_add_driver(&isl29023_driver);
	if (ret) goto ADD_DRV_FAIL;

	return 0;

ADD_DRV_FAIL:
	i2c_del_driver(&isl29023_driver);
	printk(KERN_ERR "Add isl29023 driver error\n");
	return ret;
}

static void __exit isl29023_exit(void)
{
	i2c_del_driver(&isl29023_driver);
}


MODULE_AUTHOR("Chen Shouxian");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ISL29023 ambient light sensor driver");
MODULE_VERSION(DRIVER_VERSION);

module_init(isl29023_init);
module_exit(isl29023_exit);
