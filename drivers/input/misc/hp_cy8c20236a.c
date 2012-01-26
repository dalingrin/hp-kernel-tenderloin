/******************** (C) COPYRIGHT 2010 HP ********************
*
* File Name          : hp_cy8c20236a.c
* Authors            : Hong zhang 
* Version            : V 1.0
* Date               : 27/10/2010
* Description        : proximity sensor device driver
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

******************************************************************************/

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/i2c/cy8c20236a.h>
#include <linux/hp_sensors.h>
#include <mach/gpio.h>



#define NAME			HP_PSENSORS_NAME  //"psensor"


#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5


#ifdef PS_POLLING_ENABLE
#define PS_READ_DELAY   (HZ * 1)
#endif

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
static struct cy8c20236a_data_t *cy8c20236a_ps_data=NULL;

unsigned char device_id;
u8 ch1_buf[2] ={0,0};
u8 ch2_buf[2] ={0,0};
u8 ch3_buf[3] ={0,0};

static int cy8c20236a_ps_i2c_read(struct cy8c20236a_data_t *ps_data,
					u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = ps_data->client->addr,
		 .flags = ps_data->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = ps_data->client->addr,
		 .flags = (ps_data->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do{
		err = i2c_transfer(ps_data->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&ps_data->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int cy8c20236a_ps_i2c_write(struct cy8c20236a_data_t *ps_data,
					 u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = ps_data->client->addr,
		 .flags = ps_data->client->flags,// & I2C_M_TEN,
		 .len = len+ 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(ps_data->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&ps_data->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int cypress_cy8c20236a_ps_get_status(struct cy8c20236a_data_t *ps_data){
		 int err = 0;
		 u8 buf[2];
	 
		 if(!ps_data->enabled)
			{
				pr_err("%s:proximity sensor is not enabled, return\n",__func__);
				return -1;
			}
		 mutex_lock(&ps_data->mutex_lock);
		 buf[0] = CY8C_DATA_CH1_LSB_REG;
		 buf[1] = 0;
		 err = cy8c20236a_ps_i2c_read(ps_data,buf,1);
		 if(err < 0)
			{
				pr_err("%s:read channel 1 LSB data error\n",__func__);
				goto error;
			}
		 ch1_buf[0] = buf[0];

		 buf[0] = CY8C_DATA_CH1_MSB_REG;
		 buf[1] = 0;
		 err = cy8c20236a_ps_i2c_read(ps_data,buf,1);
		 if(err < 0)
			{
				pr_err("%s:read channel 1 MSB data error\n",__func__);
				goto error;
			}
		 ch1_buf[1] = buf[0];

		 buf[0] = CY8C_DATA_CH2_LSB_REG;
		 buf[1] = 0;
		 err = cy8c20236a_ps_i2c_read(ps_data,buf,1);
		 if(err < 0)
			{
				pr_err("%s:read channel 2 LSB data error\n",__func__);
				goto error;
			}
		 ch2_buf[0] = buf[0];

		 buf[0] = CY8C_DATA_CH2_MSB_REG;
		 buf[1] = 0;
		 err = cy8c20236a_ps_i2c_read(ps_data,buf,1);
		 if(err < 0)
			{
				pr_err("%s:read channel 2 MSB data error\n",__func__);
				goto error;
			}
		 ch2_buf[1] = buf[0];

		 buf[0] = CY8C_DATA_CH3_LSB_REG;
		 buf[1] = 0;
		 err = cy8c20236a_ps_i2c_read(ps_data,buf,1);
		 if(err < 0)
			{
				pr_err("%s:read channel 3 LSB data error\n",__func__);
				goto error;
			}
		 ch3_buf[0] = buf[0];

		 buf[0] = CY8C_DATA_CH3_MSB_REG;
		 buf[1] = 0;
		 err = cy8c20236a_ps_i2c_read(ps_data,buf,1);
		 if(err < 0)
			{
				pr_err("%s:read channel 3 MSB data error\n",__func__);
				goto error;
			}
		 ch3_buf[1] = buf[0];
		 mutex_unlock(&ps_data->mutex_lock);
		 
		 pr_info("%s:channel 1 LSB[0x%x],MSB[0x%x]; channel 2 LSB[0x%x], MSB[0x%x];channel 3 LSB[0x%x],MSB[0x%x];\n",__func__,ch1_buf[0],ch1_buf[1],ch2_buf[0],ch2_buf[1],ch3_buf[0],ch3_buf[1]);

		 #ifndef PS_POLLING_ENABLE     
		 pr_info("%s:interrupter has happened, and int gpio state is %d\n",__func__,gpio_get_value(ps_data->p_out));
		 #else
		 return ((((ps_data->resume_state[2] << 8) |ps_data->resume_state[1]) > ((ch1_buf[1]<<8) | ch1_buf[0]))
					&& (((ps_data->resume_state[4] << 8) |ps_data->resume_state[3]) > ((ch2_buf[1]<<8) | ch2_buf[0]))
					&& (((ps_data->resume_state[6] << 8) |ps_data->resume_state[5]) > ((ch3_buf[1]<<8) | ch3_buf[0])));
		 #endif 
		 //need to return level,but there is something I don't understand, so block temperially.     
		 return 0;
error:
		 mutex_unlock(&ps_data->mutex_lock);
		 return err;

}

static int cypress_cy8c20236a_ps_report(struct cy8c20236a_data_t *ps_data)
{
		int rc;

		if((rc = cypress_cy8c20236a_ps_get_status(ps_data))>=0)
			{
				if(ps_data->pre_status != rc)
					{
						input_report_abs(ps_data->input_dev,ABS_DISTANCE, rc);
						input_sync(ps_data->input_dev);
						ps_data->pre_status = rc;
					}
			}
		return rc;
}

#ifdef PS_POLLING_ENABLE
static void cypress_cy8c20236a_work_func(struct work_struct *work){
		cypress_cy8c20236a_ps_report(cy8c20236a_ps_data);
		schedule_delayed_work(&cy8c20236a_ps_data->polling_work, PS_READ_DELAY);
}
#else
static int cypress_cy8c20236a_schedule_delayed_work(void* data,unsigned long delay)
{
		struct cy8c20236a_data_t * ps_data = data;
		wake_lock(&ps_data->ps_delayed_work_wake_lock);
		return queue_delayed_work(ps_data->psworkqueue, &ps_data->datareport, delay);

}

static void cypress_cy8c20236a_irq_delayed_work_func(struct work_struct *work){
		struct cy8c20236a_data_t *ps_data =
			container_of(work, struct cy8c20236a_data_t, datareport.work);
		cypress_cy8c20236a_ps_report(ps_data);
		wake_unlock(&ps_data->ps_delayed_work_wake_lock);
}
static irqreturn_t cypress_cy8c20236a_irq_handler(int irq, void *data)
{
		cypress_cy8c20236a_schedule_delayed_work(data,0);
		return IRQ_HANDLED;
}
#endif

static int cypress_cy8c2036a_ps_enable_hw(struct cy8c20236a_data_t *ps_data)
{
		 int err =-EIO;
		 u8 buf[2];
		 #ifndef PS_POLLING_ENABLE
		 //struct cypress_cy8c20236a_platform_data *pdata=ps_data->pdata;
		 int irq;
		 #endif
	 
		 pr_info("%s\n",__func__);

		 buf[0] = CY8C_THLD_LOW_CH1_REG;
		 buf[1] = ps_data->resume_state[1];

		 err = cy8c20236a_ps_i2c_write(ps_data,buf, 1);
		 if(err < 0)
			{
				 pr_err("%s: write channel 1 low threshold fail,err=%d\n",__func__,err);
				 goto done;
			}
			buf[0] = CY8C_THLD_HIGH_CH1_REG;
			buf[1] = ps_data->resume_state[2];

			err = cy8c20236a_ps_i2c_write(ps_data,buf, 1);
			if(err < 0)
			{
				 pr_err("%s: write channel 1 high threshold fail,err=%d\n",__func__,err);
				 goto done;
			}

			buf[0] = CY8C_THLD_LOW_CH2_REG;
			buf[1] = ps_data->resume_state[3];

			err = cy8c20236a_ps_i2c_write(ps_data,buf, 1);
			if(err < 0)
			{
				 pr_err("%s: write channel 2 low threshold fail,err=%d\n",__func__,err);
				 goto done;
			}
			buf[0] = CY8C_THLD_HIGH_CH2_REG;
			buf[1] = ps_data->resume_state[4];

			err = cy8c20236a_ps_i2c_write(ps_data,buf, 1);
			if(err < 0)
			{
				 pr_err("%s: write channel 2 high threshold fail,err=%d\n",__func__,err);
				 goto done;
			}

			buf[0] = CY8C_THLD_LOW_CH3_REG;
			buf[1] = ps_data->resume_state[5];

			err = cy8c20236a_ps_i2c_write(ps_data,buf, 1);
			if(err < 0)
			{
				 pr_err("%s: write channel 3 low threshold fail,err=%d\n",__func__,err);
				 goto done;
			}
			buf[0] = CY8C_THLD_HIGH_CH3_REG;
			buf[1] = ps_data->resume_state[6];

			err = cy8c20236a_ps_i2c_write(ps_data,buf, 1);
			if(err < 0)
			{
				 pr_err("%s: write channel 3 high threshold fail,err=%d\n",__func__,err);
				 goto done;
			}

		 buf[0] = CY8C_CTL_REG;
		 buf[1] = ps_data->resume_state[0];
		 err = cy8c20236a_ps_i2c_write(ps_data,buf, 1);
		 if(err < 0)
			{
				 pr_err("%s: power up proximity sensor fail,err=%d\n",__func__,err);
				 goto done;
			}
		 #ifdef PS_POLLING_ENABLE
		 schedule_delayed_work(&ps_data->polling_work, PS_READ_DELAY);
		 #else
		 irq = gpio_to_irq(ps_data->p_out); 
		 err = gpio_direction_input(ps_data->p_out);
		 if(err < 0)
			{
				pr_err("%s:failed to set gpio %d as input\n",__func__,ps_data->p_out);
				goto done;
			}
		 err = request_irq(irq,cypress_cy8c20236a_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"cypress_cy8c20236a", ps_data);
		 if(err < 0)
			{
				pr_err("%s:failed to request irq for gpio[%d]\n",__func__,ps_data->p_out);
				goto done;
			}
		 #endif
		 return 0;
done:
		 return err;
}


//enable disable for 
static int cypress_cy8c2036a_ps_enable(struct cy8c20236a_data_t *ps_data)
{
		int err =-EIO;

		if(ps_data->enabled)
			 return 0;

		err = cypress_cy8c2036a_ps_enable_hw(ps_data);
		if(err==0) {
				ps_data->enabled = 1;
				ps_data->pre_status=-1;	
			}
		else
				pr_err("%s: enable proximity sensor fail,err=%d\n",__func__,err);
		return err;         
}

static int cypress_cy8c2036a_ps_disable_hw(struct cy8c20236a_data_t *ps_data)
{
		 int err;
		 u8 buf[2]={CY8C_CTL_REG,POWER_DOWN};
	 
		 #ifndef PS_POLLING_ENABLE
		 struct cypress_cy8c20236a_platform_data *pdata=ps_data->pdata;
		 int irq;
		 #endif

		 #ifdef PS_POLLING_ENABLE
		 cancel_delayed_work(&ps_data->polling_work);
		 #else
		 if(pdata->p_out)
			{
				irq = gpio_to_irq(ps_data->p_out);
				free_irq(irq, ps_data);
			}
			#endif
			//for test
			#if 1
			buf[0]=CY8C_CTL_REG;
			buf[1]=0; 
			err = cy8c20236a_ps_i2c_read(ps_data,buf, 1);
			if(err < 0)
				return err;
			pr_info("%s:current control register's value is 0x%x\n",__func__,buf[0]);
			buf[0]=CY8C_CTL_REG;
			buf[1]=POWER_DOWN; 
			#endif
			//power down;
			err = cy8c20236a_ps_i2c_write(ps_data,buf, 1);
			return err;
}

static int cypress_cy8c2036a_ps_disable(struct cy8c20236a_data_t *ps_data)
{
		 int err;
		 pr_info("%s\n",__func__);

		 if(!ps_data->enabled)
			{
				 return 0;
			}

		 ps_data->enabled = 0;	 
		 err = cypress_cy8c2036a_ps_disable_hw(ps_data);
		 return err;
}

static int cypress_cy8c2036a_ps_open(struct inode *inode, struct file *file)
{
		 pr_info("%s\n",__func__);
		 
		 spin_lock(&cy8c20236a_ps_data->lock);
		 cy8c20236a_ps_data->dev_open_cnt++;
		 spin_unlock(&cy8c20236a_ps_data->lock);

		 file->private_data = cy8c20236a_ps_data;
		 return 0;
}

static int cypress_cy8c2036a_ps_release(struct inode *inode, struct file *file)
{
		struct cy8c20236a_data_t *ps_data;
		pr_info("%s\n",__func__);
		ps_data = (struct cy8c20236a_data_t *)file->private_data;

		spin_lock(&(ps_data->lock));
		if(ps_data->dev_open_cnt > 0) ps_data->dev_open_cnt--;
			spin_unlock(&(ps_data->lock));
		return 0;
}

static int cypress_cy8c2036a_ps_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
		void __user *argp = (void __user *)arg;
		//u8 buf[4];
		//int err;
		int interval;
		struct cy8c20236a_data_t *ps_data = file->private_data;

		switch (cmd) {
			case HP_PSENSOR_IOCTL_ENABLE:
				if (copy_from_user(&interval, argp, sizeof(interval)))
						 return -EFAULT;
				pr_info("%s: HP_PSENSOR_IOCTL_ENABLE, interval:%d\n", __func__, interval);
				if(interval)
					 cypress_cy8c2036a_ps_enable(ps_data);
				else
					 cypress_cy8c2036a_ps_disable(ps_data);
				break;
			case HP_PSENSOR_IOCTL_GET_ENABLED:
				interval = ps_data->enabled; 
				if(copy_to_user(argp, &interval, sizeof(interval)))
					 return -EINVAL;
				break;
			default:
				return -EINVAL;
		 }
		return 0;
}

static const struct file_operations cypress_cy8c2036a_ps_fops = {
	.owner = THIS_MODULE,
	.open = cypress_cy8c2036a_ps_open,
	.release = cypress_cy8c2036a_ps_release,
	.ioctl = cypress_cy8c2036a_ps_ioctl,
};

static struct miscdevice cypress_cy8c2036a_ps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &cypress_cy8c2036a_ps_fops,
};

static int hp_proximitysensor_input_init(struct cy8c20236a_data_t *ps_data)
{
		int err = -EIO;

		ps_data->input_dev = input_allocate_device();

		if(!ps_data->input_dev)
			{
			 err = -ENOMEM;
			 dev_err(&ps_data->client->dev, "input device allocate failed\n");
			 goto err0;
			}
		input_set_drvdata(ps_data->input_dev, ps_data);
		set_bit(EV_ABS, ps_data->input_dev->evbit);
		ps_data->input_dev->name = NAME;
		input_set_abs_params(ps_data->input_dev, ABS_DISTANCE, 0, 1, 0, 0);
		err = input_register_device(ps_data->input_dev);
		if(err < 0){
			 dev_err(&ps_data->client->dev,"unable to register input device %s\n",ps_data->input_dev->name);
			 goto err1;
			}
		return 0;
err1:
		input_set_drvdata(ps_data->input_dev, NULL);
		input_free_device(ps_data->input_dev);  
err0:	
		return err;
}

static void hp_proximitysensor_input_cleanup(struct cy8c20236a_data_t *ps_data)
{
		if(ps_data->input_dev)
			{
				input_set_drvdata(ps_data->input_dev, NULL);
				input_unregister_device(ps_data->input_dev);
				input_free_device(ps_data->input_dev);
				ps_data->input_dev = NULL;
			}
}

static ssize_t
cy8c20236a_get_enable_status(struct device *dev, struct device_attribute *attr, char *buf){
		struct cy8c20236a_data_t *ps_data = dev_get_drvdata(dev);	 
		return snprintf(buf, PAGE_SIZE, "%d\n",ps_data->enabled);
}

static ssize_t
cy8c20236a_set_enable_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
		struct cy8c20236a_data_t *ps_data = dev_get_drvdata(dev);
		int value;

		sscanf(buf, "%d", &value);

		if(value)
			{
				cypress_cy8c2036a_ps_enable(ps_data);
			}
		else
			 cypress_cy8c2036a_ps_disable(ps_data);
		return count;
}

static ssize_t
cy8c20236a_get_sensitivity(struct device *dev, struct device_attribute *attr, char *buf){
		 struct cy8c20236a_data_t *ps_data = dev_get_drvdata(dev);
		 u8 tempbuf[2];
		 int err=-1;

		 tempbuf[0] = CY8C_SENTIVISITY_REG;
		 tempbuf[1] = 0;
		 err = cy8c20236a_ps_i2c_read(ps_data,tempbuf, 1);
		 if(err < 0)
			{
				pr_err("%s: get sentitivity fail \n",__func__);
				return snprintf(buf, PAGE_SIZE, "%d\n", 0);
			}
		 return snprintf(buf, PAGE_SIZE, "%d\n",tempbuf[0]);
}
static ssize_t
cy8c20236a_set_sensitivity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
		 struct cy8c20236a_data_t *ps_data = dev_get_drvdata(dev);
		 u8 tempbuf[2];
		 int value;
		 u8 prestatus = 0;
		 int err = -1;

		 sscanf(buf, "%d", &value);

		 if(ps_data->enabled)
			{
				prestatus = 1;
				cypress_cy8c2036a_ps_disable(ps_data);
			}

			tempbuf[0] = CY8C_SENTIVISITY_REG;
			tempbuf[1] = value;
			err = cy8c20236a_ps_i2c_write(ps_data,tempbuf, 1);
			if(err < 0)
				{
					pr_err("%s: set sentitivity fail \n",__func__);
				}

			if(prestatus)
				{
					cypress_cy8c2036a_ps_enable(ps_data);
				}
		 return count;
}

static ssize_t
cy8c20236a_save_sensitivity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
		struct cy8c20236a_data_t *ps_data = dev_get_drvdata(dev);
		u8 tempbuf[2];
		int value;
		int err = -1;
		sscanf(buf,	"%d",&value);
		if(value <=	0)
			{
				pr_err("%s:	invalid	parameter, value =%d \n",__func__,value);
				return count;
			}
		tempbuf[0] = CY8C_CTL_REG;
		tempbuf[1] = 0;
		err	=	cy8c20236a_ps_i2c_read(ps_data,tempbuf,	1);
		if(err < 0)
			{
				pr_err("%s:	read ctrl	register fail	\n",__func__);
				return count;
			}
		tempbuf[1] = tempbuf[0]	|(1<<6);
		tempbuf[0] = CY8C_CTL_REG;
		err	=	cy8c20236a_ps_i2c_write(ps_data,tempbuf,1);
		if(err < 0)
			{
				pr_err("%s:	ctrl register	setting	cmd	fail \n",__func__);
			}
	
		return count;
}

static ssize_t
cy8c20236a_show_device_id(struct device *dev, struct device_attribute *attr, char *buf){
		return snprintf(buf, PAGE_SIZE, "0x%x\n",device_id);	 
}


static ssize_t
cy8c20236a_show_channel_data(struct device *dev, struct device_attribute *attr, char *buf){
	struct	cy8c20236a_data_t	*ps_data = dev_get_drvdata(dev);
	int size;
	#ifndef PS_POLLING_ENABLE
	u8 tempbuf[2];
	int err =-1;
	#endif

	mutex_lock(&ps_data->mutex_lock);

	if(!ps_data->enabled)
		{
		       mutex_unlock(&ps_data->mutex_lock);
			return snprintf(buf, PAGE_SIZE, "proximity sensor is disabled now, please enable it fisrt before data reading\n");
		}
	#ifndef PS_POLLING_ENABLE
 	//if not polling mode, the channels' buffer data is not the former data, so read these data from device;
	tempbuf[0] = CY8C_DATA_CH1_LSB_REG;
	tempbuf[1] = 0;
	err = cy8c20236a_ps_i2c_read(ps_data,tempbuf,1);
	if(err < 0)
		{
			pr_err("%s:read channel 1 LSB data error\n",__func__);
			goto error;
		}
	ch1_buf[0] = tempbuf[0];

	tempbuf[0] = CY8C_DATA_CH1_MSB_REG;
	tempbuf[1] = 0;
	err = cy8c20236a_ps_i2c_read(ps_data,tempbuf,1);
	if(err < 0)
		{
			pr_err("%s:read channel 1 MSB data error\n",__func__);
			goto error;
		}
	ch1_buf[1] = tempbuf[0];

	tempbuf[0] = CY8C_DATA_CH2_LSB_REG;
	tempbuf[1] = 0;
	err = cy8c20236a_ps_i2c_read(ps_data,tempbuf,1);
	if(err < 0)
		{
			pr_err("%s:read channel 2 LSB data error\n",__func__);
			goto error;
		}
	ch2_buf[0] = tempbuf[0];

	tempbuf[0] = CY8C_DATA_CH2_MSB_REG;
	tempbuf[1] = 0;
	err = cy8c20236a_ps_i2c_read(ps_data,tempbuf,1);
	if(err < 0)
		{
			pr_err("%s:read channel 2 MSB data error\n",__func__);
			goto error;
		}
	ch2_buf[1] = tempbuf[0];

	tempbuf[0] = CY8C_DATA_CH3_LSB_REG;
	tempbuf[1] = 0;
	err = cy8c20236a_ps_i2c_read(ps_data,tempbuf,1);
	if(err < 0)
		{
			pr_err("%s:read channel 3 LSB data error\n",__func__);
			goto error;
		}
	ch3_buf[0] = tempbuf[0];

	tempbuf[0] = CY8C_DATA_CH3_MSB_REG;
	tempbuf[1] = 0;
	err = cy8c20236a_ps_i2c_read(ps_data,tempbuf,1);
	if(err < 0)
		{
			pr_err("%s:read channel 3 MSB data error\n",__func__);
			goto error;
		}
	ch3_buf[1] = tempbuf[0];	
	#endif
	size = snprintf(buf, PAGE_SIZE, "Channel 1:MSB:%d LSB:%d value:%d\nChannel 2:MSB:%d LSB:%d value:%d\nChannel3:MSB:%d LSB:%d value:%d\n",
	ch1_buf[1],ch1_buf[0], ch1_buf[1]<<8|ch1_buf[0],ch2_buf[1],ch2_buf[0],ch2_buf[1]<<8|ch2_buf[0],ch3_buf[1],ch3_buf[0],ch3_buf[1]<<8|ch3_buf[0]);	
	mutex_unlock(&ps_data->mutex_lock);	
	return size;

	#ifndef PS_POLLING_ENABLE
error:
	mutex_unlock(&ps_data->mutex_lock);
	return err;
	#endif
}

static ssize_t
cy8c20236a_show_channel_threshold(struct device *dev, struct device_attribute *attr, char *buf){
	struct	cy8c20236a_data_t	*ps_data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "Channel1 threshold:MSB:%d LSB:%d value:%d\nChannel2 threshold:MSB:%d LSB:%d value:%d\nChannel3 threshold:MSB:%d LSB:%d value:%d\n",
					ps_data->resume_state[2],ps_data->resume_state[1],ps_data->resume_state[2] << 8 | ps_data->resume_state[1],
					ps_data->resume_state[4],ps_data->resume_state[3],ps_data->resume_state[4] << 8 | ps_data->resume_state[3],
					ps_data->resume_state[6],ps_data->resume_state[5],ps_data->resume_state[6] << 8 | ps_data->resume_state[5]);
}

static ssize_t
cy8c20236a_set_channel_threshold1(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
	struct	cy8c20236a_data_t	*ps_data = dev_get_drvdata(dev);
	int value;
	u8 msb_value,lsb_value;
	u8 prestatus = 0;
	int err = -1;
	u8 tempbuf[2];

	sscanf(buf, "%d", &value);

	msb_value = value >>8 &0xFF;
	lsb_value = value&0xFF;
     
	mutex_lock(&ps_data->mutex_lock);
	if(ps_data->enabled)
		{
			prestatus = 1;
			cypress_cy8c2036a_ps_disable(ps_data);
		}
	//set threshold
	tempbuf[0] = CY8C_THLD_LOW_CH1_REG;
	tempbuf[1] = lsb_value;

	err = cy8c20236a_ps_i2c_write(ps_data,tempbuf, 1);
	if(err < 0)
		{
			pr_err("%s: write channel 1 low threshold fail,err=%d\n",__func__,err);    	
			goto done;
		}
	ps_data->resume_state[1] = lsb_value;			
	tempbuf[0] = CY8C_THLD_HIGH_CH1_REG;
	tempbuf[1] = msb_value;

	err = cy8c20236a_ps_i2c_write(ps_data,tempbuf, 1);
	if(err < 0)
		{
			pr_err("%s: write channel 1 high threshold fail,err=%d\n",__func__,err);    	
			goto done;
		}
	ps_data->resume_state[2] =msb_value;
	
done:
	if(prestatus)
		{
		 	cypress_cy8c2036a_ps_enable(ps_data);
		}
	mutex_unlock(&ps_data->mutex_lock);	
	return count;	
}

static ssize_t
cy8c20236a_set_channel_threshold2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
	struct	cy8c20236a_data_t	*ps_data = dev_get_drvdata(dev);
	int value;
	u8 msb_value,lsb_value;
	u8 prestatus = 0;
	int err = -1;
	u8 tempbuf[2];

	sscanf(buf, "%d", &value);

	msb_value = value >>8 &0xFF;
	lsb_value = value&0xFF;
     
	mutex_lock(&ps_data->mutex_lock);
	if(ps_data->enabled)
		{
			prestatus = 1;
			cypress_cy8c2036a_ps_disable(ps_data);
		}
	//set threshold
	tempbuf[0] = CY8C_THLD_LOW_CH2_REG;
	tempbuf[1] = lsb_value;

	err = cy8c20236a_ps_i2c_write(ps_data,tempbuf, 1);
	if(err < 0)
		{
			pr_err("%s: write channel 2 low threshold fail,err=%d\n",__func__,err);    	
			goto done;
		}
	ps_data->resume_state[3] = lsb_value;			
	tempbuf[0] = CY8C_THLD_HIGH_CH2_REG;
	tempbuf[1] = msb_value;

	err = cy8c20236a_ps_i2c_write(ps_data,tempbuf, 1);
	if(err < 0)
		{
			pr_err("%s: write channel 2 high threshold fail,err=%d\n",__func__,err);    	
			goto done;
		}
	ps_data->resume_state[4] =msb_value;
	
done:
	if(prestatus)
		{
		 	cypress_cy8c2036a_ps_enable(ps_data);
		}
	mutex_unlock(&ps_data->mutex_lock);	
	return count;	
}

static ssize_t
cy8c20236a_set_channel_threshold3(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
	struct	cy8c20236a_data_t	*ps_data = dev_get_drvdata(dev);
	int value;
	u8 msb_value,lsb_value;
	u8 prestatus = 0;
	int err = -1;
	u8 tempbuf[2];

	sscanf(buf, "%d", &value);

	msb_value = value >>8 &0xFF;
	lsb_value = value&0xFF;
     
	mutex_lock(&ps_data->mutex_lock);
	if(ps_data->enabled)
		{
			prestatus = 1;
			cypress_cy8c2036a_ps_disable(ps_data);
		}
	//set threshold
	tempbuf[0] = CY8C_THLD_LOW_CH3_REG;
	tempbuf[1] = lsb_value;

	err = cy8c20236a_ps_i2c_write(ps_data,tempbuf, 1);
	if(err < 0)
		{
			pr_err("%s: write channel 3 low threshold fail,err=%d\n",__func__,err);    	
			goto done;
		}
	ps_data->resume_state[5] = lsb_value;			
	tempbuf[0] = CY8C_THLD_HIGH_CH3_REG;
	tempbuf[1] = msb_value;

	err = cy8c20236a_ps_i2c_write(ps_data,tempbuf, 1);
	if(err < 0)
		{
			pr_err("%s: write channel 3 high threshold fail,err=%d\n",__func__,err);    	
			goto done;
		}
	ps_data->resume_state[6] =msb_value;
	
done:
	if(prestatus)
		{
		 	cypress_cy8c2036a_ps_enable(ps_data);
		}
	mutex_unlock(&ps_data->mutex_lock);	
	return count;	
}
static int cypress_cy8c20236a_ps_gpio_config(u8 gpio_type, int gpio_no)
{
		int err=-1;

		if(gpio_no < 0)
			{
				goto err0;
			}

		switch(gpio_type){
		case GPIO_INT_TYPE:
			err = gpio_request(gpio_no, "proximity_int_gpio");
			if(err < 0)
				{
					pr_err("%s:request gpio[%d] fail \n",__func__,gpio_no);
					goto err0;
				}    
			err = gpio_tlmm_config(GPIO_CFG(gpio_no, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			if(err < 0)
				goto err1;
			break;
		case GPIO_RESET_TYPE:
			err = gpio_request(gpio_no, "proximity_reset_gpio");
			if(err < 0)
				{
					pr_err("%s:request reset gpio[%d] fail \n",__func__,gpio_no);
					goto err0;
				}  
				//default release reset;
				err = gpio_tlmm_config(GPIO_CFG(gpio_no, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);	
			if(err < 0)
				goto err1;
			gpio_set_value(gpio_no, 0); 			   
			break;
		default:
			goto err0;
	}		
		 return 0;
err1:
		 gpio_free(gpio_no);
err0:
		 return err;
}


static DEVICE_ATTR(proximity_enable, 0644,  cy8c20236a_get_enable_status, cy8c20236a_set_enable_status);
static DEVICE_ATTR(sensitivity, 0644,  cy8c20236a_get_sensitivity, cy8c20236a_set_sensitivity);
static DEVICE_ATTR(save_sensitivity,0644,NULL,cy8c20236a_save_sensitivity);
static DEVICE_ATTR(device_id,0644,cy8c20236a_show_device_id,NULL);
static DEVICE_ATTR(channel_data,0644,cy8c20236a_show_channel_data,NULL);
static DEVICE_ATTR(threshold1,0644,cy8c20236a_show_channel_threshold,cy8c20236a_set_channel_threshold1);
static DEVICE_ATTR(threshold2,0644,cy8c20236a_show_channel_threshold,cy8c20236a_set_channel_threshold2);
static DEVICE_ATTR(threshold3,0644,cy8c20236a_show_channel_threshold,cy8c20236a_set_channel_threshold3);


static int cy8c20236a_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)			
{
		int ret = -1;
		struct cy8c20236a_data_t *ps_data = cy8c20236a_ps_data;
		u8 buf[2];
		
		pr_info("%s\n", __func__);
	
		if(client->dev.platform_data == NULL) {
				dev_err(&client->dev, "platform data is NULL. exiting.\n");
				ret = -ENODEV;
				goto err0;
			}

		if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
				dev_err(&client->dev, "client not i2c capable\n");
				ret = -ENODEV;
				goto err0;
			}

		spin_lock_init(&ps_data->lock);
		mutex_init(&ps_data->mutex_lock);

		ps_data->resume_state[0] = 0x11;
		ps_data->resume_state[1] = 0x20;  //low ch1  
		ps_data->resume_state[2] = 0x02;  //high ch1 1024
		ps_data->resume_state[3] = 0x64;  //low ch2  100
		ps_data->resume_state[4] = 0x08; //high ch2  2048
		ps_data->resume_state[5] = 0xC8; //low ch3 200
		ps_data->resume_state[6] = 0x08; //high ch3 2048
		 
		ps_data->client = client;

		ps_data->pdata = kmalloc(sizeof(*ps_data->pdata), GFP_KERNEL);
		if(ps_data->pdata == NULL)
				goto err1;

		memcpy(ps_data->pdata, client->dev.platform_data, sizeof(*ps_data->pdata));

		if(ps_data->pdata->p_out)
			{
				ps_data->p_out = ps_data->pdata->p_out(GPIO_INT_TYPE);
				if(ps_data->p_out < 0)
					{
						pr_err("%s: get int gpio fail \n",__func__);
						goto err1;
					}
			}
		if(ps_data->pdata->reset_pin)
			{
				ps_data->reset_pin= ps_data->pdata->reset_pin(GPIO_RESET_TYPE);
				if(ps_data->reset_pin < 0)
					{
						pr_err("%s: get reset gpio fail \n",__func__);
						goto err1;
					}
			}
		pr_info("%s: int pin number = %d,reset pin number =%d \n",__func__, ps_data->p_out, ps_data->reset_pin);
		i2c_set_clientdata(client,ps_data);
		buf[0] = CY8C_DEVICE_ID_REG;
		buf[1] = 0;
		ret = cy8c20236a_ps_i2c_read(ps_data,buf,1);
		if(ret < 0)
			{
				pr_err("%s:there is no proximity sensor, return\n",__func__);
				goto err2;
			}
		device_id = buf[0];

		pr_info("%s: proximity sensor device ID =0x%x\n", __func__,buf[0]);

		ret = hp_proximitysensor_input_init(ps_data);
		if(ret < 0)
				goto err2;
			
		ret = misc_register(&cypress_cy8c2036a_ps_device);  
		if(ret < 0) {
				dev_err(&client->dev, "proximity misc device register failed\n");
				goto err3;
			}
	 
		//Int gpio initialization;
		ret = cypress_cy8c20236a_ps_gpio_config(GPIO_INT_TYPE,ps_data->p_out);
		if(ps_data->pdata->p_out && ret < 0)
			{
				pr_err("%s: gpio[%d] config fail \n",__func__,ps_data->p_out);
				goto err4;
			}
			 
		#ifdef PS_POLLING_ENABLE
		INIT_DELAYED_WORK(&ps_data->polling_work, cypress_cy8c20236a_work_func);
		#else
		wake_lock_init(&ps_data->ps_delayed_work_wake_lock, WAKE_LOCK_SUSPEND, "psensor_delayed_work");
		ps_data->psworkqueue = create_freezeable_workqueue(NAME);
		INIT_DELAYED_WORK(&ps_data->datareport,cypress_cy8c20236a_irq_delayed_work_func);
		#endif

		ret = device_create_file(&client->dev, &dev_attr_proximity_enable);
		ret = device_create_file(&client->dev, &dev_attr_sensitivity);
		ret	=	device_create_file(&client->dev, &dev_attr_save_sensitivity);
		ret = device_create_file(&client->dev, &dev_attr_device_id);
		ret = device_create_file(&client->dev, &dev_attr_channel_data);		
		ret = device_create_file(&client->dev, &dev_attr_threshold1);
		ret = device_create_file(&client->dev, &dev_attr_threshold2);
		ret = device_create_file(&client->dev, &dev_attr_threshold3);
		
		return 0;

err4:
		misc_deregister(&cypress_cy8c2036a_ps_device);
err3:
		hp_proximitysensor_input_cleanup(ps_data);
err2:
		i2c_set_clientdata(client, NULL);
		kfree(ps_data->pdata);
err1:
		kfree(ps_data);
err0:
		return ret;
}

static int cy8c20236a_remove(struct i2c_client *client)
{
		/* TODO: revisit ordering here once _probe order is finalized */
		struct cy8c20236a_data_t *ps_data = i2c_get_clientdata(client);

		if(ps_data)
			{
				device_remove_file(&client->dev, &dev_attr_proximity_enable);
				device_remove_file(&client->dev, &dev_attr_sensitivity);
				device_remove_file(&client->dev, &dev_attr_save_sensitivity);
				device_remove_file(&client->dev, &dev_attr_device_id);
				device_remove_file(&client->dev, &dev_attr_channel_data);
				device_remove_file(&client->dev, &dev_attr_threshold1);
				device_remove_file(&client->dev, &dev_attr_threshold2);
				device_remove_file(&client->dev, &dev_attr_threshold3);
				#ifndef PS_POLLING_ENABLE
				destroy_workqueue(ps_data->psworkqueue);
				wake_lock_destroy(&ps_data->ps_delayed_work_wake_lock);
				#endif

				misc_deregister(&cypress_cy8c2036a_ps_device);
				hp_proximitysensor_input_cleanup(ps_data);
				
				#ifdef PS_POLLING_ENABLE
				cancel_delayed_work(&ps_data->polling_work);
				#endif
				if(ps_data->pdata)
					{
						if(ps_data->pdata->p_out){
								#ifndef PS_POLLING_ENABLE
								int irq = gpio_to_irq(ps_data->p_out);
								free_irq(irq, ps_data);
								#endif	
								gpio_free(ps_data->p_out);
							 }
						if(ps_data->pdata->reset_pin)
								gpio_free(ps_data->reset_pin);

						mutex_destroy(&ps_data->mutex_lock);

						kfree(ps_data->pdata);
				}
						kfree(ps_data);
			 }  
		return 0;
}

#ifdef CONFIG_PM	/* if define power manager, define suspend and resume function */
static int cy8c20236a_suspend(struct i2c_client *client, pm_message_t mesg)
{
		struct cy8c20236a_data_t *ps_data = i2c_get_clientdata(client);
		//power down;
		if(ps_data->enabled)
				cypress_cy8c2036a_ps_disable_hw(ps_data);
		return 0;
}

static int  cy8c20236a_resume(struct i2c_client *client)
{
		struct cy8c20236a_data_t *ps_data = i2c_get_clientdata(client);
		//resume if need;
		if(ps_data->enabled)
				cypress_cy8c2036a_ps_enable_hw(ps_data);
		return 0;
}
#else
#define	cy8c20236a_suspend 	NULL
#define cy8c20236a_resume		NULL
#endif		/*ifdef CONFIG_PM end*/


static const struct i2c_device_id cy8c20236a_id[] = {
	{"cy8c20236a", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, cy8c20236a_id);

static struct i2c_driver cy8c20236a_driver = {
	.driver = {
			 .name = "cy8c20236a",
			 },
	.probe = cy8c20236a_probe,
	.remove = __devexit_p(cy8c20236a_remove),
	.resume = cy8c20236a_resume,
	.suspend = cy8c20236a_suspend,
	.id_table = cy8c20236a_id,
};

static int __init hp_proximitysensor_init(void)
{
		int ret;
		struct cy8c20236a_data_t *ps_data;

		ps_data = kzalloc(sizeof(*ps_data), GFP_KERNEL);
		if(ps_data == NULL) {
				pr_err("%s: failed to allocate memory for module data\n", __func__);
				return -ENOMEM;
			}
	
		cy8c20236a_ps_data = ps_data;
	
		/* register the i2c driver for proximity sensor */
		ret = i2c_add_driver(&cy8c20236a_driver);
		if (ret) goto ADD_DRV_FAIL;

		return 0;

ADD_DRV_FAIL:
		i2c_del_driver(&cy8c20236a_driver);
		printk(KERN_ERR "Add cm8c20236a driver error\n");
		return ret;
}

static void __exit hp_proximitysensor_exit(void)
{
		i2c_del_driver(&cy8c20236a_driver);
		return;
}

module_init(hp_proximitysensor_init);
module_exit(hp_proximitysensor_exit);

MODULE_DESCRIPTION("HP Proximity Sensor Driver");
MODULE_AUTHOR("HP");
MODULE_LICENSE("GPL");
