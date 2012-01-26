/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name          : lsm303dlh_acc.c
* Authors            : MSH - Motion Mems BU - Application Team
*		     : Carmine Iascone (carmine.iascone@st.com)
*		     : Matteo Dameno (matteo.dameno@st.com)
* Version            : V 1.1.0
* Date               : 19/03/2010
* Description        : LSM303DLH 6D module sensor API
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

26_4_2010: lsm303dlh_acc_device_power_off now calling CTRL_REG1 to set power off

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
#include <linux/i2c/lsm303dlh.h>
#include <linux/hp_sensors.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>



//guoye: add for special interrupt test mode
//typedef struct _interrupt_test_buf_
//{
//	int direction;//1 for read, 0 for write
//	char buf[6];
//	int buf_len;
//} INTERRUPT_TEST_BUF;
//#define LSM303DLH_ACC_IOCTL_INTERRUPT_TEST_MODE _IOWR(LSM303DLH_ACC_IOCTL_BASE, 9, INTERRUPT_TEST_BUF)
//#define LSM303DLH_ACC_IOCTL_GET_INT1_SRC _IOR(LSM303DLH_ACC_IOCTL_BASE, 10, int)

#define NAME			HP_GSENSOR_NAME //"lsm303dlh_acc"  //?? NAME of device shoudl be setup in BSP file. 

//#define pr_info	pr_err

/** Maximum polled-device-reported g value */
#define G_MAX			8000

#define SHIFT_ADJ_2G		4
#define SHIFT_ADJ_4G		3
#define SHIFT_ADJ_8G		2

#define AXISDATA_REG		0x28

/* ctrl 1: pm2 pm1 pm0 dr1 dr0 zenable yenable zenable */
#define CTRL_REG1		0x20	/* power control reg */
#define CTRL_REG2		0x21	/* power control reg */
#define CTRL_REG3		0x22	/* power control reg */
#define CTRL_REG4		0x23	/* interrupt control reg */
//[HPP]guoye-For interrupt mode
#define CTRL_REG5		0x24	/* sleep-to-wake function reg*/
#define INT1_THRESH		0x32	/* interrupt 1 threshold reg */
#define INT1_SRC		0x31	/* interrupt 1 source reg */
#define INT1_DURATION	0x33	/* interrupt 1 duration reg */
#define INT1_CFG		0x30	/* interrupt 1 config reg */

#define INT_MASK_XL 0x01
#define INT_MASK_XH 0x02
#define INT_MASK_YL 0x04
#define INT_MASK_YH 0x08
#define INT_MASK_ZL 0x10
#define INT_MASK_ZH 0x20

#define INT1_DEFAULT_DURATION 0x03;

static void hp_gsensor_interrupt_task(struct work_struct *work);
static	DECLARE_WORK(gsensor_work, hp_gsensor_interrupt_task);
//[HPP]guoye

#define FUZZ			0
#define FLAT			0
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
#define AUTO_INCREMENT		0x80

static struct {
	unsigned int cutoff;
	unsigned int mask;
} odr_table[] = {
	{
	3,	LSM303DLH_ACC_PM_NORMAL | LSM303DLH_ACC_ODR1000}, {
	10,	LSM303DLH_ACC_PM_NORMAL | LSM303DLH_ACC_ODR400}, {
	20,	LSM303DLH_ACC_PM_NORMAL | LSM303DLH_ACC_ODR100}, {
	100,	LSM303DLH_ACC_PM_NORMAL | LSM303DLH_ACC_ODR50}, {
	200,	LSM303DLH_ACC_ODR1000 | LSM303DLH_ACC_ODR10}, {
	500,	LSM303DLH_ACC_ODR1000 | LSM303DLH_ACC_ODR5}, {
	1000,	LSM303DLH_ACC_ODR1000 | LSM303DLH_ACC_ODR2}, {
	2000,	LSM303DLH_ACC_ODR1000 | LSM303DLH_ACC_ODR1}, {
	0,	LSM303DLH_ACC_ODR1000 | LSM303DLH_ACC_ODRHALF},};


/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
static struct lsm303dlh_acc_data *lsm303dlh_acc_misc_data=NULL;

static int lsm303dlh_acc_i2c_read(struct lsm303dlh_acc_data *acc,
				  u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
		 .flags = acc->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = acc->client->addr,
		 .flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm303dlh_acc_i2c_write(struct lsm303dlh_acc_data *acc,
				   u8 *buf, int len)
{
	int err;

	//pr_info("%s: slave address = %x \n ",__func__, acc->client->addr);

//send_cb is not defined in lsm303a.c, so acc->send_cb is NULL, mark it out to avoid confusion, Tandi
/*	if (acc->ext_adap_enabled )
	{
		if (acc->send_cb && acc->ext_handle)
		{
			pr_info("%s: call ext send callback\n ",__func__);
			err = acc->send_cb(acc->ext_handle, acc->client->addr, len, buf);
		}
		else
		{
			pr_info("%s: call ext send callback failed: ext_handle:%p, send_cb:%p\n ",__func__, acc->ext_handle, acc->send_cb);
			err= 0;
		}
		
	}
	else */
	{
		int tries = 0;
		struct i2c_msg msgs[] = {
			{
				.addr = acc->client->addr,
				.flags = acc->client->flags & I2C_M_TEN,
				.len = len + 1,
				.buf = buf,
			},
		};

		do {
			err = i2c_transfer(acc->client->adapter, msgs, 1);
			if (err != 1)
				msleep_interruptible(I2C_RETRY_DELAY);
		} while ((err != 1) && (++tries < I2C_RETRIES));
	}
	
	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm303dlh_acc_hw_init(struct lsm303dlh_acc_data *acc)
{
	int err = -1;
	u8 buf[6];

	//pr_info("%s: \n", __func__);
	buf[0] = (AUTO_INCREMENT | CTRL_REG1);
	//buf[0] = CTRL_REG1;
	buf[1] = acc->resume_state[0];
	buf[2] = acc->resume_state[1];
	buf[3] = acc->resume_state[2];
	buf[4] = acc->resume_state[3];
	buf[5] = acc->resume_state[4];
	err = lsm303dlh_acc_i2c_write(acc, buf, 5);
	if (err < 0)
		return err;

	acc->hw_initialized = 1;

	return 0;
}

static void lsm303dlh_acc_device_power_off(struct lsm303dlh_acc_data *acc)
{
	int err;
	u8 buf[2] = { CTRL_REG1, LSM303DLH_ACC_PM_OFF };

	err = lsm303dlh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed\n");

	if (acc->pdata->power_off) {
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	return;
}

static int lsm303dlh_acc_device_power_on(struct lsm303dlh_acc_data *acc)
{
	int err;

	//pr_info("%s: \n", __func__);
	if (acc->pdata->power_on) {
		//pr_info("%s: power_on\n", __func__);
		err = acc->pdata->power_on();
		if (err < 0)
			return err;
	}

//When power on ACC, we need to know whether this action comes from Gyro or native ACC, if from Gyro, we should not do HW init, otherwise I2C error will occur, Tandi
//	if (!acc->hw_initialized){
	if (!acc->hw_initialized && (!acc->ext_adap_enabled)) {
		err = lsm303dlh_acc_hw_init(acc);
		if (err < 0) {
			lsm303dlh_acc_device_power_off(acc);
			return err;
		}
	}

	return 0;
}

int lsm303dlh_acc_update_g_range(struct lsm303dlh_acc_data *acc, u8 new_g_range)
{
	int err;
	u8 shift;
	u8 buf[2];

	switch (new_g_range) {
	case LSM303DLH_G_2G:
		shift = SHIFT_ADJ_2G;
		break;
	case LSM303DLH_G_4G:
		shift = SHIFT_ADJ_4G;
		break;
	case LSM303DLH_G_8G:
		shift = SHIFT_ADJ_8G;
		break;
	default:
		return -EINVAL;
	}

	//if (atomic_read(&acc->enabled)) 
	{
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = CTRL_REG4;
		buf[1] = new_g_range;
		err = lsm303dlh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	}

	acc->resume_state[3] = new_g_range;
	acc->shift_adj = shift;

	return 0;
}

int lsm303dlh_acc_update_odr(struct lsm303dlh_acc_data *acc, int poll_interval)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(odr_table); i++) {
		config[1] = odr_table[i].mask;
		if (poll_interval < odr_table[i].cutoff)
			break;
	}

	config[1] |= LSM303DLH_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	//if (atomic_read(&acc->enabled)) 
	{
		config[0] = CTRL_REG1;
		err = lsm303dlh_acc_i2c_write(acc, config, 1);
		if (err < 0)
			return err;
	}

	acc->resume_state[0] = config[1];

	return 0;
}

static int lsm303dlh_acc_get_acceleration_data(struct lsm303dlh_acc_data *acc,
					       int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	int hw_d[3] = { 0 };

	acc_data[0] = (AUTO_INCREMENT | AXISDATA_REG);
	err = lsm303dlh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (int) (((acc_data[1]) << 8) | acc_data[0]);
	hw_d[1] = (int) (((acc_data[3]) << 8) | acc_data[2]);
	hw_d[2] = (int) (((acc_data[5]) << 8) | acc_data[4]);

	hw_d[0] = (hw_d[0] & 0x8000) ? (hw_d[0] | 0xFFFF0000) : (hw_d[0]);
	hw_d[1] = (hw_d[1] & 0x8000) ? (hw_d[1] | 0xFFFF0000) : (hw_d[1]);
	hw_d[2] = (hw_d[2] & 0x8000) ? (hw_d[2] | 0xFFFF0000) : (hw_d[2]);

	hw_d[0] >>= acc->shift_adj;
	hw_d[1] >>= acc->shift_adj;
	hw_d[2] >>= acc->shift_adj;

	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		  : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		  : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		  : (hw_d[acc->pdata->axis_map_z]));

	return err;
}

static void lsm303dlh_acc_report_values(struct lsm303dlh_acc_data *acc,
					int *xyz)
{
	// pr_info("%s: acc x, y, z -- %d, %d, %d\n", __func__, xyz[0], xyz[1], xyz[2]);
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static int lsm303dlh_acc_enable(struct lsm303dlh_acc_data *acc)
{
	int err;

	int cur_count;

	spin_lock_irq(&acc->enable_lock);
	cur_count = acc->enable_count;
	acc->enable_count++;
	spin_unlock_irq(&acc->enable_lock);

	//pr_info("%s: ext_adap_enabled:%d, cur_count:%d\n", __func__, acc->ext_adap_enabled, cur_count);

	if (!cur_count) {
		//pr_info("%s: enable  acc\n", __func__);
		err = lsm303dlh_acc_device_power_on(acc);
		if (err < 0) {
			spin_lock_irq(&acc->enable_lock);
			acc->enable_count=0;
			spin_unlock_irq(&acc->enable_lock);
			return err;
		}
	}
	if (!acc->ext_adap_enabled)
	{
		schedule_delayed_work(&acc->input_work, msecs_to_jiffies(acc->pdata->poll_interval));
	}			
	else
	{
		cancel_delayed_work_sync(&acc->input_work);
	}

	return 0;
}

static int lsm303dlh_acc_disable(struct lsm303dlh_acc_data *acc)
{
	int cur_count;

	spin_lock_irq(&acc->enable_lock);
	if (acc->enable_count>0)
		acc->enable_count--;
	cur_count = acc->enable_count;
	spin_unlock_irq(&acc->enable_lock);

	//pr_info("%s: ext_adap_enabled:%d, cur_count:%d\n", __func__, acc->ext_adap_enabled, cur_count);

	if (!cur_count) {
		//pr_info("%s: disable acc\n", __func__);
		cancel_delayed_work_sync(&acc->input_work);
		lsm303dlh_acc_device_power_off(acc);
	}
	else
	{
		if (!acc->ext_adap_enabled)
		{
    //if enable acc from gyro request, we need to re-initial acc, Tandi
			lsm303dlh_acc_hw_init(acc);
			//pr_info("%s: schedule_delayed_work\n", __func__);
			schedule_delayed_work(&acc->input_work, msecs_to_jiffies(acc->pdata->poll_interval));
		}			
	}	

	return 0;
}

int lsm303dlh_acc_enable_ext(struct lsm303dlh_acc_data *acc)
{
	int err=0;

	//pr_info("%s: \n", __func__);

	if (!acc->ext_adap_enabled)
	{
		acc->ext_adap_enabled = 1;
		err = lsm303dlh_acc_enable(acc);
	}

	return err;
}
EXPORT_SYMBOL(lsm303dlh_acc_enable_ext);

int lsm303dlh_acc_disable_ext(struct lsm303dlh_acc_data *acc)
{
	int err=0;

	//pr_info("%s: \n", __func__);
	if (acc->ext_adap_enabled)
	{
		acc->ext_adap_enabled = 0;
		err = lsm303dlh_acc_disable(acc);
	}

	return err;
}
EXPORT_SYMBOL(lsm303dlh_acc_disable_ext);

struct lsm303dlh_acc_data * lsm303dlh_acc_get_instance_ext(void)
{
	return lsm303dlh_acc_misc_data;
}
EXPORT_SYMBOL(lsm303dlh_acc_get_instance_ext);



static int lsm303dlh_acc_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = lsm303dlh_acc_misc_data;

	return 0;
}

//guoye: for interrupt test
/*static int interrupt_test(struct lsm303dlh_acc_data *acc,INTERRUPT_TEST_BUF *test_buf)
{
	int err = -1;
	
	printk("test_buf->direction = %d, test_buf->buf[0] = 0x%x, test_buf->buf[1] = 0x%x \n",test_buf->direction, test_buf->buf[0],test_buf->buf[1]);
		
	if( test_buf->direction == 1 )//read
		err = lsm303dlh_acc_i2c_read(acc,(u8 *)(test_buf->buf),test_buf->buf_len);
	else if( test_buf->direction == 0 )//write
		err = lsm303dlh_acc_i2c_write(acc,(u8 *)(test_buf->buf),test_buf->buf_len);
	
	if (err < 0)
		return err;
	
	return 0;
}

static int interrupt_test_get_INT1_SRC(struct lsm303dlh_acc_data *acc,int *val)
{
		u8 acc_data[6];
		int err = -1;

	acc_data[0] = (0x31);
	acc_data[1] = 0x0;
	err = lsm303dlh_acc_i2c_read(acc, acc_data, 1);
	if (err < 0)
		return err;
		
	printk("get acc_data[0] = 0x%x,acc_data[0] = 0x%x \n",acc_data[0],acc_data[1]);	
	
	*val = (int)acc_data[0];

	printk("val = 0x%x \n",*val);	
	
	return 0;
}*/

//[HPP]guoye-For interrupt mode
static void event_queue_init(struct int_event_queue *queue, const char *name)
{
	spin_lock_init(&queue->lock);
	queue->p_in = 0;
	queue->p_out = 0;
	queue->name = name;
	init_waitqueue_head(&queue->in_wait);
	init_waitqueue_head(&queue->out_wait);
}

//static struct lsm303dlh_acc_data *g_acc;

static void hp_gsensor_interrupt_task(struct work_struct *work)
{
	struct lsm303dlh_acc_data *acc;
	int err;
	//unsigned long flags;
	u8 buf[2];
	int event;
	
	//pr_info("%s\n", __func__);
	acc = lsm303dlh_acc_misc_data;
	
	buf[0] = INT1_SRC;
	buf[1] = 0;
	err = lsm303dlh_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		return;
		
	//pr_info("guoye: buf[0] = 0x%x, buf[1] = 0x%x \n",buf[0], buf[1] );
	
	event = buf[0];

	if( !(event & 0x40) )
	{
		pr_err("guoye: it's not a valid interrupt\n");
		goto error;
	}

	event &= (0x3F);//clear IA bit

	//pr_info("guoye: int_resume_state[0] = %d \n",acc->int_resume_state[0] );
	
	if( (event & EVENT_XL_BIT) && !(acc->int_resume_state[0] & INT_MASK_XL) )//event XL
		event &= (~EVENT_XL_BIT);
	if( (event & EVENT_XH_BIT) && !(acc->int_resume_state[0] & INT_MASK_XH) )//event XH
		event &= (~EVENT_XH_BIT);
	if( (event & EVENT_YL_BIT) && !(acc->int_resume_state[0] & INT_MASK_YL) )//event YL
		event &= (~EVENT_YL_BIT);
	if( (event & EVENT_YH_BIT) && !(acc->int_resume_state[0] & INT_MASK_YH) )//event YH
		event &= (~EVENT_YH_BIT);
	if( (event & EVENT_ZL_BIT) && !(acc->int_resume_state[0] & INT_MASK_ZL) )//event ZL
		event &= (~EVENT_ZL_BIT);
	if( (event & EVENT_ZH_BIT) && !(acc->int_resume_state[0] & INT_MASK_ZL) )//event ZH
		event &= (~EVENT_ZH_BIT);
	
	//pr_info("guoye: event = %d \n",event );
	
	//spin_lock_irqsave(&acc->event_q.lock, flags);
	spin_lock(&acc->event_q.lock);
	
	//pr_info("guoye:  acc->event_q.p_out= %d; acc->event_q.p_in = %d\n",acc->event_q.p_out, acc->event_q.p_in );

	acc->event_q.event[acc->event_q.p_out] = event;
	if( acc->event_q.p_out == (sizeof(acc->event_q.event)/sizeof(int))-1 )//to the end
		acc->event_q.p_out = 0;
	else
		acc->event_q.p_out ++;
			
	if( acc->event_q.p_out == acc->event_q.p_in )
	{
		if( acc->event_q.p_in == (sizeof(acc->event_q.event)/sizeof(int))-1 )//to the end
			acc->event_q.p_in = 0;
		else
			acc->event_q.p_in ++;
	}
	
	//pr_info("guoye:  acc->event_q.p_out= %d; acc->event_q.p_in = %d\n",acc->event_q.p_out, acc->event_q.p_in );
			
	//spin_unlock_irqrestore(&acc->event_q.lock, flags);
	spin_unlock(&acc->event_q.lock);

	wake_up_interruptible(&acc->event_q.in_wait);
error:	
	enable_irq(acc->irq);

	//pr_info("guoye: %s end\n", __func__);
}

static irqreturn_t hp_lsm303dlh_interrupt_handler(int irq, void *dev_id)
{
	disable_irq_nosync(irq);
	//printk("guoye: interrupt occur\n");
	//g_acc = (struct lsm303dlh_acc_data*)dev_id; 
	schedule_work(&gsensor_work);   
	//enable_irq(irq);
	return IRQ_HANDLED;

/*	struct lsm303dlh_acc_data *acc;
	int err;
	unsigned long flags;
	u8 buf[2];
	int event;

	acc = (struct lsm303dlh_acc_data *)dev_id;


	buf[0] = INT1_SRC;
	buf[1] = 0;
	err = lsm303dlh_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		return err;

	event = buf[0];
	event &= (0x3F);//clear IA bit

	if( (event & EVENT_XL_BIT) && !(acc->int_resume_state[0] & INT_MASK_XL) )//event XL
		event &= (~EVENT_XL_BIT);
	if( (event & EVENT_XH_BIT) && !(acc->int_resume_state[0] & INT_MASK_XH) )//event XH
		event &= (~EVENT_XH_BIT);
	if( (event & EVENT_YL_BIT) && !(acc->int_resume_state[0] & INT_MASK_YL) )//event YL
		event &= (~EVENT_YL_BIT);
	if( (event & EVENT_YH_BIT) && !(acc->int_resume_state[0] & INT_MASK_YH) )//event YH
		event &= (~EVENT_YH_BIT);
	if( (event & EVENT_ZL_BIT) && !(acc->int_resume_state[0] & INT_MASK_ZL) )//event ZL
		event &= (~EVENT_ZL_BIT);
	if( (event & EVENT_ZH_BIT) && !(acc->int_resume_state[0] & INT_MASK_ZL) )//event ZH
		event &= (~EVENT_ZH_BIT);

	spin_lock_irqsave(&acc->event_q.lock, flags);

	acc->event_q.event[acc->event_q.p_out] = event;
	if( acc->event_q.p_out == sizeof(acc->event_q.event)-1 )//to the end
		acc->event_q.p_out = 0;
	else
		acc->event_q.p_out ++;
		
	if( acc->event_q.p_out == acc->event_q.p_in )
	{
		if( acc->event_q.p_in == sizeof(acc->event_q.event)-1 )//to the end
			acc->event_q.p_in = 0;
		else
			acc->event_q.p_in ++;
	}
		
	spin_unlock_irqrestore(&acc->event_q.lock, flags);

	return IRQ_HANDLED;*/
}


static int hp_lsm303dlh_acc_interrupt_start_event(struct lsm303dlh_acc_data *acc, int event)
{
	int err;
	u8 buf[2];

	//pr_info("%s: \n", __func__);

	if( event <= 0 )
		return -EINVAL;

	//if (atomic_read(&acc->enabled)) 
	{
		buf[0] = INT1_DURATION;
		buf[1] = 0;//INT1_DEFAULT_DURATION;
		err = lsm303dlh_acc_i2c_write(acc,buf,1);
		if( err < 0 )
			return err;

		//add start
		/*buf[0] = CTRL_REG1;
		buf[1] = 0x2F;
		err = lsm303dlh_acc_i2c_write(acc,buf,1);
		if( err < 0 )
			return err;
		buf[0] = CTRL_REG2;
		buf[1] = 0;
		err = lsm303dlh_acc_i2c_write(acc,buf,1);
		if( err < 0 )
			return err;*/
		//add end

		buf[0] = CTRL_REG3;
		buf[1] = 0x00;
		err = lsm303dlh_acc_i2c_write(acc,buf,1);
		if( err < 0 )
			return err;

		buf[0] = CTRL_REG5;
		buf[1] = 0x00;
		err = lsm303dlh_acc_i2c_write(acc,buf,1);
		if( err < 0 )
			return err;

		//pr_info("%s: event = 0x%x\n",__func__,event);
		
		buf[0] = INT1_CFG;
		buf[1] = (u8)event;//0x0a;
		err = lsm303dlh_acc_i2c_write(acc,buf,1);
		if( err < 0 )
			return err;
	}

	acc->resume_state[2] = 0x0;
	acc->int_resume_state[3] = INT1_DEFAULT_DURATION;
	acc->int_resume_state[0] = 0x0a;//buf[1];
	
	return 0;
}

static int hp_lsm303dlh_acc_interrupt_stop_event(struct lsm303dlh_acc_data *acc, int event)
{
	int err;
	u8 buf[2];

	//pr_info("%s: \n", __func__);

	if( event < 0 )
		return -EINVAL;

	//if (atomic_read(&acc->enabled)) 
	{
		buf[0] = INT1_CFG;
		buf[1] = acc->int_resume_state[0] & (~event);
		err = lsm303dlh_acc_i2c_write(acc,buf,1);
		if( err < 0 )
			return err;
	}

	acc->int_resume_state[0] &= (~event);
	
	return 0;
}


static int hp_lsm303dlh_acc_interrupt_set_threshold(struct lsm303dlh_acc_data *acc, int threshold)
{
	int err;
	u8 buf[2];
	int full_scale;

	//pr_info("%s: \n", __func__);

	switch(acc->resume_state[3])
	{
	case LSM303DLH_G_2G:
		full_scale = 2000;
		break;
	case LSM303DLH_G_4G:
		full_scale = 4000;
		break;
	case LSM303DLH_G_8G:
		full_scale = 8000;
		break;
	default:
		return -EINVAL;
	}

	if( threshold >= full_scale || threshold <= 0 )
		return -EINVAL;
	
	//if (atomic_read(&acc->enabled)) 
	{
		buf[0] = INT1_THRESH;
		buf[1] = (u8)((threshold*128)/full_scale);
		//pr_info("%s: threshold = %d\n",__func__, buf[1]);
		err = lsm303dlh_acc_i2c_write(acc,buf,1);
		if( err < 0 )
			return err;
	}
	
	acc->int_resume_state[2] = buf[1];

	return 0;
}

static int hp_lsm303dlh_acc_interrupt_get_event(struct lsm303dlh_acc_data *acc, int *event)
{
	//unsigned long flags;

	if( acc->event_q.p_in == acc->event_q.p_out )
	{
		//pr_info("guoye: wait\n");
		
		if (wait_event_interruptible(acc->event_q.in_wait, (acc->event_q.p_in != acc->event_q.p_out)))
			return -ERESTARTSYS;
	}
	
	//pr_info("guoye: wait end\n");

	//spin_lock_irqsave(&acc->event_q.lock, flags);
	spin_lock(&acc->event_q.lock);
	*event = acc->event_q.event[acc->event_q.p_in];
	//pr_info("guoye: *event = %d \n", *event);
	if( acc->event_q.p_in == (sizeof(acc->event_q.event)/sizeof(int))-1  )
		acc->event_q.p_in = 0;
	else
		acc->event_q.p_in ++;
	//spin_unlock_irqrestore(&acc->event_q.lock, flags);
	spin_unlock(&acc->event_q.lock);

	return 0;
}

static int lsm303dlh_acc_misc_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 buf[4];
	int err;
	int interval;
	struct lsm303dlh_acc_data *acc = file->private_data;

	switch (cmd) {
	case HP_ACCELEROMETERSENSOR_IOCTL_GET_DELAY:
	case LSM303DLH_ACC_IOCTL_GET_DELAY:
		interval = acc->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case HP_ACCELEROMETERSENSOR_IOCTL_SET_DELAY:
	case LSM303DLH_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 200)
			return -EINVAL;

		acc->pdata->poll_interval =
		    max(interval, acc->pdata->min_interval);
		err = lsm303dlh_acc_update_odr(acc, acc->pdata->poll_interval);
		/* TODO: if update fails poll is still set */
		if (err < 0)
			return err;
		break;

	case HP_ACCELEROMETERSENSOR_IOCTL_ENABLE: 
	case LSM303DLH_ACC_IOCTL_SET_ENABLE:
		//pr_info("%s: LSM303DLH_ACC_IOCTL_SET_ENABLE\n", __func__);
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval > 1)
			return -EINVAL;

		//pr_info("%s: LSM303DLH_ACC_IOCTL_SET_ENABLE, interval:%d\n", __func__, interval);
		if (interval)
			lsm303dlh_acc_enable(acc);
		else
			lsm303dlh_acc_disable(acc);
		break;

	case HP_ACCELEROMETERSENSOR_IOCTL_GET_ENABLED: 
	case LSM303DLH_ACC_IOCTL_GET_ENABLE:
		interval = acc->enable_count; //atomic_read(&acc->enabled);
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EINVAL;
		break;

	case LSM303DLH_ACC_IOCTL_SET_G_RANGE:
		if (copy_from_user(&buf, argp, 1))
			return -EFAULT;
		err = lsm303dlh_acc_update_g_range(acc, buf[0]);
		if (err < 0)
			return err;
		break;
	//[HPP]guoye- For interrupt event
	case HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_START_EVENT:
		//pr_info("%s: HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_START_EVENT\n", __func__);
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;

		err = hp_lsm303dlh_acc_interrupt_start_event(acc, interval);
		if( err < 0 )
			return err;
		break;
	case HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_STOP_EVENT:
		//pr_info("%s: HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_ENALE\n", __func__);
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;

		err = hp_lsm303dlh_acc_interrupt_stop_event(acc, interval);
		if( err < 0 )
			return err;
		break;
	case HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_SET_THRESHOLD:
		//pr_info("%s: HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_SET_THRESHOLD\n", __func__);
		if( copy_from_user(&interval, argp, sizeof(int)) )
			return -EFAULT;

		err = hp_lsm303dlh_acc_interrupt_set_threshold(acc,interval);
		if( err < 0 )
			return err;
		break;
	case HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_GET_EVENT:
		//pr_info("%s: HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_GET_EVENT\n", __func__);
		err = hp_lsm303dlh_acc_interrupt_get_event(acc,&interval);
		if( err < 0 )
			return err;
	
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EINVAL;
		break;
	/*case LSM303DLH_ACC_IOCTL_INTERRUPT_TEST_MODE:
		printk("%s LSM303DLH_ACC_IOCTL_INTERRUPT_TEST_MODE\n",__func__);
		{
			INTERRUPT_TEST_BUF test_buf;
			if (copy_from_user(&test_buf, argp, sizeof(test_buf)) )
				return -EFAULT;
			err = interrupt_test(acc, &test_buf);
			if( err < 0 )
				return err;

			if( test_buf.direction == 1 )//read
			{
				printk("before copy_to_user: test_buf.buf[0] = 0x%x,test_buf.buf[1] = 0x%x \n",test_buf.buf[0],test_buf.buf[1]);
				if (copy_to_user(argp, &test_buf, sizeof(test_buf)))
					return -EINVAL;
			}
		}
		break;
	case LSM303DLH_ACC_IOCTL_GET_INT1_SRC:
		printk("%s LSM303DLH_ACC_IOCTL_GET_INT1_SRC\n",__func__);
		{
			int val;
			
			err = interrupt_test_get_INT1_SRC(acc,&val);
			if( err < 0 )
				return err;
				
			if (copy_to_user(argp, &val, sizeof(val)))
				return -EINVAL;
		}
		break;*/
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations lsm303dlh_acc_misc_fops = {
	.owner = THIS_MODULE,
	.open = lsm303dlh_acc_misc_open,
	.ioctl = lsm303dlh_acc_misc_ioctl,
};

static struct miscdevice lsm303dlh_acc_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &lsm303dlh_acc_misc_fops,
};

static void lsm303dlh_acc_input_work_func(struct work_struct *work)
{
	struct lsm303dlh_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			    struct lsm303dlh_acc_data, input_work);

	mutex_lock(&acc->lock);
	err = lsm303dlh_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		lsm303dlh_acc_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work,
			      msecs_to_jiffies(acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

#ifdef LSM303DLH_ACC_OPEN_ENABLE
int lsm303dlh_acc_input_open(struct input_dev *input)
{
	struct lsm303dlh_acc_data *acc = input_get_drvdata(input);

	return lsm303dlh_acc_enable(acc);
}

void lsm303dlh_acc_input_close(struct input_dev *dev)
{
	struct lsm303dlh_acc_data *acc = input_get_drvdata(dev);

	lsm303dlh_acc_disable(acc);
}
#endif

static int lsm303dlh_acc_validate_pdata(struct lsm303dlh_acc_data *acc)
{
	//pr_info("%s: \n", __func__);
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
					acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
	    acc->pdata->axis_map_y > 2 || acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			acc->pdata->axis_map_x, acc->pdata->axis_map_y,
			acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1 ||
	    acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			acc->pdata->negate_x, acc->pdata->negate_y,
			acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm303dlh_acc_input_init(struct lsm303dlh_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, lsm303dlh_acc_input_work_func);

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocate failed\n");
		goto err0;
	}

#ifdef LSM303DLH_ACC_OPEN_ENABLE
	acc->input_dev->open = lsm303dlh_acc_input_open;
	acc->input_dev->close = lsm303dlh_acc_input_close;
#endif

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	acc->input_dev->name = NAME;

	err = input_register_device(acc->input_dev); //??regist a input event device
	if (err) {
		dev_err(&acc->client->dev,
			"unable to register input polled device %s\n",
			acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void lsm303dlh_acc_input_cleanup(struct lsm303dlh_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

//[HPP]guoye-Move gpio config to board-msm8x60.c
//[HPP]guoye-For interrupt mode
#if 0
/*
static int lsm303dlh_gpio_interrupt_config(int gpio)
{
	int rc = 0;

    // configure touchscreen irq gpio   
	rc = gpio_tlmm_config(GPIO_CFG(gpio, 0, GPIO_CFG_INPUT,                GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 0);  
	if (rc)
	{
		pr_err("%s: unable to configure gpio %d\n",            __func__, gpio);
		goto fail_irq;    
	}
	
	rc = gpio_request(gpio, "gsensor_irq_gpio");   
	if (rc) 
	{
		pr_err("%s: unable to request gpio %d\n",            __func__, gpio); 
		goto fail_irq;    
	}
	
	rc = gpio_direction_input(gpio);	
	if (rc) 
	{
		pr_err("%s: FAILED: gpio_direction_input(%d) rc=%d\n", __func__, gpio, rc);	
		goto fail_irq;	
	}
	
fail_irq:	
	return rc;

}
*/
#endif
//[HPP]guoye
//[HPP]guoye

static int lsm303dlh_acc_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)					//?? be used for Device INIT
{
	struct lsm303dlh_acc_data *acc = lsm303dlh_acc_misc_data ;
	int err = -1;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto err0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}


	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);
	acc->client = client;

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL)
		goto err1;

	memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));

	//[HPP]guoye-get axis map dynamically with different board
	if( acc->pdata->get_axis_map )
	{
		int map[6];
		memset(map,0,sizeof(map));
		acc->pdata->get_axis_map(map);

		acc->pdata->axis_map_x = map[0];
		acc->pdata->axis_map_y = map[1];
		acc->pdata->axis_map_z = map[2];
		acc->pdata->negate_x = map[3];
		acc->pdata->negate_y = map[4];
		acc->pdata->negate_z = map[5];
	}
	//[HPP]guoye

	err = lsm303dlh_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, acc);

	if (acc->pdata->init) {
		//pr_info("%s: init\n", __func__);
		err = acc->pdata->init();
		if (err < 0)
			goto err1_1;
	}

	//[HPP]guoye-For interrupt mode
	//request irq to catch interrupt
#if 1
	event_queue_init(&(acc->event_q),"event");

	//printk("guoye: gpio = %d \n",acc->pdata->gpioirq);
	//[HPP]guoye-Move gpio config to board-msm8x60.c
	//err = lsm303dlh_gpio_interrupt_config(acc->pdata->gpioirq);
	//if( err )
		//goto err2;
	if( acc->pdata->gpio_config != 0 )
	{
		acc->irq = acc->pdata->gpio_config();
		if( acc->irq < 0 )
			goto err2;

	//err = request_irq(MSM_GPIO_TO_INT(124), hp_lsm303dlh_interrupt_handler, IRQF_TRIGGER_RISING,	"gsensor", acc);
	//printk("guoye: acc->client->irq = %d \n",acc->client->irq);
	//printk("guoye: MSM_GPIO_TO_INT(124) = %d \n",MSM_GPIO_TO_INT(124));
		
	//[HPP]guoye-Move gpio config to board-msm8x60.c	
	//err = request_irq(acc->client->irq, hp_lsm303dlh_interrupt_handler, IRQF_TRIGGER_RISING,	"gsensor", acc);
		err = request_irq(acc->irq, hp_lsm303dlh_interrupt_handler, IRQF_TRIGGER_RISING,	"gsensor", acc);
		if( err )
			goto err2;
	}
	//[HPP]guoye
#endif
	//[HPP]guoye

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[0] = 7;
	acc->resume_state[1] = 0;
	acc->resume_state[2] = 0;
	acc->resume_state[3] = 0;
	acc->resume_state[4] = 0;

	err = lsm303dlh_acc_device_power_on(acc);
	if (err < 0)
		goto err2;

	//atomic_set(&acc->enabled, 1);

	err = lsm303dlh_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto err2;
	}

	err = lsm303dlh_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2;
	}

	err = lsm303dlh_acc_input_init(acc);
	if (err < 0)
		goto err3;


	err = misc_register(&lsm303dlh_acc_misc_device);  //?? register a charactor device 
	if (err < 0) {
		dev_err(&client->dev, "lsm_acc_device register failed\n");
		goto err4;
	}

	lsm303dlh_acc_device_power_off(acc);

	/* As default, do not report information */
	//atomic_set(&acc->enabled, 0);

	mutex_unlock(&acc->lock);

	dev_info(&client->dev, "lsm303dlh_acc probed\n");

	return 0;

err4:
	lsm303dlh_acc_input_cleanup(acc);
err3:
	lsm303dlh_acc_device_power_off(acc);
err2:
	if (acc->pdata->exit)
		acc->pdata->exit();
err1_1:
	mutex_unlock(&acc->lock);
	kfree(acc->pdata);
err1:
	kfree(acc);
err0:
	return err;
}

static int __devexit lsm303dlh_acc_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct lsm303dlh_acc_data *acc = i2c_get_clientdata(client);

	//[HPP]guoye-For interrupt mode
	//free_irq(acc->client->irq,acc);
	//[HPP]guoye
	misc_deregister(&lsm303dlh_acc_misc_device);
	lsm303dlh_acc_input_cleanup(acc);
	lsm303dlh_acc_device_power_off(acc);
	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

static int lsm303dlh_acc_resume(struct i2c_client *client)
{
	struct lsm303dlh_acc_data *acc = i2c_get_clientdata(client);
	int err;
	if (acc->on_before_suspend)
	{
		
		//[HPP]guoye-Always return success
		//return lsm303dlh_acc_enable(acc);
		//lsm303dlh_acc_enable(acc);
		//[HPP]guoye
		err = lsm303dlh_acc_device_power_on(acc);
		if (err>=0)
		{	
			if (!acc->ext_adap_enabled)
			{
				schedule_delayed_work(&acc->input_work, msecs_to_jiffies(acc->pdata->poll_interval));
			}			
			else
			{
				cancel_delayed_work_sync(&acc->input_work);
			}
		}	
	}	
	return 0;
}

static int lsm303dlh_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lsm303dlh_acc_data *acc = i2c_get_clientdata(client);

	//acc->on_before_suspend = atomic_read(&acc->enabled);
	spin_lock_irq(&acc->enable_lock);
	acc->on_before_suspend = acc->enable_count;
	spin_unlock_irq(&acc->enable_lock);
	
	//[HPP]guoye-Always return success
	//return lsm303dlh_acc_disable(acc);
	//lsm303dlh_acc_disable(acc);
	cancel_delayed_work_sync(&acc->input_work);
	lsm303dlh_acc_device_power_off(acc);
	return 0;
	//[HPP]guoye
}

static const struct i2c_device_id lsm303dlh_acc_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lsm303dlh_acc_id);

static struct i2c_driver lsm303dlh_acc_driver = {
	.driver = {
		   .name = NAME,
		   },
	.probe = lsm303dlh_acc_probe,
	.remove = __devexit_p(lsm303dlh_acc_remove),
	.resume = lsm303dlh_acc_resume,
	.suspend = lsm303dlh_acc_suspend,
	.id_table = lsm303dlh_acc_id,
};

static int __init lsm303dlh_acc_init(void)
{
	struct lsm303dlh_acc_data *acc;

	//printk(KERN_INFO "LSM303DLH_ACC driver for the accelerometer part\n");  //?? driver init , waiting for BSP init 

	acc = kzalloc(sizeof(*acc), GFP_KERNEL);
	if (acc == NULL) {
		pr_err("%s: failed to allocate memory for module data\n", __func__);
		return -ENOMEM;
	}
	spin_lock_init (&acc->enable_lock);
	acc->enable_count = 0;
	
	lsm303dlh_acc_misc_data = acc;

	return i2c_add_driver(&lsm303dlh_acc_driver);
}

static void __exit lsm303dlh_acc_exit(void)
{
	i2c_del_driver(&lsm303dlh_acc_driver);
	return;
}

module_init(lsm303dlh_acc_init);
module_exit(lsm303dlh_acc_exit);

MODULE_DESCRIPTION("lsm303dlh accelerometer driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");

