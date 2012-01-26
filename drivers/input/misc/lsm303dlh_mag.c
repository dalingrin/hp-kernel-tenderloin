/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name          : lsm303dlh_mag.c
* Authors            : MSH - Motion Mems BU - Application Team
*		     : Carmine Iascone (carmine.iascone@st.com)
*		     : Matteo Dameno (matteo.dameno@st.com)
* Version            : V 1.1.0
* Date               : 24/05/2010
* Description        : LSM303DLH 6D module sensor device driver
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

24_5_2010: lsm303dlh_mag_get_acceleration_data now converts the saturation
	   value from 0xF000 (coming from the sensor) to 0x8000

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


#define NAME			HP_MAGNETIC_NAME  //"lsm303dlh_mag"

/** Maximum polled-device-reported g value */
#define H_MAX			8100

#define SHIFT_ADJ_2G		4
#define SHIFT_ADJ_4G		3
#define SHIFT_ADJ_8G		2

/* Magnetometer registers */
#define CRA_REG_M		0x00	/* Configuration register A */
#define CRB_REG_M		0x01	/* Configuration register B */
#define MR_REG_M		0x02	/* Mode register */
#define AUTO_INCREMENT		0x80


/* Output register start address*/
#define OUT_X_M			0x03

/* Magnetic Sensor Operation Mode */
#define NORMAL_MODE     	0x00
#define POS_BIAS         	0x01
#define NEG_BIAS         	0x02
#define CC_MODE          	0x00
#define IDLE_MODE        	0x03

/* Magnetometer X-Y sensitivity  */
#define XY_SENSITIVITY_1_3	1055	/* XY sensitivity at 1.3G */
#define XY_SENSITIVITY_1_9	795	/* XY sensitivity at 1.9G */
#define XY_SENSITIVITY_2_5	635	/* XY sensitivity at 2.5G */
#define XY_SENSITIVITY_4_0	430	/* XY sensitivity at 4.0G */
#define XY_SENSITIVITY_4_7	375	/* XY sensitivity at 4.7G */
#define XY_SENSITIVITY_5_6	320	/* XY sensitivity at 5.6G */
#define XY_SENSITIVITY_8_1	230	/* XY sensitivity at 8.1G */

/* Magnetometer Z sensitivity  */
#define Z_SENSITIVITY_1_3	950	/* Z sensitivity at 1.3G */
#define Z_SENSITIVITY_1_9	710	/* Z sensitivity at 1.9G */
#define Z_SENSITIVITY_2_5	570	/* Z sensitivity at 2.5G */
#define Z_SENSITIVITY_4_0	385	/* Z sensitivity at 4.0G */
#define Z_SENSITIVITY_4_7	335	/* Z sensitivity at 4.7G */
#define Z_SENSITIVITY_5_6	285	/* Z sensitivity at 5.6G */
#define Z_SENSITIVITY_8_1	205	/* Z sensitivity at 8.1G */

#define FUZZ			0
#define FLAT			0
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5

static struct {
	unsigned int cutoff;
	unsigned int mask;
} odr_table[] = {
	{
	34,	LSM303DLH_MAG_ODR75}, {
	67,	LSM303DLH_MAG_ODR30}, {
	134,	LSM303DLH_MAG_ODR15}, {
	334,	LSM303DLH_MAG_ODR7_5}, {
	667,	LSM303DLH_MAG_ODR3_0}, {
	1334,	LSM303DLH_MAG_ODR1_5}, {
	0,	LSM303DLH_MAG_ODR_75}, };


/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct lsm303dlh_mag_data *lsm303dlh_mag_misc_data;

static int lsm303dlh_mag_i2c_read(struct lsm303dlh_mag_data *mag,
				  u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = mag->client->addr,
		 .flags = mag->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = mag->client->addr,
		 .flags = (mag->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(mag->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&mag->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm303dlh_mag_i2c_write(struct lsm303dlh_mag_data *mag,
				   u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = mag->client->addr,
		 .flags = mag->client->flags,// & I2C_M_TEN,
		 .len = len+ 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(mag->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&mag->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm303dlh_mag_hw_init(struct lsm303dlh_mag_data *mag)
{
	int err = -1;
	u8 buf[4];

	//pr_info("%s\n", __func__);

	buf[0] = (AUTO_INCREMENT | CRA_REG_M);
	buf[1] = mag->resume_state[0];
	buf[2] = mag->resume_state[1];
	buf[3] = mag->resume_state[2];
	err = lsm303dlh_mag_i2c_write(mag, buf, 3);

	if (err < 0)
		return err;

	mag->hw_initialized = 1;

	return 0;
}

static void lsm303dlh_mag_device_power_off(struct lsm303dlh_mag_data *mag)
{
	int err;
	u8 buf[2] = { MR_REG_M, IDLE_MODE };
	err = lsm303dlh_mag_i2c_write(mag, buf, 1);
	if (err < 0)
		dev_err(&mag->client->dev, "soft power off failed\n");

	if (mag->pdata->power_off) {
		mag->pdata->power_off();
		mag->hw_initialized = 0;
	}
}

static int lsm303dlh_mag_device_power_on(struct lsm303dlh_mag_data *mag)
{
	int err;

	if (mag->pdata->power_on) {
		err = mag->pdata->power_on();
		if (err < 0)
			return err;
	}

	if (!mag->hw_initialized) {
		err = lsm303dlh_mag_hw_init(mag);
		if (err < 0) {
			lsm303dlh_mag_device_power_off(mag);
			return err;
		}
	} else {
	u8 buf[2] = { MR_REG_M, NORMAL_MODE };

	err = lsm303dlh_mag_i2c_write(mag, buf, 1);
	if (err < 0)
		dev_err(&mag->client->dev, "power on failed\n");

	}
	return 0;
}

int lsm303dlh_mag_update_h_range(struct lsm303dlh_mag_data *mag,
				 u8 new_h_range)
{
	int err;
	u8 buf[2];

	switch (new_h_range) {
	case LSM303DLH_H_1_3G:
		mag->xy_sensitivity = XY_SENSITIVITY_1_3;
		mag->z_sensitivity = Z_SENSITIVITY_1_3;
		break;
	case LSM303DLH_H_1_9G:
		mag->xy_sensitivity = XY_SENSITIVITY_1_9;
		mag->z_sensitivity = Z_SENSITIVITY_1_9;
		break;
	case LSM303DLH_H_2_5G:
		mag->xy_sensitivity = XY_SENSITIVITY_2_5;
		mag->z_sensitivity = Z_SENSITIVITY_2_5;
		break;
	case LSM303DLH_H_4_0G:
		mag->xy_sensitivity = XY_SENSITIVITY_4_0;
		mag->z_sensitivity = Z_SENSITIVITY_4_0;
		break;
	case LSM303DLH_H_4_7G:
		mag->xy_sensitivity = XY_SENSITIVITY_4_7;
		mag->z_sensitivity = Z_SENSITIVITY_4_7;
		break;
	case LSM303DLH_H_5_6G:
		mag->xy_sensitivity = XY_SENSITIVITY_5_6;
		mag->z_sensitivity = Z_SENSITIVITY_5_6;
		break;
	case LSM303DLH_H_8_1G:
		mag->xy_sensitivity = XY_SENSITIVITY_8_1;
		mag->z_sensitivity = Z_SENSITIVITY_8_1;
		break;
	default:
		return -EINVAL;
	}

	//if (atomic_read(&mag->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = CRB_REG_M;
		buf[1] = new_h_range;
		err = lsm303dlh_mag_i2c_write(mag, buf, 1);
		if (err < 0)
			return err;
	//}

	mag->resume_state[1] = new_h_range;

	return 0;
}

int lsm303dlh_mag_update_odr(struct lsm303dlh_mag_data *mag, int poll_interval)
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

	config[1] |= NORMAL_MODE;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
//	if (atomic_read(&mag->enabled)) {
		config[0] = CRA_REG_M;
		err = lsm303dlh_mag_i2c_write(mag, config, 1);
		if (err < 0)
			return err;
//	}

	mag->resume_state[0] = config[1];

	return 0;
}

static int lsm303dlh_mag_get_acceleration_data(struct lsm303dlh_mag_data *mag,
						   int *xyz)
{
	int err = -1;
	/* Data bytes from hardware HxL, HxH, HyL, HyH, HzL, HzH */
	u8 mag_data[6];
	/* x,y,z hardware data */
	int hw_d[3] = { 0 };

	mag_data[0] = OUT_X_M;
	err = lsm303dlh_mag_i2c_read(mag, mag_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (int) (((mag_data[0]) << 8) | mag_data[1]);
	hw_d[1] = (int) (((mag_data[2]) << 8) | mag_data[3]);
	hw_d[2] = (int) (((mag_data[4]) << 8) | mag_data[5]);

	hw_d[0] = (hw_d[0] & 0x8000) ? (hw_d[0] | 0xFFFF0000) : (hw_d[0]);
	hw_d[1] = (hw_d[1] & 0x8000) ? (hw_d[1] | 0xFFFF0000) : (hw_d[1]);
	hw_d[2] = (hw_d[2] & 0x8000) ? (hw_d[2] | 0xFFFF0000) : (hw_d[2]);

//	pr_err("######## mag data: %02x %02x %02x %02x %02x %02x \n", mag_data[0], mag_data[1], mag_data[2], mag_data[3], mag_data[4], mag_data[5]);
//	pr_err("######## hw_d data: %08x %08x %08x\n", hw_d[0], hw_d[1], hw_d[2]);
	if (hw_d[0] != 0XF000)
		hw_d[0] = hw_d[0] * 1000 / mag->xy_sensitivity;
	else
		hw_d[0] = 0X8000;
	if (hw_d[1] != 0XF000)
		hw_d[1] = hw_d[1] * 1000 / mag->xy_sensitivity;
	else
		hw_d[1] = 0x8000;
	if (hw_d[2] != 0XF000)
		hw_d[2] = hw_d[2] * 1000 / mag->z_sensitivity;
	else
		hw_d[2] = 0x8000;
//	pr_err("######## hw_d 2 data: %08x %08x %08x\n", hw_d[0], hw_d[1], hw_d[2]);
	if ((hw_d[mag->pdata->axis_map_x] != 0x8000) && (mag->pdata->negate_x))
		xyz[0] = -hw_d[mag->pdata->axis_map_x];
	else
		xyz[0] = hw_d[mag->pdata->axis_map_x];

	if ((hw_d[mag->pdata->axis_map_y] != 0x8000) && (mag->pdata->negate_y))
		xyz[1] = -hw_d[mag->pdata->axis_map_y];
	else
		xyz[1] = hw_d[mag->pdata->axis_map_y];

	if ((hw_d[mag->pdata->axis_map_z] != 0x8000) && (mag->pdata->negate_z))
		xyz[2] = -hw_d[mag->pdata->axis_map_z];
	else
		xyz[2] = hw_d[mag->pdata->axis_map_z];
//	pr_err("######## xyz data: %08x %08x %08x\n", xyz[0], xyz[1], xyz[2]);
	return err;
}

static void lsm303dlh_mag_report_values(struct lsm303dlh_mag_data *mag,
					int *xyz)
{
	//pr_info("guoye: reporting mag... (xyz[0], xyz[1], xyz[2]) = (%d, %d, %d) \n", xyz[0], xyz[1], xyz[2]);

	input_report_abs(mag->input_dev, ABS_X, xyz[0]);
	input_report_abs(mag->input_dev, ABS_Y, xyz[1]);
	input_report_abs(mag->input_dev, ABS_Z, xyz[2]);
	input_sync(mag->input_dev);
}

static int lsm303dlh_mag_enable(struct lsm303dlh_mag_data *mag)
{
	int err;
	int cur_count;

//	pr_info("%s\n", __func__);

	spin_lock_irq(&mag->enable_lock);
	cur_count = mag->enable_count;
	mag->enable_count++;
	spin_unlock_irq(&mag->enable_lock);

	//pr_info("%s: ext_adap_enabled:%d, cur_count:%d\n", __func__, mag->ext_adap_enabled, cur_count);

	if (!cur_count) {
		//pr_info("%s, enable mag\n", __func__);
		err = lsm303dlh_mag_device_power_on(mag);
		if (err < 0) {
			spin_lock_irq(&mag->enable_lock);
			mag->enable_count=0;
			spin_unlock_irq(&mag->enable_lock);
			return err;
		}
	}

	if (!mag->ext_adap_enabled)
	{
		//pr_info("%s: enable native mag schedule_delayed_work\n", __func__);
		schedule_delayed_work(&mag->input_work, msecs_to_jiffies(mag->pdata->poll_interval));
	}
	else
	{
		//pr_info("%s: cancel native mag schedule_delayed_work\n", __func__);
		cancel_delayed_work_sync(&mag->input_work);
	}

	return 0;
}

static int lsm303dlh_mag_disable(struct lsm303dlh_mag_data *mag)
{
	int cur_count;

	//pr_info("%s\n", __func__);

	spin_lock_irq(&mag->enable_lock);
	if (mag->enable_count>0)
		mag->enable_count--;
	cur_count = mag->enable_count;
	spin_unlock_irq(&mag->enable_lock);

	//pr_info("%s: ext_adap_enabled:%d, cur_count:%d\n", __func__, mag->ext_adap_enabled, cur_count);

	if (!cur_count) {
		//pr_info("%s, disable mag \n", __func__);
		cancel_delayed_work_sync(&mag->input_work);
		lsm303dlh_mag_device_power_off(mag);
	}
	else
	{
		if (!mag->ext_adap_enabled)
		{
	//if enable mag from gyro request, we need to re-initial mag, Tandi
			lsm303dlh_mag_hw_init(mag);
			//pr_info("%s: enable native mag schedule_delayed_work\n", __func__);
			schedule_delayed_work(&mag->input_work, msecs_to_jiffies(mag->pdata->poll_interval));
		}
	}

	return 0;
}

int lsm303dlh_mag_enable_ext(struct lsm303dlh_mag_data *mag)
{
	int err = 0;

	//pr_info("%s\n", __func__);
	if (!mag->ext_adap_enabled)
	{
		mag->ext_adap_enabled = 1;
		err = lsm303dlh_mag_enable(mag);
	}

	return err;
}
EXPORT_SYMBOL(lsm303dlh_mag_enable_ext);

int lsm303dlh_mag_disable_ext(struct lsm303dlh_mag_data *mag)
{
	int err=0;

	//pr_info("%s\n", __func__);

	if (mag->ext_adap_enabled)
	{
		mag->ext_adap_enabled = 0;
		err = lsm303dlh_mag_disable(mag);
	}
  return err;
}
EXPORT_SYMBOL(lsm303dlh_mag_disable_ext);

struct lsm303dlh_mag_data * lsm303dlh_mag_get_instance_ext(void)
{
	return lsm303dlh_mag_misc_data;
}
EXPORT_SYMBOL(lsm303dlh_mag_get_instance_ext);


static int lsm303dlh_mag_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = lsm303dlh_mag_misc_data;

	return 0;
}

static int lsm303dlh_mag_misc_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 buf[4];
	int err;
	int interval;
	struct lsm303dlh_mag_data *mag = file->private_data;

	switch (cmd) {
	case HP_MAGNETICSENSOR_IOCTL_GET_DELAY:
	case LSM303DLH_MAG_IOCTL_GET_DELAY:
		//pr_info("%s:  HP_MAGNETICSENSOR_IOCTL_GET_DELAY\n", __func__);
		interval = mag->pdata->poll_interval;
		//pr_info("LSM303DLH_MAG_IOCTL_GET_DELAY: %d\n", interval);
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case HP_MAGNETICSENSOR_IOCTL_SET_DELAY:
	case LSM303DLH_MAG_IOCTL_SET_DELAY:
		//pr_info("%s:  HP_MAGNETICSENSOR_IOCTL_SET_DELAY\n", __func__);
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		//pr_info("LSM303DLH_MAG_IOCTL_SET_DELAY: %d\n", interval);
		if (interval < 0 || interval > 1500)
			return -EINVAL;

		mag->pdata->poll_interval =
			max(interval, mag->pdata->min_interval);
		err = lsm303dlh_mag_update_odr(mag, mag->pdata->poll_interval);
		/* TODO: if update fails poll is still set */
		if (err < 0)
			return err;

		break;

	case HP_MAGNETICSENSOR_IOCTL_ENABLE:
	case LSM303DLH_MAG_IOCTL_SET_ENABLE:
		//pr_info("%s:  HP_MAGNETICSENSOR_IOCTL_ENABLE\n", __func__);
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		//pr_info("LSM303DLH_MAG_IOCTL_SET_ENABLE: %d\n", interval);
		if (interval > 1)
			return -EINVAL;
		if (interval)
			lsm303dlh_mag_enable(mag);
		else
			lsm303dlh_mag_disable(mag);
		break;

	case HP_MAGNETICSENSOR_IOCTL_GET_ENABLED:
	case LSM303DLH_MAG_IOCTL_GET_ENABLE:
		//pr_info("%s:  HP_MAGNETICSENSOR_IOCTL_GET_ENABLED\n", __func__);
//		interval = atomic_read(&mag->enabled);
		interval = mag->enable_count;
		//pr_info("LSM303DLH_MAG_IOCTL_GET_ENABLE: %d\n", interval);
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EINVAL;
		break;

	case LSM303DLH_MAG_IOCTL_SET_H_RANGE:
		//pr_info("%s:  LSM303DLH_MAG_IOCTL_SET_H_RANGE\n", __func__);
		if (copy_from_user(&buf, argp, 1))
			return -EFAULT;
		//pr_info("LSM303DLH_MAG_IOCTL_SET_H_RANGE: %d\n", buf[0]);
		err = lsm303dlh_mag_update_h_range(mag, buf[0]);
		if (err < 0)
			return err;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations lsm303dlh_mag_misc_fops = {
	.owner = THIS_MODULE,
	.open = lsm303dlh_mag_misc_open,
	.ioctl = lsm303dlh_mag_misc_ioctl,
};

static struct miscdevice lsm303dlh_mag_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &lsm303dlh_mag_misc_fops,
};

static void lsm303dlh_mag_input_work_func(struct work_struct *work)
{
	struct lsm303dlh_mag_data *mag;
	int xyz[3] = { 0 };
	int err;

	mag  = container_of((struct delayed_work *)work,
				 struct lsm303dlh_mag_data, input_work);

	mutex_lock(&mag->lock);
	err = lsm303dlh_mag_get_acceleration_data(mag, xyz);
	if (err < 0)
		dev_err(&mag->client->dev, "get_magnetometer_data failed\n");
	else
		lsm303dlh_mag_report_values(mag, xyz);

	schedule_delayed_work(&mag->input_work,
				  msecs_to_jiffies(mag->pdata->poll_interval));
	mutex_unlock(&mag->lock);
}

#ifdef LSMS303DLH_MAG_OPEN_ENABLE
int lsm303dlh_mag_input_open(struct input_dev *input)
{
	struct lsm303dlh_mag_data *mag = input_get_drvdata(input);

	return lsm303dlh_mag_enable(mag);
}

void lsm303dlh_mag_input_close(struct input_dev *dev)
{
	struct lsm303dlh_mag_data *mag = input_get_drvdata(dev);

	lsm303dlh_mag_disable(mag);
}
#endif

static int lsm303dlh_mag_validate_pdata(struct lsm303dlh_mag_data *mag)
{
	//printk("guoye: lsm303dlh_mag_validate_pdata \n");

	mag->pdata->poll_interval = max(mag->pdata->poll_interval,
					mag->pdata->min_interval);

	if (mag->pdata->axis_map_x > 2 ||
		mag->pdata->axis_map_y > 2 || mag->pdata->axis_map_z > 2) {
		dev_err(&mag->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			mag->pdata->axis_map_x, mag->pdata->axis_map_y,
			mag->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (mag->pdata->negate_x > 1 || mag->pdata->negate_y > 1 ||
		mag->pdata->negate_z > 1) {
		dev_err(&mag->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			mag->pdata->negate_x, mag->pdata->negate_y,
			mag->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (mag->pdata->poll_interval < mag->pdata->min_interval) {
		dev_err(&mag->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm303dlh_mag_input_init(struct lsm303dlh_mag_data *mag)
{
	int err;

	INIT_DELAYED_WORK(&mag->input_work, lsm303dlh_mag_input_work_func);

	mag->input_dev = input_allocate_device();
	if (!mag->input_dev) {
		err = -ENOMEM;
		dev_err(&mag->client->dev, "input device allocate failed\n");
		goto err0;
	}

#ifdef LSMS303DLH_MAG_OPEN_ENABLE
	mag->input_dev->open = lsm303dlh_mag_input_open;
	mag->input_dev->close = lsm303dlh_mag_input_close;
#endif

	input_set_drvdata(mag->input_dev, mag);

	set_bit(EV_ABS, mag->input_dev->evbit);

	input_set_abs_params(mag->input_dev, ABS_X, -H_MAX, H_MAX, FUZZ, FLAT);
	input_set_abs_params(mag->input_dev, ABS_Y, -H_MAX, H_MAX, FUZZ, FLAT);
	input_set_abs_params(mag->input_dev, ABS_Z, -H_MAX, H_MAX, FUZZ, FLAT);

	mag->input_dev->name = NAME;

	err = input_register_device(mag->input_dev);
	if (err) {
		dev_err(&mag->client->dev,
			"unable to register input polled device %s\n",
			mag->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(mag->input_dev);
err0:
	return err;
}

static void lsm303dlh_mag_input_cleanup(struct lsm303dlh_mag_data *mag)
{
	input_unregister_device(mag->input_dev);
	input_free_device(mag->input_dev);
}

static int lsm303dlh_mag_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lsm303dlh_mag_data *mag;
	int err = -1;

	//pr_info("%s\n", __func__);

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

	mag = kzalloc(sizeof(*mag), GFP_KERNEL);
	if (mag == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&mag->lock);
	mutex_lock(&mag->lock);
	mag->client = client;
	spin_lock_init (&mag->enable_lock);

	mag->pdata = kmalloc(sizeof(*mag->pdata), GFP_KERNEL);
	if (mag->pdata == NULL)
		goto err1;

	memcpy(mag->pdata, client->dev.platform_data, sizeof(*mag->pdata));

	//[HPP]guoye-get axis map dynamically with different board
	if( mag->pdata->get_axis_map )
	{
		int map[6];
		memset(map,0,sizeof(map));
		mag->pdata->get_axis_map(map);

		mag->pdata->axis_map_x = map[0];
		mag->pdata->axis_map_y = map[1];
		mag->pdata->axis_map_z = map[2];
		mag->pdata->negate_x = map[3];
		mag->pdata->negate_y = map[4];
		mag->pdata->negate_z = map[5];
	}
	//[HPP]guoye

	err = lsm303dlh_mag_validate_pdata(mag);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, mag);

	if (mag->pdata->init) {
		err = mag->pdata->init();
		if (err < 0)
			goto err1_1;
	}

	memset(mag->resume_state, 0, ARRAY_SIZE(mag->resume_state));

	mag->resume_state[0] = 0x10;
	mag->resume_state[1] = 0x20;
	mag->resume_state[2] = 0x00;

	err = lsm303dlh_mag_device_power_on(mag);
	if (err < 0)
		goto err2;

//	atomic_set(&mag->enabled, 1);

	err = lsm303dlh_mag_update_h_range(mag, mag->pdata->h_range);
	if (err < 0) {
		dev_err(&client->dev, "update_h_range failed\n");
		goto err2;
	}

	err = lsm303dlh_mag_update_odr(mag, mag->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2;
	}

	err = lsm303dlh_mag_input_init(mag);
	if (err < 0)
		goto err3;

	lsm303dlh_mag_misc_data = mag;

	err = misc_register(&lsm303dlh_mag_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "lsm_mag_device register failed\n");
		goto err4;
	}

	lsm303dlh_mag_device_power_off(mag);

	/* As default, do not report information */
//	atomic_set(&mag->enabled, 0);

	mutex_unlock(&mag->lock);

	dev_info(&client->dev, "lsm303dlh_mag probed\n");

	return 0;

err4:
	lsm303dlh_mag_input_cleanup(mag);
err3:
	lsm303dlh_mag_device_power_off(mag);
err2:
	if (mag->pdata->exit)
		mag->pdata->exit();
err1_1:
	mutex_unlock(&mag->lock);
	kfree(mag->pdata);
err1:
	kfree(mag);
err0:
	return err;
}

static int __devexit lsm303dlh_mag_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct lsm303dlh_mag_data *mag = i2c_get_clientdata(client);

	misc_deregister(&lsm303dlh_mag_misc_device);
	lsm303dlh_mag_input_cleanup(mag);
	lsm303dlh_mag_device_power_off(mag);
	if (mag->pdata->exit)
		mag->pdata->exit();
	kfree(mag->pdata);
	kfree(mag);

	return 0;
}

static int lsm303dlh_mag_resume(struct i2c_client *client)
{
	int err = 0;
	struct lsm303dlh_mag_data *mag = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);

	if (mag->on_before_suspend)
	{

		//[HPP]guoye-Always return success
		//return lsm303dlh_mag_enable(mag);
		//lsm303dlh_mag_enable(mag);
		//[HPP]guoye
		err = lsm303dlh_mag_device_power_on(mag);
		if (err>=0)
		{
			if (!mag->ext_adap_enabled)
			{
				schedule_delayed_work(&mag->input_work, msecs_to_jiffies(mag->pdata->poll_interval));
			}
			else
			{
				cancel_delayed_work_sync(&mag->input_work);
			}
		}
	}
	return err;
}

static int lsm303dlh_mag_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lsm303dlh_mag_data *mag = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);

//	mag->on_before_suspend = atomic_read(&mag->enabled);
	spin_lock_irq(&mag->enable_lock);
	mag->on_before_suspend = mag->enable_count;
	spin_unlock_irq(&mag->enable_lock);

	//[HPP]guoye-Always return success
	//return lsm303dlh_mag_disable(mag);
//	lsm303dlh_mag_disable(mag);
	cancel_delayed_work_sync(&mag->input_work);
	lsm303dlh_mag_device_power_off(mag);
	return 0;
	//[HPP]guoye
}

static const struct i2c_device_id lsm303dlh_mag_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lsm303dlh_mag_id);

static struct i2c_driver lsm303dlh_mag_driver = {
	.driver = {
		   .name = NAME,
		   },
	.probe = lsm303dlh_mag_probe,
	.remove = __devexit_p(lsm303dlh_mag_remove),
	.resume = lsm303dlh_mag_resume,
	.suspend = lsm303dlh_mag_suspend,
	.id_table = lsm303dlh_mag_id,
};

static int __init lsm303dlh_mag_init(void)
{
	//printk(KERN_INFO "lsm303dlh magnetometer driver\n");
	return i2c_add_driver(&lsm303dlh_mag_driver);
}

static void __exit lsm303dlh_mag_exit(void)
{
	i2c_del_driver(&lsm303dlh_mag_driver);
	return;
}

module_init(lsm303dlh_mag_init);
module_exit(lsm303dlh_mag_exit);

MODULE_DESCRIPTION("lsm303dlh driver for the magnetometer section");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");
