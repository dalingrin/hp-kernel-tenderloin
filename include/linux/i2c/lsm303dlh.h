/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name          : lsm303dlh.h
* Authors            : MSH - Motion Mems BU - Application Team
*		     : Carmine Iascone (carmine.iascone@st.com)
*		     : Matteo Dameno (matteo.dameno@st.com)
* Version            : V 1.0.1
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
*******************************************************************************/

#ifndef __LSM303DLH_H__
#define __LSM303DLH_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define LSM303DLH_ACC_IOCTL_BASE 'a'
/* The following define the IOCTL command values via the ioctl macros */
#define LSM303DLH_ACC_IOCTL_SET_DELAY	_IOW(LSM303DLH_ACC_IOCTL_BASE, 0, int)
#define LSM303DLH_ACC_IOCTL_GET_DELAY	_IOR(LSM303DLH_ACC_IOCTL_BASE, 1, int)
#define LSM303DLH_ACC_IOCTL_SET_ENABLE	_IOW(LSM303DLH_ACC_IOCTL_BASE, 2, int)
#define LSM303DLH_ACC_IOCTL_GET_ENABLE	_IOR(LSM303DLH_ACC_IOCTL_BASE, 3, int)
#define LSM303DLH_ACC_IOCTL_SET_G_RANGE	_IOW(LSM303DLH_ACC_IOCTL_BASE, 4, int)

#define LSM303DLH_MAG_IOCTL_BASE 'm'
/* The following define the IOCTL command values via the ioctl macros */
#define LSM303DLH_MAG_IOCTL_SET_DELAY	_IOW(LSM303DLH_MAG_IOCTL_BASE, 0, int)
#define LSM303DLH_MAG_IOCTL_GET_DELAY	_IOR(LSM303DLH_MAG_IOCTL_BASE, 1, int)
#define LSM303DLH_MAG_IOCTL_SET_ENABLE	_IOW(LSM303DLH_MAG_IOCTL_BASE, 2, int)
#define LSM303DLH_MAG_IOCTL_GET_ENABLE	_IOR(LSM303DLH_MAG_IOCTL_BASE, 3, int)
#define LSM303DLH_MAG_IOCTL_SET_H_RANGE	_IOW(LSM303DLH_MAG_IOCTL_BASE, 4, int)

/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define LSM303DLH_G_2G 			0x00
#define LSM303DLH_G_4G 			0x10
#define LSM303DLH_G_8G 			0x30

/* Accelerometer Sensor Operating Mode */
#define LSM303DLH_ACC_PM_OFF		0x00
#define LSM303DLH_ACC_PM_NORMAL		0x20
#define LSM303DLH_ACC_ENABLE_ALL_AXES	0x07

/* Accelerometer output data rate  */
#define LSM303DLH_ACC_ODRHALF		0x40	/* 0.5Hz output data rate */
#define LSM303DLH_ACC_ODR1		0x60	/* 1Hz output data rate */
#define LSM303DLH_ACC_ODR2		0x80	/* 2Hz output data rate */
#define LSM303DLH_ACC_ODR5		0xA0	/* 5Hz output data rate */
#define LSM303DLH_ACC_ODR10		0xC0	/* 10Hz output data rate */
#define LSM303DLH_ACC_ODR50		0x00	/* 50Hz output data rate */
#define LSM303DLH_ACC_ODR100		0x08	/* 100Hz output data rate */
#define LSM303DLH_ACC_ODR400		0x10	/* 400Hz output data rate */
#define LSM303DLH_ACC_ODR1000		0x18	/* 1000Hz output data rate */

/************************************************/
/* 	Magnetometer section defines	 	*/
/************************************************/

/* Magnetometer Sensor Full Scale */
#define LSM303DLH_H_1_3G		0x20
#define LSM303DLH_H_1_9G		0x40
#define LSM303DLH_H_2_5G		0x60
#define LSM303DLH_H_4_0G		0x80
#define LSM303DLH_H_4_7G		0xA0
#define LSM303DLH_H_5_6G		0xC0
#define LSM303DLH_H_8_1G		0xE0

/* Magnetic Sensor Operating Mode */
#define LSM303DLH_MAG_NORMAL_MODE	0x00
#define LSM303DLH_MAG_POS_BIAS		0x01
#define LSM303DLH_MAG_NEG_BIAS		0x02
#define LSM303DLH_MAG_CC_MODE		0x00
#define LSM303DLH_MAG_SC_MODE		0x01
#define LSM303DLH_MAG_SLEEP_MODE	0x03

/* Magnetometer output data rate  */
#define LSM303DLH_MAG_ODR_75		0x00	/* 0.75Hz output data rate */
#define LSM303DLH_MAG_ODR1_5		0x04	/* 1.5Hz output data rate */
#define LSM303DLH_MAG_ODR3_0		0x08	/* 3Hz output data rate */
#define LSM303DLH_MAG_ODR7_5		0x09	/* 7.5Hz output data rate */
#define LSM303DLH_MAG_ODR15		0x10	/* 15Hz output data rate */
#define LSM303DLH_MAG_ODR30		0x14	/* 30Hz output data rate */
#define LSM303DLH_MAG_ODR75		0x18	/* 75Hz output data rate */

#ifdef __KERNEL__
struct lsm303dlh_acc_platform_data {

	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	//[HPP]guoye-For interrupt mode
	//int gpioirq;
	//[HPP]guoye

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	
	int (*gpio_config)(void);
	//[HPP]guoye-get axis map dynamically with different board
	void (*get_axis_map)(int*);
	//[HPP]guoye

};

struct int_event_queue 
{
	int event[50];
	int p_in, p_out;
	spinlock_t lock;
	wait_queue_head_t in_wait,out_wait;
	const char *name;
};

struct lsm303dlh_acc_data {
	struct i2c_client *client;
	struct lsm303dlh_acc_platform_data *pdata;

	spinlock_t enable_lock;
	struct mutex lock;

	struct delayed_work input_work;
	struct input_dev *input_dev;

	int		hw_initialized;
	int		enable_count;
	int		on_before_suspend;

	u8		shift_adj;
	u8		resume_state[5];
	
	//[HPP]guoye-For interrupt mode
	//int irq;
	u8		int_resume_state[4];//for INT1_CFG,INT1_SRC,INT1_THS,INT1_DURATION registers
	struct int_event_queue event_q;	
	//[HPP]guoye-For interrupt mode
	int irq;
	//[HPP]guoye
	//[HPP]guoye

	u8		ext_adap_enabled;
	void *	ext_handle;
	int		(*send_cb)(void* ext_handle, unsigned char addr, unsigned short len, unsigned char const *buf);
};

struct lsm303dlh_mag_data {
	struct i2c_client *client;
	struct lsm303dlh_mag_platform_data *pdata;

	struct mutex lock;
	spinlock_t enable_lock;

	struct delayed_work input_work;
	struct input_dev *input_dev;

	int hw_initialized;
	int		enable_count;
	int on_before_suspend;

	u16 xy_sensitivity;
	u16 z_sensitivity;
	u8 resume_state[3];

	u8	ext_adap_enabled;
	void *	ext_handle;
};



struct lsm303dlh_mag_platform_data {

	int poll_interval;
	int min_interval;

	u8 h_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	//[HPP]guoye-get axis map dynamically with different board
	void (*get_axis_map)(int*);
	//[HPP]guoye

};
#endif /* __KERNEL__ */

#endif  /* __LSM303DLH_H__ */
