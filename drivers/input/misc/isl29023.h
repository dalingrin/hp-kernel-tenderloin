/******************************************************************************
 * isl29023.h - Linux kernel module for Intersil ISL29023 ambient light sensor
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


#ifndef __ISL29023_H__
#define __ISL29023_H__


#define ISL29023_ADDR	0x44
#define	ISL29023_MAJOR	218
#define	DEVICE_NAME		"isl29023"
#define	DRIVER_VERSION	"1.0"

/* IOCTL cmd define */
#define WR_CMD1			0x0
#define WR_CMD2			0x1
#define WR_INT_LT		0x2
#define	WR_INT_HT		0x3
#define RD_CMD1			0x4
#define RD_CMD2			0x5
#define	RD_INT_LT		0x6
#define	RD_INT_HT		0x7
#define	RD_DATA			0x8
#define	WR_ENABLE		0x9
#define	WR_DISABLE	0x9

/* Each client has this additional data */
struct isl29023_data_t {
	spinlock_t	lock;
	u8	minor;
	u8	dev_open_cnt;
	struct i2c_client* client;
	u8	pwr_status;
	struct input_dev *input_dev;
	u8	enabled;
	int pre_status;
	struct delayed_work polling_work;
	//HP zhanghong:Oct 18 9:36 CST 2010
	struct intersil_isl29023_platform_data *pdata;
	//End
};

#endif

