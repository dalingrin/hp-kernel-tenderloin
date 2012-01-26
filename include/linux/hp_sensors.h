/*
 * Copyright(C) 2009 Hewlett-Packard Co Ltd. 
 * 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _HP_SENSORS_H
#define _HP_SENSORS_H

#include <linux/ioctl.h>

#include <linux/types.h>
#include <linux/ioctl.h>

#define HP_GSENSOR_NAME		"gsensor"
#define HP_MAGNETIC_NAME		"magsensor"
#define HP_ECOMPASS_NAME		"compasssensor"
#define HP_PSENSORS_NAME		"psensor"
#define HP_LIGHTSENSORS_NAME	"lightsensor"
#define HP_GYROSENSORS_NAME		"gyro"
#define HP_GRAVITYSENSORS_NAME	"gravity"
#define HP_ROTATIONSENSORS_NAME	"rotation"

#define HP_GSENSOR_DEV_NAME		"/dev/gsensor"
#define HP_MAGNETIC_DEV_NAME		"/dev/magsensor"
#define HP_ECOMPASS_DEV_NAME		"/dev/compasssensor"
#define HP_PSENSORS_DEV_NAME 		"/dev/psensor"
#define HP_LIGHTSENSORS_DEV_NAME 	"/dev/lightsensor"
#define HP_GYROSENSORS_DEV_NAME		"/dev/gyro"
#define HP_GRAVITYSENSORS_DEV_NAME	"/dev/gravity"
#define HP_ROTATIONSENSORS_DEV_NAME	"/dev/rotation"

#define HP_PSENSOR_IOCTL_MAGIC 'c'
#define HP_PSENSOR_IOCTL_GET_ENABLED			_IOR(HP_PSENSOR_IOCTL_MAGIC, 1, int *)
#define HP_PSENSOR_IOCTL_ENABLE					_IOW(HP_PSENSOR_IOCTL_MAGIC, 2, int *)

#define HP_LIGHTSENSOR_IOCTL_MAGIC 'l'
#define HP_LIGHTSENSOR_IOCTL_GET_ENABLED		_IOR(HP_LIGHTSENSOR_IOCTL_MAGIC, 1, int *)
#define HP_LIGHTSENSOR_IOCTL_ENABLE				_IOW(HP_LIGHTSENSOR_IOCTL_MAGIC, 2, int *)

/* IOCTLs for APPs */
#define HP_MAGNETICSENSOR_IOCTL_MAGIC 'M'
#define HP_MAGNETICSENSOR_IOCTL_GET_ENABLED		_IOR(HP_MAGNETICSENSOR_IOCTL_MAGIC, 1, int *)
#define HP_MAGNETICSENSOR_IOCTL_ENABLE				_IOW(HP_MAGNETICSENSOR_IOCTL_MAGIC, 2, int *)
#define HP_MAGNETICSENSOR_IOCTL_SET_DELAY		_IOW(HP_MAGNETICSENSOR_IOCTL_MAGIC, 3, int *)
#define HP_MAGNETICSENSOR_IOCTL_GET_DELAY		_IOW(HP_MAGNETICSENSOR_IOCTL_MAGIC, 4, int *)

#define HP_ORIENTATIONSENSOR_IOCTL_MAGIC 'O'
#define HP_ORIENTATIONSENSOR_IOCTL_GET_ENABLED		_IOR(HP_ORIENTATIONSENSOR_IOCTL_MAGIC, 1, int *)
#define HP_ORIENTATIONSENSOR_IOCTL_ENABLE				_IOW(HP_ORIENTATIONSENSOR_IOCTL_MAGIC, 2, int *)
#define HP_ORIENTATIONSENSOR_IOCTL_SET_DELAY		_IOW(HP_ORIENTATIONSENSOR_IOCTL_MAGIC, 3, int *)
#define HP_ORIENTATIONSENSOR_IOCTL_GET_DELAY		_IOW(HP_ORIENTATIONSENSOR_IOCTL_MAGIC, 4, int *)
#define HP_ORIENTATIONSENSOR_IOCTL_GET_STATUS		_IOR(HP_ORIENTATIONSENSOR_IOCTL_MAGIC, 5, int *)

#define HP_ACCELEROMETERSENSOR_IOCTL_MAGIC 'A'
#define HP_ACCELEROMETERSENSOR_IOCTL_GET_ENABLED		_IOR(HP_ACCELEROMETERSENSOR_IOCTL_MAGIC, 1, int *)
#define HP_ACCELEROMETERSENSOR_IOCTL_ENABLE				_IOW(HP_ACCELEROMETERSENSOR_IOCTL_MAGIC, 2, int *)
#define HP_ACCELEROMETERSENSOR_IOCTL_SET_DELAY		_IOW(HP_ACCELEROMETERSENSOR_IOCTL_MAGIC, 3, int *)
#define HP_ACCELEROMETERSENSOR_IOCTL_GET_DELAY		_IOW(HP_ACCELEROMETERSENSOR_IOCTL_MAGIC, 4, int *)

#define HP_GYROSENSOR_IOCTL_MAGIC 'G'
#define HP_GYROSENSOR_IOCTL_GET_ENABLED		_IOR(HP_GYROSENSOR_IOCTL_MAGIC, 1, int *)
#define HP_GYROSENSOR_IOCTL_ENABLE				_IOW(HP_GYROSENSOR_IOCTL_MAGIC, 2, int *)
#define HP_GYROSENSOR_IOCTL_SET_DELAY		_IOW(HP_GYROSENSOR_IOCTL_MAGIC, 3, int *)
#define HP_GYROSENSOR_IOCTL_GET_DELAY		_IOW(HP_GYROSENSOR_IOCTL_MAGIC, 4, int *)

#define HP_GRAVITYSENSOR_IOCTL_MAGIC 'g'
#define HP_GRAVITYSENSOR_IOCTL_GET_ENABLED		_IOR(HP_GRAVITYSENSOR_IOCTL_MAGIC, 1, int *)
#define HP_GRAVITYSENSOR_IOCTL_ENABLE				_IOW(HP_GRAVITYSENSOR_IOCTL_MAGIC, 2, int *)
#define HP_GRAVITYSENSOR_IOCTL_SET_DELAY		_IOW(HP_GRAVITYSENSOR_IOCTL_MAGIC, 3, int *)
#define HP_GRAVITYSENSOR_IOCTL_GET_DELAY		_IOW(HP_GRAVITYSENSOR_IOCTL_MAGIC, 4, int *)

#define HP_ROTATIONSENSOR_IOCTL_MAGIC 'o'
#define HP_ROTATIONSENSOR_IOCTL_GET_ENABLED		_IOR(HP_ROTATIONSENSOR_IOCTL_MAGIC, 1, int *)
#define HP_ROTATIONSENSOR_IOCTL_ENABLE				_IOW(HP_ROTATIONSENSOR_IOCTL_MAGIC, 2, int *)
#define HP_ROTATIONSENSOR_IOCTL_SET_DELAY		_IOW(HP_ROTATIONSENSOR_IOCTL_MAGIC, 3, int *)
#define HP_ROTATIONSENSOR_IOCTL_GET_DELAY		_IOW(HP_ROTATIONSENSOR_IOCTL_MAGIC, 4, int *)



#define HP_SENSORS_TYPE_NO			36
#define PSENSOR_CALL_BEGIN 			_IO(HP_SENSORS_TYPE_NO, 0x00)
#define PSENSOR_CALL_END   			_IO(HP_SENSORS_TYPE_NO, 0x01)
#define GSENSOR_GET_ORIENTATION   	_IO(HP_SENSORS_TYPE_NO, 0x02)
#if defined(ERISED) || defined(MACH_TYPE_ERISED)
#define ENABLE_GSENSOR				_IO(HP_SENSORS_TYPE_NO, 0x03)
#define DISABLE_GSENSOR				_IO(HP_SENSORS_TYPE_NO, 0x04)
#endif

//[HPP]guoye-For interrupt mode
#define HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_START_EVENT _IOW(LSM303DLH_ACC_IOCTL_BASE, 5, int)
#define HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_STOP_EVENT _IOW(LSM303DLH_ACC_IOCTL_BASE, 6, int)
#define HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_SET_THRESHOLD _IOW(LSM303DLH_ACC_IOCTL_BASE, 7, int)
#define HP_ACCELEROMETERSENSOR_IOCTL_INTERRUPT_GET_EVENT _IOR(LSM303DLH_ACC_IOCTL_BASE, 8, int)

#define EVENT_6D_BIT 0X80
#define EVENT_ZH_BIT 0x20
#define EVENT_ZL_BIT 0x10
#define EVENT_YH_BIT 0x08
#define EVENT_YL_BIT 0x04
#define EVENT_XH_BIT 0x02
#define EVENT_XL_BIT 0x01
//[HPP]guoye

#ifdef __KERNEL__

#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <mach/mpp.h>


#define SENSOR_MODULE_NAME "hp-sensors"

#if defined(MACH_TYPE_ERISED)
typedef enum {
CAM_ANGLE_0		= 0x0,
CAM_ANGLE_90, 
CAM_ANGLE_180,
CAM_ANGLE_270,
CAM_ANGLE_UNKNOWN,
} ANGLE_TYPE;

typedef struct {
	struct mutex    mutex;
	bool            ischanged;
	int x;
	int y;
	int z;
} GSENSOR_ORIENTATION;
#endif

typedef struct {
    struct mpp*             pmpp;
    int                     irq_num;
    struct mutex		    irq_mutex;
    volatile unsigned int 	irq_status;
    struct mutex		    call_status_mutex;
    volatile unsigned int   call_status;
} PSENSOR_STATUS;


typedef struct {
	struct class* 			sensors_class;
	struct cdev* 			cdev;
	struct device*			pdevice;
	struct workqueue_struct*	sensors_workqueue;
	wait_queue_head_t 		sensors_wait;
	PSENSOR_STATUS			psensor_status;
    
    #ifdef MACH_TYPE_ERISED
	GSENSOR_ORIENTATION		gsensor_orientation;
    #endif
    
	dev_t 					devno;
    struct mutex            sensor_mutex;
	bool 					sensor_is_on;
} sensors_dev;
#endif

#endif

