/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */

/**
 *  @defgroup   ACCELDL (Motion Library - Accelerometer Driver Layer)
 *  @brief      Provides the interface to setup and handle an accelerometers
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   mma8451.c
 *      @brief  Accelerometer setup and handling methods for Freescale MMA8451
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#ifdef __KERNEL__
#include <linux/module.h>
#endif

#include "mpu.h"
#include "mlsl.h"
#include "mlos.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-acc"

#define ACCEL_MMA8451_SLEEP_REG      (0x2A)
#define ACCEL_MMA8451_SLEEP_MASK     (0x01)


/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

/*****************************************
    Accelerometer Initialization Functions
*****************************************/

int mma8451_suspend(void *mlsl_handle,
		    struct ext_slave_descr *slave,
		    struct ext_slave_platform_data *pdata)
{
	int result;
	unsigned char reg;
	result =
	    MLSLSerialRead(mlsl_handle, pdata->address,
			   ACCEL_MMA8451_SLEEP_REG, 1, &reg);
	ERROR_CHECK(result);
	reg &= ~ACCEL_MMA8451_SLEEP_MASK;
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  ACCEL_MMA8451_SLEEP_REG, reg);
	ERROR_CHECK(result);
	return result;
}

/* full scale setting - register & mask */
#define ACCEL_MMA8451_CTRL_REG      (0x0E)
#define ACCEL_MMA8451_CTRL_MASK     (0x03)

int mma8451_resume(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	unsigned char reg;

	result =
	    MLSLSerialRead(mlsl_handle, pdata->address, 0x0E, 1, &reg);
	ERROR_CHECK(result);

	/* data rate = 200Hz */

	/* Full Scale */
	reg &= ~ACCEL_MMA8451_CTRL_MASK;
	if (slave->range.mantissa == 2)
		reg |= 0x0;
	else if (slave->range.mantissa == 4)
		reg |= 0x1;
	else if (slave->range.mantissa == 8)
		reg |= 0x2;

	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address, 0x0E, reg);
	ERROR_CHECK(result);
	/* 200Hz + active mode */
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address, 0x2A, 0x11);
	ERROR_CHECK(result);

	return result;
}

int mma8451_read(void *mlsl_handle,
		 struct ext_slave_descr *slave,
		 struct ext_slave_platform_data *pdata,
		 unsigned char *data)
{
	return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
}

struct ext_slave_descr mma8451_descr = {
	/*.suspend          = */ mma8451_suspend,
	/*.resume           = */ mma8451_resume,
	/*.read             = */ mma8451_read,
	/*.name             = */ "mma8451",
	/*.type             = */ EXT_SLAVE_TYPE_ACCELEROMETER,
	/*.id               = */ ACCEL_ID_MMA8451,
	/*.reg              = */ 0x00,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_FS16_BIG_ENDIAN,
	/*.range            = */ {2, 0},
};

struct ext_slave_descr *mma8451_get_slave_descr(void)
{
	return &mma8451_descr;
}

#ifdef __KERNEL__
EXPORT_SYMBOL(mma8451_get_slave_descr);
#endif

/**
 *  @}
**/
