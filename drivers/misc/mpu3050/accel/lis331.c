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
 *      @file   lis331.c
 *      @brief  Accelerometer setup and handling methods for ST LIS331
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

/* full scale setting - register & mask */
#define ACCEL_ST_CTRL_REG1         (0x20)
#define ACCEL_ST_CTRL_REG2         (0x21)
#define ACCEL_ST_CTRL_REG3         (0x22)
#define ACCEL_ST_CTRL_REG4         (0x23)
#define ACCEL_ST_CTRL_REG5         (0x24)
#define ACCEL_ST_HP_FILTER_RESET   (0x25)
#define ACCEL_ST_REFERENCE         (0x26)
#define ACCEL_ST_STATUS_REG        (0x27)
#define ACCEL_ST_OUT_X_L           (0x28)
#define ACCEL_ST_OUT_X_H           (0x29)
#define ACCEL_ST_OUT_Y_L           (0x2a)
#define ACCEL_ST_OUT_Y_H           (0x2b)
#define ACCEL_ST_OUT_Z_L           (0x2b)
#define ACCEL_ST_OUT_Z_H           (0x2d)

#define ACCEL_ST_INT1_CFG          (0x30)
#define ACCEL_ST_INT1_SRC          (0x31)
#define ACCEL_ST_INT1_THS          (0x32)
#define ACCEL_ST_INT1_DURATION     (0x33)

#define ACCEL_ST_INT2_CFG          (0x34)
#define ACCEL_ST_INT2_SRC          (0x35)
#define ACCEL_ST_INT2_THS          (0x36)
#define ACCEL_ST_INT2_DURATION     (0x37)

#define ACCEL_ST_CTRL_MASK         (0x30)
#define ACCEL_ST_SLEEP_MASK        (0x20)

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

/*****************************************
    Accelerometer Initialization Functions
*****************************************/

int lis331dlh_suspend(void *mlsl_handle,
		      struct ext_slave_descr *slave,
		      struct ext_slave_platform_data *pdata)
{
	int result;

	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_CTRL_REG1, 0x47);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_CTRL_REG2, 0x0f);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_CTRL_REG3, 0x00);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_CTRL_REG4, 0x40);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_INT1_THS, 0x05);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_INT1_DURATION, 0x01);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_INT1_CFG, 0x2a);
	return result;
}

int lis331dlh_resume(void *mlsl_handle,
		     struct ext_slave_descr *slave,
		     struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	unsigned char reg;

	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_CTRL_REG1, 0x37);
	ERROR_CHECK(result);
	MLOSSleep(500);

	/* Full Scale */
	reg = 0x40;
	reg &= ~ACCEL_ST_CTRL_MASK;
	if (slave->range.mantissa == 2
	    && slave->range.fraction == 480) {
		reg |= 0x00;
	} else if (slave->range.mantissa == 4
		   && slave->range.fraction == 960) {
		reg |= 0x10;
	} else if (slave->range.mantissa == 8
		   && slave->range.fraction == 1920) {
		reg |= 0x30;
	}
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_CTRL_REG4, reg);
	ERROR_CHECK(result);

	/* Configure high pass filter */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_CTRL_REG2, 0x0F);
	ERROR_CHECK(result);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_CTRL_REG3, 0x00);
	ERROR_CHECK(result);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_INT1_THS, 0x10);
	ERROR_CHECK(result);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_INT1_DURATION, 0x10);
	ERROR_CHECK(result);
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       ACCEL_ST_INT1_CFG, 0x00);
	ERROR_CHECK(result);
	MLOSSleep(50);
	return result;
}

int lis331dlh_read(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata,
		   unsigned char *data)
{
	return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
}

struct ext_slave_descr lis331dlh_descr = {
	/*.suspend          = */ lis331dlh_suspend,
	/*.resume           = */ lis331dlh_resume,
	/*.read             = */ lis331dlh_read,
	/*.name             = */ "lis331dlh",
	/*.type             = */ EXT_SLAVE_TYPE_ACCELEROMETER,
	/*.id               = */ ACCEL_ID_LIS331,
	/*.reg              = */ 0x28,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_BIG_ENDIAN,
	/*.range            = */ {2, 480},
};

struct ext_slave_descr *lis331dlh_get_slave_descr(void)
{
	return &lis331dlh_descr;
}

#ifdef __KERNEL__
EXPORT_SYMBOL(lis331dlh_get_slave_descr);
#endif

/**
 *  @}
**/
