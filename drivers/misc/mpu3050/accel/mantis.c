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

#define ACCEL_ST_SLEEP_REG          (0x20)
#define ACCEL_ST_SLEEP_MASK         (0x20)

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

/*****************************************
    Accelerometer Initialization Functions
*****************************************/

int mantis_suspend(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	return ML_SUCCESS;
}

/* full scale setting - register & mask */
#define ACCEL_ST_CTRL_REG          (0x23)
#define ACCEL_ST_CTRL_MASK         (0x30)

int mantis_resume(void *mlsl_handle,
		  struct ext_slave_descr *slave,
		  struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
#ifdef M_HW
	unsigned char reg;

	if (slave->range.mantissa == 2)
		reg = 0;
	else if (slave->range.mantissa == 4)
		reg = 1 << 3;
	else if (slave->range.mantissa == 8)
		reg = 2 << 3;
	else if (slave->range.mantissa == 16)
		reg = 3 << 3;
	else
		return ML_ERROR;

	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       MPUREG_ACCEL_CONFIG, reg);
#endif
	return result;
}

int mantis_read(void *mlsl_handle,
		struct ext_slave_descr *slave,
		struct ext_slave_platform_data *pdata, unsigned char *data)
{
	return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
}

struct ext_slave_descr mantis_descr = {
	/*.suspend          = */ mantis_suspend,
	/*.resume           = */ mantis_resume,
	/*.read             = */ mantis_read,
	/*.name             = */ "mantis",
	/*.type             = */ EXT_SLAVE_TYPE_ACCELEROMETER,
	/*.id               = */ ID_INVALID,
	/*.reg              = */ 0xA8,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_BIG_ENDIAN,
	/*.range            = */ {2, 0},
};

struct ext_slave_descr *mantis_get_slave_descr(void)
{
	return &mantis_descr;
}

#ifdef __KERNEL__
EXPORT_SYMBOL(mantis_get_slave_descr);
#endif

/**
 *  @}
 */

