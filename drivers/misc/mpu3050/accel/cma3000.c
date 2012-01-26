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
/*******************************************************************************
 *
 * $Id: cma3000.c 3863 2010-10-08 22:05:31Z nroyer $
 *
 *******************************************************************************/

/** 
 *  @defgroup   ACCELDL (Motion Library - Accelerometer Driver Layer)
 *  @brief      Provides the interface to setup and handle an accelerometers
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   cma3000.c
 *      @brief  Accelerometer setup and handling methods for VTI CMA3000
**/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#ifdef __KERNEL__
#include <linux/module.h>
#endif
#include "mpu3050.h"
#include "mlsl.h"
#include "mlos.h"
#include "accel.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-acc"

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

/*****************************************
    Accelerometer Initialization Functions
*****************************************/

int cma3000_suspend(mlsl_handle_t mlsl_handle,
                   struct ext_slave_descr *slave,
                   struct ext_slave_platform_data *pdata)
{
    int result;
    // RAM reset
    result = MLSLSerialWriteSingle(mlsl_handle, pdata->address, 0x1d, 0xcd );
    return result;
}

int cma3000_resume(mlsl_handle_t mlsl_handle,
                  struct ext_slave_descr *slave, 
                  struct ext_slave_platform_data *pdata)
{
    int result = ML_SUCCESS;



    return ML_SUCCESS;
}

int cma3000_read(mlsl_handle_t mlsl_handle,
               struct ext_slave_descr *slave, 
               struct ext_slave_platform_data *pdata,
               unsigned char * data)
{
    return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
}
struct ext_slave_descr cma3000_descr = {
    /*.suspend          = */ cma3000_suspend,
    /*.resume           = */ cma3000_resume,
    /*.read             = */ cma3000_read,
    /*.name             = */ "cma3000",
    /*.type             = */ EXT_SENS_TYPE_ACCELEROMETER,
    /*.id               = */ ID_INVALID, // fixme - id to added when support becomes available
    /*.reg              = */ 0x06,
    /*.len              = */ 6,
    /*.endian           = */ EXT_SLAVE_LITTLE_ENDIAN,
    /*.range            = */ 65536,
};

struct ext_slave_descr* cma3000_get_slave_descr(voiid)
{
    return &cma3000_descr;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(cma3000_get_slave_descr);
#endif

/**
 *  @}
**/
