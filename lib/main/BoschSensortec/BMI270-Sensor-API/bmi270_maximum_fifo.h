/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bmi270_maximum_fifo.h
* @date       2020-11-04
* @version    v2.63.1
*
*/

/**
 * \ingroup bmi2xy
 * \defgroup bmi270_maximum_fifo BMI270_MAXIMUM_FIFO
 * @brief Sensor driver for BMI270_MAXIMUM_FIFO sensor
 */

#ifndef BMI270_MAXIMUM_FIFO_H_
#define BMI270_MAXIMUM_FIFO_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************/

/*!             Header files
 ****************************************************************************/
#include "bmi2.h"

/***************************************************************************/

/*!               Macro definitions
 ****************************************************************************/

/*! @name BMI270 Chip identifier */
#define BMI270_MAXIMUM_FIFO_CHIP_ID       UINT8_C(0x24)

/*! @name Defines maximum number of pages */
#define BMI270_MAXIMUM_FIFO_MAX_PAGE_NUM  UINT8_C(0)

/*! @name Defines maximum number of feature input configurations */
#define BMI270_MAXIMUM_FIFO_MAX_FEAT_IN   UINT8_C(0)

/*! @name Defines maximum number of feature outputs */
#define BMI270_MAXIMUM_FIFO_MAX_FEAT_OUT  UINT8_C(0)

/*! @name Mask definitions for feature interrupt status bits */

/***************************************************************************/

/*!     BMI270 User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bmi270_maximum_fifo
 * \defgroup bmi270_maximum_fifoApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bmi270_maximum_fifoApiInit
 * \page bmi270_maximum_fifo_api_bmi270_maximum_fifo_init bmi270_maximum_fifo_init
 * \code
 * int8_t bmi270_maximum_fifo_init(struct bmi2_dev *dev);
 * \endcode
 * @details This API:
 *  1) updates the device structure with address of the configuration file.
 *  2) Initializes BMI270 sensor.
 *  3) Writes the configuration file.
 *  4) Updates the feature offset parameters in the device structure.
 *  5) Updates the maximum number of pages, in the device structure.
 *
 * @param[in, out] dev      : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_maximum_fifo_init(struct bmi2_dev *dev);

/******************************************************************************/
/*! @name       C++ Guard Macros                                      */
/******************************************************************************/
#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* BMI270_MAXIMUM_FIFO_H_ */
