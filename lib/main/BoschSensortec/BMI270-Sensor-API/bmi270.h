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
* @file bmi270.h
* @date 2020-01-10
* @version  v2.46.1
*
*/#ifndef BMI270_H_
#define BMI270_H_

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
#define BMI270_CHIP_ID                      UINT8_C(0x24)

/*! @name BMI270 feature input start addresses */
#define BMI270_MAX_BURST_LEN_STRT_ADDR      UINT8_C(0x02)
#define BMI270_CRT_GYRO_SELF_TEST_STRT_ADDR UINT8_C(0x03)
#define BMI270_ABORT_STRT_ADDR              UINT8_C(0x03)
#define BMI270_AXIS_MAP_STRT_ADDR           UINT8_C(0x04)
#define BMI270_GYRO_SELF_OFF_STRT_ADDR      UINT8_C(0x05)
#define BMI270_NVM_PROG_PREP_STRT_ADDR      UINT8_C(0x05)
#define BMI270_GYRO_GAIN_UPDATE_STRT_ADDR   UINT8_C(0x06)
#define BMI270_ANY_MOT_STRT_ADDR            UINT8_C(0x0C)
#define BMI270_NO_MOT_STRT_ADDR             UINT8_C(0x00)
#define BMI270_SIG_MOT_STRT_ADDR            UINT8_C(0x04)
#define BMI270_STEP_CNT_1_STRT_ADDR         UINT8_C(0x00)
#define BMI270_STEP_CNT_4_STRT_ADDR         UINT8_C(0x02)
#define BMI270_WRIST_GEST_STRT_ADDR         UINT8_C(0x06)
#define BMI270_WRIST_WEAR_WAKE_UP_STRT_ADDR UINT8_C(0x00)

/*! @name BMI270 feature output start addresses */
#define BMI270_STEP_CNT_OUT_STRT_ADDR       UINT8_C(0x00)
#define BMI270_STEP_ACT_OUT_STRT_ADDR       UINT8_C(0x04)
#define BMI270_WRIST_GEST_OUT_STRT_ADDR     UINT8_C(0x06)
#define BMI270_GYR_USER_GAIN_OUT_STRT_ADDR  UINT8_C(0x08)
#define BMI270_GYRO_CROSS_SENSE_STRT_ADDR   UINT8_C(0x0C)
#define BMI270_NVM_VFRM_OUT_STRT_ADDR       UINT8_C(0x0E)

/*! @name Defines maximum number of pages */
#define BMI270_MAX_PAGE_NUM                 UINT8_C(8)

/*! @name Defines maximum number of feature input configurations */
#define BMI270_MAX_FEAT_IN                  UINT8_C(16)

/*! @name Defines maximum number of feature outputs */
#define BMI270_MAX_FEAT_OUT                 UINT8_C(7)

/*! @name Mask definitions for feature interrupt status bits */
#define BMI270_SIG_MOT_STATUS_MASK          UINT8_C(0x01)
#define BMI270_STEP_CNT_STATUS_MASK         UINT8_C(0x02)
#define BMI270_STEP_ACT_STATUS_MASK         UINT8_C(0x04)
#define BMI270_WRIST_WAKE_UP_STATUS_MASK    UINT8_C(0x08)
#define BMI270_WRIST_GEST_STATUS_MASK       UINT8_C(0x10)
#define BMI270_NO_MOT_STATUS_MASK           UINT8_C(0x20)
#define BMI270_ANY_MOT_STATUS_MASK          UINT8_C(0x40)

/***************************************************************************/

/*!     BMI270 User Interface function prototypes
 ****************************************************************************/

/*!
 *  @brief This API:
 *  1) updates the device structure with address of the configuration file.
 *  2) Initializes BMI270 sensor.
 *  3) Writes the configuration file.
 *  4) Updates the feature offset parameters in the device structure.
 *  5) Updates the maximum number of pages, in the device structure.
 *
 * @param[in, out] dev      : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 *
 * @retval BMI2_OK - Success
 * @retval BMI2_E_NULL_PTR - Error: Null pointer error
 * @retval BMI2_E_COM_FAIL - Error: Communication fail
 * @retval BMI2_E_DEV_NOT_FOUND - Invalid device
 */
int8_t bmi270_init(struct bmi2_dev *dev);

/******************************************************************************/
/*! @name       C++ Guard Macros                                      */
/******************************************************************************/
#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* BMI270_H_ */
