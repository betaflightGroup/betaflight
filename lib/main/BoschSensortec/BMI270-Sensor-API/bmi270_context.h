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
* @file       bmi270_context.h
* @date       2020-11-04
* @version    v2.63.1
*
*/

/**
 * \ingroup bmi2xy
 * \defgroup bmi270_context BMI270_CONTEXT
 * @brief Sensor driver for BMI270_CONTEXT sensor
 */

#ifndef BMI270_CONTEXT_H_
#define BMI270_CONTEXT_H_

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

/*! @name BMI270_CONTEXT Chip identifier */
#define BMI270_CONTEXT_CHIP_ID                       UINT8_C(0x24)

/*! @name BMI270_CONTEXT feature input start addresses */
#define BMI270_CONTEXT_CONFIG_ID_STRT_ADDR           UINT8_C(0x06)
#define BMI270_CONTEXT_STEP_CNT_1_STRT_ADDR          UINT8_C(0x00)
#define BMI270_CONTEXT_STEP_CNT_4_STRT_ADDR          UINT8_C(0x02)
#define BMI270_CONTEXT_MAX_BURST_LEN_STRT_ADDR       UINT8_C(0x08)
#define BMI270_CONTEXT_CRT_GYRO_SELF_TEST_STRT_ADDR  UINT8_C(0x09)
#define BMI270_CONTEXT_ABORT_STRT_ADDR               UINT8_C(0x09)
#define BMI270_CONTEXT_NVM_PROG_PREP_STRT_ADDR       UINT8_C(0x0A)
#define BMI270_CONTEXT_ACT_RGN_SETT_STRT_ADDR        UINT8_C(0x00)
#define BMI270_CONTEXT_ACT_RGN_STRT_ADDR             UINT8_C(0x0A)

/*! @name BMI270_CONTEXT feature output start addresses */
#define BMI270_CONTEXT_STEP_CNT_OUT_STRT_ADDR        UINT8_C(0x00)
#define BMI270_CONTEXT_GYR_USER_GAIN_OUT_STRT_ADDR   UINT8_C(0x04)
#define BMI270_CONTEXT_GYRO_CROSS_SENSE_STRT_ADDR    UINT8_C(0x0C)
#define BMI270_CONTEXT_NVM_VFRM_OUT_STRT_ADDR        UINT8_C(0x0E)

/*! @name Defines maximum number of pages */
#define BMI270_CONTEXT_MAX_PAGE_NUM                  UINT8_C(8)

/*! @name Defines maximum number of feature input configurations */
#define BMI270_CONTEXT_MAX_FEAT_IN                   UINT8_C(10)

/*! @name Defines maximum number of feature outputs */
#define BMI270_CONTEXT_MAX_FEAT_OUT                  UINT8_C(5)

/*! @name Mask definitions for feature interrupt status bits */
#define BMI270_CONTEXT_STEP_CNT_STATUS_MASK          UINT8_C(0x01)

/*! @name Mask definitions for feature interrupt mapping bits */
#define BMI270_C_INT_STEP_COUNTER_MASK               UINT8_C(0x01)
#define BMI270_C_INT_STEP_DETECTOR_MASK              UINT8_C(0x01)

/*! @name Defines maximum number of feature interrupts */
#define BMI270_C_MAX_INT_MAP                         UINT8_C(2)

/***************************************************************************/

/*!     BMI270_CONTEXT User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bmi270_context
 * \defgroup bmi270_contextApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bmi270_contextApiInit
 * \page bmi270_context_api_bmi270_context_init bmi270_context_init
 * \code
 * int8_t bmi270_context_init(struct bmi2_dev *dev);
 * \endcode
 * @details This API:
 *  1) updates the device structure with address of the configuration file.
 *  2) Initializes BMI270_CONTEXT sensor.
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
int8_t bmi270_context_init(struct bmi2_dev *dev);

/**
 * \ingroup bmi270_context
 * \defgroup bmi270_contextApiSensor Feature Set
 * @brief Enable / Disable features of the sensor
 */

/*!
 * \ingroup bmi270_contextApiSensor
 * \page bmi270_context_api_bmi270_context_sensor_enable bmi270_context_sensor_enable
 * \code
 * int8_t bmi270_context_sensor_enable(const uint8_t *sens_list, uint8_t n_sens, struct bmi2_dev *dev);
 * \endcode
 * @details This API selects the sensors/features to be enabled.
 *
 * @param[in]       sens_list   : Pointer to select the sensor/feature.
 * @param[in]       n_sens      : Number of sensors selected.
 * @param[in, out]  dev         : Structure instance of bmi2_dev.
 *
 * @note Sensors/features that can be enabled.
 *
 *@verbatim
 *    sens_list                |  Values
 * ----------------------------|-----------
 * BMI2_ACCEL                  |  0
 * BMI2_GYRO                   |  1
 * BMI2_AUX                    |  2
 * BMI2_STEP_DETECTOR          |  6
 * BMI2_STEP_COUNTER           |  7
 * BMI2_GYRO_GAIN_UPDATE       |  9
 * BMI2_ACTIVITY_RECOGNITION   |  34
 *@endverbatim
 *
 * @note :
 * example  uint8_t sens_list[2]  = {BMI2_ACCEL, BMI2_GYRO};
 *           uint8_t n_sens        = 2;
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_sensor_enable(const uint8_t *sens_list, uint8_t n_sens, struct bmi2_dev *dev);

/*!
 * \ingroup bmi270_contextApiSensor
 * \page bmi270_context_api_bmi270_context_sensor_disable bmi270_context_sensor_disable
 * \code
 * int8_t bmi270_context_sensor_disable(const uint8_t *sens_list, uint8_t n_sens, struct bmi2_dev *dev);
 * \endcode
 * @details This API selects the sensors/features to be disabled.
 *
 * @param[in]       sens_list   : Pointer to select the sensor/feature.
 * @param[in]       n_sens      : Number of sensors selected.
 * @param[in, out]  dev         : Structure instance of bmi2_dev.
 *
 * @note Sensors/features that can be disabled.
 *
 *@verbatim
 *    sens_list                |  Values
 * ----------------------------|-----------
 * BMI2_ACCEL                  |  0
 * BMI2_GYRO                   |  1
 * BMI2_AUX                    |  2
 * BMI2_STEP_DETECTOR          |  6
 * BMI2_STEP_COUNTER           |  7
 * BMI2_GYRO_GAIN_UPDATE       |  9
 * BMI2_ACTIVITY_RECOGNITION   |  34
 *@endverbatim
 *
 * @note :
 * example  uint8_t sens_list[2]  = {BMI2_ACCEL, BMI2_GYRO};
 *           uint8_t n_sens        = 2;
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_sensor_disable(const uint8_t *sens_list, uint8_t n_sens, struct bmi2_dev *dev);

/**
 * \ingroup bmi270_context
 * \defgroup bmi270_contextApiSensorC Sensor Configuration
 * @brief Enable / Disable feature configuration of the sensor
 */

/*!
 * \ingroup bmi270_contextApiSensorC
 * \page bmi270_context_api_bmi270_context_set_sensor_config bmi270_context_set_sensor_config
 * \code
 * int8_t bmi270_context_set_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens, struct bmi2_dev *dev);
 * \endcode
 * @details This API sets the sensor/feature configuration.
 *
 * @param[in]       sens_cfg     : Structure instance of bmi2_sens_config.
 * @param[in]       n_sens       : Number of sensors selected.
 * @param[in, out]  dev          : Structure instance of bmi2_dev.
 *
 * @note Sensors/features that can be configured
 *
 *@verbatim
 *    sens_list                |  Values
 * ----------------------------|-----------
 * BMI2_STEP_DETECTOR          |  6
 * BMI2_STEP_COUNTER           |  7
 * BMI2_STEP_COUNTER_PARAMS    |  28
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_set_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens, struct bmi2_dev *dev);

/*!
 * \ingroup bmi270_contextApiSensorC
 * \page bmi270_context_api_bmi270_context_get_sensor_config bmi270_context_get_sensor_config
 * \code
 * int8_t bmi270_context_get_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens, struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the sensor/feature configuration.
 *
 * @param[in]       sens_cfg     : Structure instance of bmi2_sens_config.
 * @param[in]       n_sens       : Number of sensors selected.
 * @param[in, out]  dev          : Structure instance of bmi2_dev.
 *
 * @note Sensors/features whose configurations can be read.
 *
 *@verbatim
 *    sens_list                |  Values
 * ----------------------------|-----------
 * BMI2_STEP_DETECTOR          |  6
 * BMI2_STEP_COUNTER           |  7
 * BMI2_STEP_COUNTER_PARAMS    |  28
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_get_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens, struct bmi2_dev *dev);

/**
 * \ingroup bmi270_context
 * \defgroup bmi270_contextApiSensorD Sensor Data
 * @brief Get sensor data
 */

/*!
 * \ingroup bmi270_contextApiSensorD
 * \page bmi270_context_api_bmi270_context_get_sensor_data bmi270_context_get_sensor_data
 * \code
 * int8_t bmi270_context_get_sensor_data(struct bmi2_sensor_data *sensor_data, uint8_t n_sens, struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the sensor/feature data for accelerometer, gyroscope,
 * auxiliary sensor, step counter, high-g, gyroscope user-gain update,
 * orientation, gyroscope cross sensitivity and error status for NVM and VFRM.
 *
 * @param[out] sensor_data   : Structure instance of bmi2_sensor_data.
 * @param[in]  n_sens        : Number of sensors selected.
 * @param[in]  dev           : Structure instance of bmi2_dev.
 *
 * @note Sensors/features whose data can be read
 *
 *@verbatim
 *  sens_list           |  Values
 * ---------------------|-----------
 * BMI2_STEP_COUNTER    |   7
 * BMI2_NVM_STATUS      |  38
 * BMI2_VFRM_STATUS     |  39
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_get_sensor_data(struct bmi2_sensor_data *sensor_data, uint8_t n_sens, struct bmi2_dev *dev);

/**
 * \ingroup bmi270_context
 * \defgroup bmi270_contextApiARecog Activity recognition settings
 * @brief Set / Get activity recognition settings of the sensor
 */

/*!
 * \ingroup bmi270_contextApiARecog
 * \page bmi270_context_api_bmi270_context_get_act_recg_sett bmi270_context_get_act_recg_sett
 * \code
 * int8_t bmi270_context_get_act_recg_sett(struct bmi2_act_recg_sett *sett, struct bmi2_dev *dev);
 * \endcode
 * @details This api is used for retrieving the following activity recognition settings currently set.
 * enable/disable post processing(0/1) by default -> 1(enable),
 * Setting the min & max Gini's diversity index (GDI) threshold. min_GDI_tres(0-0XFFFF) by default ->(0x06e1)
 * max_GDI_tres(0-0xFFFF) by default ->(0x0A66)
 * buffer size for post processing. range (1-0x0A) default -> (0x0A)
 * min segment confidence.  range (1-0x0A) default -> (0x0A)
 *
 * @param[in] sett  : Structure instance of bmi2_act_recg_sett.
 * @param[in] dev   : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_get_act_recg_sett(struct bmi2_act_recg_sett *sett, struct bmi2_dev *dev);

/*!
 * \ingroup bmi270_contextApiARecog
 * \page bmi270_context_api_bmi270_context_set_act_recg_sett bmi270_context_set_act_recg_sett
 * \code
 * int8_t bmi270_context_set_act_recg_sett(const struct bmi2_act_recg_sett *sett, struct bmi2_dev *dev);
 * \endcode
 * @details This api is used for setting the following activity recognition settings
 * enable/disable post processing(0/1) by default -> 1(enable),
 * Setting the min & max Gini's diversity index (GDI) threshold. min_GDI_tres(0-0XFFFF) by default ->(0x06e1)
 * max_GDI_tres(0-0xFFFF) by default ->(0x0A66)
 * buffer size for post processing. range (1-0x0A) default -> (0x0A)
 * min segment confidence.  range (1-0x0A) default -> (0x0A)
 *
 * @param[in] sett  : Structure instance of bmi2_act_recg_sett.
 * @param[in] dev   : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_set_act_recg_sett(const struct bmi2_act_recg_sett *sett, struct bmi2_dev *dev);

/**
 * \ingroup bmi270_contex
 * \defgroup bmi270_contextApiactOut Activity Output
 * @brief Activity output operations of the sensor
 */

/*!
 * \ingroup bmi270_contextApiactOut
 * \page bmi270_context_api_bmi270_context_get_act_recog_output bmi270_context_get_act_recog_output
 * \code
 * int8_t bmi270_context_get_act_recog_output(struct bmi2_act_recog_output *act_recog,
 *                                uint16_t *act_frm_len,
 *                                struct bmi2_fifo_frame *fifo,
 *                                const struct bmi2_dev *dev);
 *
 * \endcode
 * @details This internal API is used to parse the activity output from the
 * FIFO in header mode.
 *
 * @param[out] act_recog    : Pointer to buffer where the parsed activity data
 *                           bytes are stored.
 * @param[in] act_frm_len   : Number of activity frames parsed.
 * @param[in] fifo          : Structure instance of bmi2_fifo_frame.
 * @param[in] dev           : Structure instance of bmi2_dev.
 *
 * @verbatim
 * ----------------------------------------------------------------------------
 *  bmi2_act_rec_output     |
 *  Structure parameters    |               Description
 *--------------------------|--------------------------------------------------
 *      time_stamp          |  time-stamp (expressed in 50Hz ticks)
 * -------------------------|---------------------------------------------------
 *      type                |           Type of activity
 * -------------------------|---------------------------------------------------
 *      stat                |           Activity status
 * -------------------------|---------------------------------------------------
 * @endverbatim
 *
 *@verbatim
 *  type    |  Activities
 *----------|---------------------
 *  0       |  UNKNOWN
 *  1       |  STILL
 *  2       |  WALK
 *  3       |  RUN
 *  4       |  BIKE
 *  5       | VEHICLE
 *  6       | TILTED
 *@endverbatim
 *
 *
 *@verbatim
 *  stat    |  Activity status
 *----------|---------------------
 *  1       |  START
 *  2       |  END
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_get_act_recog_output(struct bmi2_act_recog_output *act_recog,
                                           uint16_t *act_frm_len,
                                           struct bmi2_fifo_frame *fifo,
                                           const struct bmi2_dev *dev);

/**
 * \ingroup bmi270_contex
 * \defgroup bmi270_contextApiGyroUG Gyro User Gain
 * @brief Set / Get Gyro User Gain of the sensor
 */

/*!
 * \ingroup bmi270_contextApiGyroUG
 * \page bmi270_context_api_bmi270_context_update_gyro_user_gain bmi270_context_update_gyro_user_gain
 * \code
 * int8_t bmi270_context_update_gyro_user_gain(const struct bmi2_gyro_user_gain_config *user_gain, struct bmi2_dev *dev);
 * \endcode
 * @details This API updates the gyroscope user-gain.
 *
 * @param[in] user_gain      : Structure that stores user-gain configurations.
 * @param[in] dev            : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_update_gyro_user_gain(const struct bmi2_gyro_user_gain_config *user_gain, struct bmi2_dev *dev);

/*!
 * \ingroup bmi270_contextApiGyroUG
 * \page bmi270_context_api_bmi270_context_read_gyro_user_gain bmi270_context_read_gyro_user_gain
 * \code
 * int8_t bmi270_context_read_gyro_user_gain(struct bmi2_gyro_user_gain_data *gyr_usr_gain, const struct bmi2_dev *dev);
 * \endcode
 * @details This API reads the compensated gyroscope user-gain values.
 *
 * @param[out] gyr_usr_gain   : Structure that stores gain values.
 * @param[in]  dev            : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_read_gyro_user_gain(struct bmi2_gyro_user_gain_data *gyr_usr_gain, struct bmi2_dev *dev);

/*!
 * \ingroup bmi270_contextApiInt
 * \page bmi270_context_api_bmi270_context_map_feat_int bmi270_context_map_feat_int
 * \code
 * int8_t bmi270_context_map_feat_int(const struct bmi2_sens_int_config *sens_int, uint8_t n_sens, struct bmi2_dev *dev)
 * \endcode
 * @details This API maps/unmaps feature interrupts to that of interrupt pins.
 *
 * @param[in] sens_int     : Structure instance of bmi2_sens_int_config.
 * @param[in] n_sens       : Number of interrupts to be mapped.
 * @param[in] dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi270_context_map_feat_int(const struct bmi2_sens_int_config *sens_int, uint8_t n_sens, struct bmi2_dev *dev);

/******************************************************************************/
/*! @name       C++ Guard Macros                                      */
/******************************************************************************/
#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* BMI270_CONTEXT_H_ */
