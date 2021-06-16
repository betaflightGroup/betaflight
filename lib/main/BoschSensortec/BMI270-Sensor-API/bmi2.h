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
* @file       bmi2.h
* @date       2020-11-04
* @version    v2.63.1
*
*/

/*!
 * @defgroup bmi2xy BMI2XY
 */

/**
 * \ingroup bmi2xy
 * \defgroup bmi2 BMI2
 * @brief Sensor driver for BMI2 sensor
 */

#ifndef BMI2_H_
#define BMI2_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************/

/*!             Header files
 ****************************************************************************/
#include "bmi2_defs.h"

/***************************************************************************/

/*!     BMI2XY User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bmi2ApiInit
 * \page bmi2_api_bmi2_sec_init bmi2_sec_init
 * \code
 * int8_t bmi2_sec_init(struct bmi2_dev *dev);
 * \endcode
 * @details This API is the entry point for bmi2 sensor. It selects between
 * I2C/SPI interface, based on user selection. It also reads the chip-id of
 * the sensor.
 *
 * @param[in,out] dev  : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_sec_init(struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiRegs Registers
 * @brief Set / Get data from the given register address of the sensor
 */

/*!
 * \ingroup bmi2ApiRegs
 * \page bmi2_api_bmi2_get_regs bmi2_get_regs
 * \code
 * int8_t bmi2_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi2_dev *dev);
 * \endcode
 * @details This API reads the data from the given register address of bmi2
 * sensor.
 *
 * @param[in] reg_addr  : Register address from which data is read.
 * @param[out] data     : Pointer to data buffer where read data is stored.
 * @param[in] len       : No. of bytes of data to be read.
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * @note For most of the registers auto address increment applies, with the
 * exception of a few special registers, which trap the address. For e.g.,
 * Register address - 0x26, 0x5E.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiRegs
 * \page bmi2_api_bmi2_set_regs bmi2_set_regs
 * \code
 * int8_t bmi2_set_regs(uint8_t reg_addr, const uint8_t *data, uint16_t len, struct bmi2_dev *dev);
 * \endcode
 * @details This API writes data to the given register address of bmi2 sensor.
 *
 * @param[in] reg_addr  : Register address to which the data is written.
 * @param[in] data      : Pointer to data buffer in which data to be written
 *            is stored.
 * @param[in] len       : No. of bytes of data to be written.
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_regs(uint8_t reg_addr, const uint8_t *data, uint16_t len, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiSR Soft reset
 * @brief Set / Get data from the given register address of the sensor
 */

/*!
 * \ingroup bmi2ApiSR
 * \page bmi2_api_bmi2_soft_reset bmi2_soft_reset
 * \code
 * int8_t bmi2_soft_reset(struct bmi2_dev *dev);
 * \endcode
 * @details This API resets bmi2 sensor. All registers are overwritten with
 * their default values.
 *
 * @note If selected interface is SPI, an extra dummy byte is read to bring the
 * interface back to SPI from default, after the soft reset command.
 *
 * @param[in] dev : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_soft_reset(struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiConfig Configuration
 * @brief Functions related to configuration of the sensor
 */

/*!
 * \ingroup bmi2ApiConfig
 * \page bmi2_api_bmi2_get_config_file_version bmi2_get_config_file_version
 * \code
 * int8_t bmi2_get_config_file_version(uint8_t *config_major, uint8_t *config_minor, struct bmi2_dev *dev);
 * \endcode
 * @details This API is used to get the config file major and minor information.
 *
 * @param[in] dev   : Structure instance of bmi2_dev.
 * @param[out] config_major    : pointer to data buffer to store the config major.
 * @param[out] config_minor    : pointer to data buffer to store the config minor.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_config_file_version(uint8_t *config_major, uint8_t *config_minor, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiPowersave Advanced power save mode
 * @brief Set / Get Advanced power save mode of the sensor
 */

/*!
 * \ingroup bmi2ApiPowersave
 * \page bmi2_api_bmi2_set_adv_power_save bmi2_set_adv_power_save
 * \code
 * int8_t bmi2_set_adv_power_save(uint8_t enable, struct bmi2_dev *dev);
 * \endcode
 * @details This API enables/disables the advance power save mode in the sensor.
 *
 * @param[in] enable         : To enable/disable advance power mode.
 * @param[in] dev            : Structure instance of bmi2_dev.
 *
 *@verbatim
 *    Enable    |  Description
 * -------------|---------------
 * BMI2_DISABLE | Disables advance power save.
 * BMI2_ENABLE  | Enables advance power save.
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_adv_power_save(uint8_t enable, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiPowersave
 * \page bmi2_api_bmi2_get_adv_power_save bmi2_get_adv_power_save
 * \code
 * int8_t bmi2_get_adv_power_save(uint8_t *aps_status, struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the status of advance power save mode in the sensor.
 *
 * @param[out] aps_status    : Pointer to get the status of APS mode.
 * @param[in] dev            : Structure instance of bmi2_dev.
 *
 *@verbatim
 *  aps_status  |  Description
 * -------------|---------------
 * BMI2_DISABLE | Advance power save disabled.
 * BMI2_ENABLE  | Advance power save enabled.
 *@endverbatim
 *
 * @return Result of API execution status
 *
 * @retval BMI2_OK - Success.
 * @retval BMI2_E_NULL_PTR - Error: Null pointer error
 * @retval BMI2_E_COM_FAIL - Error: Communication fail
 * @retval BMI2_E_SET_APS_FAIL - Error: Set Advance Power Save Fail
 */
int8_t bmi2_get_adv_power_save(uint8_t *aps_status, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiConfig
 * \page bmi2_api_bmi2_write_config_file bmi2_write_config_file
 * \code
 * int8_t bmi2_write_config_file(struct bmi2_dev *dev);
 * \endcode
 * @details This API loads the configuration file to the bmi2 sensor.
 *
 * @param[in] dev           : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_write_config_file(struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiInt Interrupt
 * @brief Interrupt operations of the sensor
 */

/*!
 * \ingroup bmi2ApiInt
 * \page bmi2_api_bmi2_set_int_pin_config bmi2_set_int_pin_config
 * \code
 * int8_t bmi2_set_int_pin_config(const struct bmi2_int_pin_config *int_cfg, struct bmi2_dev *dev);
 * \endcode
 * @details This API sets:
 *        1) The input output configuration of the selected interrupt pin:
 *           INT1 or INT2.
 *        2) The interrupt mode: permanently latched or non-latched.
 *
 * @param[in] int_cfg       : Structure instance of bmi2_int_pin_config.
 * @param[in] dev           : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_int_pin_config(const struct bmi2_int_pin_config *int_cfg, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiInt
 * \page bmi2_api_bmi2_get_int_pin_config bmi2_get_int_pin_config
 * \code
 * int8_t bmi2_get_int_pin_config(struct bmi2_int_pin_config *int_cfg, const struct bmi2_dev *dev);
 * \endcode
 * @details This API gets:
 *        1) The input output configuration of the selected interrupt pin:
 *           INT1 or INT2.
 *        2) The interrupt mode: permanently latched or non-latched.
 *
 * @param[in,out] int_cfg  : Structure instance of bmi2_int_pin_config.
 * @param[in]     dev      : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_int_pin_config(struct bmi2_int_pin_config *int_cfg, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiInt
 * \page bmi2_api_bmi2_get_int_status bmi2_get_int_status
 * \code
 * int8_t bmi2_get_int_status(uint16_t *int_status, const struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the interrupt status of both feature and data
 * interrupts.
 *
 * @param[out] int_status    : Pointer to get the status of the interrupts.
 * @param[in]  dev           : Structure instance of bmi2_dev.
 *
 *@verbatim
 * int_status |  Status
 * -----------|------------
 * 0x00       |  BIT0
 * 0x01       |  BIT1
 * 0x02       |  BIT2
 * 0x03       |  BIT3
 * 0x04       |  BIT4
 * 0x05       |  BIT5
 * 0x06       |  BIT6
 * 0x07       |  BIT7
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_int_status(uint16_t *int_status, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiSensorC Sensor Configuration
 * @brief Enable / Disable feature configuration of the sensor
 */

/*!
 * \ingroup bmi2ApiSensorC
 * \page bmi2_api_bmi2_set_sensor_config bmi2_set_sensor_config
 * \code
 * int8_t bmi2_set_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens, struct bmi2_dev *dev);
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
 * BMI2_ACCEL                  |  0
 * BMI2_GYRO                   |  1
 * BMI2_AUX                    |  2
 * BMI2_GYRO_GAIN_UPDATE       |  9
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiSensorC
 * \page bmi2_api_bmi2_get_sensor_config bmi2_get_sensor_config
 * \code
 * int8_t bmi2_get_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens, struct bmi2_dev *dev);
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
 *  sens_list               |  Values
 * -------------------------|-----------
 * BMI2_ACCEL                  |  0
 * BMI2_GYRO                   |  1
 * BMI2_AUX                    |  2
 * BMI2_GYRO_GAIN_UPDATE       |  9
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiSensor Feature Set
 * @brief Enable / Disable features of the sensor
 */

/*!
 * \ingroup bmi2ApiSensor
 * \page bmi2_api_bmi2_sensor_enable bmi2_sensor_enable
 * \code
 * int8_t bmi2_sensor_enable(const uint8_t *sens_list, uint8_t n_sens, struct bmi2_dev *dev);
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
 *    sens_list             |  Values
 * -------------------------|-----------
 * BMI2_ACCEL                  |  0
 * BMI2_GYRO                   |  1
 * BMI2_AUX                    |  2
 * BMI2_TEMP                   |  31
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
int8_t bmi2_sensor_enable(const uint8_t *sens_list, uint8_t n_sens, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiSensor
 * \page bmi2_api_bmi2_sensor_disable bmi2_sensor_disable
 * \code
 * int8_t bmi2_sensor_disable(const uint8_t *sens_list, uint8_t n_sens, struct bmi2_dev *dev);
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
 * BMI2_TEMP                   |  31
 *@endverbatim
 *
 * @note :
 * example   uint8_t sens_list[2]  = {BMI2_ACCEL, BMI2_GYRO};
 *           uint8_t n_sens        = 2;
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_sensor_disable(const uint8_t *sens_list, uint8_t n_sens, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiSensorD Sensor Data
 * @brief Get sensor data
 */

/*!
 * \ingroup bmi2ApiSensorD
 * \page bmi2_api_bmi2_get_sensor_data bmi2_get_sensor_data
 * \code
 * int8_t bmi2_get_sensor_data(struct bmi2_sensor_data *sensor_data, uint8_t n_sens, struct bmi2_dev *dev);
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
 * BMI2_ACCEL           |  0
 * BMI2_GYRO            |  1
 * BMI2_AUX             |  2
 * BMI2_GYRO_GAIN_UPDATE|  12
 * BMI2_GYRO_CROSS_SENSE|  28
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_sensor_data(struct bmi2_sensor_data *sensor_data, uint8_t n_sens, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiFIFO FIFO
 * @brief FIFO operations of the sensor
 */

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_set_fifo_config bmi2_set_fifo_config
 * \code
 * int8_t bmi2_set_fifo_config(uint16_t config, uint8_t enable, struct bmi2_dev *dev);
 * \endcode
 * @details This API sets the FIFO configuration in the sensor.
 *
 * @param[in] config        : FIFO configurations to be enabled/disabled.
 * @param[in] enable        : Enable/Disable FIFO configurations.
 * @param[in] dev           : Structure instance of bmi2_dev.
 *
 *@verbatim
 *  enable      |  Description
 * -------------|---------------
 * BMI2_DISABLE | Disables FIFO configuration.
 * BMI2_ENABLE  | Enables FIFO configuration.
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_fifo_config(uint16_t config, uint8_t enable, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_get_fifo_config bmi2_get_fifo_config
 * \code
 * int8_t bmi2_get_fifo_config(uint16_t *fifo_config, const struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the FIFO configuration from the sensor.
 *
 * @param[out] fifo_config   : Pointer variable to get FIFO configuration value.
 * @param[in]  dev           : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_fifo_config(uint16_t *fifo_config, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_read_fifo_data bmi2_read_fifo_data
 * \code
 * int8_t bmi2_read_fifo_data(struct bmi2_fifo_frame *fifo, const struct bmi2_dev *dev);
 * \endcode
 * @details This API reads FIFO data.
 *
 * @param[in, out] fifo     : Structure instance of bmi2_fifo_frame.
 * @param[in]      dev      : Structure instance of bmi2_dev.
 *
 * @note APS has to be disabled before calling this function.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_read_fifo_data(struct bmi2_fifo_frame *fifo, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_extract_accel bmi2_extract_accel
 * \code
 * int8_t bmi2_extract_accel(struct bmi2_sens_axes_data *accel_data,
 *                          uint16_t *accel_length,
 *                          struct bmi2_fifo_frame *fifo,
 *                          const struct bmi2_dev *dev);
 * \endcode
 * @details This API parses and extracts the accelerometer frames from FIFO data read by
 * the "bmi2_read_fifo_data" API and stores it in the "accel_data" structure
 * instance.
 *
 * @param[out]    accel_data   : Structure instance of bmi2_sens_axes_data
 *                               where the parsed data bytes are stored.
 * @param[in,out] accel_length : Number of accelerometer frames.
 * @param[in,out] fifo         : Structure instance of bmi2_fifo_frame.
 * @param[in]     dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_extract_accel(struct bmi2_sens_axes_data *accel_data,
                          uint16_t *accel_length,
                          struct bmi2_fifo_frame *fifo,
                          const struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_extract_aux bmi2_extract_aux
 * \code
 * int8_t bmi2_extract_aux(struct bmi2_aux_fifo_data *aux,
 *                       uint16_t *aux_length,
 *                       struct bmi2_fifo_frame *fifo,
 *                       const struct bmi2_dev *dev);
 *
 * \endcode
 * @details This API parses and extracts the auxiliary frames from FIFO data
 * read by the "bmi2_read_fifo_data" API and stores it in "aux_data" buffer.
 *
 * @param[out]    aux          : Pointer to structure where the parsed auxiliary
 *                              data bytes are stored.
 * @param[in,out] aux_length   : Number of auxiliary frames.
 * @param[in,out] fifo         : Structure instance of bmi2_fifo_frame.
 * @param[in]     dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_extract_aux(struct bmi2_aux_fifo_data *aux,
                        uint16_t *aux_length,
                        struct bmi2_fifo_frame *fifo,
                        const struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_extract_gyro bmi2_extract_gyro
 * \code
 * int8_t bmi2_extract_gyro(struct bmi2_sens_axes_data *gyro_data,
 *                        uint16_t *gyro_length,
 *                        struct bmi2_fifo_frame *fifo,
 *                        const struct bmi2_dev *dev);
 * \endcode
 * @details This API parses and extracts the gyroscope frames from FIFO data read by the
 * "bmi2_read_fifo_data" API and stores it in the "gyro_data"
 * structure instance.
 *
 * @param[out]    gyro_data    : Structure instance of bmi2_sens_axes_data
 *                               where the parsed data bytes are stored.
 * @param[in,out] gyro_length  : Number of gyroscope frames.
 * @param[in,out] fifo         : Structure instance of bmi2_fifo_frame.
 * @param[in]     dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_extract_gyro(struct bmi2_sens_axes_data *gyro_data,
                         uint16_t *gyro_length,
                         struct bmi2_fifo_frame *fifo,
                         const struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiCmd Command Register
 * @brief Write commands to the sensor
 */

/*!
 * \ingroup bmi2ApiCmd
 * \page bmi2_api_bmi2_set_command_register bmi2_set_command_register
 * \code
 * int8_t bmi2_set_command_register(uint8_t command, struct bmi2_dev *dev);
 * \endcode
 * @details This API writes the available sensor specific commands to the sensor.
 *
 * @param[in] command     : Commands to be given to the sensor.
 * @param[in] dev         : Structure instance of bmi2_dev.
 *
 *@verbatim
 * Commands             |  Values
 * ---------------------|---------------------
 * BMI2_SOFT_RESET_CMD  |  0xB6
 * BMI2_FIFO_FLUSH_CMD  |  0xB0
 * BMI2_USR_GAIN_CMD    |  0x03
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_command_register(uint8_t command, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_set_fifo_self_wake_up bmi2_set_fifo_self_wake_up
 * \code
 * int8_t bmi2_set_fifo_self_wake_up(uint8_t fifo_self_wake_up, struct bmi2_dev *dev);
 * \endcode
 * @details This API sets the FIFO self wake up functionality in the sensor.
 *
 * @param[in] fifo_self_wake_up : Variable to set FIFO self wake-up.
 * @param[in] dev               : Structure instance of bmi2_dev.
 *
 *@verbatim
 *  fifo_self_wake_up |  Description
 * -------------------|---------------
 * BMI2_DISABLE       | Disables self wake-up.
 * BMI2_ENABLE        | Enables self wake-up.
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_fifo_self_wake_up(uint8_t fifo_self_wake_up, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_get_fifo_self_wake_up bmi2_get_fifo_self_wake_up
 * \code
 * int8_t bmi2_get_fifo_self_wake_up(uint8_t *fifo_self_wake_up, const struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the FIFO self wake up functionality from the sensor.
 *
 * @param[out] fifo_self_wake_up : Pointer variable to get the status of FIFO
 *                                 self wake-up.
 * @param[in]  dev               : Structure instance of bmi2_dev.
 *
 *@verbatim
 *  fifo_self_wake_up |  Description
 * -------------------|---------------
 * BMI2_DISABLE       | Self wake-up disabled
 * BMI2_ENABLE        | Self wake-up enabled.
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_fifo_self_wake_up(uint8_t *fifo_self_wake_up, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_set_fifo_wm bmi2_set_fifo_wm
 * \code
 * int8_t bmi2_set_fifo_wm(uint16_t fifo_wm, struct bmi2_dev *dev);
 * \endcode
 * @details This API sets the FIFO water mark level which is set in the sensor.
 *
 * @param[in] fifo_wm          : Variable to set FIFO water-mark level.
 * @param[in] dev              : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_fifo_wm(uint16_t fifo_wm, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_get_fifo_wm bmi2_get_fifo_wm
 * \code
 * int8_t bmi2_get_fifo_wm(uint16_t *fifo_wm, const struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the FIFO water mark level which is set in the sensor.
 *
 * @param[out] fifo_wm        : Pointer variable to store FIFO water-mark level.
 * @param[in]  dev            : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_fifo_wm(uint16_t *fifo_wm, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_set_fifo_filter_data bmi2_set_fifo_filter_data
 * \code
 * int8_t bmi2_set_fifo_filter_data(uint8_t sens_sel, uint8_t fifo_filter_data, struct bmi2_dev *dev);
 * \endcode
 * @details This API sets either filtered or un-filtered FIFO accelerometer or
 * gyroscope data.
 *
 * @param[in] sens_sel          : Selects either accelerometer or
 *                                gyroscope sensor.
 * @param[in] fifo_filter_data  : Variable to set the filter data.
 * @param[in] dev               : Structure instance of bmi2_dev.
 *
 *@verbatim
 *  sens_sel        |  values
 * -----------------|----------
 * BMI2_ACCEL       |  0x01
 * BMI2_GYRO        |  0x02
 *@endverbatim
 *
 *@verbatim
 * Value    |  fifo_filter_data
 * ---------|---------------------
 * 0x00     |  Un-filtered data
 * 0x01     |  Filtered data
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_fifo_filter_data(uint8_t sens_sel, uint8_t fifo_filter_data, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_get_fifo_filter_data bmi2_get_fifo_filter_data
 * \code
 * int8_t bmi2_get_fifo_filter_data(uint8_t sens_sel, uint8_t *fifo_filter_data, const struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the FIFO accelerometer or gyroscope filter data.
 *
 * @param[in] sens_sel           : Selects either accelerometer or
 *                                 gyroscope sensor.
 * @param[out] fifo_filter_data  : Pointer variable to get the filter data.
 * @param[in]  dev               : Structure instance of bmi2_dev.
 *
 *@verbatim
 *  sens_sel        |  values
 * -----------------|----------
 * BMI2_ACCEL       |  0x01
 * BMI2_GYRO        |  0x02
 *@endverbatim
 *
 *@verbatim
 * Value    |  fifo_filter_data
 * ---------|---------------------
 * 0x00     |  Un-filtered data
 * 0x01     |  Filtered data
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_fifo_filter_data(uint8_t sens_sel, uint8_t *fifo_filter_data, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_set_fifo_down_sample bmi2_set_fifo_down_sample
 * \code
 * int8_t bmi2_set_fifo_down_sample(uint8_t sens_sel, uint8_t fifo_down_samp, struct bmi2_dev *dev);
 * \endcode
 * @details This API sets the down sampling rate for FIFO accelerometer or
 * gyroscope FIFO data.
 *
 * @param[in] sens_sel       : Selects either either accelerometer or
 *                             gyroscope sensor.
 * @param[in] fifo_down_samp : Variable to set the down sampling rate.
 * @param[in] dev            : Structure instance of bmi2_dev.
 *
 *@verbatim
 *     sens_sel    |  values
 * ----------------|----------
 * BMI2_ACCEL      |  0x01
 * BMI2_GYRO       |  0x02
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_fifo_down_sample(uint8_t sens_sel, uint8_t fifo_down_samp, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_get_fifo_down_sample bmi2_get_fifo_down_sample
 * \code
 * int8_t bmi2_get_fifo_down_sample(uint8_t sens_sel, uint8_t *fifo_down_samp, const struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the down sampling rate, configured for FIFO
 * accelerometer or gyroscope data.
 *
 * @param[in] sens_sel        : Selects either either accelerometer or
 *                              gyroscope sensor.
 * @param[out] fifo_down_samp : Pointer variable to store the down sampling rate
 * @param[in]  dev            : Structure instance of bmi2_dev.
 *
 *@verbatim
 *  sens_sel       |  values
 * ----------------|----------
 * BMI2_ACCEL      |  0x01
 * BMI2_GYRO       |  0x02
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_fifo_down_sample(uint8_t sens_sel, uint8_t *fifo_down_samp, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFIFO
 * \page bmi2_api_bmi2_get_fifo_length bmi2_get_fifo_length
 * \code
 * int8_t bmi2_get_fifo_length(uint16_t *fifo_length, const struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the length of FIFO data available in the sensor in
 * bytes.
 *
 * @param[out] fifo_length  : Pointer variable to store the value of FIFO byte
 *                            counter.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @note The byte counter is updated each time a complete frame is read or
 * written.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_fifo_length(uint16_t *fifo_length, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiOIS OIS
 * @brief OIS operations of the sensor
 */

/*!
 * \ingroup bmi2ApiOIS
 * \page bmi2_api_bmi2_set_ois_interface bmi2_set_ois_interface
 * \code
 * int8_t bmi2_set_ois_interface(uint8_t enable, struct bmi2_dev *dev);
 * \endcode
 * @details This API enables/disables OIS interface.
 *
 * @param[in] enable   : To enable/disable OIS interface.
 * @param[in] dev      : Structure instance of bmi2_dev.
 *
 *@verbatim
 *  Enable      |  Description
 * -------------|---------------
 * BMI2_DISABLE | Disables OIS interface.
 * BMI2_ENABLE  | Enables OIS interface.
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_ois_interface(uint8_t enable, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiAux Auxiliary sensor
 * @brief Auxiliary sensor operations of the sensor
 */

/*!
 * \ingroup bmi2ApiAux
 * \page bmi2_api_bmi2_read_aux_man_mode bmi2_read_aux_man_mode
 * \code
 * int8_t bmi2_read_aux_man_mode(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, struct bmi2_dev *dev);
 * \endcode
 * @details This API reads the user-defined bytes of data from the given register
 * address of auxiliary sensor in manual mode.
 *
 * @param[in]  reg_addr     : Address from where data is read.
 * @param[out] aux_data     : Pointer to the stored buffer.
 * @param[in]  len          : Total length of data to be read.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @note Change of BMI2_AUX_RD_ADDR is only allowed if AUX is not busy.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_read_aux_man_mode(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiAux
 * \page bmi2_api_bmi2_write_aux_man_mode bmi2_write_aux_man_mode
 * \code
 * int8_t bmi2_write_aux_man_mode(uint8_t reg_addr, const uint8_t *aux_data, uint16_t len, struct bmi2_dev *dev);
 * \endcode
 * @details This API writes the user-defined bytes of data and the address of
 * auxiliary sensor where data is to be written in manual mode.
 *
 * @param[in]  reg_addr     : AUX address where data is to be written.
 * @param[in] aux_data      : Pointer to data to be written.
 * @param[in]  len          : Total length of data to be written.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @note Change of BMI2_AUX_WR_ADDR is only allowed if AUX is not busy.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_write_aux_man_mode(uint8_t reg_addr, const uint8_t *aux_data, uint16_t len, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiAux
 * \page bmi2_api_bmi2_write_aux_interleaved bmi2_write_aux_interleaved
 * \code
 * int8_t bmi2_write_aux_interleaved(uint8_t reg_addr, const uint8_t *aux_data, uint16_t len, struct bmi2_dev *dev);
 * \endcode
 * @details This API writes the user-defined bytes of data and the address of
 * auxiliary sensor where data is to be written, from an interleaved input,
 * in manual mode.
 *
 * @param[in]  reg_addr     : AUX address where data is to be written.
 * @param[in] aux_data      : Pointer to data to be written.
 * @param[in]  len          : Total length of data to be written.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @note Change of BMI2_AUX_WR_ADDR is only allowed if AUX is not busy.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_write_aux_interleaved(uint8_t reg_addr, const uint8_t *aux_data, uint16_t len, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiStatus Sensor Status
 * @brief Get sensor status
 */

/*!
 * \ingroup bmi2ApiStatus
 * \page bmi2_api_bmi2_get_status bmi2_get_status
 * \code
 * int8_t bmi2_get_status(uint8_t *status, const struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the data ready status of accelerometer, gyroscope,
 * auxiliary, ready status of command decoder and busy status of auxiliary.
 *
 * @param[out] status     : Pointer variable to the status.
 * @param[in]  dev        : Structure instance of bmi2_dev.
 *
 *@verbatim
 * Value    |  Status
 * ---------|---------------------
 * 0x80     |  DRDY_ACC
 * 0x40     |  DRDY_GYR
 * 0x20     |  DRDY_AUX
 * 0x10     |  CMD_RDY
 * 0x04     |  AUX_BUSY
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_status(uint8_t *status, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiWSync Sync commands
 * @brief Write sync commands
 */

/*!
 * \ingroup bmi2ApiWSync
 * \page bmi2_api_bmi2_write_sync_commands bmi2_write_sync_commands
 * \code
 * int8_t bmi2_write_sync_commands(const uint8_t *command, uint8_t n_comm, struct bmi2_dev *dev);
 * \endcode
 * @details This API can be used to write sync commands like ODR, sync period,
 * frequency and phase, resolution ratio, sync time and delay time.
 *
 * @param[in] command     : Sync command to be written.
 * @param[in]  n_comm     : Length of the command.
 * @param[in]  dev        : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_write_sync_commands(const uint8_t *command, uint8_t n_comm, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiASelftest Accel self test
 * @brief Perform accel self test
 */

/*!
 * \ingroup bmi2ApiASelftest
 * \page bmi2_api_bmi2_perform_accel_self_test bmi2_perform_accel_self_test
 * \code
 * int8_t bmi2_perform_accel_self_test(struct bmi2_dev *dev);
 * \endcode
 * @details This API performs self-test to check the proper functionality of the
 * accelerometer sensor.
 *
 * @param[in]  dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_perform_accel_self_test(struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiInt
 * \page bmi2_api_bmi2_map_feat_int bmi2_map_feat_int
 * \code
 * int8_t bmi2_map_feat_int(const struct bmi2_sens_int_config *sens_int, struct bmi2_dev *dev);
 * \endcode
 * @details This API maps/unmaps feature interrupts to that of interrupt pins.
 *
 * @param[in] sens_int     : Structure instance of bmi2_sens_int_config.
 * @param[in] dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_map_feat_int(uint8_t type, enum bmi2_hw_int_pin hw_int_pin, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiInt
 * \page bmi2_api_bmi2_map_data_int bmi2_map_data_int
 * \code
 * int8_t bmi2_map_data_int(uint8_t data_int, enum bmi2_hw_int_pin int_pin, struct bmi2_dev *dev);
 * \endcode
 * @details This API maps/un-maps data interrupts to that of interrupt pins.
 *
 * @param[in] int_pin      : Interrupt pin selected.
 * @param[in] data_int     : Type of data interrupt to be mapped.
 * @param[in] dev          : Structure instance of bmi2_dev.
 *
 *@verbatim
 * data_int             |  Mask values
 * ---------------------|---------------------
 * BMI2_FFULL_INT       |  0x01
 * BMI2_FWM_INT         |  0x02
 * BMI2_DRDY_INT        |  0x04
 * BMI2_ERR_INT         |  0x08
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_map_data_int(uint8_t data_int, enum bmi2_hw_int_pin int_pin, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiRemap Remap Axes
 * @brief Set / Get remap axes values from the sensor
 */

/*!
 * \ingroup bmi2ApiRemap
 * \page bmi2_api_bmi2_get_remap_axes bmi2_get_remap_axes
 * \code
 * int8_t bmi2_get_remap_axes(struct bmi2_remap *remapped_axis, struct bmi2_dev *dev);
 * \endcode
 * @details This API gets the re-mapped x, y and z axes from the sensor and
 * updates the values in the device structure.
 *
 * @param[out] remapped_axis : Structure that stores re-mapped axes.
 * @param[in] dev            : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_remap_axes(struct bmi2_remap *remapped_axis, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiRemap
 * \page bmi2_api_bmi2_set_remap_axes bmi2_set_remap_axes
 * \code
 * int8_t bmi2_set_remap_axes(const struct bmi2_remap *remapped_axis, struct bmi2_dev *dev);
 * \endcode
 * @details This API sets the re-mapped x, y and z axes to the sensor and
 * updates them in the device structure.
 *
 * @param[in] remapped_axis  : Structure that stores re-mapped axes.
 * @param[in] dev            : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_remap_axes(const struct bmi2_remap *remapped_axis, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiGyroOC Gyro Offset Compensation
 * @brief Gyro Offset Compensation operations of the sensor
 */

/*!
 * \ingroup bmi2ApiGyroOC
 * \page bmi2_api_bmi2_set_gyro_offset_comp bmi2_set_gyro_offset_comp
 * \code
 * int8_t bmi2_set_gyro_offset_comp(uint8_t enable, struct bmi2_dev *dev);
 * \endcode
 * @details This API enables/disables gyroscope offset compensation. It adds the
 * offsets defined in the offset register with gyroscope data.
 *
 * @param[in] enable          : Enables/Disables gyroscope offset compensation.
 * @param[in]  dev            : Structure instance of bmi2_dev.
 *
 *@verbatim
 *  enable      |  Description
 * -------------|---------------
 * BMI2_ENABLE  | Enables gyroscope offset compensation.
 * BMI2_DISABLE | Disables gyroscope offset compensation.
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_gyro_offset_comp(uint8_t enable, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiGyroOC
 * \page bmi2_api_bmi2_read_gyro_offset_comp_axes bmi2_read_gyro_offset_comp_axes
 * \code
 * int8_t bmi2_read_gyro_offset_comp_axes(struct bmi2_sens_axes_data *gyr_off_comp_axes, const struct bmi2_dev *dev);
 * \endcode
 * @details This API reads the gyroscope bias values for each axis which is used
 * for gyroscope offset compensation.
 *
 * @param[out] gyr_off_comp_axes: Structure to store gyroscope offset
 *                                compensated values.
 * @param[in]  dev              : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_read_gyro_offset_comp_axes(struct bmi2_sens_axes_data *gyr_off_comp_axes, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiGyroOC
 * \page bmi2_api_bmi2_write_gyro_offset_comp_axes bmi2_write_gyro_offset_comp_axes
 * \code
 * int8_t bmi2_write_gyro_offset_comp_axes(const struct bmi2_sens_axes_data *gyr_off_comp_axes, struct bmi2_dev *dev);
 * \endcode
 * @details This API writes the gyroscope bias values for each axis which is used
 * for gyroscope offset compensation.
 *
 * @param[in] gyr_off_comp_axes : Structure to store gyroscope offset
 *                              compensated values.
 * @param[in]  dev              : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_write_gyro_offset_comp_axes(const struct bmi2_sens_axes_data *gyr_off_comp_axes, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiGyroCS Gyro cross sensitivity
 * @brief Gyro Cross sensitivity operation
 */

/*!
 * \ingroup bmi2ApiGyroCS
 * \page bmi2_api_bmi2_get_gyro_cross_sense bmi2_get_gyro_cross_sense
 * \code
 * int8_t bmi2_get_gyro_cross_sense(struct bmi2_dev *dev);
 * \endcode
 * @details This API updates the cross sensitivity coefficient between gyroscope's
 * X and Z axes.
 *
 * @param[in, out]  dev     : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_gyro_cross_sense(struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiInts Internal Status
 * @brief Get Internal Status of the sensor
 */

/*!
 * \ingroup bmi2ApiInts
 * \page bmi2_api_bmi2_get_internal_status bmi2_get_internal_status
 * \code
 * int8_t bmi2_get_internal_status(uint8_t *int_stat, const struct bmi2_dev *dev);
 * \endcode
 * @details This API gets error bits and message indicating internal status.
 *
 * @param[in]  dev          : Structure instance of bmi2_dev.
 * @param[out] int_stat     : Pointer variable to store error bits and
 *                          message.
 *
 *@verbatim
 * Internal status      |  *int_stat
 * ---------------------|---------------------
 * BMI2_NOT_INIT        |  0x00
 * BMI2_INIT_OK         |  0x01
 * BMI2_INIT_ERR        |  0x02
 * BMI2_DRV_ERR         |  0x03
 * BMI2_SNS_STOP        |  0x04
 * BMI2_NVM_ERROR       |  0x05
 * BMI2_START_UP_ERROR  |  0x06
 * BMI2_COMPAT_ERROR    |  0x07
 * BMI2_VFM_SKIPPED     |  0x10
 * BMI2_AXES_MAP_ERROR  |  0x20
 * BMI2_ODR_50_HZ_ERROR |  0x40
 * BMI2_ODR_HIGH_ERROR  |  0x80
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_internal_status(uint8_t *int_stat, struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiFOC FOC
 * @brief FOC operations of the sensor
 */

/*!
 * \ingroup bmi2ApiFOC
 * \page bmi2_api_bmi2_perform_accel_foc bmi2_perform_accel_foc
 * \code
 * int8_t bmi2_perform_accel_foc(const struct bmi2_accel_foc_g_value *accel_g_value, struct bmi2_dev *dev);
 * \endcode
 * @details This API performs Fast Offset Compensation for accelerometer.
 *
 * @param[in] accel_g_value : This parameter selects the accel foc
 * axis to be performed
 *
 * input format is {x, y, z, sign}. '1' to enable. '0' to disable
 *
 * eg to choose x axis  {1, 0, 0, 0}
 * eg to choose -x axis {1, 0, 0, 1}
 *
 *
 * @param[in]  dev              : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_perform_accel_foc(const struct bmi2_accel_foc_g_value *accel_g_value, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiFOC
 * \page bmi2_api_bmi2_perform_gyro_foc bmi2_perform_gyro_foc
 * \code
 * int8_t bmi2_perform_gyro_foc(struct bmi2_dev *dev);
 * \endcode
 * @details This API performs Fast Offset Compensation for gyroscope.
 *
 * @param[in]  dev              : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_perform_gyro_foc(struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiCRT CRT
 * @brief CRT operations of the sensor
 */

/*!
 * \ingroup bmi2ApiCRT
 * \page bmi2_api_bmi2_do_crt bmi2_do_crt
 * \code
 * int8_t bmi2_do_crt(struct bmi2_dev *dev);
 * \endcode
 * @details API performs Component Re-Trim calibration (CRT).
 *
 *
 * @param[in]  dev  : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 * @note CRT calibration takes approximately 500ms & maximum time out configured as 2 seconds
 */
int8_t bmi2_do_crt(struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiCRTSt CRT and self test
 * @brief Enable / Abort CRT and self test operations of gyroscope
 */

/*!
 * \ingroup bmi2ApiCRTSt
 * \page bmi2_api_bmi2_set_gyro_self_test_crt bmi2_set_gyro_self_test_crt
 * \code
 * int8_t bmi2_set_gyro_self_test_crt
 * \endcode
 * @details This api is used to enable the gyro self-test and crt.
 *   *gyro_self_test_crt -> 0  then gyro self test enable
 *   *gyro_self_test_crt -> 1 then CRT enable
 *
 * @param[in] gyro_self_test_crt  : enable the gyro self test or crt.
 * @param[in] dev                 : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_set_gyro_self_test_crt(uint8_t *gyro_self_test_crt, struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiCRTSt
 * \page bmi2_api_bmi2_abort_crt_gyro_st bmi2_abort_crt_gyro_st
 * \code
 * int8_t bmi2_abort_crt_gyro_st(struct bmi2_dev *dev);
 * \endcode
 * @details This api is used to abort ongoing crt or gyro self test.
 *
 * @param[in] dev        : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_abort_crt_gyro_st(struct bmi2_dev *dev);

/*!
 * \ingroup bmi2ApiASelftest
 * \page bmi2_api_bmi2_do_gyro_st bmi2_do_gyro_st
 * \code
 * int8_t bmi2_do_gyro_st
 * \endcode
 * @details this api is used to perform gyroscope self test.
 *
 * @param[in] dev   :   Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_do_gyro_st(struct bmi2_dev *dev);

/**
 * \ingroup bmi2
 * \defgroup bmi2ApiNVM NVM
 * @brief NVM operations of the sensor
 */

/*!
 * \ingroup bmi2ApiNVM
 * \page bmi2_api_bmi2_nvm_prog bmi2_nvm_prog
 * \code
 * int8_t bmi2_nvm_prog
 * \endcode
 * @details This api is used for programming the non volatile memory(nvm)
 *
 * @param[in] dev   :   Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_nvm_prog(struct bmi2_dev *dev);

/*!
 * @brief This API extracts the input feature configuration
 * details like page and start address from the look-up table.
 *
 * @param[out] feat_config  : Structure that stores feature configurations.
 * @param[in] type          : Type of feature or sensor.
 * @param[in] dev           : Structure instance of bmi2_dev.
 *
 * @return Returns the feature found flag.
 *
 * @retval  BMI2_FALSE : Feature not found
 *          BMI2_TRUE  : Feature found
 */
uint8_t bmi2_extract_input_feat_config(struct bmi2_feature_config *feat_config, uint8_t type,
                                       const struct bmi2_dev *dev);

/*!
 * @brief This API is used to get the feature configuration from the
 * selected page.
 *
 * @param[in]  sw_page       : Switches to the desired page.
 * @param[out] feat_config   : Pointer to the feature configuration.
 * @param[in]  dev           : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_feat_config(uint8_t sw_page, uint8_t *feat_config, struct bmi2_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* BMI2_H_ */
