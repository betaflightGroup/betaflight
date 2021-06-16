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
* @file       bmi2.c
* @date       2020-11-04
* @version    v2.63.1
*
*/

/******************************************************************************/

/*!  @name          Header Files                                  */
/******************************************************************************/
#include "bmi2.h"

/***************************************************************************/

/*!         Local structures
 ****************************************************************************/

/*! @name Structure to define the difference in accelerometer values  */
struct bmi2_selftest_delta_limit
{
    /*! X  data */
    int32_t x;

    /*! Y  data */
    int32_t y;

    /*! Z  data */
    int32_t z;
};

/*! @name Structure to store temporary accelerometer/gyroscope values */
struct bmi2_foc_temp_value
{
    /*! X data */
    int32_t x;

    /*! Y data */
    int32_t y;

    /*! Z data */
    int32_t z;
};

/*! @name Structure to store accelerometer data deviation from ideal value */
struct bmi2_offset_delta
{
    /*! X axis */
    int16_t x;

    /*! Y axis */
    int16_t y;

    /*! Z axis */
    int16_t z;
};

/*! @name Structure to store accelerometer offset values */
struct bmi2_accel_offset
{
    /*! offset X  data */
    uint8_t x;

    /*! offset Y  data */
    uint8_t y;

    /*! offset Z  data */
    uint8_t z;
};

/******************************************************************************/

/*!         Local Function Prototypes
 ******************************************************************************/

/*!
 * @brief This internal API writes the configuration file.
 *
 * @param[in]  dev     : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t write_config_file(struct bmi2_dev *dev);

/*!
 * @brief This internal API enables/disables the loading of the configuration
 * file.
 *
 * @param[in] enable   : To enable/disable configuration load.
 * @param[in] dev      : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_config_load(uint8_t enable, struct bmi2_dev *dev);

/*!
 * @brief This internal API loads the configuration file.
 *
 * @param[in] config_data   : Pointer to the configuration file.
 * @param[in] index         : Variable to define array index.
 * @param[in] dev           : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t upload_file(const uint8_t *config_data, uint16_t index, uint16_t write_len, struct bmi2_dev *dev);

/*!
 * @brief  This internal API sets accelerometer configurations like ODR,
 * bandwidth, performance mode and g-range.
 *
 * @param[in,out] config    : Structure instance of bmi2_accel_config.
 * @param[in,out] dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_accel_config(struct bmi2_accel_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API validates bandwidth and performance mode of the
 * accelerometer set by the user.
 *
 * @param[in, out] bandwidth : Pointer to bandwidth value set by the user.
 * @param[in, out] perf_mode : Pointer to performance mode set by the user.
 * @param[in, out] dev       : Structure instance of bmi2_dev.
 *
 * @note dev->info contains two warnings: BMI2_I_MIN_VALUE and BMI2_I_MAX_VALUE
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t validate_bw_perf_mode(uint8_t *bandwidth, uint8_t *perf_mode, struct bmi2_dev *dev);

/*!
 * @brief This internal API validates ODR and range of the accelerometer set by
 * the user.
 *
 * @param[in, out] odr    : Pointer to ODR value set by the user.
 * @param[in, out] range  : Pointer to range value set by the user.
 * @param[in, out] dev    : Structure instance of bmi2_dev.
 *
 * @note dev->info contains two warnings: BMI2_I_MIN_VALUE and BMI2_I_MAX_VALUE
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t validate_odr_range(uint8_t *odr, uint8_t *range, struct bmi2_dev *dev);

/*!
 * @brief  This internal API sets gyroscope configurations like ODR, bandwidth,
 * low power/high performance mode, performance mode and range.
 *
 * @param[in,out] config    : Structure instance of bmi2_gyro_config.
 * @param[in,out] dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_gyro_config(struct bmi2_gyro_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API validates bandwidth, performance mode, low power/
 * high performance mode, ODR, and range set by the user.
 *
 * @param[in] config   : Structure instance of bmi2_gyro_config.
 * @param[in] dev      : Structure instance of bmi2_dev.
 *
 * @note dev->info contains two warnings: BMI2_I_MIN_VALUE and BMI2_I_MAX_VALUE
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t validate_gyro_config(struct bmi2_gyro_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API shows the error status when illegal sensor
 * configuration is set.
 *
 * @param[in]  dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t cfg_error_status(struct bmi2_dev *dev);

/*!
 * @brief This internal API:
 * 1) Enables/Disables auxiliary interface.
 * 2) Sets auxiliary interface configurations like I2C address, manual/auto
 * mode enable, manual burst read length, AUX burst read length and AUX read
 * address.
 * 3)It maps/un-maps data interrupts to that of hardware interrupt line.
 *
 * @param[in] config            : Structure instance of bmi2_aux_config.
 * @param[in, out] dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_aux_config(struct bmi2_aux_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API sets gyroscope user-gain configurations like gain
 * update value for x, y and z-axis.
 *
 * @param[in] config      : Structure instance of bmi2_gyro_user_gain_config.
 * @param[in] dev         : Structure instance of bmi2_dev.
 *
 * @verbatim
 *----------------------------------------------------------------------------
 * bmi2_gyro_user_gain_config|
 *  Structure parameters    |                   Description
 *--------------------------|--------------------------------------------------
 *      ratio_x             | Gain update value for x-axis
 * -------------------------|---------------------------------------------------
 *      ratio_y             | Gain update value for y-axis
 * -------------------------|---------------------------------------------------
 *      ratio_z             | Gain update value for z-axis
 * @endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_gyro_user_gain_config(const struct bmi2_gyro_user_gain_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API enables/disables auxiliary interface.
 *
 * @param[in] config    : Structure instance of bmi2_aux_config.
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_aux_interface(const struct bmi2_aux_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API sets auxiliary configurations like manual/auto mode
 * FCU write command enable and read burst length for both data and manual mode.
 *
 * @param[in] config    : Structure instance of bmi2_aux_config.
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * @note Auxiliary sensor should not be busy when configuring aux_i2c_addr,
 * man_rd_burst_len, aux_rd_burst_len and aux_rd_addr.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t config_aux_interface(const struct bmi2_aux_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API triggers read out offset and sets ODR of the
 * auxiliary sensor.
 *
 * @param[in] config    : Structure instance of bmi2_aux_config.
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t config_aux(const struct bmi2_aux_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API validates auxiliary configuration set by the user.
 *
 * @param[in, out] config : Structure instance of bmi2_aux_config.
 * @param[in, out] dev    : Structure instance of bmi2_dev.
 *
 * @note dev->info contains two warnings: BMI2_I_MIN_VALUE and BMI2_I_MAX_VALUE
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t validate_aux_config(struct bmi2_aux_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API gets accelerometer configurations like ODR,
 * bandwidth, performance mode and g-range.
 *
 * @param[out] config    : Structure instance of bmi2_accel_config.
 * @param[in]  dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_accel_config(struct bmi2_accel_config *config, struct bmi2_dev *dev);

/*!
 * @brief  This internal API gets gyroscope configurations like ODR, bandwidth,
 * low power/ high performance mode, performance mode and range.
 *
 * @param[out] config    : Structure instance of bmi2_gyro_config.
 * @param[in]  dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_gyro_config(struct bmi2_gyro_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API:
 * 1) Gets the status of auxiliary interface enable.
 * 2) Gets auxiliary interface configurations like I2C address, manual/auto
 * mode enable, manual burst read length, AUX burst read length and AUX read
 * address.
 * 3) Gets ODR and offset.
 *
 * @param[out] config    : Structure instance of bmi2_aux_config.
 * @param[in]  dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_aux_config(struct bmi2_aux_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API gets gyroscope user-gain configurations like gain
 * update value for x, y and z-axis.
 *
 * @param[out] config      : Structure instance of bmi2_gyro_user_gain_config.
 * @param[in] dev          : Structure instance of bmi2_dev.
 *
 * @verbatim
 *----------------------------------------------------------------------------
 * bmi2_gyro_user_gain_config|
 *  Structure parameters   |                   Description
 *-------------------------|--------------------------------------------------
 *      ratio_x            | Gain update value for x-axis
 * ------------------------|---------------------------------------------------
 *      ratio_y            | Gain update value for y-axis
 * ------------------------|---------------------------------------------------
 *      ratio_z            | Gain update value for z-axis
 *
 * @endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_gyro_gain_update_config(struct bmi2_gyro_user_gain_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API gets the enable status of auxiliary interface.
 *
 * @param[out] config    : Structure instance of bmi2_aux_config.
 * @param[in]  dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_aux_interface(struct bmi2_aux_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API gets auxiliary configurations like manual/auto mode
 * FCU write command enable and read burst length for both data and manual mode.
 *
 * @param[out] config   : Structure instance of bmi2_aux_config.
 * @param[in]  dev      : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_aux_interface_config(struct bmi2_aux_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API gets read out offset and ODR of the auxiliary
 * sensor.
 *
 * @param[out] config    : Structure instance of bmi2_aux_config.
 * @param[in]  dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_aux_cfg(struct bmi2_aux_config *config, struct bmi2_dev *dev);

/*!
 * @brief This internal API gets the saturation status for the gyroscope user
 * gain update.
 *
 * @param[out] user_gain_stat   : Stores the saturation status of the axes.
 * @param[in]  dev              : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_gyro_gain_update_status(struct bmi2_gyr_user_gain_status *user_gain, struct bmi2_dev *dev);

/*!
 * @brief This internal API is used to extract the output feature configuration
 * details like page and start address from the look-up table.
 *
 * @param[out] feat_output      : Structure that stores output feature
 *                              configurations.
 * @param[in] type              : Type of feature or sensor.
 * @param[in] dev               : Structure instance of bmi2_dev.
 *
 * @return Returns the feature found flag.
 *
 * @retval  BMI2_FALSE : Feature not found
 *          BMI2_TRUE  : Feature found
 */
static uint8_t extract_output_feat_config(struct bmi2_feature_config *feat_output,
                                          uint8_t type,
                                          const struct bmi2_dev *dev);

/*!
 * @brief This internal API gets the cross sensitivity coefficient between
 * gyroscope's X and Z axes.
 *
 * @param[out] cross_sense  : Pointer to the stored cross sensitivity
 *              coefficient.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_gyro_cross_sense(int16_t *cross_sense, struct bmi2_dev *dev);

/*!
 * @brief This internal API gets the accelerometer data from the register.
 *
 * @param[out] data         : Structure instance of sensor_data.
 * @param[in]  reg_addr     : Register address where data is stored.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_accel_sensor_data(struct bmi2_sens_axes_data *data, uint8_t reg_addr, struct bmi2_dev *dev);

/*!
 * @brief This internal API gets the gyroscope data from the register.
 *
 * @param[out] data         : Structure instance of sensor_data.
 * @param[in]  reg_addr     : Register address where data is stored.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_gyro_sensor_data(struct bmi2_sens_axes_data *data, uint8_t reg_addr, struct bmi2_dev *dev);

/*!
 * @brief This internal API gets the accelerometer/gyroscope data.
 *
 * @param[out] data         : Structure instance of sensor_data.
 * @param[in]  reg_data     : Data stored in the register.
 *
 * @return None
 *
 * @retval None
 */
static void get_acc_gyr_data(struct bmi2_sens_axes_data *data, const uint8_t *reg_data);

/*!
 * @brief This internal API gets the re-mapped accelerometer/gyroscope data.
 *
 * @param[out] data         : Structure instance of sensor_data.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @return None
 *
 * @retval None
 */
static void get_remapped_data(struct bmi2_sens_axes_data *data, const struct bmi2_dev *dev);

/*!
 * @brief This internal API reads the user-defined bytes of data from the given
 * register address of auxiliary sensor in data mode.
 *
 * @param[out] aux_data     : Pointer to the stored auxiliary data.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t read_aux_data_mode(uint8_t *aux_data, struct bmi2_dev *dev);

/*!
 * @brief This internal API reads the user-defined bytes of data from the given
 * register address of auxiliary sensor in manual mode.
 *
 * @param[in]  reg_addr     : AUX address from where data is read.
 * @param[out] aux_data     : Pointer to the stored auxiliary data.
 * @param[in]   len         : Total bytes to be read.
 * @param[in]   burst_len   : Bytes of data to be read in bursts.
 * @param[in]   dev         : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t read_aux_data(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, uint8_t burst_len, struct bmi2_dev *dev);

/*!
 * @brief This internal API checks the busy status of auxiliary sensor and sets
 * the auxiliary register addresses when not busy.
 *
 * @param[in]  reg_addr     : Address in which AUX register address is
 *                              set.
 * @param[in]  reg_data     : Auxiliary register address to be set when AUX is
 *                              not busy.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_if_aux_not_busy(uint8_t reg_addr, uint8_t reg_data, struct bmi2_dev *dev);

/*!
 * @brief his internal API maps the actual burst read length with that of the
 * register value set by user.
 *
 * @param[out] len      : Actual burst length.
 * @param[in]  dev      : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t map_read_len(uint8_t *len, const struct bmi2_dev *dev);

/*!
 * @brief This internal API writes AUX write address and the user-defined bytes
 * of data to the AUX sensor in manual mode.
 *
 * @note Change of BMI2_AUX_WR_ADDR is only allowed if AUX is not busy.
 *
 * @param[in]  reg_addr   : AUX address in which data is to be written.
 * @param[in]  reg_data   : Data to be written
 * @param[in]  dev        : Structure instance of bmi2_dev.
 *
 * @note Change of BMI2_AUX_WR_ADDR is only allowed if AUX is not busy.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t write_aux_data(uint8_t reg_addr, uint8_t reg_data, struct bmi2_dev *dev);

/*!
 * @brief This internal API maps/unmaps feature interrupts to that of interrupt
 * pins.
 *
 * @param[in] int_pin      : Interrupt pin selected.
 * @param[in] feat_int     : Type of feature interrupt to be mapped.
 * @param[in] dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t map_feat_int(uint8_t *reg_data_array, enum bmi2_hw_int_pin int_pin, uint8_t int_mask);

/*!
 * @brief This internal API computes the number of bytes of accelerometer FIFO
 * data which is to be parsed in header-less mode.
 *
 * @param[out] start_idx   : The start index for parsing data.
 * @param[out] len         : Number of bytes to be parsed.
 * @param[in]  acc_count   : Number of accelerometer frames to be read.
 * @param[in]  fifo        : Structure instance of bmi2_fifo_frame.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t parse_fifo_accel_len(uint16_t *start_idx,
                                   uint16_t *len,
                                   const uint16_t *acc_count,
                                   const struct bmi2_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse accelerometer data from the FIFO
 * data in header mode.
 *
 * @param[out] acc          : Structure instance of bmi2_sens_axes_data where
 *                            the parsed accelerometer data bytes are stored.
 * @param[in] accel_length  : Number of accelerometer frames (x,y,z data).
 * @param[in] fifo          : Structure instance of bmi2_fifo_frame.
 * @param[in] dev           : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t extract_accel_header_mode(struct bmi2_sens_axes_data *acc,
                                        uint16_t *accel_length,
                                        struct bmi2_fifo_frame *fifo,
                                        const struct bmi2_dev *dev);

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO data in both header and header-less mode. It updates the current data
 * byte to be parsed.
 *
 * @param[in,out] acc       : Structure instance of bmi2_sens_axes_data where
 *                            where the parsed data bytes are stored.
 * @param[in,out] idx       : Index value of number of bytes parsed.
 * @param[in,out] acc_idx   : Index value of accelerometer data (x,y,z axes)
 *                            frame to be parsed.
 * @param[in]     frame     : Either data is enabled by user in header-less
 *                            mode or header frame value in header mode.
 * @param[in]     fifo      : Structure instance of bmi2_fifo_frame.
 * @param[in]     dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t unpack_accel_frame(struct bmi2_sens_axes_data *acc,
                                 uint16_t *idx,
                                 uint16_t *acc_idx,
                                 uint8_t frame,
                                 const struct bmi2_fifo_frame *fifo,
                                 const struct bmi2_dev *dev);

/*!
 * @brief This internal API is used to parse accelerometer data from the FIFO
 * data.
 *
 * @param[out] acc              : Structure instance of bmi2_sens_axes_data
 *                                where the parsed data bytes are stored.
 * @param[in]  data_start_index : Index value of the accelerometer data bytes
 *                                which is to be parsed from the FIFO data.
 * @param[in]  fifo             : Structure instance of bmi2_fifo_frame.
 * @param[in]  dev              : Structure instance of bmi2_dev.
 *
 * @return None
 * @retval None
 */
static void unpack_accel_data(struct bmi2_sens_axes_data *acc,
                              uint16_t data_start_index,
                              const struct bmi2_fifo_frame *fifo,
                              const struct bmi2_dev *dev);

/*!
 * @brief This internal API computes the number of bytes of gyroscope FIFO data
 * which is to be parsed in header-less mode.
 *
 * @param[out] start_idx   : The start index for parsing data.
 * @param[out] len         : Number of bytes to be parsed.
 * @param[in]  gyr_count   : Number of gyroscope frames to be read.
 * @param[in]  frame       : Either data enabled by user in header-less
 *                            mode or header frame value in header mode.
 * @param[in]  fifo        : Structure instance of bmi2_fifo_frame.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t parse_fifo_gyro_len(uint16_t *start_idx,
                                  uint16_t(*len),
                                  const uint16_t *gyr_count,
                                  const struct bmi2_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse the gyroscope data from the FIFO
 * data in both header and header-less mode It updates the current data byte to
 * be parsed.
 *
 * @param[in,out] gyr       : Structure instance of bmi2_sens_axes_data.
 * @param[in,out] idx       : Index value of number of bytes parsed
 * @param[in,out] gyr_idx   : Index value of gyroscope data (x,y,z axes)
 *                            frame to be parsed.
 * @param[in]     frame     : Either data is enabled by user in header-less
 *                            mode or header frame value in header mode.
 * @param[in]     fifo      : Structure instance of bmi2_fifo_frame.
 * @param[in]     dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t unpack_gyro_frame(struct bmi2_sens_axes_data *gyr,
                                uint16_t *idx,
                                uint16_t *gyr_idx,
                                uint8_t frame,
                                const struct bmi2_fifo_frame *fifo,
                                const struct bmi2_dev *dev);

/*!
 * @brief This internal API is used to parse gyroscope data from the FIFO data.
 *
 * @param[out] gyr             : Structure instance of bmi2_sens_axes_data where
 *                               the parsed gyroscope data bytes are stored.
 * @param[in] data_start_index : Index value of the gyroscope data bytes
 *                               which is to be parsed from the FIFO data.
 * @param[in] fifo             : Structure instance of bmi2_fifo_frame.
 * @param[in]  dev             : Structure instance of bmi2_dev.
 *
 * @return None
 * @retval None
 */
static void unpack_gyro_data(struct bmi2_sens_axes_data *gyr,
                             uint16_t data_start_index,
                             const struct bmi2_fifo_frame *fifo,
                             const struct bmi2_dev *dev);

/*!
 * @brief This internal API is used to parse the gyroscope data from the
 * FIFO data in header mode.
 *
 * @param[out] gyr            : Structure instance of bmi2_sens_axes_data where
 *                              the parsed gyroscope data bytes are stored.
 * @param[in] gyro_length     : Number of gyroscope frames (x,y,z data).
 * @param[in] fifo            : Structure instance of bmi2_fifo_frame.
 * @param[in]  dev            : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t extract_gyro_header_mode(struct bmi2_sens_axes_data *gyr,
                                       uint16_t *gyro_length,
                                       struct bmi2_fifo_frame *fifo,
                                       const struct bmi2_dev *dev);

/*!
 * @brief This API computes the number of bytes of auxiliary FIFO data
 * which is to be parsed in header-less mode.
 *
 * @param[out] start_idx   : The start index for parsing data.
 * @param[out] len         : Number of bytes to be parsed.
 * @param[in]  aux_count   : Number of accelerometer frames to be read.
 * @param[in]  fifo        : Structure instance of bmi2_fifo_frame.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t parse_fifo_aux_len(uint16_t *start_idx,
                                 uint16_t(*len),
                                 const uint16_t *aux_count,
                                 const struct bmi2_fifo_frame *fifo);

/*!
 * @brief This API is used to parse auxiliary data from the FIFO data.
 *
 * @param[out] aux         : Pointer to buffer where the parsed auxiliary data
 *                           bytes are stored.
 * @param[in] aux_length   : Number of auxiliary frames (x,y,z data).
 * @param[in] fifo         : Structure instance of bmi2_fifo_frame.
 * @param[in] dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t extract_aux_header_mode(struct bmi2_aux_fifo_data *aux,
                                      uint16_t *aux_length,
                                      struct bmi2_fifo_frame *fifo,
                                      const struct bmi2_dev *dev);

/*!
 * @brief This API is used to parse the auxiliary data from the FIFO data in
 * both header and header-less mode. It updates the current data byte to be
 * parsed.
 *
 * @param[out]    aux     : Pointer to structure where the parsed auxiliary data
 *                          bytes are stored.
 * @param[in,out] idx     : Index value of number of bytes parsed
 * @param[in,out] aux_idx : Index value of auxiliary data (x,y,z axes)
 *                          frame to be parsed
 * @param[in]     frame   : Either data is enabled by user in header-less
 *                          mode or header frame value in header mode.
 * @param[in]     fifo    : Structure instance of bmi2_fifo_frame.
 * @param[in]     dev     : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t unpack_aux_frame(struct bmi2_aux_fifo_data *aux,
                               uint16_t *idx,
                               uint16_t *aux_idx,
                               uint8_t frame,
                               const struct bmi2_fifo_frame *fifo,
                               const struct bmi2_dev *dev);

/*!
 * @brief This API is used to parse auxiliary data from the FIFO data.
 *
 * @param[out] aux              : Pointer to structure where the parsed
 *                              auxiliary data bytes are stored.
 * @param[in] data_start_index  : Index value of the auxiliary data bytes which
 *                              is to be parsed from the FIFO data.
 * @param[in] fifo              : Structure instance of bmi2_fifo_frame.
 *
 * @return None
 * @retval None
 */
static void unpack_aux_data(struct bmi2_aux_fifo_data *aux,
                            uint16_t data_start_index,
                            const struct bmi2_fifo_frame *fifo);

/*!
 * @brief This internal API is used to reset the FIFO related configurations
 * in the FIFO frame structure for the next FIFO read.
 *
 * @param[in, out] fifo     : Structure instance of bmi2_fifo_frame.
 * @param[in]      dev      : Structure instance of bmi2_dev.
 *
 * @return None
 * @retval None
 */
static void reset_fifo_frame_structure(struct bmi2_fifo_frame *fifo, const struct bmi2_dev *dev);

/*!
 * @brief This internal API checks whether the FIFO data read is an empty frame.
 * If empty frame, index is moved to the last byte.
 *
 * @param[in,out] data_index   : The index of the current data to be parsed from
 *                               FIFO data.
 * @param[in]     fifo         : Structure instance of bmi2_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval BMI2_OK - Success.
 * @retval BMI2_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval BMI2_W_PARTIAL_READ - Warning : There are more frames to be read
 */
static int8_t check_empty_fifo(uint16_t *data_index, const struct bmi2_fifo_frame *fifo);

/*!
 * @brief This internal API is used to move the data index ahead of the
 * current frame length parameter when unnecessary FIFO data appears while
 * extracting the user specified data.
 *
 * @param[in,out] data_index           : Index of the FIFO data which is to be
 *                                       moved ahead of the current frame length
 * @param[in]     current_frame_length : Number of bytes in the current frame.
 * @param[in]     fifo                 : Structure instance of bmi2_fifo_frame.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmi2_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse and store the sensor time from the
 * FIFO data.
 *
 * @param[in,out] data_index : Index of the FIFO data which has the sensor time.
 * @param[in]     fifo       : Structure instance of bmi2_fifo_frame.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t unpack_sensortime_frame(uint16_t *data_index, struct bmi2_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse and store the skipped frame count
 * from the FIFO data.
 *
 * @param[in,out] data_index : Index of the FIFO data which contains skipped
 *                             frame count.
 * @param[in] fifo           : Structure instance of bmi2_fifo_frame.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t unpack_skipped_frame(uint16_t *data_index, struct bmi2_fifo_frame *fifo);

/*!
 * @brief This internal API enables and configures the accelerometer which is
 * needed for self-test operation. It also sets the amplitude for the self-test.
 *
 * @param[in]  dev      : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t pre_self_test_config(struct bmi2_dev *dev);

/*!
 * @brief This internal API performs the steps needed for self-test operation
 * before reading the accelerometer self-test data.
 *
 * @param[in] sign      : Selects sign of self-test excitation
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * sign         |  Description
 * -------------|---------------
 * BMI2_ENABLE  | positive excitation
 * BMI2_DISABLE | negative excitation
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t self_test_config(uint8_t sign, struct bmi2_dev *dev);

/*!
 * @brief This internal API enables or disables the accelerometer self-test
 * feature in the sensor.
 *
 * @param[in] enable            : Enables/ Disables self-test.
 * @param[in] dev               : Structure instance of bmi2_dev.
 *
 * sign         |  Description
 * -------------|---------------
 * BMI2_ENABLE  | Enables self-test
 * BMI2_DISABLE | Disables self-test
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_accel_self_test_enable(uint8_t enable, struct bmi2_dev *dev);

/*!
 * @brief This internal API selects the sign for accelerometer self-test
 * excitation.
 *
 * @param[in] sign      : Selects sign of self-test excitation
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * sign         |  Description
 * -------------|---------------
 * BMI2_ENABLE  | positive excitation
 * BMI2_DISABLE | negative excitation
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_acc_self_test_sign(uint8_t sign, struct bmi2_dev *dev);

/*!
 * @brief This internal API sets the amplitude of the accelerometer self-test
 * deflection in the sensor.
 *
 * @param[in] amp       : Select amplitude of the self-test deflection.
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * amp          |  Description
 * -------------|---------------
 * BMI2_ENABLE  | self-test amplitude is high
 * BMI2_DISABLE | self-test amplitude is low
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_accel_self_test_amp(uint8_t amp, struct bmi2_dev *dev);

/*!
 * @brief This internal API reads the accelerometer data for x,y and z axis from
 * the sensor. The data units is in LSB format.
 *
 * @param[out] accel        : Buffer to store the acceleration value.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t read_accel_xyz(struct bmi2_sens_axes_data *accel, struct bmi2_dev *dev);

/*!
 * @brief This internal API converts LSB value of accelerometer axes to form
 * 'g' to 'mg' for self-test.
 *
 * @param[in] acc_data_diff    : Stores the acceleration value difference in g.
 * @param[out]acc_data_diff_mg : Stores the acceleration value difference in mg.
 * @param[in]  dev             : Structure instance of bmi2_dev.
 *
 * @return None
 * @retval None
 */
static void convert_lsb_g(const struct bmi2_selftest_delta_limit *acc_data_diff,
                          struct bmi2_selftest_delta_limit *acc_data_diff_mg,
                          const struct bmi2_dev *dev);

/*!
 * @brief This internal API is used to calculate the power of a value.
 *
 * @param[in] base          : base for power calculation.
 * @param[in] resolution    : exponent for power calculation.
 *
 * @return the calculated power
 * @retval the power value
 */
static int32_t power(int16_t base, uint8_t resolution);

/*!
 * @brief This internal API validates the accelerometer self-test data and
 * decides the result of self-test operation.
 *
 * @param[in] accel_data_diff   : Stores the acceleration value difference.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t validate_self_test(const struct bmi2_selftest_delta_limit *accel_data_diff);

/*!
 * @brief This internal API gets the re-mapped x, y and z axes from the sensor.
 *
 * @param[out] remap    : Structure that stores local copy of re-mapped axes.
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_remap_axes(struct bmi2_axes_remap *remap, struct bmi2_dev *dev);

/*!
 * @brief This internal API sets the re-mapped x, y and z axes in the sensor.
 *
 * @param[in] remap     : Structure that stores local copy of re-mapped axes.
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_remap_axes(const struct bmi2_axes_remap *remap, struct bmi2_dev *dev);

/*!
 * @brief Interface to get max burst length
 *
 * @param[in] max_burst_len : Pointer to store max burst length
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_maxburst_len(uint8_t *max_burst_len, struct bmi2_dev *dev);

/*!
 * @brief This internal API sets the max burst length.
 *
 * @param[in] write_len_byte    : read & write length
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_maxburst_len(const uint16_t write_len_byte, struct bmi2_dev *dev);

/*!
 * @brief This internal API parses virtual frame header from the FIFO data.
 *
 * @param[in, out] frame_header : FIFO frame header.
 * @param[in, out] data_index   : Index value of the FIFO data bytes
 *                                from which sensor frame header is to be parsed
 * @param[in]      fifo         : Structure instance of bmi2_fifo_frame.
 *
 * @return None
 * @retval None
 */
static void parse_if_virtual_header(uint8_t *frame_header, uint16_t *data_index, const struct bmi2_fifo_frame *fifo);

/*!
 * @brief This internal API gets sensor time from the accelerometer and
 * gyroscope virtual frames and updates in the data structure.
 *
 * @param[out] sens       : Sensor data structure
 * @param[in, out] idx    : Index of FIFO from where the data is to retrieved.
 * @param[in] fifo        : Structure instance of bmi2_fifo_frame.
 *
 * @return None
 * @retval None
 */
static void unpack_virt_sensor_time(struct bmi2_sens_axes_data *sens, uint16_t *idx,
                                    const struct bmi2_fifo_frame *fifo);

/*!
 * @brief This internal API gets sensor time from the auxiliary virtual
 * frames and updates in the data structure.
 *
 * @param[out] aux        : Auxiliary sensor data structure
 * @param[in, out] idx    : Index of FIFO from where the data is to retrieved.
 * @param[in] fifo        : Structure instance of bmi2_fifo_frame.
 *
 * @return None
 * @retval None
 */
static void unpack_virt_aux_sensor_time(struct bmi2_aux_fifo_data *aux,
                                        uint16_t *idx,
                                        const struct bmi2_fifo_frame *fifo);

/*!
 * @brief This internal API corrects the gyroscope cross-axis sensitivity
 * between the z and the x axis.
 *
 * @param[in]  dev              : Structure instance of bmi2_dev.
 * @param[out] gyr_data         : Structure instance of gyroscope data
 *
 * @return Result of API execution status
 *
 * @return None
 * @retval None
 */
static void comp_gyro_cross_axis_sensitivity(struct bmi2_sens_axes_data *gyr_data, const struct bmi2_dev *dev);

/*!
 * @brief This internal API saves the configurations before performing FOC.
 *
 * @param[out] acc_cfg      : Accelerometer configuration value
 * @param[out] aps          : Advance power mode value
 * @param[out] acc_en       : Accelerometer enable value
 * @param[in] dev           : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t save_accel_foc_config(struct bmi2_accel_config *acc_cfg,
                                    uint8_t *aps,
                                    uint8_t *acc_en,
                                    struct bmi2_dev *dev);

/*!
 * @brief This internal API performs Fast Offset Compensation for accelerometer.
 *
 * @param[in] accel_g_value : This parameter selects the accel foc
 * axis to be performed
 *
 * input format is {x, y, z, sign}. '1' to enable. '0' to disable
 *
 * eg to choose x axis  {1, 0, 0, 0}
 * eg to choose -x axis {1, 0, 0, 1}
 *
 * @param[in] acc_cfg       : Accelerometer configuration value
 * @param[in] dev           : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t perform_accel_foc(const struct bmi2_accel_foc_g_value *accel_g_value,
                                const struct bmi2_accel_config *acc_cfg,
                                struct bmi2_dev *dev);

/*!
 * @brief This internal sets configurations for performing accelerometer FOC.
 *
 * @param[in] dev       : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_accel_foc_config(struct bmi2_dev *dev);

/*!
 * @brief This internal API enables/disables the offset compensation for
 * filtered and un-filtered accelerometer data.
 *
 * @param[in] offset_en     : enables/disables offset compensation.
 * @param[in] dev           : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_accel_offset_comp(uint8_t offset_en, struct bmi2_dev *dev);

/*!
 * @brief This internal API converts the range value into accelerometer
 * corresponding integer value.
 *
 * @param[in] range_in      : Input range value.
 * @param[out] range_out    : Stores the integer value of range.
 *
 * @return None
 * @retval None
 */
static void map_accel_range(uint8_t range_in, uint8_t *range_out);

/*!
 * @brief This internal API compensate the accelerometer data against gravity.
 *
 * @param[in] lsb_per_g     : LSB value pre 1g.
 * @param[in] g_val         : G reference value of all axis.
 * @param[in] data          : Accelerometer data
 * @param[out] comp_data    : Stores the data that is compensated by taking the
 *                            difference in accelerometer data and lsb_per_g
 *                            value.
 *
 * @return None
 * @retval None
 */
static void comp_for_gravity(uint16_t lsb_per_g,
                             const struct bmi2_accel_foc_g_value *g_val,
                             const struct bmi2_sens_axes_data *data,
                             struct bmi2_offset_delta *comp_data);

/*!
 * @brief This internal API scales the compensated accelerometer data according
 * to the offset register resolution.
 *
 * @param[in] range         : G-range of the accelerometer.
 * @param[out] comp_data    : Data that is compensated by taking the
 *                            difference in accelerometer data and lsb_per_g
 *                            value.
 * @param[out] data         : Stores offset data
 *
 * @return None
 * @retval None
 */
static void scale_accel_offset(uint8_t range, const struct bmi2_offset_delta *comp_data,
                               struct bmi2_accel_offset *data);

/*!
 * @brief This internal API finds the bit position of 3.9mg according to given
 * range and resolution.
 *
 * @param[in] range     : G-range of the accelerometer.
 *
 * @return Result of API execution status
 * @retval Bit position of 3.9mg
 */
static int8_t get_bit_pos_3_9mg(uint8_t range);

/*!
 * @brief This internal API inverts the accelerometer offset data.
 *
 * @param[out] offset_data  : Stores the inverted offset data
 *
 * @return None
 * @retval None
 */
static void invert_accel_offset(struct bmi2_accel_offset *offset_data);

/*!
 * @brief This internal API writes the offset data in the offset compensation
 * register.
 *
 * @param[in] offset        : offset data
 * @param[in] dev           : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t write_accel_offset(const struct bmi2_accel_offset *offset, struct bmi2_dev *dev);

/*!
 * @brief This internal API restores the configurations saved before performing
 * accelerometer FOC.
 *
 * @param[in] acc_cfg       : Accelerometer configuration value
 * @param[in] acc_en        : Accelerometer enable value
 * @param[in] aps           : Advance power mode value
 * @param[in] dev           : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t restore_accel_foc_config(struct bmi2_accel_config *acc_cfg,
                                       uint8_t aps,
                                       uint8_t acc_en,
                                       struct bmi2_dev *dev);

/*!
 * @brief This internal API saves the configurations before performing gyroscope
 * FOC.
 *
 * @param[out] gyr_cfg  : Gyroscope configuration value
 * @param[out] gyr_en   : Gyroscope enable value
 * @param[out] aps      : Advance power mode value
 * @param[in] dev       : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t save_gyro_config(struct bmi2_gyro_config *gyr_cfg, uint8_t *aps, uint8_t *gyr_en, struct bmi2_dev *dev);

/*!
 * @brief This internal sets configurations for performing gyroscope FOC.
 *
 * @param[in] dev   : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_gyro_foc_config(struct bmi2_dev *dev);

/*!
 * @brief This internal API inverts the gyroscope offset data.
 *
 * @param[out] offset_data  : Stores the inverted offset data
 *
 * @return None
 * @retval None
 */
static void invert_gyro_offset(struct bmi2_sens_axes_data *offset_data);

/*!
 * @brief This internal API restores the gyroscope configurations saved
 * before performing FOC.
 *
 * @param[in] gyr_cfg   : Gyroscope configuration value
 * @param[in] gyr_en    : Gyroscope enable value
 * @param[in] aps       : Advance power mode value
 * @param[in] dev       : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t restore_gyro_config(struct bmi2_gyro_config *gyr_cfg, uint8_t aps, uint8_t gyr_en, struct bmi2_dev *dev);

/*!
 * @brief This internal API saturates the gyroscope data value before writing to
 * to 10 bit offset register.
 *
 * @param[in, out] gyr_off  : Gyroscope data to be stored in offset register
 *
 * @return None
 * @retval None
 */
static void saturate_gyro_data(struct bmi2_sens_axes_data *gyr_off);

/*!
 * @brief This internal API reads the gyroscope data for x, y and z axis from
 * the sensor.
 *
 * @param[out] gyro         : Buffer to store the gyroscope value.
 * @param[in]  dev          : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t read_gyro_xyz(struct bmi2_sens_axes_data *gyro, struct bmi2_dev *dev);

/*!
 * @brief This internal API is used to check the boundary conditions.
 *
 * @param[in]     dev    : Structure instance of bmi2_dev.
 * @param[in,out] val    : Pointer to the value to be validated.
 * @param[in]     min    : minimum value.
 * @param[in]     max    : maximum value.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t check_boundary_val(uint8_t *val, uint8_t min, uint8_t max, struct bmi2_dev *dev);

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t null_ptr_check(const struct bmi2_dev *dev);

/*!
 * @brief This updates the result for CRT or gyro self-test.
 *
 * @param[in] dev : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t crt_gyro_st_update_result(struct bmi2_dev *dev);

/*!
 * @brief This function is to get the st_status status.
 *
 * @param[in] *st_status: gets the crt running status
 * @param[in] dev   : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_st_running(uint8_t *st_status, struct bmi2_dev *dev);

/*!
 * @brief This function is to set crt bit to running.
 *
 * @param[in] enable
 * @param[in] dev   : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_st_running(uint8_t st_status, struct bmi2_dev *dev);

/*!
 * @brief This function is to make the initial changes for CRT.
 * Disable the gyro, OIS, aps
 * Note: For the purpose of preparing CRT Gyro, OIS and APS are disabled
 *
 * @param[in] dev   : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t crt_prepare_setup(struct bmi2_dev *dev);

static int8_t do_gtrigger_test(uint8_t gyro_st_crt, struct bmi2_dev *dev);

/*!
 * @brief This function is to get the rdy for dl bit status
 * this will toggle from 0 to 1 and visevers according to the
 * dowload status
 *
 * @param[in] *rdy_for_dl: gets the rdy_for_dl status
 * @param[in] dev   : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_rdy_for_dl(uint8_t *rdy_for_dl, struct bmi2_dev *dev);

/*!
 * @brief This function is to write the config file in the given location for crt.
 * which inter checks the status of the rdy_for_dl bit and also the crt running, and
 * wirtes the given size.
 *
 * @param[in] write_len: length of the words to be written
 * @param[in] config_file_size: length of the words to be written
 * @param[in] start_index: provide the start index from where config file has to written
 * @param[in] dev   : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t write_crt_config_file(uint16_t write_len,
                                    uint16_t config_file_size,
                                    uint16_t start_index,
                                    struct bmi2_dev *dev);

/*!
 * @brief This function is to check for rdy_for_dl bit to toggle for CRT process
 *
 * @param[in] retry_complete: wait for given time to toggle
 * @param[in] download_ready:  get the status for rdy_for_dl
 * @param[in] dev   : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t wait_rdy_for_dl_toggle(uint8_t retry_complete, uint8_t download_ready, struct bmi2_dev *dev);

/*!
 * @brief This function is to wait till the CRT or gyro self-test process is completed
 *
 * @param[in] retry_complete: wait for given time to complete the crt process
 * @param[in] dev   : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t wait_st_running(uint8_t retry_complete, struct bmi2_dev *dev);

/*!
 * @brief This function is to complete the crt process if max burst length is not zero
 * this checks for the crt status and rdy_for_dl bit to toggle
 *
 * @param[in] last_byte_flag: to provide the last toggled state
 * @param[in] dev   : Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t process_crt_download(uint8_t last_byte_flag, struct bmi2_dev *dev);

/*!
 * @brief This api is used to enable the gyro self-test or crt.
 *
 * @param[in] dev        : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t select_self_test(uint8_t gyro_st_crt, struct bmi2_dev *dev);

/*!
 * @brief This api is used to enable/disable abort.
 *
 * @param[in] abort_enable  : variable to enable the abort feature.
 * @param[in] dev        : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t abort_bmi2(uint8_t abort_enable, struct bmi2_dev *dev);

/*!
 * @brief This api is use to wait till  gyro self-test is completed and update the status of gyro
 * self-test.
 *
 * @param[in] dev   : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t gyro_self_test_completed(struct bmi2_gyro_self_test_status *gyro_st_result, struct bmi2_dev *dev);

/*!
 * @brief This api is used to trigger the preparation for system for NVM programming.
 *
 * @param[out] nvm_prep : pointer to variable to store the status of nvm_prep_prog.
 * @param[in] dev       : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_nvm_prep_prog(uint8_t nvm_prep, struct bmi2_dev *dev);

/*!
 * @brief This api validates accel foc position as per the range
 *
 * @param[in] sens_list : Sensor type
 * @param[in] accel_g_axis : accel axis to foc. NA for gyro foc
 * @param[in] avg_foc_data : average value of sensor sample datas
 * @param[in] dev : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t validate_foc_position(uint8_t sens_list,
                                    const struct bmi2_accel_foc_g_value *accel_g_axis,
                                    struct bmi2_sens_axes_data avg_foc_data,
                                    struct bmi2_dev *dev);

/*!
 * @brief This api validates accel foc axis given as input
 *
 * @param[in] avg_foc_data : average value of sensor sample datas
 * @param[in] dev : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t validate_foc_accel_axis(int16_t avg_foc_data, struct bmi2_dev *dev);

/*!
 * @brief This api is used to verify the right position of the sensor before doing accel foc
 *
 * @param[in] dev : Structure instance of bmi2_dev.
 * @param[in] sens_list: Sensor type
 * @param[in] accel_g_axis: Accel Foc axis and sign input
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t verify_foc_position(uint8_t sens_list,
                                  const struct bmi2_accel_foc_g_value *accel_g_axis,
                                  struct bmi2_dev *dev);

/*!
 * @brief This API reads and provides average for 128 samples of sensor data for foc operation
 * gyro.
 *
 * @param[in] sens_list : Sensor type.
 * @param[in] bmi2_dev: Structure instance of bmi2_dev.
 * @param[in] temp_foc_data: to store data samples
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_average_of_sensor_data(uint8_t sens_list,
                                         struct bmi2_foc_temp_value *temp_foc_data,
                                         struct bmi2_dev *dev);

/*!
 * @brief This internal api gets major and minor version for config file
 *
 * @param[out] config_major     :   Pointer to store the major version
 * @param[out] config_minor     :   Pointer to store the minor version
 * @param[in]  dev              :   Structure instance of bmi2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t extract_config_file(uint8_t *config_major, uint8_t *config_minor, struct bmi2_dev *dev);

/*!
 * @brief This internal API is used to map the interrupts to the sensor.
 *
 * @param[in] map_int        : Structure instance of bmi2_map_int.
 * @param[in] type           : Type of feature or sensor.
 * @param[in] dev            : Structure instance of bmi2_dev.
 *
 * @return None
 * @retval None
 */
static void extract_feat_int_map(struct bmi2_map_int *map_int, uint8_t type, const struct bmi2_dev *dev);

/*!
 * @brief This internal API selects the sensors/features to be enabled or
 * disabled.
 *
 * @param[in]  sens_list    : Pointer to select the sensor.
 * @param[in]  n_sens       : Number of sensors selected.
 * @param[out] sensor_sel   : Gets the selected sensor.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t select_sensor(const uint8_t *sens_list, uint8_t n_sens, uint64_t *sensor_sel);

/*!
 * @brief This internal API enables the selected sensor/features.
 *
 * @param[in]       sensor_sel    : Selects the desired sensor.
 * @param[in, out]  dev           : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t sensor_enable(uint64_t sensor_sel, struct bmi2_dev *dev);

/*!
 * @brief This internal API disables the selected sensor/features.
 *
 * @param[in]       sensor_sel    : Selects the desired sensor.
 * @param[in, out]  dev           : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t sensor_disable(uint64_t sensor_sel, struct bmi2_dev *dev);

/******************************************************************************/
/*!  @name      User Interface Definitions                            */
/******************************************************************************/

/*!
 * @brief This API is the entry point for bmi2 sensor. It selects between
 * I2C/SPI interface, based on user selection. It reads and validates the
 * chip-id of the sensor.
 */
int8_t bmi2_sec_init(struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to assign chip id */
    uint8_t chip_id = 0;

    /* Structure to define the default values for axes re-mapping */
    struct bmi2_axes_remap axes_remap = {
        .x_axis = BMI2_MAP_X_AXIS, .x_axis_sign = BMI2_POS_SIGN, .y_axis = BMI2_MAP_Y_AXIS,
        .y_axis_sign = BMI2_POS_SIGN, .z_axis = BMI2_MAP_Z_AXIS, .z_axis_sign = BMI2_POS_SIGN
    };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Perform soft-reset to bring all register values to their
         * default values
         */
        rslt = bmi2_soft_reset(dev);

        if (rslt == BMI2_OK)
        {
            /* Read chip-id of the BMI2 sensor */
            rslt = bmi2_get_regs(BMI2_CHIP_ID_ADDR, &chip_id, 1, dev);

            if (rslt == BMI2_OK)
            {
                /* Validate chip-id */
                if (chip_id == dev->chip_id)
                {
                    /* Assign resolution to the structure */
                    dev->resolution = 16;

                    /* Set manual enable flag */
                    dev->aux_man_en = 1;

                    /* Set the default values for axis
                     *  re-mapping in the device structure
                     */
                    dev->remap = axes_remap;
                }
                else
                {
                    /* Storing the chip-id value read from
                     * the register to identify the sensor
                     */
                    dev->chip_id = chip_id;
                    rslt = BMI2_E_DEV_NOT_FOUND;
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address of bmi2
 * sensor.
 *
 * @note For most of the registers auto address increment applies, with the
 * exception of a few special registers, which trap the address. For e.g.,
 * Register address - 0x26, 0x5E.
 */
int8_t bmi2_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define loop */
    uint16_t index = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (data != NULL))
    {
        /* Variable to define temporary length */
        uint16_t temp_len = len + dev->dummy_byte;

        /* Variable to define temporary buffer */
        uint8_t temp_buf[temp_len];

        /* Configuring reg_addr for SPI Interface */
        if (dev->intf == BMI2_SPI_INTF)
        {
            reg_addr = (reg_addr | BMI2_SPI_RD_MASK);
        }

        dev->intf_rslt = dev->read(reg_addr, temp_buf, temp_len, dev->intf_ptr);

        if (dev->aps_status == BMI2_ENABLE)
        {
            dev->delay_us(450, dev->intf_ptr);
        }
        else
        {
            dev->delay_us(2, dev->intf_ptr);
        }

        if (dev->intf_rslt == BMI2_INTF_RET_SUCCESS)
        {
            /* Read the data from the position next to dummy byte */
            while (index < len)
            {
                data[index] = temp_buf[index + dev->dummy_byte];
                index++;
            }
        }
        else
        {
            rslt = BMI2_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes data to the given register address of bmi2 sensor.
 */
int8_t bmi2_set_regs(uint8_t reg_addr, const uint8_t *data, uint16_t len, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (data != NULL))
    {
        /* Configuring reg_addr for SPI Interface */
        if (dev->intf == BMI2_SPI_INTF)
        {
            reg_addr = (reg_addr & BMI2_SPI_WR_MASK);
        }

        dev->intf_rslt = dev->write(reg_addr, data, len, dev->intf_ptr);

        /* Delay for Low power mode of the sensor is 450 us */
        if (dev->aps_status == BMI2_ENABLE)
        {
            dev->delay_us(450, dev->intf_ptr);
        }
        /* Delay for Normal mode of the sensor is 2 us */
        else
        {
            dev->delay_us(2, dev->intf_ptr);
        }

        /* updating the advance power saver flag */
        if (reg_addr == BMI2_PWR_CONF_ADDR)
        {
            if (*data & BMI2_ADV_POW_EN_MASK)
            {
                dev->aps_status = BMI2_ENABLE;
            }
            else
            {
                dev->aps_status = BMI2_DISABLE;
            }
        }

        if (dev->intf_rslt != BMI2_INTF_RET_SUCCESS)
        {
            rslt = BMI2_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API resets bmi2 sensor. All registers are overwritten with
 * their default values.
 */
int8_t bmi2_soft_reset(struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define soft reset value */
    uint8_t data = BMI2_SOFT_RESET_CMD;

    /* Variable to read the dummy byte */
    uint8_t dummy_read = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Reset bmi2 device */
        rslt = bmi2_set_regs(BMI2_CMD_REG_ADDR, &data, 1, dev);
        dev->delay_us(2000, dev->intf_ptr);

        /* set APS flag as after soft reset the sensor is on advance power save mode */
        dev->aps_status = BMI2_ENABLE;

        /* Performing a dummy read to bring interface back to SPI from
         * I2C after a soft-reset
         */
        if ((rslt == BMI2_OK) && (dev->intf == BMI2_SPI_INTF))
        {
            rslt = bmi2_get_regs(BMI2_CHIP_ID_ADDR, &dummy_read, 1, dev);
        }

        if (rslt == BMI2_OK)
        {
            /* Write the configuration file */
            rslt = bmi2_write_config_file(dev);
        }

        /* Reset the sensor status flag in the device structure */
        if (rslt == BMI2_OK)
        {
            dev->sens_en_stat = 0;
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to get the config file major and minor version information.
 */
int8_t bmi2_get_config_file_version(uint8_t *config_major, uint8_t *config_minor, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* NULL pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (config_major != NULL) && (config_minor != NULL))
    {
        /* Extract the config file identification from the dmr page and get the major and minor version */
        rslt = extract_config_file(config_major, config_minor, dev);
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API enables/disables the advance power save mode in the sensor.
 */
int8_t bmi2_set_adv_power_save(uint8_t enable, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_get_regs(BMI2_PWR_CONF_ADDR, &reg_data, 1, dev);
        if (rslt == BMI2_OK)
        {
            reg_data = BMI2_SET_BIT_POS0(reg_data, BMI2_ADV_POW_EN, enable);
            rslt = bmi2_set_regs(BMI2_PWR_CONF_ADDR, &reg_data, 1, dev);

            if (rslt != BMI2_OK)
            {
                /* Return error if enable/disable APS fails */
                rslt = BMI2_E_SET_APS_FAIL;
            }

            if (rslt == BMI2_OK)
            {
                dev->aps_status = BMI2_GET_BIT_POS0(reg_data, BMI2_ADV_POW_EN);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the status of advance power save mode in the sensor.
 */
int8_t bmi2_get_adv_power_save(uint8_t *aps_status, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (aps_status != NULL))
    {
        rslt = bmi2_get_regs(BMI2_PWR_CONF_ADDR, &reg_data, 1, dev);
        if (rslt == BMI2_OK)
        {
            *aps_status = BMI2_GET_BIT_POS0(reg_data, BMI2_ADV_POW_EN);
            dev->aps_status = *aps_status;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API loads the configuration file into the bmi2 sensor.
 */
int8_t bmi2_write_config_file(struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to know the load status */
    uint8_t load_status = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (dev->config_size != 0))
    {
        /* Bytes written are multiples of 2 */
        if ((dev->read_write_len % 2) != 0)
        {
            dev->read_write_len = dev->read_write_len - 1;
        }

        if (dev->read_write_len < 2)
        {
            dev->read_write_len = 2;
        }

        /* Write the configuration file */
        rslt = write_config_file(dev);
        if (rslt == BMI2_OK)
        {
            /* Check the configuration load status */
            rslt = bmi2_get_internal_status(&load_status, dev);

            /* Return error if loading not successful */
            if ((rslt == BMI2_OK) && (!(load_status & BMI2_CONFIG_LOAD_SUCCESS)))
            {
                rslt = BMI2_E_CONFIG_LOAD;
            }
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets:
 *        1) The input output configuration of the selected interrupt pin:
 *           INT1 or INT2.
 *        2) The interrupt mode: permanently latched or non-latched.
 */
int8_t bmi2_set_int_pin_config(const struct bmi2_int_pin_config *int_cfg, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define data array */
    uint8_t data_array[3] = { 0 };

    /* Variable to store register data */
    uint8_t reg_data = 0;

    /* Variable to define type of interrupt pin  */
    uint8_t int_pin = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (int_cfg != NULL))
    {
        /* Copy the pin type to a local variable */
        int_pin = int_cfg->pin_type;
        if ((int_pin > BMI2_INT_NONE) && (int_pin < BMI2_INT_PIN_MAX))
        {
            /* Get the previous configuration data */
            rslt = bmi2_get_regs(BMI2_INT1_IO_CTRL_ADDR, data_array, 3, dev);
            if (rslt == BMI2_OK)
            {
                /* Set interrupt pin 1 configuration */
                if ((int_pin == BMI2_INT1) || (int_pin == BMI2_INT_BOTH))
                {
                    /* Configure active low or high */
                    reg_data = BMI2_SET_BITS(data_array[0], BMI2_INT_LEVEL, int_cfg->pin_cfg[0].lvl);

                    /* Configure push-pull or open drain */
                    reg_data = BMI2_SET_BITS(reg_data, BMI2_INT_OPEN_DRAIN, int_cfg->pin_cfg[0].od);

                    /* Configure output enable */
                    reg_data = BMI2_SET_BITS(reg_data, BMI2_INT_OUTPUT_EN, int_cfg->pin_cfg[0].output_en);

                    /* Configure input enable */
                    reg_data = BMI2_SET_BITS(reg_data, BMI2_INT_INPUT_EN, int_cfg->pin_cfg[0].input_en);

                    /* Copy the data to be written in the respective array */
                    data_array[0] = reg_data;
                }

                /* Set interrupt pin 2 configuration */
                if ((int_pin == BMI2_INT2) || (int_pin == BMI2_INT_BOTH))
                {
                    /* Configure active low or high */
                    reg_data = BMI2_SET_BITS(data_array[1], BMI2_INT_LEVEL, int_cfg->pin_cfg[1].lvl);

                    /* Configure push-pull or open drain */
                    reg_data = BMI2_SET_BITS(reg_data, BMI2_INT_OPEN_DRAIN, int_cfg->pin_cfg[1].od);

                    /* Configure output enable */
                    reg_data = BMI2_SET_BITS(reg_data, BMI2_INT_OUTPUT_EN, int_cfg->pin_cfg[1].output_en);

                    /* Configure input enable */
                    reg_data = BMI2_SET_BITS(reg_data, BMI2_INT_INPUT_EN, int_cfg->pin_cfg[1].input_en);

                    /* Copy the data to be written in the respective array */
                    data_array[1] = reg_data;
                }

                /* Configure the interrupt mode */
                data_array[2] = BMI2_SET_BIT_POS0(data_array[2], BMI2_INT_LATCH, int_cfg->int_latch);

                /* Set the configurations simultaneously as
                 * INT1_IO_CTRL, INT2_IO_CTRL, and INT_LATCH lie
                 * in consecutive addresses
                 */
                rslt = bmi2_set_regs(BMI2_INT1_IO_CTRL_ADDR, data_array, 3, dev);
            }
        }
        else
        {
            rslt = BMI2_E_INVALID_INT_PIN;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets:
 *        1) The input output configuration of the selected interrupt pin:
 *           INT1 or INT2.
 *        2) The interrupt mode: permanently latched or non-latched.
 */
int8_t bmi2_get_int_pin_config(struct bmi2_int_pin_config *int_cfg, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define data array */
    uint8_t data_array[3] = { 0 };

    /* Variable to define type of interrupt pin  */
    uint8_t int_pin = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (int_cfg != NULL))
    {
        /* Copy the pin type to a local variable */
        int_pin = int_cfg->pin_type;

        /* Get the previous configuration data */
        rslt = bmi2_get_regs(BMI2_INT1_IO_CTRL_ADDR, data_array, 3, dev);
        if (rslt == BMI2_OK)
        {
            /* Get interrupt pin 1 configuration */
            if ((int_pin == BMI2_INT1) || (int_pin == BMI2_INT_BOTH))
            {
                /* Get active low or high */
                int_cfg->pin_cfg[0].lvl = BMI2_GET_BITS(data_array[0], BMI2_INT_LEVEL);

                /* Get push-pull or open drain */
                int_cfg->pin_cfg[0].od = BMI2_GET_BITS(data_array[0], BMI2_INT_OPEN_DRAIN);

                /* Get output enable */
                int_cfg->pin_cfg[0].output_en = BMI2_GET_BITS(data_array[0], BMI2_INT_OUTPUT_EN);

                /* Get input enable */
                int_cfg->pin_cfg[0].input_en = BMI2_GET_BITS(data_array[0], BMI2_INT_INPUT_EN);
            }

            /* Get interrupt pin 2 configuration */
            if ((int_pin == BMI2_INT2) || (int_pin == BMI2_INT_BOTH))
            {
                /* Get active low or high */
                int_cfg->pin_cfg[1].lvl = BMI2_GET_BITS(data_array[1], BMI2_INT_LEVEL);

                /* Get push-pull or open drain */
                int_cfg->pin_cfg[1].od = BMI2_GET_BITS(data_array[1], BMI2_INT_OPEN_DRAIN);

                /* Get output enable */
                int_cfg->pin_cfg[1].output_en = BMI2_GET_BITS(data_array[1], BMI2_INT_OUTPUT_EN);

                /* Get input enable */
                int_cfg->pin_cfg[1].input_en = BMI2_GET_BITS(data_array[1], BMI2_INT_INPUT_EN);
            }

            /* Get interrupt mode */
            int_cfg->int_latch = BMI2_GET_BIT_POS0(data_array[2], BMI2_INT_LATCH);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the interrupt status of both feature and data
 * interrupts
 */
int8_t bmi2_get_int_status(uint16_t *int_status, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store data */
    uint8_t data_array[2] = { 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (int_status != NULL))
    {
        /* Get the interrupt status */
        rslt = bmi2_get_regs(BMI2_INT_STATUS_0_ADDR, data_array, 2, dev);
        if (rslt == BMI2_OK)
        {
            *int_status = (uint16_t) data_array[0] | ((uint16_t) data_array[1] << 8);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API selects the sensors/features to be enabled.
 */
int8_t bmi2_sensor_enable(const uint8_t *sens_list, uint8_t n_sens, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to select sensor */
    uint64_t sensor_sel = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (sens_list != NULL))
    {
        /* Get the selected sensors */
        rslt = select_sensor(sens_list, n_sens, &sensor_sel);
        if (rslt == BMI2_OK)
        {
            /* Enable the selected sensors */
            rslt = sensor_enable(sensor_sel, dev);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API selects the sensors/features to be disabled.
 */
int8_t bmi2_sensor_disable(const uint8_t *sens_list, uint8_t n_sens, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to select sensor */
    uint64_t sensor_sel = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (sens_list != NULL))
    {
        /* Get the selected sensors */
        rslt = select_sensor(sens_list, n_sens, &sensor_sel);
        if (rslt == BMI2_OK)
        {
            /* Disable the selected sensors */
            rslt = sensor_disable(sensor_sel, dev);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the sensor/feature configuration.
 */
int8_t bmi2_set_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define loop */
    uint8_t loop;

    /* Variable to get the status of advance power save */
    uint8_t aps_stat = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (sens_cfg != NULL))
    {
        /* Get status of advance power save mode */
        aps_stat = dev->aps_status;

        for (loop = 0; loop < n_sens; loop++)
        {
            /* Disable Advance power save if enabled for auxiliary
             * and feature configurations
             */
            if (aps_stat == BMI2_ENABLE)
            {
                /* Disable advance power save if
                 * enabled
                 */
                rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
            }

            if (rslt == BMI2_OK)
            {
                switch (sens_cfg[loop].type)
                {
                    /* Set accelerometer configuration */
                    case BMI2_ACCEL:
                        rslt = set_accel_config(&sens_cfg[loop].cfg.acc, dev);
                        break;

                    /* Set gyroscope configuration */
                    case BMI2_GYRO:
                        rslt = set_gyro_config(&sens_cfg[loop].cfg.gyr, dev);
                        break;

                    /* Set auxiliary configuration */
                    case BMI2_AUX:
                        rslt = set_aux_config(&sens_cfg[loop].cfg.aux, dev);
                        break;

                    /* Set gyroscope user gain configuration */
                    case BMI2_GYRO_GAIN_UPDATE:
                        rslt = set_gyro_user_gain_config(&sens_cfg[loop].cfg.gyro_gain_update, dev);
                        break;

                    default:
                        rslt = BMI2_E_INVALID_SENSOR;
                        break;
                }
            }

            /* Return error if any of the set configurations fail */
            if (rslt != BMI2_OK)
            {
                break;
            }
        }

        /* Enable Advance power save if disabled while configuring and
         * not when already disabled
         */
        if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
        {
            rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the sensor/feature configuration.
 */
int8_t bmi2_get_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define loop */
    uint8_t loop;

    /* Variable to get the status of advance power save */
    uint8_t aps_stat = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (sens_cfg != NULL))
    {
        /* Get status of advance power save mode */
        aps_stat = dev->aps_status;
        for (loop = 0; loop < n_sens; loop++)
        {
            /* Disable Advance power save if enabled for auxiliary
             * and feature configurations
             */
            if ((sens_cfg[loop].type >= BMI2_MAIN_SENS_MAX_NUM) || (sens_cfg[loop].type == BMI2_AUX))
            {

                if (aps_stat == BMI2_ENABLE)
                {
                    /* Disable advance power save if
                     * enabled
                     */
                    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
                }
            }

            if (rslt == BMI2_OK)
            {
                switch (sens_cfg[loop].type)
                {
                    /* Get accelerometer configuration */
                    case BMI2_ACCEL:
                        rslt = get_accel_config(&sens_cfg[loop].cfg.acc, dev);
                        break;

                    /* Get gyroscope configuration */
                    case BMI2_GYRO:
                        rslt = get_gyro_config(&sens_cfg[loop].cfg.gyr, dev);
                        break;

                    /* Get auxiliary configuration */
                    case BMI2_AUX:
                        rslt = get_aux_config(&sens_cfg[loop].cfg.aux, dev);
                        break;

                    /* Get gyroscope user gain configuration */
                    case BMI2_GYRO_GAIN_UPDATE:
                        rslt = get_gyro_gain_update_config(&sens_cfg[loop].cfg.gyro_gain_update, dev);
                        break;

                    default:
                        rslt = BMI2_E_INVALID_SENSOR;
                        break;
                }
            }

            /* Return error if any of the get configurations fail */
            if (rslt != BMI2_OK)
            {
                break;
            }
        }

        /* Enable Advance power save if disabled while configuring and
         * not when already disabled
         */
        if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
        {
            rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the sensor/feature data for accelerometer, gyroscope,
 * auxiliary sensor, step counter, high-g, gyroscope user-gain update,
 * orientation, gyroscope cross sensitivity and error status for NVM and VFRM.
 */
int8_t bmi2_get_sensor_data(struct bmi2_sensor_data *sensor_data, uint8_t n_sens, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define loop */
    uint8_t loop;

    /* Variable to get the status of advance power save */
    uint8_t aps_stat = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (sensor_data != NULL))
    {
        /* Get status of advance power save mode */
        aps_stat = dev->aps_status;
        for (loop = 0; loop < n_sens; loop++)
        {
            /* Disable Advance power save if enabled for feature
             * configurations
             */
            if (sensor_data[loop].type >= BMI2_MAIN_SENS_MAX_NUM)
            {
                if (aps_stat == BMI2_ENABLE)
                {
                    /* Disable advance power save if
                     * enabled
                     */
                    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
                }
            }

            if (rslt == BMI2_OK)
            {
                switch (sensor_data[loop].type)
                {
                    case BMI2_ACCEL:

                        /* Get accelerometer data */
                        rslt = get_accel_sensor_data(&sensor_data[loop].sens_data.acc, BMI2_ACC_X_LSB_ADDR, dev);
                        break;
                    case BMI2_GYRO:

                        /* Get gyroscope data */
                        rslt = get_gyro_sensor_data(&sensor_data[loop].sens_data.gyr, BMI2_GYR_X_LSB_ADDR, dev);
                        break;
                    case BMI2_AUX:

                        /* Get auxiliary sensor data in data mode */
                        rslt = read_aux_data_mode(sensor_data[loop].sens_data.aux_data, dev);
                        break;

                    case BMI2_GYRO_CROSS_SENSE:

                        /* Get Gyroscope cross sense value of z axis */
                        rslt = get_gyro_cross_sense(&sensor_data[loop].sens_data.correction_factor_zx, dev);
                        break;

                    case BMI2_GYRO_GAIN_UPDATE:

                        /* Get saturation status of gyroscope user gain update  */
                        rslt = get_gyro_gain_update_status(&sensor_data[loop].sens_data.gyro_user_gain_status, dev);
                        break;
                    default:
                        rslt = BMI2_E_INVALID_SENSOR;
                        break;
                }

                /* Return error if any of the get sensor data fails */
                if (rslt != BMI2_OK)
                {
                    break;
                }
            }

            /* Enable Advance power save if disabled while
             * configuring and not when already disabled
             */
            if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
            {
                rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
            }
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO configuration in the sensor.
 */
int8_t bmi2_set_fifo_config(uint16_t config, uint8_t enable, struct bmi2_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };
    uint8_t max_burst_len = 0;

    /* Variable to store data of FIFO configuration register 0 */
    uint8_t fifo_config_0 = (uint8_t)(config & BMI2_FIFO_CONFIG_0_MASK);

    /* Variable to store data of FIFO configuration register 1 */
    uint8_t fifo_config_1 = (uint8_t)((config & BMI2_FIFO_CONFIG_1_MASK) >> 8);

    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_get_regs(BMI2_FIFO_CONFIG_0_ADDR, data, BMI2_FIFO_CONFIG_LENGTH, dev);
        if (rslt == BMI2_OK)
        {
            /* Get data to set FIFO configuration register 0 */
            if (fifo_config_0 > 0)
            {
                if (enable == BMI2_ENABLE)
                {
                    data[0] = data[0] | fifo_config_0;
                }
                else
                {
                    data[0] = data[0] & (~fifo_config_0);
                }
            }

            /* Get data to set FIFO configuration register 1 */
            if (enable == BMI2_ENABLE)
            {
                data[1] = data[1] | fifo_config_1;
                if (dev->variant_feature & BMI2_CRT_RTOSK_ENABLE)
                {

                    /* Burst length is needed for CRT
                     *  FIFO enable will reset the default values
                     *  So configure the max burst length again.
                     */
                    rslt = get_maxburst_len(&max_burst_len, dev);
                    if (rslt == BMI2_OK && max_burst_len == 0)
                    {
                        rslt = set_maxburst_len(BMI2_CRT_MIN_BURST_WORD_LENGTH, dev);
                    }
                }
            }
            else
            {
                data[1] = data[1] & (~fifo_config_1);
            }

            /* Set the FIFO configurations */
            if (rslt == BMI2_OK)
            {
                rslt = bmi2_set_regs(BMI2_FIFO_CONFIG_0_ADDR, data, 2, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the FIFO configuration from the sensor.
 */
int8_t bmi2_get_fifo_config(uint16_t *fifo_config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store data */
    uint8_t data[2] = { 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (fifo_config != NULL))
    {
        /* Get the FIFO configuration value */
        rslt = bmi2_get_regs(BMI2_FIFO_CONFIG_0_ADDR, data, BMI2_FIFO_CONFIG_LENGTH, dev);
        if (rslt == BMI2_OK)
        {
            (*fifo_config) = (uint16_t)((uint16_t) data[0] & BMI2_FIFO_CONFIG_0_MASK);
            (*fifo_config) |= (uint16_t)(((uint16_t) data[1] << 8) & BMI2_FIFO_CONFIG_1_MASK);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the FIFO data.
 */
int8_t bmi2_read_fifo_data(struct bmi2_fifo_frame *fifo, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store FIFO configuration data */
    uint8_t config_data[2] = { 0 };

    /* Variable to define FIFO address */
    uint8_t addr = BMI2_FIFO_DATA_ADDR;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (fifo != NULL))
    {
        /* Clear the FIFO data structure */
        reset_fifo_frame_structure(fifo, dev);

        /* Read FIFO data */
        rslt = bmi2_get_regs(addr, fifo->data, fifo->length, dev);

        if (rslt == BMI2_OK)
        {

            /* Get the set FIFO frame configurations */
            rslt = bmi2_get_regs(BMI2_FIFO_CONFIG_0_ADDR, config_data, 2, dev);
            if (rslt == BMI2_OK)
            {
                /* Get FIFO header status */
                fifo->header_enable = (uint8_t)((config_data[1]) & (BMI2_FIFO_HEADER_EN >> 8));

                /* Get sensor enable status, of which the data is to be read */
                fifo->data_enable =
                    (uint16_t)(((config_data[0]) | ((uint16_t) config_data[1] << 8)) & BMI2_FIFO_ALL_EN);
            }
        }
        else
        {
            rslt = BMI2_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API parses and extracts the accelerometer frames from FIFO data
 * read by the "bmi2_read_fifo_data" API and stores it in the "accel_data"
 * structure instance.
 */
int8_t bmi2_extract_accel(struct bmi2_sens_axes_data *accel_data,
                          uint16_t *accel_length,
                          struct bmi2_fifo_frame *fifo,
                          const struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to index the bytes */
    uint16_t data_index = 0;

    /* Variable to index accelerometer frames */
    uint16_t accel_index = 0;

    /* Variable to store the number of bytes to be read */
    uint16_t data_read_length = 0;

    /* Variable to define the data enable byte */
    uint8_t data_enable = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (accel_data != NULL) && (accel_length != NULL) && (fifo != NULL))
    {
        /* Parsing the FIFO data in header-less mode */
        if (fifo->header_enable == 0)
        {

            /* Get the number of accelerometer bytes to be read */
            rslt = parse_fifo_accel_len(&data_index, &data_read_length, accel_length, fifo);

            /* Convert word to byte since all sensor enables are in a byte */
            data_enable = (uint8_t)(fifo->data_enable >> 8);
            for (; (data_index < data_read_length) && (rslt != BMI2_W_FIFO_EMPTY);)
            {
                /* Unpack frame to get the accelerometer data */
                rslt = unpack_accel_frame(accel_data, &data_index, &accel_index, data_enable, fifo, dev);

                if (rslt != BMI2_W_FIFO_EMPTY)
                {
                    /* Check for the availability of next two bytes of FIFO data */
                    rslt = check_empty_fifo(&data_index, fifo);
                }
            }

            /* Update number of accelerometer frames to be read */
            (*accel_length) = accel_index;

            /* Update the accelerometer byte index */
            fifo->acc_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            rslt = extract_accel_header_mode(accel_data, accel_length, fifo, dev);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API parses and extracts the gyroscope frames from FIFO data
 * read by the "bmi2_read_fifo_data" API and stores it in the "gyro_data"
 * structure instance.
 */
int8_t bmi2_extract_gyro(struct bmi2_sens_axes_data *gyro_data,
                         uint16_t *gyro_length,
                         struct bmi2_fifo_frame *fifo,
                         const struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to index the bytes */
    uint16_t data_index = 0;

    /* Variable to index gyroscope frames */
    uint16_t gyro_index = 0;

    /* Variable to store the number of bytes to be read */
    uint16_t data_read_length = 0;

    /* Variable to define the data enable byte */
    uint8_t data_enable = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (gyro_data != NULL) && (gyro_length != NULL) && (fifo != NULL))
    {
        /* Parsing the FIFO data in header-less mode */
        if (fifo->header_enable == 0)
        {
            /* Get the number of gyro bytes to be read */
            rslt = parse_fifo_gyro_len(&data_index, &data_read_length, gyro_length, fifo);

            /* Convert word to byte since all sensor enables are in a byte */
            data_enable = (uint8_t)(fifo->data_enable >> 8);
            for (; (data_index < data_read_length) && (rslt != BMI2_W_FIFO_EMPTY);)
            {
                /* Unpack frame to get gyroscope data */
                rslt = unpack_gyro_frame(gyro_data, &data_index, &gyro_index, data_enable, fifo, dev);
                if (rslt != BMI2_W_FIFO_EMPTY)
                {
                    /* Check for the availability of next two bytes of FIFO data */
                    rslt = check_empty_fifo(&data_index, fifo);
                }
            }

            /* Update number of gyroscope frames to be read */
            (*gyro_length) = gyro_index;

            /* Update the gyroscope byte index */
            fifo->acc_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            rslt = extract_gyro_header_mode(gyro_data, gyro_length, fifo, dev);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API parses and extracts the auxiliary frames from FIFO data
 * read by the "bmi2_read_fifo_data" API and stores it in "aux_data" buffer.
 */
int8_t bmi2_extract_aux(struct bmi2_aux_fifo_data *aux,
                        uint16_t *aux_length,
                        struct bmi2_fifo_frame *fifo,
                        const struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to index the bytes */
    uint16_t data_index = 0;

    /* Variable to index auxiliary frames */
    uint16_t aux_index = 0;

    /* Variable to store the number of bytes to be read */
    uint16_t data_read_length = 0;

    /* Variable to define the data enable byte */
    uint8_t data_enable = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (aux != NULL) && (aux_length != NULL) && (fifo != NULL))
    {
        /* Parsing the FIFO data in header-less mode */
        if (fifo->header_enable == 0)
        {
            rslt = parse_fifo_aux_len(&data_index, &data_read_length, aux_length, fifo);

            /* Convert word to byte since all sensor enables are in
             * a byte
             */
            data_enable = (uint8_t)(fifo->data_enable >> 8);
            for (; (data_index < data_read_length) && (rslt != BMI2_W_FIFO_EMPTY);)
            {
                /* Unpack frame to get auxiliary data */
                rslt = unpack_aux_frame(aux, &data_index, &aux_index, data_enable, fifo, dev);
                if (rslt != BMI2_W_FIFO_EMPTY)
                {
                    /* Check for the availability of next
                     * two bytes of FIFO data
                     */
                    rslt = check_empty_fifo(&data_index, fifo);
                }
            }

            /* Update number of auxiliary frames to be read */
            *aux_length = aux_index;

            /* Update the auxiliary byte index */
            fifo->aux_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            rslt = extract_aux_header_mode(aux, aux_length, fifo, dev);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes the available sensor specific commands to the sensor.
 */
int8_t bmi2_set_command_register(uint8_t command, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Set the command in the command register */
        rslt = bmi2_set_regs(BMI2_CMD_REG_ADDR, &command, 1, dev);
    }

    return rslt;
}

/*
 * @brief This API sets the FIFO self wake up functionality in the sensor.
 */
int8_t bmi2_set_fifo_self_wake_up(uint8_t fifo_self_wake_up, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Set FIFO self wake-up */
        rslt = bmi2_get_regs(BMI2_PWR_CONF_ADDR, &data, 1, dev);
        if (rslt == BMI2_OK)
        {
            data = BMI2_SET_BITS(data, BMI2_FIFO_SELF_WAKE_UP, fifo_self_wake_up);
            rslt = bmi2_set_regs(BMI2_PWR_CONF_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the status of FIFO self wake up functionality from
 * the sensor.
 */
int8_t bmi2_get_fifo_self_wake_up(uint8_t *fifo_self_wake_up, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (fifo_self_wake_up != NULL))
    {
        /* Get the status of FIFO self wake-up */
        rslt = bmi2_get_regs(BMI2_PWR_CONF_ADDR, &data, 1, dev);
        if (rslt == BMI2_OK)
        {
            (*fifo_self_wake_up) = BMI2_GET_BITS(data, BMI2_FIFO_SELF_WAKE_UP);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO water-mark level in the sensor.
 */
int8_t bmi2_set_fifo_wm(uint16_t fifo_wm, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store data */
    uint8_t data[2] = { 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Get LSB value of FIFO water-mark */
        data[0] = BMI2_GET_LSB(fifo_wm);

        /* Get MSB value of FIFO water-mark */
        data[1] = BMI2_GET_MSB(fifo_wm);

        /* Set the FIFO water-mark level */
        rslt = bmi2_set_regs(BMI2_FIFO_WTM_0_ADDR, data, 2, dev);
    }

    return rslt;
}

/*!
 * @brief This API reads the FIFO water mark level set in the sensor.
 */
int8_t bmi2_get_fifo_wm(uint16_t *fifo_wm, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to to store data */
    uint8_t data[2] = { 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (fifo_wm != NULL))
    {
        /* Read the FIFO water mark level */
        rslt = bmi2_get_regs(BMI2_FIFO_WTM_0_ADDR, data, BMI2_FIFO_WM_LENGTH, dev);
        if (rslt == BMI2_OK)
        {
            (*fifo_wm) = (uint16_t)((uint16_t) data[1] << 8) | (data[0]);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets either filtered or un-filtered FIFO accelerometer or
 * gyroscope data.
 */
int8_t bmi2_set_fifo_filter_data(uint8_t sens_sel, uint8_t fifo_filter_data, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        switch (sens_sel)
        {
            case BMI2_ACCEL:

                /* Validate filter mode */
                if (fifo_filter_data <= BMI2_MAX_VALUE_FIFO_FILTER)
                {
                    /* Set the accelerometer FIFO filter data */
                    rslt = bmi2_get_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                    if (rslt == BMI2_OK)
                    {
                        data = BMI2_SET_BITS(data, BMI2_ACC_FIFO_FILT_DATA, fifo_filter_data);
                        rslt = bmi2_set_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                    }
                }
                else
                {
                    rslt = BMI2_E_OUT_OF_RANGE;
                }

                break;
            case BMI2_GYRO:

                /* Validate filter mode */
                if (fifo_filter_data <= BMI2_MAX_VALUE_FIFO_FILTER)
                {
                    /* Set the gyroscope FIFO filter data */
                    rslt = bmi2_get_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                    if (rslt == BMI2_OK)
                    {
                        data = BMI2_SET_BITS(data, BMI2_GYR_FIFO_FILT_DATA, fifo_filter_data);
                        rslt = bmi2_set_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                    }
                }
                else
                {
                    rslt = BMI2_E_OUT_OF_RANGE;
                }

                break;
            default:
                rslt = BMI2_E_INVALID_SENSOR;
                break;
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the FIFO accelerometer or gyroscope filter data.
 */
int8_t bmi2_get_fifo_filter_data(uint8_t sens_sel, uint8_t *fifo_filter_data, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store FIFO filter mode */
    uint8_t data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (fifo_filter_data != NULL))
    {
        switch (sens_sel)
        {
            case BMI2_ACCEL:

                /* Read the accelerometer FIFO filter data */
                rslt = bmi2_get_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                if (rslt == BMI2_OK)
                {
                    (*fifo_filter_data) = BMI2_GET_BITS(data, BMI2_ACC_FIFO_FILT_DATA);
                }

                break;
            case BMI2_GYRO:

                /* Read the gyroscope FIFO filter data */
                rslt = bmi2_get_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                if (rslt == BMI2_OK)
                {
                    (*fifo_filter_data) = BMI2_GET_BITS(data, BMI2_GYR_FIFO_FILT_DATA);
                }

                break;
            default:
                rslt = BMI2_E_INVALID_SENSOR;
                break;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the down-sampling rates for accelerometer or gyroscope
 * FIFO data.
 */
int8_t bmi2_set_fifo_down_sample(uint8_t sens_sel, uint8_t fifo_down_samp, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store sampling rate */
    uint8_t data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        switch (sens_sel)
        {
            case BMI2_ACCEL:

                /* Set the accelerometer FIFO down sampling rate */
                rslt = bmi2_get_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                if (rslt == BMI2_OK)
                {
                    data = BMI2_SET_BITS(data, BMI2_ACC_FIFO_DOWNS, fifo_down_samp);
                    rslt = bmi2_set_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                }

                break;
            case BMI2_GYRO:

                /* Set the gyroscope FIFO down sampling rate */
                rslt = bmi2_get_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                if (rslt == BMI2_OK)
                {
                    data = BMI2_SET_BIT_POS0(data, BMI2_GYR_FIFO_DOWNS, fifo_down_samp);
                    rslt = bmi2_set_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                }

                break;
            default:
                rslt = BMI2_E_INVALID_SENSOR;
                break;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the down sampling rates which is configured for
 * accelerometer or gyroscope FIFO data.
 */
int8_t bmi2_get_fifo_down_sample(uint8_t sens_sel, uint8_t *fifo_down_samp, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store sampling rate */
    uint8_t data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (fifo_down_samp != NULL))
    {
        switch (sens_sel)
        {
            case BMI2_ACCEL:

                /* Read the accelerometer FIFO down data sampling rate */
                rslt = bmi2_get_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                if (rslt == BMI2_OK)
                {
                    (*fifo_down_samp) = BMI2_GET_BITS(data, BMI2_ACC_FIFO_DOWNS);
                }

                break;
            case BMI2_GYRO:

                /* Read the gyroscope FIFO down data sampling rate */
                rslt = bmi2_get_regs(BMI2_FIFO_DOWNS_ADDR, &data, 1, dev);
                if (rslt == BMI2_OK)
                {
                    (*fifo_down_samp) = BMI2_GET_BIT_POS0(data, BMI2_GYR_FIFO_DOWNS);
                }

                break;
            default:
                rslt = BMI2_E_INVALID_SENSOR;
                break;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the length of FIFO data available in the sensor in
 * bytes.
 */
int8_t bmi2_get_fifo_length(uint16_t *fifo_length, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define byte index */
    uint8_t index = 0;

    /* Array to store FIFO data length */
    uint8_t data[BMI2_FIFO_DATA_LENGTH] = { 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (fifo_length != NULL))
    {
        /* Read FIFO length */
        rslt = bmi2_get_regs(BMI2_FIFO_LENGTH_0_ADDR, data, BMI2_FIFO_DATA_LENGTH, dev);
        if (rslt == BMI2_OK)
        {
            /* Get the MSB byte index */
            index = BMI2_FIFO_LENGTH_MSB_BYTE;

            /* Get the MSB byte of FIFO length */
            data[index] = BMI2_GET_BIT_POS0(data[index], BMI2_FIFO_BYTE_COUNTER_MSB);

            /* Get total FIFO length */
            (*fifo_length) = ((data[index] << 8) | data[index - 1]);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the user-defined bytes of data from the given register
 * address of auxiliary sensor in manual mode.
 *
 * @note Change of BMI2_AUX_RD_ADDR is only allowed if AUX is not busy.
 */
int8_t bmi2_read_aux_man_mode(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store burst length */
    uint8_t burst_len = 0;

    /* Variable to define APS status */
    uint8_t aps_stat = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (aux_data != NULL))
    {
        /* Validate if manual mode */
        if (dev->aux_man_en)
        {
            /* Get status of advance power save mode */
            aps_stat = dev->aps_status;
            if (aps_stat == BMI2_ENABLE)
            {
                /* Disable APS if enabled */
                rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
            }

            if (rslt == BMI2_OK)
            {
                /* Map the register value set to that of burst
                 * length
                 */
                rslt = map_read_len(&burst_len, dev);
                if (rslt == BMI2_OK)
                {
                    /* Read auxiliary data */
                    rslt = read_aux_data(reg_addr, aux_data, len, burst_len, dev);
                }
            }

            /* Enable Advance power save if disabled for reading
             * data and not when already disabled
             */
            if ((rslt == BMI2_OK) && (aps_stat == BMI2_ENABLE))
            {
                rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
            }
        }
        else
        {
            rslt = BMI2_E_AUX_INVALID_CFG;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes the user-defined bytes of data and the address of
 * auxiliary sensor where data is to be written in manual mode.
 *
 * @note Change of BMI2_AUX_WR_ADDR is only allowed if AUX is not busy.
 */
int8_t bmi2_write_aux_man_mode(uint8_t reg_addr, const uint8_t *aux_data, uint16_t len, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define loop */
    uint8_t loop = 0;

    /* Variable to define APS status */
    uint8_t aps_stat = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (aux_data != NULL))
    {
        /* Validate if manual mode */
        if (dev->aux_man_en)
        {
            /* Get status of advance power save mode */
            aps_stat = dev->aps_status;
            if (aps_stat == BMI2_ENABLE)
            {
                /* Disable APS if enabled */
                rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
            }

            /* Byte write data in the corresponding address */
            if (rslt == BMI2_OK)
            {
                for (; ((loop < len) && (rslt == BMI2_OK)); loop++)
                {
                    rslt = write_aux_data((reg_addr + loop), aux_data[loop], dev);
                    dev->delay_us(1000, dev->intf_ptr);
                }
            }

            /* Enable Advance power save if disabled for writing
             * data and not when already disabled
             */
            if ((rslt == BMI2_OK) && (aps_stat == BMI2_ENABLE))
            {
                rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
            }
        }
        else
        {
            rslt = BMI2_E_AUX_INVALID_CFG;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes the user-defined bytes of data and the address of
 * auxiliary sensor where data is to be written, from an interleaved input,
 * in manual mode.
 *
 * @note Change of BMI2_AUX_WR_ADDR is only allowed if AUX is not busy.
 */
int8_t bmi2_write_aux_interleaved(uint8_t reg_addr, const uint8_t *aux_data, uint16_t len, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define loop */
    uint8_t loop = 1;

    /* Variable to define APS status */
    uint8_t aps_stat = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (aux_data != NULL))
    {
        /* Validate if manual mode */
        if (dev->aux_man_en)
        {
            /* Get status of advance power save mode */
            aps_stat = dev->aps_status;
            if (aps_stat == BMI2_ENABLE)
            {
                /* Disable APS if enabled */
                rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
            }

            if (rslt == BMI2_OK)
            {
                /* Write the start register address extracted
                 * from the interleaved data
                 */
                rslt = write_aux_data(reg_addr, aux_data[0], dev);

                /* Extract the remaining address and data from
                 * the interleaved data and write it in the
                 * corresponding addresses byte by byte
                 */
                for (; ((loop < len) && (rslt == BMI2_OK)); loop += 2)
                {
                    rslt = write_aux_data(aux_data[loop], aux_data[loop + 1], dev);
                    dev->delay_us(1000, dev->intf_ptr);
                }

                /* Enable Advance power save if disabled for
                 * writing data and not when already disabled
                 */
                if ((rslt == BMI2_OK) && (aps_stat == BMI2_ENABLE))
                {
                    rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
                }
            }
        }
        else
        {
            rslt = BMI2_E_AUX_INVALID_CFG;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the data ready status of accelerometer, gyroscope,
 * auxiliary, command decoder and busy status of auxiliary.
 */
int8_t bmi2_get_status(uint8_t *status, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (status != NULL))
    {
        rslt = bmi2_get_regs(BMI2_STATUS_ADDR, status, 1, dev);
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API enables/disables OIS interface.
 */
int8_t bmi2_set_ois_interface(uint8_t enable, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_get_regs(BMI2_IF_CONF_ADDR, &reg_data, 1, dev);
        if (rslt == BMI2_OK)
        {
            /* Enable/Disable OIS interface */
            reg_data = BMI2_SET_BITS(reg_data, BMI2_OIS_IF_EN, enable);
            if (enable)
            {
                /* Disable auxiliary interface if OIS is enabled */
                reg_data = BMI2_SET_BIT_VAL0(reg_data, BMI2_AUX_IF_EN);
            }

            /* Set the OIS interface configurations */
            rslt = bmi2_set_regs(BMI2_IF_CONF_ADDR, &reg_data, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API can be used to write sync commands like ODR, sync period,
 * frequency and phase, resolution ratio, sync time and delay time.
 */
int8_t bmi2_write_sync_commands(const uint8_t *command, uint8_t n_comm, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (command != NULL))
    {
        rslt = bmi2_set_regs(BMI2_SYNC_COMMAND_ADDR, command, n_comm, dev);
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API performs self-test to check the proper functionality of the
 * accelerometer sensor.
 */
int8_t bmi2_perform_accel_self_test(struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store self-test result */
    int8_t st_rslt = 0;

    /* Structure to define positive accelerometer axes */
    struct bmi2_sens_axes_data positive = { 0, 0, 0, 0 };

    /* Structure to define negative accelerometer axes */
    struct bmi2_sens_axes_data negative = { 0, 0, 0, 0 };

    /* Structure for difference of accelerometer values in g */
    struct bmi2_selftest_delta_limit accel_data_diff = { 0, 0, 0 };

    /* Structure for difference of accelerometer values in mg */
    struct bmi2_selftest_delta_limit accel_data_diff_mg = { 0, 0, 0 };

    /* Initialize the polarity of self-test as positive */
    int8_t sign = BMI2_ENABLE;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Sets the configuration required before enabling self-test */
        rslt = pre_self_test_config(dev);

        /* Wait for greater than 2 milliseconds */
        dev->delay_us(3000, dev->intf_ptr);
        if (rslt == BMI2_OK)
        {
            do
            {
                /* Select positive first, then negative polarity
                 * after enabling self-test
                 */
                rslt = self_test_config((uint8_t) sign, dev);
                if (rslt == BMI2_OK)
                {
                    /* Wait for greater than 50 milli-sec */
                    dev->delay_us(51000, dev->intf_ptr);

                    /* If polarity is positive */
                    if (sign == BMI2_ENABLE)
                    {
                        /* Read and store positive acceleration value */
                        rslt = read_accel_xyz(&positive, dev);
                    }
                    /* If polarity is negative */
                    else if (sign == BMI2_DISABLE)
                    {
                        /* Read and store negative acceleration value */
                        rslt = read_accel_xyz(&negative, dev);
                    }
                }
                else
                {
                    /* Break if error */
                    break;
                }

                /* Break if error */
                if (rslt != BMI2_OK)
                {
                    break;
                }

                /* Turn the polarity of self-test negative */
                sign--;
            } while (sign >= 0);
            if (rslt == BMI2_OK)
            {
                /* Subtract -ve acceleration values from that of +ve values */
                accel_data_diff.x = (positive.x) - (negative.x);
                accel_data_diff.y = (positive.y) - (negative.y);
                accel_data_diff.z = (positive.z) - (negative.z);

                /* Convert differences of acceleration values
                 * from 'g' to 'mg'
                 */
                convert_lsb_g(&accel_data_diff, &accel_data_diff_mg, dev);

                /* Validate self-test for acceleration values
                 * in mg and get the self-test result
                 */
                st_rslt = validate_self_test(&accel_data_diff_mg);

                /* Trigger a soft reset after performing self-test */
                rslt = bmi2_soft_reset(dev);

                /* Return the self-test result */
                if (rslt == BMI2_OK)
                {
                    rslt = st_rslt;
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API maps/unmaps feature interrupts to that of interrupt pins.
 */
int8_t bmi2_map_feat_int(uint8_t type, enum bmi2_hw_int_pin hw_int_pin, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define the value of feature interrupts */
    uint8_t feat_int = 0;

    /* Array to store the interrupt mask bits */
    uint8_t data_array[2] = { 0 };

    /* Structure to define map the interrupts */
    struct bmi2_map_int map_int = { 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Read interrupt map1 and map2 and register */
        rslt = bmi2_get_regs(BMI2_INT1_MAP_FEAT_ADDR, data_array, 2, dev);

        if (rslt == BMI2_OK)
        {
            /* Get the value of the feature interrupt to be mapped */
            extract_feat_int_map(&map_int, type, dev);

            feat_int = map_int.sens_map_int;

            /* Map the interrupts */
            rslt = map_feat_int(data_array, hw_int_pin, feat_int);

            /* Map the interrupts to INT1 and INT2 map register */
            if (rslt == BMI2_OK)
            {
                rslt = bmi2_set_regs(BMI2_INT1_MAP_FEAT_ADDR, &data_array[0], 1, dev);
                if (rslt == BMI2_OK)
                {
                    rslt = bmi2_set_regs(BMI2_INT2_MAP_FEAT_ADDR, &data_array[1], 1, dev);
                }
            }
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API maps/un-maps data interrupts to that of interrupt pins.
 */
int8_t bmi2_map_data_int(uint8_t data_int, enum bmi2_hw_int_pin int_pin, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to mask interrupt pin 1 - lower nibble */
    uint8_t int1_mask = data_int;

    /* Variable to mask interrupt pin 2 - higher nibble */
    uint8_t int2_mask = (uint8_t)(data_int << 4);

    /* Variable to store register data */
    uint8_t reg_data = 0;

    /* Read interrupt map1 and map2 and register */
    rslt = bmi2_get_regs(BMI2_INT_MAP_DATA_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        if (int_pin < BMI2_INT_PIN_MAX)
        {
            switch (int_pin)
            {
                case BMI2_INT_NONE:

                    /* Un-Map the corresponding data
                     * interrupt to both interrupt pin 1 and 2
                     */
                    reg_data &= ~(int1_mask | int2_mask);
                    break;
                case BMI2_INT1:

                    /* Map the corresponding data interrupt to
                     * interrupt pin 1
                     */
                    reg_data |= int1_mask;
                    break;
                case BMI2_INT2:

                    /* Map the corresponding data interrupt to
                     * interrupt pin 2
                     */
                    reg_data |= int2_mask;
                    break;
                case BMI2_INT_BOTH:

                    /* Map the corresponding data
                     * interrupt to both interrupt pin 1 and 2
                     */
                    reg_data |= (int1_mask | int2_mask);
                    break;
                default:
                    break;
            }

            /* Set the interrupts in the map register */
            rslt = bmi2_set_regs(BMI2_INT_MAP_DATA_ADDR, &reg_data, 1, dev);
        }
        else
        {
            /* Return error if invalid pin selection */
            rslt = BMI2_E_INVALID_INT_PIN;
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the re-mapped x, y and z axes from the sensor and
 * updates the values in the device structure.
 */
int8_t bmi2_get_remap_axes(struct bmi2_remap *remapped_axis, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Initialize the local structure for axis re-mapping */
    struct bmi2_axes_remap remap = { 0, 0, 0, 0, 0, 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (remapped_axis != NULL))
    {
        /* Get the re-mapped axes from the sensor */
        rslt = get_remap_axes(&remap, dev);
        if (rslt == BMI2_OK)
        {
            /* Store the re-mapped x-axis value in device structure
             * and its user-value in the interface structure
             */
            switch (remap.x_axis)
            {
                case BMI2_MAP_X_AXIS:

                    /* If mapped to x-axis */
                    dev->remap.x_axis = BMI2_MAP_X_AXIS;
                    remapped_axis->x = BMI2_X;
                    break;
                case BMI2_MAP_Y_AXIS:

                    /* If mapped to y-axis */
                    dev->remap.x_axis = BMI2_MAP_Y_AXIS;
                    remapped_axis->x = BMI2_Y;
                    break;
                case BMI2_MAP_Z_AXIS:

                    /* If mapped to z-axis */
                    dev->remap.x_axis = BMI2_MAP_Z_AXIS;
                    remapped_axis->x = BMI2_Z;
                    break;
                default:
                    break;
            }

            /* Store the re-mapped x-axis sign in device structure
             * and its user-value in the interface structure
             */
            if (remap.x_axis_sign)
            {
                /* If x-axis is mapped to -ve sign */
                dev->remap.x_axis_sign = BMI2_NEG_SIGN;
                remapped_axis->x |= BMI2_AXIS_SIGN;
            }
            else
            {
                dev->remap.x_axis_sign = BMI2_POS_SIGN;
            }

            /* Store the re-mapped y-axis value in device structure
             *  and its user-value in the interface structure
             */
            switch (remap.y_axis)
            {
                case BMI2_MAP_X_AXIS:

                    /* If mapped to x-axis */
                    dev->remap.y_axis = BMI2_MAP_X_AXIS;
                    remapped_axis->y = BMI2_X;
                    break;
                case BMI2_MAP_Y_AXIS:

                    /* If mapped to y-axis */
                    dev->remap.y_axis = BMI2_MAP_Y_AXIS;
                    remapped_axis->y = BMI2_Y;
                    break;
                case BMI2_MAP_Z_AXIS:

                    /* If mapped to z-axis */
                    dev->remap.y_axis = BMI2_MAP_Z_AXIS;
                    remapped_axis->y = BMI2_Z;
                    break;
                default:
                    break;
            }

            /* Store the re-mapped y-axis sign in device structure
             * and its user-value in the interface structure
             */
            if (remap.y_axis_sign)
            {
                /* If y-axis is mapped to -ve sign */
                dev->remap.y_axis_sign = BMI2_NEG_SIGN;
                remapped_axis->y |= BMI2_AXIS_SIGN;
            }
            else
            {
                dev->remap.y_axis_sign = BMI2_POS_SIGN;
            }

            /* Store the re-mapped z-axis value in device structure
             *  and its user-value in the interface structure
             */
            switch (remap.z_axis)
            {
                case BMI2_MAP_X_AXIS:

                    /* If mapped to x-axis */
                    dev->remap.z_axis = BMI2_MAP_X_AXIS;
                    remapped_axis->z = BMI2_X;
                    break;
                case BMI2_MAP_Y_AXIS:

                    /* If mapped to y-axis */
                    dev->remap.z_axis = BMI2_MAP_Y_AXIS;
                    remapped_axis->z = BMI2_Y;
                    break;
                case BMI2_MAP_Z_AXIS:

                    /* If mapped to z-axis */
                    dev->remap.z_axis = BMI2_MAP_Z_AXIS;
                    remapped_axis->z = BMI2_Z;
                    break;
                default:
                    break;
            }

            /* Store the re-mapped z-axis sign in device structure
             * and its user-value in the interface structure
             */
            if (remap.z_axis_sign)
            {
                /* If z-axis is mapped to -ve sign */
                dev->remap.z_axis_sign = BMI2_NEG_SIGN;
                remapped_axis->z |= BMI2_AXIS_SIGN;
            }
            else
            {
                dev->remap.z_axis_sign = BMI2_POS_SIGN;
            }
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the re-mapped x, y and z axes to the sensor and
 * updates the them in the device structure.
 */
int8_t bmi2_set_remap_axes(const struct bmi2_remap *remapped_axis, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store all the re-mapped axes */
    uint8_t remap_axes = 0;

    /* Variable to store the re-mapped x-axes */
    uint8_t remap_x = 0;

    /* Variable to store the re-mapped y-axes */
    uint8_t remap_y = 0;

    /* Variable to store the re-mapped z-axes */
    uint8_t remap_z = 0;

    /* Initialize the local structure for axis re-mapping */
    struct bmi2_axes_remap remap = { 0, 0, 0, 0, 0, 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (remapped_axis != NULL))
    {
        /* Check whether all the axes are re-mapped */
        remap_axes = remapped_axis->x | remapped_axis->y | remapped_axis->z;

        /* If all the axes are re-mapped */
        if ((remap_axes & BMI2_AXIS_MASK) == BMI2_AXIS_MASK)
        {
            /* Get the re-mapped value of x, y and z axis */
            remap_x = remapped_axis->x & BMI2_AXIS_MASK;
            remap_y = remapped_axis->y & BMI2_AXIS_MASK;
            remap_z = remapped_axis->z & BMI2_AXIS_MASK;

            /* Store the value of re-mapped x-axis in both
             * device structure and the local structure
             */
            switch (remap_x)
            {
                case BMI2_X:

                    /* If mapped to x-axis */
                    dev->remap.x_axis = BMI2_MAP_X_AXIS;
                    remap.x_axis = BMI2_MAP_X_AXIS;
                    break;
                case BMI2_Y:

                    /* If mapped to y-axis */
                    dev->remap.x_axis = BMI2_MAP_Y_AXIS;
                    remap.x_axis = BMI2_MAP_Y_AXIS;
                    break;
                case BMI2_Z:

                    /* If mapped to z-axis */
                    dev->remap.x_axis = BMI2_MAP_Z_AXIS;
                    remap.x_axis = BMI2_MAP_Z_AXIS;
                    break;
                default:
                    break;
            }

            /* Store the re-mapped x-axis sign in the device
             * structure and its value in local structure
             */
            if (remapped_axis->x & BMI2_AXIS_SIGN)
            {
                /* If x-axis is mapped to -ve sign */
                dev->remap.x_axis_sign = BMI2_NEG_SIGN;
                remap.x_axis_sign = BMI2_MAP_NEGATIVE;
            }
            else
            {
                dev->remap.x_axis_sign = BMI2_POS_SIGN;
                remap.x_axis_sign = BMI2_MAP_POSITIVE;
            }

            /* Store the value of re-mapped y-axis in both
             * device structure and the local structure
             */
            switch (remap_y)
            {
                case BMI2_X:

                    /* If mapped to x-axis */
                    dev->remap.y_axis = BMI2_MAP_X_AXIS;
                    remap.y_axis = BMI2_MAP_X_AXIS;
                    break;
                case BMI2_Y:

                    /* If mapped to y-axis */
                    dev->remap.y_axis = BMI2_MAP_Y_AXIS;
                    remap.y_axis = BMI2_MAP_Y_AXIS;
                    break;
                case BMI2_Z:

                    /* If mapped to z-axis */
                    dev->remap.y_axis = BMI2_MAP_Z_AXIS;
                    remap.y_axis = BMI2_MAP_Z_AXIS;
                    break;
                default:
                    break;
            }

            /* Store the re-mapped y-axis sign in the device
             * structure and its value in local structure
             */
            if (remapped_axis->y & BMI2_AXIS_SIGN)
            {
                /* If y-axis is mapped to -ve sign */
                dev->remap.y_axis_sign = BMI2_NEG_SIGN;
                remap.y_axis_sign = BMI2_MAP_NEGATIVE;
            }
            else
            {
                dev->remap.y_axis_sign = BMI2_POS_SIGN;
                remap.y_axis_sign = BMI2_MAP_POSITIVE;
            }

            /* Store the value of re-mapped z-axis in both
             * device structure and the local structure
             */
            switch (remap_z)
            {
                case BMI2_X:

                    /* If mapped to x-axis */
                    dev->remap.z_axis = BMI2_MAP_X_AXIS;
                    remap.z_axis = BMI2_MAP_X_AXIS;
                    break;
                case BMI2_Y:

                    /* If mapped to y-axis */
                    dev->remap.z_axis = BMI2_MAP_Y_AXIS;
                    remap.z_axis = BMI2_MAP_Y_AXIS;
                    break;
                case BMI2_Z:

                    /* If mapped to z-axis */
                    dev->remap.z_axis = BMI2_MAP_Z_AXIS;
                    remap.z_axis = BMI2_MAP_Z_AXIS;
                    break;
                default:
                    break;
            }

            /* Store the re-mapped z-axis sign in the device
             * structure and its value in local structure
             */
            if (remapped_axis->z & BMI2_AXIS_SIGN)
            {
                /* If z-axis is mapped to -ve sign */
                dev->remap.z_axis_sign = BMI2_NEG_SIGN;
                remap.z_axis_sign = BMI2_MAP_NEGATIVE;
            }
            else
            {
                dev->remap.z_axis_sign = BMI2_POS_SIGN;
                remap.z_axis_sign = BMI2_MAP_POSITIVE;
            }

            /* Set the re-mapped axes in the sensor */
            rslt = set_remap_axes(&remap, dev);
        }
        else
        {
            rslt = BMI2_E_REMAP_ERROR;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API enables/disables gyroscope offset compensation. It adds the
 * offsets defined in the offset register with gyroscope data.
 */
int8_t bmi2_set_gyro_offset_comp(uint8_t enable, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define register data */
    uint8_t reg_data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Get the status of gyroscope offset enable */
        rslt = bmi2_get_regs(BMI2_GYR_OFF_COMP_6_ADDR, &reg_data, 1, dev);
        if (rslt == BMI2_OK)
        {
            reg_data = BMI2_SET_BITS(reg_data, BMI2_GYR_OFF_COMP_EN, enable);

            /* Enable/Disable gyroscope offset compensation */
            rslt = bmi2_set_regs(BMI2_GYR_OFF_COMP_6_ADDR, &reg_data, 1, dev);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the gyroscope bias values for each axis which is used
 * for gyroscope offset compensation.
 */
int8_t bmi2_read_gyro_offset_comp_axes(struct bmi2_sens_axes_data *gyr_off_comp_axes, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define register data */
    uint8_t reg_data[4] = { 0 };

    /* Variable to store LSB value of offset compensation for x-axis */
    uint8_t gyr_off_lsb_x;

    /* Variable to store LSB value of offset compensation for y-axis */
    uint8_t gyr_off_lsb_y;

    /* Variable to store LSB value of offset compensation for z-axis */
    uint8_t gyr_off_lsb_z;

    /* Variable to store MSB value of offset compensation for x-axis */
    uint8_t gyr_off_msb_x;

    /* Variable to store MSB value of offset compensation for y-axis */
    uint8_t gyr_off_msb_y;

    /* Variable to store MSB value of offset compensation for z-axis */
    uint8_t gyr_off_msb_z;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (gyr_off_comp_axes != NULL))
    {
        /* Get the gyroscope compensated offset values */
        rslt = bmi2_get_regs(BMI2_GYR_OFF_COMP_3_ADDR, reg_data, 4, dev);
        if (rslt == BMI2_OK)
        {
            /* Get LSB and MSB values of offset compensation for
             * x, y and z axis
             */
            gyr_off_lsb_x = reg_data[0];
            gyr_off_lsb_y = reg_data[1];
            gyr_off_lsb_z = reg_data[2];
            gyr_off_msb_x = reg_data[3] & BMI2_GYR_OFF_COMP_MSB_X_MASK;
            gyr_off_msb_y = reg_data[3] & BMI2_GYR_OFF_COMP_MSB_Y_MASK;
            gyr_off_msb_z = reg_data[3] & BMI2_GYR_OFF_COMP_MSB_Z_MASK;

            /* Gyroscope offset compensation value for x-axis */
            gyr_off_comp_axes->x = (int16_t)(((uint16_t) gyr_off_msb_x << 8) | gyr_off_lsb_x);

            /* Gyroscope offset compensation value for y-axis */
            gyr_off_comp_axes->y = (int16_t)(((uint16_t) gyr_off_msb_y << 6) | gyr_off_lsb_y);

            /* Gyroscope offset compensation value for z-axis */
            gyr_off_comp_axes->z = (int16_t)(((uint16_t) gyr_off_msb_z << 4) | gyr_off_lsb_z);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes the gyroscope bias values for each axis which is used
 * for gyroscope offset compensation.
 */
int8_t bmi2_write_gyro_offset_comp_axes(const struct bmi2_sens_axes_data *gyr_off_comp_axes, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define register data */
    uint8_t reg_data[4] = { 0 };

    /* Variable to store MSB value of offset compensation for x-axis */
    uint8_t gyr_off_msb_x;

    /* Variable to store MSB value of offset compensation for y-axis */
    uint8_t gyr_off_msb_y;

    /* Variable to store MSB value of offset compensation for z-axis */
    uint8_t gyr_off_msb_z;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (gyr_off_comp_axes != NULL))
    {
        /* Get the MSB values of gyroscope compensated offset values */
        rslt = bmi2_get_regs(BMI2_GYR_OFF_COMP_6_ADDR, &reg_data[3], 1, dev);
        if (rslt == BMI2_OK)
        {
            /* Get MSB value of x-axis from user-input */
            gyr_off_msb_x = (uint8_t)((gyr_off_comp_axes->x & BMI2_GYR_OFF_COMP_MSB_MASK) >> 8);

            /* Get MSB value of y-axis from user-input */
            gyr_off_msb_y = (uint8_t)((gyr_off_comp_axes->y & BMI2_GYR_OFF_COMP_MSB_MASK) >> 8);

            /* Get MSB value of z-axis from user-input */
            gyr_off_msb_z = (uint8_t)((gyr_off_comp_axes->z & BMI2_GYR_OFF_COMP_MSB_MASK) >> 8);

            /* Get LSB value of x-axis from user-input */
            reg_data[0] = (uint8_t)(gyr_off_comp_axes->x & BMI2_GYR_OFF_COMP_LSB_MASK);

            /* Get LSB value of y-axis from user-input */
            reg_data[1] = (uint8_t)(gyr_off_comp_axes->y & BMI2_GYR_OFF_COMP_LSB_MASK);

            /* Get LSB value of z-axis from user-input */
            reg_data[2] = (uint8_t)(gyr_off_comp_axes->z & BMI2_GYR_OFF_COMP_LSB_MASK);

            /* Get MSB value of x-axis to be set */
            reg_data[3] = BMI2_SET_BIT_POS0(reg_data[3], BMI2_GYR_OFF_COMP_MSB_X, gyr_off_msb_x);

            /* Get MSB value of y-axis to be set */
            reg_data[3] = BMI2_SET_BITS(reg_data[3], BMI2_GYR_OFF_COMP_MSB_Y, gyr_off_msb_y);

            /* Get MSB value of z-axis to be set */
            reg_data[3] = BMI2_SET_BITS(reg_data[3], BMI2_GYR_OFF_COMP_MSB_Z, gyr_off_msb_z);

            /* Set the offset compensation values of axes */
            rslt = bmi2_set_regs(BMI2_GYR_OFF_COMP_3_ADDR, reg_data, 4, dev);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API updates the cross sensitivity coefficient between gyroscope's
 * X and Z axes.
 */
int8_t bmi2_get_gyro_cross_sense(struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;
    struct bmi2_sensor_data data;

    /* Check if the feature is supported by this variant */
    if (dev->variant_feature & BMI2_GYRO_CROSS_SENS_ENABLE)
    {
        rslt = null_ptr_check(dev);
        if (rslt == BMI2_OK)
        {
            /* Select the feature whose data is to be acquired */
            data.type = BMI2_GYRO_CROSS_SENSE;

            /* Get the respective data */
            rslt = bmi2_get_sensor_data(&data, 1, dev);
            if (rslt == BMI2_OK)
            {
                /* Update the gyroscope cross sense value of z axis
                 * in the device structure
                 */
                dev->gyr_cross_sens_zx = data.sens_data.correction_factor_zx;
            }
        }
        else
        {
            rslt = BMI2_E_NULL_PTR;
        }
    }

    return rslt;
}

/*!
 * @brief This API gets Error bits and message indicating internal status.
 */
int8_t bmi2_get_internal_status(uint8_t *int_stat, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (int_stat != NULL))
    {
        /* Delay to read the internal status */
        dev->delay_us(20000, dev->intf_ptr);

        /* Get the error bits and message */
        rslt = bmi2_get_regs(BMI2_INTERNAL_STATUS_ADDR, int_stat, 1, dev);
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/*!
 * @brief This API verifies and allows only the correct position to do Fast Offset Compensation for
 * accelerometer & gyro.
 */
static int8_t verify_foc_position(uint8_t sens_list,
                                  const struct bmi2_accel_foc_g_value *accel_g_axis,
                                  struct bmi2_dev *dev)
{
    int8_t rslt;

    struct bmi2_sens_axes_data avg_foc_data = { 0 };
    struct bmi2_foc_temp_value temp_foc_data = { 0 };

    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Enable sensor */
        rslt = bmi2_sensor_enable(&sens_list, 1, dev);
    }

    if (rslt == BMI2_OK)
    {

        rslt = get_average_of_sensor_data(sens_list, &temp_foc_data, dev);
        if (rslt == BMI2_OK)
        {
            if (sens_list == BMI2_ACCEL)
            {

                /* Taking modulus to make negative values as positive */
                if ((accel_g_axis->x == 1) && (accel_g_axis->sign == 1))
                {
                    temp_foc_data.x = temp_foc_data.x * -1;
                }
                else if ((accel_g_axis->y == 1) && (accel_g_axis->sign == 1))
                {
                    temp_foc_data.y = temp_foc_data.y * -1;
                }
                else if ((accel_g_axis->z == 1) && (accel_g_axis->sign == 1))
                {
                    temp_foc_data.z = temp_foc_data.z * -1;
                }
            }

            /* Typecasting into 16bit */
            avg_foc_data.x = (int16_t)(temp_foc_data.x);
            avg_foc_data.y = (int16_t)(temp_foc_data.y);
            avg_foc_data.z = (int16_t)(temp_foc_data.z);

            rslt = validate_foc_position(sens_list, accel_g_axis, avg_foc_data, dev);
        }
    }

    return rslt;
}

/*! @endcond */

/*!
 * @brief This API performs Fast Offset Compensation for accelerometer.
 */
int8_t bmi2_perform_accel_foc(const struct bmi2_accel_foc_g_value *accel_g_value, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Structure to define the accelerometer configurations */
    struct bmi2_accel_config acc_cfg = { 0, 0, 0, 0 };

    /* Variable to store status of advance power save */
    uint8_t aps = 0;

    /* Variable to store status of accelerometer enable */
    uint8_t acc_en = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (accel_g_value != NULL))
    {
        /* Check for input validity */
        if ((((BMI2_ABS(accel_g_value->x)) + (BMI2_ABS(accel_g_value->y)) + (BMI2_ABS(accel_g_value->z))) == 1) &&
            ((accel_g_value->sign == 1) || (accel_g_value->sign == 0)))
        {
            rslt = verify_foc_position(BMI2_ACCEL, accel_g_value, dev);
            if (rslt == BMI2_OK)
            {

                /* Save accelerometer configurations, accelerometer
                 * enable status and advance power save status
                 */
                rslt = save_accel_foc_config(&acc_cfg, &aps, &acc_en, dev);
            }

            /* Set configurations for FOC */
            if (rslt == BMI2_OK)
            {
                rslt = set_accel_foc_config(dev);
            }

            /* Perform accelerometer FOC */
            if (rslt == BMI2_OK)
            {
                rslt = perform_accel_foc(accel_g_value, &acc_cfg, dev);
            }

            /* Restore the saved configurations */
            if (rslt == BMI2_OK)
            {
                rslt = restore_accel_foc_config(&acc_cfg, aps, acc_en, dev);
            }
        }
        else
        {
            rslt = BMI2_E_INVALID_INPUT;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API performs Fast Offset Compensation for gyroscope.
 */
int8_t bmi2_perform_gyro_foc(struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Structure to define the gyroscope configurations */
    struct bmi2_gyro_config gyr_cfg = { 0, 0, 0, 0, 0, 0 };

    /* Variable to store status of advance power save */
    uint8_t aps = 0;

    /* Variable to store status of gyroscope enable */
    uint8_t gyr_en = 0;

    /* Array of structure to store gyroscope data */
    struct bmi2_sens_axes_data gyr_value[128];

    /* Structure to store gyroscope data temporarily */
    struct bmi2_foc_temp_value temp = { 0, 0, 0 };

    /* Variable to store status read from the status register */
    uint8_t reg_status = 0;

    /* Variable to define count */
    uint8_t loop = 0;

    /* Structure to store the offset values to be stored in the register */
    struct bmi2_sens_axes_data gyro_offset = { 0, 0, 0, 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Argument2 is not applicable for gyro */
        rslt = verify_foc_position(BMI2_GYRO, 0, dev);
        if (rslt == BMI2_OK)
        {
            /* Save gyroscope configurations, gyroscope enable
             * status and advance power save status
             */
            rslt = save_gyro_config(&gyr_cfg, &aps, &gyr_en, dev);

            /* Set configurations for gyroscope FOC */
            if (rslt == BMI2_OK)
            {
                rslt = set_gyro_foc_config(dev);
            }

            /* Perform FOC */
            if (rslt == BMI2_OK)
            {
                for (loop = 0; loop < 128; loop++)
                {
                    /* Giving a delay of more than 40ms since ODR is configured as 25Hz */
                    dev->delay_us(50000, dev->intf_ptr);

                    /* Get gyroscope data ready interrupt status */
                    rslt = bmi2_get_status(&reg_status, dev);

                    /* Read 128 samples of gyroscope data on data ready interrupt */
                    if ((rslt == BMI2_OK) && (reg_status & BMI2_DRDY_GYR))
                    {
                        rslt = read_gyro_xyz(&gyr_value[loop], dev);
                        if (rslt == BMI2_OK)
                        {
                            /* Store the data in a temporary structure */
                            temp.x = temp.x + (int32_t)gyr_value[loop].x;
                            temp.y = temp.y + (int32_t)gyr_value[loop].y;
                            temp.z = temp.z + (int32_t)gyr_value[loop].z;
                        }
                    }

                    if (rslt != BMI2_OK)
                    {
                        break;
                    }
                    else if ((reg_status & BMI2_DRDY_GYR) != BMI2_DRDY_GYR)
                    {
                        rslt = BMI2_E_INVALID_STATUS;
                        break;
                    }
                }

                if (rslt == BMI2_OK)
                {
                    /* Take average of x, y and z data for lesser
                     * noise. It is same as offset data since lsb/dps
                     * is same for both data and offset register
                     */
                    gyro_offset.x = (int16_t)(temp.x / 128);
                    gyro_offset.y = (int16_t)(temp.y / 128);
                    gyro_offset.z = (int16_t)(temp.z / 128);

                    /* Saturate gyroscope data since the offset
                     * registers are of 10 bit value where as the
                     * gyroscope data is of 16 bit value
                     */
                    saturate_gyro_data(&gyro_offset);

                    /* Invert the gyroscope offset  data */
                    invert_gyro_offset(&gyro_offset);

                    /* Write offset data in the gyroscope offset
                     * compensation register
                     */
                    rslt = bmi2_write_gyro_offset_comp_axes(&gyro_offset, dev);
                }

                /* Enable gyroscope offset compensation */
                if (rslt == BMI2_OK)
                {
                    rslt = bmi2_set_gyro_offset_comp(BMI2_ENABLE, dev);
                }

                /* Restore the saved gyroscope configurations */
                if (rslt == BMI2_OK)
                {
                    rslt = restore_gyro_config(&gyr_cfg, aps, gyr_en, dev);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to get the feature configuration from the
 * selected page.
 */
int8_t bmi2_get_feat_config(uint8_t sw_page, uint8_t *feat_config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define bytes remaining to read */
    uint8_t bytes_remain = BMI2_FEAT_SIZE_IN_BYTES;

    /* Variable to define the read-write length */
    uint8_t read_write_len = 0;

    /* Variable to define the feature configuration address */
    uint8_t addr = BMI2_FEATURES_REG_ADDR;

    /* Variable to define index */
    uint8_t index = 0;

    if ((feat_config == NULL) || (dev == NULL))
    {
        rslt = BMI2_E_NULL_PTR;
    }
    else
    {
        /* Check whether the page is valid */
        if (sw_page < dev->page_max)
        {
            /* Switch page */
            rslt = bmi2_set_regs(BMI2_FEAT_PAGE_ADDR, &sw_page, 1, dev);

            /* If user length is less than feature length */
            if ((rslt == BMI2_OK) && (dev->read_write_len < BMI2_FEAT_SIZE_IN_BYTES))
            {
                /* Read-write should be even */
                if ((dev->read_write_len % 2) != 0)
                {
                    dev->read_write_len--;
                }

                while (bytes_remain > 0)
                {
                    if (bytes_remain >= dev->read_write_len)
                    {
                        /* Read from the page */
                        rslt = bmi2_get_regs(addr, &feat_config[index], dev->read_write_len, dev);

                        /* Update index */
                        index += (uint8_t) dev->read_write_len;

                        /* Update address */
                        addr += (uint8_t) dev->read_write_len;

                        /* Update read-write length */
                        read_write_len += (uint8_t) dev->read_write_len;
                    }
                    else
                    {
                        /* Read from the page */
                        rslt = bmi2_get_regs(addr, (uint8_t *) (feat_config + index), (uint16_t) bytes_remain, dev);

                        /* Update read-write length */
                        read_write_len += bytes_remain;
                    }

                    /* Remaining bytes */
                    bytes_remain = BMI2_FEAT_SIZE_IN_BYTES - read_write_len;

                    if (rslt != BMI2_OK)
                    {
                        break;
                    }
                }
            }
            else if (rslt == BMI2_OK)
            {
                /* Get configuration from the page */
                rslt = bmi2_get_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev);
            }
        }
        else
        {
            rslt = BMI2_E_INVALID_PAGE;
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to extract the input feature configuration
 * details from the look-up table.
 */
uint8_t bmi2_extract_input_feat_config(struct bmi2_feature_config *feat_config, uint8_t type,
                                       const struct bmi2_dev *dev)
{
    /* Variable to define loop */
    uint8_t loop = 0;

    /* Variable to set flag */
    uint8_t feat_found = BMI2_FALSE;

    /* Search for the input feature from the input configuration array */
    while (loop < dev->input_sens)
    {
        if (dev->feat_config[loop].type == type)
        {
            *feat_config = dev->feat_config[loop];
            feat_found = BMI2_TRUE;
            break;
        }

        loop++;
    }

    /* Return flag */
    return feat_found;
}

/***************************************************************************/

/*!         Local Function Definitions
 ****************************************************************************/

/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/*!
 * @brief This internal API writes the configuration file.
 */
static int8_t write_config_file(struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to update the configuration file index */
    uint16_t index = 0;

    /* config file size */
    uint16_t config_size = dev->config_size;

    /* Variable to get the remainder */
    uint8_t remain = (uint8_t)(config_size % dev->read_write_len);

    /* Variable to get the balance bytes */
    uint16_t bal_byte = 0;

    /* Variable to define temporary read/write length */
    uint16_t read_write_len = 0;

    /* Disable advanced power save mode */
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
    if (rslt == BMI2_OK)
    {
        /* Disable loading of the configuration */
        rslt = set_config_load(BMI2_DISABLE, dev);
        if (rslt == BMI2_OK)
        {
            if (!remain)
            {
                /* Write the configuration file */
                for (index = 0; (index < config_size) && (rslt == BMI2_OK); index += dev->read_write_len)
                {
                    rslt = upload_file((dev->config_file_ptr + index), index, dev->read_write_len, dev);
                }
            }
            else
            {
                /* Get the balance bytes */
                bal_byte = (uint16_t) config_size - (uint16_t) remain;

                /* Write the configuration file for the balancem bytes */
                for (index = 0; (index < bal_byte) && (rslt == BMI2_OK); index += dev->read_write_len)
                {
                    rslt = upload_file((dev->config_file_ptr + index), index, dev->read_write_len, dev);
                }

                if (rslt == BMI2_OK)
                {
                    /* Update length in a temporary variable */
                    read_write_len = dev->read_write_len;

                    /* Write the remaining bytes in 2 bytes length */
                    dev->read_write_len = 2;

                    /* Write the configuration file for the remaining bytes */
                    for (index = bal_byte;
                         (index < config_size) && (rslt == BMI2_OK);
                         index += dev->read_write_len)
                    {
                        rslt = upload_file((dev->config_file_ptr + index), index, dev->read_write_len, dev);
                    }

                    /* Restore the user set length back from the temporary variable */
                    dev->read_write_len = read_write_len;
                }
            }

            if (rslt == BMI2_OK)
            {
                /* Enable loading of the configuration */
                rslt = set_config_load(BMI2_ENABLE, dev);

                /* Wait till ASIC is initialized */
                dev->delay_us(150000, dev->intf_ptr);
                if (rslt == BMI2_OK)
                {
                    /* Enable advanced power save mode */
                    rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API enables/disables the loading of the configuration
 * file.
 */
static int8_t set_config_load(uint8_t enable, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data = 0;

    rslt = bmi2_get_regs(BMI2_INIT_CTRL_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        reg_data = BMI2_SET_BIT_POS0(reg_data, BMI2_CONF_LOAD_EN, enable);
        rslt = bmi2_set_regs(BMI2_INIT_CTRL_ADDR, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API loads the configuration file.
 */
static int8_t upload_file(const uint8_t *config_data, uint16_t index, uint16_t write_len, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store address */
    uint8_t addr_array[2] = { 0 };

    if (config_data != NULL)
    {
        /* Store 0 to 3 bits of address in first byte */
        addr_array[0] = (uint8_t)((index / 2) & 0x0F);

        /* Store 4 to 11 bits of address in the second byte */
        addr_array[1] = (uint8_t)((index / 2) >> 4);

        /* Write the 2 bytes of address in consecutive locations */
        rslt = bmi2_set_regs(BMI2_INIT_ADDR_0, addr_array, 2, dev);
        if (rslt == BMI2_OK)
        {
            /* Burst write configuration file data corresponding to user set length */
            rslt = bmi2_set_regs(BMI2_INIT_DATA_ADDR, (uint8_t *)config_data, write_len, dev);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API validates bandwidth and performance mode of the
 * accelerometer set by the user.
 */
static int8_t validate_bw_perf_mode(uint8_t *bandwidth, uint8_t *perf_mode, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Validate and auto-correct performance mode */
    rslt = check_boundary_val(perf_mode, BMI2_POWER_OPT_MODE, BMI2_PERF_OPT_MODE, dev);
    if (rslt == BMI2_OK)
    {
        /* Validate and auto-correct bandwidth parameter */
        if (*perf_mode == BMI2_PERF_OPT_MODE)
        {
            /* Validate for continuous filter mode */
            rslt = check_boundary_val(bandwidth, BMI2_ACC_OSR4_AVG1, BMI2_ACC_CIC_AVG8, dev);
        }
        else
        {
            /* Validate for CIC averaging mode */
            rslt = check_boundary_val(bandwidth, BMI2_ACC_OSR4_AVG1, BMI2_ACC_RES_AVG128, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API validates ODR and range of the accelerometer set by
 * the user.
 */
static int8_t validate_odr_range(uint8_t *odr, uint8_t *range, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Validate and auto correct ODR */
    rslt = check_boundary_val(odr, BMI2_ACC_ODR_0_78HZ, BMI2_ACC_ODR_1600HZ, dev);
    if (rslt == BMI2_OK)
    {
        /* Validate and auto correct Range */
        rslt = check_boundary_val(range, BMI2_ACC_RANGE_2G, BMI2_ACC_RANGE_16G, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API validates bandwidth, performance mode, low power/
 * high performance mode, ODR, and range set by the user.
 */
static int8_t validate_gyro_config(struct bmi2_gyro_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Validate and auto-correct performance mode */
    rslt = check_boundary_val(&config->filter_perf, BMI2_POWER_OPT_MODE, BMI2_PERF_OPT_MODE, dev);
    if (rslt == BMI2_OK)
    {
        /* Validate and auto-correct bandwidth parameter */
        rslt = check_boundary_val(&config->bwp, BMI2_GYR_OSR4_MODE, BMI2_GYR_CIC_MODE, dev);
        if (rslt == BMI2_OK)
        {
            /* Validate and auto-correct low power/high-performance parameter */
            rslt = check_boundary_val(&config->noise_perf, BMI2_POWER_OPT_MODE, BMI2_PERF_OPT_MODE, dev);
            if (rslt == BMI2_OK)
            {
                /* Validate and auto-correct ODR parameter */
                rslt = check_boundary_val(&config->odr, BMI2_GYR_ODR_25HZ, BMI2_GYR_ODR_3200HZ, dev);
                if (rslt == BMI2_OK)
                {
                    /* Validate and auto-correct OIS range */
                    rslt = check_boundary_val(&config->ois_range, BMI2_GYR_OIS_250, BMI2_GYR_OIS_2000, dev);
                    if (rslt == BMI2_OK)
                    {
                        /* Validate and auto-correct range parameter */
                        rslt = check_boundary_val(&config->range, BMI2_GYR_RANGE_2000, BMI2_GYR_RANGE_125, dev);
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API shows the error status when illegal sensor
 * configuration is set.
 */
static int8_t cfg_error_status(struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data;

    /* Get error status of the set sensor configuration */
    rslt = bmi2_get_regs(BMI2_EVENT_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        reg_data = BMI2_GET_BITS(reg_data, BMI2_EVENT_FLAG);
        switch (reg_data)
        {
            case BMI2_NO_ERROR:
                rslt = BMI2_OK;
                break;
            case BMI2_ACC_ERROR:
                rslt = BMI2_E_ACC_INVALID_CFG;
                break;
            case BMI2_GYR_ERROR:
                rslt = BMI2_E_GYRO_INVALID_CFG;
                break;
            case BMI2_ACC_GYR_ERROR:
                rslt = BMI2_E_ACC_GYR_INVALID_CFG;
                break;
            default:
                break;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API:
 * 1) Enables/Disables auxiliary interface.
 * 2) Sets auxiliary interface configurations like I2C address, manual/auto
 * mode enable, manual burst read length, AUX burst read length and AUX read
 * address.
 * 3)It maps/un-maps data interrupts to that of hardware interrupt line.
 */
static int8_t set_aux_config(struct bmi2_aux_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Validate auxiliary configurations */
    rslt = validate_aux_config(config, dev);
    if (rslt == BMI2_OK)
    {
        /* Enable/Disable auxiliary interface */
        rslt = set_aux_interface(config, dev);
        if (rslt == BMI2_OK)
        {
            /* Set the auxiliary interface configurations */
            rslt = config_aux_interface(config, dev);
            if (rslt == BMI2_OK)
            {
                /* Set read out offset and ODR */
                rslt = config_aux(config, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API sets gyroscope user-gain configurations like gain
 * update value for x, y and z-axis.
 */
static int8_t set_gyro_user_gain_config(const struct bmi2_gyro_user_gain_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };

    /* Variable to define the array offset */
    uint8_t idx = 0;

    /* Variable to define index */
    uint8_t index = 0;

    /* Variable to set flag */
    uint8_t feat_found;

    /* Initialize feature configuration for user-gain */
    struct bmi2_feature_config user_gain_config = { 0, 0, 0 };

    /* Copy the feature configuration address to a local pointer */
    uint16_t *data_p = (uint16_t *) (void *)feat_config;

    /* Search for user-gain feature and extract its configuration details */
    feat_found = bmi2_extract_input_feat_config(&user_gain_config, BMI2_GYRO_GAIN_UPDATE, dev);
    if (feat_found)
    {
        /* Get the configuration from the page where user-gain feature resides */
        rslt = bmi2_get_feat_config(user_gain_config.page, feat_config, dev);
        if (rslt == BMI2_OK)
        {
            /* Define the offset in bytes for user-gain select */
            idx = user_gain_config.start_addr;

            /* Get offset in words since all the features are set in words length */
            idx = idx / 2;

            /* Set ratio_x */
            *(data_p + idx) = BMI2_SET_BIT_POS0(*(data_p + idx), BMI2_GYR_USER_GAIN_RATIO_X, config->ratio_x);

            /* Increment offset by 1 word to set ratio_y */
            idx++;

            /* Set ratio_y */
            *(data_p + idx) = BMI2_SET_BIT_POS0(*(data_p + idx), BMI2_GYR_USER_GAIN_RATIO_Y, config->ratio_y);

            /* Increment offset by 1 word to set ratio_z */
            idx++;

            /* Set ratio_z */
            *(data_p + idx) = BMI2_SET_BIT_POS0(*(data_p + idx), BMI2_GYR_USER_GAIN_RATIO_Z, config->ratio_z);

            /* Increment offset by 1 more word to get the total length in words */
            idx++;

            /* Get total length in bytes to copy from local pointer to the array */
            idx = (uint8_t)(idx * 2) - user_gain_config.start_addr;

            /* Copy the bytes to be set back to the array */
            for (index = 0; index < idx; index++)
            {
                feat_config[user_gain_config.start_addr +
                            index] = *((uint8_t *) data_p + user_gain_config.start_addr + index);
            }

            /* Set the configuration back to the page */
            rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev);
        }
    }
    else
    {
        rslt = BMI2_E_INVALID_SENSOR;
    }

    return rslt;
}

/*!
 * @brief This internal API enables/disables auxiliary interface.
 */
static int8_t set_aux_interface(const struct bmi2_aux_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data;

    rslt = bmi2_get_regs(BMI2_IF_CONF_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        reg_data = BMI2_SET_BITS(reg_data, BMI2_AUX_IF_EN, config->aux_en);

        /* Enable/Disable auxiliary interface */
        rslt = bmi2_set_regs(BMI2_IF_CONF_ADDR, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API sets auxiliary configurations like manual/auto mode
 * FCU write command enable and read burst length for both data and manual mode.
 *
 * @note Auxiliary sensor should not be busy when configuring aux_i2c_addr,
 * man_rd_burst_len, aux_rd_burst_len and aux_rd_addr.
 */
static int8_t config_aux_interface(const struct bmi2_aux_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data[2] = { 0 };

    /* Variable to store status */
    uint8_t status = 0;

    /* Variable to define count */
    uint8_t count = 0;

    rslt = bmi2_get_regs(BMI2_AUX_DEV_ID_ADDR, reg_data, 2, dev);
    if (rslt == BMI2_OK)
    {
        /* Set I2C address for AUX sensor */
        reg_data[0] = BMI2_SET_BITS(reg_data[0], BMI2_AUX_SET_I2C_ADDR, config->i2c_device_addr);

        /* Set the AUX IF to either manual or auto mode */
        reg_data[1] = BMI2_SET_BITS(reg_data[1], BMI2_AUX_MAN_MODE_EN, config->manual_en);

        /* Enables FCU write command on AUX IF for auxiliary sensors that need a trigger */
        reg_data[1] = BMI2_SET_BITS(reg_data[1], BMI2_AUX_FCU_WR_EN, config->fcu_write_en);

        /* Set the burst read length for manual mode */
        reg_data[1] = BMI2_SET_BITS(reg_data[1], BMI2_AUX_MAN_READ_BURST, config->man_rd_burst);

        /* Set the burst read length for data mode */
        reg_data[1] = BMI2_SET_BIT_POS0(reg_data[1], BMI2_AUX_READ_BURST, config->aux_rd_burst);
        for (;;)
        {
            /* Check if auxiliary sensor is busy */
            rslt = bmi2_get_status(&status, dev);
            if ((rslt == BMI2_OK) && (!(status & BMI2_AUX_BUSY)))
            {
                /* Set the configurations if AUX is not busy */
                rslt = bmi2_set_regs(BMI2_AUX_DEV_ID_ADDR, reg_data, 2, dev);
                dev->delay_us(1000, dev->intf_ptr);
                if (rslt == BMI2_OK)
                {
                    /* If data mode */
                    if (!config->manual_en)
                    {
                        /* Disable manual enable flag in device structure */
                        dev->aux_man_en = 0;

                        /* Set the read address of the AUX sensor */
                        rslt = bmi2_set_regs(BMI2_AUX_RD_ADDR, (uint8_t *) &config->read_addr, 1, dev);
                        dev->delay_us(1000, dev->intf_ptr);
                    }
                    else
                    {
                        /* Enable manual enable flag in device structure */
                        dev->aux_man_en = 1;

                        /* Update manual read burst length in device structure */
                        dev->aux_man_rd_burst_len = config->man_rd_burst;
                    }
                }

                /* Break after setting the register */
                break;
            }

            /* Increment count after every 10 seconds */
            dev->delay_us(10000, dev->intf_ptr);
            count++;

            /* Break after 2 seconds if AUX still busy - since slowest ODR is 0.78Hz*/
            if (count > 20)
            {
                rslt = BMI2_E_AUX_BUSY;
                break;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API triggers read out offset and sets ODR of the
 * auxiliary sensor.
 */
static int8_t config_aux(const struct bmi2_aux_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data;

    rslt = bmi2_get_regs(BMI2_AUX_CONF_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        /* Trigger read out offset */
        reg_data = BMI2_SET_BITS(reg_data, BMI2_AUX_OFFSET_READ_OUT, config->offset);

        /* Set ODR */
        reg_data = BMI2_SET_BIT_POS0(reg_data, BMI2_AUX_ODR_EN, config->odr);

        /* Set auxiliary configuration register */
        rslt = bmi2_set_regs(BMI2_AUX_CONF_ADDR, &reg_data, 1, dev);
        dev->delay_us(1000, dev->intf_ptr);
    }

    return rslt;
}

/*!
 * @brief This internal API checks the busy status of auxiliary sensor and sets
 * the auxiliary register addresses when not busy.
 */
static int8_t set_if_aux_not_busy(uint8_t reg_addr, uint8_t reg_data, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to get status of AUX_BUSY */
    uint8_t status = 0;

    /* Variable to define count for time-out */
    uint8_t count = 0;

    for (;;)
    {
        /* Check if AUX is busy */
        rslt = bmi2_get_status(&status, dev);

        /* Set the registers if not busy */
        if ((rslt == BMI2_OK) && (!(status & BMI2_AUX_BUSY)))
        {
            rslt = bmi2_set_regs(reg_addr, &reg_data, 1, dev);
            dev->delay_us(1000, dev->intf_ptr);

            /* Break after setting the register */
            break;
        }

        /* Increment count after every 10 seconds */
        dev->delay_us(10000, dev->intf_ptr);
        count++;

        /* Break after 2 seconds if AUX still busy - since slowest ODR is 0.78Hz*/
        if (count > 20)
        {
            rslt = BMI2_E_AUX_BUSY;
            break;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API validates auxiliary configuration set by the user.
 */
static int8_t validate_aux_config(struct bmi2_aux_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Validate ODR for auxiliary sensor */
    rslt = check_boundary_val(&config->odr, BMI2_AUX_ODR_0_78HZ, BMI2_AUX_ODR_800HZ, dev);

    return rslt;
}

/*!
 * @brief This internal API gets accelerometer configurations like ODR,
 * bandwidth, performance mode and g-range.
 */
static int8_t get_accel_config(struct bmi2_accel_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store data */
    uint8_t data_array[2] = { 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (config != NULL))
    {
        /* Read the sensor configuration details */
        rslt = bmi2_get_regs(BMI2_ACC_CONF_ADDR, data_array, 2, dev);
        if (rslt == BMI2_OK)
        {
            /* Get accelerometer performance mode */
            config->filter_perf = BMI2_GET_BITS(data_array[0], BMI2_ACC_FILTER_PERF_MODE);

            /* Get accelerometer bandwidth */
            config->bwp = BMI2_GET_BITS(data_array[0], BMI2_ACC_BW_PARAM);

            /* Get accelerometer ODR */
            config->odr = BMI2_GET_BIT_POS0(data_array[0], BMI2_ACC_ODR);

            /* Get accelerometer range */
            config->range = BMI2_GET_BIT_POS0(data_array[1], BMI2_ACC_RANGE);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API gets gyroscope configurations like ODR, bandwidth,
 * low power/high performance mode, performance mode and range.
 */
static int8_t get_gyro_config(struct bmi2_gyro_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store data */
    uint8_t data_array[2] = { 0 };

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (config != NULL))
    {
        /* Read the sensor configuration details */
        rslt = bmi2_get_regs(BMI2_GYR_CONF_ADDR, data_array, 2, dev);
        if (rslt == BMI2_OK)
        {
            /* Get gyroscope performance mode */
            config->filter_perf = BMI2_GET_BITS(data_array[0], BMI2_GYR_FILTER_PERF_MODE);

            /* Get gyroscope noise performance mode */
            config->noise_perf = BMI2_GET_BITS(data_array[0], BMI2_GYR_NOISE_PERF_MODE);

            /* Get gyroscope bandwidth */
            config->bwp = BMI2_GET_BITS(data_array[0], BMI2_GYR_BW_PARAM);

            /* Get gyroscope ODR */
            config->odr = BMI2_GET_BIT_POS0(data_array[0], BMI2_GYR_ODR);

            /* Get gyroscope OIS range */
            config->ois_range = BMI2_GET_BITS(data_array[1], BMI2_GYR_OIS_RANGE);

            /* Get gyroscope range */
            config->range = BMI2_GET_BIT_POS0(data_array[1], BMI2_GYR_RANGE);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API:
 * 1) Gets the status of auxiliary interface enable.
 * 2) Gets auxiliary interface configurations like I2C address, manual/auto
 * mode enable, manual burst read length, AUX burst read length and AUX read
 * address.
 * 3) Gets ODR and offset.
 */
static int8_t get_aux_config(struct bmi2_aux_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI2_OK) && (config != NULL))
    {
        /* Get enable status of auxiliary interface */
        rslt = get_aux_interface(config, dev);
        if (rslt == BMI2_OK)
        {
            /* Get the auxiliary interface configurations */
            rslt = get_aux_interface_config(config, dev);
            if (rslt == BMI2_OK)
            {
                /* Get read out offset and ODR */
                rslt = get_aux_cfg(config, dev);
            }
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API gets gyroscope user-gain configurations like gain
 * update value for x, y and z-axis.
 */
static int8_t get_gyro_gain_update_config(struct bmi2_gyro_user_gain_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };

    /* Variable to define the array offset */
    uint8_t idx = 0;

    /* Variable to define LSB */
    uint16_t lsb = 0;

    /* Variable to define MSB */
    uint16_t msb = 0;

    /* Variable to define a word */
    uint16_t lsb_msb = 0;

    /* Variable to set flag */
    uint8_t feat_found;

    /* Initialize feature configuration for user-gain */
    struct bmi2_feature_config user_gain_config = { 0, 0, 0 };

    /* Search for user-gain feature and extract its configuration details */
    feat_found = bmi2_extract_input_feat_config(&user_gain_config, BMI2_GYRO_GAIN_UPDATE, dev);
    if (feat_found)
    {
        /* Get the configuration from the page where user-gain feature resides */
        rslt = bmi2_get_feat_config(user_gain_config.page, feat_config, dev);
        if (rslt == BMI2_OK)
        {
            /* Define the offset in bytes for user-gain select */
            idx = user_gain_config.start_addr;

            /* Get word to calculate ratio_x */
            lsb = (uint16_t) feat_config[idx++];
            msb = ((uint16_t) feat_config[idx++] << 8);
            lsb_msb = lsb | msb;

            /* Get ratio_x */
            config->ratio_x = lsb_msb & BMI2_GYR_USER_GAIN_RATIO_X_MASK;

            /* Get word to calculate ratio_y */
            lsb = (uint16_t) feat_config[idx++];
            msb = ((uint16_t) feat_config[idx++] << 8);
            lsb_msb = lsb | msb;

            /* Get ratio_y */
            config->ratio_y = lsb_msb & BMI2_GYR_USER_GAIN_RATIO_Y_MASK;

            /* Get word to calculate ratio_z */
            lsb = (uint16_t) feat_config[idx++];
            msb = ((uint16_t) feat_config[idx++] << 8);
            lsb_msb = lsb | msb;

            /* Get ratio_z */
            config->ratio_z = lsb_msb & BMI2_GYR_USER_GAIN_RATIO_Z_MASK;
        }
    }
    else
    {
        rslt = BMI2_E_INVALID_SENSOR;
    }

    return rslt;
}

/*!
 * @brief This internal API gets the enable status of auxiliary interface.
 */
static int8_t get_aux_interface(struct bmi2_aux_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data;

    /* Get the enable status of auxiliary interface */
    rslt = bmi2_get_regs(BMI2_IF_CONF_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        config->aux_en = BMI2_GET_BITS(reg_data, BMI2_AUX_IF_EN);
    }

    return rslt;
}

/*!
 * @brief This internal API gets auxiliary configurations like manual/auto mode
 * FCU write command enable and read burst length for both data and manual mode.
 */
static int8_t get_aux_interface_config(struct bmi2_aux_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data[2] = { 0 };

    rslt = bmi2_get_regs(BMI2_AUX_DEV_ID_ADDR, reg_data, 2, dev);
    if (rslt == BMI2_OK)
    {
        /* Get I2C address for auxiliary sensor */
        config->i2c_device_addr = BMI2_GET_BITS(reg_data[0], BMI2_AUX_SET_I2C_ADDR);

        /* Get the AUX IF to either manual or auto mode */
        config->manual_en = BMI2_GET_BITS(reg_data[1], BMI2_AUX_MAN_MODE_EN);

        /* Enables FCU write command on AUX IF for auxiliary sensors that need a trigger */
        config->fcu_write_en = BMI2_GET_BITS(reg_data[1], BMI2_AUX_FCU_WR_EN);

        /* Get the burst read length for manual mode */
        config->man_rd_burst = BMI2_GET_BITS(reg_data[1], BMI2_AUX_MAN_READ_BURST);

        /* Get the burst read length for data mode */
        config->aux_rd_burst = BMI2_GET_BIT_POS0(reg_data[1], BMI2_AUX_READ_BURST);

        /* If data mode, get the read address of the auxiliary sensor from where data is to be read */
        if (!config->manual_en)
        {
            rslt = bmi2_get_regs(BMI2_AUX_RD_ADDR, &config->read_addr, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API gets read out offset and ODR of the auxiliary
 * sensor.
 */
static int8_t get_aux_cfg(struct bmi2_aux_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data;

    rslt = bmi2_get_regs(BMI2_AUX_CONF_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        /* Get read out offset */
        config->offset = BMI2_GET_BITS(reg_data, BMI2_AUX_OFFSET_READ_OUT);

        /* Get ODR */
        config->odr = BMI2_GET_BIT_POS0(reg_data, BMI2_AUX_ODR_EN);
    }

    return rslt;
}

/*!
 * @brief This internal API maps/un-maps feature interrupts to that of interrupt
 * pins.
 */
static int8_t map_feat_int(uint8_t *reg_data_array, enum bmi2_hw_int_pin int_pin, uint8_t int_mask)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Check for NULL error */
    if (reg_data_array != NULL)
    {
        /* Check validity on interrupt pin selection */
        if (int_pin < BMI2_INT_PIN_MAX)
        {
            switch (int_pin)
            {
                case BMI2_INT_NONE:

                    /* Un-Map the corresponding feature interrupt to interrupt pin 1 and 2 */
                    reg_data_array[0] &= ~(int_mask);
                    reg_data_array[1] &= ~(int_mask);
                    break;
                case BMI2_INT1:

                    /* Map the corresponding feature interrupt to interrupt pin 1 */
                    reg_data_array[0] |= int_mask;

                    /* Un-map the corresponding feature interrupt to interrupt pin 2 */
                    reg_data_array[1] &= ~(int_mask);
                    break;
                case BMI2_INT2:

                    /* Map the corresponding feature interrupt to interrupt pin 2 */
                    reg_data_array[1] |= int_mask;

                    /* Un-map the corresponding feature interrupt to interrupt pin 1 */
                    reg_data_array[0] &= ~(int_mask);
                    break;
                case BMI2_INT_BOTH:

                    /* Map the corresponding feature interrupt to interrupt pin 1 and 2 */
                    reg_data_array[0] |= int_mask;
                    reg_data_array[1] |= int_mask;
                    break;
                default:
                    break;
            }
        }
        else
        {
            /* Return error if invalid pin selection */
            rslt = BMI2_E_INVALID_INT_PIN;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API gets the accelerometer data from the register.
 */
static int8_t get_accel_sensor_data(struct bmi2_sens_axes_data *data, uint8_t reg_addr, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define data stored in register */
    uint8_t reg_data[BMI2_ACC_GYR_NUM_BYTES] = { 0 };

    /* Read the sensor data */
    rslt = bmi2_get_regs(reg_addr, reg_data, BMI2_ACC_GYR_NUM_BYTES, dev);
    if (rslt == BMI2_OK)
    {
        /* Get accelerometer data from the register */
        get_acc_gyr_data(data, reg_data);

        /* Get the re-mapped accelerometer data */
        get_remapped_data(data, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API gets the gyroscope data from the register.
 */
static int8_t get_gyro_sensor_data(struct bmi2_sens_axes_data *data, uint8_t reg_addr, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define data stored in register */
    uint8_t reg_data[BMI2_ACC_GYR_NUM_BYTES] = { 0 };

    /* Read the sensor data */
    rslt = bmi2_get_regs(reg_addr, reg_data, BMI2_ACC_GYR_NUM_BYTES, dev);
    if (rslt == BMI2_OK)
    {
        /* Get gyroscope data from the register */
        get_acc_gyr_data(data, reg_data);

        /* Get the compensated gyroscope data */
        comp_gyro_cross_axis_sensitivity(data, dev);

        /* Get the re-mapped gyroscope data */
        get_remapped_data(data, dev);

    }

    return rslt;
}

/*!
 * @brief This internal API gets the accelerometer/gyroscope data.
 */
static void get_acc_gyr_data(struct bmi2_sens_axes_data *data, const uint8_t *reg_data)
{
    /* Variables to store msb value */
    uint8_t msb;

    /* Variables to store lsb value */
    uint8_t lsb;

    /* Variables to store both msb and lsb value */
    uint16_t msb_lsb;

    /* Variables to define index */
    uint8_t index = 0;

    /* Read x-axis data */
    lsb = reg_data[index++];
    msb = reg_data[index++];
    msb_lsb = ((uint16_t) msb << 8) | (uint16_t) lsb;
    data->x = (int16_t) msb_lsb;

    /* Read y-axis data */
    lsb = reg_data[index++];
    msb = reg_data[index++];
    msb_lsb = ((uint16_t) msb << 8) | (uint16_t) lsb;
    data->y = (int16_t) msb_lsb;

    /* Read z-axis data */
    lsb = reg_data[index++];
    msb = reg_data[index++];
    msb_lsb = ((uint16_t) msb << 8) | (uint16_t) lsb;
    data->z = (int16_t) msb_lsb;
}

/*!
 * @brief This internal API gets the re-mapped accelerometer/gyroscope data.
 */
static void get_remapped_data(struct bmi2_sens_axes_data *data, const struct bmi2_dev *dev)
{
    /* Array to defined the re-mapped sensor data */
    int16_t remap_data[3] = { 0 };
    int16_t pos_multiplier = INT16_C(1);
    int16_t neg_multiplier = INT16_C(-1);

    /* Fill the array with the un-mapped sensor data */
    remap_data[0] = data->x;
    remap_data[1] = data->y;
    remap_data[2] = data->z;

    /* Get the re-mapped x axis data */
    if (dev->remap.x_axis_sign == BMI2_POS_SIGN)
    {
        data->x = (int16_t)(remap_data[dev->remap.x_axis] * pos_multiplier);
    }
    else
    {
        data->x = (int16_t)(remap_data[dev->remap.x_axis] * neg_multiplier);
    }

    /* Get the re-mapped y axis data */
    if (dev->remap.y_axis_sign == BMI2_POS_SIGN)
    {
        data->y = (int16_t)(remap_data[dev->remap.y_axis] * pos_multiplier);
    }
    else
    {
        data->y = (int16_t)(remap_data[dev->remap.y_axis] * neg_multiplier);
    }

    /* Get the re-mapped z axis data */
    if (dev->remap.z_axis_sign == BMI2_POS_SIGN)
    {
        data->z = (int16_t)(remap_data[dev->remap.z_axis] * pos_multiplier);
    }
    else
    {
        data->z = (int16_t)(remap_data[dev->remap.z_axis] * neg_multiplier);
    }
}

/*!
 * @brief This internal API reads the user-defined bytes of data from the given
 * register address of auxiliary sensor in manual mode.
 */
static int8_t read_aux_data(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, uint8_t burst_len, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Array to store the register data */
    uint8_t reg_data[15] = { 0 };

    /* Variable to define number of bytes to read */
    uint16_t read_length = 0;

    /* Variable to define loop */
    uint8_t loop = 0;

    /* Variable to define counts to get the correct array index */
    uint8_t count = 0;

    /* Variable to define index for the array */
    uint8_t idx = 0;

    while (len > 0)
    {
        /* Set the read address if AUX is not busy */
        rslt = set_if_aux_not_busy(BMI2_AUX_RD_ADDR, reg_addr, dev);
        if (rslt == BMI2_OK)
        {
            /* Read data from bmi2 data register */
            rslt = bmi2_get_regs(BMI2_AUX_X_LSB_ADDR, reg_data, (uint16_t) burst_len, dev);
            dev->delay_us(1000, dev->intf_ptr);
            if (rslt == BMI2_OK)
            {
                /* Get number of bytes to be read */
                if (len < burst_len)
                {
                    read_length = (uint8_t) len;
                }
                else
                {
                    read_length = burst_len;
                }

                /* Update array index and store the data */
                for (loop = 0; loop < read_length; loop++)
                {
                    idx = loop + count;
                    aux_data[idx] = reg_data[loop];
                }
            }
        }

        /* Update address for the next read */
        reg_addr += burst_len;

        /* Update count for the array index */
        count += burst_len;

        /* Update no of bytes left to read */
        len -= read_length;
    }

    return rslt;
}

/*!
 * @brief This internal API writes AUX write address and the user-defined bytes
 * of data to the AUX sensor in manual mode.
 *
 * @note Change of BMI2_AUX_WR_ADDR is only allowed if AUX is not busy.
 */
static int8_t write_aux_data(uint8_t reg_addr, uint8_t reg_data, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Write data to be written to the AUX sensor in bmi2 register */
    rslt = bmi2_set_regs(BMI2_AUX_WR_DATA_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        /* Write the AUX address where data is to be stored when AUX is not busy */
        rslt = set_if_aux_not_busy(BMI2_AUX_WR_ADDR, reg_addr, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API reads the user-defined bytes of data from the given
 * register address of auxiliary sensor in data mode.
 */
static int8_t read_aux_data_mode(uint8_t *aux_data, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variables to define loop */
    uint8_t count = 0;

    /* Variables to define index */
    uint8_t index = 0;

    /* Array to define data stored in register */
    uint8_t reg_data[BMI2_AUX_NUM_BYTES] = { 0 };

    /* Check if data mode */
    if (!dev->aux_man_en)
    {
        /* Read the auxiliary sensor data */
        rslt = bmi2_get_regs(BMI2_AUX_X_LSB_ADDR, reg_data, BMI2_AUX_NUM_BYTES, dev);
        if (rslt == BMI2_OK)
        {
            /* Get the 8 bytes of auxiliary data */
            do
            {
                *(aux_data + count++) = *(reg_data + index++);
            } while (count < BMI2_AUX_NUM_BYTES);
        }
    }
    else
    {
        rslt = BMI2_E_AUX_INVALID_CFG;
    }

    return rslt;
}

/*!
 * @brief This internal API maps the actual burst read length with that of the
 * register value set by user.
 */
static int8_t map_read_len(uint8_t *len, const struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Get the burst read length against the values set by the user */
    switch (dev->aux_man_rd_burst_len)
    {
        case BMI2_AUX_READ_LEN_0:
            *len = 1;
            break;
        case BMI2_AUX_READ_LEN_1:
            *len = 2;
            break;
        case BMI2_AUX_READ_LEN_2:
            *len = 6;
            break;
        case BMI2_AUX_READ_LEN_3:
            *len = 8;
            break;
        default:
            rslt = BMI2_E_AUX_INVALID_CFG;
            break;
    }

    return rslt;
}

/*!
 * @brief This internal API computes the number of bytes of accelerometer FIFO
 * data which is to be parsed in header-less mode.
 */
static int8_t parse_fifo_accel_len(uint16_t *start_idx,
                                   uint16_t *len,
                                   const uint16_t *acc_count,
                                   const struct bmi2_fifo_frame *fifo)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Data start index */
    (*start_idx) = fifo->acc_byte_start_idx;

    /* If only accelerometer is enabled */
    if (fifo->data_enable == BMI2_FIFO_ACC_EN)
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*acc_count) * BMI2_FIFO_ACC_LENGTH);
    }
    /* If only accelerometer and auxiliary are enabled */
    else if (fifo->data_enable == (BMI2_FIFO_ACC_EN | BMI2_FIFO_AUX_EN))
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*acc_count) * BMI2_FIFO_ACC_AUX_LENGTH);
    }
    /* If only accelerometer and gyroscope are enabled */
    else if (fifo->data_enable == (BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN))
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*acc_count) * BMI2_FIFO_ACC_GYR_LENGTH);
    }
    /* If only accelerometer, gyroscope and auxiliary are enabled */
    else if (fifo->data_enable == (BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN | BMI2_FIFO_AUX_EN))
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*acc_count) * BMI2_FIFO_ALL_LENGTH);
    }
    else
    {
        /* Move the data index to the last byte to mark completion when
         * no sensors or sensors apart from accelerometer are enabled
         */
        (*start_idx) = fifo->length;

        /* FIFO is empty */
        rslt = BMI2_W_FIFO_EMPTY;
    }

    /* If more data is requested than available */
    if ((*len) > fifo->length)
    {
        (*len) = fifo->length;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO in header mode.
 */
static int8_t extract_accel_header_mode(struct bmi2_sens_axes_data *acc,
                                        uint16_t *accel_length,
                                        struct bmi2_fifo_frame *fifo,
                                        const struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Variable to define header frame */
    uint8_t frame_header = 0;

    /* Variable to index the data bytes */
    uint16_t data_index;

    /* Variable to index accelerometer frames */
    uint16_t accel_index = 0;

    /* Variable to indicate accelerometer frames read */
    uint16_t frame_to_read = *accel_length;

    for (data_index = fifo->acc_byte_start_idx; data_index < fifo->length;)
    {
        /* Get frame header byte */
        frame_header = fifo->data[data_index] & BMI2_FIFO_TAG_INTR_MASK;

        /* Parse virtual header if S4S is enabled */
        parse_if_virtual_header(&frame_header, &data_index, fifo);

        /* Index shifted to next byte where data starts */
        data_index++;
        switch (frame_header)
        {
            /* If header defines accelerometer frame */
            case BMI2_FIFO_HEADER_ACC_FRM:
            case BMI2_FIFO_HEADER_AUX_ACC_FRM:
            case BMI2_FIFO_HEADER_GYR_ACC_FRM:
            case BMI2_FIFO_HEADER_ALL_FRM:

                /* Unpack from normal frames */
                rslt = unpack_accel_frame(acc, &data_index, &accel_index, frame_header, fifo, dev);
                break;

            /* If header defines only gyroscope frame */
            case BMI2_FIFO_HEADER_GYR_FRM:
                rslt = move_next_frame(&data_index, fifo->gyr_frm_len, fifo);
                break;

            /* If header defines only auxiliary frame */
            case BMI2_FIFO_HEADER_AUX_FRM:
                rslt = move_next_frame(&data_index, fifo->aux_frm_len, fifo);
                break;

            /* If header defines only auxiliary and gyroscope frame */
            case BMI2_FIFO_HEADER_AUX_GYR_FRM:
                rslt = move_next_frame(&data_index, fifo->aux_gyr_frm_len, fifo);
                break;

            /* If header defines sensor time frame */
            case BMI2_FIFO_HEADER_SENS_TIME_FRM:
                rslt = unpack_sensortime_frame(&data_index, fifo);
                break;

            /* If header defines skip frame */
            case BMI2_FIFO_HEADER_SKIP_FRM:
                rslt = unpack_skipped_frame(&data_index, fifo);
                break;

            /* If header defines Input configuration frame */
            case BMI2_FIFO_HEADER_INPUT_CFG_FRM:
                rslt = move_next_frame(&data_index, BMI2_FIFO_INPUT_CFG_LENGTH, fifo);
                break;

            /* If header defines invalid frame or end of valid data */
            case BMI2_FIFO_HEAD_OVER_READ_MSB:

                /* Move the data index to the last byte to mark completion */
                data_index = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            case BMI2_FIFO_VIRT_ACT_RECOG_FRM:
                rslt = move_next_frame(&data_index, BMI2_FIFO_VIRT_ACT_DATA_LENGTH, fifo);
                break;
            default:

                /* Move the data index to the last byte in case of invalid values */
                data_index = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
        }

        /* Break if Number of frames to be read is complete or FIFO is mpty */
        if ((frame_to_read == accel_index) || (rslt == BMI2_W_FIFO_EMPTY))
        {
            break;
        }
    }

    /* Update the accelerometer frame index */
    (*accel_length) = accel_index;

    /* Update the accelerometer byte index */
    fifo->acc_byte_start_idx = data_index;

    return rslt;
}

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO data in both header and header-less mode. It updates the current data
 * byte to be parsed.
 */
static int8_t unpack_accel_frame(struct bmi2_sens_axes_data *acc,
                                 uint16_t *idx,
                                 uint16_t *acc_idx,
                                 uint8_t frame,
                                 const struct bmi2_fifo_frame *fifo,
                                 const struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    switch (frame)
    {
        /* If frame contains only accelerometer data */
        case BMI2_FIFO_HEADER_ACC_FRM:
        case BMI2_FIFO_HEAD_LESS_ACC_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->acc_frm_len) > fifo->length)
            {
                /* Update the data index as complete*/
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the accelerometer data */
            unpack_accel_data(&acc[(*acc_idx)], *idx, fifo, dev);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_ACC_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_sensor_time(&acc[(*acc_idx)], idx, fifo);
            }

            /* Update accelerometer frame index */
            (*acc_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains accelerometer and gyroscope data */
        case BMI2_FIFO_HEADER_GYR_ACC_FRM:
        case BMI2_FIFO_HEAD_LESS_GYR_ACC_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->acc_gyr_frm_len) > fifo->length)
            {
                /* Move the data index to the last byte */
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the accelerometer data */
            unpack_accel_data(&acc[(*acc_idx)], ((*idx) + BMI2_FIFO_GYR_LENGTH), fifo, dev);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_ACC_GYR_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_sensor_time(&acc[(*acc_idx)], idx, fifo);
            }

            /* Update accelerometer frame index */
            (*acc_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains accelerometer and auxiliary data */
        case BMI2_FIFO_HEADER_AUX_ACC_FRM:
        case BMI2_FIFO_HEAD_LESS_AUX_ACC_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->acc_aux_frm_len) > fifo->length)
            {
                /* Move the data index to the last byte */
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the accelerometer data */
            unpack_accel_data(&acc[(*acc_idx)], ((*idx) + BMI2_FIFO_AUX_LENGTH), fifo, dev);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_ACC_AUX_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_sensor_time(&acc[(*acc_idx)], idx, fifo);
            }

            /* Update accelerometer frame index */
            (*acc_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains accelerometer, gyroscope and auxiliary data */
        case BMI2_FIFO_HEADER_ALL_FRM:
        case BMI2_FIFO_HEAD_LESS_ALL_FRM:

            /* Partially read, then skip the data*/
            if ((*idx + fifo->all_frm_len) > fifo->length)
            {
                /* Move the data index to the last byte */
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the accelerometer data */
            unpack_accel_data(&acc[(*acc_idx)], ((*idx) + BMI2_FIFO_GYR_AUX_LENGTH), fifo, dev);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_ALL_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_sensor_time(&acc[(*acc_idx)], idx, fifo);
            }

            /* Update accelerometer frame index */
            (*acc_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains gyroscope and auxiliary data */
        case BMI2_FIFO_HEADER_AUX_GYR_FRM:
        case BMI2_FIFO_HEAD_LESS_GYR_AUX_FRM:

            /* Update data index */
            (*idx) = (*idx) + fifo->aux_gyr_frm_len;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains only auxiliary data */
        case BMI2_FIFO_HEADER_AUX_FRM:
        case BMI2_FIFO_HEAD_LESS_AUX_FRM:

            /* Update data index */
            (*idx) = (*idx) + fifo->aux_frm_len;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains only gyroscope data */
        case BMI2_FIFO_HEADER_GYR_FRM:
        case BMI2_FIFO_HEAD_LESS_GYR_FRM:

            /* Update data index */
            (*idx) = (*idx) + fifo->gyr_frm_len;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;
        default:

            /* Move the data index to the last byte in case of invalid values */
            (*idx) = fifo->length;

            /* FIFO is empty */
            rslt = BMI2_W_FIFO_EMPTY;
            break;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse accelerometer data from the
 * FIFO data.
 */
static void unpack_accel_data(struct bmi2_sens_axes_data *acc,
                              uint16_t data_start_index,
                              const struct bmi2_fifo_frame *fifo,
                              const struct bmi2_dev *dev)
{
    /* Variables to store LSB value */
    uint16_t data_lsb;

    /* Variables to store MSB value */
    uint16_t data_msb;

    /* Accelerometer raw x data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Accelerometer raw y data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Accelerometer raw z data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->z = (int16_t)((data_msb << 8) | data_lsb);

    /* Get the re-mapped accelerometer data */
    get_remapped_data(acc, dev);
}

/*!
 * @brief This internal API computes the number of bytes of gyroscope FIFO data
 * which is to be parsed in header-less mode.
 */
static int8_t parse_fifo_gyro_len(uint16_t *start_idx,
                                  uint16_t(*len),
                                  const uint16_t *gyr_count,
                                  const struct bmi2_fifo_frame *fifo)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Data start index */
    (*start_idx) = fifo->gyr_byte_start_idx;

    /* If only gyroscope is enabled */
    if (fifo->data_enable == BMI2_FIFO_GYR_EN)
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*gyr_count) * BMI2_FIFO_GYR_LENGTH);
    }
    /* If only gyroscope and auxiliary are enabled */
    else if (fifo->data_enable == (BMI2_FIFO_GYR_EN | BMI2_FIFO_AUX_EN))
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*gyr_count) * BMI2_FIFO_GYR_AUX_LENGTH);
    }
    /* If only accelerometer and gyroscope are enabled */
    else if (fifo->data_enable == (BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN))
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*gyr_count) * BMI2_FIFO_ACC_GYR_LENGTH);
    }
    /* If only accelerometer, gyroscope and auxiliary are enabled */
    else if (fifo->data_enable == (BMI2_FIFO_GYR_EN | BMI2_FIFO_AUX_EN | BMI2_FIFO_ACC_EN))
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*gyr_count) * BMI2_FIFO_ALL_LENGTH);
    }
    else
    {
        /* Move the data index to the last byte to mark completion when
         * no sensors or sensors apart from gyroscope are enabled
         */
        (*start_idx) = fifo->length;

        /* FIFO is empty */
        rslt = BMI2_W_FIFO_EMPTY;
    }

    /* If more data is requested than available */
    if (((*len)) > fifo->length)
    {
        (*len) = fifo->length;
    }

    return rslt;
}

/*!
 *  @brief This internal API is used to parse the gyroscope data from the
 *  FIFO data in header mode.
 */
static int8_t extract_gyro_header_mode(struct bmi2_sens_axes_data *gyr,
                                       uint16_t *gyro_length,
                                       struct bmi2_fifo_frame *fifo,
                                       const struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Variable to define header frame */
    uint8_t frame_header = 0;

    /* Variable to index the data bytes */
    uint16_t data_index;

    /* Variable to index gyroscope frames */
    uint16_t gyro_index = 0;

    /* Variable to indicate gyroscope frames read */
    uint16_t frame_to_read = (*gyro_length);

    for (data_index = fifo->gyr_byte_start_idx; data_index < fifo->length;)
    {
        /* Get frame header byte */
        frame_header = fifo->data[data_index] & BMI2_FIFO_TAG_INTR_MASK;

        /* Parse virtual header if S4S is enabled */
        parse_if_virtual_header(&frame_header, &data_index, fifo);

        /* Index shifted to next byte where data starts */
        data_index++;
        switch (frame_header)
        {
            /* If header defines gyroscope frame */
            case BMI2_FIFO_HEADER_GYR_FRM:
            case BMI2_FIFO_HEADER_GYR_ACC_FRM:
            case BMI2_FIFO_HEADER_AUX_GYR_FRM:
            case BMI2_FIFO_HEADER_ALL_FRM:

                /* Unpack from normal frames */
                rslt = unpack_gyro_frame(gyr, &data_index, &gyro_index, frame_header, fifo, dev);
                break;

            /* If header defines only accelerometer frame */
            case BMI2_FIFO_HEADER_ACC_FRM:
                rslt = move_next_frame(&data_index, fifo->acc_frm_len, fifo);
                break;

            /* If header defines only auxiliary frame */
            case BMI2_FIFO_HEADER_AUX_FRM:
                rslt = move_next_frame(&data_index, fifo->aux_frm_len, fifo);
                break;

            /* If header defines only auxiliary and accelerometer frame */
            case BMI2_FIFO_HEADER_AUX_ACC_FRM:
                rslt = move_next_frame(&data_index, fifo->acc_aux_frm_len, fifo);
                break;

            /* If header defines sensor time frame */
            case BMI2_FIFO_HEADER_SENS_TIME_FRM:
                rslt = unpack_sensortime_frame(&data_index, fifo);
                break;

            /* If header defines skip frame */
            case BMI2_FIFO_HEADER_SKIP_FRM:
                rslt = unpack_skipped_frame(&data_index, fifo);
                break;

            /* If header defines Input configuration frame */
            case BMI2_FIFO_HEADER_INPUT_CFG_FRM:
                rslt = move_next_frame(&data_index, BMI2_FIFO_INPUT_CFG_LENGTH, fifo);
                break;

            /* If header defines invalid frame or end of valid data */
            case BMI2_FIFO_HEAD_OVER_READ_MSB:

                /* Move the data index to the last byte */
                data_index = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            case BMI2_FIFO_VIRT_ACT_RECOG_FRM:
                rslt = move_next_frame(&data_index, BMI2_FIFO_VIRT_ACT_DATA_LENGTH, fifo);
                break;
            default:

                /* Move the data index to the last byte in case of invalid values */
                data_index = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
        }

        /* Break if number of frames to be read is complete or FIFO is empty */
        if ((frame_to_read == gyro_index) || (rslt == BMI2_W_FIFO_EMPTY))
        {
            break;
        }
    }

    /* Update the gyroscope frame index */
    (*gyro_length) = gyro_index;

    /* Update the gyroscope byte index */
    fifo->gyr_byte_start_idx = data_index;

    return rslt;
}

/*!
 * @brief This internal API is used to parse the gyroscope data from the FIFO
 * data in both header and header-less mode. It updates the current data byte to
 * be parsed.
 */
static int8_t unpack_gyro_frame(struct bmi2_sens_axes_data *gyr,
                                uint16_t *idx,
                                uint16_t *gyr_idx,
                                uint8_t frame,
                                const struct bmi2_fifo_frame *fifo,
                                const struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    switch (frame)
    {
        /* If frame contains only gyroscope data */
        case BMI2_FIFO_HEADER_GYR_FRM:
        case BMI2_FIFO_HEAD_LESS_GYR_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->gyr_frm_len) > fifo->length)
            {
                /* Update the data index as complete*/
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the gyroscope data */
            unpack_gyro_data(&gyr[(*gyr_idx)], *idx, fifo, dev);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_GYR_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_sensor_time(&gyr[(*gyr_idx)], idx, fifo);
            }

            /* Update gyroscope frame index */
            (*gyr_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains accelerometer and gyroscope data */
        case BMI2_FIFO_HEADER_GYR_ACC_FRM:
        case BMI2_FIFO_HEAD_LESS_GYR_ACC_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->acc_gyr_frm_len) > fifo->length)
            {
                /* Move the data index to the last byte */
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the gyroscope data */
            unpack_gyro_data(&gyr[(*gyr_idx)], (*idx), fifo, dev);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_ACC_GYR_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_sensor_time(&gyr[(*gyr_idx)], idx, fifo);
            }

            /* Update gyroscope frame index */
            (*gyr_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains gyroscope and auxiliary data */
        case BMI2_FIFO_HEADER_AUX_GYR_FRM:
        case BMI2_FIFO_HEAD_LESS_GYR_AUX_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->aux_gyr_frm_len) > fifo->length)
            {
                /* Move the data index to the last byte */
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the gyroscope data */
            unpack_gyro_data(&gyr[(*gyr_idx)], ((*idx) + BMI2_FIFO_AUX_LENGTH), fifo, dev);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_GYR_AUX_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_sensor_time(&gyr[(*gyr_idx)], idx, fifo);
            }

            /* Update gyroscope frame index */
            (*gyr_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains accelerometer, gyroscope and auxiliary data */
        case BMI2_FIFO_HEADER_ALL_FRM:
        case BMI2_FIFO_HEAD_LESS_ALL_FRM:

            /* Partially read, then skip the data*/
            if ((*idx + fifo->all_frm_len) > fifo->length)
            {
                /* Move the data index to the last byte */
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the gyroscope data */
            unpack_gyro_data(&gyr[(*gyr_idx)], ((*idx) + BMI2_FIFO_AUX_LENGTH), fifo, dev);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_ALL_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_sensor_time(&gyr[(*gyr_idx)], idx, fifo);
            }

            /* Update gyroscope frame index */
            (*gyr_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains accelerometer and auxiliary data */
        case BMI2_FIFO_HEADER_AUX_ACC_FRM:
        case BMI2_FIFO_HEAD_LESS_AUX_ACC_FRM:

            /* Update data index */
            (*idx) = (*idx) + fifo->acc_aux_frm_len;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains only auxiliary data */
        case BMI2_FIFO_HEADER_AUX_FRM:
        case BMI2_FIFO_HEAD_LESS_AUX_FRM:

            /* Update data index */
            (*idx) = (*idx) + fifo->aux_frm_len;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains only accelerometer data */
        case BMI2_FIFO_HEADER_ACC_FRM:
        case BMI2_FIFO_HEAD_LESS_ACC_FRM:

            /* Update data index */
            (*idx) = (*idx) + fifo->acc_frm_len;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;
        default:

            /* Move the data index to the last byte in case of invalid values */
            (*idx) = fifo->length;

            /* FIFO is empty */
            rslt = BMI2_W_FIFO_EMPTY;
            break;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse gyroscope data from the FIFO data.
 */
static void unpack_gyro_data(struct bmi2_sens_axes_data *gyr,
                             uint16_t data_start_index,
                             const struct bmi2_fifo_frame *fifo,
                             const struct bmi2_dev *dev)
{
    /* Variables to store LSB value */
    uint16_t data_lsb;

    /* Variables to store MSB value */
    uint16_t data_msb;

    /* Gyroscope raw x data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    gyr->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Gyroscope raw y data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    gyr->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Gyroscope raw z data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    gyr->z = (int16_t)((data_msb << 8) | data_lsb);

    /* Get the compensated gyroscope data */
    comp_gyro_cross_axis_sensitivity(gyr, dev);

    /* Get the re-mapped gyroscope data */
    get_remapped_data(gyr, dev);
}

/*!
 * @brief This API computes the number of bytes of auxiliary FIFO data which is
 * to be parsed in header-less mode.
 */
static int8_t parse_fifo_aux_len(uint16_t *start_idx,
                                 uint16_t(*len),
                                 const uint16_t *aux_count,
                                 const struct bmi2_fifo_frame *fifo)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Data start index */
    *start_idx = fifo->aux_byte_start_idx;

    /* If only auxiliary is enabled */
    if (fifo->data_enable == BMI2_FIFO_AUX_EN)
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*aux_count) * BMI2_FIFO_AUX_LENGTH);
    }
    /* If only accelerometer and auxiliary are enabled */
    else if (fifo->data_enable == (BMI2_FIFO_AUX_EN | BMI2_FIFO_ACC_EN))
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*aux_count) * BMI2_FIFO_ACC_AUX_LENGTH);
    }
    /* If only accelerometer and gyroscope are enabled */
    else if (fifo->data_enable == (BMI2_FIFO_AUX_EN | BMI2_FIFO_GYR_EN))
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*aux_count) * BMI2_FIFO_GYR_AUX_LENGTH);
    }
    /* If only accelerometer, gyroscope and auxiliary are enabled */
    else if (fifo->data_enable == (BMI2_FIFO_AUX_EN | BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN))
    {
        /* Number of bytes to be read */
        (*len) = (uint16_t)((*aux_count) * BMI2_FIFO_ALL_LENGTH);
    }
    else
    {
        /* Move the data index to the last byte to mark completion when
         * no sensors or sensors apart from gyroscope are enabled
         */
        (*start_idx) = fifo->length;

        /* FIFO is empty */
        rslt = BMI2_W_FIFO_EMPTY;
    }

    /* If more data is requested than available */
    if (((*len)) > fifo->length)
    {
        (*len) = fifo->length;
    }

    return rslt;
}

/*!
 * @brief This API is used to parse the auxiliary data from the FIFO data in
 * header mode.
 */
static int8_t extract_aux_header_mode(struct bmi2_aux_fifo_data *aux,
                                      uint16_t *aux_len,
                                      struct bmi2_fifo_frame *fifo,
                                      const struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Variable to define header frame */
    uint8_t frame_header = 0;

    /* Variable to index the data bytes */
    uint16_t data_index;

    /* Variable to index gyroscope frames */
    uint16_t aux_index = 0;

    /* Variable to indicate auxiliary frames read */
    uint16_t frame_to_read = *aux_len;

    for (data_index = fifo->aux_byte_start_idx; data_index < fifo->length;)
    {
        /* Get frame header byte */
        frame_header = fifo->data[data_index] & BMI2_FIFO_TAG_INTR_MASK;

        /* Parse virtual header if S4S is enabled */
        parse_if_virtual_header(&frame_header, &data_index, fifo);

        /* Index shifted to next byte where data starts */
        data_index++;
        switch (frame_header)
        {
            /* If header defines auxiliary frame */
            case BMI2_FIFO_HEADER_AUX_FRM:
            case BMI2_FIFO_HEADER_AUX_ACC_FRM:
            case BMI2_FIFO_HEADER_AUX_GYR_FRM:
            case BMI2_FIFO_HEADER_ALL_FRM:

                /* Unpack from normal frames */
                rslt = unpack_aux_frame(aux, &data_index, &aux_index, frame_header, fifo, dev);
                break;

            /* If header defines only accelerometer frame */
            case BMI2_FIFO_HEADER_ACC_FRM:
                rslt = move_next_frame(&data_index, fifo->acc_frm_len, fifo);
                break;

            /* If header defines only gyroscope frame */
            case BMI2_FIFO_HEADER_GYR_FRM:
                rslt = move_next_frame(&data_index, fifo->gyr_frm_len, fifo);
                break;

            /* If header defines only gyroscope and accelerometer frame */
            case BMI2_FIFO_HEADER_GYR_ACC_FRM:
                rslt = move_next_frame(&data_index, fifo->acc_gyr_frm_len, fifo);
                break;

            /* If header defines sensor time frame */
            case BMI2_FIFO_HEADER_SENS_TIME_FRM:
                rslt = unpack_sensortime_frame(&data_index, fifo);
                break;

            /* If header defines skip frame */
            case BMI2_FIFO_HEADER_SKIP_FRM:
                rslt = unpack_skipped_frame(&data_index, fifo);
                break;

            /* If header defines Input configuration frame */
            case BMI2_FIFO_HEADER_INPUT_CFG_FRM:
                rslt = move_next_frame(&data_index, BMI2_FIFO_INPUT_CFG_LENGTH, fifo);
                break;

            /* If header defines invalid frame or end of valid data */
            case BMI2_FIFO_HEAD_OVER_READ_MSB:

                /* Move the data index to the last byte */
                data_index = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            case BMI2_FIFO_VIRT_ACT_RECOG_FRM:
                rslt = move_next_frame(&data_index, BMI2_FIFO_VIRT_ACT_DATA_LENGTH, fifo);
                break;
            default:

                /* Move the data index to the last byte in case
                 * of invalid values
                 */
                data_index = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
        }

        /* Break if number of frames to be read is complete or FIFO is
         * empty
         */
        if ((frame_to_read == aux_index) || (rslt == BMI2_W_FIFO_EMPTY))
        {
            break;
        }
    }

    /* Update the gyroscope frame index */
    (*aux_len) = aux_index;

    /* Update the gyroscope byte index */
    fifo->aux_byte_start_idx = data_index;

    return rslt;
}

/*!
 * @brief This API is used to parse the auxiliary frame from the FIFO data in
 * both header mode and header-less mode and update the data_index value which
 * is used to store the index of the current data byte which is parsed.
 */
static int8_t unpack_aux_frame(struct bmi2_aux_fifo_data *aux,
                               uint16_t *idx,
                               uint16_t *aux_idx,
                               uint8_t frame,
                               const struct bmi2_fifo_frame *fifo,
                               const struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    switch (frame)
    {
        /* If frame contains only auxiliary data */
        case BMI2_FIFO_HEADER_AUX_FRM:
        case BMI2_FIFO_HEAD_LESS_AUX_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->aux_frm_len) > fifo->length)
            {
                /* Update the data index as complete*/
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the auxiliary data */
            unpack_aux_data(&aux[(*aux_idx)], (*idx), fifo);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_AUX_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_aux_sensor_time(&aux[(*aux_idx)], idx, fifo);
            }

            /* Update auxiliary frame index */
            (*aux_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains accelerometer and auxiliary data */
        case BMI2_FIFO_HEADER_AUX_ACC_FRM:
        case BMI2_FIFO_HEAD_LESS_AUX_ACC_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->acc_aux_frm_len) > fifo->length)
            {
                /* Move the data index to the last byte */
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the auxiliary data */
            unpack_aux_data(&aux[(*aux_idx)], (*idx), fifo);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_ACC_AUX_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_aux_sensor_time(&aux[(*aux_idx)], idx, fifo);
            }

            /* Update auxiliary frame index */
            (*aux_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains gyroscope and auxiliary data */
        case BMI2_FIFO_HEADER_AUX_GYR_FRM:
        case BMI2_FIFO_HEAD_LESS_GYR_AUX_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->aux_gyr_frm_len) > fifo->length)
            {
                /* Move the data index to the last byte */
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the auxiliary data */
            unpack_aux_data(&aux[(*aux_idx)], (*idx), fifo);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_GYR_AUX_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_aux_sensor_time(&aux[(*aux_idx)], idx, fifo);
            }

            /* Update auxiliary frame index */
            (*aux_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains accelerometer, gyroscope and auxiliary data */
        case BMI2_FIFO_HEADER_ALL_FRM:
        case BMI2_FIFO_HEAD_LESS_ALL_FRM:

            /* Partially read, then skip the data */
            if ((*idx + fifo->all_frm_len) > fifo->length)
            {
                /* Move the data index to the last byte */
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI2_W_FIFO_EMPTY;
                break;
            }

            /* Get the auxiliary data */
            unpack_aux_data(&aux[(*aux_idx)], (*idx), fifo);

            /* Update data index */
            (*idx) = (*idx) + BMI2_FIFO_ALL_LENGTH;

            /* Get virtual sensor time if S4S is enabled */
            if (dev->sens_en_stat & BMI2_EXT_SENS_SEL)
            {
                unpack_virt_aux_sensor_time(&aux[(*aux_idx)], idx, fifo);
            }

            /* Update auxiliary frame index */
            (*aux_idx)++;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains only accelerometer data */
        case BMI2_FIFO_HEADER_ACC_FRM:
        case BMI2_FIFO_HEAD_LESS_ACC_FRM:

            /* Update data index */
            (*idx) = (*idx) + fifo->acc_frm_len;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains only gyroscope data */
        case BMI2_FIFO_HEADER_GYR_FRM:
        case BMI2_FIFO_HEAD_LESS_GYR_FRM:

            /* Update data index */
            (*idx) = (*idx) + fifo->gyr_frm_len;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;

        /* If frame contains accelerometer and gyroscope data */
        case BMI2_FIFO_HEADER_GYR_ACC_FRM:
        case BMI2_FIFO_HEAD_LESS_GYR_ACC_FRM:

            /* Update data index */
            (*idx) = (*idx) + fifo->acc_gyr_frm_len;

            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
            break;
        default:

            /* Move the data index to the last byte in case of
             * invalid values
             */
            (*idx) = fifo->length;

            /* FIFO is empty */
            rslt = BMI2_W_FIFO_EMPTY;
            break;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse auxiliary data from the FIFO data.
 */
static void unpack_aux_data(struct bmi2_aux_fifo_data *aux,
                            uint16_t data_start_index,
                            const struct bmi2_fifo_frame *fifo)
{
    /* Variables to store index */
    uint16_t idx = 0;

    /* Get auxiliary data */
    for (; idx < 8; idx++)
    {
        aux->data[idx] = fifo->data[data_start_index++];
    }
}

/*!
 * @brief This internal API parses virtual frame header from the FIFO data.
 */
static void parse_if_virtual_header(uint8_t *frame_header, uint16_t *data_index, const struct bmi2_fifo_frame *fifo)
{
    /* Variable to extract virtual header byte */
    uint8_t virtual_header_mode;

    /* Extract virtual header mode from the frame header */
    virtual_header_mode = BMI2_GET_BITS(*frame_header, BMI2_FIFO_VIRT_FRM_MODE);

    /* If the extracted header byte is a virtual header */
    if (virtual_header_mode == BMI2_FIFO_VIRT_FRM_MODE)
    {
        /* If frame header is not activity recognition header */
        if (*frame_header != 0xC8)
        {
            /* Index shifted to next byte where sensor frame is present */
            (*data_index) = (*data_index) + 1;

            /* Get the sensor frame header */
            *frame_header = fifo->data[*data_index] & BMI2_FIFO_TAG_INTR_MASK;
        }
    }
}

/*!
 * @brief This internal API gets sensor time from the accelerometer and
 * gyroscope virtual frames and updates in the data structure.
 */
static void unpack_virt_sensor_time(struct bmi2_sens_axes_data *sens, uint16_t *idx, const struct bmi2_fifo_frame *fifo)
{
    /* Variables to define 3 bytes of sensor time */
    uint32_t sensor_time_byte3;
    uint16_t sensor_time_byte2;
    uint8_t sensor_time_byte1;

    /* Get sensor time from the FIFO data */
    sensor_time_byte3 = (uint32_t)(fifo->data[(*idx) + BMI2_SENSOR_TIME_MSB_BYTE] << 16);
    sensor_time_byte2 = (uint16_t) fifo->data[(*idx) + BMI2_SENSOR_TIME_XLSB_BYTE] << 8;
    sensor_time_byte1 = fifo->data[(*idx)];

    /* Store sensor time in the sensor data structure */
    sens->virt_sens_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);

    /* Move the data index by 3 bytes */
    (*idx) = (*idx) + BMI2_SENSOR_TIME_LENGTH;
}

/*!
 * @brief This internal API gets sensor time from the auxiliary virtual
 * frames and updates in the data structure.
 */
static void unpack_virt_aux_sensor_time(struct bmi2_aux_fifo_data *aux,
                                        uint16_t *idx,
                                        const struct bmi2_fifo_frame *fifo)
{
    /* Variables to define 3 bytes of sensor time */
    uint32_t sensor_time_byte3;
    uint16_t sensor_time_byte2;
    uint8_t sensor_time_byte1;

    /* Get sensor time from the FIFO data */
    sensor_time_byte3 = (uint32_t)(fifo->data[(*idx) + BMI2_SENSOR_TIME_MSB_BYTE] << 16);
    sensor_time_byte2 = (uint16_t) fifo->data[(*idx) + BMI2_SENSOR_TIME_XLSB_BYTE] << 8;
    sensor_time_byte1 = fifo->data[(*idx)];

    /* Store sensor time in the sensor data structure */
    aux->virt_sens_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);

    /* Move the data index by 3 bytes */
    (*idx) = (*idx) + BMI2_SENSOR_TIME_LENGTH;
}

/*!
 * @brief This internal API is used to reset the FIFO related configurations in
 * the FIFO frame structure for the next FIFO read.
 */
static void reset_fifo_frame_structure(struct bmi2_fifo_frame *fifo, const struct bmi2_dev *dev)
{
    /* Reset FIFO data structure */
    fifo->acc_byte_start_idx = 0;
    fifo->aux_byte_start_idx = 0;
    fifo->gyr_byte_start_idx = 0;
    fifo->sensor_time = 0;
    fifo->skipped_frame_count = 0;
    fifo->act_recog_byte_start_idx = 0;

    /* If S4S is enabled */
    if ((dev->sens_en_stat & BMI2_EXT_SENS_SEL) == BMI2_EXT_SENS_SEL)
    {
        fifo->acc_frm_len = BMI2_FIFO_VIRT_ACC_LENGTH;
        fifo->gyr_frm_len = BMI2_FIFO_VIRT_GYR_LENGTH;
        fifo->aux_frm_len = BMI2_FIFO_VIRT_AUX_LENGTH;
        fifo->acc_gyr_frm_len = BMI2_FIFO_VIRT_ACC_GYR_LENGTH;
        fifo->acc_aux_frm_len = BMI2_FIFO_VIRT_ACC_AUX_LENGTH;
        fifo->aux_gyr_frm_len = BMI2_FIFO_VIRT_GYR_AUX_LENGTH;
        fifo->all_frm_len = BMI2_FIFO_VIRT_ALL_LENGTH;

        /* If S4S is not enabled */
    }
    else
    {
        fifo->acc_frm_len = BMI2_FIFO_ACC_LENGTH;
        fifo->gyr_frm_len = BMI2_FIFO_GYR_LENGTH;
        fifo->aux_frm_len = BMI2_FIFO_AUX_LENGTH;
        fifo->acc_gyr_frm_len = BMI2_FIFO_ACC_GYR_LENGTH;
        fifo->acc_aux_frm_len = BMI2_FIFO_ACC_AUX_LENGTH;
        fifo->aux_gyr_frm_len = BMI2_FIFO_GYR_AUX_LENGTH;
        fifo->all_frm_len = BMI2_FIFO_ALL_LENGTH;
    }
}

/*!
 * @brief This API internal checks whether the FIFO data read is an empty frame.
 * If empty frame, index is moved to the last byte.
 */
static int8_t check_empty_fifo(uint16_t *data_index, const struct bmi2_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI2_OK;

    /* Validate data index */
    if (((*data_index) + 6) < fifo->length)
    {
        /* Check if FIFO is empty */
        if (((fifo->data[(*data_index)] == BMI2_FIFO_MSB_CONFIG_CHECK) &&
             (fifo->data[(*data_index) + 1] == BMI2_FIFO_LSB_CONFIG_CHECK)) &&
            ((fifo->data[(*data_index) + 2] == BMI2_FIFO_MSB_CONFIG_CHECK) &&
             (fifo->data[(*data_index) + 3] == BMI2_FIFO_LSB_CONFIG_CHECK)) &&
            ((fifo->data[(*data_index) + 4] == BMI2_FIFO_MSB_CONFIG_CHECK) &&
             (fifo->data[(*data_index) + 5] == BMI2_FIFO_LSB_CONFIG_CHECK)))
        {
            /* Move the data index to the last byte to mark completion */
            (*data_index) = fifo->length;

            /* FIFO is empty */
            rslt = BMI2_W_FIFO_EMPTY;
        }
        else
        {
            /* More frames could be read */
            rslt = BMI2_W_PARTIAL_READ;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to move the data index ahead of the
 * current_frame_length parameter when unnecessary FIFO data appears while
 * extracting the user specified data.
 */
static int8_t move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmi2_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI2_OK;

    /* Validate data index */
    if (((*data_index) + current_frame_length) > fifo->length)
    {
        /* Move the data index to the last byte */
        (*data_index) = fifo->length;

        /* FIFO is empty */
        rslt = BMI2_W_FIFO_EMPTY;
    }
    else
    {
        /* Move the data index to next frame */
        (*data_index) = (*data_index) + current_frame_length;

        /* More frames could be read */
        rslt = BMI2_W_PARTIAL_READ;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse and store the sensor time from the
 * FIFO data.
 */
static int8_t unpack_sensortime_frame(uint16_t *data_index, struct bmi2_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI2_OK;

    /* Variables to define 3 bytes of sensor time */
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    /* Validate data index */
    if (((*data_index) + BMI2_SENSOR_TIME_LENGTH) > fifo->length)
    {
        /* Move the data index to the last byte */
        (*data_index) = fifo->length;

        /* FIFO is empty */
        rslt = BMI2_W_FIFO_EMPTY;
    }
    else
    {
        /* Get sensor time from the FIFO data */
        sensor_time_byte3 = fifo->data[(*data_index) + BMI2_SENSOR_TIME_MSB_BYTE] << 16;
        sensor_time_byte2 = fifo->data[(*data_index) + BMI2_SENSOR_TIME_XLSB_BYTE] << 8;
        sensor_time_byte1 = fifo->data[(*data_index)];

        /* Update sensor time in the FIFO structure */
        fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);

        /* Move the data index by 3 bytes */
        (*data_index) = (*data_index) + BMI2_SENSOR_TIME_LENGTH;

        /* More frames could be read */
        rslt = BMI2_W_PARTIAL_READ;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse and store the skipped frame count
 * from the FIFO data.
 */
static int8_t unpack_skipped_frame(uint16_t *data_index, struct bmi2_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI2_OK;

    /* Validate data index */
    if ((*data_index) >= fifo->length)
    {
        /* Update the data index to the last byte */
        (*data_index) = fifo->length;

        /* FIFO is empty */
        rslt = BMI2_W_FIFO_EMPTY;
    }
    else
    {
        /* Update skipped frame count in the FIFO structure */
        fifo->skipped_frame_count = fifo->data[(*data_index)];

        /* Move the data index by 1 byte */
        (*data_index) = (*data_index) + 1;

        /* More frames could be read */
        rslt = BMI2_W_PARTIAL_READ;
    }

    return rslt;
}

/*!
 * @brief This internal API enables and configures the accelerometer which is
 * needed for self-test operation. It also sets the amplitude for the self-test.
 */
static int8_t pre_self_test_config(struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Structure to define sensor configurations */
    struct bmi2_sens_config sens_cfg;

    /* List the sensors to be selected */
    uint8_t sens_sel = BMI2_ACCEL;

    /* Enable accelerometer */
    rslt = bmi2_sensor_enable(&sens_sel, 1, dev);
    dev->delay_us(1000, dev->intf_ptr);

    /* Enable self-test amplitude */
    if (rslt == BMI2_OK)
    {
        rslt = set_accel_self_test_amp(BMI2_ENABLE, dev);
    }

    if (rslt == BMI2_OK)
    {
        /* Select accelerometer for sensor configurations */
        sens_cfg.type = BMI2_ACCEL;

        /* Get the default values */
        rslt = bmi2_get_sensor_config(&sens_cfg, 1, dev);
        if (rslt == BMI2_OK)
        {
            /* Set the configurations required for self-test */
            sens_cfg.cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
            sens_cfg.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
            sens_cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
            sens_cfg.cfg.acc.range = BMI2_ACC_RANGE_16G;

            /* Set accelerometer configurations */
            rslt = bmi2_set_sensor_config(&sens_cfg, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API performs the steps needed for self-test operation
 * before reading the accelerometer self-test data.
 */
static int8_t self_test_config(uint8_t sign, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Enable the accelerometer self-test feature */
    rslt = set_accel_self_test_enable(BMI2_ENABLE, dev);
    if (rslt == BMI2_OK)
    {
        /* Selects the sign of accelerometer self-test excitation */
        rslt = set_acc_self_test_sign(sign, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API enables or disables the accelerometer self-test
 * feature in the sensor.
 */
static int8_t set_accel_self_test_enable(uint8_t enable, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define data */
    uint8_t data = 0;

    /* Enable/Disable self-test feature */
    rslt = bmi2_get_regs(BMI2_ACC_SELF_TEST_ADDR, &data, 1, dev);
    if (rslt == BMI2_OK)
    {
        data = BMI2_SET_BIT_POS0(data, BMI2_ACC_SELF_TEST_EN, enable);
        rslt = bmi2_set_regs(BMI2_ACC_SELF_TEST_ADDR, &data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API selects the sign for accelerometer self-test
 * excitation.
 */
static int8_t set_acc_self_test_sign(uint8_t sign, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define data */
    uint8_t data = 0;

    /* Select the sign for self-test excitation */
    rslt = bmi2_get_regs(BMI2_ACC_SELF_TEST_ADDR, &data, 1, dev);
    if (rslt == BMI2_OK)
    {
        data = BMI2_SET_BITS(data, BMI2_ACC_SELF_TEST_SIGN, sign);
        rslt = bmi2_set_regs(BMI2_ACC_SELF_TEST_ADDR, &data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API sets the amplitude of the accelerometer self-test
 * deflection in the sensor.
 */
static int8_t set_accel_self_test_amp(uint8_t amp, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define data */
    uint8_t data = 0;

    /* Select amplitude of the self-test deflection */
    rslt = bmi2_get_regs(BMI2_ACC_SELF_TEST_ADDR, &data, 1, dev);
    if (rslt == BMI2_OK)
    {
        data = BMI2_SET_BITS(data, BMI2_ACC_SELF_TEST_AMP, amp);
        rslt = bmi2_set_regs(BMI2_ACC_SELF_TEST_ADDR, &data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API reads the accelerometer data for x,y and z axis from
 * the sensor. The data units is in LSB format.
 */
static int8_t read_accel_xyz(struct bmi2_sens_axes_data *accel, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define LSB */
    uint16_t lsb = 0;

    /* Variable to define MSB */
    uint16_t msb = 0;

    /* Array to define data buffer */
    uint8_t data[BMI2_ACC_GYR_NUM_BYTES] = { 0 };

    rslt = bmi2_get_regs(BMI2_ACC_X_LSB_ADDR, data, BMI2_ACC_GYR_NUM_BYTES, dev);
    if (rslt == BMI2_OK)
    {
        /* Accelerometer data x axis */
        msb = data[1];
        lsb = data[0];
        accel->x = (int16_t)((msb << 8) | lsb);

        /* Accelerometer data y axis */
        msb = data[3];
        lsb = data[2];
        accel->y = (int16_t)((msb << 8) | lsb);

        /* Accelerometer data z axis */
        msb = data[5];
        lsb = data[4];
        accel->z = (int16_t)((msb << 8) | lsb);
    }

    return rslt;
}

/*!
 * @brief This internal API reads the gyroscope data for x, y and z axis from
 * the sensor. The data units is in LSB format.
 */
static int8_t read_gyro_xyz(struct bmi2_sens_axes_data *gyro, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define LSB */
    uint16_t lsb = 0;

    /* Variable to define MSB */
    uint16_t msb = 0;

    /* Array to define data buffer */
    uint8_t data[BMI2_ACC_GYR_NUM_BYTES] = { 0 };

    rslt = bmi2_get_regs(BMI2_GYR_X_LSB_ADDR, data, BMI2_ACC_GYR_NUM_BYTES, dev);
    if (rslt == BMI2_OK)
    {
        /* Gyroscope data x axis */
        msb = data[1];
        lsb = data[0];
        gyro->x = (int16_t)((msb << 8) | lsb);

        /* Gyroscope data y axis */
        msb = data[3];
        lsb = data[2];
        gyro->y = (int16_t)((msb << 8) | lsb);

        /* Gyroscope data z axis */
        msb = data[5];
        lsb = data[4];
        gyro->z = (int16_t)((msb << 8) | lsb);
    }

    return rslt;
}

/*!
 * @brief This internal API converts LSB value of accelerometer axes to form
 * 'g' to 'mg' for self-test.
 */
static void convert_lsb_g(const struct bmi2_selftest_delta_limit *acc_data_diff,
                          struct bmi2_selftest_delta_limit *acc_data_diff_mg,
                          const struct bmi2_dev *dev)
{
    /* Variable to define LSB/g value of axes */
    uint32_t lsb_per_g;

    /* Range considered for self-test is +/-16g */
    uint8_t range = BMI2_ACC_SELF_TEST_RANGE;

    /* lsb_per_g for the respective resolution and 16g range */
    lsb_per_g = (uint32_t)(power(2, dev->resolution) / (2 * range));

    /* Accelerometer x value in mg */
    acc_data_diff_mg->x = (acc_data_diff->x / (int32_t) lsb_per_g) * 1000;

    /* Accelerometer y value in mg */
    acc_data_diff_mg->y = (acc_data_diff->y / (int32_t) lsb_per_g) * 1000;

    /* Accelerometer z value in mg */
    acc_data_diff_mg->z = (acc_data_diff->z / (int32_t) lsb_per_g) * 1000;
}

/*!
 * @brief This internal API is used to calculate the power of a value.
 */
static int32_t power(int16_t base, uint8_t resolution)
{
    /* Initialize loop */
    uint8_t loop = 1;

    /* Initialize variable to store the power of 2 value */
    int32_t value = 1;

    for (; loop <= resolution; loop++)
    {
        value = (int32_t)(value * base);
    }

    return value;
}

/*!
 * @brief This internal API validates the accelerometer self-test data and
 * decides the result of self-test operation.
 */
static int8_t validate_self_test(const struct bmi2_selftest_delta_limit *accel_data_diff)
{
    /* Variable to define error */
    int8_t rslt;

    /* As per the data sheet, The actually measured signal differences should be significantly
     * larger than the minimum differences for each axis in order for the self-test to pass.
     */
    if ((accel_data_diff->x > BMI2_ST_ACC_X_SIG_MIN_DIFF) && (accel_data_diff->y < BMI2_ST_ACC_Y_SIG_MIN_DIFF) &&
        (accel_data_diff->z > BMI2_ST_ACC_Z_SIG_MIN_DIFF))
    {
        /* Self-test pass */
        rslt = BMI2_OK;
    }
    else
    {
        /* Self-test fail*/
        rslt = BMI2_E_SELF_TEST_FAIL;
    }

    return rslt;
}

/*!
 * @brief This internal API gets the re-mapped x, y and z axes from the sensor.
 */
static int8_t get_remap_axes(struct bmi2_axes_remap *remap, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Array to define the feature configuration */
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };

    /* Variable to define the array offset */
    uint8_t idx = 0;

    /* Variable to set flag */
    uint8_t feat_found;

    /* Initialize feature configuration for axis re-mapping */
    struct bmi2_feature_config remap_config = { 0, 0, 0 };

    /* Variable to get the status of advance power save */
    uint8_t aps_stat;

    /* Get status of advance power save mode */
    aps_stat = dev->aps_status;
    if (aps_stat == BMI2_ENABLE)
    {
        /* Disable advance power save if enabled */
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
    }

    if (rslt == BMI2_OK)
    {
        /* Search for axis re-mapping and extract its configuration details */
        feat_found = bmi2_extract_input_feat_config(&remap_config, BMI2_AXIS_MAP, dev);
        if (feat_found)
        {
            rslt = bmi2_get_feat_config(remap_config.page, feat_config, dev);
            if (rslt == BMI2_OK)
            {
                /* Define the offset for axis re-mapping */
                idx = remap_config.start_addr;

                /* Get the re-mapped x-axis */
                remap->x_axis = BMI2_GET_BIT_POS0(feat_config[idx], BMI2_X_AXIS);

                /* Get the re-mapped x-axis polarity */
                remap->x_axis_sign = BMI2_GET_BITS(feat_config[idx], BMI2_X_AXIS_SIGN);

                /* Get the re-mapped y-axis */
                remap->y_axis = BMI2_GET_BITS(feat_config[idx], BMI2_Y_AXIS);

                /* Get the re-mapped y-axis polarity */
                remap->y_axis_sign = BMI2_GET_BITS(feat_config[idx], BMI2_Y_AXIS_SIGN);

                /* Get the re-mapped z-axis */
                remap->z_axis = BMI2_GET_BITS(feat_config[idx], BMI2_Z_AXIS);

                /* Increment byte to fetch the next data */
                idx++;

                /* Get the re-mapped z-axis polarity */
                remap->z_axis_sign = BMI2_GET_BIT_POS0(feat_config[idx], BMI2_Z_AXIS_SIGN);
            }
        }
        else
        {
            rslt = BMI2_E_INVALID_SENSOR;
        }

        /* Enable Advance power save if disabled while configuring and
         * not when already disabled
         */
        if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
        {
            rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API sets the re-mapped x, y and z axes in the sensor.
 */
static int8_t set_remap_axes(const struct bmi2_axes_remap *remap, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Array to define the feature configuration */
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };

    /* Variable to define the array offset */
    uint8_t idx = 0;

    /* Variable to define the register address */
    uint8_t reg_addr = 0;

    /* Variable to set the re-mapped x-axes in the sensor */
    uint8_t x_axis = 0;

    /* Variable to set the re-mapped y-axes in the sensor */
    uint8_t y_axis = 0;

    /* Variable to set the re-mapped z-axes in the sensor */
    uint8_t z_axis = 0;

    /* Variable to set the re-mapped x-axes sign in the sensor */
    uint8_t x_axis_sign = 0;

    /* Variable to set the re-mapped y-axes sign in the sensor */
    uint8_t y_axis_sign = 0;

    /* Variable to set the re-mapped z-axes sign in the sensor */
    uint8_t z_axis_sign = 0;

    /* Variable to set flag */
    uint8_t feat_found;

    /* Initialize feature configuration for axis re-mapping */
    struct bmi2_feature_config remap_config = { 0, 0, 0 };

    /* Variable to get the status of advance power save */
    uint8_t aps_stat;

    /* Get status of advance power save mode */
    aps_stat = dev->aps_status;
    if (aps_stat == BMI2_ENABLE)
    {
        /* Disable advance power save if enabled */
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
    }

    if (rslt == BMI2_OK)
    {
        /* Search for axis-re-mapping and extract its configuration details */
        feat_found = bmi2_extract_input_feat_config(&remap_config, BMI2_AXIS_MAP, dev);
        if (feat_found)
        {
            /* Get the configuration from the page where axis re-mapping feature resides */
            rslt = bmi2_get_feat_config(remap_config.page, feat_config, dev);
            if (rslt == BMI2_OK)
            {
                /* Define the offset in bytes */
                idx = remap_config.start_addr;

                /* Set the value of re-mapped x-axis */
                x_axis = remap->x_axis & BMI2_X_AXIS_MASK;

                /* Set the value of re-mapped x-axis sign */
                x_axis_sign = ((remap->x_axis_sign << BMI2_X_AXIS_SIGN_POS) & BMI2_X_AXIS_SIGN_MASK);

                /* Set the value of re-mapped y-axis */
                y_axis = ((remap->y_axis << BMI2_Y_AXIS_POS) & BMI2_Y_AXIS_MASK);

                /* Set the value of re-mapped y-axis sign */
                y_axis_sign = ((remap->y_axis_sign << BMI2_Y_AXIS_SIGN_POS) & BMI2_Y_AXIS_SIGN_MASK);

                /* Set the value of re-mapped z-axis */
                z_axis = ((remap->z_axis << BMI2_Z_AXIS_POS) & BMI2_Z_AXIS_MASK);

                /* Set the value of re-mapped z-axis sign */
                z_axis_sign = remap->z_axis_sign & BMI2_Z_AXIS_SIGN_MASK;

                /* Arrange axes in the first byte */
                feat_config[idx] = x_axis | x_axis_sign | y_axis | y_axis_sign | z_axis;

                /* Increment the index */
                idx++;

                /* Cannot OR in the second byte since it holds
                 * gyroscope self-offset correction bit
                 */
                feat_config[idx] = BMI2_SET_BIT_POS0(feat_config[idx], BMI2_Z_AXIS_SIGN, z_axis_sign);

                /* Update the register address */
                reg_addr = BMI2_FEATURES_REG_ADDR + remap_config.start_addr;

                /* Set the configuration back to the page */
                rslt = bmi2_set_regs(reg_addr, &feat_config[remap_config.start_addr], 2, dev);
            }
        }
        else
        {
            rslt = BMI2_E_INVALID_SENSOR;
        }

        /* Enable Advance power save if disabled while configuring and
         * not when already disabled
         */
        if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
        {
            rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API corrects the gyroscope cross-axis sensitivity
 * between the z and the x axis.
 */
static void comp_gyro_cross_axis_sensitivity(struct bmi2_sens_axes_data *gyr_data, const struct bmi2_dev *dev)
{
    /* Get the compensated gyroscope x-axis */
    gyr_data->x = gyr_data->x - (int16_t)(((int32_t) dev->gyr_cross_sens_zx * (int32_t) gyr_data->z) / 512);
}

/*!
 * @brief This internal API is used to validate the boundary conditions.
 */
static int8_t check_boundary_val(uint8_t *val, uint8_t min, uint8_t max, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    if (val != NULL)
    {
        /* Check if value is below minimum value */
        if (*val < min)
        {
            /* Auto correct the invalid value to minimum value */
            *val = min;
            dev->info |= BMI2_I_MIN_VALUE;
        }

        /* Check if value is above maximum value */
        if (*val > max)
        {
            /* Auto correct the invalid value to maximum value */
            *val = max;
            dev->info |= BMI2_I_MAX_VALUE;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API saves the configurations before performing FOC.
 */
static int8_t save_accel_foc_config(struct bmi2_accel_config *acc_cfg,
                                    uint8_t *aps,
                                    uint8_t *acc_en,
                                    struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to get the status from PWR_CTRL register */
    uint8_t pwr_ctrl_data = 0;

    /* Get accelerometer configurations to be saved */
    rslt = get_accel_config(acc_cfg, dev);
    if (rslt == BMI2_OK)
    {
        /* Get accelerometer enable status to be saved */
        rslt = bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &pwr_ctrl_data, 1, dev);
        *acc_en = BMI2_GET_BITS(pwr_ctrl_data, BMI2_ACC_EN);

        /* Get advance power save mode to be saved */
        if (rslt == BMI2_OK)
        {
            rslt = bmi2_get_adv_power_save(aps, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal sets configurations for performing accelerometer FOC.
 */
static int8_t set_accel_foc_config(struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to select the sensor */
    uint8_t sens_list = BMI2_ACCEL;

    /* Variable to set the accelerometer configuration value */
    uint8_t acc_conf_data = BMI2_FOC_ACC_CONF_VAL;

    /* Disabling offset compensation */
    rslt = set_accel_offset_comp(BMI2_DISABLE, dev);
    if (rslt == BMI2_OK)
    {
        /* Set accelerometer configurations to 50Hz, continuous mode, CIC mode */
        rslt = bmi2_set_regs(BMI2_ACC_CONF_ADDR, &acc_conf_data, 1, dev);
        if (rslt == BMI2_OK)
        {
            /* Set accelerometer to normal mode by enabling it */
            rslt = bmi2_sensor_enable(&sens_list, 1, dev);

            if (rslt == BMI2_OK)
            {
                /* Disable advance power save mode */
                rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API performs Fast Offset Compensation for accelerometer.
 */
static int8_t perform_accel_foc(const struct bmi2_accel_foc_g_value *accel_g_value,
                                const struct bmi2_accel_config *acc_cfg,
                                struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_E_INVALID_STATUS;

    /* Variable to define count */
    uint8_t loop;

    /* Variable to store status read from the status register */
    uint8_t reg_status = 0;

    /* Array of structure to store accelerometer data */
    struct bmi2_sens_axes_data accel_value[128] = { { 0 } };

    /* Structure to store accelerometer data temporarily */
    struct bmi2_foc_temp_value temp = { 0, 0, 0 };

    /* Structure to store the average of accelerometer data */
    struct bmi2_sens_axes_data accel_avg = { 0, 0, 0, 0 };

    /* Variable to define LSB per g value */
    uint16_t lsb_per_g = 0;

    /* Variable to define range */
    uint8_t range = 0;

    /* Structure to store accelerometer data deviation from ideal value */
    struct bmi2_offset_delta delta = { 0, 0, 0 };

    /* Structure to store accelerometer offset values */
    struct bmi2_accel_offset offset = { 0, 0, 0 };

    /* Variable tries max 5 times for interrupt then generates timeout */
    uint8_t try_cnt;

    for (loop = 0; loop < 128; loop++)
    {
        try_cnt = 5;
        while (try_cnt && (!(reg_status & BMI2_DRDY_ACC)))
        {
            /* 20ms delay for 50Hz ODR */
            dev->delay_us(20000, dev->intf_ptr);
            rslt = bmi2_get_status(&reg_status, dev);
            try_cnt--;
        }

        if ((rslt == BMI2_OK) && (reg_status & BMI2_DRDY_ACC))
        {
            rslt = read_accel_xyz(&accel_value[loop], dev);
        }

        if (rslt == BMI2_OK)
        {
            rslt = read_accel_xyz(&accel_value[loop], dev);
        }

        if (rslt == BMI2_OK)
        {
            /* Store the data in a temporary structure */
            temp.x = temp.x + (int32_t)accel_value[loop].x;
            temp.y = temp.y + (int32_t)accel_value[loop].y;
            temp.z = temp.z + (int32_t)accel_value[loop].z;
        }
        else
        {
            break;
        }
    }

    if (rslt == BMI2_OK)
    {
        /* Take average of x, y and z data for lesser noise */
        accel_avg.x = (int16_t)(temp.x / 128);
        accel_avg.y = (int16_t)(temp.y / 128);
        accel_avg.z = (int16_t)(temp.z / 128);

        /* Get the exact range value */
        map_accel_range(acc_cfg->range, &range);

        /* Get the smallest possible measurable acceleration level given the range and
         * resolution */
        lsb_per_g = (uint16_t)(power(2, dev->resolution) / (2 * range));

        /* Compensate acceleration data against gravity */
        comp_for_gravity(lsb_per_g, accel_g_value, &accel_avg, &delta);

        /* Scale according to offset register resolution */
        scale_accel_offset(range, &delta, &offset);

        /* Invert the accelerometer offset data */
        invert_accel_offset(&offset);

        /* Write offset data in the offset compensation register */
        rslt = write_accel_offset(&offset, dev);

        /* Enable offset compensation */
        if (rslt == BMI2_OK)
        {
            rslt = set_accel_offset_comp(BMI2_ENABLE, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API enables/disables the offset compensation for
 * filtered and un-filtered accelerometer data.
 */
static int8_t set_accel_offset_comp(uint8_t offset_en, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t data = 0;

    /* Enable/Disable offset compensation */
    rslt = bmi2_get_regs(BMI2_NV_CONF_ADDR, &data, 1, dev);
    if (rslt == BMI2_OK)
    {
        data = BMI2_SET_BITS(data, BMI2_NV_ACC_OFFSET, offset_en);
        rslt = bmi2_set_regs(BMI2_NV_CONF_ADDR, &data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API converts the accelerometer range value into
 * corresponding integer value.
 */
static void map_accel_range(uint8_t range_in, uint8_t *range_out)
{
    switch (range_in)
    {
        case BMI2_ACC_RANGE_2G:
            *range_out = 2;
            break;
        case BMI2_ACC_RANGE_4G:
            *range_out = 4;
            break;
        case BMI2_ACC_RANGE_8G:
            *range_out = 8;
            break;
        case BMI2_ACC_RANGE_16G:
            *range_out = 16;
            break;
        default:

            /* By default RANGE 8G is set */
            *range_out = 8;
            break;
    }
}

/*!
 * @brief This internal API compensate the accelerometer data against gravity.
 */
static void comp_for_gravity(uint16_t lsb_per_g,
                             const struct bmi2_accel_foc_g_value *g_val,
                             const struct bmi2_sens_axes_data *data,
                             struct bmi2_offset_delta *comp_data)
{
    /* Array to store the accelerometer values in LSB */
    int16_t accel_value_lsb[3] = { 0 };

    /* Convert g-value to LSB */
    accel_value_lsb[BMI2_X_AXIS] = (int16_t)(lsb_per_g * g_val->x);
    accel_value_lsb[BMI2_Y_AXIS] = (int16_t)(lsb_per_g * g_val->y);
    accel_value_lsb[BMI2_Z_AXIS] = (int16_t)(lsb_per_g * g_val->z);

    /* Get the compensated values for X, Y and Z axis */
    comp_data->x = (data->x - accel_value_lsb[BMI2_X_AXIS]);
    comp_data->y = (data->y - accel_value_lsb[BMI2_Y_AXIS]);
    comp_data->z = (data->z - accel_value_lsb[BMI2_Z_AXIS]);
}

/*!
 * @brief This internal API scales the compensated accelerometer data according
 * to the offset register resolution.
 *
 * @note The bit position is always greater than 0 since accelerometer data is
 * 16 bit wide.
 */
static void scale_accel_offset(uint8_t range, const struct bmi2_offset_delta *comp_data, struct bmi2_accel_offset *data)
{
    /* Variable to store the position of bit having 3.9mg resolution */
    int8_t bit_pos_3_9mg;

    /* Variable to store the position previous of bit having 3.9mg resolution */
    int8_t bit_pos_3_9mg_prev_bit;

    /* Variable to store the round-off value */
    uint8_t round_off;

    /* Find the bit position of 3.9mg */
    bit_pos_3_9mg = get_bit_pos_3_9mg(range);

    /* Round off, consider if the next bit is high */
    bit_pos_3_9mg_prev_bit = bit_pos_3_9mg - 1;
    round_off = (uint8_t)(power(2, ((uint8_t) bit_pos_3_9mg_prev_bit)));

    /* Scale according to offset register resolution */
    data->x = (uint8_t)((comp_data->x + round_off) / power(2, ((uint8_t) bit_pos_3_9mg)));
    data->y = (uint8_t)((comp_data->y + round_off) / power(2, ((uint8_t) bit_pos_3_9mg)));
    data->z = (uint8_t)((comp_data->z + round_off) / power(2, ((uint8_t) bit_pos_3_9mg)));
}

/*!
 * @brief This internal API finds the bit position of 3.9mg according to given
 * range and resolution.
 */
static int8_t get_bit_pos_3_9mg(uint8_t range)
{
    /* Variable to store the bit position of 3.9mg resolution */
    int8_t bit_pos_3_9mg;

    /* Variable to shift the bits according to the resolution  */
    uint32_t divisor = 1;

    /* Scaling factor to get the bit position of 3.9 mg resolution */
    int16_t scale_factor = -1;

    /* Variable to store temporary value */
    uint16_t temp;

    /* Shift left by the times of resolution */
    divisor = divisor << 16;

    /* Get the bit position to be shifted */
    temp = (uint16_t)(divisor / (range * 256));

    /* Get the scaling factor until bit position is shifted to last bit */
    while (temp != 1)
    {
        scale_factor++;
        temp = temp >> 1;
    }

    /* Scaling factor is the bit position of 3.9 mg resolution */
    bit_pos_3_9mg = (int8_t) scale_factor;

    return bit_pos_3_9mg;
}

/*!
 * @brief This internal API inverts the accelerometer offset data.
 */
static void invert_accel_offset(struct bmi2_accel_offset *offset_data)
{
    /* Get the offset data */
    offset_data->x = (uint8_t)((offset_data->x) * (-1));
    offset_data->y = (uint8_t)((offset_data->y) * (-1));
    offset_data->z = (uint8_t)((offset_data->z) * (-1));
}

/*!
 * @brief This internal API writes the offset data in the offset compensation
 * register.
 */
static int8_t write_accel_offset(const struct bmi2_accel_offset *offset, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store the offset data */
    uint8_t data_array[3] = { 0 };

    data_array[0] = offset->x;
    data_array[1] = offset->y;
    data_array[2] = offset->z;

    /* Offset values are written in the offset register */
    rslt = bmi2_set_regs(BMI2_ACC_OFF_COMP_0_ADDR, data_array, 3, dev);

    return rslt;
}

/*!
 * @brief This internal API restores the configurations saved before performing
 * accelerometer FOC.
 */
static int8_t restore_accel_foc_config(struct bmi2_accel_config *acc_cfg,
                                       uint8_t aps,
                                       uint8_t acc_en,
                                       struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to get the status from PWR_CTRL register */
    uint8_t pwr_ctrl_data = 0;

    /* Restore the saved accelerometer configurations */
    rslt = set_accel_config(acc_cfg, dev);
    if (rslt == BMI2_OK)
    {
        /* Restore the saved accelerometer enable status */
        rslt = bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &pwr_ctrl_data, 1, dev);
        if (rslt == BMI2_OK)
        {
            pwr_ctrl_data = BMI2_SET_BITS(pwr_ctrl_data, BMI2_ACC_EN, acc_en);
            rslt = bmi2_set_regs(BMI2_PWR_CTRL_ADDR, &pwr_ctrl_data, 1, dev);

            /* Restore the saved advance power save */
            if (rslt == BMI2_OK)
            {
                rslt = bmi2_set_adv_power_save(aps, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API sets accelerometer configurations like ODR,
 * bandwidth, performance mode and g-range.
 */
static int8_t set_accel_config(struct bmi2_accel_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data;

    /* Array to store the default value of accelerometer configuration
     * reserved registers
     */
    uint8_t data_array[2] = { 0 };

    /* Validate bandwidth and performance mode */
    rslt = validate_bw_perf_mode(&config->bwp, &config->filter_perf, dev);
    if (rslt == BMI2_OK)
    {
        /* Validate ODR and range */
        rslt = validate_odr_range(&config->odr, &config->range, dev);
        if (rslt == BMI2_OK)
        {
            /* Set accelerometer performance mode */
            reg_data = BMI2_SET_BITS(data_array[0], BMI2_ACC_FILTER_PERF_MODE, config->filter_perf);

            /* Set accelerometer bandwidth */
            reg_data = BMI2_SET_BITS(reg_data, BMI2_ACC_BW_PARAM, config->bwp);

            /* Set accelerometer ODR */
            reg_data = BMI2_SET_BIT_POS0(reg_data, BMI2_ACC_ODR, config->odr);

            /* Copy the register data to the array */
            data_array[0] = reg_data;

            /* Set accelerometer range */
            reg_data = BMI2_SET_BIT_POS0(data_array[1], BMI2_ACC_RANGE, config->range);

            /* Copy the register data to the array */
            data_array[1] = reg_data;

            /* Write accelerometer configuration to ACC_CONFand
             * ACC_RANGE registers simultaneously as they lie in consecutive places
             */
            rslt = bmi2_set_regs(BMI2_ACC_CONF_ADDR, data_array, 2, dev);

            /* Get error status to check for invalid configurations */
            if (rslt == BMI2_OK)
            {
                rslt = cfg_error_status(dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API sets gyroscope configurations like ODR, bandwidth,
 * low power/high performance mode, performance mode and range. It also
 * maps/un-maps data interrupts to that of hardware interrupt line.
 */
static int8_t set_gyro_config(struct bmi2_gyro_config *config, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data;

    /* Array to store the default value of gyroscope configuration reserved registers  */
    uint8_t data_array[2] = { 0 };

    /* Validate gyroscope configurations */
    rslt = validate_gyro_config(config, dev);
    if (rslt == BMI2_OK)
    {
        /* Set gyroscope performance mode */
        reg_data = BMI2_SET_BITS(data_array[0], BMI2_GYR_FILTER_PERF_MODE, config->filter_perf);

        /* Set gyroscope noise performance mode */
        reg_data = BMI2_SET_BITS(reg_data, BMI2_GYR_NOISE_PERF_MODE, config->noise_perf);

        /* Set gyroscope bandwidth */
        reg_data = BMI2_SET_BITS(reg_data, BMI2_GYR_BW_PARAM, config->bwp);

        /* Set gyroscope ODR */
        reg_data = BMI2_SET_BIT_POS0(reg_data, BMI2_GYR_ODR, config->odr);

        /* Copy the register data to the array */
        data_array[0] = reg_data;

        /* Set gyroscope OIS range */
        reg_data = BMI2_SET_BITS(data_array[1], BMI2_GYR_OIS_RANGE, config->ois_range);

        /* Set gyroscope range */
        reg_data = BMI2_SET_BIT_POS0(reg_data, BMI2_GYR_RANGE, config->range);

        /* Copy the register data to the array */
        data_array[1] = reg_data;

        /* Write accelerometer configuration to GYR_CONF and GYR_RANGE
         * registers simultaneously as they lie in consecutive places
         */
        rslt = bmi2_set_regs(BMI2_GYR_CONF_ADDR, data_array, 2, dev);

        /* Get error status to check for invalid configurations */
        if (rslt == BMI2_OK)
        {
            rslt = cfg_error_status(dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API saves the configurations before performing gyroscope
 * FOC.
 */
static int8_t save_gyro_config(struct bmi2_gyro_config *gyr_cfg, uint8_t *aps, uint8_t *gyr_en, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to get the status from PWR_CTRL register */
    uint8_t pwr_ctrl_data = 0;

    /* Get gyroscope configurations to be saved */
    rslt = get_gyro_config(gyr_cfg, dev);
    if (rslt == BMI2_OK)
    {
        /* Get gyroscope enable status to be saved */
        rslt = bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &pwr_ctrl_data, 1, dev);
        *gyr_en = BMI2_GET_BITS(pwr_ctrl_data, BMI2_GYR_EN);

        /* Get advance power save mode to be saved */
        if (rslt == BMI2_OK)
        {
            rslt = bmi2_get_adv_power_save(aps, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal sets configurations for performing gyroscope FOC.
 */
static int8_t set_gyro_foc_config(struct bmi2_dev *dev)
{
    int8_t rslt;

    /* Variable to select the sensor */
    uint8_t sens_list = BMI2_GYRO;

    /* Array to set the gyroscope configuration value (ODR, Performance mode
     * and bandwidth) and gyroscope range
     */
    uint8_t gyr_conf_data[2] = { BMI2_FOC_GYR_CONF_VAL, BMI2_GYR_RANGE_2000 };

    /* Disabling gyroscope offset compensation */
    rslt = bmi2_set_gyro_offset_comp(BMI2_DISABLE, dev);
    if (rslt == BMI2_OK)
    {
        /* Set gyroscope configurations to 25Hz, continuous mode,
         * CIC mode, and 2000 dps range
         */
        rslt = bmi2_set_regs(BMI2_GYR_CONF_ADDR, gyr_conf_data, 2, dev);
        if (rslt == BMI2_OK)
        {
            /* Set gyroscope to normal mode by enabling it */
            rslt = bmi2_sensor_enable(&sens_list, 1, dev);

            if (rslt == BMI2_OK)
            {
                /* Disable advance power save mode */
                rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API inverts the gyroscope offset data.
 */
static void invert_gyro_offset(struct bmi2_sens_axes_data *offset_data)
{
    /* Invert the values */
    offset_data->x = (int16_t)((offset_data->x) * (-1));
    offset_data->y = (int16_t)((offset_data->y) * (-1));
    offset_data->z = (int16_t)((offset_data->z) * (-1));
}

/*!
 * @brief This internal API restores the gyroscope configurations saved
 * before performing FOC.
 */
static int8_t restore_gyro_config(struct bmi2_gyro_config *gyr_cfg, uint8_t aps, uint8_t gyr_en, struct bmi2_dev *dev)
{
    int8_t rslt;
    uint8_t pwr_ctrl_data = 0;

    /* Restore the saved gyroscope configurations */
    rslt = set_gyro_config(gyr_cfg, dev);
    if (rslt == BMI2_OK)
    {
        /* Restore the saved gyroscope enable status */
        rslt = bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &pwr_ctrl_data, 1, dev);
        if (rslt == BMI2_OK)
        {
            pwr_ctrl_data = BMI2_SET_BITS(pwr_ctrl_data, BMI2_GYR_EN, gyr_en);
            rslt = bmi2_set_regs(BMI2_PWR_CTRL_ADDR, &pwr_ctrl_data, 1, dev);

            /* Restore the saved advance power save */
            if (rslt == BMI2_OK)
            {
                rslt = bmi2_set_adv_power_save(aps, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API saturates the gyroscope data value before writing to
 * to 10 bit offset register.
 */
static void saturate_gyro_data(struct bmi2_sens_axes_data *gyr_off)
{
    if (gyr_off->x > 511)
    {
        gyr_off->x = 511;
    }

    if (gyr_off->x < -512)
    {
        gyr_off->x = -512;
    }

    if (gyr_off->y > 511)
    {
        gyr_off->y = 511;
    }

    if (gyr_off->y < -512)
    {
        gyr_off->y = -512;
    }

    if (gyr_off->z > 511)
    {
        gyr_off->z = 511;
    }

    if (gyr_off->z < -512)
    {
        gyr_off->z = -512;
    }
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi2_dev *dev)
{
    int8_t rslt = BMI2_OK;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API is to get the status of st_status from gry_crt_conf register
 */
static int8_t get_st_running(uint8_t *st_status, struct bmi2_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = 0;

    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Get the status of crt running */
        rslt = bmi2_get_regs(BMI2_GYR_CRT_CONF_ADDR, &reg_data, 1, dev);
        if (rslt == BMI2_OK)
        {
            (*st_status) = BMI2_GET_BITS(reg_data, BMI2_GYR_CRT_RUNNING);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API enables/disables the CRT running.
 */
static int8_t set_st_running(uint8_t st_status, struct bmi2_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = 0;

    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_get_regs(BMI2_GYR_CRT_CONF_ADDR, &reg_data, 1, dev);
        if (rslt == BMI2_OK)
        {
            reg_data = BMI2_SET_BITS(reg_data, BMI2_GYR_CRT_RUNNING, st_status);
            rslt = bmi2_set_regs(BMI2_GYR_CRT_CONF_ADDR, &reg_data, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the status of rdy for dl bit.
 */
static int8_t get_rdy_for_dl(uint8_t *rdy_for_dl, struct bmi2_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = 0;

    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Get the status of rdy_fo_dl */
        rslt = bmi2_get_regs(BMI2_GYR_CRT_CONF_ADDR, &reg_data, 1, dev);
        if (rslt == BMI2_OK)
        {
            (*rdy_for_dl) = BMI2_GET_BITS(reg_data, BMI2_GYR_RDY_FOR_DL);
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API does the crt process if max burst length is not zero.
 */
static int8_t process_crt_download(uint8_t last_byte_flag, struct bmi2_dev *dev)
{
    int8_t rslt;
    uint8_t rdy_for_dl = 0;
    uint8_t cmd = BMI2_G_TRIGGER_CMD;

    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        rslt = get_rdy_for_dl(&rdy_for_dl, dev);
    }

    /* Trigger next CRT command */
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_set_regs(BMI2_CMD_REG_ADDR, &cmd, 1, dev);
    }

    if ((!last_byte_flag) && (rslt == BMI2_OK))
    {
        rslt = wait_rdy_for_dl_toggle(BMI2_CRT_READY_FOR_DOWNLOAD_RETRY, rdy_for_dl, dev);
    }

    return rslt;
}

/*!
 * @brief This API to write the 2kb size of crt configuration
 */
static int8_t write_crt_config_file(uint16_t write_len,
                                    uint16_t config_file_size,
                                    uint16_t start_index,
                                    struct bmi2_dev *dev)
{
    int8_t rslt = BMI2_OK;
    uint16_t index = 0;
    uint8_t last_byte_flag = 0;
    uint8_t remain = (uint8_t)(config_file_size % write_len);
    uint16_t balance_byte = 0;

    if (!remain)
    {

        /* Write the configuration file */
        for (index = start_index;
             (index < (start_index + config_file_size)) && (rslt == BMI2_OK);
             index += write_len)
        {
            rslt = upload_file((dev->config_file_ptr + index), index, write_len, dev);
            if (index >= ((start_index + config_file_size) - (write_len)))
            {
                last_byte_flag = 1;
            }

            if (rslt == BMI2_OK)
            {
                rslt = process_crt_download(last_byte_flag, dev);
            }
        }
    }
    else
    {
        /* Get the balance bytes */
        balance_byte = (uint16_t)start_index + (uint16_t)config_file_size - (uint16_t)remain;

        /* Write the configuration file for the balance bytes */
        for (index = start_index; (index < balance_byte) && (rslt == BMI2_OK); index += write_len)
        {
            rslt = upload_file((dev->config_file_ptr + index), index, write_len, dev);
            if (rslt == BMI2_OK)
            {
                rslt = process_crt_download(last_byte_flag, dev);
            }
        }

        if (rslt == BMI2_OK)
        {
            /* Write the remaining bytes in 2 bytes length */
            write_len = 2;
            rslt = set_maxburst_len(write_len, dev);

            /* Write the configuration file for the remaining bytes */
            for (index = balance_byte;
                 (index < (start_index + config_file_size)) && (rslt == BMI2_OK);
                 index += write_len)
            {
                rslt = upload_file((dev->config_file_ptr + index), index, write_len, dev);
                if (index < ((start_index + config_file_size) - write_len))
                {
                    last_byte_flag = 1;
                }

                if (rslt == BMI2_OK)
                {
                    rslt = process_crt_download(last_byte_flag, dev);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API is to wait till the rdy for dl bit toggles after every pack of bytes.
 */
static int8_t wait_rdy_for_dl_toggle(uint8_t retry_complete, uint8_t download_ready, struct bmi2_dev *dev)
{
    int8_t rslt = BMI2_OK;
    uint8_t dl_ready = 0;
    uint8_t st_status = 0;

    while ((rslt == BMI2_OK) && (retry_complete--))
    {
        rslt = get_rdy_for_dl(&dl_ready, dev);
        if (download_ready != dl_ready)
        {
            break;
        }

        dev->delay_us(BMI2_CRT_READY_FOR_DOWNLOAD_US, dev->intf_ptr);
    }

    if ((rslt == BMI2_OK) && (download_ready == dl_ready))
    {
        rslt = BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT;
    }

    if (rslt == BMI2_OK)
    {
        rslt = get_st_running(&st_status, dev);
        if ((rslt == BMI2_OK) && (st_status == 0))
        {
            rslt = BMI2_E_ST_ALREADY_RUNNING;
        }
    }

    return rslt;
}

/*!
 * @brief This API is to wait till crt status complete.
 */
static int8_t wait_st_running(uint8_t retry_complete, struct bmi2_dev *dev)
{
    uint8_t st_status = 1;
    int8_t rslt = BMI2_OK;

    while (retry_complete--)
    {
        rslt = get_st_running(&st_status, dev);
        if ((rslt == BMI2_OK) && (st_status == 0))
        {
            break;
        }

        dev->delay_us(BMI2_CRT_WAIT_RUNNING_US, dev->intf_ptr);
    }

    if ((rslt == BMI2_OK) && (st_status == 1))
    {
        rslt = BMI2_E_ST_ALREADY_RUNNING;
    }

    return rslt;
}

/*!
 * @brief This api is used to perform gyroscope self-test.
 */
int8_t bmi2_do_gyro_st(struct bmi2_dev *dev)
{
    int8_t rslt;

    rslt = do_gtrigger_test(BMI2_SELECT_GYRO_SELF_TEST, dev);

    return rslt;
}

/*!
 * @brief This API is to run the CRT process for both max burst length 0 and non zero condition.
 */
int8_t bmi2_do_crt(struct bmi2_dev *dev)
{
    int8_t rslt;

    rslt = do_gtrigger_test(BMI2_SELECT_CRT, dev);

    return rslt;
}

/*!
 * @brief This API is to run the crt process for both max burst length 0 and non zero condition.
 */
static int8_t do_gtrigger_test(uint8_t gyro_st_crt, struct bmi2_dev *dev)
{
    int8_t rslt;
    int8_t rslt_crt = BMI2_OK;
    uint8_t st_status = 0;
    uint8_t max_burst_length = 0;
    uint8_t download_ready = 0;
    uint8_t cmd = BMI2_G_TRIGGER_CMD;
    struct bmi2_gyro_self_test_status gyro_st_result = { 0 };

    /* Variable to get the status of advance power save */
    uint8_t aps_stat = 0;

    rslt = null_ptr_check(dev);
    if (rslt == BMI2_OK)
    {
        /* Check if the variant supports this feature */
        if (dev->variant_feature & BMI2_CRT_RTOSK_ENABLE)
        {
            /* Get status of advance power save mode */
            aps_stat = dev->aps_status;
            if (aps_stat == BMI2_ENABLE)
            {
                /* Disable advance power save if enabled */
                rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
            }

            /* Get max burst length */
            if (rslt == BMI2_OK)
            {
                rslt = get_maxburst_len(&max_burst_length, dev);
            }

            /* Checking for CRT running status */
            if (rslt == BMI2_OK)
            {
                rslt = get_st_running(&st_status, dev);
            }

            /* CRT is not running  and Max burst length is zero */
            if (st_status == 0)
            {
                if (rslt == BMI2_OK)
                {
                    rslt = set_st_running(BMI2_ENABLE, dev);
                }

                /* Preparing the setup */
                if (rslt == BMI2_OK)
                {
                    rslt = crt_prepare_setup(dev);
                }

                /* Enable the gyro self-test, CRT */
                if (rslt == BMI2_OK)
                {
                    rslt = select_self_test(gyro_st_crt, dev);
                }

                /* Check if FIFO is unchanged by checking the max burst length */
                if ((rslt == BMI2_OK) && (max_burst_length == 0))
                {
                    /* Trigger CRT */
                    rslt = bmi2_set_regs(BMI2_CMD_REG_ADDR, &cmd, 1, dev);
                    if (rslt == BMI2_OK)
                    {
                        /* Wait until st_status = 0 or time out is 2 seconds */
                        rslt = wait_st_running(BMI2_CRT_WAIT_RUNNING_RETRY_EXECUTION, dev);

                        /* CRT Running wait & check is successful */
                        if (rslt == BMI2_OK)
                        {
                            rslt = crt_gyro_st_update_result(dev);
                        }
                    }
                }
                else
                {
                    /* FIFO may be used */
                    if (rslt == BMI2_OK)
                    {
                        if (dev->read_write_len < 2)
                        {
                            dev->read_write_len = 2;
                        }

                        if (dev->read_write_len > (BMI2_CRT_MAX_BURST_WORD_LENGTH * 2))
                        {
                            dev->read_write_len = BMI2_CRT_MAX_BURST_WORD_LENGTH * 2;
                        }

                        /* Reset the max burst length to default value */
                        rslt = set_maxburst_len(dev->read_write_len, dev);
                    }

                    if (rslt == BMI2_OK)
                    {
                        rslt = get_rdy_for_dl(&download_ready, dev);
                    }

                    /* Trigger CRT  */
                    if (rslt == BMI2_OK)
                    {
                        rslt = bmi2_set_regs(BMI2_CMD_REG_ADDR, &cmd, 1, dev);
                    }

                    /* Wait till either ready for download toggle or crt running = 0 */
                    if (rslt == BMI2_OK)
                    {
                        rslt = wait_rdy_for_dl_toggle(BMI2_CRT_READY_FOR_DOWNLOAD_RETRY, download_ready, dev);
                        if (rslt == BMI2_OK)
                        {
                            rslt = write_crt_config_file(dev->read_write_len, BMI2_CRT_CONFIG_FILE_SIZE, 0x1800, dev);
                        }

                        if (rslt == BMI2_OK)
                        {
                            rslt = wait_st_running(BMI2_CRT_WAIT_RUNNING_RETRY_EXECUTION, dev);
                            rslt_crt = crt_gyro_st_update_result(dev);
                            if (rslt == BMI2_OK)
                            {
                                rslt = rslt_crt;
                            }
                        }
                    }
                }
            }
            else
            {
                rslt = BMI2_E_ST_ALREADY_RUNNING;
            }

            if (rslt == BMI2_OK)
            {
                if (gyro_st_crt == BMI2_SELECT_GYRO_SELF_TEST)
                {
                    rslt = gyro_self_test_completed(&gyro_st_result, dev);
                }
            }

            /* Enable Advance power save if disabled while configuring and
             * not when already disabled
             */
            if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
            {
                rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API to set up environment for processing the crt.
 */
static int8_t crt_prepare_setup(struct bmi2_dev *dev)
{
    int8_t rslt;

    /* Variable to select the sensor */
    uint8_t sens_list = BMI2_GYRO;

    rslt = null_ptr_check(dev);

    if (rslt == BMI2_OK)
    {
        /* Disable gyroscope */
        rslt = bmi2_sensor_disable(&sens_list, 1, dev);
    }

    /* Disable FIFO for all sensors */
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, dev);
    }

    if (rslt == BMI2_OK)
    {
        /* Enable accelerometer */
        sens_list = BMI2_ACCEL;
        rslt = bmi2_sensor_enable(&sens_list, 1, dev);
    }

    if (rslt == BMI2_OK)
    {
        /* Disable Abort after 1 msec */
        dev->delay_us(1000, dev->intf_ptr);
        rslt = abort_bmi2(BMI2_DISABLE, dev);
    }

    return rslt;
}

/*!
 * @brief This API is to update the CRT or gyro self-test final result.
 */
static int8_t crt_gyro_st_update_result(struct bmi2_dev *dev)
{
    int8_t rslt;
    struct bmi2_gyr_user_gain_status user_gain_stat = { 0, 0, 0, 0 };

    rslt = null_ptr_check(dev);

    /* CRT status has to be read from the config register map */
    if (rslt == BMI2_OK)
    {
        rslt = get_gyro_gain_update_status(&user_gain_stat, dev);
    }

    if (rslt == BMI2_OK)
    {
        switch (user_gain_stat.g_trigger_status)
        {
            case BMI2_G_TRIGGER_NO_ERROR:

                /* CRT is successful - Reset the Max Burst Length */
                rslt = set_maxburst_len(0, dev);
                break;

            case BMI2_G_TRIGGER_DL_ERROR:

                /* CRT is Download Error - Keep non zero value for Max Burst Length */
                rslt = set_maxburst_len(dev->read_write_len, dev);
                if (rslt == BMI2_OK)
                {
                    rslt = BMI2_E_DL_ERROR;
                }

                break;
            case BMI2_G_TRIGGER_ABORT_ERROR:

                /* Command is aborted either by host via the block bit or due to motion
                 * detection. Keep non zero value for Max Burst Length
                 */
                rslt = set_maxburst_len(dev->read_write_len, dev);
                if (rslt == BMI2_OK)
                {
                    rslt = BMI2_E_ABORT_ERROR;
                }

                break;

            case BMI2_G_TRIGGER_PRECON_ERROR:

                /* Pre-condition to start the feature was not completed. */
                rslt = BMI2_E_PRECON_ERROR;
                break;

            default:
                rslt = BMI2_E_INVALID_STATUS;

                break;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API gets the max burst length.
 */
static int8_t get_maxburst_len(uint8_t *max_burst_len, struct bmi2_dev *dev)
{
    int8_t rslt = BMI2_OK;
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };
    uint8_t idx = 0;
    uint8_t feat_found = 0;
    struct bmi2_feature_config maxburst_length_bytes = { 0, 0, 0 };
    uint8_t aps_stat;

    if ((dev->variant_feature & BMI2_CRT_IN_FIFO_NOT_REQ) != 0)
    {
        *max_burst_len = 0;

        return BMI2_OK;
    }

    /* Get status of advance power save mode */
    aps_stat = dev->aps_status;
    if (aps_stat == BMI2_ENABLE)
    {
        /* Disable advance power save if enabled */
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
    }

    if (rslt == BMI2_OK)
    {
        /* Search for max burst length */
        feat_found = bmi2_extract_input_feat_config(&maxburst_length_bytes, BMI2_MAX_BURST_LEN, dev);
        if (feat_found)
        {
            rslt = bmi2_get_feat_config(maxburst_length_bytes.page, feat_config, dev);
            if (rslt == BMI2_OK)
            {
                /* Define the offset for max burst length */
                idx = maxburst_length_bytes.start_addr;

                /* Get the max burst length */
                *max_burst_len = feat_config[idx];
            }
        }
        else
        {
            rslt = BMI2_E_INVALID_SENSOR;
        }

        /* Enable Advance power save if disabled while configuring and
         * not when already disabled
         */
        if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
        {
            rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API sets the max burst length.
 */
static int8_t set_maxburst_len(const uint16_t write_len_byte, struct bmi2_dev *dev)
{
    int8_t rslt = BMI2_OK;
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };
    uint8_t idx = 0;
    uint8_t reg_addr = 0;
    uint8_t max_burst_len = 0;
    uint8_t feat_found = 0;
    struct bmi2_feature_config maxburst_length_bytes = { 0, 0, 0 };
    uint8_t aps_stat;
    uint16_t burst_len = write_len_byte / 2;

    /* for variant that support crt outside fifo, do not modify the max burst len */
    if ((dev->variant_feature & BMI2_CRT_IN_FIFO_NOT_REQ) != 0)
    {
        return BMI2_OK;
    }

    /* Max burst length is only 1 byte */
    if (burst_len > BMI2_CRT_MAX_BURST_WORD_LENGTH)
    {
        max_burst_len = UINT8_C(0xFF);
    }
    else
    {
        max_burst_len = (uint8_t)burst_len;
    }

    /* Get status of advance power save mode */
    aps_stat = dev->aps_status;
    if (aps_stat == BMI2_ENABLE)
    {
        /* Disable advance power save if enabled */
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
    }

    if (rslt == BMI2_OK)
    {
        /* Search for axis-re-mapping and extract its configuration details */
        feat_found = bmi2_extract_input_feat_config(&maxburst_length_bytes, BMI2_MAX_BURST_LEN, dev);
        if (feat_found)
        {
            /* Get the configuration from the page where axis
             * re-mapping feature resides
             */
            rslt = bmi2_get_feat_config(maxburst_length_bytes.page, feat_config, dev);
            if (rslt == BMI2_OK)
            {
                /* Define the offset in bytes */
                idx = maxburst_length_bytes.start_addr;

                /* update Max burst length */
                feat_config[idx] = max_burst_len;

                /* Update the register address */
                reg_addr = BMI2_FEATURES_REG_ADDR + maxburst_length_bytes.start_addr;

                /* Set the configuration back to the page */
                rslt = bmi2_set_regs(reg_addr, &feat_config[maxburst_length_bytes.start_addr], 2, dev);
            }
        }
        else
        {
            rslt = BMI2_E_INVALID_SENSOR;
        }

        /* Enable Advance power save if disabled while configuring and
         * not when already disabled
         */
        if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
        {
            rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This api is used to trigger the preparation for system for NVM programming.
 */
static int8_t set_nvm_prep_prog(uint8_t nvm_prep, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Array to define the feature configuration */
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };

    /* Variable to define the array offset */
    uint8_t idx = 0;

    /* Variable to set flag */
    uint8_t feat_found;
    uint8_t reg_addr = 0;

    /* Initialize feature configuration for nvm preparation*/
    struct bmi2_feature_config nvm_config = { 0, 0, 0 };

    /* Search for bmi2 gyro self offset correction feature as nvm program preparation feature is
     * present in the same Word and extract its configuration details
     */
    feat_found = bmi2_extract_input_feat_config(&nvm_config, BMI2_NVM_PROG_PREP, dev);
    if (feat_found)
    {
        /* Get the configuration from the page where nvm preparation feature enable feature
         * resides */
        rslt = bmi2_get_feat_config(nvm_config.page, feat_config, dev);
        if (rslt == BMI2_OK)
        {
            /* Define the offset for nvm preparation feature enable */
            idx = nvm_config.start_addr;

            /* update nvm_prog_prep  enable bit */
            feat_config[idx] = BMI2_SET_BITS(feat_config[idx], BMI2_NVM_PREP_FEATURE_EN, nvm_prep);

            /* Update the register address */
            reg_addr = BMI2_FEATURES_REG_ADDR + nvm_config.start_addr - 1;

            /* Set the configuration back to the page */
            rslt = bmi2_set_regs(reg_addr, &feat_config[nvm_config.start_addr - 1], 2, dev);
        }
    }
    else
    {
        rslt = BMI2_E_INVALID_SENSOR;
    }

    return rslt;
}

/*!
 * @brief This api is used to enable the CRT.
 */
static int8_t select_self_test(uint8_t gyro_st_crt, struct bmi2_dev *dev)
{
    int8_t rslt;

    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };

    uint8_t idx = 0;

    uint8_t feat_found;
    uint8_t reg_addr = 0;

    struct bmi2_feature_config gyro_self_test_crt_config = { 0, 0, 0 };

    /* Search for bmi2 crt gyro self-test feature and extract its configuration details */
    feat_found = bmi2_extract_input_feat_config(&gyro_self_test_crt_config, BMI2_CRT_GYRO_SELF_TEST, dev);
    if (feat_found)
    {
        /* Get the configuration from the page where gyro self-test and crt enable feature
         * resides */
        rslt = bmi2_get_feat_config(gyro_self_test_crt_config.page, feat_config, dev);
        if (rslt == BMI2_OK)
        {
            /* Define the offset in bytes */
            idx = gyro_self_test_crt_config.start_addr;

            /* update the gyro self-test crt enable bit */
            feat_config[idx] = BMI2_SET_BIT_POS0(feat_config[idx], BMI2_GYRO_SELF_TEST_CRT_EN, gyro_st_crt);

            /* Update the register address */
            reg_addr = BMI2_FEATURES_REG_ADDR + (gyro_self_test_crt_config.start_addr - 1);

            /* Set the configuration back to the page */
            rslt = bmi2_set_regs(reg_addr, &feat_config[gyro_self_test_crt_config.start_addr - 1], 2, dev);
        }
    }
    else
    {
        rslt = BMI2_E_INVALID_SENSOR;
    }

    return rslt;
}

/*!
 * @brief This api is used to abort ongoing crt or gyro self-test.
 */
int8_t bmi2_abort_crt_gyro_st(struct bmi2_dev *dev)
{
    int8_t rslt = BMI2_OK;
    uint8_t aps_stat;
    uint8_t st_running = 0;
    uint8_t cmd = BMI2_G_TRIGGER_CMD;

    /* Get status of advance power save mode */
    aps_stat = dev->aps_status;
    if (aps_stat == BMI2_ENABLE)
    {
        /* Disable advance power save if enabled */
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
    }

    /* Checking for ST running status */
    if (rslt == BMI2_OK)
    {
        rslt = get_st_running(&st_running, dev);
        if (rslt == BMI2_OK)
        {
            /* ST is not running  */
            if (st_running == 0)
            {
                rslt = BMI2_E_ST_NOT_RUNING;
            }
        }
    }

    if (rslt == BMI2_OK)
    {
        rslt = abort_bmi2(BMI2_ENABLE, dev);
    }

    /* send the g trigger command */
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_set_regs(BMI2_CMD_REG_ADDR, &cmd, 1, dev);
    }

    if (rslt == BMI2_OK)
    {
        /* wait until st_status = 0 or time out is 2 seconds */
        rslt = wait_st_running(BMI2_CRT_WAIT_RUNNING_RETRY_EXECUTION, dev);
    }

    /* Check G trigger status for error */
    if (rslt == BMI2_OK)
    {
        rslt = crt_gyro_st_update_result(dev);
        if (rslt == BMI2_E_ABORT_ERROR)
        {
            rslt = BMI2_OK;
        }
        else
        {
            rslt = BMI2_E_ABORT_ERROR;
        }
    }

    /* Enable Advance power save if disabled while configuring and
     * not when already disabled
     */
    if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
    {
        rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
    }

    return rslt;
}

/*!
 * @brief This api is used to enable/disable abort.
 */
static int8_t abort_bmi2(uint8_t abort_enable, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };

    /* Variable to define the array offset */
    uint8_t idx = 0;

    /* Variable to set flag */
    uint8_t feat_found;
    uint8_t reg_addr = 0;

    /* Initialize feature configuration for blocking a feature */
    struct bmi2_feature_config block_config = { 0, 0, 0 };

    /* Search for bmi2 Abort feature and extract its configuration details */
    feat_found = bmi2_extract_input_feat_config(&block_config, BMI2_ABORT_CRT_GYRO_SELF_TEST, dev);
    if (feat_found)
    {
        /* Get the configuration from the page where abort(block) feature resides */
        rslt = bmi2_get_feat_config(block_config.page, feat_config, dev);
        if (rslt == BMI2_OK)
        {
            /* Define the offset in bytes */
            idx = block_config.start_addr;

            /* update the gyro self-test crt abort enable bit */
            feat_config[idx] = BMI2_SET_BITS(feat_config[idx], BMI2_ABORT_FEATURE_EN, abort_enable);

            /* Update the register address */
            reg_addr = BMI2_FEATURES_REG_ADDR + (block_config.start_addr - 1);

            /* Set the configuration back to the page */
            rslt = bmi2_set_regs(reg_addr, &feat_config[block_config.start_addr - 1], 2, dev);
        }
    }
    else
    {
        rslt = BMI2_E_INVALID_SENSOR;
    }

    return rslt;
}

/*!
 * @brief This api is use to wait till  gyro self-test is completed and update the status of gyro
 * self-test.
 */
static int8_t gyro_self_test_completed(struct bmi2_gyro_self_test_status *gyro_st_result, struct bmi2_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    rslt = bmi2_get_regs(BMI2_GYR_SELF_TEST_AXES_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        gyro_st_result->gyr_st_axes_done = BMI2_GET_BIT_POS0(reg_data, BMI2_GYR_ST_AXES_DONE);
        if (gyro_st_result->gyr_st_axes_done == 0x01)
        {
            gyro_st_result->gyr_axis_x_ok = BMI2_GET_BITS(reg_data, BMI2_GYR_AXIS_X_OK);
            gyro_st_result->gyr_axis_y_ok = BMI2_GET_BITS(reg_data, BMI2_GYR_AXIS_Y_OK);
            gyro_st_result->gyr_axis_z_ok = BMI2_GET_BITS(reg_data, BMI2_GYR_AXIS_Z_OK);
        }
        else
        {
            rslt = BMI2_E_SELF_TEST_NOT_DONE;
        }
    }

    return rslt;
}

/*!
 * @brief This api validates accel foc position as per the range
 */
static int8_t validate_foc_position(uint8_t sens_list,
                                    const struct bmi2_accel_foc_g_value *accel_g_axis,
                                    struct bmi2_sens_axes_data avg_foc_data,
                                    struct bmi2_dev *dev)
{
    int8_t rslt = BMI2_E_INVALID_INPUT;

    if (sens_list == BMI2_ACCEL)
    {
        if (accel_g_axis->x == 1)
        {
            rslt = validate_foc_accel_axis(avg_foc_data.x, dev);
        }
        else if (accel_g_axis->y == 1)
        {
            rslt = validate_foc_accel_axis(avg_foc_data.y, dev);
        }
        else
        {
            rslt = validate_foc_accel_axis(avg_foc_data.z, dev);
        }
    }
    else if (sens_list == BMI2_GYRO)
    {
        if (((avg_foc_data.x >= BMI2_GYRO_FOC_NOISE_LIMIT_NEGATIVE) &&
             (avg_foc_data.x <= BMI2_GYRO_FOC_NOISE_LIMIT_POSITIVE)) &&
            ((avg_foc_data.y >= BMI2_GYRO_FOC_NOISE_LIMIT_NEGATIVE) &&
             (avg_foc_data.y <= BMI2_GYRO_FOC_NOISE_LIMIT_POSITIVE)) &&
            ((avg_foc_data.z >= BMI2_GYRO_FOC_NOISE_LIMIT_NEGATIVE) &&
             (avg_foc_data.z <= BMI2_GYRO_FOC_NOISE_LIMIT_POSITIVE)))
        {
            rslt = BMI2_OK;
        }
        else
        {
            rslt = BMI2_E_INVALID_FOC_POSITION;
        }
    }

    return rslt;
}

/*!
 * @brief This api validates depends on accel foc access input
 */
static int8_t validate_foc_accel_axis(int16_t avg_foc_data, struct bmi2_dev *dev)
{
    struct bmi2_sens_config sens_cfg = { 0 };
    uint8_t range;
    int8_t rslt;

    sens_cfg.type = BMI2_ACCEL;
    rslt = bmi2_get_sensor_config(&sens_cfg, 1, dev);
    range = sens_cfg.cfg.acc.range;

    /* reference LSB value of 16G */
    if ((range == BMI2_ACC_RANGE_2G) && (avg_foc_data > BMI2_ACC_2G_MIN_NOISE_LIMIT) &&
        (avg_foc_data < BMI2_ACC_2G_MAX_NOISE_LIMIT))
    {
        rslt = BMI2_OK;
    }
    /* reference LSB value of 16G */
    else if ((range == BMI2_ACC_RANGE_4G) && (avg_foc_data > BMI2_ACC_4G_MIN_NOISE_LIMIT) &&
             (avg_foc_data < BMI2_ACC_4G_MAX_NOISE_LIMIT))
    {
        rslt = BMI2_OK;
    }
    /* reference LSB value of 16G */
    else if ((range == BMI2_ACC_RANGE_8G) && (avg_foc_data > BMI2_ACC_8G_MIN_NOISE_LIMIT) &&
             (avg_foc_data < BMI2_ACC_8G_MAX_NOISE_LIMIT))
    {
        rslt = BMI2_OK;
    }
    /* reference LSB value of 16G */
    else if ((range == BMI2_ACC_RANGE_16G) && (avg_foc_data > BMI2_ACC_16G_MIN_NOISE_LIMIT) &&
             (avg_foc_data < BMI2_ACC_16G_MAX_NOISE_LIMIT))
    {
        rslt = BMI2_OK;
    }
    else
    {
        rslt = BMI2_E_INVALID_FOC_POSITION;
    }

    return rslt;
}

/*! @brief This api is used for programming the non volatile memory(nvm) */
int8_t bmi2_nvm_prog(struct bmi2_dev *dev)
{
    int8_t rslt = BMI2_OK;

    /* Variable to get the status of advance power save */
    uint8_t aps_stat;
    uint8_t status;
    uint8_t cmd_rdy;
    uint8_t reg_data;
    uint8_t write_timeout = 100;

    /* Get status of advance power save mode */
    aps_stat = dev->aps_status;
    if (aps_stat == BMI2_ENABLE)
    {
        /* Disable advance power save if enabled */
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
    }

    /* Check the Write status and proceed only if there is no ongoing write cycle */
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_get_status(&status, dev);

        cmd_rdy = BMI2_GET_BITS(status, BMI2_CMD_RDY);
        if (cmd_rdy)
        {
            rslt = set_nvm_prep_prog(BMI2_ENABLE, dev);
            if (rslt == BMI2_OK)
            {
                dev->delay_us(40000, dev->intf_ptr);

                /* Set the NVM_CONF.nvm_prog_en bit in order to enable the NVM
                 * programming */
                reg_data = BMI2_NVM_UNLOCK_ENABLE;
                rslt = bmi2_set_regs(BMI2_NVM_CONF_ADDR, &reg_data, 1, dev);
                if (rslt == BMI2_OK)
                {
                    /* Send NVM prog command to command register */
                    reg_data = BMI2_NVM_PROG_CMD;
                    rslt = bmi2_set_regs(BMI2_CMD_REG_ADDR, &reg_data, 1, dev);
                }

                /* Wait till write operation is completed */
                if (rslt == BMI2_OK)
                {
                    while (write_timeout--)
                    {
                        rslt = bmi2_get_status(&status, dev);
                        if (rslt == BMI2_OK)
                        {
                            cmd_rdy = BMI2_GET_BITS(status, BMI2_CMD_RDY);

                            /* Nvm is complete once cmd_rdy is 1, break if 1 */
                            if (cmd_rdy)
                            {
                                break;
                            }

                            /* Wait till cmd_rdy becomes 1 indicating
                             * nvm process completes */
                            dev->delay_us(20000, dev->intf_ptr);
                        }
                    }
                }

                if ((rslt == BMI2_OK) && (cmd_rdy != BMI2_TRUE))
                {
                    rslt = BMI2_E_WRITE_CYCLE_ONGOING;
                }
            }
        }
        else
        {
            rslt = BMI2_E_WRITE_CYCLE_ONGOING;
        }
    }

    if (rslt == BMI2_OK)
    {
        /* perform soft reset */
        rslt = bmi2_soft_reset(dev);
    }

    /* Enable Advance power save if disabled while configuring and not when already disabled */
    if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
    {
        rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
    }

    return rslt;
}

/*!
 * @brief This API reads and provides average for 128 samples of sensor data for foc operation
 * gyro.
 */
static int8_t get_average_of_sensor_data(uint8_t sens_list,
                                         struct bmi2_foc_temp_value *temp_foc_data,
                                         struct bmi2_dev *dev)
{
    int8_t rslt = 0;
    struct bmi2_sensor_data sensor_data = { 0 };
    uint8_t sample_count = 0;
    uint8_t datardy_try_cnt;
    uint8_t drdy_status = 0;
    uint8_t sensor_drdy = 0;

    sensor_data.type = sens_list;
    if (sens_list == BMI2_ACCEL)
    {
        sensor_drdy = BMI2_DRDY_ACC;
    }
    else
    {
        sensor_drdy = BMI2_DRDY_GYR;
    }

    /* Read sensor values before FOC */
    while (sample_count < BMI2_FOC_SAMPLE_LIMIT)
    {
        datardy_try_cnt = 5;
        do
        {
            dev->delay_us(20000, dev->intf_ptr);
            rslt = bmi2_get_status(&drdy_status, dev);
            datardy_try_cnt--;
        } while ((rslt == BMI2_OK) && (!(drdy_status & sensor_drdy)) && (datardy_try_cnt));

        if ((rslt != BMI2_OK) || (datardy_try_cnt == 0))
        {
            rslt = BMI2_E_DATA_RDY_INT_FAILED;
            break;
        }

        rslt = bmi2_get_sensor_data(&sensor_data, 1, dev);

        if (rslt == BMI2_OK)
        {
            if (sensor_data.type == BMI2_ACCEL)
            {
                temp_foc_data->x += sensor_data.sens_data.acc.x;
                temp_foc_data->y += sensor_data.sens_data.acc.y;
                temp_foc_data->z += sensor_data.sens_data.acc.z;
            }
            else if (sensor_data.type == BMI2_GYRO)
            {
                temp_foc_data->x += sensor_data.sens_data.gyr.x;
                temp_foc_data->y += sensor_data.sens_data.gyr.y;
                temp_foc_data->z += sensor_data.sens_data.gyr.z;
            }
        }
        else
        {
            break;
        }

        sample_count++;
    }

    if (rslt == BMI2_OK)
    {
        temp_foc_data->x = (temp_foc_data->x / BMI2_FOC_SAMPLE_LIMIT);
        temp_foc_data->y = (temp_foc_data->y / BMI2_FOC_SAMPLE_LIMIT);
        temp_foc_data->z = (temp_foc_data->z / BMI2_FOC_SAMPLE_LIMIT);
    }

    return rslt;
}

/*!
 * @brief This internal API extract the identification feature from the DMR page
 * and retrieve the config file major and minor version.
 */
static int8_t extract_config_file(uint8_t *config_major, uint8_t *config_minor, struct bmi2_dev *dev)
{
    /* Variable to define the result */
    int8_t rslt = BMI2_OK;

    /* Variable to define the array offset */
    uint8_t idx = 0;

    /* Variable to define LSB */
    uint16_t lsb = 0;

    /* Variable to define MSB */
    uint16_t msb = 0;

    /* Variable to define a word */
    uint16_t lsb_msb = 0;

    /* Variable to set flag */
    uint8_t feat_found;

    /* Variable to define advance power save mode status */
    uint8_t aps_stat;

    /* Array to define the feature configuration */
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };

    /* Initialize feature configuration for config file identification */
    struct bmi2_feature_config config_id = { 0, 0, 0 };

    /* Check the power mode status */
    aps_stat = dev->aps_status;
    if (aps_stat == BMI2_ENABLE)
    {
        /* Disable advance power save if enabled */
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
    }

    if (rslt == BMI2_OK)
    {
        /* Search for config file identification feature and extract its configuration
         * details */
        feat_found = bmi2_extract_input_feat_config(&config_id, BMI2_CONFIG_ID, dev);
        if (feat_found)
        {
            /* Get the configuration from the page where config file identification
             * feature resides */
            rslt = bmi2_get_feat_config(config_id.page, feat_config, dev);
            if (rslt == BMI2_OK)
            {
                /* Define the offset for config file identification */
                idx = config_id.start_addr;

                /* Get word to calculate config file identification */
                lsb = (uint16_t) feat_config[idx++];
                msb = ((uint16_t) feat_config[idx++] << 8);
                lsb_msb = lsb | msb;

                /* Get major and minor version */
                *config_major = BMI2_GET_BITS(lsb_msb, BMI2_CONFIG_MAJOR);
                *config_minor = BMI2_GET_BIT_POS0(lsb, BMI2_CONFIG_MINOR);
            }
        }

        /* Enable Advance power save if disabled while configuring and
         * not when already disabled
         */
        if ((aps_stat == BMI2_ENABLE) && (rslt == BMI2_OK))
        {
            rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev);
        }
    }
    else
    {
        rslt = BMI2_E_INVALID_SENSOR;
    }

    return rslt;
}

/*!
 *@brief This internal API is used to map the interrupts to the sensor.
 */
static void extract_feat_int_map(struct bmi2_map_int *map_int, uint8_t type, const struct bmi2_dev *dev)
{
    /* Variable to define loop */
    uint8_t loop = 0;

    /* Search for the interrupts from the input configuration array */
    while (loop < dev->sens_int_map)
    {
        if (dev->map_int[loop].type == type)
        {
            *map_int = dev->map_int[loop];
            break;
        }

        loop++;
    }
}

/*!
 * @brief This internal API gets the saturation status for the gyroscope user
 * gain update.
 */
static int8_t get_gyro_gain_update_status(struct bmi2_gyr_user_gain_status *user_gain_stat, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };

    /* Variables to define index */
    uint8_t idx = 0;

    /* Variable to set flag */
    uint8_t feat_found;

    /* Initialize feature output for gyroscope user gain status */
    struct bmi2_feature_config user_gain_cfg = { 0, 0, 0 };

    /* Search for gyroscope user gain status output feature and extract its
     * configuration details
     */
    feat_found = extract_output_feat_config(&user_gain_cfg, BMI2_GYRO_GAIN_UPDATE, dev);
    if (feat_found)
    {
        /* Get the feature output configuration for gyroscope user gain  status */
        rslt = bmi2_get_feat_config(user_gain_cfg.page, feat_config, dev);
        if (rslt == BMI2_OK)
        {
            /* Define the offset in bytes for gyroscope user gain status */
            idx = user_gain_cfg.start_addr;

            /* Get the saturation status for x-axis */
            user_gain_stat->sat_x = BMI2_GET_BIT_POS0(feat_config[idx], BMI2_GYR_USER_GAIN_SAT_STAT_X);

            /* Get the saturation status for y-axis */
            user_gain_stat->sat_y = BMI2_GET_BITS(feat_config[idx], BMI2_GYR_USER_GAIN_SAT_STAT_Y);

            /* Get the saturation status for z-axis */
            user_gain_stat->sat_z = BMI2_GET_BITS(feat_config[idx], BMI2_GYR_USER_GAIN_SAT_STAT_Z);

            /* Get g trigger status */
            user_gain_stat->g_trigger_status = BMI2_GET_BITS(feat_config[idx], BMI2_G_TRIGGER_STAT);
        }
    }
    else
    {
        rslt = BMI2_E_INVALID_SENSOR;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to extract the output feature configuration
 * details from the look-up table.
 */
static uint8_t extract_output_feat_config(struct bmi2_feature_config *feat_output,
                                          uint8_t type,
                                          const struct bmi2_dev *dev)
{
    /* Variable to define loop */
    uint8_t loop = 0;

    /* Variable to set flag */
    uint8_t feat_found = BMI2_FALSE;

    /* Search for the output feature from the output configuration array */
    while (loop < dev->out_sens)
    {
        if (dev->feat_output[loop].type == type)
        {
            *feat_output = dev->feat_output[loop];
            feat_found = BMI2_TRUE;
            break;
        }

        loop++;
    }

    /* Return flag */
    return feat_found;
}

/*!
 * @brief This internal API gets the cross sensitivity coefficient between
 * gyroscope's X and Z axes.
 */
static int8_t get_gyro_cross_sense(int16_t *cross_sense, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint8_t feat_config[BMI2_FEAT_SIZE_IN_BYTES] = { 0 };

    /* Variable to define index */
    uint8_t idx = 0;

    /* Variable to set flag */
    uint8_t feat_found;

    uint8_t corr_fact_zx;

    /* Initialize feature output for gyroscope cross sensitivity */
    struct bmi2_feature_config cross_sense_out_config = { 0, 0, 0 };

    if (dev->variant_feature & BMI2_MAXIMUM_FIFO_VARIANT)
    {
        /* For maximum_fifo variant fetch the correction factor from GPIO0 */
        rslt = bmi2_get_regs(BMI2_GYR_CAS_GPIO0_ADDR, &corr_fact_zx, 1, dev);
        if (rslt == BMI2_OK)
        {
            /* Get the gyroscope cross sensitivity coefficient */
            if (corr_fact_zx & BMI2_GYRO_CROSS_AXES_SENSE_SIGN_BIT_MASK)
            {
                *cross_sense = (int16_t)(((int16_t)corr_fact_zx) - 128);
            }
            else
            {
                *cross_sense = (int16_t)(corr_fact_zx);
            }
        }
    }
    else
    {
        /* Search for gyroscope cross sensitivity feature and extract its configuration details */
        feat_found = extract_output_feat_config(&cross_sense_out_config, BMI2_GYRO_CROSS_SENSE, dev);
        if (feat_found)
        {
            /* Get the feature output configuration for gyroscope cross sensitivity
             * feature */
            rslt = bmi2_get_feat_config(cross_sense_out_config.page, feat_config, dev);
            if (rslt == BMI2_OK)
            {
                /* Define the offset in bytes for gyroscope cross sensitivity output */
                idx = cross_sense_out_config.start_addr;

                /* discard the MSB as GYR_CAS is of only 7 bit */
                feat_config[idx] = feat_config[idx] & BMI2_GYRO_CROSS_AXES_SENSE_MASK;

                /* Get the gyroscope cross sensitivity coefficient */
                if (feat_config[idx] & BMI2_GYRO_CROSS_AXES_SENSE_SIGN_BIT_MASK)
                {
                    *cross_sense = (int16_t)(((int16_t)feat_config[idx]) - 128);
                }
                else
                {
                    *cross_sense = (int16_t)(feat_config[idx]);
                }
            }
        }
        else
        {
            rslt = BMI2_E_INVALID_SENSOR;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API selects the sensor/features to be enabled or
 * disabled.
 */
static int8_t select_sensor(const uint8_t *sens_list, uint8_t n_sens, uint64_t *sensor_sel)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OK;

    /* Variable to define loop */
    uint8_t count;

    for (count = 0; count < n_sens; count++)
    {
        switch (sens_list[count])
        {
            case BMI2_ACCEL:
                *sensor_sel |= BMI2_ACCEL_SENS_SEL;
                break;
            case BMI2_GYRO:
                *sensor_sel |= BMI2_GYRO_SENS_SEL;
                break;
            case BMI2_AUX:
                *sensor_sel |= BMI2_AUX_SENS_SEL;
                break;
            case BMI2_TEMP:
                *sensor_sel |= BMI2_TEMP_SENS_SEL;
                break;
            default:
                rslt = BMI2_E_INVALID_SENSOR;
                break;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API enables the selected sensor/features.
 */
static int8_t sensor_enable(uint64_t sensor_sel, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store register values */
    uint8_t reg_data = 0;

    rslt = bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        /* Enable accelerometer */
        if (sensor_sel & BMI2_ACCEL_SENS_SEL)
        {
            reg_data = BMI2_SET_BITS(reg_data, BMI2_ACC_EN, BMI2_ENABLE);
        }

        /* Enable gyroscope */
        if (sensor_sel & BMI2_GYRO_SENS_SEL)
        {
            reg_data = BMI2_SET_BITS(reg_data, BMI2_GYR_EN, BMI2_ENABLE);
        }

        /* Enable auxiliary sensor */
        if (sensor_sel & BMI2_AUX_SENS_SEL)
        {
            reg_data = BMI2_SET_BIT_POS0(reg_data, BMI2_AUX_EN, BMI2_ENABLE);
        }

        /* Enable temperature sensor */
        if (sensor_sel & BMI2_TEMP_SENS_SEL)
        {
            reg_data = BMI2_SET_BITS(reg_data, BMI2_TEMP_EN, BMI2_ENABLE);
        }

        /* Enable the sensors that are set in the power control register */
        if (sensor_sel & BMI2_MAIN_SENSORS)
        {
            rslt = bmi2_set_regs(BMI2_PWR_CTRL_ADDR, &reg_data, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API disables the selected sensors/features.
 */
static int8_t sensor_disable(uint64_t sensor_sel, struct bmi2_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store register values */
    uint8_t reg_data = 0;

    rslt = bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &reg_data, 1, dev);
    if (rslt == BMI2_OK)
    {
        /* Disable accelerometer */
        if (sensor_sel & BMI2_ACCEL_SENS_SEL)
        {
            reg_data = BMI2_SET_BIT_VAL0(reg_data, BMI2_ACC_EN);
        }

        /* Disable gyroscope */
        if (sensor_sel & BMI2_GYRO_SENS_SEL)
        {
            reg_data = BMI2_SET_BIT_VAL0(reg_data, BMI2_GYR_EN);
        }

        /* Disable auxiliary sensor */
        if (sensor_sel & BMI2_AUX_SENS_SEL)
        {
            reg_data = BMI2_SET_BIT_VAL0(reg_data, BMI2_AUX_EN);
        }

        /* Disable temperature sensor */
        if (sensor_sel & BMI2_TEMP_SENS_SEL)
        {
            reg_data = BMI2_SET_BIT_VAL0(reg_data, BMI2_TEMP_EN);
        }

        /* Enable the sensors that are set in the power control register */
        if (sensor_sel & BMI2_MAIN_SENSORS)
        {
            rslt = bmi2_set_regs(BMI2_PWR_CTRL_ADDR, &reg_data, 1, dev);
        }
    }

    return rslt;
}

/*! @endcond */
