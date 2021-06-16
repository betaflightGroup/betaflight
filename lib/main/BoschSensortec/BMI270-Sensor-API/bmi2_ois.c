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
 * @file       bmi2_ois.c
 * @date       2020-05-05
 * @version    v2.23.2
 *
 */

/******************************************************************************/

/*!  @name          Header Files                                  */
/******************************************************************************/
#include "bmi2_ois.h"

/******************************************************************************/

/*!         Local Function Prototypes
 ******************************************************************************/

/*!
 * @brief This internal API gets the OIS accelerometer and the gyroscope data.
 *
 * @param[out] ois_data                     : Structure instance of bmi2_sens_axes_data.
 * @param[in] reg_addr                      : Register address where data is stored.
 * @param[in] ois_dev                       : Structure instance of bmi2_ois_dev.
 * @param[in] ois_gyr_cross_sens_zx         :"gyroscope cross sensitivity value which was calculated during
 * bmi2xy_init(), refer the example ois_accel_gyro.c for more info"
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_ois_acc_gyr_data(struct bmi2_ois_sens_axes_data *ois_data,
                                   uint8_t reg_addr,
                                   struct bmi2_ois_dev *ois_dev,
                                   int16_t ois_gyr_cross_sens_zx);

/*!
 * @brief This internal API is used to validate the OIS device pointer for null
 * conditions.
 *
 * @param[in] ois_dev : Structure instance of bmi2_ois_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t null_ptr_check(const struct bmi2_ois_dev *ois_dev);

/*!
 * @brief This internal API corrects the gyroscope cross-axis sensitivity
 * between the z and the x axis.
 *
 * @param[in] ois_dev               : Structure instance of bmi2_ois_dev.
 * @param[in] ois_gyr_cross_sens_zx : "gyroscope cross sensitivity value which was calculated during bmi2xy_init(),
 * refer the example ois_accel_gyro.c for more info"
 *
 */
static void comp_gyro_cross_axis_sensitivity(struct bmi2_ois_sens_axes_data *ois_data, int16_t ois_gyr_cross_sens_zx);

/******************************************************************************/
/*!  @name      User Interface Definitions                            */
/******************************************************************************/

/*!
 * @brief This API reads the data from the given OIS register address of bmi2
 * sensor.
 */
int8_t bmi2_ois_get_regs(uint8_t ois_reg_addr, uint8_t *ois_reg_data, uint16_t data_len, struct bmi2_ois_dev *ois_dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define dummy byte for SPI read */
    uint8_t dummy_byte = 1;

    /* Variable to define temporary length */
    uint16_t temp_len = data_len + dummy_byte;

    /* Variable to define temporary buffer */
    uint8_t temp_buf[temp_len];

    /* Variable to index bytes read */
    uint16_t index = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(ois_dev);
    if ((rslt == BMI2_OIS_OK) && (ois_reg_data != NULL))
    {
        /* Configuring reg_addr for SPI Interface */
        ois_reg_addr = (ois_reg_addr | BMI2_OIS_SPI_RD_MASK);

        /* Read from OIS register through OIS interface */
        ois_dev->intf_rslt = ois_dev->ois_read(ois_reg_addr, temp_buf, temp_len, ois_dev->intf_ptr);
        if (ois_dev->intf_rslt == BMI2_INTF_RET_SUCCESS)
        {
            /* Read the data from the position next to dummy byte */
            while (index < data_len)
            {
                ois_reg_data[index] = temp_buf[index + dummy_byte];
                index++;
            }
        }
        else
        {
            rslt = BMI2_OIS_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMI2_OIS_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes data to the given OIS register address of bmi2 sensor.
 */
int8_t bmi2_ois_set_regs(uint8_t ois_reg_addr,
                         const uint8_t *ois_reg_data,
                         uint16_t data_len,
                         struct bmi2_ois_dev *ois_dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(ois_dev);
    if ((rslt == BMI2_OIS_OK) && (ois_reg_data != NULL))
    {
        /* Configuring reg_addr for SPI Interface */
        ois_reg_addr = (ois_reg_addr & BMI2_OIS_SPI_WR_MASK);

        /* Burst write to OIS register through OIS interface */
        ois_dev->intf_rslt = ois_dev->ois_write(ois_reg_addr, ois_reg_data, data_len, ois_dev->intf_ptr);
        if (ois_dev->intf_rslt != BMI2_INTF_RET_SUCCESS)
        {
            rslt = BMI2_OIS_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMI2_OIS_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API enables/disables accelerometer/gyroscope data read through
 * OIS interface.
 */
int8_t bmi2_ois_set_config(struct bmi2_ois_dev *ois_dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(ois_dev);
    if (rslt == BMI2_OIS_OK)
    {
        rslt = bmi2_ois_get_regs(BMI2_OIS_CONFIG_ADDR, &reg_data, 1, ois_dev);
        if (rslt == BMI2_OIS_OK)
        {
            /* Enable/Disable Low pass filter */
            reg_data = BMI2_SET_BIT_POS0(reg_data, BMI2_OIS_LP_FILTER_EN, ois_dev->lp_filter_en);

            /* Configures Low pass filter cut-off frequency */
            reg_data = BMI2_OIS_SET_BITS(reg_data, BMI2_OIS_LP_FILTER_CONFIG, ois_dev->lp_filter_config);

            /* Low pass filter - mute on secondary interface */
            reg_data = BMI2_OIS_SET_BITS(reg_data, BMI2_OIS_LP_FILTER_MUTE, ois_dev->lp_filter_mute);

            /* Enable/Disable ois on accelerometer */
            reg_data = BMI2_OIS_SET_BITS(reg_data, BMI2_OIS_ACC_EN, ois_dev->acc_en);

            /* Enable/Disable ois on gyroscope */
            reg_data = BMI2_OIS_SET_BITS(reg_data, BMI2_OIS_GYR_EN, ois_dev->gyr_en);

            /* Set the OIS configurations */
            rslt = bmi2_ois_set_regs(BMI2_OIS_CONFIG_ADDR, &reg_data, 1, ois_dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the status of accelerometer/gyroscope enable for data
 * read through OIS interface.
 */
int8_t bmi2_ois_get_config(struct bmi2_ois_dev *ois_dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(ois_dev);
    if (rslt == BMI2_OIS_OK)
    {
        rslt = bmi2_ois_get_regs(BMI2_OIS_CONFIG_ADDR, &reg_data, 1, ois_dev);
        if (rslt == BMI2_OIS_OK)
        {
            /* Get the status of Low pass filter enable */
            ois_dev->lp_filter_en = BMI2_GET_BIT_POS0(reg_data, BMI2_OIS_LP_FILTER_EN);

            /* Get the status of Low pass filter cut-off frequency */
            ois_dev->lp_filter_config = BMI2_OIS_GET_BITS(reg_data, BMI2_OIS_LP_FILTER_CONFIG);

            /* Get the status of Low pass filter mute */
            ois_dev->lp_filter_mute = BMI2_OIS_GET_BITS(reg_data, BMI2_OIS_LP_FILTER_MUTE);

            /* Get the status of accelerometer enable */
            ois_dev->acc_en = BMI2_OIS_GET_BITS(reg_data, BMI2_OIS_ACC_EN);

            /* Get the status of gyroscope enable */
            ois_dev->gyr_en = BMI2_OIS_GET_BITS(reg_data, BMI2_OIS_GYR_EN);
        }
    }

    return rslt;
}

/*!
 * @brief This API reads accelerometer/gyroscope data through OIS interface.
 */
int8_t bmi2_ois_read_data(const uint8_t *sens_sel,
                          uint8_t n_sens,
                          struct bmi2_ois_dev *ois_dev,
                          int16_t gyr_cross_sens_zx)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define loop */
    uint8_t loop = 0;

    /* Variable to update register address */
    uint8_t reg_addr = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(ois_dev);
    if ((rslt == BMI2_OIS_OK) && (sens_sel != NULL))
    {

        for (loop = 0; loop < n_sens; loop++)
        {
            switch (sens_sel[loop])
            {
                case BMI2_OIS_ACCEL:

                    /* Update OIS accelerometer address */
                    reg_addr = BMI2_OIS_ACC_X_LSB_ADDR;

                    /* Get OIS accelerometer data */
                    rslt = get_ois_acc_gyr_data(&ois_dev->acc_data, reg_addr, ois_dev, 0);
                    break;
                case BMI2_OIS_GYRO:

                    /* Update OIS gyroscope address */
                    reg_addr = BMI2_OIS_GYR_X_LSB_ADDR;

                    /* Get OIS gyroscope data */
                    rslt = get_ois_acc_gyr_data(&ois_dev->gyr_data, reg_addr, ois_dev, gyr_cross_sens_zx);
                    break;
                default:
                    rslt = BMI2_OIS_E_INVALID_SENSOR;
                    break;
            }

            /* Return error if any of the get sensor data fails */
            if (rslt != BMI2_OIS_OK)
            {
                break;
            }
        }
    }
    else
    {
        rslt = BMI2_OIS_E_NULL_PTR;
    }

    return rslt;
}

/***************************************************************************/

/*!         Local Function Definitions
 ****************************************************************************/

/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/*!
 * @brief This internal API gets the accelerometer and the gyroscope data.
 */
static int8_t get_ois_acc_gyr_data(struct bmi2_ois_sens_axes_data *ois_data,
                                   uint8_t reg_addr,
                                   struct bmi2_ois_dev *ois_dev,
                                   int16_t ois_gyr_cross_sens_zx)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variables to store MSB value */
    uint8_t msb;

    /* Variables to store LSB value */
    uint8_t lsb;

    /* Variables to store both MSB and LSB value */
    uint16_t msb_lsb;

    /* Variables to define index */
    uint8_t index = 0;

    /* Array to define data stored in register */
    uint8_t reg_data[BMI2_OIS_ACC_GYR_NUM_BYTES] = { 0 };

    /* Read the sensor data */
    rslt = bmi2_ois_get_regs(reg_addr, reg_data, BMI2_OIS_ACC_GYR_NUM_BYTES, ois_dev);
    if (rslt == BMI2_OIS_OK)
    {
        /* Read x-axis data */
        lsb = reg_data[index++];
        msb = reg_data[index++];
        msb_lsb = ((uint16_t)msb << 8) | (uint16_t)lsb;
        ois_data->x = (int16_t)msb_lsb;

        /* Read y-axis data */
        lsb = reg_data[index++];
        msb = reg_data[index++];
        msb_lsb = ((uint16_t)msb << 8) | (uint16_t)lsb;
        ois_data->y = (int16_t)msb_lsb;

        /* Read z-axis data */
        lsb = reg_data[index++];
        msb = reg_data[index++];
        msb_lsb = ((uint16_t)msb << 8) | (uint16_t)lsb;
        ois_data->z = (int16_t)msb_lsb;

        comp_gyro_cross_axis_sensitivity(ois_data, ois_gyr_cross_sens_zx);
    }

    return rslt;
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi2_ois_dev *ois_dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OIS_OK;

    if ((ois_dev == NULL) || (ois_dev->ois_read == NULL) || (ois_dev->ois_write == NULL) ||
        (ois_dev->ois_delay_us == NULL))
    {
        /* Device structure pointer is NULL */
        rslt = BMI2_OIS_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API corrects the gyroscope cross-axis sensitivity
 * between the z and the x axis.
 */
static void comp_gyro_cross_axis_sensitivity(struct bmi2_ois_sens_axes_data *ois_data, int16_t ois_gyr_cross_sens_zx)
{

    /* Get the compensated gyroscope x-axis */
    ois_data->x = ois_data->x - (int16_t)(((int32_t)ois_gyr_cross_sens_zx * (int32_t)ois_data->z) / 512);
}

/*! @endcond */
