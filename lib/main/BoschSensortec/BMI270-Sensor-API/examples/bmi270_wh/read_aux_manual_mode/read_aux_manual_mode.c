/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi270_wh.h"
#include "bmm150.h"
#include "coines.h"
#include "common.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! Macros to select the sensors                   */
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)
#define AUX            UINT8_C(0x02)

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*****************************************************************************/
/*!         Structure declaration                                            */

/* Sensor initialization configuration. */
struct bmi2_dev bmi2_dev;

/******************************************************************************/
/*!                 Functions                                                 */

/**
 * user_aux_read - Reads data from auxiliary sensor in manual mode.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] aux_data    : Aux data pointer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
static int8_t user_aux_read(uint8_t reg_addr, uint8_t *aux_data, uint32_t len, void *intf_ptr);

/**
 * user_aux_write - Writes data to the auxiliary sensor in manual mode.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] aux_data    : Aux data pointer to store the data being written.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
static int8_t user_aux_write(uint8_t reg_addr, const uint8_t *aux_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period_us    : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 */
static void user_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Gravity.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to select the pull-up resistor which is set to trim register */
    uint8_t regdata;

    /* Variable to define limit to print aux data. */
    uint8_t limit = 20;

    /* Accel, Gyro and Aux sensors are listed in array. */
    uint8_t sensor_list[3] = { BMI2_ACCEL, BMI2_GYRO, BMI2_AUX };

    /* Sensor initialization configuration. */
    struct bmm150_dev bmm150_dev;

    /* bmm150 settings configuration */
    struct bmm150_settings settings;

    /* bmm150 magnetometer data */
    struct bmm150_mag_data mag_data;

    /* Structure to define the type of the sensor and its configurations. */
    struct bmi2_sens_config config[3];

    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sensor_data sensor_data[2];

    /* Variables to define read the accel and gyro data in float */
    float x = 0, y = 0, z = 0;

    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;
    config[AUX].type = BMI2_AUX;

    sensor_data[ACCEL].type = BMI2_ACCEL;
    sensor_data[GYRO].type = BMI2_GYRO;

    /* Array of eight bytes to store x, y, z and r axis aux data. */
    uint8_t aux_data[8] = { 0 };

    /* To enable the i2c interface settings for bmm150. */
    uint8_t bmm150_dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
    bmm150_dev.intf_ptr = &bmm150_dev_addr;
    bmm150_dev.read = user_aux_read;
    bmm150_dev.write = user_aux_write;
    bmm150_dev.delay_us = user_delay_us;

    /* As per datasheet, aux interface with bmi270_wh will support only for I2C */
    bmm150_dev.intf = BMM150_I2C_INTF;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi2_dev, BMI2_SPI_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270_wh. */
    rslt = bmi270_wh_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Pull-up resistor 2k is set to the trim register */
    regdata = BMI2_ASDA_PUPSEL_2K;
    rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Enable the accel, gyro and aux sensor. */
    rslt = bmi270_wh_sensor_enable(sensor_list, 3, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_wh_get_sensor_config(config, 3, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Configurations for accel. */
    config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

    /* Configurations for gyro. */
    config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[GYRO].cfg.gyr.noise_perf = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    /* Configurations for aux. */
    config[AUX].cfg.aux.odr = BMI2_AUX_ODR_100HZ;
    config[AUX].cfg.aux.aux_en = BMI2_ENABLE;
    config[AUX].cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    config[AUX].cfg.aux.manual_en = BMI2_ENABLE;
    config[AUX].cfg.aux.fcu_write_en = BMI2_ENABLE;
    config[AUX].cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;

    /* Set new configurations for accel, gyro and aux. */
    rslt = bmi270_wh_set_sensor_config(config, 3, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmm150. */
    rslt = bmm150_init(&bmm150_dev);
    bmi2_error_codes_print_result(rslt);

    /* Set the power mode to normal mode. */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, &bmm150_dev);
    bmi2_error_codes_print_result(rslt);

    printf("MAGNETOMETER, ACCEL AND GYRO DATA IN MANUAL MODE\n");

    if (bmm150_dev.chip_id == BMM150_CHIP_ID)
    {
        while (limit--)
        {
            /* Delay has been added to get aux, accel and gyro data at the interval of every 0.05 second. */
            bmi2_dev.delay_us(50000, bmi2_dev.intf_ptr);

            rslt = bmi270_wh_get_sensor_data(sensor_data, 2, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            if (rslt == BMI2_OK)
            {
                printf("\nAcc_x = %d\t", sensor_data[ACCEL].sens_data.acc.x);
                printf("Acc_y = %d\t", sensor_data[ACCEL].sens_data.acc.y);
                printf("Acc_z = %d", sensor_data[ACCEL].sens_data.acc.z);

                /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                x = lsb_to_mps2(sensor_data[ACCEL].sens_data.acc.x, 2, bmi2_dev.resolution);
                y = lsb_to_mps2(sensor_data[ACCEL].sens_data.acc.y, 2, bmi2_dev.resolution);
                z = lsb_to_mps2(sensor_data[ACCEL].sens_data.acc.z, 2, bmi2_dev.resolution);

                /* Print the data in m/s2. */
                printf("\nAcc_ms2_X = %4.2f, Acc_ms2_Y = %4.2f, Acc_ms2_Z = %4.2f\n", x, y, z);

                printf("\nGyr_X = %d\t", sensor_data[GYRO].sens_data.gyr.x);
                printf("Gyr_Y = %d\t", sensor_data[GYRO].sens_data.gyr.y);
                printf("Gyr_Z = %d", sensor_data[GYRO].sens_data.gyr.z);

                /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                x = lsb_to_dps(sensor_data[GYRO].sens_data.gyr.x, 2000, bmi2_dev.resolution);
                y = lsb_to_dps(sensor_data[GYRO].sens_data.gyr.y, 2000, bmi2_dev.resolution);
                z = lsb_to_dps(sensor_data[GYRO].sens_data.gyr.z, 2000, bmi2_dev.resolution);

                /* Print the data in dps. */
                printf("\nGyro_DPS_X = %4.2f, Gyro_DPS_Y = %4.2f, Gyro_DPS_Z = %4.2f\n", x, y, z);
            }

            /* Read aux data from the bmm150 data registers. */
            rslt = bmi2_read_aux_man_mode(BMM150_REG_DATA_X_LSB, aux_data, 8, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);
            if (rslt == BMI2_OK)
            {
                /* Compensating the raw auxiliary data available from the BMM150 API. */
                rslt = bmm150_aux_mag_data(aux_data, &mag_data, &bmm150_dev);
                bmi2_error_codes_print_result(rslt);
                printf("Mag_x_axis = %d \t Mag_y_axis = %d \t Mag_z_axis = %d \t\n", mag_data.x, mag_data.y,
                       mag_data.z);
            }
        }
    }

    bmi2_coines_deinit();

    return rslt;
}

/*!
 * @brief This function reads the data from auxiliary sensor in manual mode.
 */
int8_t user_aux_read(uint8_t reg_addr, uint8_t *aux_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt;

    /* Discarding the parameter id as it is redundant */
    rslt = bmi2_read_aux_man_mode(reg_addr, aux_data, len, &bmi2_dev);

    return rslt;
}

/*!
 * @brief This function writes the data to auxiliary sensor in manual mode.
 */
int8_t user_aux_write(uint8_t reg_addr, const uint8_t *aux_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt;

    /* Discarding the parameter id as it is redundant */
    rslt = bmi2_write_aux_man_mode(reg_addr, aux_data, len, &bmi2_dev);

    return rslt;
}

/*!
 * Delay function map to COINES platform
 */
static void user_delay_us(uint32_t period, void *intf_ptr)
{
    coines_delay_usec(period);
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI2_GYR_RANGE_2000)) * (val);
}
