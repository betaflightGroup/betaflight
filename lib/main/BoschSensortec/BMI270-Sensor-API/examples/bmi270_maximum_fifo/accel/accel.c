/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi270_maximum_fifo.h"
#include "common.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] dev       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_config(struct bmi2_dev *bmi2_dev);

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

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to define limit to print accel data. */
    uint8_t limit = 20;

    /* Assign accel sensor to variable. */
    uint8_t sensor_list = BMI2_ACCEL;

    /* Sensor initialization configuration. */
    struct bmi2_dev bmi2_dev;

    /* Create an instance of sensor data structure. */
    struct bmi2_sensor_data sensor_data = { 0 };

    /* Initialize the interrupt status of accel. */
    uint16_t int_status = 0;

    uint8_t indx = 0;
    float x = 0, y = 0, z = 0;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi2_dev, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270_maximum_fifo. */
    rslt = bmi270_maximum_fifo_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* Enable the accel sensor. */
        rslt = bmi2_sensor_enable(&sensor_list, 1, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK)
        {
            /* Accel configuration settings. */
            rslt = set_accel_config(&bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            /* Assign accel sensor. */
            sensor_data.type = BMI2_ACCEL;
            printf("Accel and m/s2 data \n");
            printf("Accel data collected at 2G Range with 16-bit resolution\n");

            /* Loop to print the accel data when interrupt occurs. */
            while (indx <= limit)
            {
                /* To get the status of accel data ready interrupt. */
                rslt = bmi2_get_int_status(&int_status, &bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                /* To check the accel data ready interrupt status and print the status for 10 samples. */
                if (int_status & BMI2_ACC_DRDY_INT_MASK)
                {
                    /* Get accelerometer data for x, y and z axis. */
                    rslt = bmi2_get_sensor_data(&sensor_data, 1, &bmi2_dev);
                    bmi2_error_codes_print_result(rslt);
                    printf("\nAcc_X = %d\t", sensor_data.sens_data.acc.x);
                    printf("Acc_Y = %d\t", sensor_data.sens_data.acc.y);
                    printf("Acc_Z = %d\r\n", sensor_data.sens_data.acc.z);

                    /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                    x = lsb_to_mps2(sensor_data.sens_data.acc.x, 2, bmi2_dev.resolution);
                    y = lsb_to_mps2(sensor_data.sens_data.acc.y, 2, bmi2_dev.resolution);
                    z = lsb_to_mps2(sensor_data.sens_data.acc.z, 2, bmi2_dev.resolution);

                    /* Print the data in m/s2. */
                    printf("\nAcc_ms2_X = %4.2f, Acc_ms2_Y = %4.2f, Acc_ms2_Z = %4.2f\n", x, y, z);

                    indx++;
                }
            }
        }
    }

    bmi2_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel.
 */
static int8_t set_accel_config(struct bmi2_dev *bmi2_dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi2_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI2_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config.cfg.acc.odr = BMI2_ACC_ODR_200HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config.cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel configurations. */
        rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi2_dev);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
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
