/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi270_wh.h"
#include "common.h"

/******************************************************************************/
/*!            Functions                                        */
/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi2_dev bmi2_dev;

    /* Structure to define the type of sensor and their respective data. */
    struct bmi2_sensor_data sensor_data;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config config;

    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Accel sensor and step activity feature are listed in array. */
    uint8_t sensor_sel[2] = { BMI2_ACCEL, BMI2_STEP_ACTIVITY };

    /* Type of sensor to get step activity data. */
    sensor_data.type = BMI2_STEP_ACTIVITY;

    /* Variable to get step activity interrupt status. */
    uint16_t int_status = 0;

    uint16_t loop = 10;

    /* Select features and their pins to be mapped to. */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_STEP_ACTIVITY, .hw_int_pin = BMI2_INT2 };

    /* The step activities are listed in array. */
    const char *activity_output[4] = { "BMI2_STILL", "BMI2_WALK", "BMI2_RUN", "BMI2_UNKNOWN" };

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi2_dev, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270. */
    rslt = bmi270_wh_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* Enable the selected sensor. */
        rslt = bmi270_wh_sensor_enable(sensor_sel, 2, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* Configure the type of sensor. */
        config.type = BMI2_STEP_ACTIVITY;

        if (rslt == BMI2_OK)
        {
            /* Get default configurations for the type of feature selected. */
            rslt = bmi270_wh_get_sensor_config(&config, 1, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            if (rslt == BMI2_OK)
            {
                /* Map the feature interrupt for step activity. */
                rslt = bmi270_wh_map_feat_int(&sens_int, 1, &bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                printf("\nMove the board in steps to perform step activity:\n");

                /* Loop to get step activity. */
                while (loop)
                {
                    /* To get the interrupt status of step detector. */
                    rslt = bmi2_get_int_status(&int_status, &bmi2_dev);
                    bmi2_error_codes_print_result(rslt);

                    /* To check the interrupt status of step detector. */
                    if (int_status & BMI270_WH_STEP_ACT_STATUS_MASK)
                    {
                        printf("Step detected\n");

                        /* Get step activity output. */
                        rslt = bmi270_wh_get_sensor_data(&sensor_data, 1, &bmi2_dev);
                        bmi2_error_codes_print_result(rslt);

                        /* Print the step activity output. */
                        printf("Step activity = %s\n", activity_output[sensor_data.sens_data.activity_output]);

                        loop--;
                    }
                }
            }
        }
    }

    bmi2_coines_deinit();

    return rslt;
}
