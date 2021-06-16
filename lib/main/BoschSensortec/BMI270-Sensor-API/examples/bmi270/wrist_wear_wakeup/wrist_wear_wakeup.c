/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!            Header Files                                  */
#include <stdio.h>
#include "bmi270.h"
#include "common.h"

/******************************************************************************/
/*!         Static function declaration                                       */

/*!
 *  @brief This internal API is used to set the sensor configuration.
 *
 *  @param[in] bmi2_dev     : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t bmi2_set_config(struct bmi2_dev *bmi2_dev);

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program */
int main(void)
{
    /* Variable to define result */
    int8_t rslt;

    /* Initialize status of wrist wear wakeup interrupt */
    uint16_t int_status = 0;

    /* Sensor initialization configuration. */
    struct bmi2_dev bmi2_dev;

    /* Select features and their pins to be mapped to */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_WRIST_WEAR_WAKE_UP, .hw_int_pin = BMI2_INT1 };

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi2_dev, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270. */
    rslt = bmi270_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* Set the sensor configuration */
        rslt = bmi2_set_config(&bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK)
        {
            /* Map the feature interrupt */
            rslt = bmi270_map_feat_int(&sens_int, 1, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            if (rslt == BMI2_OK)
            {
                printf("Tilt the board in landscape position to trigger wrist wear wakeup\n");

                /* Loop to print the wrist wear wakeup data when interrupt occurs */
                while (1)
                {
                    int_status = 0;

                    /* To get the interrupt status of the wrist wear wakeup */
                    rslt = bmi2_get_int_status(&int_status, &bmi2_dev);
                    bmi2_error_codes_print_result(rslt);

                    /* To check the interrupt status of the wrist gesture */
                    if ((rslt == BMI2_OK) && (int_status & BMI270_WRIST_WAKE_UP_STATUS_MASK))
                    {
                        printf("Wrist wear wakeup detected\n");
                        break;
                    }
                }
            }
        }
    }

    bmi2_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API sets the sensor configuration
 */
static int8_t bmi2_set_config(struct bmi2_dev *bmi2_dev)
{
    /* Variable to define result */
    int8_t rslt;

    /* List the sensors which are required to enable */
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_WRIST_WEAR_WAKE_UP };

    /* Structure to define the type of the sensor and its configurations */
    struct bmi2_sens_config config;

    /* Configure type of feature */
    config.type = BMI2_WRIST_WEAR_WAKE_UP;

    /* Enable the selected sensors */
    rslt = bmi270_sensor_enable(sens_list, 2, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* Get default configurations for the type of feature selected */
        rslt = bmi270_get_sensor_config(&config, 1, bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK)
        {
            /* Set the new configuration */
            rslt = bmi270_set_sensor_config(&config, 1, bmi2_dev);
            bmi2_error_codes_print_result(rslt);
        }
    }

    return rslt;
}
