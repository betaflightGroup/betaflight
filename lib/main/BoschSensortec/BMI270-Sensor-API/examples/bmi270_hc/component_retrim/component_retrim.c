/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi270_hc.h"
#include "common.h"

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi2_dev bmi2_dev;

    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi2_dev, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270_hc. */
    rslt = bmi270_hc_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* This API runs the CRT process. */
        rslt = bmi2_do_crt(&bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* Do not move the board while doing CRT. If so, it will throw an abort error. */
        if (rslt == BMI2_OK)
        {
            printf("CRT successfully completed.");
        }
    }

    bmi2_coines_deinit();

    return rslt;
}
