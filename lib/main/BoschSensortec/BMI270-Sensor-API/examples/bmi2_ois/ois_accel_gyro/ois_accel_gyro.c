/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
#include <stdio.h>

#include "bmi2_ois.h"
#include "bmi2_ois_common.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! Macro that holds the total number of accel x,y and z axes sample counts to be printed */
#define ACCEL_GYRO_SAMPLE_COUNT  UINT8_C(50)

/******************************************************************************/
/*!                         Functions                                         */

int main(void)
{
    int8_t rslt;

    struct bmi2_ois_dev ois_dev;

    int8_t idx;

    /* To store the gyroscope cross sensitivity value */
    int16_t ois_gyr_cross_sens_zx = 0;

    /* Array to enable sensor through OIS interface */
    uint8_t sens_sel[2] = { BMI2_OIS_ACCEL, BMI2_OIS_GYRO };

    /* Variable that holds the accel and gyro sample count */
    uint8_t n_data = ACCEL_GYRO_SAMPLE_COUNT;

    /* Initialize the device structure */
    rslt = bmi2_ois_init(&ois_dev);
    bmi2_ois_error_codes_print_result("bmi2_ois_init", rslt);

    /* Get configurations for OIS */
    rslt = bmi2_ois_get_config(&ois_dev);
    bmi2_ois_error_codes_print_result("bmi2_ois_get_config", rslt);

    /* OIS configurations */
    ois_dev.acc_en = BMI2_OIS_ENABLE;
    ois_dev.gyr_en = BMI2_OIS_ENABLE;
    ois_dev.lp_filter_en = BMI2_OIS_ENABLE;

    /* Set configurations for OIS */
    rslt = bmi2_ois_set_config(&ois_dev);
    bmi2_ois_error_codes_print_result("bmi2_ois_set_config", rslt);

    /* Get configurations for OIS */
    rslt = bmi2_ois_get_config(&ois_dev);
    bmi2_ois_error_codes_print_result("bmi2_ois_get_config", rslt);

    if (rslt == BMI2_OIS_OK)
    {
        for (idx = 0; idx < n_data; idx++)
        {
            ois_dev.ois_delay_us(156, ois_dev.intf_ptr);

            /* Accel ODR is 1600hz and gyro ODR is 6400hz.Delay required for
             * accel 156us and 625us for gyro.
             * taken modules from accel and gyro ODR which results every 4th sample accel and gyro
             * read happens, rest of three samples gyro data alone will be read */
            if (idx % 4 == 0)
            {
                /* Get OIS accelerometer and gyro data through OIS interface
                 * @note for sensor which support gyro cross axes sensitivity pass the
                 * gyr_cross_sens_zx from the bmi2_dev structure */
                rslt = bmi2_ois_read_data(sens_sel, 2, &ois_dev, ois_gyr_cross_sens_zx);
                bmi2_ois_error_codes_print_result("bmi2_ois_read_data", rslt);

                if (rslt == BMI2_OIS_OK)
                {
                    printf("\n");

                    /* Print accelerometer data */
                    printf("OIS Accel x-axis = %d ", ois_dev.acc_data.x);
                    printf("OIS Accel y-axis = %d ", ois_dev.acc_data.y);
                    printf("OIS Accel z-axis = %d \n", ois_dev.acc_data.z);

                    /* Print gyro data */
                    printf("OIS Gyro x-axis = %d ", ois_dev.gyr_data.x);
                    printf("OIS Gyro y-axis = %d ", ois_dev.gyr_data.y);
                    printf("OIS Gyro z-axis = %d \n", ois_dev.gyr_data.z);

                    printf("\n");
                }
            }
            else
            {
                /* Get OIS gyro data through OIS interface
                 * @note for sensor which support gyro cross axes sensitivity pass the
                 * gyr_cross_sens_zx from the bmi2_dev structure */
                rslt = bmi2_ois_read_data(&sens_sel[1], 1, &ois_dev, ois_gyr_cross_sens_zx);
                bmi2_ois_error_codes_print_result("bmi2_ois_read_data", rslt);
                if (rslt == BMI2_OIS_OK)
                {
                    /* Print gyro data */
                    printf("OIS Gyro x-axis = %d ", ois_dev.gyr_data.x);
                    printf("OIS Gyro y-axis = %d ", ois_dev.gyr_data.y);
                    printf("OIS Gyro z-axis = %d ", ois_dev.gyr_data.z);

                    printf("\n");
                }
            }
        }
    }

    bmi2_ois_coines_deinit();

    return rslt;
}
