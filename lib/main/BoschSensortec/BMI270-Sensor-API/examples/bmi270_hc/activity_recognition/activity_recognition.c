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
/*!         Static function declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for step counter.
 *
 *  @param[in] bmi2_dev       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_config(struct bmi2_dev *bmi2_dev);

/******************************************************************************/
/*!            Functions                                        */
/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi2_dev bmi2_dev;

    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to define index for FIFO data */
    uint8_t count;

    /* Various Activity recognitions are listed in array. */
    const char *activity_reg_output[6] = { "OTHERS", "STILL", "WALKING", "RUNNING", "ON_BICYCLE", "IN_VEHICLE" };

    /* Accel sensor and step activity feature are listed in array. */
    uint8_t sensor_sel[2] = { BMI2_ACCEL, BMI2_ACTIVITY_RECOGNITION };

    /* Array to define FIFO data */
    uint8_t fifo_data[516] = { 0 };

    /* Variable to store number of activity frames */
    uint16_t act_frames;

    /* Structure to define a FIFO */
    struct bmi2_fifo_frame fifoframe = { 0 };

    /* Structure to define output for activity recognition */
    struct bmi2_act_recog_output act_recog_data[5] = { { 0 } };

    uint16_t fifo_length;

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
        /* Enable the selected sensor. */
        rslt = bmi270_hc_sensor_enable(sensor_sel, 2, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK)
        {
            rslt = set_config(&bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            if (rslt == BMI2_OK)
            {
                /* Update FIFO structure */
                fifoframe.data = fifo_data;
                fifoframe.length = 516;

                rslt = bmi2_get_fifo_length(&fifo_length, &bmi2_dev);
                printf("Fifo length = %d \n", fifo_length);

                /* Read FIFO data */
                rslt = bmi2_read_fifo_data(&fifoframe, &bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                while (rslt != BMI2_W_FIFO_EMPTY)
                {
                    /* Provide the number of frames to be read */
                    act_frames = 5;

                    printf("Requested FIFO data frames : %d\n", act_frames);

                    /* Get the activity output */
                    rslt = bmi270_hc_get_act_recog_output(act_recog_data, &act_frames, &fifoframe, &bmi2_dev);
                    bmi2_error_codes_print_result(rslt);

                    printf("Parsed FIFO data frames : %d\r\n", act_frames);

                    for (count = 0; count < act_frames; count++)
                    {
                        printf(
                            "Activity Recognition output[%d]:Sensor time: %lu\t Previous activity: %s\t current: %s\n",
                            count,
                            act_recog_data[count].time_stamp,
                            activity_reg_output[act_recog_data[count].prev_act],
                            activity_reg_output[act_recog_data[count].curr_act]);
                    }
                }
            }
        }
    }

    bmi2_coines_deinit();

    return rslt;
}

static int8_t set_config(struct bmi2_dev *bmi2_dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    uint8_t temp_data[2];

    /* Disable advanced power save */
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* Reset FIFO */
        rslt = bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD, bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        printf("Move the board for activity recognition for 30 sec :\n");
        bmi2_dev->delay_us(30000000, bmi2_dev->intf_ptr);

        if (rslt == BMI2_OK)
        {
            rslt = bmi2_get_regs(BMI2_FIFO_CONFIG_0_ADDR, temp_data, 2, bmi2_dev);
            bmi2_error_codes_print_result(rslt);
        }
    }

    return rslt;
}
