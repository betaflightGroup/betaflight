/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "coines.h"
#include "common.h"

/******************************************************************************/
/*!                     Global declaration                                    */

/*! Device address for primary and secondary interface */
static uint8_t ois_dev_addr;

/******************************************************************************/
/*!                        Functions                                          */

/*!
 * SPI read function map to COINES platform
 */
int8_t bmi2_ois_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_spi(dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * SPI write function map to COINES platform
 */
int8_t bmi2_ois_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_write_spi(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * Delay function map to COINES platform
 */
void bmi2_ois_delay_us(uint32_t period, void *intf_ptr)
{
    coines_delay_usec(period);
}

/******************************************************************************/
int8_t bmi2_ois_init(struct bmi2_ois_dev *ois_dev)
{
    int8_t rslt;

    if (ois_dev != NULL)
    {
        int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB);
        if (result < COINES_SUCCESS)
        {
            printf(
                "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
                " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
            exit(result);
        }

        coines_set_shuttleboard_vdd_vddio_config(0, 0);
        coines_delay_msec(100);

        printf("SPI Interface \n");

        /* To initialize the user SPI function */
        ois_dev_addr = COINES_SHUTTLE_PIN_8; /*COINES_SHUTTLE_PIN_7 */
        ois_dev->ois_read = bmi2_ois_spi_read;
        ois_dev->ois_write = bmi2_ois_spi_write;

        /* Assign device address to interface pointer */
        ois_dev->intf_ptr = &ois_dev_addr;

        /* Configure delay in microseconds */
        ois_dev->ois_delay_us = bmi2_ois_delay_us;

        /* SDO pin is made low for selecting I2C address 0x76*/
        coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

        /*        coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3); */
        coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_7_5_MHZ, COINES_SPI_MODE0);
        coines_delay_msec(10);

        /* PS pin is made high for selecting I2C protocol (gyroscope)*/
        coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

        coines_delay_usec(10000);

        coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

        coines_delay_usec(10000);

    }
    else
    {
        rslt = BMI2_OIS_E_NULL_PTR;
    }

    return rslt;
}

/******************************************************************************/

/*!
 * @brief This internal API prints the execution status
 */
void bmi2_ois_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMI2_OIS_OK)
    {
        printf("%s\t", api_name);

        if (rslt == BMI2_OIS_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer error.\r\n", rslt);
            printf(
                "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
        }
        else if (rslt == BMI2_OIS_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure error.\r\n", rslt);
            printf(
                "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
        }
        else if (rslt == BMI2_OIS_E_INVALID_SENSOR)
        {
            printf(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the available one\r\n",
                rslt);
        }
        else
        {
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/*!
 *  @brief Deinitializes coines platform
 *
 *  @return void.
 */
void bmi2_ois_coines_deinit(void)
{
    coines_close_comm_intf(COINES_COMM_INTF_USB);
}
