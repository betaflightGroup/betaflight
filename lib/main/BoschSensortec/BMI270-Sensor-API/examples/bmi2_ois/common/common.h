/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#ifndef _COMMON_H
#define _COMMON_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "bmi2_ois.h"

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI2_INTF_RET_SUCCESS -> Success
 *  @retval != BMI2_INTF_RET_SUCCESS  -> Failure Info
 *
 */
int8_t bmi2_ois_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI2_INTF_RET_SUCCESS -> Success
 *  @retval != BMI2_INTF_RET_SUCCESS  -> Failure Info
 *
 */
int8_t bmi2_ois_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period_us    : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 *
 */
void bmi2_ois_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief Function to initialize device structure and coines platform
 *
 *  @param[in] bma  : Structure instance of bmi2_dev
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bmi2_ois_init(struct bmi2_ois_dev *ois_dev);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : API name with return status
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmi2_ois_error_codes_print_result(const char api_name[], int8_t rslt);

/*!
 *  @brief Deinitializes coines platform
 *
 *  @return void.
 */
void bmi2_ois_coines_deinit(void);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* End _COMMON_H */
