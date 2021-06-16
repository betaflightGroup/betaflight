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
 * @file       bmi2_ois.h
 * @date       2020-05-05
 * @version    v2.23.2
 *
 */

/**
 * \ingroup bmi2xy
 * \defgroup bmi2_ois BMI2_OIS
 * @brief Sensor driver for BMI2_OIS sensor
 */
#ifndef _BMI2_OIS_H
#define _BMI2_OIS_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************/

/*!             Header files
 ****************************************************************************/
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/******************************************************************************/
/*!  @name          Macros                                        */
/******************************************************************************/

#ifndef BMI2_INTF_RETURN_TYPE
#define BMI2_INTF_RETURN_TYPE  int8_t
#endif

/*! @name  Utility macros */
#define BMI2_OIS_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) | \
     ((data << bitname##_POS) & bitname##_MASK))

#define BMI2_OIS_GET_BITS(reg_data, bitname) \
    ((reg_data & (bitname##_MASK)) >> \
     (bitname##_POS))

#define BMI2_SET_BIT_POS0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) | \
     (data & bitname##_MASK))

#define BMI2_GET_BIT_POS0(reg_data, bitname)  (reg_data & (bitname##_MASK))

/*! @name For enable and disable */
#define BMI2_OIS_ENABLE                       UINT8_C(1)
#define BMI2_OIS_DISABLE                      UINT8_C(0)

/*! @name To define sensor interface success code */
#define BMI2_INTF_RET_SUCCESS                 INT8_C(0)

/*! @name To define success code */
#define BMI2_OIS_OK                           UINT8_C(0)

/*! @name To define error codes */
#define BMI2_OIS_E_NULL_PTR                   INT8_C(-1)
#define BMI2_OIS_E_COM_FAIL                   INT8_C(-2)
#define BMI2_OIS_E_INVALID_SENSOR             INT8_C(-8)

/*! @name Mask definitions for SPI read/write address for OIS */
#define BMI2_OIS_SPI_RD_MASK                  UINT8_C(0x80)
#define BMI2_OIS_SPI_WR_MASK                  UINT8_C(0x7F)

/*! @name BMI2 OIS data bytes */
#define BMI2_OIS_ACC_GYR_NUM_BYTES            UINT8_C(6)

/*!  @name Macros to select sensor for OIS data read */
#define BMI2_OIS_ACCEL                        UINT8_C(0x01)
#define BMI2_OIS_GYRO                         UINT8_C(0x02)

/*!  @name Macros to define OIS register addresses */
#define BMI2_OIS_CONFIG_ADDR                  UINT8_C(0x40)
#define BMI2_OIS_ACC_X_LSB_ADDR               UINT8_C(0x0C)
#define BMI2_OIS_GYR_X_LSB_ADDR               UINT8_C(0x12)

/*! @name Mask definitions for OIS configurations */
#define BMI2_OIS_GYR_EN_MASK                  UINT8_C(0x40)
#define BMI2_OIS_ACC_EN_MASK                  UINT8_C(0x80)

/*! @name Bit Positions for OIS configurations */
#define BMI2_OIS_GYR_EN_POS                   UINT8_C(0x06)
#define BMI2_OIS_ACC_EN_POS                   UINT8_C(0x07)

/*! Low pass filter configuration position and mask */
#define BMI2_OIS_LP_FILTER_EN_POS             UINT8_C(0x00)
#define BMI2_OIS_LP_FILTER_EN_MASK            UINT8_C(0x01)

#define BMI2_OIS_LP_FILTER_CONFIG_POS         UINT8_C(0x01)
#define BMI2_OIS_LP_FILTER_CONFIG_MASK        UINT8_C(0x06)

#define BMI2_OIS_LP_FILTER_MUTE_POS           UINT8_C(0x05)
#define BMI2_OIS_LP_FILTER_MUTE_MASK          UINT8_C(0x20)

/******************************************************************************/
/*! @name           Function Pointers                             */
/******************************************************************************/

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data     : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 *  retval =  BMI2_INTF_RET_SUCCESS -> Success
 *  retval != BMI2_INTF_RET_SUCCESS -> Failure
 *
 */
typedef BMI2_INTF_RETURN_TYPE (*bmi2_ois_read_fptr_t)(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data     : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * retval  = BMI2_INTF_RET_SUCCESS -> Success
 * retval != BMI2_INTF_RET_SUCCESS -> Failure
 *
 */
typedef BMI2_INTF_RETURN_TYPE (*bmi2_ois_write_fptr_t)(uint8_t reg_addr, const uint8_t *data, uint32_t len,
                                                       void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*bmi2_ois_delay_fptr_t)(uint32_t period, void *intf_ptr);

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/
/*! @name Structure to define accelerometer and gyroscope sensor axes for OIS */
struct bmi2_ois_sens_axes_data
{
    /*! Data in x-axis */
    int16_t x;

    /*! Data in y-axis */
    int16_t y;

    /*! Data in z-axis */
    int16_t z;

};

/*!  @name Structure to define bmi2 OIS sensor configurations */
struct bmi2_ois_dev
{
    /*! Read function pointer */
    bmi2_ois_read_fptr_t ois_read;

    /*! Write function pointer */
    bmi2_ois_write_fptr_t ois_write;

    /*!  Delay function pointer */
    bmi2_ois_delay_fptr_t ois_delay_us;

    /*!  Low pass filter en/dis  */
    uint8_t lp_filter_en;

    /*! Void interface pointer */
    void *intf_ptr;

    /*! To store interface pointer error */
    int8_t intf_rslt;

    /*!  Low pass filter cut-off frequency  */
    uint8_t lp_filter_config;

    /*!  Low pass filter mute  */
    uint8_t lp_filter_mute;

    /*! Accelerometer enable for OIS */
    uint8_t acc_en;

    /*! Gyroscope enable for OIS */
    uint8_t gyr_en;

    /*! Accelerometer data axes */
    struct bmi2_ois_sens_axes_data acc_data;

    /*! Gyroscope data axes */
    struct bmi2_ois_sens_axes_data gyr_data;
};

/***************************************************************************/

/*!     BMI2 OIS User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bmi2_ois
 * \defgroup bmi2_oisApiRegs Registers
 * @brief Read data from the given OIS register address of bmi2
 */

/*!
 * \ingroup bmi2_oisApiRegs
 * \page bmi2_ois_api_bmi2_ois_get_regs bmi2_ois_get_regs
 * \code
 * int8_t bmi2_ois_get_regs(uint8_t ois_reg_addr,
 *                        uint8_t *ois_reg_data,
 *                        uint16_t data_len,
 *                        const struct bmi2_ois_dev *ois_dev);
 * \endcode
 *  @details This API reads the data from the given OIS register address of bmi2
 * sensor.
 *
 * @param[in]  ois_reg_addr : OIS register address from which data is read.
 * @param[out] ois_reg_data : Pointer to data buffer where read data is stored.
 * @param[in]  data_len     : No. of bytes of data to be read.
 * @param[in]  ois_dev      : Structure instance of bmi2_ois_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_ois_get_regs(uint8_t ois_reg_addr, uint8_t *ois_reg_data, uint16_t data_len, struct bmi2_ois_dev *ois_dev);

/*!
 * \ingroup bmi2_oisApiRegs
 * \page bmi2_ois_api_bmi2_ois_set_regs bmi2_ois_set_regs
 * \code
 * int8_t bmi2_ois_set_regs(uint8_t ois_reg_addr,
 *                        uint8_t *ois_reg_data,
 *                        uint16_t data_len,
 *                        const struct bmi2_ois_dev *ois_dev);
 * \endcode
 *  @details This API writes data to the given OIS register address of bmi2 sensor.
 *
 * @param[in] ois_reg_addr  : OIS register address to which the data is written.
 * @param[in] ois_reg_data  : Pointer to data buffer in which data to be written
 *                is stored.
 * @param[in] data_len      : No. of bytes of data to be written.
 * @param[in] ois_dev       : Structure instance of bmi2_ois_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_ois_set_regs(uint8_t ois_reg_addr,
                         const uint8_t *ois_reg_data,
                         uint16_t data_len,
                         struct bmi2_ois_dev *ois_dev);

/**
 * \ingroup bmi2_ois
 * \defgroup bmi2_oisApiConfig Status update
 * @brief Get / Set the status of Enable / Disable accelerometer / gyroscope data read through OIS interface
 */

/*!
 * \ingroup bmi2_oisApiConfig
 * \page bmi2_ois_api_bmi2_ois_set_config bmi2_ois_set_config
 * \code
 * int8_t bmi2_ois_set_config(const struct bmi2_ois_dev *ois_dev);
 * \endcode
 *  @details This API sets the status of enable/disable accelerometer/gyroscope data read through
 * OIS interface.
 *
 * @param[in] ois_dev   : Structure instance of bmi2_ois_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_ois_set_config(struct bmi2_ois_dev *ois_dev);

/*!
 * \ingroup bmi2_oisApiConfig
 * \page bmi2_ois_api_bmi2_ois_get_config bmi2_ois_get_config
 * \code
 * int8_t bmi2_ois_get_config(struct bmi2_ois_dev *ois_dev);
 * \endcode
 *  @details This API gets the status of accelerometer/gyroscope enable for data
 * read through OIS interface.
 *
 * @param[in, out] ois_dev : Structure instance of bmi2_ois_dev.
 *
 * @note Enabling and disabling is done during OIS structure initialization.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_ois_get_config(struct bmi2_ois_dev *ois_dev);

/**
 * \ingroup bmi2_ois
 * \defgroup bmi2_oisApiRead Data read
 * @brief Read accelerometer / gyroscope data through OIS interface
 */

/*!
 * \ingroup bmi2_oisApiRead
 * \page bmi2_ois_api_bmi2_ois_read_data bmi2_ois_read_data
 * \code
 * int8_t bmi2_ois_read_data(const uint8_t *sens_sel,
 *                         uint8_t n_sens,
 *                         struct bmi2_ois_dev *ois_dev,
 *                         int16_t gyr_cross_sens_zx);
 * \endcode
 *  @details This API reads accelerometer/gyroscope data through OIS interface.
 *
 * @param[in] sens_sel          : Select sensor whose data is to be read.
 * @param[in] n_sens            : Number of sensors selected.
 * @param[in, out] ois_dev      : Structure instance of bmi2_ois_dev.
 * @param[in] gyr_cross_sens_zx : Store the gyroscope cross sensitivity values taken from the bmi2xy
 *                                (refer bmi2_ois example).
 *
 *@verbatim
 *  sens_sel      |   Value
 * ---------------|---------------
 * BMI2_OIS_ACCEL |   0x01
 * BMI2_OIS_GYRO  |   0x02
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_ois_read_data(const uint8_t *sens_sel,
                          uint8_t n_sens,
                          struct bmi2_ois_dev *ois_dev,
                          int16_t gyr_cross_sens_zx);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* End of _BMI2_OIS_H */
