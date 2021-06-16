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
* @file       bmi270_maximum_fifo.c
* @date       2020-11-04
* @version    v2.63.1
*
*/

/***************************************************************************/

/*!             Header files
 ****************************************************************************/
#include "bmi270_maximum_fifo.h"

/***************************************************************************/

/*!              Global Variable
 ****************************************************************************/

/*! @name  Global array that stores the configuration file of BMI270 */
const uint8_t bmi270_maximum_fifo_config_file[] = {
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
    0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
    0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
    0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
    0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
    0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
    0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
    0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
    0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
    0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
    0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
    0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
    0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
    0xf5, 0xeb, 0x2c, 0xe1, 0x6f
};

// The rest of this is not needed, avoid compiler errors due to pedantic settings
#if false

/*! @name  Global array that stores the feature input configuration of BMI270 */
const struct bmi2_feature_config bmi270_maximum_fifo_feat_in[BMI270_MAXIMUM_FIFO_MAX_FEAT_IN] = {

};

/*! @name  Global array that stores the feature output configuration */
const struct bmi2_feature_config bmi270_maximum_fifo_feat_out[BMI270_MAXIMUM_FIFO_MAX_FEAT_OUT] = {

};

/******************************************************************************/

/*!         Local Function Prototypes
 ******************************************************************************/

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t null_ptr_check(const struct bmi2_dev *dev);

/***************************************************************************/

/*!         User Interface Definitions
 ****************************************************************************/

/*!
 *  @brief This API:
 *  1) updates the device structure with address of the configuration file.
 *  2) Initializes BMI270 sensor.
 *  3) Writes the configuration file.
 *  4) Updates the feature offset parameters in the device structure.
 *  5) Updates the maximum number of pages, in the device structure.
 */
int8_t bmi270_maximum_fifo_init(struct bmi2_dev *dev)
{
    /* Variable to define result */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);

    if (rslt == BMI2_OK)
    {
        /* Assign chip id of BMI270 */
        dev->chip_id = BMI270_MAXIMUM_FIFO_CHIP_ID;

        dev->config_size = sizeof(bmi270_maximum_fifo_config_file);

        /* Enable the variant specific features if any */
        dev->variant_feature = BMI2_GYRO_CROSS_SENS_ENABLE | BMI2_MAXIMUM_FIFO_VARIANT;

        /* An extra dummy byte is read during SPI read */
        if (dev->intf == BMI2_SPI_INTF)
        {
            dev->dummy_byte = 1;
        }
        else
        {
            dev->dummy_byte = 0;
        }

        /* If configuration file pointer is not assigned any address */
        if (!dev->config_file_ptr)
        {
            /* Give the address of the configuration file array to
             * the device pointer
             */
            dev->config_file_ptr = bmi270_maximum_fifo_config_file;
        }

        /* Initialize BMI2 sensor */
        rslt = bmi2_sec_init(dev);

        if (rslt == BMI2_OK)
        {
            /* Assign the offsets of the feature input
             * configuration to the device structure
             */
            dev->feat_config = bmi270_maximum_fifo_feat_in;

            /* Assign the offsets of the feature output to
             * the device structure
             */
            dev->feat_output = bmi270_maximum_fifo_feat_out;

            /* Assign the maximum number of pages to the
             * device structure
             */
            dev->page_max = BMI270_MAXIMUM_FIFO_MAX_PAGE_NUM;

            /* Assign maximum number of input sensors/
             * features to device structure
             */
            dev->input_sens = BMI270_MAXIMUM_FIFO_MAX_FEAT_IN;

            /* Assign maximum number of output sensors/
             * features to device structure
             */
            dev->out_sens = BMI270_MAXIMUM_FIFO_MAX_FEAT_OUT;

            /* Get the gyroscope cross axis sensitivity */
            rslt = bmi2_get_gyro_cross_sense(dev);
        }
    }

    return rslt;
}

/***************************************************************************/

/*!         Local Function Definitions
 ****************************************************************************/

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi2_dev *dev)
{
    /* Variable to define result */
    int8_t rslt = BMI2_OK;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}
#endif
