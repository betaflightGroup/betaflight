#include <stdio.h>
#include "bmi2.h"
#include "bmi270.h"


void delay_us(uint32_t period);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(int8_t rslt);

int main(void)
{

    struct bmi2_dev dev;

    /* Variable to define rslt */
    int8_t rslt;

    /* Array to select sensors */
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_WRIST_GESTURE };

    /* Variable to get wrist gesture status */
    uint16_t wrist_gest_status = 0;

    /* Select features and their pins to be mapped to */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_WRIST_GESTURE, .hw_int_pin = BMI2_INT1 };

    /* Sensor configuration structure */
    struct bmi2_sens_config config = { 0 };

    /* Sensor data structure */
    struct bmi2_sensor_data sens_data = { 0 };

    /*To enable I2C interface*/
    dev.read = i2c_reg_read;
    dev.write = i2c_reg_write;
    dev.delay_us = delay_us;
    dev.read_write_len = 128;
    dev.intf = BMI2_I2C_INTERFACE;
    dev.dev_id = BMI2_I2C_PRIM_ADDR;

    /* To enable SPI interface*/

    /*
     * dev.read = spi_reg_read;
     * dev.write = spi_reg_write;
     * dev.delay_us = delay_us;
     * dev.read_write_len = 4096;
     * dev.intf = BMI2_SPI_INTERFACE;
     * dev.dev_id = SPI_CS;
     * dev.dummy_byte = 1;
     */

    dev.config_file_ptr = NULL;

    /* Initialize BMI270  */
    rslt = bmi270_init(&dev);
    print_rslt(rslt);

    /* Enable the selected sensors */
    rslt = bmi2_sensor_enable(sens_list, 2, &dev);
    print_rslt(rslt);

    /* Configure type of feature */
    config.type = BMI2_WRIST_GESTURE;

    /* Get default configurations for the type of feature selected */
    rslt = bmi2_get_sensor_config(&config, 1, &dev);
    print_rslt(rslt);
    if (rslt == BMI2_OK)
    {
        /* Arm Position where the device is attached */
        config.cfg.wrist_gest.wearable_arm = BMI2_ARM_LEFT;

        /* Set the new configuration along with interrupt mapping */
        rslt = bmi2_set_sensor_config(&config, 1, &dev);
        print_rslt(rslt);
    }

    /* Map the feature interrupt */
    rslt = bmi2_map_feat_int(&sens_int, 1, &dev);
    print_rslt(rslt);
    printf("Flip the board in portrait landscape mode:\n");
    while (1)
    {
        /* Check the interrupt status of the wrist gesture */
        rslt = bmi2_get_int_status(&wrist_gest_status, &dev);
        print_rslt(rslt);
        if (rslt == BMI2_OK)
        {
            if (wrist_gest_status & BMI270_WRIST_GEST_STATUS_MASK)
            {
                printf("Wrist gesture detected\n");

                /* Get wrist gesture output */
                rslt = bmi2_get_sensor_data(&sens_data, 1, &dev);
                print_rslt(rslt);

                /* Print the wrist gesture output */
                printf("wrist gesture = %d\r\n", sens_data.sens_data.wrist_gesture_output);
                break;
            }

            dev.delay_us(100000);
        }

        return 0;
    }
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bma4_write_config_file",
 *      "bma4_write_regs", "bma4_set_accel_config"  and so on.
 *
 *  @param[in] period_us  : the required wait time in microseconds.
 *  @return void.
 *
 */
void delay_us(uint32_t period)
{
     /* Wait for a period amount of us*/
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length)
{

    /* Write to registers using I2C. Return 0 for a successful execution. */
    return 0;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Read from registers using I2C. Return 0 for a successful execution. */
    return 0;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] cs           : Chip select to enable the sensor.
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

      /* Write to registers using SPI. Return 0 for a successful execution. */
      return 0;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] cs       : Chip select to enable the sensor.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

     /* Read from registers using SPI. Return 0 for a successful execution. */
     return 0;
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(int8_t rslt)
{
     switch (rslt)
      {
            case BMI2_OK:

                /* Do nothing */
                break;
            case BMI2_E_NULL_PTR:
                printf("Error [%d] : Null pointer\r\n", rslt);
                break;
            case BMI2_E_COM_FAIL:
                printf("Error [%d] : Communication failure\r\n", rslt);
                break;
            case BMI2_E_DEV_NOT_FOUND:
                printf("Error [%d] : Device not found\r\n", rslt);
                break;
            case BMI2_E_INVALID_SENSOR:
                printf("Error [%d] : Invalid sensor\r\n", rslt);
                break;
            case BMI2_E_SELF_TEST_FAIL:
                printf("Warning [%d] : Self test failed\r\n", rslt);
                break;
            case BMI2_E_INVALID_INT_PIN:
                printf("warning [%d] : invalid int pin\r\n", rslt);
                break;
            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
      }
}
