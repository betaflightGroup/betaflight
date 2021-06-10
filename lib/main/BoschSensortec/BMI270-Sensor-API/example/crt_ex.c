#include <stdio.h>
#include "bmi2.h"
#include "bmi270.h"


void delay_us(uint32_t period);
int8_t i2c_reg_read(uint8_t i2c_add, uint8_t reg_add, uint8_t *regd_data, uint16_t length);
int8_t i2c_reg_write(uint8_t i2c_add, uint8_t reg_add, const uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_add, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_write(uint8_t cs, uint8_t reg_add, uint8_t *reg_data, uint16_t length);
void print_rslt(int8_t rslt);

int main(void)
{

    struct bmi2_dev dev;

    /*! @name Structure to define type of sensor and their respective data */
    struct bmi2_sensor_data sensor_data;

    uint8_t sens_sel[2] = { BMI2_ACCEL, BMI2_GYRO };

    /*variable to define rslt*/
    int8_t rslt;

    /*to enable i2c interface*/
    dev.read = i2c_reg_read;
    dev.write = i2c_reg_write;
    dev.delay_us = delay_us;
    dev.read_write_len = 128;
    dev.intf = BMI2_I2C_INTERFACE;
    dev.dev_id = BMI2_I2C_PRIM_ADDR;

    /*to enable spi interface*/
   /* dev.read = spi_reg_read;
    dev.write = spi_reg_write;
    dev.delay_us = delay_us;
    dev.read_write_len = 4096;
    dev.intf = BMI2_SPI_INTERFACE;
    dev.dev_id = SPI_CS;
    dev.dummy_byte = 1;
    */
    dev.config_file_ptr = NULL;

    /*to Initialize bmi270*/
    rslt = bmi270_init(&dev);
    print_rslt(rslt);

    rslt = bmi2_sensor_enable(&sens_sel[0], 1, &dev);
    print_rslt(rslt);

    rslt = bmi2_sensor_disable(&sens_sel[1], 1, &dev);
    print_rslt(rslt);

    sensor_data.type = BMI2_ACCEL;

    dev.delay_us(100000);

    printf("\nbefore CRT Accel x,y,z values\n");

    /* read the accel data before CRT*/
    rslt = bmi2_get_sensor_data(&sensor_data, 1, &dev);
    print_rslt(rslt);
    printf("\nX axes: %d, Y axes: %d, Z axes: %d \n", sensor_data.sens_data.acc.x, sensor_data.sens_data.acc.y, sensor_data.sens_data.acc.z );

    /*brief This API is to run the CRT process*/
    rslt = bmi2_do_crt(&dev);
    print_rslt(rslt);
    if(rslt == BMI2_OK)
    {
        printf("\nAfter CRT Accel x,y,z values\n");

        /* read the accel data after CRT*/
        rslt = bmi2_get_sensor_data(&sensor_data, 1, &dev);
        print_rslt(rslt);

        printf("X axes: %d, Y axes: %d, Z axes: %d",sensor_data.sens_data.acc.x, sensor_data.sens_data.acc.y, sensor_data.sens_data.acc.z );
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
        case BMI2_E_CRT_ERROR:
            printf("warning[%d] : CRT fail\r\n", rslt);
            break;
        case BMI2_E_ABORT_ERROR:
            printf("warning[%d] : Abort eror\r\n", rslt);
            break;
        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}
