# Sensor API for the BMI2's OIS interface

## Table of Contents
 - [Introduction](#Intro)
 - [Integration details](#Integration)
 - [Driver files information](#file)
 - [Sensor interfaces](#interface)
 - [Integration Examples](#examples)

### Introduction<a name=Intro></a>
 This package contains Bosch Sensortec's BMI2 Sensor API.
 
### Integration details<a name=Integration></a>
- Integrate _bmi2.c_, _bmi2.h_, _bmi2_ois.c_, _bmi2_ois.h_, _bmi2_defs.h_ and the required variant files in your project.
- User has to include _bmi2_ois.h_ in the code to call OIS related APIs and a _variant header_ for initialization as
well as BMI2 related API calls, as shown below:
``` c
#include "bmi270.h"
#include "bmi2_ois.h"
````
### Driver files information<a name=file></a>
- *_bmi2_ois.c_*
   * This file has function definitions of OIS related API interfaces.
- *_bmi2_ois.h_*
   * This header file has necessary include files, function declarations, required to make OIS related API calls.
 
### Sensor interfaces<a name=interface></a>
#### _Host Interface_
- I2C interface
- SPI interface  
_Note: By default, the interface is I2C._

#### _OIS Interface_
- SPI interface  

### Integration examples<a name=examples></a>
#### Configuring SPI/I2C for host interface.
To configure host interface, an instance of the bmi2_dev structure should be
created for initializing BMI2 sensor. "_Refer **README** for initializing BMI2 
sensor._"

#### Configuring SPI for OIS interface.
To configure OIS interface, an instance of the bmi2_ois_dev structure should be
created. The following parameters are required to be updated in the structure,
by the user.

Parameters    | Details
--------------|-----------------------------------
_intf_ptr_    | device address reference of SPI interface        
_ois_read_    | read through SPI interface
_ois_write_   | read through SPI interface
_ois_delay_us_| delay in micro seconds
_acc_en_      | for enabling accelerometer
_gyr_en_      | for enabling gyroscope   

``` c
int8_t rslt = 0;

struct bmi2_ois_dev ois_dev = {  
        .intf_ptr = intf_ptr will contain the chip selection info of SPI CS pin,  
        .ois_read = user_spi_reg_read,
        .ois_write = user_spi_reg_write,
        .ois_delay_us = user_delay_us
};
```
>**_Important Note_**: For initializing and configuring BMI2 sensors, which is 
done through host interface, the API's are to be used from bmi2.c file. Rest 
of the API's, for OIS configurations and the reading of OIS data, which is done 
through OIS interface, are to be used from bmi2_ois.c file.

##### Get accelerometer and gyroscope data through OIS interface 
``` c
int8_t rslt;
/* Array to enable sensor through host interface */
uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
/* Array to enable sensor through OIS interface */
uint8_t sens_sel[2] = {BMI2_OIS_ACCEL, BMI2_OIS_GYRO};
/* Initialize the configuration structure */
struct bmi2_sens_config sens_cfg = {0};

/* Initialize BMI2 */
rslt = bmi2_init(&dev);
if (rslt != BMI2_OK) {
        printf("Error: %d\n", rslt);
        return;
}

/* Enable accelerometer and gyroscope through host interface */
rslt = bmi270_sensor_enable(sens_list, 2, &dev);
if (rslt != BMI2_OK) {
        printf("Error: %d\n", rslt);
        return;
}

/* Setting of OIS Range is done through host interface */
/* Select the gyroscope sensor for OIS Range configuration */
sens_cfg.type = BMI2_GYRO;

/* Get gyroscope configuration */
rslt = bmi2_get_sensor_config(&sens_cfg, 1, &dev);
if (rslt != BMI2_OK) {
        printf("Error: %d\n", rslt);
        return;
}

/* Set the desired OIS Range */
sens_cfg.cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

/* Set gyroscope configuration for default values */
rslt = bmi2_set_sensor_config(&sens_cfg, 1, &dev);
if (rslt != BMI2_OK) {
         printf("Error: %d\n", rslt);
         return;
}

/* Enable OIS through host interface */
rslt = bmi2_set_ois_interface(BMI2_ENABLE, &dev);
if (rslt != BMI2_OK) {
         printf("Error: %d\n", rslt);
         return;
}

/* Disable Advance Power Save Mode through host interface */
rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &dev);
if (rslt != BMI2_OK) {
         printf("Error: %d\n", rslt);
         return;
}

/* Get configurations for OIS through OIS interface for default values */
rslt = bmi2_ois_get_config(&ois_dev);
if (rslt != BMI2_OK) {
         printf("Error: %d\n", rslt);
         return;
}

/* Enable accelerometer and gyroscope for reading OIS data */
ois_dev.acc_en = BMI2_ENABLE;
ois_dev.gyr_en = BMI2_ENABLE;

/* Set configurations for OIS through OIS interface */
rslt = bmi2_ois_set_config(&ois_dev);
if (rslt == BMI2_OK) {
        /* Get OIS accelerometer and gyroscope data through OIS interface */
        rslt = bmi2_ois_read_data(sens_sel, 2, &ois_dev);
        if (rslt == BMI2_OK) {
                /* Print accelerometer data */
                printf("OIS Accel x-axis = %d\t", ois_dev.acc_data.x);
                printf("OIS Accel y-axis= %d\t", ois_dev.acc_data.y);
                printf("OIS Accel z-axis = %d\r\n", ois_dev.acc_data.z);        
        
                /* Print gyroscope data */
                printf("OIS Gyro x-axis = %d\t", ois_dev.gyr_data.x);
                printf("OIS Gyro y-axis= %d\t", ois_dev.gyr_data.y);
                printf("OIS Gyro z-axis = %d\r\n", ois_dev.gyr_data.z);
        }        
}

if (rslt != BMI2_OK) {
         printf("Error code: %d\n", rslt);
         return;
}

/* Enable Advance Power Save Mode through host interface */
rslt = bmi2_set_adv_power_save(BMI2_ENABLE, &dev);
if (rslt != BMI2_OK) {
        printf("Error: %d\n", rslt);
        return;
}
```