# Create a Customized Version

The flight controllers have some limitations in space and computing capacity. The developers must decide what features enable and what others disable to create a firmware file that can be used for the major part or users.

For example, the official firmware published to users include support for a lot of brands of transmitters and receivers, but habitually you will use only one. This is necessary for a public version, but the protocols not used are spending space that can be used for other features.

This document gives a little guide of how-to start creating your own version, activating and deactivating features, especially if your flight controller is an older one with not too much space.

Keep in mind that when you create your own firmware you're using a piece of software created by you and can have some bugs that are not present in official version. **You use it at your own risk**.

## Build the firmware

This document is aimed to people who has some knowledge about programming skills and can build its own firmware. You can find information about this process in the [`development`](development/) documentation page.

Once you are able to compile your own firmware, you can continue to the next section of this document.

When you compile the firmware, the `make` process ends with an info summary of the firmware created, something like this:
```
   text    data     bss     dec     hex filename
 126312    1444   18260  146016   23a60 ./obj/main/cleanflight_NAZE.elf
```

The 'text + data' gives you the flash size, and the 'data + bss' is the (static) ram usage. It's recommended to keep the customized version under the values of the unmodified version.

## Commons features for all Flight Controllers

The first file where the developers specify the features that are activated or not for a flight controller, is the `target/common_pre.h`.

This file specifies the features enabled/disabled depending on the memory flash size of the flight controller, or other conditions.

The first interesting part is where it specifies the features activated for all flight controllers. In the actual version, for example:
```
#define USE_CLI
#define USE_PPM
#define USE_PWM
#define SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
```

The next part are features enabled if your memory flash size is bigger than 64 megabits.
```
#if (FLASH_SIZE > 64)
#define BLACKBOX
#define LED_STRIP
#define TELEMETRY
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_LTM
#define TELEMETRY_SMARTPORT
#define USE_RESOURCE_MGMT
#define USE_SERVOS
#endif
```

And bigger than 128 megabits:
```
#if (FLASH_SIZE > 128)
#define GPS
#define CMS
#define TELEMETRY_CRSF
#define TELEMETRY_IBUS
#define TELEMETRY_JETIEXBUS
#define TELEMETRY_MAVLINK
#define TELEMETRY_SRXL
#define USE_DASHBOARD
#define USE_MSP_DISPLAYPORT
#define USE_RX_MSP
#define USE_SERIALRX_JETIEXBUS
#define USE_SENSOR_NAMES
#define VTX_COMMON
#define VTX_CONTROL
#define VTX_SMARTAUDIO
#define VTX_TRAMP
#endif
```

Another interesting thing of this file, are others features deactivated by the computing capacity of the flight controller, or the model of the processor. For example:
```
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
#define DEFAULT_AUX_CHANNEL_COUNT       MAX_AUX_CHANNEL_COUNT
#else
#define DEFAULT_AUX_CHANNEL_COUNT       6
#endif
```
If the flight controller hasn't a FPU (F1 processors for example) you can use only 6 auxiliary channels of the receiver.

After looking carefully to this file, you must know what features you want to disable or enable in your customized build.

*NOTE: It is better to not change this file, but it's useful to see what features you can activate or deactivate in your custom build.*

## Specific features for each Flight Controller

Each flight controller has its own file to specify what features are enabled or disabled only for it. Sometimes they have been disabled by space limitations, but other times it's for limited computing capacity or a bug, so enable them at your own risk. 

This file is located in `target/[FLIGHT_CONTROLLER_NAME]/target.h` and it's loaded **after** the `target/common_pre.h`. Any changes in this file will overwrite the default settings, so this file is the place where you must touch to create your custom firmware.

The first thing to do is to *#undef* all the features that we want to disable from the *common_pre.h*. 

For example, in a NAZE32, if we're using Serial RX, with a FlySky receiver (that uses de iBus protocol) and we don't have a led strip we will add all this *#undef* to the file.

```
#undef USE_PPM
#undef USE_PWM
#undef USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#undef USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#undef USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#undef USE_SERIALRX_SUMD       // Graupner Hott protocol
#undef USE_SERIALRX_SUMH       // Graupner legacy protocol
#undef USE_SERIALRX_XBUS       // JR

#undef LED_STRIP
#undef TELEMETRY_FRSKY
#undef TELEMETRY_HOTT
#undef TELEMETRY_LTM
#undef TELEMETRY_SMARTPORT
#undef USE_SERVOS
```

With this change, we have won space to add some missing features. With this space, we will add GPS and telemetry by iBus too, adding some *#define* that are only activated by default for flight controllers with more than 128 megabits:
```
#define GPS
#define TELEMETRY_IBUS
```

Be careful, some features are dependent, for example, the `TELEMETRY_IBUS` needs the `TELEMETRY` enabled.

To customize more if needed, this file defines all the hardware supported by different versions of the flight controller, for example, in the `NAZE` file:
```
#define GYRO
#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500

#define ACC
#define USE_ACC_ADXL345
#define USE_ACC_BMA280
#define USE_ACC_MMA8452
#define USE_ACC_MPU6050
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500

#define BARO
#define USE_BARO_MS5611 // needed for Flip32 board
#define USE_BARO_BMP280
```
Here you have different gyroscopes, accelerometers and barometers defined, to support all the variants of the flight controller. You can comment all the models that you're sure that your flight controller is not using and you will get some free space.

You can find too some features disabled for this flight controller, in this case, for example:
```
/*
#define MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW180_DEG
*/
```

The magnetometer is commented.  If you uncomment it, then it will be activated and you can use it in the NAZE flight controller.

After modifying this file, you only need to build your firmware again and you will have your own customized version with all and only the features that you need.

## Final considerations

As has been said several times in this document, when you activate a feature disabled by the developers, you can overcharge the processor, or include a bug in your firmware version. **So be careful and do it at your own risk**.
