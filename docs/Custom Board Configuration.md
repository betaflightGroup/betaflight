## Custom Board Configuration Using CLI
Warning: This section covers beyond the normal use-case for Betaflight. It's recommended that you use a pre-made target configuration for a flight controller that has been thoroughly tested before. Those configurations can be found in the [unified targets list](https://github.com/betaflight/unified-targets/tree/master/configs/default).

However, Betaflight can provide support for custom-made flight controllers, assuming it contains supported hardware. Before using Betaflight at all, it's highly recommended that you, first, check to make sure your MCU and your peripherals are supported. Checking the [Supported Sensors](https://github.com/betaflight/betaflight/wiki/Supported-Sensors) and [Hardware Reference](https://github.com/betaflight/betaflight/wiki/Hardware-Reference) wiki pages are good places to look. Betaflight is supported across many STM32 processors. Second, thoroughly test the functionality of your hardware by programming your board in an IDE (i.e. STMCube) or using any other hardware debugging methods/tools. 

Assuming your hardware is supported and you've thoroughly tested your peripherals, flash generic firmware into your board. It's recommended to flash betaflight firmware using the configurator and [USB Flashing](https://github.com/betaflight/betaflight/blob/master/docs/USB%20Flashing.md). 

To create your own configuration, you must first use the `resource` command in the command line interface to map your peripheral pins on your controller. Use the CLI documentation linked above and other wiki pages for command reference. Then, you use the `set` command to set the bus type, i2c address, lowpass filter frequency/type, baro_hardware, gyro_hardware, etc. This will mainly be dependent on your hardware and will involve a lot of trial and error. Use the `save` command to save your configuration and reboot the controller. If your hardware is not detected, then you need to modify or create more settings in the CLI to accurately describe your hardware to Betaflight to detect it. Use your custom board's and respective peripherals' datasheets and schematics to help you.

Alongside `resource` and `set` commands, there are `timer` and `dma` commands. For certain functions, such as battery voltage monitoring and motor output, you need to associate a timer and/or a Direct Memory Access (DMA) channel to those pins for those to work.
- `# timer`
- `timer <pin> AF(x) [1-3]`
- `# dma`
- `dma [SPI_TX|ADC|pin] [index|pin] [0|1]`

For custom configuration like this outside the normal use-case, it's recommended that you use the pre-made [configs](https://github.com/betaflight/unified-targets/tree/master/configs/default) as a source of reference.
