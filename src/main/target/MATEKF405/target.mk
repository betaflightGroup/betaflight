F405_TARGETS    += $(TARGET)
ifeq ($(TARGET), MATEKF405MINI)
FEATURES        += VCP ONBOARDFLASH
else
FEATURES        += VCP SDCARD
endif

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/max7456.c
