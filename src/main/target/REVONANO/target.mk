F411_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
           drivers/compass/compass_ak8963.c \
           drivers/compass/compass_mpu925x_ak8963.c \
           drivers/accgyro/accgyro_spi_mpu9250.c \
           drivers/barometer/barometer_ms5611.c
