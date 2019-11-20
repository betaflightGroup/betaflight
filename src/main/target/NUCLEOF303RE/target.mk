F3_TARGETS  += $(TARGET)

FEATURES    = VCP SDCARD_SPI

FEATURE_CUT_LEVEL = 0

FLASH_SIZE  = 512

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8963.c 

