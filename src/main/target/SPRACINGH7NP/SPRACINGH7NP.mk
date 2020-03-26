FEATURES       += VCP ONBOARDFLASH SDCARD_SDIO SPRACING_PIXEL_OSD

ifneq ($(EXST),)
EXST = yes
LD_SCRIPT       = $(LINKER_DIR)/stm32_ram_h750_exst_spracingpixelosd.ld
endif

ifneq ($(EXST),yes)
TARGET_FLASH_SIZE := 1024
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_h750_1m_spracingpixelosd.ld
endif

TARGET_SRC += \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/barometer/barometer_bmp388.c \
