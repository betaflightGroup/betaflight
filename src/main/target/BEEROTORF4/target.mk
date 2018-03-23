F405_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD MSC

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/max7456.c \
            drivers/transponder_ir.c \
            io/transponder_ir.c \
			msc/usbd_storage_sd_spi.c \
			msc/usbd_msc_desc.c
