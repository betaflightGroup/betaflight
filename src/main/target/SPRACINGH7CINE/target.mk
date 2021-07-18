H750xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

FEATURES       += VCP ONBOARDFLASH SDCARD_SDIO

EXST ?= yes
EXST_ADJUST_VMA = 0x97CE0000

TARGET_SRC += \
            drivers/bus_quadspi_hal.c \
            drivers/bus_quadspi.c \

