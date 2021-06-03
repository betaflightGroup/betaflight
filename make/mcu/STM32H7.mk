#
# H7 Make file include
#

ifeq ($(DEBUG_HARDFAULTS),H7)
CFLAGS               += -DDEBUG_HARDFAULTS
endif

#CMSIS
CMSIS_DIR      := $(ROOT)/lib/main/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32H7/Drivers/STM32H7xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))

EXCLUDES        = \
                stm32h7xx_hal_cec.c \
                stm32h7xx_hal_comp.c \
                stm32h7xx_hal_crc.c \
                stm32h7xx_hal_crc_ex.c \
                stm32h7xx_hal_cryp.c \
                stm32h7xx_hal_cryp_ex.c \
                stm32h7xx_hal_dcmi.c \
                stm32h7xx_hal_dfsdm.c \
                stm32h7xx_hal_dma2d.c \
                stm32h7xx_hal_dsi.c \
                stm32h7xx_hal_eth.c \
                stm32h7xx_hal_eth_ex.c \
                stm32h7xx_hal_fdcan.c \
                stm32h7xx_hal_hash.c \
                stm32h7xx_hal_hash_ex.c \
                stm32h7xx_hal_hcd.c \
                stm32h7xx_hal_hrtim.c \
                stm32h7xx_hal_hsem.c \
                stm32h7xx_hal_i2s.c \
                stm32h7xx_hal_i2s_ex.c \
                stm32h7xx_hal_irda.c \
                stm32h7xx_hal_iwdg.c \
                stm32h7xx_hal_jpeg.c \
                stm32h7xx_hal_lptim.c \
                stm32h7xx_hal_ltdc.c \
                stm32h7xx_hal_ltdc_ex.c \
                stm32h7xx_hal_mdios.c \
                stm32h7xx_hal_mdma.c \
                stm32h7xx_hal_mmc.c \
                stm32h7xx_hal_mmc_ex.c \
                stm32h7xx_hal_msp_template.c \
                stm32h7xx_hal_nand.c \
                stm32h7xx_hal_nor.c \
                stm32h7xx_hal_opamp.c \
                stm32h7xx_hal_opamp_ex.c \
                stm32h7xx_hal_ramecc.c \
                stm32h7xx_hal_rng.c \
                stm32h7xx_hal_rtc.c \
                stm32h7xx_hal_sai.c \
                stm32h7xx_hal_sai_ex.c \
                stm32h7xx_hal_sd_ex.c \
                stm32h7xx_hal_sdram.c \
                stm32h7xx_hal_smartcard.c \
                stm32h7xx_hal_smartcard_ex.c \
                stm32h7xx_hal_smbus.c \
                stm32h7xx_hal_spdifrx.c \
                stm32h7xx_hal_spi.c \
                stm32h7xx_hal_sram.c \
                stm32h7xx_hal_swpmi.c \
                stm32h7xx_hal_usart.c \
                stm32h7xx_hal_usart_ex.c \
                stm32h7xx_hal_wwdg.c \
                stm32h7xx_ll_adc.c \
                stm32h7xx_ll_bdma.c \
                stm32h7xx_ll_comp.c \
                stm32h7xx_ll_crc.c \
                stm32h7xx_ll_dac.c \
                stm32h7xx_ll_delayblock.c \
                stm32h7xx_ll_dma2d.c \
                stm32h7xx_ll_exti.c \
                stm32h7xx_ll_fmc.c \
                stm32h7xx_ll_gpio.c \
                stm32h7xx_ll_hrtim.c \
                stm32h7xx_ll_i2c.c \
                stm32h7xx_ll_lptim.c \
                stm32h7xx_ll_lpuart.c \
                stm32h7xx_ll_mdma.c \
                stm32h7xx_ll_opamp.c \
                stm32h7xx_ll_pwr.c \
                stm32h7xx_ll_rcc.c \
                stm32h7xx_ll_rng.c \
                stm32h7xx_ll_rtc.c \
                stm32h7xx_ll_swpmi.c \
                stm32h7xx_ll_usart.c \
                stm32h7xx_ll_utils.c


STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

#USB
USBCORE_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/Src/*.c))
EXCLUDES    = usbd_conf_template.c
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/Src/*.c))
EXCLUDES   = usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

USBHID_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Class/HID
USBHID_SRC = $(notdir $(wildcard $(USBHID_DIR)/Src/*.c))

USBMSC_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Class/MSC
USBMSC_SRC = $(notdir $(wildcard $(USBMSC_DIR)/Src/*.c))
EXCLUDES   = usbd_msc_storage_template.c
USBMSC_SRC := $(filter-out ${EXCLUDES}, $(USBMSC_SRC))

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src:$(USBHID_DIR)/Src:$(USBMSC_DIR)/Src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC) \
                        $(USBHID_SRC) \
                        $(USBMSC_SRC)

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device/ST/STM32H7xx
VPATH           := $(VPATH):$(STDPERIPH_DIR)/Src
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCORE_DIR)/Inc \
                   $(USBCDC_DIR)/Inc \
                   $(USBHID_DIR)/Inc \
                   $(USBMSC_DIR)/Inc \
                   $(CMSIS_DIR)/Core/Include \
                   $(ROOT)/lib/main/STM32H7/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
                   $(ROOT)/src/main/vcp_hal

ifneq ($(filter SDCARD_SPI,$(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR)
VPATH           := $(VPATH):$(FATFS_DIR)
endif

ifneq ($(filter SDCARD_SDIO,$(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR)
VPATH           := $(VPATH):$(FATFS_DIR)
endif

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -fsingle-precision-constant -Wdouble-promotion

# Flags that are used in the STM32 libraries
DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER

#
# H743xI : 2M FLASH, 512KB AXI SRAM + 512KB D2 & D3 SRAM (H753xI also)
# H743xG : 1M FLASH, 512KB AXI SRAM + 512KB D2 & D3 SRAM (H753xG also)
# H7A3xI : 2M FLASH, 1MB   AXI SRAM + 160KB AHB & SRD SRAM
# H750xB : 128K FLASH, 1M RAM
#
ifeq ($(TARGET),$(filter $(TARGET),$(H743xI_TARGETS)))
DEVICE_FLAGS       += -DSTM32H743xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h743_2m.ld
STARTUP_SRC         = startup_stm32h743xx.s
MCU_FLASH_SIZE     := 2048
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16

ifeq ($(RAM_BASED),yes)
FIRMWARE_SIZE      := 448
# TARGET_FLASH now becomes the amount of RAM memory that is occupied by the firmware
# and the maximum size of the data stored on the external storage device.
MCU_FLASH_SIZE     := FIRMWARE_SIZE
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_ram_h743.ld
endif

else ifeq ($(TARGET),$(filter $(TARGET),$(H7A3xIQ_TARGETS)))
DEVICE_FLAGS       += -DSTM32H7A3xxQ
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h7a3_2m.ld
STARTUP_SRC         = startup_stm32h7a3xx.s
MCU_FLASH_SIZE     := 2048
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16

ifeq ($(RAM_BASED),yes)
FIRMWARE_SIZE      := 448
# TARGET_FLASH now becomes the amount of RAM memory that is occupied by the firmware
# and the maximum size of the data stored on the external storage device.
MCU_FLASH_SIZE     := FIRMWARE_SIZE
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h7a3_ram_based.ld
endif

else ifeq ($(TARGET),$(filter $(TARGET),$(H7A3xI_TARGETS)))
DEVICE_FLAGS       += -DSTM32H7A3xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h7a3_2m.ld
STARTUP_SRC         = startup_stm32h7a3xx.s
MCU_FLASH_SIZE     := 2048
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16

ifeq ($(RAM_BASED),yes)
FIRMWARE_SIZE      := 448
# TARGET_FLASH now becomes the amount of RAM memory that is occupied by the firmware
# and the maximum size of the data stored on the external storage device.
MCU_FLASH_SIZE     := FIRMWARE_SIZE
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h7a3_ram_based.ld
endif

else ifeq ($(TARGET),$(filter $(TARGET),$(H723xG_TARGETS)))
DEVICE_FLAGS       += -DSTM32H723xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h723_1m.ld
STARTUP_SRC         = startup_stm32h723xx.s
MCU_FLASH_SIZE     := 1024
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16

else ifeq ($(TARGET),$(filter $(TARGET),$(H725xG_TARGETS)))
DEVICE_FLAGS       += -DSTM32H725xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h723_1m.ld
STARTUP_SRC         = startup_stm32h723xx.s
MCU_FLASH_SIZE     := 1024
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16

else ifeq ($(TARGET),$(filter $(TARGET),$(H750xB_TARGETS)))
DEVICE_FLAGS       += -DSTM32H750xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h750_128k.ld
STARTUP_SRC         = startup_stm32h743xx.s
DEFAULT_TARGET_FLASH := 128

ifeq ($(TARGET_FLASH),)
MCU_FLASH_SIZE := $(DEFAULT_TARGET_FLASH) 
endif

ifeq ($(EXST),yes)
FIRMWARE_SIZE      := 448
# TARGET_FLASH now becomes the amount of RAM memory that is occupied by the firmware
# and the maximum size of the data stored on the external storage device.
MCU_FLASH_SIZE     := FIRMWARE_SIZE
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_ram_h750_exst.ld
endif

ifeq ($(EXST),yes)
# Upper 8 regions are reserved for a boot loader in EXST environment
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=8
else
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16
endif

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Os
OPTIMISE_SPEED      := -Os
OPTIMISE_SIZE       := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
endif

else
$(error Unknown MCU for H7 target)
endif

ifeq ($(LD_SCRIPT),)
LD_SCRIPT = $(DEFAULT_LD_SCRIPT)
endif

ifneq ($(FIRMWARE_SIZE),)
DEVICE_FLAGS   += -DFIRMWARE_SIZE=$(FIRMWARE_SIZE) 
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DHSE_STARTUP_TIMEOUT=1000

TARGET_FLAGS    = -D$(TARGET)

VCP_SRC = \
            vcp_hal/usbd_desc.c \
            vcp_hal/usbd_conf_stm32h7xx.c \
            vcp_hal/usbd_cdc_hid.c \
            vcp_hal/usbd_cdc_interface.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            startup/system_stm32h7xx.c \
            drivers/system_stm32h7xx.c \
            drivers/timer_hal.c \
            drivers/timer_stm32h7xx.c \
            drivers/serial_uart_hal.c \
            drivers/serial_uart_stm32h7xx.c \
            drivers/bus_quadspi_hal.c \
            drivers/bus_spi_ll.c \
            drivers/dma_stm32h7xx.c \
            drivers/dshot_bitbang.c \
            drivers/dshot_bitbang_decode.c \
            drivers/dshot_bitbang_ll.c \
            drivers/light_ws2811strip_hal.c \
            drivers/adc_stm32h7xx.c \
            drivers/bus_i2c_hal.c \
            drivers/bus_i2c_hal_init.c \
            drivers/bus_i2c_timing.c \
            drivers/pwm_output_dshot_hal.c \
            drivers/pwm_output_dshot_shared.c \
            drivers/persistent.c \
            drivers/transponder_ir_io_hal.c \
            drivers/audio_stm32h7xx.c \
            drivers/memprot_hal.c \
            drivers/memprot_stm32h7xx.c \
            #drivers/accgyro/accgyro_mpu.c \

MCU_EXCLUDES = \
            drivers/bus_i2c.c \
            drivers/timer.c

MSC_SRC = \
            drivers/usb_msc_common.c \
            drivers/usb_msc_h7xx.c \
            msc/usbd_storage.c

ifneq ($(filter SDCARD_SDIO,$(FEATURES)),)
MCU_COMMON_SRC += \
            drivers/sdio_h7xx.c
MSC_SRC += \
            msc/usbd_storage_sdio.c
endif

ifneq ($(filter SDCARD_SPI,$(FEATURES)),)
MSC_SRC += \
            msc/usbd_storage_sd_spi.c
endif

ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
MSC_SRC += \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c
endif

DSP_LIB := $(ROOT)/lib/main/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM7

