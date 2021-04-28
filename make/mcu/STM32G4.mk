#
# G4 Make file include
#

ifeq ($(DEBUG_HARDFAULTS),G4)
CFLAGS               += -DDEBUG_HARDFAULTS
endif

#CMSIS
CMSIS_DIR      := $(ROOT)/lib/main/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32G4/Drivers/STM32G4xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))
EXCLUDES        = \
                stm32g4xx_hal_comp.c \
                stm32g4xx_hal_crc.c \
                stm32g4xx_hal_crc_ex.c \
                stm32g4xx_hal_cryp.c \
                stm32g4xx_hal_cryp_ex.c \
                stm32g4xx_hal_dac.c \
                stm32g4xx_hal_dac_ex.c \
                stm32g4xx_hal_dma_ex.c \
                stm32g4xx_hal_flash_ramfunc.c \
                stm33g4xx_hal_fmac.c \
                stm32g4xx_hal_hrtim.c \
                stm32g4xx_hal_i2s.c \
                stm32g4xx_hal_irda.c \
                stm32g4xx_hal_iwdg.c \
                stm32g4xx_hal_lptim.c \
                stm32g4xx_hal_msp_template.c \
                stm32g4xx_hal_nand.c \
                stm32g4xx_hal_nor.c \
                stm32g4xx_hal_opamp.c \
                stm32g4xx_hal_opamp_ex.c \
                stm32g4xx_hal_qspi.c \
                stm32g4xx_hal_rng.c \
                stm32g4xx_hal_sai.c \
                stm32g4xx_hal_sai_ex.c \
                stm32g4xx_hal_smartcard.c \
                stm32g4xx_hal_smartcard_ex.c \
                stm32g4xx_hal_smbus.c \
                stm32g4xx_hal_spi_ex.c \
                stm32g4xx_hal_sram.c \
                stm32g4xx_hal_timebase_tim_template.c \
                stm32g4xx_hal_usart.c \
                stm32g4xx_hal_usart_ex.c \
                stm32g4xx_hal_wwdg.c \
                stm32g4xx_ll_adc.c \
                stm32g4xx_ll_comp.c \
                stm32g4xx_ll_cordic.c \
                stm32g4xx_ll_crc.c \
                stm32g4xx_ll_crs.c \
                stm32g4xx_ll_dac.c \
                stm32g4xx_ll_exti.c \
                stm32g4xx_ll_fmac.c \
                stm32g4xx_ll_fmc.c \
                stm32g4xx_ll_gpio.c \
                stm32g4xx_ll_hrtim.c \
                stm32g4xx_ll_i2c.c \
                stm32g4xx_ll_lptim.c \
                stm32g4xx_ll_lpuart.c \
                stm32g4xx_ll_opamp.c \
                stm32g4xx_ll_pwr.c \
                stm32g4xx_ll_rcc.c \
                stm32g4xx_ll_rng.c \
                stm32g4xx_ll_rtc.c \
                stm32g4xx_ll_spi.c \
                stm32g4xx_ll_ucpd.c \
                stm32g4xx_ll_usart.c \
                stm32g4xx_ll_utils.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

#USB
USBCORE_DIR = $(ROOT)/lib/main/STM32G4/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/Src/*.c))
EXCLUDES    = usbd_conf_template.c
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(ROOT)/lib/main/STM32G4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/Src/*.c))
EXCLUDES   = usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

USBHID_DIR = $(ROOT)/lib/main/STM32G4/Middlewares/ST/STM32_USB_Device_Library/Class/HID
USBHID_SRC = $(notdir $(wildcard $(USBHID_DIR)/Src/*.c))

USBMSC_DIR = $(ROOT)/lib/main/STM32G4/Middlewares/ST/STM32_USB_Device_Library/Class/MSC
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
VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device/ST/STM32G4xx
VPATH           := $(VPATH):$(STDPERIPH_DIR)/Src
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCORE_DIR)/Inc \
                   $(USBCDC_DIR)/Inc \
                   $(USBHID_DIR)/Inc \
                   $(USBMSC_DIR)/Inc \
                   $(CMSIS_DIR)/Core/Include \
                   $(ROOT)/lib/main/STM32G4/Drivers/CMSIS/Device/ST/STM32G4xx/Include \
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
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion

DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -DUSE_DMA_RAM -DMAX_MPU_REGIONS=16

# G47X_TARGETS includes G47{3,4}{RE,CE,CEU}

ifeq ($(TARGET),$(filter $(TARGET),$(G47X_TARGETS)))
DEVICE_FLAGS   += -DSTM32G474xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_g474.ld
STARTUP_SRC     = startup_stm32g474xx.s
MCU_FLASH_SIZE  = 512
# Override the OPTIMISE_SPEED compiler setting to save flash space on these 512KB targets.
# Performance is only slightly affected but around 50 kB of flash are saved.
OPTIMISE_SPEED = -O2
else
$(error Unknown MCU for G4 target)
endif
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

TARGET_FLAGS    = -D$(TARGET)

VCP_SRC = \
            vcp_hal/usbd_desc.c \
            vcp_hal/usbd_conf_stm32g4xx.c \
            vcp_hal/usbd_cdc_hid.c \
            vcp_hal/usbd_cdc_interface.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/adc_stm32g4xx.c \
            drivers/bus_i2c_hal.c \
            drivers/bus_i2c_hal_init.c \
            drivers/bus_spi_hal.c \
            drivers/dma_stm32g4xx.c \
            drivers/dshot_bitbang.c \
            drivers/dshot_bitbang_decode.c \
            drivers/dshot_bitbang_ll.c \
            drivers/light_ws2811strip_hal.c \
            drivers/memprot_hal.c \
            drivers/memprot_stm32g4xx.c \
            drivers/persistent.c \
            drivers/pwm_output_dshot_shared.c \
            drivers/pwm_output_dshot_hal.c \
            drivers/timer_hal.c \
            drivers/timer_stm32g4xx.c \
            drivers/transponder_ir_io_hal.c \
            drivers/system_stm32g4xx.c \
            drivers/serial_uart_stm32g4xx.c \
            drivers/serial_uart_hal.c \
            startup/system_stm32g4xx.c

MCU_EXCLUDES = \
            drivers/bus_i2c.c \
            drivers/timer.c

# G4's MSC use the same driver layer file with F7
MSC_SRC = \
            drivers/usb_msc_common.c \
            drivers/usb_msc_f7xx.c \
            msc/usbd_storage.c

ifneq ($(filter SDCARD_SDIO,$(FEATURES)),)
MCU_COMMON_SRC += \
            drivers/sdio_g4xx.c            
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
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4
