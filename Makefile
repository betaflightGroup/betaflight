###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the betaflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#
###############################################################################


# Things that the user might override on the commandline
#

# The target to build, see VALID_TARGETS below
TARGET    ?= STM32F405

# Compile-time options
OPTIONS   ?=

# compile for OpenPilot BootLoader support
OPBL      ?= no

# compile for External Storage Bootloader support
EXST      ?= no

# compile for target loaded into RAM
RAM_BASED ?= no

# reserve space for custom defaults
CUSTOM_DEFAULTS_EXTENDED ?= no

# Debugger optons:
#   empty           - ordinary build with all optimizations enabled
#   RELWITHDEBINFO  - ordinary build with debug symbols and all optimizations enabled
#   GDB             - debug build with minimum number of optimizations
DEBUG     ?=

# Insert the debugging hardfault debugger
# releases should not be built with this flag as it does not disable pwm output
DEBUG_HARDFAULTS ?=

# Serial port/Device for flashing
SERIAL_DEVICE   ?= $(firstword $(wildcard /dev/ttyACM*) $(firstword $(wildcard /dev/ttyUSB*) no-port-found))

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=


###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME      = betaflight

# Working directories
ROOT            := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR         := $(ROOT)/src/main
OBJECT_DIR      := $(ROOT)/obj/main
BIN_DIR         := $(ROOT)/obj
CMSIS_DIR       := $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS    := $(SRC_DIR) \
                   $(ROOT)/src/main/target \
                   $(ROOT)/src/main/startup
LINKER_DIR      := $(ROOT)/src/link

## V                 : Set verbosity level based on the V= parameter
##                     V=0 Low
##                     V=1 High
include $(ROOT)/make/build_verbosity.mk

# Build tools, so we all share the same versions
# import macros common to all supported build systems
include $(ROOT)/make/system-id.mk

# developer preferences, edit these at will, they'll be gitignored
-include $(ROOT)/make/local.mk

# pre-build sanity checks
include $(ROOT)/make/checks.mk

# configure some directories that are relative to wherever ROOT_DIR is located
TOOLS_DIR ?= $(ROOT)/tools
DL_DIR    := $(ROOT)/downloads

export RM := rm

# import macros that are OS specific
include $(ROOT)/make/$(OSFAMILY).mk

# include the tools makefile
include $(ROOT)/make/tools.mk

# default xtal value for F4 targets
HSE_VALUE       ?= 8000000

# used for turning on features like VCP and SDCARD
FEATURES        =

# used to disable features based on flash space shortage (larger number => more features disabled)
FEATURE_CUT_LEVEL_SUPPLIED := $(FEATURE_CUT_LEVEL)
FEATURE_CUT_LEVEL =

# The list of targets to build for 'pre-push'
PRE_PUSH_TARGET_LIST ?= $(UNIFIED_TARGETS) NUCLEOH743 SITL STM32F4DISCOVERY_DEBUG test-representative

include $(ROOT)/make/targets.mk

REVISION := norevision
ifeq ($(shell git diff --shortstat),)
REVISION := $(shell git log -1 --format="%h")
endif

FC_VER_MAJOR := $(shell grep " FC_VERSION_MAJOR" src/main/build/version.h | awk '{print $$3}' )
FC_VER_MINOR := $(shell grep " FC_VERSION_MINOR" src/main/build/version.h | awk '{print $$3}' )
FC_VER_PATCH := $(shell grep " FC_VERSION_PATCH" src/main/build/version.h | awk '{print $$3}' )

FC_VER := $(FC_VER_MAJOR).$(FC_VER_MINOR).$(FC_VER_PATCH)

# Search path for sources
VPATH           := $(SRC_DIR):$(SRC_DIR)/startup
USBFS_DIR       = $(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC   = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))
FATFS_DIR       = $(ROOT)/lib/main/FatFS
FATFS_SRC       = $(notdir $(wildcard $(FATFS_DIR)/*.c))

CSOURCES        := $(shell find $(SRC_DIR) -name '*.c')

LD_FLAGS        :=
EXTRA_LD_FLAGS  :=

#
# Default Tool options - can be overridden in {mcu}.mk files.
#
ifeq ($(DEBUG),GDB)
OPTIMISE_DEFAULT      := -Og

LTO_FLAGS             := $(OPTIMISE_DEFAULT)
DEBUG_FLAGS            = -ggdb3 -DDEBUG
else
ifeq ($(DEBUG),INFO)
DEBUG_FLAGS            = -ggdb3
endif
OPTIMISATION_BASE     := -flto -fuse-linker-plugin -ffast-math
OPTIMISE_DEFAULT      := -O2
OPTIMISE_SPEED        := -Ofast
OPTIMISE_SIZE         := -Os

LTO_FLAGS             := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
endif

VPATH 			:= $(VPATH):$(ROOT)/make/mcu
VPATH 			:= $(VPATH):$(ROOT)/make

# start specific includes
include $(ROOT)/make/mcu/$(TARGET_MCU).mk

# openocd specific includes
include $(ROOT)/make/openocd.mk

# Configure default flash sizes for the targets (largest size specified gets hit first) if flash not specified already.
ifeq ($(TARGET_FLASH_SIZE),)
ifneq ($(MCU_FLASH_SIZE),)
TARGET_FLASH_SIZE := $(MCU_FLASH_SIZE)
else
$(error MCU_FLASH_SIZE not configured for target $(TARGET))
endif
endif

DEVICE_FLAGS  := $(DEVICE_FLAGS) -DTARGET_FLASH_SIZE=$(TARGET_FLASH_SIZE)

ifneq ($(HSE_VALUE),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DHSE_VALUE=$(HSE_VALUE)
endif

ifneq ($(FEATURE_CUT_LEVEL_SUPPLIED),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DFEATURE_CUT_LEVEL=$(FEATURE_CUT_LEVEL_SUPPLIED)
else ifneq ($(FEATURE_CUT_LEVEL),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DFEATURE_CUT_LEVEL=$(FEATURE_CUT_LEVEL)
endif

TARGET_DIR     = $(ROOT)/src/main/target/$(BASE_TARGET)
TARGET_DIR_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

ifeq ($(OPBL),yes)
TARGET_FLAGS := -DOPBL $(TARGET_FLAGS)
.DEFAULT_GOAL := binary
else
.DEFAULT_GOAL := hex
endif

ifeq ($(CUSTOM_DEFAULTS_EXTENDED),yes)
TARGET_FLAGS += -DUSE_CUSTOM_DEFAULTS=
EXTRA_LD_FLAGS += -Wl,--defsym=USE_CUSTOM_DEFAULTS_EXTENDED=1
endif

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(ROOT)/lib/main/MAVLink

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_DIR)

VPATH           := $(VPATH):$(TARGET_DIR)

include $(ROOT)/make/source.mk

###############################################################################
# Things that might need changing to use different tools
#

# Find out if ccache is installed on the system
CCACHE := ccache
RESULT = $(shell (which $(CCACHE) > /dev/null 2>&1; echo $$?) )
ifneq ($(RESULT),0)
CCACHE :=
endif

# Tool names
CROSS_CC    := $(CCACHE) $(ARM_SDK_PREFIX)gcc
CROSS_CXX   := $(CCACHE) $(ARM_SDK_PREFIX)g++
CROSS_GDB   := $(ARM_SDK_PREFIX)gdb
OBJCOPY     := $(ARM_SDK_PREFIX)objcopy
OBJDUMP     := $(ARM_SDK_PREFIX)objdump
SIZE        := $(ARM_SDK_PREFIX)size
DFUSE-PACK  := src/utils/dfuse-pack.py

#
# Tool options.
#
CC_DEBUG_OPTIMISATION   := $(OPTIMISE_DEFAULT)
CC_DEFAULT_OPTIMISATION := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
CC_SPEED_OPTIMISATION   := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
CC_SIZE_OPTIMISATION    := $(OPTIMISATION_BASE) $(OPTIMISE_SIZE)
CC_NO_OPTIMISATION      := 

#
# Added after GCC version update, remove once the warnings have been fixed
#
TEMPORARY_FLAGS :=

CFLAGS     += $(ARCH_FLAGS) \
              $(addprefix -D,$(OPTIONS)) \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              $(DEBUG_FLAGS) \
              -std=gnu11 \
              -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
              -ffunction-sections \
              -fdata-sections \
              -fno-common \
              -pedantic \
              $(TEMPORARY_FLAGS) \
              $(DEVICE_FLAGS) \
              -D_GNU_SOURCE \
              -DUSE_STDPERIPH_DRIVER \
              -D$(TARGET) \
              $(TARGET_FLAGS) \
              -D'__FORKNAME__="$(FORKNAME)"' \
              -D'__TARGET__="$(TARGET)"' \
              -D'__REVISION__="$(REVISION)"' \
              -save-temps=obj \
              -MMD -MP \
              $(EXTRA_FLAGS)

ASFLAGS     = $(ARCH_FLAGS) \
              $(DEBUG_FLAGS) \
              -x assembler-with-cpp \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              -MMD -MP

ifeq ($(LD_FLAGS),)
LD_FLAGS     = -lm \
              -nostartfiles \
              --specs=nano.specs \
              -lc \
              -lnosys \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -static \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -Wl,--no-wchar-size-warning \
              -Wl,--print-memory-usage \
              -T$(LD_SCRIPT) \
               $(EXTRA_LD_FLAGS)
endif

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK        = cppcheck $(CSOURCES) --enable=all --platform=unix64 \
                  --std=c99 --inline-suppr --quiet --force \
                  $(addprefix -I,$(INCLUDE_DIRS)) \
                  -I/usr/include -I/usr/include/linux


TARGET_BASENAME = $(BIN_DIR)/$(FORKNAME)_$(FC_VER)_$(TARGET)_$(REVISION)

#
# Things we will build
#
TARGET_BIN      = $(TARGET_BASENAME).bin
TARGET_HEX      = $(TARGET_BASENAME).hex
TARGET_DFU      = $(TARGET_BASENAME).dfu
TARGET_ZIP      = $(TARGET_BASENAME).zip
TARGET_ELF      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_EXST_ELF = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET)_EXST.elf
TARGET_UNPATCHED_BIN = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET)_UNPATCHED.bin
TARGET_LST      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).lst
TARGET_OBJS     = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(SRC))))
TARGET_DEPS     = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(SRC))))
TARGET_MAP      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

TARGET_EXST_HASH_SECTION_FILE = $(OBJECT_DIR)/$(TARGET)/exst_hash_section.bin

CLEAN_ARTIFACTS := $(TARGET_BIN)
CLEAN_ARTIFACTS += $(TARGET_HEX)
CLEAN_ARTIFACTS += $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
CLEAN_ARTIFACTS += $(TARGET_LST)
CLEAN_ARTIFACTS += $(TARGET_DFU)

# Make sure build date and revision is updated on every incremental build
$(OBJECT_DIR)/$(TARGET)/build/version.o : $(SRC)

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_LST): $(TARGET_ELF)
	$(V0) $(OBJDUMP) -S --disassemble $< > $@

ifeq ($(EXST),no)
$(TARGET_BIN): $(TARGET_ELF)
	@echo "Creating BIN $(TARGET_BIN)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O binary $< $@
	
$(TARGET_HEX): $(TARGET_ELF)
	@echo "Creating HEX $(TARGET_HEX)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_DFU): $(TARGET_HEX)
	@echo "Creating DFU $(TARGET_DFU)" "$(STDOUT)"
	$(V1) $(PYTHON) $(DFUSE-PACK) -i $< $@

else
CLEAN_ARTIFACTS += $(TARGET_UNPATCHED_BIN) $(TARGET_EXST_HASH_SECTION_FILE) $(TARGET_EXST_ELF)

$(TARGET_UNPATCHED_BIN): $(TARGET_ELF)
	@echo "Creating BIN (without checksum) $(TARGET_UNPATCHED_BIN)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O binary $< $@

$(TARGET_BIN): $(TARGET_UNPATCHED_BIN)
	@echo "Creating EXST $(TARGET_BIN)" "$(STDOUT)"
# Linker script should allow .bin generation from a .elf which results in a file that is the same length as the FIRMWARE_SIZE.
# These 'dd' commands will pad a short binary to length FIRMWARE_SIZE.
	$(V1) dd if=/dev/zero ibs=1k count=$(FIRMWARE_SIZE) of=$(TARGET_BIN)
	$(V1) dd if=$(TARGET_UNPATCHED_BIN) of=$(TARGET_BIN) conv=notrunc

	@echo "Generating MD5 hash of binary" "$(STDOUT)"
	$(V1) openssl dgst -md5 $(TARGET_BIN) > $(TARGET_UNPATCHED_BIN).md5 
	
	@echo "Patching MD5 hash into binary" "$(STDOUT)"
	$(V1) cat $(TARGET_UNPATCHED_BIN).md5 | awk '{printf("%08x: %s",(1024*$(FIRMWARE_SIZE))-16,$$2);}' | xxd -r - $(TARGET_BIN)
	$(V1) echo $(FIRMWARE_SIZE) | awk '{printf("-s 0x%08x -l 16 -c 16 %s",(1024*$$1)-16,"$(TARGET_BIN)");}' | xargs xxd

# Note: From the objcopy manual "If you do not specify outfile, objcopy creates a temporary file and destructively renames the result with the name of infile"
# Due to this a temporary file must be created and removed, even though we're only extracting data from the input file.
# If this temporary file is NOT used the $(TARGET_ELF) is modified, and running make a second time will result in
# a) regeneration of $(TARGET_BIN), and
# b) the results of $(TARGET_BIN) will not be as expected.
	@echo "Extracting HASH section from unpatched EXST elf $(TARGET_ELF)" "$(STDOUT)"
	$(OBJCOPY) $(TARGET_ELF) $(TARGET_EXST_ELF).tmp --dump-section .exst_hash=$(TARGET_EXST_HASH_SECTION_FILE)
	rm $(TARGET_EXST_ELF).tmp
	
	@echo "Patching MD5 hash into HASH section" "$(STDOUT)"
	$(V1) cat $(TARGET_UNPATCHED_BIN).md5 | awk '{printf("%08x: %s",64-16,$$2);}' | xxd -r - $(TARGET_EXST_HASH_SECTION_FILE)
	
	@echo "Patching updated HASH section into $(TARGET_EXST_ELF)" "$(STDOUT)"
	$(OBJCOPY) $(TARGET_ELF) $(TARGET_EXST_ELF) --update-section .exst_hash=$(TARGET_EXST_HASH_SECTION_FILE)

$(TARGET_HEX): $(TARGET_BIN)
	$(if $(EXST_ADJUST_VMA),,$(error "EXST_ADJUST_VMA not specified"))

	@echo "Creating EXST HEX from patched EXST BIN $(TARGET_BIN), VMA Adjust $(EXST_ADJUST_VMA)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -I binary -O ihex --adjust-vma=$(EXST_ADJUST_VMA) $(TARGET_BIN) $@

endif

$(TARGET_ELF): $(TARGET_OBJS) $(LD_SCRIPT)
	@echo "Linking $(TARGET)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -o $@ $(filter-out %.ld,$^) $(LD_FLAGS)
	$(V1) $(SIZE) $(TARGET_ELF)

# Compile

## compile_file takes two arguments: (1) optimisation description string and (2) optimisation compiler flag
define compile_file
	echo "%% ($(1)) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(2) $<
endef

ifeq ($(DEBUG),GDB)
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(if $(findstring $<,$(NOT_OPTIMISED_SRC)), \
		$(call compile_file,not optimised, $(CC_NO_OPTIMISATION)) \
	, \
		$(call compile_file,debug,$(CC_DEBUG_OPTIMISATION)) \
	)
else
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(if $(findstring $<,$(NOT_OPTIMISED_SRC)), \
		$(call compile_file,not optimised,$(CC_NO_OPTIMISATION)) \
	, \
		$(if $(findstring $(subst ./src/main/,,$<),$(SPEED_OPTIMISED_SRC)), \
			$(call compile_file,speed optimised,$(CC_SPEED_OPTIMISATION)) \
		, \
			$(if $(findstring $(subst ./src/main/,,$<),$(SIZE_OPTIMISED_SRC)), \
				$(call compile_file,size optimised,$(CC_SIZE_OPTIMISATION)) \
			, \
				$(call compile_file,optimised,$(CC_DEFAULT_OPTIMISATION)) \
			) \
		) \
	)
endif

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	$(V1) mkdir -p $(dir $@)
	@echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	$(V1) mkdir -p $(dir $@)
	@echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<


## all               : Build all currently built targets
all: $(CI_TARGETS)

## all_all : Build all targets (including legacy / unsupported)
all_all: $(VALID_TARGETS)

## unified : build all Unified Targets
unified: $(UNIFIED_TARGETS)

## unified_zip : build all Unified Targets as zip files (for posting on GitHub)
unified_zip: $(addsuffix _clean,$(UNIFIED_TARGETS)) $(addsuffix _zip,$(UNIFIED_TARGETS))

## legacy : Build legacy targets
legacy: $(LEGACY_TARGETS)

## unsupported : Build unsupported targets
unsupported: $(UNSUPPORTED_TARGETS)

## pre-push : The minimum verification that should be run before pushing, to check if CI has a chance of succeeding
pre-push:
	$(MAKE) $(addsuffix _clean,$(PRE_PUSH_TARGET_LIST)) $(PRE_PUSH_TARGET_LIST) EXTRA_FLAGS=-Werror

## targets-group-1   : build some targets
targets-group-1: $(GROUP_1_TARGETS)

## targets-group-2   : build some targets
targets-group-2: $(GROUP_2_TARGETS)

## targets-group-rest: build the rest of the targets (not listed in the other groups)
targets-group-rest: $(GROUP_OTHER_TARGETS)

$(VALID_TARGETS):
	$(V0) @echo "Building $@" && \
	$(MAKE) binary hex TARGET=$@ && \
	echo "Building $@ succeeded."

$(NOBUILD_TARGETS):
	$(MAKE) TARGET=$@

TARGETS_CLEAN = $(addsuffix _clean,$(VALID_TARGETS))

## clean             : clean up temporary / machine-generated files
clean:
	@echo "Cleaning $(TARGET)"
	$(V0) rm -f $(CLEAN_ARTIFACTS)
	$(V0) rm -rf $(OBJECT_DIR)/$(TARGET)
	@echo "Cleaning $(TARGET) succeeded."

## test_clean        : clean up temporary / machine-generated files (tests)
test-%_clean:
	$(MAKE) test_clean

test_clean:
	$(V0) cd src/test && $(MAKE) clean || true

## <TARGET>_clean    : clean up one specific target (alias for above)
$(TARGETS_CLEAN):
	$(V0) $(MAKE) -j TARGET=$(subst _clean,,$@) clean

## clean_all         : clean all valid targets
clean_all: $(TARGETS_CLEAN) test_clean

TARGETS_FLASH = $(addsuffix _flash,$(VALID_TARGETS))

## <TARGET>_flash    : build and flash a target
$(TARGETS_FLASH):
	$(V0) $(MAKE) hex TARGET=$(subst _flash,,$@)
ifneq (,$(findstring /dev/ttyUSB,$(SERIAL_DEVICE)))
	$(V0) $(MAKE) tty_flash TARGET=$(subst _flash,,$@)
else
	$(V0) $(MAKE) dfu_flash TARGET=$(subst _flash,,$@)
endif

## tty_flash         : flash firmware (.hex) onto flight controller via a serial port
tty_flash:
	$(V0) stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	$(V0) echo -n 'R' > $(SERIAL_DEVICE)
	$(V0) stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## dfu_flash         : flash firmware (.bin) onto flight controller via a DFU mode
dfu_flash:
ifneq (no-port-found,$(SERIAL_DEVICE))
	# potentially this is because the MCU already is in DFU mode, try anyway
	$(V0) echo -n 'R' > $(SERIAL_DEVICE)
	$(V0) sleep 1
endif
	$(V0) $(MAKE) $(TARGET_DFU)
	$(V0) dfu-util -a 0 -D $(TARGET_DFU) -s :leave

st-flash_$(TARGET): $(TARGET_BIN)
	$(V0) st-flash --reset write $< 0x08000000

## st-flash          : flash firmware (.bin) onto flight controller
st-flash: st-flash_$(TARGET)

ifneq ($(OPENOCD_COMMAND),)
openocd-gdb: $(TARGET_ELF)
	$(V0) $(OPENOCD_COMMAND) & $(CROSS_GDB) $(TARGET_ELF) -ex "target remote localhost:3333" -ex "load"
endif

TARGETS_ZIP = $(addsuffix _zip,$(VALID_TARGETS))

## <TARGET>_zip    : build target and zip it (useful for posting to GitHub)
$(TARGETS_ZIP):
	$(V0) $(MAKE) hex TARGET=$(subst _zip,,$@)
	$(V0) $(MAKE) zip TARGET=$(subst _zip,,$@)

zip:
	$(V0) zip $(TARGET_ZIP) $(TARGET_HEX)

binary:
	$(V0) $(MAKE) -j $(TARGET_BIN)

hex:
	$(V0) $(MAKE) -j $(TARGET_HEX)

unbrick_$(TARGET): $(TARGET_HEX)
	$(V0) stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	$(V0) stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## unbrick           : unbrick flight controller
unbrick: unbrick_$(TARGET)

## cppcheck          : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(V0) $(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(V0) $(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

# mkdirs
$(DL_DIR):
	mkdir -p $@

$(TOOLS_DIR):
	mkdir -p $@

## version           : print firmware version
version:
	@echo $(FC_VER)

## help              : print this help message and exit
help: Makefile make/tools.mk
	@echo ""
	@echo "Makefile for the $(FORKNAME) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [V=<verbosity>] [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo "Or:"
	@echo "        make <target> [V=<verbosity>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
	@sed -n 's/^## //p' $?

## targets           : print a list of all valid target platforms (for consumption by scripts)
targets:
	@echo "Valid targets:       $(VALID_TARGETS)"
	@echo "Built targets:       $(CI_TARGETS)"
	@echo "Unified targets:     $(UNIFIED_TARGETS)"
	@echo "Legacy targets:      $(LEGACY_TARGETS)"
	@echo "Unsupported targets: $(UNSUPPORTED_TARGETS)"
	@echo "Target:              $(TARGET)"
	@echo "Base target:         $(BASE_TARGET)"
	@echo "targets-group-1:     $(GROUP_1_TARGETS)"
	@echo "targets-group-2:     $(GROUP_2_TARGETS)"
	@echo "targets-group-rest:  $(GROUP_OTHER_TARGETS)"

	@echo "targets-group-1:     $(words $(GROUP_1_TARGETS)) targets"
	@echo "targets-group-2:     $(words $(GROUP_2_TARGETS)) targets"
	@echo "targets-group-rest:  $(words $(GROUP_OTHER_TARGETS)) targets"
	@echo "total in all groups  $(words $(CI_TARGETS)) targets"

## target-mcu        : print the MCU type of the target
target-mcu:
	@echo $(TARGET_MCU)

## targets-by-mcu    : make all targets that have a MCU_TYPE mcu
targets-by-mcu:
	$(V1) for target in $${TARGETS}; do \
		TARGET_MCU_TYPE=$$($(MAKE) -s TARGET=$${target} target-mcu); \
		if [ "$${TARGET_MCU_TYPE}" = "$${MCU_TYPE}" ]; then \
			if [ "$${DO_BUILD}" = 1 ]; then \
				echo "Building target $${target}..."; \
				$(MAKE) TARGET=$${target}; \
				if [ $$? -ne 0 ]; then \
					echo "Building target $${target} failed, aborting."; \
					exit 1; \
				fi; \
			else \
				echo -n "$${target} "; \
			fi; \
		fi; \
	done
	@echo

## targets-f3        : make all F3 targets
targets-f3:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F3 TARGETS="$(VALID_TARGETS)" DO_BUILD=1

targets-f3-print:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F3 TARGETS="$(VALID_TARGETS)"

## targets-f4        : make all F4 targets
targets-f4:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F4 TARGETS="$(VALID_TARGETS)" DO_BUILD=1

targets-f4-print:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F4 TARGETS="$(VALID_TARGETS)"

targets-ci-f4-print:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F4 TARGETS="$(CI_TARGETS)"

## targets-f7        : make all F7 targets
targets-f7:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F7 TARGETS="$(VALID_TARGETS)" DO_BUILD=1

targets-f7-print:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F7 TARGETS="$(VALID_TARGETS)"

targets-ci-f7-print:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F7 TARGETS="$(CI_TARGETS)"

## test              : run the Betaflight test suite
## junittest         : run the Betaflight test suite, producing Junit XML result files.
## test-representative: run a representative subset of the Betaflight test suite (i.e. run all tests, but run each expanded test only for one target)
## test-all: run the Betaflight test suite including all per-target expanded tests
test junittest test-all test-representative:
	$(V0) cd src/test && $(MAKE) $@

## test_help         : print the help message for the test suite (including a list of the available tests)
test_help:
	$(V0) cd src/test && $(MAKE) help

## test_versions         : print the compiler versions used for the test suite
test_versions:
	$(V0) cd src/test && $(MAKE) versions

## test_%            : run test 'test_%' from the test suite
test_%:
	$(V0) cd src/test && $(MAKE) $@


# rebuild everything when makefile changes
$(TARGET_OBJS): Makefile $(TARGET_DIR)/target.mk $(wildcard make/*)

# include auto-generated dependencies
-include $(TARGET_DEPS)
