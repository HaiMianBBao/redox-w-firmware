PROJECT_NAME := redow-w-keyboard-basic

export OUTPUT_FILENAME

SDK_ROOT ?= ../../../..

TEMPLATE_PATH = $(SDK_ROOT)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO :=
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(SDK_ROOT)/components/toolchain/system_nrf51.c \
$(SDK_ROOT)/components/drivers_nrf/delay/nrf_delay.c \
$(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
$(SDK_ROOT)/components/libraries/util/app_util_platform.c \
$(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
$(SDK_ROOT)/components/drivers_nrf/rtc/nrf_drv_rtc.c \
$(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
$(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
$(abspath ../../pmw3610/pmw3610.c) \
$(abspath ../../pmw3610/port_nrf.c) \
$(abspath ../../spi/iopincfg_nrfx.c) \
$(abspath ../../spi/spi_nrfx.c) 

#assembly files common to all targets
ASM_SOURCE_FILES  = $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.s

#assembly files common to all targets
LIBS  = $(SDK_ROOT)/components/properitary_rf/gzll/gcc/gzll_gcc.a

#includes common to all targets
INC_PATHS  = -I$(abspath ../../config)
INC_PATHS += -I$(SDK_ROOT)/components/device
INC_PATHS += -I$(SDK_ROOT)/components/toolchain/CMSIS/Include
INC_PATHS += -I$(SDK_ROOT)/components/properitary_rf/gzll
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/hal
INC_PATHS += -I$(SDK_ROOT)/components/toolchain/gcc
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/delay
INC_PATHS += -I$(SDK_ROOT)/examples/bsp
INC_PATHS += -I$(SDK_ROOT)/components/toolchain
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/common
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/clock
INC_PATHS += -I$(SDK_ROOT)/components/libraries/util
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/config
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/rtc
INC_PATHS += -I$(SDK_ROOT)/external/segger_rtt
INC_PATHS += -I$(abspath ../../pmw3610)
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/gpiote
INC_PATHS += -I$(abspath ../../spi)

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DNRF51
CFLAGS += -DGAZELL_PRESENT
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DNRF_LOG_USES_RTT=1
CFLAGS += -DGPIO_COUNT=0
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -O3 -g3
CFLAGS += -Wno-unused-function
CFLAGS += -Wno-unused-variable
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums
CFLAGS += -Wno-format

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys
#suppress wchar errors
LDFLAGS += -Wl,--no-wchar-size-warning

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF51
ASMFLAGS += -DGAZELL_PRESENT
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DBSP_DEFINES_ONLY

#default target - first one defined
default: $(BUILD_DIRECTORIES) all

#building all targets
all: $(OUTPUT_BINARY_DIRECTORY)/nrf51822_xxac-keyboard-left.hex  \
     $(OUTPUT_BINARY_DIRECTORY)/nrf51822_xxac-keyboard-right.hex \
	 $(OUTPUT_BINARY_DIRECTORY)/nrf51822_xxac-keyboard-left.asm \
     $(OUTPUT_BINARY_DIRECTORY)/nrf51822_xxac-keyboard-right.asm

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

LINKER_SCRIPT=gzll_gcc_nrf51.ld

.SECONDARY:

## Create build directories
$(BUILD_DIRECTORIES):
	$(MK) $@

$(OBJECT_DIRECTORY)/main-keyboard-left.o: CFLAGS += -DCOMPILE_LEFT
$(OBJECT_DIRECTORY)/main-keyboard-right.o: CFLAGS += -DCOMPILE_RIGHT

$(OBJECT_DIRECTORY)/main-%.o: $(abspath ../../main.c)
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

$(OUTPUT_BINARY_DIRECTORY)/nrf51822_xxac-%.out: $(OBJECTS) $(OBJECT_DIRECTORY)/main-%.o
	@echo Linking target: $@
	$(NO_ECHO)$(CC) $(LDFLAGS) $^ $(LIBS) -lm -o $@

$(OUTPUT_BINARY_DIRECTORY)/nrf51822_xxac-%.asm: $(OUTPUT_BINARY_DIRECTORY)/nrf51822_xxac-%.out
	@echo Generating assembly file: $@
	$(NO_ECHO)$(OBJDUMP) -S $< > $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<

%.bin: %.out
	@echo Preparing: $@
	$(NO_ECHO)$(OBJCOPY) -O binary $< $@
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out

## Create binary .hex file from the .out file
%.hex: %.out
	@echo Preparing: $@
	$(NO_ECHO)$(OBJCOPY) -O ihex $^ $@

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash-%: $(OUTPUT_BINARY_DIRECTORY)/nrf51822_xxac-%.hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf51  --chiperase
	nrfjprog --reset -f nrf51
