# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = emptyProject_gd32f303


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Src/main.c \
Src/gd32f30x_it.c \
Src/systick.c \
Drivers/CMSIS/GD/GD32F30x/Source/system_gd32f30x.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_adc.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_bkp.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_can.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_crc.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_ctc.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_dac.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_dbg.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_dma.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_enet.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_exmc.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_exti.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_fmc.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_fwdgt.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_gpio.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_i2c.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_misc.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_pmu.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_rcu.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_rtc.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_sdio.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_spi.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_timer.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_usart.c \
Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_wwdgt.c 

# ASM sources
ASM_SOURCES =  \
startup_gd32f30x_hd.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DHSE_VALUE=8000000 \
-DHSE_STARTUP_TIMEOUT=100 \
-DLSE_STARTUP_TIMEOUT=5000 \
-DLSE_VALUE=32768 \
-DEXTERNAL_CLOCK_VALUE=8000000 \
-DHSI_VALUE=8000000 \
-DLSI_VALUE=40000 \
-DVDD_VALUE=3300 \
-DPREFETCH_ENABLE=1 \
-DGD32F30X_HD


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/CMSIS \
-IDrivers/CMSIS/GD/GD32F30x/Include \
-IDrivers/GD32F30x_standard_peripheral/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = GD32F303CC_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -Wl,--print-memory-usage

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***