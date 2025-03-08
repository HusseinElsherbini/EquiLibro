# Project configuration
PROJ_NAME := sbr
OUTPUT_DIR := out
OBJDIR := $(OUTPUT_DIR)/obj
OUT_BIN := $(OUTPUT_DIR)/bin

# Compiler and tool definitions
CC := arm-none-eabi-gcc
OBJDMP := arm-none-eabi-objdump
MACH := cortex-m4

# MCU-specific flags (STM32F4 with FPU)
CPU_FLAGS := -mcpu=$(MACH) -mfloat-abi=hard -mthumb -mfpu=fpv4-sp-d16

# Debug configuration flags
PRINT_IMU_DATA := 0
USE_TASK_ANALYSIS := 0
DEBUG_LEVEL := 3

# Macro definitions passed to compiler
MACROS := \
  -DPRINT_IMU_DATA=$(PRINT_IMU_DATA) \
  -DUSE_TASK_ANALYSIS=$(USE_TASK_ANALYSIS) \
  -DDEBUG_LEVEL=$(DEBUG_LEVEL)

# Include paths
INC_PATHS := \
  -Iinc \
  -Iinc/hardware_abstraction \
  -Iinc/drivers \
  -Iinc/drivers/sensors \
  -Iinc/drivers/actuators \
  -Iinc/drivers/communication \
  -Iinc/middleware \
  -Iinc/middleware/os \
  -Iinc/middleware/signal_processing \
  -Iinc/middleware/system_services \
  -Iinc/application \
  -Iinc/application/robot_logic \
  -Iinc/application/system_state \
  -Iinc/application/tasks \
  -Iinc/common \
  -Ilib/FreeRTOS/inc \
  -Ilib/SEGGER/inc \

# Compiler flags for C files
CFLAGS := $(CPU_FLAGS) -c $(MACROS) $(INC_PATHS) -std=gnu11 -Wall -O0 -g3 -DDEBUG -w

# Assembler flags
ASFLAGS := $(CPU_FLAGS) -c -x assembler-with-cpp $(INC_PATHS)

# Linker flags
LDFLAGS := $(CPU_FLAGS) --specs=nano.specs -T scripts/linker/stm32_ls.ld -Wl,-Map=$(OUTPUT_DIR)/$(PROJ_NAME).map

# Source directories
HW_ABSTRACTION_DIR := src/hardware_abstraction
DRIVERS_DIR := src/drivers
MIDDLEWARE_DIR := src/middleware
APPLICATION_DIR := src/application
COMMON_DIR := src/common
LIB_DIR := lib
STARTUP_DIR := startup

# Source files by layer
HW_ABSTRACTION_SRCS := $(wildcard $(HW_ABSTRACTION_DIR)/*.c)
DRIVERS_SRCS := $(wildcard $(DRIVERS_DIR)/*/*.c) $(wildcard $(DRIVERS_DIR)/*.c)
MIDDLEWARE_SRCS := $(wildcard $(MIDDLEWARE_DIR)/*/*.c) $(wildcard $(MIDDLEWARE_DIR)/*.c)
APPLICATION_SRCS := $(wildcard $(APPLICATION_DIR)/*/*.c) $(wildcard $(APPLICATION_DIR)/*.c)
COMMON_SRCS := $(wildcard $(COMMON_DIR)/*.c)
FREERTOS_SRCS := $(wildcard $(LIB_DIR)/FreeRTOS/src/*.c)
SEGGER_SRCS := $(wildcard $(LIB_DIR)/SEGGER/src/*.c)
STARTUP_SRCS := $(wildcard $(STARTUP_DIR)/*.c)

# Main source file
MAIN_SRC := main.c

# Combine all C source files
C_SRCS := \
  $(HW_ABSTRACTION_SRCS) \
  $(DRIVERS_SRCS) \
  $(MIDDLEWARE_SRCS) \
  $(APPLICATION_SRCS) \
  $(COMMON_SRCS) \
  $(FREERTOS_SRCS) \
  $(SEGGER_SRCS) \
  $(STARTUP_SRCS) \
  $(MAIN_SRC)

# Assembly source files
ASM_SRCS := $(wildcard $(LIB_DIR)/SEGGER/src/*.s)

# Generate object file names
C_OBJS := $(C_SRCS:%.c=$(OBJDIR)/%.o)
ASM_OBJS := $(ASM_SRCS:%.S=$(OBJDIR)/%.o)
ALL_OBJS := $(C_OBJS) $(ASM_OBJS)

# Directory creation function
define make-dir
	@mkdir -p $(@D)
endef

# Build targets
.PHONY: all clean

all: $(OUT_BIN)/$(PROJ_NAME).elf

# Rules for compilation
$(OBJDIR)/%.o: %.c
	$(make-dir)
	$(CC) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: %.S
	$(make-dir)
	$(CC) $(ASFLAGS) $< -o $@

# Link everything together
$(OUT_BIN)/$(PROJ_NAME).elf: $(ALL_OBJS)
	$(make-dir)
	$(CC) $(LDFLAGS) -o $@ $^ -lm
	$(OBJDMP) -D $@ > $(OUTPUT_DIR)/$(PROJ_NAME).txt

# Clean target
clean:
	rm -rf $(OUTPUT_DIR)

# Debugging helpers
print-%:
	@echo $* = $($*)

print-all-sources:
  @echo "All sources: $(C_SRCS)


  