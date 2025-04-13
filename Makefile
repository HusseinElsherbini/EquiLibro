# Enhanced Build System for Self-Balancing Robot Project
# This Makefile supports the multi-layered architecture with:
# - Hardware abstraction layer
# - Middleware layer
# - Drivers layer
# - Application layer
# - Supports error-first compilation approach

# Project configuration
PROJ_NAME := sbr
OUTPUT_DIR := out
OBJDIR := $(OUTPUT_DIR)/obj
OUT_BIN := $(OUTPUT_DIR)/bin

# Compiler and tool definitions
CC := arm-none-eabi-gcc
CXX := arm-none-eabi-g++
OBJDMP := arm-none-eabi-objdump
OBJCPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size
MACH := cortex-m4

# Optimization and debug flags
OPT_FLAGS := -O0
DBG_FLAGS := -g3

# Warning levels - separate from core compilation
# No warnings (errors only)
NO_WARN_FLAGS := -w
# Standard warnings
STD_WARN_FLAGS := -Wall
# Extra warnings
EXTRA_WARN_FLAGS := -Wall -Wextra
# Pedantic warnings
PEDANTIC_WARN_FLAGS := -Wall -Wextra -Wpedantic

# Default to no warnings for initial error-fixing phase
WARN_FLAGS := $(NO_WARN_FLAGS)

# Error flags - always active
ERROR_FLAGS := 

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
  -DDEBUG_LEVEL=$(DEBUG_LEVEL) \
  -DDEBUG

# Include paths - expanded to include all project directories
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
  -Iinc/middleware/utils \
  -Iinc/application \
  -Iinc/application/robot_logic \
  -Iinc/application/system_state \
  -Iinc/application/tasks \
  -Iinc/common \
  -Ilib/FreeRTOS/inc \
  -Ilib/SEGGER/inc

# Common compiler flags for C files
COMMON_FLAGS := $(CPU_FLAGS) $(OPT_FLAGS) $(DBG_FLAGS) $(ERROR_FLAGS) $(WARN_FLAGS) $(MACROS) $(INC_PATHS) $(PEDANTIC_WARN_FLAGS)

# Optimization and debug flags
APP_OPT_FLAGS := -O0
FREERTOS_OPT_FLAGS := -O2  # Higher optimization for FreeRTOS

# Language-specific flags
CFLAGS := $(COMMON_FLAGS) $(APP_OPT_FLAGS) -c -std=gnu11 -fdiagnostics-color=never
CXXFLAGS := $(COMMON_FLAGS) $(APP_OPT_FLAGS) -c -std=c++14 -fno-rtti -fno-exceptions -fno-use-cxa-atexit -fdiagnostics-color=never -Wno-psabi

# Assembler flags
ASFLAGS := $(CPU_FLAGS) -c -x assembler-with-cpp $(INC_PATHS)

# Linker flags
LDFLAGS := $(CPU_FLAGS) --specs=nano.specs -T scripts/linker/stm32_ls.ld \
           -Wl,-Map=$(OUTPUT_DIR)/$(PROJ_NAME).map -Wl,--gc-sections 

# Libraries to be linked
LIBS := -lm -lc -lgcc 

# Source directories with expanded structure
HW_ABSTRACTION_DIR := src/hardware_abstraction
DRIVERS_DIR := src/drivers
MIDDLEWARE_DIR := src/middleware
APPLICATION_DIR := src/application
COMMON_DIR := src/common
LIB_DIR := lib
STARTUP_DIR := startup

# Expanded middleware subdirectories
MIDDLEWARE_OS_DIR := $(MIDDLEWARE_DIR)/os
MIDDLEWARE_SYSTEM_SERVICES_DIR := $(MIDDLEWARE_DIR)/system_services
MIDDLEWARE_SIGNAL_PROCESSING_DIR := $(MIDDLEWARE_DIR)/signal_processing
MIDDLEWARE_UTILS_DIR := $(MIDDLEWARE_DIR)/utils

# Expanded application subdirectories
APPLICATION_ROBOT_LOGIC_DIR := $(APPLICATION_DIR)/robot_logic
APPLICATION_SYSTEM_STATE_DIR := $(APPLICATION_DIR)/system_state
APPLICATION_TASKS_DIR := $(APPLICATION_DIR)/tasks

# Expanded drivers subdirectories
DRIVERS_SENSORS_DIR := $(DRIVERS_DIR)/sensors
DRIVERS_ACTUATORS_DIR := $(DRIVERS_DIR)/actuators
DRIVERS_COMMUNICATION_DIR := $(DRIVERS_DIR)/communication

# Source files by layer - now expanded to include all subdirectories
HW_ABSTRACTION_SRCS := $(wildcard $(HW_ABSTRACTION_DIR)/*.c) $(wildcard $(HW_ABSTRACTION_DIR)/*.cpp)
DRIVERS_SRCS := $(wildcard $(DRIVERS_DIR)/*.c) $(wildcard $(DRIVERS_DIR)/*.cpp) \
                $(wildcard $(DRIVERS_SENSORS_DIR)/*.c) $(wildcard $(DRIVERS_SENSORS_DIR)/*.cpp) \
                $(wildcard $(DRIVERS_ACTUATORS_DIR)/*.c) $(wildcard $(DRIVERS_ACTUATORS_DIR)/*.cpp) \
                $(wildcard $(DRIVERS_COMMUNICATION_DIR)/*.c) $(wildcard $(DRIVERS_COMMUNICATION_DIR)/*.cpp)
MIDDLEWARE_SRCS := $(wildcard $(MIDDLEWARE_DIR)/*.c) $(wildcard $(MIDDLEWARE_DIR)/*.cpp) \
                   $(wildcard $(MIDDLEWARE_OS_DIR)/*.c) $(wildcard $(MIDDLEWARE_OS_DIR)/*.cpp) \
                   $(wildcard $(MIDDLEWARE_SYSTEM_SERVICES_DIR)/*.c) $(wildcard $(MIDDLEWARE_SYSTEM_SERVICES_DIR)/*.cpp) \
                   $(wildcard $(MIDDLEWARE_SIGNAL_PROCESSING_DIR)/*.c) $(wildcard $(MIDDLEWARE_SIGNAL_PROCESSING_DIR)/*.cpp) \
                   $(wildcard $(MIDDLEWARE_UTILS_DIR)/*.c) $(wildcard $(MIDDLEWARE_UTILS_DIR)/*.cpp)
APPLICATION_SRCS := $(wildcard $(APPLICATION_DIR)/*.c) $(wildcard $(APPLICATION_DIR)/*.cpp) \
                    $(wildcard $(APPLICATION_ROBOT_LOGIC_DIR)/*.c) $(wildcard $(APPLICATION_ROBOT_LOGIC_DIR)/*.cpp) \
                    $(wildcard $(APPLICATION_SYSTEM_STATE_DIR)/*.c) $(wildcard $(APPLICATION_SYSTEM_STATE_DIR)/*.cpp) \
                    $(wildcard $(APPLICATION_TASKS_DIR)/*.c) $(wildcard $(APPLICATION_TASKS_DIR)/*.cpp)
COMMON_SRCS := $(wildcard $(COMMON_DIR)/*.c) $(wildcard $(COMMON_DIR)/*.cpp)
FREERTOS_SRCS := $(wildcard $(LIB_DIR)/FreeRTOS/src/*.c)
SEGGER_SRCS := $(wildcard $(LIB_DIR)/SEGGER/src/*.c)
STARTUP_SRCS := $(wildcard $(STARTUP_DIR)/*.c) $(wildcard $(STARTUP_DIR)/*.cpp)

# Main source file
MAIN_SRC := main.c

# Combine all source files
C_SRCS := $(filter %.c, \
  $(HW_ABSTRACTION_SRCS) \
  $(DRIVERS_SRCS) \
  $(MIDDLEWARE_SRCS) \
  $(APPLICATION_SRCS) \
  $(COMMON_SRCS) \
  $(FREERTOS_SRCS) \
  $(SEGGER_SRCS) \
  $(STARTUP_SRCS) \
  $(MAIN_SRC))

CPP_SRCS := $(filter %.cpp, \
  $(HW_ABSTRACTION_SRCS) \
  $(DRIVERS_SRCS) \
  $(MIDDLEWARE_SRCS) \
  $(APPLICATION_SRCS) \
  $(COMMON_SRCS) \
  $(STARTUP_SRCS))

# Assembly source files
ASM_SRCS := $(wildcard $(LIB_DIR)/SEGGER/src/*.S) $(wildcard $(STARTUP_DIR)/*.S)

# Generate object file names
C_OBJS := $(C_SRCS:%.c=$(OBJDIR)/%.o)
CPP_OBJS := $(CPP_SRCS:%.cpp=$(OBJDIR)/%.o)
ASM_OBJS := $(ASM_SRCS:%.S=$(OBJDIR)/%.o)
ALL_OBJS := $(C_OBJS) $(CPP_OBJS) $(ASM_OBJS)

# Directory creation function
define make-dir
	@mkdir -p $(@D)
endef

# Build targets
.PHONY: all clean debug size flash erase help warnings warnings-std warnings-extra warnings-pedantic errors-only

# Default target - only errors, no warnings
all: $(OUT_BIN)/$(PROJ_NAME).elf $(OUT_BIN)/$(PROJ_NAME).hex $(OUT_BIN)/$(PROJ_NAME).bin
	@echo "Build completed successfully with errors-only mode"
	@echo "Use 'make warnings' to check for warnings after fixing errors"

# Create directories
directories:
	@mkdir -p $(OBJDIR) $(OBJDIR)/src $(OBJDIR)/lib $(OBJDIR)/startup
	@mkdir -p $(OBJDIR)/src/hardware_abstraction
	@mkdir -p $(OBJDIR)/src/drivers $(OBJDIR)/src/drivers/sensors $(OBJDIR)/src/drivers/actuators $(OBJDIR)/src/drivers/communication
	@mkdir -p $(OBJDIR)/src/middleware $(OBJDIR)/src/middleware/os $(OBJDIR)/src/middleware/system_services
	@mkdir -p $(OBJDIR)/src/middleware/signal_processing $(OBJDIR)/src/middleware/utils 
	@mkdir -p $(OBJDIR)/src/application $(OBJDIR)/src/application/robot_logic
	@mkdir -p $(OBJDIR)/src/application/system_state $(OBJDIR)/src/application/tasks
	@mkdir -p $(OBJDIR)/src/common
	@mkdir -p $(OUT_BIN)

# Rules for compilation

# Rules for compilation with different optimization levels
$(OBJDIR)/lib/FreeRTOS/src/%.o: $(LIB_DIR)/FreeRTOS/src/%.c | directories
	$(make-dir)
	@echo "Compiling FreeRTOS $< with optimization"
	@$(CC) $(CPU_FLAGS) $(FREERTOS_OPT_FLAGS) $(DBG_FLAGS) $(ERROR_FLAGS) $(WARN_FLAGS) $(MACROS) $(INC_PATHS) -c $< -o $@

# Regular C files (non-FreeRTOS)
$(OBJDIR)/%.o: %.c | directories
	$(make-dir)
	@echo "Compiling $<"
	@$(CC) $(CPU_FLAGS) $(APP_OPT_FLAGS) $(DBG_FLAGS) $(ERROR_FLAGS) $(WARN_FLAGS) $(MACROS) $(INC_PATHS) -c $< -o $@
	
$(OBJDIR)/%.o: %.cpp | directories
	$(make-dir)
	@echo "Compiling $<"
	@$(CXX) $(CXXFLAGS) $< -o $@

$(OBJDIR)/%.o: %.S | directories
	$(make-dir)
	@echo "Assembling $<"
	@$(CC) $(ASFLAGS) $< -o $@

# Link everything together
$(OUT_BIN)/$(PROJ_NAME).elf: $(ALL_OBJS)
	$(make-dir)
	@echo "Linking $@"
	@$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)
	@$(OBJDMP) -D $@ > $(OUTPUT_DIR)/$(PROJ_NAME).txt
	@$(SIZE) $@

# Generate bin file
$(OUT_BIN)/$(PROJ_NAME).bin: $(OUT_BIN)/$(PROJ_NAME).elf
	@echo "Generating binary $@"
	@$(OBJCPY) -O binary $< $@

# Generate hex file
$(OUT_BIN)/$(PROJ_NAME).hex: $(OUT_BIN)/$(PROJ_NAME).elf
	@echo "Generating hex $@"
	@$(OBJCPY) -O ihex $< $@

# Show binary size
size: $(OUT_BIN)/$(PROJ_NAME).elf
	@echo "Size of binary:"
	@$(SIZE) $<

# Clean target
clean:
	@echo "Cleaning project"
	@rm -rf $(OUTPUT_DIR)

# Debug build - with additional debugging flags
debug: CFLAGS += -DDEBUG -g3 -O0
debug: CXXFLAGS += -DDEBUG -g3 -O0
debug: all

# Warning level targets
errors-only: WARN_FLAGS := $(NO_WARN_FLAGS)
errors-only: clean all

warnings-std: WARN_FLAGS := $(STD_WARN_FLAGS)
warnings-std: clean all
	@echo "Build completed with standard warnings enabled (-Wall)"

warnings-extra: WARN_FLAGS := $(EXTRA_WARN_FLAGS)
warnings-extra: clean all
	@echo "Build completed with extra warnings enabled (-Wall -Wextra)"

warnings-pedantic: WARN_FLAGS := $(PEDANTIC_WARN_FLAGS)
warnings-pedantic: clean all
	@echo "Build completed with pedantic warnings enabled (-Wall -Wextra -Wpedantic)"

# Default warnings target - uses extra warnings
warnings: warnings-extra

# Make errors fatal - use this after fixing all errors
errors-as-warnings: ERROR_FLAGS := -Werror
errors-as-warnings: warnings-extra
	@echo "Build completed with errors treated as warnings"

# Flash the board using OpenOCD
flash: $(OUT_BIN)/$(PROJ_NAME).bin
	@echo "Flashing board..."
	@openocd -f board/st_nucleo_f4.cfg -c "program $< 0x08000000 verify reset exit"

# Erase flash memory
erase:
	@echo "Erasing flash memory..."
	@openocd -f board/st_nucleo_f4.cfg -c "init; halt; stm32f4x mass_erase 0; exit"

# Help documentation
help:
	@echo "=== Self-Balancing Robot Build System ==="
	@echo "Available targets:"
	@echo ""
	@echo "  all              - Build with errors-only mode (default)"
	@echo "  errors-only      - Build with all warnings disabled (same as default)"
	@echo "  warnings         - Build with standard extra warnings enabled"
	@echo "  warnings-std     - Build with standard warnings (-Wall)"
	@echo "  warnings-extra   - Build with extra warnings (-Wall -Wextra)"
	@echo "  warnings-pedantic - Build with pedantic warnings (-Wall -Wextra -Wpedantic)"
	@echo "  errors-as-warnings - Treat all warnings as errors"
	@echo ""
	@echo "  clean            - Clean build artifacts"
	@echo "  debug            - Build with additional debug flags"
	@echo "  size             - Show binary size"
	@echo "  flash            - Flash the binary to the board"
	@echo "  erase            - Erase the flash memory"
	@echo "  help             - Show this help message"
	@echo ""
	@echo "Usage example:"
	@echo "  1. make              # Build with errors-only first"
	@echo "  2. make warnings     # Check for warnings after fixing errors"
	@echo ""
	@echo "=== Other Useful Commands ==="
	@echo "  make print-C_SRCS    # List all C source files"
	@echo "  make print-CPP_SRCS  # List all C++ source files"

# Dependency tracking
-include $(ALL_OBJS:.o=.d)

# Debugging helpers
print-%:
	@echo $* = $($*)

print-sources:
	@echo "C sources: $(C_SRCS)"
	@echo "C++ sources: $(CPP_SRCS)"
	@echo "ASM sources: $(ASM_SRCS)"

ELF_FILE=$(find out/bin -name "*.elf" | sort | tail -n1)

analyze-size: $(OUT_BIN)/$(PROJ_NAME).elf
	@echo "===== Largest Functions ====="
	@arm-none-eabi-nm --print-size --size-sort --radix=d $< | grep -v " [a-z] " | tail -n 50
	@echo "\n===== Section Sizes ====="
	@arm-none-eabi-size -A -d $(OUT_BIN)/$(PROJ_NAME).elf

analyze-components:
	@echo "===== Component Size Analysis ====="
	@echo "Hardware Abstraction Layer:"
	@find $(OBJDIR)/src/hardware_abstraction -name "*.o" -exec $(SIZE) {} \; | \
		awk '{text += $$1; data += $$2; bss += $$3} END {printf "  Text: %d bytes, Data: %d bytes, BSS: %d bytes\n", text, data, bss}'
	@echo "Drivers:"
	@find $(OBJDIR)/src/drivers -name "*.o" -exec $(SIZE) {} \; | \
		awk '{text += $$1; data += $$2; bss += $$3} END {printf "  Text: %d bytes, Data: %d bytes, BSS: %d bytes\n", text, data, bss}'
	@echo "Middleware:"
	@find $(OBJDIR)/src/middleware -name "*.o" -exec $(SIZE) {} \; | \
		awk '{text += $$1; data += $$2; bss += $$3} END {printf "  Text: %d bytes, Data: %d bytes, BSS: %d bytes\n", text, data, bss}'
	@echo "Application:"
	@find $(OBJDIR)/src/application -name "*.o" -exec $(SIZE) {} \; | \
		awk '{text += $$1; data += $$2; bss += $$3} END {printf "  Text: %d bytes, Data: %d bytes, BSS: %d bytes\n", text, data, bss}'
	@echo "FreeRTOS:"
	@find $(OBJDIR)/lib/FreeRTOS -name "*.o" -exec $(SIZE) {} \; | \
		awk '{text += $$1; data += $$2; bss += $$3} END {printf "  Text: %d bytes, Data: %d bytes, BSS: %d bytes\n", text, data, bss}'
	@echo "SEGGER:"
	@find $(OBJDIR)/lib/SEGGER -name "*.o" -exec $(SIZE) {} \; | \
		awk '{text += $$1; data += $$2; bss += $$3} END {printf "  Text: %d bytes, Data: %d bytes, BSS: %d bytes\n", text, data, bss}'

# Detailed analysis by file
analyze-files:
	@echo "===== File Size Analysis ====="
	@find $(OBJDIR) -name "*.o" -exec sh -c '$(SIZE) {} | tail -n 1 | awk "{print \$$1+\$$2 \" bytes: {}\"}"' \; | sort -nr | head -n 30

# Simple analysis for largest object files
largest-objects:
	@echo "===== Largest Object Files ====="
	@find $(OBJDIR) -name "*.o" -type f -exec du -h {} \; | sort -hr | head -n 20

# Generate a map file and analyze it
map-analysis: $(OUT_BIN)/$(PROJ_NAME).elf
	@echo "===== Map File Analysis ====="
	@awk '/^\.text/ {text += $$3} /^\.data/ {data += $$3} /^\.bss/ {bss += $$3} END {printf "Text: %d bytes, Data: %d bytes, BSS: %d bytes\n", text, data, bss}' $(OUTPUT_DIR)/$(PROJ_NAME).map

# Function count analysis
function-count:
	@echo "===== Function Count by Component ====="
	@echo "Hardware Abstraction Layer: `arm-none-eabi-nm $(OUT_BIN)/$(PROJ_NAME).elf | grep -c 'hardware_abstraction'`"
	@echo "Drivers: `arm-none-eabi-nm $(OUT_BIN)/$(PROJ_NAME).elf | grep -c 'drivers'`"
	@echo "Middleware: `arm-none-eabi-nm $(OUT_BIN)/$(PROJ_NAME).elf | grep -c 'middleware'`"
	@echo "Application: `arm-none-eabi-nm $(OUT_BIN)/$(PROJ_NAME).elf | grep -c 'application'`"
	@echo "FreeRTOS: `arm-none-eabi-nm $(OUT_BIN)/$(PROJ_NAME).elf | grep -c 'FreeRTOS'`"
	@echo "SEGGER: `arm-none-eabi-nm $(OUT_BIN)/$(PROJ_NAME).elf | grep -c 'SEGGER'`"