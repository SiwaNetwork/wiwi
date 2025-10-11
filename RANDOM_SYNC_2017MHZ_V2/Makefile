# Makefile for RANDOM_SYNC_2017MHZ_V2 project

# Project configuration
PROJECT_NAME = RANDOM_SYNC_2017MHZ_V2
VERSION = 1.0.0
TARGET = stm32h7

# Toolchain
CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
LD = arm-none-eabi-ld
AR = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

# Directories
SRC_DIR = .
BUILD_DIR = build
FIRMWARE_DIR = firmware
HARDWARE_DIR = hardware
SOFTWARE_DIR = software

# Include paths
INCLUDES = -I$(FIRMWARE_DIR)/stm32h7 \
           -I$(FIRMWARE_DIR)/fpga \
           -I$(SOFTWARE_DIR)/ptp_stack \
           -I$(HARDWARE_DIR)/schematics

# Source files
C_SOURCES = $(wildcard $(FIRMWARE_DIR)/stm32h7/*.c) \
            $(wildcard $(FIRMWARE_DIR)/fpga/*.c) \
            $(wildcard $(SOFTWARE_DIR)/ptp_stack/*.c)

CXX_SOURCES = $(wildcard $(FIRMWARE_DIR)/stm32h7/*.cpp) \
              $(wildcard $(FIRMWARE_DIR)/fpga/*.cpp) \
              $(wildcard $(SOFTWARE_DIR)/ptp_stack/*.cpp)

ASM_SOURCES = $(wildcard $(FIRMWARE_DIR)/stm32h7/*.s)

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/, $(notdir $(CXX_SOURCES:.cpp=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/, $(notdir $(ASM_SOURCES:.s=.o)))

# Compiler flags
CFLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
         -DUSE_HAL_DRIVER -DSTM32H747xx -DCORE_CM7 \
         $(INCLUDES) -Wall -Wextra -Wpedantic -g -O2

CXXFLAGS = $(CFLAGS) -std=c++17 -fno-exceptions -fno-rtti

LDFLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
          -TSTM32H747XIHx_FLASH.ld -lc -lm -lnosys

# Targets
.PHONY: all clean flash debug help

all: $(BUILD_DIR)/$(PROJECT_NAME).elf $(BUILD_DIR)/$(PROJECT_NAME).hex $(BUILD_DIR)/$(PROJECT_NAME).bin

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile C files
$(BUILD_DIR)/%.o: $(FIRMWARE_DIR)/stm32h7/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(FIRMWARE_DIR)/fpga/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(SOFTWARE_DIR)/ptp_stack/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

# Compile C++ files
$(BUILD_DIR)/%.o: $(FIRMWARE_DIR)/stm32h7/%.cpp | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(FIRMWARE_DIR)/fpga/%.cpp | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(SOFTWARE_DIR)/ptp_stack/%.cpp | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) $< -o $@

# Link
$(BUILD_DIR)/$(PROJECT_NAME).elf: $(OBJECTS)
	$(CXX) $(LDFLAGS) $^ -o $@
	$(SIZE) $@

# Generate hex file
$(BUILD_DIR)/$(PROJECT_NAME).hex: $(BUILD_DIR)/$(PROJECT_NAME).elf
	$(OBJCOPY) -O ihex $< $@

# Generate bin file
$(BUILD_DIR)/$(PROJECT_NAME).bin: $(BUILD_DIR)/$(PROJECT_NAME).elf
	$(OBJCOPY) -O binary $< $@

# Flash target
flash: $(BUILD_DIR)/$(PROJECT_NAME).bin
	st-flash write $< 0x8000000

# Debug target
debug: $(BUILD_DIR)/$(PROJECT_NAME).elf
	arm-none-eabi-gdb $< -ex "target remote localhost:3333"

# Clean target
clean:
	rm -rf $(BUILD_DIR)
	rm -rf *.hex *.bin *.elf *.map

# Help target
help:
	@echo "Available targets:"
	@echo "  all       - Build the project"
	@echo "  clean     - Clean build files"
	@echo "  flash     - Flash the binary to the target"
	@echo "  debug     - Start debugging with GDB"
	@echo "  help      - Show this help message"

# Documentation targets
docs: docs/html docs/latex

docs/html:
	doxygen Doxyfile

docs/latex:
	doxygen -w latex header.tex footer.tex doxygen.sty Doxyfile

# Test targets
test: test-unit test-integration

test-unit:
	@echo "Running unit tests..."
	# Add unit test commands here

test-integration:
	@echo "Running integration tests..."
	# Add integration test commands here

# Package target
package: all docs
	@echo "Creating release package..."
	tar -czf $(PROJECT_NAME)_v$(VERSION).tar.gz \
		$(BUILD_DIR) \
		docs/html \
		README.md \
		$(HARDWARE_DIR) \
		$(FIRMWARE_DIR) \
		$(SOFTWARE_DIR)

# Install dependencies (example)
install-deps:
	@echo "Installing dependencies..."
	# Add dependency installation commands here

# Default target
.DEFAULT_GOAL := all

# Print build information
info:
	@echo "Project: $(PROJECT_NAME)"
	@echo "Version: $(VERSION)"
	@echo "Target: $(TARGET)"
	@echo "Build directory: $(BUILD_DIR)"
	@echo "Source files: $(words $(C_SOURCES) $(CXX_SOURCES) $(ASM_SOURCES))"
	@echo "Object files: $(words $(OBJECTS))"
