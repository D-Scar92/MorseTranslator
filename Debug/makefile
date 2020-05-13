################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Morse_Translator.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Morse_Translator.list \

OBJCOPY_BIN += \
Morse_Translator.bin \


# All Target
all: Morse_Translator.elf secondary-outputs

# Tool invocations
Morse_Translator.elf: $(OBJS) $(USER_OBJS) D:\MEDIA\STM32CubeIDE\workspace_1.0.0\Morse_Translator\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "Morse_Translator.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\MEDIA\STM32CubeIDE\workspace_1.0.0\Morse_Translator\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Morse_Translator.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Morse_Translator.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Morse_Translator.list"
	@echo 'Finished building: $@'
	@echo ' '

Morse_Translator.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Morse_Translator.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
