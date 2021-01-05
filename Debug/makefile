################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Flash_HAL_L476RG.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Flash_HAL_L476RG.list \

OBJCOPY_BIN += \
Flash_HAL_L476RG.bin \


# All Target
all: Flash_HAL_L476RG.elf secondary-outputs

# Tool invocations
Flash_HAL_L476RG.elf: $(OBJS) $(USER_OBJS) C:\Users\Souleymane\ BAH\Desktop\5AE-ESPE\Projet\ interdisciplinaire\Software\Flash_HAL_L476RG\STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "Flash_HAL_L476RG.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Souleymane BAH\Desktop\5AE-ESPE\Projet interdisciplinaire\Software\Flash_HAL_L476RG\STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Flash_HAL_L476RG.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Flash_HAL_L476RG.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Flash_HAL_L476RG.list"
	@echo 'Finished building: $@'
	@echo ' '

Flash_HAL_L476RG.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Flash_HAL_L476RG.bin"
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
