################################################################################
# Automatically-generated file. Do not edit!
################################################################################

#Progress monitor hint: 14
first : all
-include ../makefile.init

# This file contains definitions of environment variables used in the makefiles and .args files if exist.
-include makefile.local

RM := "$(GNU_Make_Install_DirEnv)/rm" -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include Sources/subdir.mk
-include Project_Settings/Startup_Code/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(SX_DEPS)),)
-include $(SX_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
PowerBank.elf \

EXECUTABLES_QUOTED += \
"PowerBank.elf" \

EXECUTABLES_OS_FORMAT += \
PowerBank.elf \


# All Target
all: warning-messages explicit-dependencies PowerBank.elf secondary-outputs

# Tool invocations
PowerBank.elf: $(OBJS) $(USER_OBJS) ../Project_Settings/Linker_Files/MKL26Z128_flash.ld
	@echo 'Building target: $@'
	@echo 'Executing target #15 $@'
	@echo 'Invoking: ARM Ltd Windows GCC C Linker'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc"  $(USER_OBJS_QUOTED) $(LIBS_QUOTED) @"PowerBank.args" -o"PowerBank.elf"
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(S_DEPS_QUOTED) "./*/*/*.o" "./*/*.o"  $(S_UPPER_DEPS_QUOTED) $(ASM_DEPS_QUOTED) $(ASM_UPPER_DEPS_QUOTED) $(SX_DEPS_QUOTED) $(EXECUTABLES_QUOTED) "./*/*/*.d" "./*/*.d"  $(ProjDirPath)/FLASH/PowerBank.elf
	-@echo ' '

post-build:
	-"C:\Program Files (x86)\CW MCU v10.5\Cross_Tools\arm-none-eabi-gcc-4_7_3\bin\arm-none-eabi-objcopy.exe" -O binary PowerBank.elf  PowerBank.bin
	-@echo ' '

secondary-outputs:

.PHONY: all clean dependents explicit-dependencies warning-messages
.SECONDARY: post-build

-include ../makefile.targets

explicit-dependencies: ../Project_Settings/Linker_Files/MKL26Z128_flash.ld 

warning-messages: 

