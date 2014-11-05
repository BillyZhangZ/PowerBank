################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/adc16.c" \
"../Sources/button.c" \
"../Sources/charge.c" \
"../Sources/delay.c" \
"../Sources/led.c" \
"../Sources/llwu.c" \
"../Sources/lptimer.c" \
"../Sources/main.c" \
"../Sources/platform.c" \
"../Sources/sa_mtb.c" \
"../Sources/watchdog.c" \

C_SRCS += \
../Sources/adc16.c \
../Sources/button.c \
../Sources/charge.c \
../Sources/delay.c \
../Sources/led.c \
../Sources/llwu.c \
../Sources/lptimer.c \
../Sources/main.c \
../Sources/platform.c \
../Sources/sa_mtb.c \
../Sources/watchdog.c \

OBJS += \
./Sources/adc16.o \
./Sources/button.o \
./Sources/charge.o \
./Sources/delay.o \
./Sources/led.o \
./Sources/llwu.o \
./Sources/lptimer.o \
./Sources/main.o \
./Sources/platform.o \
./Sources/sa_mtb.o \
./Sources/watchdog.o \

C_DEPS += \
./Sources/adc16.d \
./Sources/button.d \
./Sources/charge.d \
./Sources/delay.d \
./Sources/led.d \
./Sources/llwu.d \
./Sources/lptimer.d \
./Sources/main.d \
./Sources/platform.d \
./Sources/sa_mtb.d \
./Sources/watchdog.d \

OBJS_QUOTED += \
"./Sources/adc16.o" \
"./Sources/button.o" \
"./Sources/charge.o" \
"./Sources/delay.o" \
"./Sources/led.o" \
"./Sources/llwu.o" \
"./Sources/lptimer.o" \
"./Sources/main.o" \
"./Sources/platform.o" \
"./Sources/sa_mtb.o" \
"./Sources/watchdog.o" \

C_DEPS_QUOTED += \
"./Sources/adc16.d" \
"./Sources/button.d" \
"./Sources/charge.d" \
"./Sources/delay.d" \
"./Sources/led.d" \
"./Sources/llwu.d" \
"./Sources/lptimer.d" \
"./Sources/main.d" \
"./Sources/platform.d" \
"./Sources/sa_mtb.d" \
"./Sources/watchdog.d" \

OBJS_OS_FORMAT += \
./Sources/adc16.o \
./Sources/button.o \
./Sources/charge.o \
./Sources/delay.o \
./Sources/led.o \
./Sources/llwu.o \
./Sources/lptimer.o \
./Sources/main.o \
./Sources/platform.o \
./Sources/sa_mtb.o \
./Sources/watchdog.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/adc16.o: ../Sources/adc16.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/adc16.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/adc16.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/button.o: ../Sources/button.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/button.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/button.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/charge.o: ../Sources/charge.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/charge.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/charge.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/delay.o: ../Sources/delay.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/delay.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/delay.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/led.o: ../Sources/led.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/led.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/led.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/llwu.o: ../Sources/llwu.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/llwu.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/llwu.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/lptimer.o: ../Sources/lptimer.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/lptimer.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/lptimer.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/main.o: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/main.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/main.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/platform.o: ../Sources/platform.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/platform.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/platform.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/sa_mtb.o: ../Sources/sa_mtb.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/sa_mtb.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/sa_mtb.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/watchdog.o: ../Sources/watchdog.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/watchdog.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/watchdog.o"
	@echo 'Finished building: $<'
	@echo ' '


