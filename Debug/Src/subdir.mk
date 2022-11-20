################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MorseConversionLayer.c \
../Src/main.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32l4xx.c \
../Src/vl53l0x_api.c \
../Src/vl53l0x_api_calibration.c \
../Src/vl53l0x_api_core.c \
../Src/vl53l0x_api_ranging.c \
../Src/vl53l0x_api_strings.c \
../Src/vl53l0x_platform.c 

OBJS += \
./Src/MorseConversionLayer.o \
./Src/main.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32l4xx.o \
./Src/vl53l0x_api.o \
./Src/vl53l0x_api_calibration.o \
./Src/vl53l0x_api_core.o \
./Src/vl53l0x_api_ranging.o \
./Src/vl53l0x_api_strings.o \
./Src/vl53l0x_platform.o 

C_DEPS += \
./Src/MorseConversionLayer.d \
./Src/main.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32l4xx.d \
./Src/vl53l0x_api.d \
./Src/vl53l0x_api_calibration.d \
./Src/vl53l0x_api_core.d \
./Src/vl53l0x_api_ranging.d \
./Src/vl53l0x_api_strings.d \
./Src/vl53l0x_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/MorseConversionLayer.d ./Src/MorseConversionLayer.o ./Src/MorseConversionLayer.su ./Src/main.d ./Src/main.o ./Src/main.su ./Src/stm32l4xx_hal_msp.d ./Src/stm32l4xx_hal_msp.o ./Src/stm32l4xx_hal_msp.su ./Src/stm32l4xx_it.d ./Src/stm32l4xx_it.o ./Src/stm32l4xx_it.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32l4xx.d ./Src/system_stm32l4xx.o ./Src/system_stm32l4xx.su ./Src/vl53l0x_api.d ./Src/vl53l0x_api.o ./Src/vl53l0x_api.su ./Src/vl53l0x_api_calibration.d ./Src/vl53l0x_api_calibration.o ./Src/vl53l0x_api_calibration.su ./Src/vl53l0x_api_core.d ./Src/vl53l0x_api_core.o ./Src/vl53l0x_api_core.su ./Src/vl53l0x_api_ranging.d ./Src/vl53l0x_api_ranging.o ./Src/vl53l0x_api_ranging.su ./Src/vl53l0x_api_strings.d ./Src/vl53l0x_api_strings.o ./Src/vl53l0x_api_strings.su ./Src/vl53l0x_platform.d ./Src/vl53l0x_platform.o ./Src/vl53l0x_platform.su

.PHONY: clean-Src

