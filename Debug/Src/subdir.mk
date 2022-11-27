################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MorseConversionLayer.c \
../Src/freertos.c \
../Src/hts221.c \
../Src/lis3mdl.c \
../Src/lps22hb.c \
../Src/lsm6dsl.c \
../Src/main.c \
../Src/stm32l4s5i_iot01.c \
../Src/stm32l4s5i_iot01_accelero.c \
../Src/stm32l4s5i_iot01_gyro.c \
../Src/stm32l4s5i_iot01_hsensor.c \
../Src/stm32l4s5i_iot01_magneto.c \
../Src/stm32l4s5i_iot01_psensor.c \
../Src/stm32l4s5i_iot01_qspi.c \
../Src/stm32l4s5i_iot01_tsensor.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_hal_timebase_tim.c \
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
./Src/freertos.o \
./Src/hts221.o \
./Src/lis3mdl.o \
./Src/lps22hb.o \
./Src/lsm6dsl.o \
./Src/main.o \
./Src/stm32l4s5i_iot01.o \
./Src/stm32l4s5i_iot01_accelero.o \
./Src/stm32l4s5i_iot01_gyro.o \
./Src/stm32l4s5i_iot01_hsensor.o \
./Src/stm32l4s5i_iot01_magneto.o \
./Src/stm32l4s5i_iot01_psensor.o \
./Src/stm32l4s5i_iot01_qspi.o \
./Src/stm32l4s5i_iot01_tsensor.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_hal_timebase_tim.o \
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
./Src/freertos.d \
./Src/hts221.d \
./Src/lis3mdl.d \
./Src/lps22hb.d \
./Src/lsm6dsl.d \
./Src/main.d \
./Src/stm32l4s5i_iot01.d \
./Src/stm32l4s5i_iot01_accelero.d \
./Src/stm32l4s5i_iot01_gyro.d \
./Src/stm32l4s5i_iot01_hsensor.d \
./Src/stm32l4s5i_iot01_magneto.d \
./Src/stm32l4s5i_iot01_psensor.d \
./Src/stm32l4s5i_iot01_qspi.d \
./Src/stm32l4s5i_iot01_tsensor.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_hal_timebase_tim.d \
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
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/MorseConversionLayer.d ./Src/MorseConversionLayer.o ./Src/MorseConversionLayer.su ./Src/freertos.d ./Src/freertos.o ./Src/freertos.su ./Src/hts221.d ./Src/hts221.o ./Src/hts221.su ./Src/lis3mdl.d ./Src/lis3mdl.o ./Src/lis3mdl.su ./Src/lps22hb.d ./Src/lps22hb.o ./Src/lps22hb.su ./Src/lsm6dsl.d ./Src/lsm6dsl.o ./Src/lsm6dsl.su ./Src/main.d ./Src/main.o ./Src/main.su ./Src/stm32l4s5i_iot01.d ./Src/stm32l4s5i_iot01.o ./Src/stm32l4s5i_iot01.su ./Src/stm32l4s5i_iot01_accelero.d ./Src/stm32l4s5i_iot01_accelero.o ./Src/stm32l4s5i_iot01_accelero.su ./Src/stm32l4s5i_iot01_gyro.d ./Src/stm32l4s5i_iot01_gyro.o ./Src/stm32l4s5i_iot01_gyro.su ./Src/stm32l4s5i_iot01_hsensor.d ./Src/stm32l4s5i_iot01_hsensor.o ./Src/stm32l4s5i_iot01_hsensor.su ./Src/stm32l4s5i_iot01_magneto.d ./Src/stm32l4s5i_iot01_magneto.o ./Src/stm32l4s5i_iot01_magneto.su ./Src/stm32l4s5i_iot01_psensor.d ./Src/stm32l4s5i_iot01_psensor.o ./Src/stm32l4s5i_iot01_psensor.su ./Src/stm32l4s5i_iot01_qspi.d ./Src/stm32l4s5i_iot01_qspi.o ./Src/stm32l4s5i_iot01_qspi.su ./Src/stm32l4s5i_iot01_tsensor.d ./Src/stm32l4s5i_iot01_tsensor.o ./Src/stm32l4s5i_iot01_tsensor.su ./Src/stm32l4xx_hal_msp.d ./Src/stm32l4xx_hal_msp.o ./Src/stm32l4xx_hal_msp.su ./Src/stm32l4xx_hal_timebase_tim.d ./Src/stm32l4xx_hal_timebase_tim.o ./Src/stm32l4xx_hal_timebase_tim.su ./Src/stm32l4xx_it.d ./Src/stm32l4xx_it.o ./Src/stm32l4xx_it.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32l4xx.d ./Src/system_stm32l4xx.o ./Src/system_stm32l4xx.su ./Src/vl53l0x_api.d ./Src/vl53l0x_api.o ./Src/vl53l0x_api.su ./Src/vl53l0x_api_calibration.d ./Src/vl53l0x_api_calibration.o ./Src/vl53l0x_api_calibration.su ./Src/vl53l0x_api_core.d ./Src/vl53l0x_api_core.o ./Src/vl53l0x_api_core.su ./Src/vl53l0x_api_ranging.d ./Src/vl53l0x_api_ranging.o ./Src/vl53l0x_api_ranging.su ./Src/vl53l0x_api_strings.d ./Src/vl53l0x_api_strings.o ./Src/vl53l0x_api_strings.su ./Src/vl53l0x_platform.d ./Src/vl53l0x_platform.o ./Src/vl53l0x_platform.su

.PHONY: clean-Src

