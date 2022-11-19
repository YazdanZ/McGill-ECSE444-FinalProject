# McGill-ECSE444-FinalProject

## MX configuration for distance sensor
USART1
Asynchronous
PB7
PB6
I2C2
PB10
PB11

## Project setup for distance sensor and morse code conversion layer
Download software from https://www.st.com/en/ecosystems/x-cube-53l0a1.html#st-get-software  

From X-CUBE-53L0A1\STM32CubeExpansion_VL53L0X_V1.2.0\Drivers\BSP\Components\vl53l0x , copy all files to appropriate project directories (either src or inc)

From X-CUBE-53L0A1\STM32CubeExpansion_VL53L0X_V1.2.0\Drivers\BSP\X-NUCLEO-53L0A1 , copy vl53l0x_platform.c and .h to project directories

Add MorseConversionLayer.c and .h to appropriate project folders

Use the provided main.c

Compile. There should be no errors