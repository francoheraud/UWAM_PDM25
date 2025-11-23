################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/can_driver.c \
../App/Src/lin_driver.c \
../App/Src/pdm_logic.c 

OBJS += \
./App/Src/can_driver.o \
./App/Src/lin_driver.o \
./App/Src/pdm_logic.o 

C_DEPS += \
./App/Src/can_driver.d \
./App/Src/lin_driver.d \
./App/Src/pdm_logic.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Franc/STM32CubeIDE/workspace_1.17.0/PDM_Firmware/App" -I"C:/Users/Franc/STM32CubeIDE/workspace_1.17.0/PDM_Firmware/App/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/can_driver.cyclo ./App/Src/can_driver.d ./App/Src/can_driver.o ./App/Src/can_driver.su ./App/Src/lin_driver.cyclo ./App/Src/lin_driver.d ./App/Src/lin_driver.o ./App/Src/lin_driver.su ./App/Src/pdm_logic.cyclo ./App/Src/pdm_logic.d ./App/Src/pdm_logic.o ./App/Src/pdm_logic.su

.PHONY: clean-App-2f-Src

