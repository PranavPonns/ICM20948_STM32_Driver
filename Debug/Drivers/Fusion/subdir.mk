################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Fusion/FusionAhrs.c \
../Drivers/Fusion/FusionCompass.c \
../Drivers/Fusion/FusionOffset.c 

C_DEPS += \
./Drivers/Fusion/FusionAhrs.d \
./Drivers/Fusion/FusionCompass.d \
./Drivers/Fusion/FusionOffset.d 

OBJS += \
./Drivers/Fusion/FusionAhrs.o \
./Drivers/Fusion/FusionCompass.o \
./Drivers/Fusion/FusionOffset.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Fusion/%.o Drivers/Fusion/%.su Drivers/Fusion/%.cyclo: ../Drivers/Fusion/%.c Drivers/Fusion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Fusion

clean-Drivers-2f-Fusion:
	-$(RM) ./Drivers/Fusion/FusionAhrs.cyclo ./Drivers/Fusion/FusionAhrs.d ./Drivers/Fusion/FusionAhrs.o ./Drivers/Fusion/FusionAhrs.su ./Drivers/Fusion/FusionCompass.cyclo ./Drivers/Fusion/FusionCompass.d ./Drivers/Fusion/FusionCompass.o ./Drivers/Fusion/FusionCompass.su ./Drivers/Fusion/FusionOffset.cyclo ./Drivers/Fusion/FusionOffset.d ./Drivers/Fusion/FusionOffset.o ./Drivers/Fusion/FusionOffset.su

.PHONY: clean-Drivers-2f-Fusion

