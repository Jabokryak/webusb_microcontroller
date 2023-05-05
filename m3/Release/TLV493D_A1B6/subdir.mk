################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TLV493D_A1B6/TLV493D_A1B6.c 

OBJS += \
./TLV493D_A1B6/TLV493D_A1B6.o 

C_DEPS += \
./TLV493D_A1B6/TLV493D_A1B6.d 


# Each subdirectory must supply rules for building sources it contributes
TLV493D_A1B6/%.o TLV493D_A1B6/%.su TLV493D_A1B6/%.cyclo: ../TLV493D_A1B6/%.c TLV493D_A1B6/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu17 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -I"D:/MagLight/F4prog/m3/TLV493D_A1B6" -I"D:/MagLight/F4prog/m3/QMC5883L" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TLV493D_A1B6

clean-TLV493D_A1B6:
	-$(RM) ./TLV493D_A1B6/TLV493D_A1B6.cyclo ./TLV493D_A1B6/TLV493D_A1B6.d ./TLV493D_A1B6/TLV493D_A1B6.o ./TLV493D_A1B6/TLV493D_A1B6.su

.PHONY: clean-TLV493D_A1B6

