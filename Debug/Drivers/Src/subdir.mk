################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f4xx_gp_timer.c \
../Drivers/Src/stm32f4xx_gpio.c 

OBJS += \
./Drivers/Src/stm32f4xx_gp_timer.o \
./Drivers/Src/stm32f4xx_gpio.o 

C_DEPS += \
./Drivers/Src/stm32f4xx_gp_timer.d \
./Drivers/Src/stm32f4xx_gpio.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/STM32F407_stop_watch_baremetal/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f4xx_gp_timer.cyclo ./Drivers/Src/stm32f4xx_gp_timer.d ./Drivers/Src/stm32f4xx_gp_timer.o ./Drivers/Src/stm32f4xx_gp_timer.su ./Drivers/Src/stm32f4xx_gpio.cyclo ./Drivers/Src/stm32f4xx_gpio.d ./Drivers/Src/stm32f4xx_gpio.o ./Drivers/Src/stm32f4xx_gpio.su

.PHONY: clean-Drivers-2f-Src

