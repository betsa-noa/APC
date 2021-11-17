################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ringbuff/src/ringbuff/ringbuff.c 

OBJS += \
./Middlewares/ringbuff/src/ringbuff/ringbuff.o 

C_DEPS += \
./Middlewares/ringbuff/src/ringbuff/ringbuff.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ringbuff/src/ringbuff/%.o: ../Middlewares/ringbuff/src/ringbuff/%.c Middlewares/ringbuff/src/ringbuff/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/Users/TecMonterrey/Documents/STM32CubeSistEmbAvReto/ProyectoFinal/Common/inc" -I"/Users/TecMonterrey/Documents/STM32CubeSistEmbAvReto/ProyectoFinal/CM4/Middlewares/ringbuff/src/include/ringbuff" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

