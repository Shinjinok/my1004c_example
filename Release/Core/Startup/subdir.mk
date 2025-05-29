################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l041g6uxs.s 

OBJS += \
./Core/Startup/startup_stm32l041g6uxs.o 

S_DEPS += \
./Core/Startup/startup_stm32l041g6uxs.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -c -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/deca_driver" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/Port" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/instance" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/config" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/utils" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/cmd" -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/mems" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32l041g6uxs.d ./Core/Startup/startup_stm32l041g6uxs.o

.PHONY: clean-Core-2f-Startup

