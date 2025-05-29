################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../mems/lis3dh.c 

OBJS += \
./mems/lis3dh.o 

C_DEPS += \
./mems/lis3dh.d 


# Each subdirectory must supply rules for building sources it contributes
mems/%.o mems/%.su mems/%.cyclo: ../mems/%.c mems/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_FULL_LL_DRIVER -DSTM32L041xx -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DMSI_VALUE=131072 -DHSI_VALUE=16000000 -DLSI_VALUE=37000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=0 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/deca_driver" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/Port" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/instance" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/config" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/utils" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/cmd" -I"/home/sjo/STM32CubeIDE/workspace_1.18.0/my1004c/mems" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-mems

clean-mems:
	-$(RM) ./mems/lis3dh.cyclo ./mems/lis3dh.d ./mems/lis3dh.o ./mems/lis3dh.su

.PHONY: clean-mems

