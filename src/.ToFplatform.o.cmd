cmd_/home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/ToFplatform.o := arm-none-eabi-gcc -Wp,-MD,/home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/.ToFplatform.o.d     -I/home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/student/projects/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/student/projects/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/student/projects/crazyflie-firmware/vendor/libdw1000/inc   -I/home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/student/projects/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/student/projects/crazyflie-firmware/src/config   -I/home/student/projects/crazyflie-firmware/src/platform/interface   -I/home/student/projects/crazyflie-firmware/src/deck/interface   -I/home/student/projects/crazyflie-firmware/src/deck/drivers/interface   -I/home/student/projects/crazyflie-firmware/src/drivers/interface   -I/home/student/projects/crazyflie-firmware/src/drivers/bosch/interface   -I/home/student/projects/crazyflie-firmware/src/drivers/esp32/interface   -I/home/student/projects/crazyflie-firmware/src/hal/interface   -I/home/student/projects/crazyflie-firmware/src/modules/interface   -I/home/student/projects/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/student/projects/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/student/projects/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/student/projects/crazyflie-firmware/src/modules/interface/cpx   -I/home/student/projects/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/student/projects/crazyflie-firmware/src/modules/interface/controller   -I/home/student/projects/crazyflie-firmware/src/modules/interface/estimator   -I/home/student/projects/crazyflie-firmware/src/utils/interface   -I/home/student/projects/crazyflie-firmware/src/utils/interface/kve   -I/home/student/projects/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/student/projects/crazyflie-firmware/src/utils/interface/tdoa   -I/home/student/projects/crazyflie-firmware/src/lib/FatFS   -I/home/student/projects/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/student/projects/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/student/projects/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/student/projects/crazyflie-firmware/src/lib/vl53l1   -I/home/student/projects/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src -Wno-error   -c -o /home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/ToFplatform.o /home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/ToFplatform.c

source_/home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/ToFplatform.o := /home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/ToFplatform.c

deps_/home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/ToFplatform.o := \
  /home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/ToFplatform.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /usr/include/newlib/string.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/newlib.h \
  /usr/include/newlib/_newlib_version.h \
  /usr/include/newlib/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/include/newlib/machine/ieeefp.h \
  /usr/include/newlib/sys/features.h \
  /usr/include/newlib/sys/reent.h \
  /usr/include/newlib/_ansi.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /usr/include/newlib/sys/_types.h \
  /usr/include/newlib/machine/_types.h \
  /usr/include/newlib/machine/_default_types.h \
  /usr/include/newlib/sys/lock.h \
  /usr/include/newlib/sys/cdefs.h \
  /usr/include/newlib/sys/_locale.h \
  /usr/include/newlib/strings.h \
  /usr/include/newlib/sys/string.h \
  /home/student/projects/crazyflie-firmware/src/drivers/interface/i2cdev.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/student/projects/crazyflie-firmware/src/drivers/interface/i2c_drv.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /home/student/projects/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /home/student/projects/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/student/projects/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /home/student/projects/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/student/projects/crazyflie-firmware/src/config/trace.h \
  /home/student/projects/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/student/projects/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/student/projects/crazyflie-firmware/src/config/stm32fxxx.h \
  /home/student/projects/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /home/student/projects/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /home/student/projects/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_version.h \
  /home/student/projects/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /home/student/projects/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /home/student/projects/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/mpu_armv7.h \
  /home/student/projects/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/system_stm32f4xx.h \
  /home/student/projects/crazyflie-firmware/src/config/stm32f4xx_conf.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_crc.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h \
    $(wildcard include/config/it.h) \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_iwdg.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_wwdg.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_misc.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_cryp.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_hash.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rng.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_can.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dac.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dcmi.h \
  /home/student/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fsmc.h \
  /home/student/projects/crazyflie-firmware/vendor/FreeRTOS/include/task.h \

/home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/ToFplatform.o: $(deps_/home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/ToFplatform.o)

$(deps_/home/student/projects/crazyflie-firmware/examples/quad-tof-crazyflie-example-app/src/ToFplatform.o):
