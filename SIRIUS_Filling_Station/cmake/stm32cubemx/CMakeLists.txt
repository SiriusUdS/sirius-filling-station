cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32F411xE
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../Drivers/STM32F4xx_HAL_Driver/Inc
    ../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ../../FATFS/App
    ../../FATFS/Target
    ../../Middlewares/Third_Party/FatFs/src
    ../../USB_DEVICE/App
    ../../USB_DEVICE/Target
    ../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/sirius-embedded-common/Src/Sensor/TemperatureSensor/TemperatureSensor.c
    ../../Core/sirius-embedded-common/Src/Sensor/TemperatureSensor/NTC3950.c
    ../../Core/sirius-embedded-common/Src/Sensor/PressureSensor/PressureSensor.c
    ../../Core/sirius-embedded-common/Src/Sensor/PressureSensor/ETM375.c
    ../../Core/sirius-embedded-common/Src/Device/Valve/Valve.c
    ../../Core/sirius-embedded-common/Src/Device/Valve/HBL388.c
    ../../Core/sirius-embedded-common/Src/Device/Storage/Storage.c
    ../../Core/sirius-embedded-common/Src/Device/Storage/SDCard.c
    ../../Core/sirius-embedded-common/Src/Device/Telecommunication/Telecommunication.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/ADC/ADC12Channel.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/ADC/ADC12ChannelHAL.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/ADC/ADC12.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/ADC/ADC12HAL.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/GPIO/GPIO.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/GPIO/GPIOHAL.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/UART/UART.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/UART/UARTHAL.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/PWM/PWM.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/PWM/PWMHAL.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/USB/USB.c
    ../../Core/sirius-embedded-common/Src/LowLevelDriver/USB/USBHAL.c
    ../../Core/Src/Engine.c
    ../../Core/Src/main.c
    ../../FATFS/App/fatfs.c
    ../../FATFS/Target/sd_diskio.c
    ../../FATFS/Target/bsp_driver_sd.c
    ../../USB_DEVICE/App/usb_device.c
    ../../USB_DEVICE/App/usbd_cdc_if.c
    ../../USB_DEVICE/App/usbd_desc.c
    ../../USB_DEVICE/Target/usbd_conf.c
    ../../Middlewares/Third_Party/FatFs/src/ff.c
    ../../Middlewares/Third_Party/FatFs/src/diskio.c
    ../../Middlewares/Third_Party/FatFs/src/option/ccsbcs.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ../../Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
    ../../Core/Src/stm32f4xx_it.c
    ../../Core/Src/stm32f4xx_hal_msp.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hcd.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    ../../Core/Src/system_stm32f4xx.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32f411xe.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


