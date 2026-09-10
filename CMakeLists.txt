add_library(clai_ecf STATIC
    Bsp/SPI/bsp_spi.cpp
    MotorCore/dm_driver_core.cpp
    MotorCore/mit_controller.cpp
    Spi2Can/mcp2518fd.cpp
)

target_include_directories(clai_ecf PUBLIC
    .
    Bsp
    Bsp/CAN
    Bsp/CAN/can
    Bsp/CAN/fdcan
    Bsp/DWT
    Bsp/SPI
    Config
    DMMotor
    MotorCore
    Spi2Can
)

target_link_libraries(clai_ecf PUBLIC stm32cubemx)

# 独立 BSP 目标；G431 只构建 FDCAN 分支。
add_library(clai_ecf_bsp STATIC
    Bsp/CAN/fdcan/bsp_fdcan.cpp
    Bsp/DWT/bsp_dwt.cpp
    Bsp/SPI/bsp_spi.cpp
)

target_include_directories(clai_ecf_bsp PUBLIC
    Bsp/CAN
    Bsp/CAN/fdcan
    Bsp/DWT
    Bsp/SPI
    Config
)

target_link_libraries(clai_ecf_bsp PUBLIC stm32cubemx)
