# CLAI ECF

面向 STM32 HAL 的轻量嵌入式通信与电机驱动模块。

## 目录

- `Bsp/CAN`：CAN/FDCAN BSP 和 `CanBase_c`。
- `Bsp/SPI`：SPI BSP。
- `Bsp/DWT`：DWT 计时 BSP。
- `Config/ecf_configList.h`：板级外设功能开关配置。
- `Spi2Can/mcp2518fd.hpp/.cpp`：继承 `CanBase_c` 的 MCP2518FD 通信对象。
- `MotorCore/dm_driver_core.hpp/.cpp`：达妙电机协议与控制核心。
- `MotorCore/lho6_driver_core.hpp/.cpp`：LHO6 灵巧手协议与控制核心。
- `DMMotor/dm_driver.hpp`：达妙电机驱动入口头文件。
- `LHO6Hand/lho6_driver.hpp`：LHO6 灵巧手驱动入口头文件。
- `docs/lho6-hand-driver.md`：灵巧手驱动接入记录，随步骤追加。

接入具体项目时，先根据 MCU 和实际启用的外设修改`Config/ecf_configList.h`；再在 `CMakeLists.txt` 的 `clai_ecf_bsp` 目标中选择 CAN 或 FDCAN 源文件。
SPI、CAN/FDCAN、DMA、GPIO 和中断对应的 HAL句柄及初始化代码由使用方工程提供。
