#ifndef CLAI_ECF_DM_DRIVER_HPP_
#define CLAI_ECF_DM_DRIVER_HPP_

/**
 * @file dm_driver.hpp
 * @brief 达妙电机驱动入口。
 *
 * @verbatim
 * // 使用 SPI2CAN 通信对象
 * DmDriver_c spi_motor(spi_can, 0x0E, 0x10E, false, DM4310);
 * spi_motor.Enable();
 *
 * // 使用片内 FDCAN
 * BSP_n::Fdcan_c fdcan(&hfdcan1, 0x0E, 0x10E, FDCAN_STANDARD_ID);
 * DmDriver_c fdcan_motor(fdcan, 0x0E, 0x10E, false, DM4310);
 * fdcan_motor.Enable();
 * @endverbatim
 *
 * @attention 通信对象的生命周期必须长于使用它的 DmDriver_c 对象。
 */

#include "MotorCore/dm_driver_core.hpp"

#endif  // CLAI_ECF_DM_DRIVER_HPP_
