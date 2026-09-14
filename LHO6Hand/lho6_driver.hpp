#ifndef CLAI_ECF_LHO6_DRIVER_HPP_
#define CLAI_ECF_LHO6_DRIVER_HPP_

/**
 * @file lho6_driver.hpp
 * @brief LHO6 灵巧手驱动入口。
 *
 * @verbatim
 * // 使用片内 FDCAN。LHO6 收发共用同一个标准帧 ID。
 * BSP_n::Fdcan_c left_can(&hfdcan1, LHO6_LEFT_CAN_ID, LHO6_LEFT_CAN_ID, FDCAN_STANDARD_ID);
 * Lho6Hand_c left(left_can, LHO6_LEFT_CAN_ID, Lho6HandSide_e::LEFT);
 * left.SetPosition(target);
 * left.ReadSpeed();
 *
 * // 使用 SPI2CAN 通信对象
 * Lho6Hand_c spi_hand(spi_can, LHO6_RIGHT_CAN_ID, Lho6HandSide_e::RIGHT);
 * @endverbatim
 *
 * @attention 通信对象的生命周期必须长于使用它的 Lho6Hand_c 对象。
 */

#include "MotorCore/lho6_driver_core.hpp"

#endif  // CLAI_ECF_LHO6_DRIVER_HPP_
