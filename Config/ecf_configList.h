#ifndef __ECF_CONFIGLIST_H
#define __ECF_CONFIGLIST_H

/*
 * 当前板级配置：STM32G431CBT6 只启用 FDCAN1。
 * 其他开发板可参照 ecf_configList.h.example 扩展对应外设宏。
 */
#if defined(STM32G431xx)
#define USE_FDCAN1
#elif defined(STM32H723xx)
#define USE_FDCAN1
#define USE_FDCAN2
#define USE_FDCAN3
#elif defined(STM32F405xx)
#define USE_CAN1
#define USE_CAN2
#endif

#endif
