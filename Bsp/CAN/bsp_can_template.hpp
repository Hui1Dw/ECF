/**
 * @file    bsp_can_template.hpp
 * @brief   CAN的BSP层模板文件
 *
 * @verbatim
 *
 * @attention
 *
 */

#ifndef  __BSP_CAN_TEMP_HPP
#define  __BSP_CAN_TEMP_HPP

#define MX_REGISTER_CNT 28u//18u     // 这个数量取决于CAN总线的负载(不能等于这个数)
#define MX_FILTER_CNT (2 * 14)  // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE 2                // 根据板子设定,F407IG有CAN1,CAN2,因此为2;F334只有一个,则设为1

#include <stdint.h>
#include <functional>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32H723xx) || defined(STM32G431xx)
#include "fdcan.h"
#else
#include "can.h"
#endif // USE_H7_if_or_not
#include "ecf_configList.h"

#ifdef __cplusplus
}
#endif

namespace BSP_n {

    typedef enum
    {
        UNDEFINE    = 0,
        DEFINE
     }DefineState_e;

    typedef enum
    {
        CAN_OK,
        CAN_ERROR
    }CanState_e;

    #if defined(STM32H723xx) || defined(STM32G431xx)
        class Fdcan_c;
            typedef std::function<void(BSP_n::Fdcan_c* register_instance)> FdcanRxCallback_t;
            /* FDCAN实例初始化结构体,将此结构体指针传入注册函数 */
            struct CanInitConfig_s
            {
                FDCAN_HandleTypeDef *fdcan_handle;          // fdcan句柄
                uint32_t tx_id;                             // 发送id
                uint32_t rx_id;                             // 接收id
                FdcanRxCallback_t fdcan_module_callback;
                uint32_t SAND_IDE;                          // 标准帧还是拓展帧
            };
    #else
        class Can_c;
            typedef std::function<void(BSP_n::Can_c* register_instance)> CanRxCallback_t;
            /* CAN实例初始化结构体,将此结构体指针传入注册函数 */
            // typedef struct
            struct CanInitConfig_s
            {
                CAN_HandleTypeDef *can_handle;              // can句柄
                uint32_t tx_id;                             // 发送id
                uint32_t rx_id;                             // 接收id
                CanRxCallback_t can_module_callback;
                uint32_t SAND_IDE;                          // 标准帧还是拓展帧
            };
    #endif // USE_H7_if_or_not

            class CanBase_c
            {
            protected:
                static uint8_t idx_;                      // 全局CAN实例索引,每次有新的模块注册会自增
                uint32_t tx_mailbox_;                      // CAN消息填入的邮箱号

            public:
                uint32_t tx_id_;                          // 发送id
                uint32_t rx_id_;                          // 接收id
                uint8_t rx_len_ = 0;                      // 接收长度,可能为0-8
                uint8_t rx_buff_[8];                       // 数据接收
                uint8_t tx_buff_[8];                       // 发送缓存,发送消息长度可以通过ECF_CAN_SetDLC()设定,最大为8
                float tx_wait_time_;                      // block时间
                virtual void SetDLC(uint32_t length) = 0;
                virtual CanState_e Transmit(float timeout) = 0;
                CanBase_c(uint32_t tx_id,uint32_t rx_id):tx_id_ (tx_id), rx_id_(rx_id){}
                CanBase_c(){}
            };
}

#endif /* BSP_CAN_TEMP_HPP */
