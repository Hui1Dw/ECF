#ifndef __BSP_FDCAN_HPP
#define __BSP_FDCAN_HPP

#include <functional>
#include "bsp_can_template.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"

#ifdef __cplusplus
}
#endif

namespace BSP_n
{

//只做继承用
class Fdcan_c:public CanBase_c
{
    public:
        CanBase_c &get_base(){return *this;}
        static Fdcan_c *fdcan_instance_[MX_REGISTER_CNT]; // CAN实例指针数组
        FDCAN_HandleTypeDef *fdcan_handle_;                      // fdcan句柄
        FDCAN_TxHeaderTypeDef txconf_;                            // FDCAN报文发送配置
        // 接收的回调函数,用于解析接收到的数据
    public:
    FdcanRxCallback_t fdcan_module_callback;
    Fdcan_c(
            FDCAN_HandleTypeDef *fdcan_handle,
            uint32_t tx_id,
            uint32_t rx_id,
            uint32_t SAND_IDE
        );
    Fdcan_c(
            FDCAN_HandleTypeDef *fdcan_handle,
            uint32_t tx_id,
            uint32_t rx_id,
            uint32_t SAND_IDE,
            FdcanRxCallback_t fdcan_module_callback
        );
    Fdcan_c(
        uint32_t tx_id,
        uint32_t rx_id);

        Fdcan_c(CanInitConfig_s fdcan_config);
        Fdcan_c(){}
        void MotorInit(FDCAN_HandleTypeDef *fdcan_handle,uint32_t SAND_IDE);
        FDCAN_HandleTypeDef* GetFdcanhandle();
        static void FIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox); // 自己写的FDCAN接收回调函数
        void SetRxCallBack(FdcanRxCallback_t fdcan_module_callback);
        void SetDLC(uint32_t length);
        CanState_e Transmit(float timeout);
        void FilterConfig(Fdcan_c *instance);
        void GetData(uint8_t* data) const;
        //用于扩展帧修改发送ID
        void SetExtTxId(uint32_t ext_id)
        {
            this->tx_id_ = ext_id;
        }
};

}

#endif /* BSP_FDCAN_HPP */