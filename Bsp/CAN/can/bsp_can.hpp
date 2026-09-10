#ifndef __BSP_CAN_HPP
#define __BSP_CAN_HPP

#include <functional>
#include "bsp_can_template.hpp"
#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"

#ifdef __cplusplus
}
#endif

namespace BSP_n
{
class Can_c:public CanBase_c
{
    public:
        CanBase_c &get_base(){return *this;}
        Can_c(
            CAN_HandleTypeDef *can_handle,
            uint32_t tx_id,
            uint32_t rx_id,
            uint32_t SAND_IDE,
            CanRxCallback_t can_module_callback
        );
        Can_c(uint32_t tx_id, uint32_t rx_id);

        Can_c(CanInitConfig_s can_config);
        Can_c(){}
        // 外部调用函数
        static void FIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox); // 自己写的CAN接收回调函数
        void SetDLC(uint32_t length);
        CanState_e Transmit(float timeout);
        void SetRxCallBack(CanRxCallback_t callback);
        CAN_HandleTypeDef* GetCanhandle();
        void MotorInit(CAN_HandleTypeDef *can_handle,uint32_t SAND_CanIDE); // 电机发送对象的初始化函数
        uint32_t GetRxId() const { return rx_id_; }
        void GetData(uint8_t* data) const;
        //用于扩展帧修改发送ID
        void SetExtTxId(uint32_t ext_id)
        {
            this->tx_id_ = ext_id;
        }
    protected:
        static Can_c *can_instance_[MX_REGISTER_CNT]; // CAN实例指针数组
        void FilterConfig(Can_c *instance);
        CAN_TxHeaderTypeDef txconf_;               // CAN报文发送配置
        CAN_HandleTypeDef   *can_handle_;           // can句柄
        void* private_data;
        // 接收的回调函数,用于解析接收到的数据
        CanRxCallback_t can_module_callback_; // callback needs an instance to tell among registered ones
};

}
#endif /* BSP_CAN_HPP */