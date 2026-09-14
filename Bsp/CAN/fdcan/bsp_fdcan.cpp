/**
 * @file    bsp_fdcan.cpp
 * @brief   FDCAN的BSP层文件
 *
 * @verbatim
 *
 * @attention
 * FDCAN发生BUS-OFF时，发送函数会尝试退出初始化状态以恢复通信。
 */
#include "bsp_fdcan.hpp"
#include "memory.h"
#include "stdlib.h"

#include "bsp_dwt.hpp"
#include <map>

static void FDCANServiceInit()
{
    #ifdef USE_FDCAN1
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // 启动中断
    // HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);// 启动错误中断
    #endif
    #ifdef USE_FDCAN2
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // 启动中断
    // HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);// 启动错误中断
    #endif
    #ifdef USE_FDCAN3
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // 启动中断
    // HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0);// 启动错误中断
    #endif
}

namespace BSP_n
{
// 定义FDCAN复合键
struct FDCANKey {
    FDCAN_HandleTypeDef* fdcan_handle;
    uint32_t rx_id;

    bool operator<(const FDCANKey& other) const {
        if (fdcan_handle != other.fdcan_handle) {
            return fdcan_handle < other.fdcan_handle;
        }
        return rx_id < other.rx_id;
    }
};
    // 全局分发器（静态实现）
    static std::map<FDCANKey, FdcanRxCallback_t> fdcan_callbacks_map;

    Fdcan_c *Fdcan_c::fdcan_instance_[MX_REGISTER_CNT] = {nullptr};
    uint8_t CanBase_c::idx_                                     = 0;

    // FDCAN结构体方式实例化构造函数
    Fdcan_c::Fdcan_c(CanInitConfig_s fdcan_config)
                : CanBase_c(fdcan_config.tx_id, fdcan_config.rx_id),// 设置回调函数和接收发送id
                  fdcan_handle_(fdcan_config.fdcan_handle),
                  fdcan_module_callback(fdcan_config.fdcan_module_callback)
    {
        if(!idx_)
        {
            FDCANServiceInit();
        }
        if (fdcan_config.SAND_IDE == FDCAN_STANDARD_ID) // 标准帧
        {
            txconf_.IdType     = FDCAN_STANDARD_ID;
            txconf_.Identifier = this->tx_id_;
        } else // 拓展帧
        {
            txconf_.IdType     = FDCAN_EXTENDED_ID;
            txconf_.Identifier = this->tx_id_;
        }
        txconf_.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
        txconf_.DataLength          = FDCAN_DLC_BYTES_8;  // 8位DLC
        txconf_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 指定错误状态指示器（发送节点错误活跃
        txconf_.BitRateSwitch       = FDCAN_BRS_OFF;      // 指定发送的T恤帧是带位率转换还是不带
        txconf_.FDFormat            = FDCAN_CLASSIC_CAN;  // 指定发送帧是classic 还是fd
        txconf_.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 指定帧开始时捕获的时间戳计数器值传播（不存储tx事件
        txconf_.MessageMarker       = 0;                  // 指定复制到Tx EventFIFO元素中的消息标记用于识别Tx信息状态
        fdcan_instance_[idx_++]    = this;
        if (idx_ >= MX_REGISTER_CNT)
        {
            while (1)
            {
                // 卡在这就是负载太多了
            }
        }
        FilterConfig(this);

        // 注册到分发器
        if (fdcan_module_callback)
        {
            FDCANKey key{fdcan_handle_, rx_id_};
            fdcan_callbacks_map[key] = fdcan_module_callback;
        }
    }

    /// @brief FDCAN实例化构造函数(继承后可以用这个)
    /// @param fdcan_handle fdcan句柄
    /// @param tx_id        发送id
    /// @param rx_id        接收id|电机id
    /// @param SAND_IDE     帧类型
    /// @param fdcan_module_callback 对反馈值进行处理的回调函数指针
    Fdcan_c::Fdcan_c(FDCAN_HandleTypeDef *fdcan_handle,
                                     uint32_t tx_id,
                                     uint32_t rx_id,
                                     uint32_t SAND_IDE)
        : CanBase_c(tx_id, rx_id),
          fdcan_handle_(fdcan_handle)

    {
        if(!idx_)
        {
            FDCANServiceInit();
        }
        if (SAND_IDE == FDCAN_STANDARD_ID) // 标准帧
        {
            txconf_.IdType     = FDCAN_STANDARD_ID;
            txconf_.Identifier = tx_id_;
        } else // 拓展帧
        {
            txconf_.IdType     = FDCAN_EXTENDED_ID;
            txconf_.Identifier = tx_id_;
        }
        txconf_.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
        txconf_.DataLength          = FDCAN_DLC_BYTES_8;  // 8位DLC
        txconf_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 指定错误状态指示器（发送节点错误活跃
        txconf_.BitRateSwitch       = FDCAN_BRS_OFF;      // 指定发送的T恤帧是带位率转换还是不带
        txconf_.FDFormat            = FDCAN_CLASSIC_CAN;  // 指定发送帧是classic 还是fd
        txconf_.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 指定帧开始时捕获的时间戳计数器值传播（不存储tx事件
        txconf_.MessageMarker       = 0;                  // 指定复制到Tx EventFIFO元素中的消息标记用于识别Tx信息状态
        fdcan_instance_[idx_++]    = this;

        FilterConfig(this);
    }

    /// @brief 若直接调用调试可以用这个
    /// @param fdcan_handle
    /// @param tx_id
    /// @param rx_id
    /// @param SAND_IDE
    /// @param fdcan_module_callback
    Fdcan_c::Fdcan_c(
        FDCAN_HandleTypeDef *fdcan_handle,
        uint32_t tx_id,
        uint32_t rx_id,
        uint32_t SAND_IDE,
        FdcanRxCallback_t fdcan_module_callback)
        : CanBase_c(tx_id, rx_id),
          fdcan_handle_(fdcan_handle),
          fdcan_module_callback(fdcan_module_callback)
    {
        // 初始化CAN外设
        if(!idx_)
        {
            FDCANServiceInit();
        }
        if (SAND_IDE == FDCAN_STANDARD_ID) // 标准帧
        {
            txconf_.IdType     = FDCAN_STANDARD_ID;
            txconf_.Identifier = tx_id_;
        } else // 拓展帧
        {
            txconf_.IdType     = FDCAN_EXTENDED_ID;
            txconf_.Identifier = tx_id_;
        }
        txconf_.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
        txconf_.DataLength          = FDCAN_DLC_BYTES_8;  // 8位DLC
        txconf_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 指定错误状态指示器（发送节点错误活跃
        txconf_.BitRateSwitch       = FDCAN_BRS_OFF;      // 指定发送的T恤帧是带位率转换还是不带
        txconf_.FDFormat            = FDCAN_CLASSIC_CAN;  // 指定发送帧是classic 还是fd
        txconf_.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 指定帧开始时捕获的时间戳计数器值传播（不存储tx事件
        txconf_.MessageMarker       = 0;                  // 指定复制到Tx EventFIFO元素中的消息标记用于识别Tx信息状态
        fdcan_instance_[idx_++]    = this;

        if (idx_ >= MX_REGISTER_CNT) {
            while (1)
            {
                // 卡在这就是负载太多了
            }
        }
        FilterConfig(this);

        // 注册到分发器
        if (fdcan_module_callback)
        {
            FDCANKey key{fdcan_handle_, rx_id_};
            fdcan_callbacks_map[key] = fdcan_module_callback;
        }
    }

    /// @brief 继承后用可以将回调函数编写好后，调用此函数进行回调
    /// @param fdcan_module_callback
    void Fdcan_c::SetRxCallBack(FdcanRxCallback_t fdcan_module_callback)
    {
        this->fdcan_module_callback = fdcan_module_callback;

        FDCANKey key{fdcan_handle_, rx_id_};

        // 注册到全局分发器
        if (fdcan_module_callback)
        {
            fdcan_callbacks_map[key] = fdcan_module_callback;
        }
        else
        {
            fdcan_callbacks_map.erase(key);
        }
    }

    Fdcan_c::Fdcan_c(
        uint32_t tx_id,
        uint32_t rx_id)
        : CanBase_c(tx_id, rx_id)// 设置回调函数和接收发送id
    {

    }

    // 仅用于DJIMotor层的9个FDCAN发送对象的构造
    void Fdcan_c::MotorInit(FDCAN_HandleTypeDef *fdcan_handle,uint32_t SAND_IDE)
    {
        this->fdcan_handle_ = fdcan_handle;
        if(!idx_)
        {
            FDCANServiceInit();
        }
        if (SAND_IDE == FDCAN_STANDARD_ID) // 标准帧
        {
            txconf_.IdType     = FDCAN_STANDARD_ID;
            txconf_.Identifier = this->tx_id_;
        } else // 拓展帧
        {
            txconf_.IdType     = FDCAN_EXTENDED_ID;
            txconf_.Identifier = this->tx_id_;
        }
        txconf_.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
        txconf_.DataLength          = FDCAN_DLC_BYTES_8;  // 8位DLC
        txconf_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 指定错误状态指示器（发送节点错误活跃
        txconf_.BitRateSwitch       = FDCAN_BRS_OFF;      // 指定发送的T恤帧是带位率转换还是不带
        txconf_.FDFormat            = FDCAN_CLASSIC_CAN;  // 指定发送帧是classic 还是fd
        txconf_.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 指定帧开始时捕获的时间戳计数器值传播（不存储tx事件
        txconf_.MessageMarker       = 0;                  // 指定复制到Tx EventFIFO元素中的消息标记用于识别Tx信息状态
        FilterConfig(this);
    }

    void Fdcan_c::GetData(uint8_t* data) const
    {
        memcpy(data, rx_buff_, rx_len_);
    }
    /// @brief 修改DLC
    /// @param length 数据长度
    void Fdcan_c::SetDLC(uint32_t length)
    {
        // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
        if (length > 8 || length == 0) // 安全检查
        {
            while (1)
            {
            }
        }
        static constexpr uint32_t dlc_table[8] = {
            FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
            FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4,
            FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6,
            FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};
        txconf_.DataLength = dlc_table[length - 1U];
    }
    FDCAN_HandleTypeDef* Fdcan_c::GetFdcanhandle()
    {
        return this->fdcan_handle_;
    }
    /// @brief FDCAN发送函数
    /// @param timeout 延时时间
    /// @return 返回发送状态
    CanState_e Fdcan_c::Transmit(float timeout)
    {
        this->txconf_.Identifier = this->tx_id_;
        // 在每次发送数据前通过读取PSR寄存器中的BO位来判断是否触发BUSOFF,如果触发则写入CCCR寄存器中的INIT位来恢复
        if(this->fdcan_handle_->Instance->PSR & FDCAN_PSR_BO_Msk)
        {
            this->fdcan_handle_->Instance->CCCR &= ~FDCAN_CCCR_INIT;
            return CAN_ERROR;
        }
        BSP_n::DWT_c *dwt_time_fdcan = BSP_n::DWT_c::Get_DwtInstance(); // 获取DWT实例

        static volatile float wait_time __attribute__((unused)); // for cancel warning
        float dwt_start     = dwt_time_fdcan->GetTimeline_ms();

        while (HAL_FDCAN_GetTxFifoFreeLevel(this->fdcan_handle_) == 0) // 等待邮箱空闲
        {
            if (dwt_time_fdcan->GetTimeline_ms() - dwt_start > timeout) // 超时
            {
                return CAN_ERROR;
            }
        }
        if (HAL_FDCAN_AddMessageToTxFifoQ(fdcan_handle_, &txconf_, tx_buff_))
        {
            return CAN_ERROR;
        }
         return CAN_OK; // 发送成功
    }

    /// @brief 过滤器设置
    /// @param instance 实例指针
    void Fdcan_c::FilterConfig(Fdcan_c *instance)
    {
        FDCAN_FilterTypeDef filter;
        assert_param(instance->fdcan_handle_ != NULL);
        filter.IdType           = instance->txconf_.IdType; // 32位工作
        filter.FilterType       = FDCAN_FILTER_RANGE;      // 范围过滤
        filter.FilterIndex      = 0;                       // bank
        filter.FilterID1        = 0;                       // 传统模式
        filter.FilterID2        = 0;
        #ifdef STM32H723xx
        filter.RxBufferIndex    = 0x0000;
        filter.IsCalibrationMsg = 0;
        #endif
        filter.FilterConfig     = FDCAN_FILTER_TO_RXFIFO0;
        if (HAL_FDCAN_ConfigFilter(instance->fdcan_handle_, &filter) != HAL_OK) {
            Error_Handler();
        }
    }

    /// @brief 把 FDCAN DLC 编码转成经典 CAN 的真实字节数。
    /// @param dlc HAL 的 DataLength / FDCAN_DLC_BYTES_x。
    /// @return 0..8；超过 8 字节的 FD 帧返回 0，避免按编码值 memcpy。
    static uint8_t ClassicCanLengthFromDlc(uint32_t dlc)
    {
        switch (dlc)
        {
            case FDCAN_DLC_BYTES_0: return 0;
            case FDCAN_DLC_BYTES_1: return 1;
            case FDCAN_DLC_BYTES_2: return 2;
            case FDCAN_DLC_BYTES_3: return 3;
            case FDCAN_DLC_BYTES_4: return 4;
            case FDCAN_DLC_BYTES_5: return 5;
            case FDCAN_DLC_BYTES_6: return 6;
            case FDCAN_DLC_BYTES_7: return 7;
            case FDCAN_DLC_BYTES_8: return 8;
            default: return 0;
        }
    }

    /// @brief 回调函数，在接收之后回调解析数据
    /// @param _hfdcan fdcan句柄
    /// @param fifox   哪个fifo
    void Fdcan_c::FIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
    {
        static FDCAN_RxHeaderTypeDef rxconf; // 数据身份信息
        uint8_t fdcan_rx_data[8];

        while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
        {
            HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, fdcan_rx_data); // 从FIFO中获取数据

            uint32_t received_id = rxconf.Identifier;

            // 使用分发器机制
            FDCANKey key{_hfdcan, received_id};
            auto it = fdcan_callbacks_map.find(key);
            if (it != fdcan_callbacks_map.end() && it->second)
            {
                // 创建临时实例用于回调
                static Fdcan_c temp_instance;
                const uint8_t rx_len = ClassicCanLengthFromDlc(rxconf.DataLength);
                temp_instance.rx_id_ = received_id;
                temp_instance.rx_len_ = rx_len;
                memset(temp_instance.rx_buff_, 0, sizeof(temp_instance.rx_buff_));
                memcpy(temp_instance.rx_buff_, fdcan_rx_data, rx_len);
                temp_instance.fdcan_handle_ = _hfdcan;

                // 调用回调函数
                it->second(&temp_instance);
            }
        }
    }
}

// BUSOFF后重启（阻塞中断）
// void CAN_bus_off_check_reset(FDCAN_HandleTypeDef *hfdcan)
// {
//     FDCAN_ProtocolStatusTypeDef protocolStatus = {0};
//     HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
//     if (protocolStatus.BusOff) {
//         // 进入初始化模式
//         SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
//         uint32_t startTick = HAL_GetTick();
//         // 等待进入初始化模式
//         while ((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) == 0) {
//             if ((HAL_GetTick() - startTick) > 10) {
//                 // 超时，退出避免死循环
//                 return;
//             }
//         }
//         // 此时硬件已进入初始化状态，Bus-Off 状态被复位
//         // 退出初始化模式，重新开始正常通信
//         CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
//         startTick = HAL_GetTick();
//         // 等待退出初始化模式完成
//         while ((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0) {
//             if ((HAL_GetTick() - startTick) > 10) {
//                 return;
//             }
//         }
//     }
// }

/**
  * @brief  Error status callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  ErrorStatusITs indicates which Error Status interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Error_Status_Interrupts.
  * @retval None
  */
// void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
//     if ((ErrorStatusITs & FDCAN_IT_BUS_OFF))
//         hfdcan->Instance->CCCR &= ~FDCAN_CCCR_INIT; // 适用于电机掉线导致的长时间、可预测busoff
// }

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    BSP_n::Fdcan_c::FIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    BSP_n::Fdcan_c::FIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}
