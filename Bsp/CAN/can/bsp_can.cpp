/**
 * @file    bsp_can.cpp
 * @brief   CAN的BSP层文件
 *
 * @verbatim
 *  支持CAN1和CAN2，支持标准帧和扩展帧
 *  使用方法：
 *      在目标文件里面定义一个Can_c对象，并传入can句柄，发送id，接收id，标准帧/拓展帧，回调函数。
 *      然后即可通过该对象调用成员函数。
 *  demo：
 *      // 定义一个Can_c对象
 *      @param hcan1    hcan句柄
 *      @param 0x123         发送id
 *      @param 0x456         接收id
 *      @param CAN_ID_EXT  CAN报文类型 拓展帧
 *      @param dji_motor_callback 回调函数
 *      void dji_motor_callback(Can_c* register_instance)
 *      {
 *
 *      }
 *      // can传输函数，发送数据帧到总线，用于驱动电机/CAN双板/多板通信
 *      @param  5 是邮箱堵塞的等待的最大时间,单位ms
        int16_t set = 1000;
        BSP_CAN_Part_n::Can_c dji_motor2(&hcan1, 0x200, 0x201, CAN_ID_STD, dji_motor_callback);
        dji_motor2.tx_buff_[0] = set >> 8;
        dji_motor2.tx_buff_[1] = set;
        dji_motor2.tx_buff_[2] = set >> 8;
        dji_motor2.tx_buff_[3] = set;
        dji_motor2.tx_buff_[4] = set >> 8;
        dji_motor2.tx_buff_[5] = set;
        dji_motor2.tx_buff_[6] = set >> 8;
        dji_motor2.tx_buff_[7] = set;
        while (1)
        {
            dji_motor2.Transmit(1); // 发送数据
        }
 *      // 修改CAN发送报文的数据帧长度
 *      @param  7        数据长度
 *      dji_motor1.CAN_SetDLC(7); // 设置数据长度
 *
 * @attention
 *      在CANFIFOxCallback函数里面的条件判断(rxconf.IDE = CAN_ID_EXT) 目前拓展帧就用到DM电机，他的Extid是包含消息，是不断变换的。
 *      如果其他地方用到拓展帧，需要Extid完全匹配的（ID不会发生变化），可以考虑使用ID List Mode
 *
 *      目前把CAN过滤器设置为ID Mask Mode,过滤器不过滤，由后面CAN接收中断函数来判断接收的数据是不是要使用
 *      如果想要开启ID List Mode，在CANFilterConfig函数里面把部分代码注释取消，加上两行注释的代码即可，
 *      或者考虑是不是要Can_c加上一个成员变量，来判断使用实例使用 ID Mask Mode  或者  ID List Mode
 */
// 本文件头文件
#include "bsp_can.hpp"

#include "memory.h"
#include "stdlib.h"

#include "bsp_dwt.hpp"
#include <map>
#include <functional>
/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动CAN1和CAN2,开启CAN1和CAN2的FIFO0 & FIFO1溢出通知
 *
 */
static void CANServiceInit()
{
    #ifdef USE_CAN1
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    #endif
    #ifdef USE_CAN2
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);  // 如果测试时，只需要一个CAN的话，在这里把另一个can的初始化注释掉
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);  // F4只有两个CAN  H7 老老实实 用FDCAN文件，兼容传统的CAN和FDCAN的设置
    #endif
}

namespace BSP_n
{
// 定义复合键
struct CANKey {
    CAN_HandleTypeDef* can_handle;
    uint32_t rx_id;

    bool operator<(const CANKey& other) const {
        if (can_handle != other.can_handle) {
            return can_handle < other.can_handle;
        }
        return rx_id < other.rx_id;
    }
};
// 全局分发器（静态实现）
static std::map<CANKey, CanRxCallback_t> can_callbacks_map;
// 初始化CAN实例指针数组
Can_c *Can_c::can_instance_[MX_REGISTER_CNT] = {nullptr};
// 初始化CAN实例指针数组下标
uint8_t CanBase_c::idx_ = 0;
/**
 * @brief Can_c构造函数
 *
 * @param can_handle    hcan句柄
 * @param tx_id         发送id
 * @param rx_id         接收id
 * @param SAND_CanIDE  CAN报文类型，标准帧/拓展帧
 * @param can_module_callback 回调函数
 **/
// 用于调试的CAN构造函数
Can_c::Can_c(
    CAN_HandleTypeDef *can_handle,
    uint32_t tx_id,
    uint32_t rx_id,
    uint32_t SAND_CanIDE,
    CanRxCallback_t can_module_callback)
    : CanBase_c(tx_id, rx_id),// 设置回调函数和接收发送id
      can_handle_(can_handle),
      can_module_callback_(can_module_callback)
{
    // 初始化CAN外设
    if(!idx_)
    {
        CANServiceInit();
    }
    // 进行发送报文的配置
    if (SAND_CanIDE == CAN_ID_STD)
    {
        txconf_.IDE = CAN_ID_STD; // 使用标准id
        txconf_.StdId = tx_id_;    // 发送id
    }
    else
    {
        txconf_.IDE = CAN_ID_EXT; // 使用扩展id
        txconf_.ExtId = tx_id_;    // 发送id
    }
    txconf_.RTR = CAN_RTR_DATA; // 发送数据帧(目前没有远程帧的需求)
    txconf_.DLC = 0x08;         // 默认发送长度为8
    // 将当前实例加入指针数组中
    can_instance_[idx_++] = this;
    if (idx_ >= MX_REGISTER_CNT)
    {
        // 超过最大实例数，错误处理
        while (true)
        {

        }
    }
    FilterConfig(this);     // 添加CAN过滤器规则
    // 注册到分发器
if (can_module_callback_)
{
    CANKey key{can_handle_, rx_id_};
    can_callbacks_map[key] = can_module_callback_;
}
}


Can_c::Can_c(
    uint32_t tx_id,
    uint32_t rx_id)
    : CanBase_c(tx_id, rx_id)
{

}
// 仅用于DJIMotor层的6个CAN发送对象的构造
void Can_c::MotorInit(CAN_HandleTypeDef *can_handle,uint32_t SAND_CanIDE)
{
    this->can_handle_ = can_handle;
    if(!idx_)
    {
        CANServiceInit();
    }
    // 进行发送报文的配置
    if (SAND_CanIDE == CAN_ID_STD)
    {
        txconf_.IDE   = CAN_ID_STD; // 使用标准id
        txconf_.StdId = this->tx_id_;    // 发送id
    }
    else
    {
        txconf_.IDE   = CAN_ID_EXT; // 使用扩展id
        txconf_.ExtId = this->tx_id_;    // 发送id
    }
    txconf_.RTR = CAN_RTR_DATA; // 发送数据帧(目前没有远程帧的需求)
    txconf_.DLC = 0x08;         // 默认发送长度为8
    FilterConfig(this);     // 添加CAN过滤器规则
}

// 上层模块调用的CAN构造函数
Can_c::Can_c(CanInitConfig_s can_config)
    : CanBase_c(can_config.tx_id, can_config.rx_id),// 设置回调函数和接收发送id
      can_handle_(can_config.can_handle),
      can_module_callback_(can_config.can_module_callback)
{
    if(!idx_)
    {
        CANServiceInit();
    }
    // 进行发送报文的配置
    if (can_config.SAND_IDE == CAN_ID_STD)
    {
        txconf_.IDE   = CAN_ID_STD; // 使用标准id
        txconf_.StdId = can_config.tx_id;    // 发送id
    }
    else
    {
        txconf_.IDE = CAN_ID_EXT; // 使用扩展id
        txconf_.ExtId = can_config.tx_id;    // 发送id
    }
    txconf_.RTR = CAN_RTR_DATA; // 发送数据帧(目前没有远程帧的需求)
    txconf_.DLC = 0x08;         // 默认发送长度为8
    // 将当前实例加入指针数组中
    can_instance_[idx_++] = this;
    if (idx_ >= MX_REGISTER_CNT)
    {
        // 超过最大实例数，错误处理
        while (true)
        {

        }
    }
    FilterConfig(this);     // 添加CAN过滤器规则
    // 注册到分发器
    if (can_module_callback_)
    {
        CANKey key{can_handle_, rx_id_};
        can_callbacks_map[key] = can_module_callback_;
    }
}

void Can_c::GetData(uint8_t* data) const
{
    memcpy(data, rx_buff_, rx_len_);
}
void Can_c::SetRxCallBack(CanRxCallback_t callback)
    {
        can_module_callback_ = callback;

        CANKey key{can_handle_, rx_id_};

        // 注册到全局分发器
        if (callback)
        {
            can_callbacks_map[key] = callback;
        }
        else
        {
            can_callbacks_map.erase(key);
        }
    }

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param length    设定长度
 */
void Can_c::SetDLC(uint32_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
    {
        while (1)
        {

        }
    }
    this->txconf_.DLC = length;
}

    /**
     * @brief CAN传输函数，发送数据帧到总线，用于驱动电机
     * @param timeout 设定超时时间，单位ms
     * **/
    CanState_e Can_c::Transmit(float timeout)
    {
        BSP_n::DWT_c *dwt_time_can = BSP_n::DWT_c::Get_DwtInstance();
        float dwt_start                    = dwt_time_can->GetTimeline_ms();
        // dwt_time_can->Delay_ms(timeout);
        this->tx_wait_time_ = dwt_time_can->GetTimeline_ms() - dwt_start;
        // tx_mailbox_会保存实际填入了这一帧消息的邮箱,但是知道是哪个邮箱发的似乎也没啥用
        // 每次发送前同步当前 ID，允许上层协议临时切换报文 ID。
        if (txconf_.IDE == CAN_ID_EXT) {
            this->txconf_.ExtId = this->tx_id_;
        } else {
            this->txconf_.StdId = this->tx_id_;
        }

        uint32_t ret = HAL_CAN_AddTxMessage(can_handle_, &txconf_, tx_buff_, &tx_mailbox_);
        while (ret != HAL_OK) {
            /* Transmission request Error */
            this->tx_wait_time_ = dwt_time_can->GetTimeline_ms() - dwt_start;
            ret = HAL_CAN_AddTxMessage(can_handle_, &txconf_, tx_buff_, &tx_mailbox_);
            if (tx_wait_time_ > timeout)
                return CAN_ERROR;
        }
        return CAN_OK;
    }

CAN_HandleTypeDef* Can_c::GetCanhandle()
{
    return this->can_handle_;
}

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被Can_c的构造函数调用
 *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
 *
 * @note f407的bxCAN有28个过滤器,这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
 *       初始化时,奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
 *       注册到CAN1的模块使用过滤器0-13,CAN2使用过滤器14-27
 *
 * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
 *            详细寄存器含义请查阅芯片官方 reference manual。
 *
 * @param _instance can instance owned by specific module
 */
void Can_c::FilterConfig(Can_c *instance)
{
    CAN_FilterTypeDef can_filter_init_structure;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用
    // 检测关键传参
    assert_param(instance->can_handle != NULL);
    if (instance->txconf_.IDE == CAN_ID_STD || instance->txconf_.IDE == CAN_ID_EXT)
    {
        // 数据帧
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = 0x00;
        // 掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = 0x00;
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = 0x00;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = 0x00;
    }
    else
    {
        // 遥控帧
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = instance->txconf_.StdId;
        // 掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ((instance->txconf_.IDE & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = 0;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = 0;
    }

    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;

    // 滤波器序号, 0-27, 共28个滤波器, can1是0~13, can2是14~27
    #ifdef USE_CAN1
    if (instance->can_handle_ == &hcan1)
    {
        can_filter_init_structure.FilterBank = can1_filter_idx++;
    }
    #endif
    #ifdef USE_CAN2
    if (instance->can_handle_ == &hcan2)
    {
        can_filter_init_structure.FilterBank = can2_filter_idx++;
    }
    #endif
    // 滤波器绑定FIFOx, 只能绑定一个
    can_filter_init_structure.FilterFIFOAssignment = (instance->tx_id_ & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1; // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
    // 使能滤波器
    can_filter_init_structure.FilterActivation = CAN_FILTER_ENABLE;
    // 从机模式选择开始单元
    can_filter_init_structure.SlaveStartFilterBank = 14; // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机

    HAL_CAN_ConfigFilter(instance->can_handle_, &can_filter_init_structure);
}

/* -----------------------belows are callback definitions--------------------------*/

/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
void Can_c::FIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
    {
        static CAN_RxHeaderTypeDef rxconf;
        uint8_t can_rx_data[8];

        while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox))
        {
            HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_data);

            uint32_t received_id = (rxconf.IDE == CAN_ID_STD) ? rxconf.StdId : rxconf.ExtId;

            // 使用复合键查找
            CANKey key{_hcan, received_id};
            auto it = can_callbacks_map.find(key);
            if (it != can_callbacks_map.end() && it->second)
            {
                // 创建临时实例用于回调
                static Can_c temp_instance;
                temp_instance.rx_id_ = received_id;
                temp_instance.rx_len_ = rxconf.DLC;
                memcpy(temp_instance.rx_buff_, can_rx_data, rxconf.DLC);
                temp_instance.can_handle_ = _hcan;

                // 调用回调函数
                it->second(&temp_instance);
            }
        }
    }
}

/**
 * @brief 注意,STM32的两个CAN设备共享两个FIFO
 * 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
 * 当FIFO0或FIFO1溢出时会调用这两个函数
 */
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BSP_n::Can_c::FIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BSP_n::Can_c::FIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}
