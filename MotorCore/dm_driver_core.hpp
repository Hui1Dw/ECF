#ifndef MOTOR_CORE_DM_DRIVER_CORE_HPP_
#define MOTOR_CORE_DM_DRIVER_CORE_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>

#include "CAN/bsp_can_template.hpp"
#include "MotorCore/mit_controller.hpp"
#include "MotorCore/motor_types.hpp"

extern DM_Para_Set dm_para_set[DM_TYPE_COUNT];

class DmDriver_c {
 private:
    uint16_t CAN_ID;
    uint16_t CAN_ID_RX;
    bool Reverse;
    DM_MotorType_e Motor_Type;

    bool self_lock_enabled_ = false;
    float self_lock_pos_rad_ = 0.0f;
    float last_absolute_angle_ = 0.0f;
    bool first_frame_ = true;
    bool motor_init_flag_ = false;
    float last_read_tmax_ = 0.0f;
    uint32_t last_motion_feedback_tick_ms_ = 0U;
    bool has_motion_feedback_ = false;

    DM_MIT_Control_t motor_mit_data_;
    mit_controller::Controller mit_controller_;
    std::function<bool(uint32_t, const uint8_t*, uint8_t)> send_frame_;

    uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    void send_param_frame(uint8_t cmd, uint8_t rid, const uint8_t payload[4]);
    void UpdateTorqueLimit(float t_max);

    std::array<uint8_t, 8> setCtrl(uint8_t cmd);
    void send_MIT_data(float pos, float vel, float kp, float kd, float torq);
    static DM_ErrorCode_e DecodeErrorCode(uint8_t raw_err);

 public:
    DM_Data_Info Motor_Data;
    DM_Para_Set Motor_para_set;
    bool online = false;
    bool enabled = false;
    std::function<void()> online_cb_;

    /**
     * @brief 传入具体通信对象创建达妙电机驱动。
     * @param can 必须提供 SetRxCallBack、SetDLC 和 Transmit 的通信对象。
     * @param can_id 电机命令 CAN ID。
     * @param can_id_rx 电机反馈 CAN ID。
     * @param reverse 是否反转电机方向。
     * @param motor_type 达妙电机型号。
     */
    template <typename CanType>
    DmDriver_c(CanType& can,
               uint16_t can_id,
               uint16_t can_id_rx,
               bool reverse,
               DM_MotorType_e motor_type)
        : CAN_ID(can_id),
          CAN_ID_RX(can_id_rx),
          Reverse(reverse),
          Motor_Type(motor_type),
          mit_controller_({dm_para_set[motor_type].MAX_TOR,
                           dm_para_set[motor_type].MAX_POS,
                           dm_para_set[motor_type].MAX_VEL}) {
        can.SetRxCallBack([this](CanType* instance) {
            this->Analysis(instance->rx_buff_, instance->rx_id_);
        });
        send_frame_ = [&can](uint32_t id, const uint8_t* data, uint8_t length) {
            const uint32_t previous_id = can.tx_id_;
            can.tx_id_ = id;
            can.SetDLC(length);
            std::memcpy(can.tx_buff_, data, length);
            const bool success = can.Transmit(1.0f) == BSP_n::CAN_OK;
            can.tx_id_ = previous_id;
            return success;
        };

        Motor_para_set = dm_para_set[motor_type];
        mit_controller_.setEnable(true);
        last_read_tmax_ = Motor_para_set.MAX_TOR;
        std::memset(&Motor_Data, 0, sizeof(Motor_Data));
        std::memset(&motor_mit_data_, 0, sizeof(motor_mit_data_));
    }

    void Analysis(uint8_t* rx_data, uint32_t rx_id);
    void Control_MIT(float angle, float speed, float kp, float kd, float torque);
    void Control_MIT_Torque(float angle, float speed, float kp, float kd, float torque);
    bool HasFreshMotionFeedback(uint32_t now_ms, uint32_t timeout_ms) const;
    bool HasFreshEnabledFeedback(uint32_t now_ms, uint32_t timeout_ms) const;
    void SendZeroTorque();

    void Enable();
    void Disable();
    void OnlyReceive();
    void ClearErrorFlag();
    void SetZero();
    void SavePosZero();
    void ClearError();

    void SelfLock(float kp, float kd, float torque = 0.0f);
    void CancelSelfLock();
    void SendReadParam(uint8_t rid);
    void SendWriteParamFloat(uint8_t rid, float value);
    void SendStoreParam();

    void SetLimits(float p_max, float v_max, float t_max);
    void SetOnlineCallback(std::function<void()> cb);
    float GetLastReadTmax() const;
};

#endif  // MOTOR_CORE_DM_DRIVER_CORE_HPP_
