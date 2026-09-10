#include "MotorCore/dm_driver_core.hpp"

#include <cstdio>
#include <cstring>

DM_Para_Set dm_para_set[DM_TYPE_COUNT] = {
    {500.0f, 5.0f, 50.0f, 12.5f, 10.0f, DM4310_RATIO},
    {500.0f, 5.0f, 45.0f, 12.5f, 40.0f, DM4340_RATIO},
    {500.0f, 5.0f, 45.0f, 12.5f, 10.0f, DM6220_RATIO},
    {500.0f, 5.0f, 45.0f, 12.5f, 54.0f, DM8006_RATIO},
    {500.0f, 5.0f, 45.0f, 12.5f, 54.0f, DM8009_RATIO},
    {500.0f, 5.0f, 20.0f, 12.5f, 200.0f, DM10010_RATIO},
    {500.0f, 5.0f, 25.0f, 12.5f, 200.0f, DM10010L_RATIO},
    {500.0f, 5.0f, 45.0f, 12.5f, 5.0f, DM2325_RATIO},
    {500.0f, 5.0f, 45.0f, 12.5f, 40.0f, DM3519_RATIO},
};

uint16_t DmDriver_c::float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) {
        x = x_max;
    } else if (x < x_min) {
        x = x_min;
    }
    return static_cast<uint16_t>((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float DmDriver_c::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void DmDriver_c::send_param_frame(uint8_t cmd, uint8_t rid, const uint8_t payload[4]) {
    uint8_t txdata[8] = {0};
    txdata[0] = static_cast<uint8_t>(CAN_ID & 0xFF);
    txdata[1] = static_cast<uint8_t>((CAN_ID >> 8) & 0xFF);
    txdata[2] = cmd;
    txdata[3] = rid;
    txdata[4] = payload[0];
    txdata[5] = payload[1];
    txdata[6] = payload[2];
    txdata[7] = payload[3];

    send_frame_(DM_PARAM_FRAME_ID, txdata, sizeof(txdata));
}

void DmDriver_c::UpdateTorqueLimit(float t_max) {
    Motor_para_set.MAX_TOR = t_max;
    last_read_tmax_ = t_max;
    mit_controller_.setLimit({t_max, Motor_para_set.MAX_POS, Motor_para_set.MAX_VEL});
}

std::array<uint8_t, 8> DmDriver_c::setCtrl(uint8_t cmd) {
    return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
}

DM_ErrorCode_e DmDriver_c::DecodeErrorCode(uint8_t raw_err) {
    switch (raw_err) {
        case 0x0:
            return DM_ERR_DISABLED;
        case 0x1:
            return DM_ERR_ENABLED;
        case 0x8:
            return DM_ERR_OVERVOLTAGE;
        case 0x9:
            return DM_ERR_UNDERVOLTAGE;
        case 0xA:
            return DM_ERR_OVERCURRENT;
        case 0xB:
            return DM_ERR_MOS_OVERTEMP;
        case 0xC:
            return DM_ERR_COIL_OVERTEMP;
        case 0xD:
            return DM_ERR_COMM_LOSS;
        case 0xE:
            return DM_ERR_OVERLOAD;
        default:
            return DM_ERR_UNKNOWN;
    }
}

uint32_t rx_id_test_dm = 0;
void DmDriver_c::Analysis(uint8_t* rx_data, uint32_t rx_id) {
    rx_id_test_dm = rx_id;
    if (rx_id == DM_PARAM_FRAME_ID) {
        uint16_t payload_can_id = static_cast<uint16_t>(rx_data[0]) | (static_cast<uint16_t>(rx_data[1]) << 8);
        if (payload_can_id == CAN_ID &&
            (rx_data[2] == DM_PARAM_READ || rx_data[2] == DM_PARAM_WRITE) &&
            rx_data[3] == DM_PARAM_RID_TMAX) {
            float tmax = 0.0f;
            memcpy(&tmax, &rx_data[4], sizeof(float));
            UpdateTorqueLimit(tmax);
        }
        return;
    }

    if (rx_id != CAN_ID_RX) {
        return;
    }

    uint8_t payload_motor_id = rx_data[0] & 0x0F;
    if (payload_motor_id != (CAN_ID & 0x0F)) {
        return;
    }

    uint8_t error_code_raw = (rx_data[0] >> 4) & 0x0F;

    int p_int = (rx_data[1] << 8) | rx_data[2];
    int v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    int t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

    float pos = uint_to_float(p_int, -this->Motor_para_set.MAX_POS, this->Motor_para_set.MAX_POS, 16);
    float vel = uint_to_float(v_int, -this->Motor_para_set.MAX_VEL, this->Motor_para_set.MAX_VEL, 12);
    float torq = uint_to_float(t_int, -this->Motor_para_set.MAX_TOR, this->Motor_para_set.MAX_TOR, 12);

    if (Reverse) {
        pos = -pos;
        vel = -vel;
        torq = -torq;
    }

    Motor_Data.Position = pos;
    Motor_Data.Velocity = vel;
    Motor_Data.Torque = torq;
    Motor_Data.T_MOS = static_cast<float>(static_cast<int8_t>(rx_data[6]));
    Motor_Data.T_Rotor = static_cast<float>(static_cast<int8_t>(rx_data[7]));
    Motor_Data.Error_Code = DecodeErrorCode(error_code_raw);
    Motor_Data.Current = torq / (Motor_para_set.MAX_TOR / 30.0f);

    last_motion_feedback_tick_ms_ = HAL_GetTick();
    has_motion_feedback_ = true;

    online = true;
    if (online_cb_) {
        online_cb_();
    }
}

void DmDriver_c::Control_MIT(float angle, float speed, float kp, float kd, float torque) {
    if (Reverse) {
        angle = -angle;
        speed = -speed;
        torque = -torque;
    }
    motor_mit_data_.pos = angle;
    motor_mit_data_.vel = speed;
    motor_mit_data_.kp = kp;
    motor_mit_data_.kd = kd;
    motor_mit_data_.torq = torque;
    send_MIT_data(angle, speed, kp, kd, torque);
}

void DmDriver_c::Control_MIT_Torque(float angle,
                                    float speed,
                                    float kp,
                                    float kd,
                                    float torque) {
    mit_controller_.setCommand({angle, speed, kp, kd, torque});
    mit_controller_.setFeedback({Motor_Data.Position, Motor_Data.Velocity});
    mit_controller_.update();
    const float motor_torque = Reverse ? -mit_controller_.output.tau_cmd
                                       : mit_controller_.output.tau_cmd;
    send_MIT_data(0.0f, 0.0f, 0.0f, 0.0f, motor_torque);
}

bool DmDriver_c::HasFreshMotionFeedback(uint32_t now_ms, uint32_t timeout_ms) const {
    return has_motion_feedback_ &&
           (now_ms - last_motion_feedback_tick_ms_ <= timeout_ms);
}

bool DmDriver_c::HasFreshEnabledFeedback(uint32_t now_ms, uint32_t timeout_ms) const {
    return HasFreshMotionFeedback(now_ms, timeout_ms) &&
           Motor_Data.Error_Code == DM_ERR_ENABLED;
}

void DmDriver_c::SendZeroTorque() {
    send_MIT_data(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void DmDriver_c::send_MIT_data(float pos, float vel, float kp, float kd, float torq) {
    uint16_t pos_tmp = float_to_uint(pos, -this->Motor_para_set.MAX_POS, this->Motor_para_set.MAX_POS, 16);
    uint16_t spd_tmp = float_to_uint(vel, -this->Motor_para_set.MAX_VEL, this->Motor_para_set.MAX_VEL, 12);
    uint16_t kp_tmp = float_to_uint(kp, 0, this->Motor_para_set.KP, 12);
    uint16_t kd_tmp = float_to_uint(kd, 0, this->Motor_para_set.KD, 12);
    uint16_t tor_tmp = float_to_uint(torq, -this->Motor_para_set.MAX_TOR, this->Motor_para_set.MAX_TOR, 12);
    uint8_t txdata[8];
    txdata[0] = pos_tmp >> 8;
    txdata[1] = pos_tmp;
    txdata[2] = spd_tmp >> 4;
    txdata[3] = ((spd_tmp & 0xF) << 4) | (kp_tmp >> 8);
    txdata[4] = kp_tmp;
    txdata[5] = kd_tmp >> 4;
    txdata[6] = ((kd_tmp & 0xf) << 4) | (tor_tmp >> 8);
    txdata[7] = tor_tmp;

    send_frame_(CAN_ID, txdata, sizeof(txdata));
}

void DmDriver_c::Enable() {
    uint8_t txdata[8];
    std::array<uint8_t, 8> cmd = setCtrl(DM_ENABLE);
    memcpy(txdata, cmd.data(), 8);

    send_frame_(CAN_ID, txdata, sizeof(txdata));
    motor_init_flag_ = true;
    enabled = true;
}

void DmDriver_c::Disable() {
    uint8_t txdata[8];
    std::array<uint8_t, 8> cmd = setCtrl(DM_DISABLE);
    memcpy(txdata, cmd.data(), 8);

    send_frame_(CAN_ID, txdata, sizeof(txdata));
    enabled = false;
}

void DmDriver_c::OnlyReceive() {}

void DmDriver_c::ClearErrorFlag() {
    uint8_t txdata[8];
    std::array<uint8_t, 8> cmd = setCtrl(DM_CLEAR_ERROR);
    memcpy(txdata, cmd.data(), 8);

    send_frame_(CAN_ID, txdata, sizeof(txdata));
}

void DmDriver_c::SetZero() {
    SavePosZero();
}

void DmDriver_c::SavePosZero() {
    uint8_t txdata[8];
    std::array<uint8_t, 8> cmd = setCtrl(DM_SAVE_POS_ZERO);
    memcpy(txdata, cmd.data(), 8);

    send_frame_(CAN_ID, txdata, sizeof(txdata));
}

void DmDriver_c::ClearError() {
    ClearErrorFlag();
}

void DmDriver_c::SelfLock(float kp, float kd, float torque) {
    if (!self_lock_enabled_) {
        self_lock_pos_rad_ = Motor_Data.Position;
        self_lock_enabled_ = true;
    }
    send_MIT_data(self_lock_pos_rad_, 0.0f, kp, kd, torque);
}

void DmDriver_c::CancelSelfLock() { self_lock_enabled_ = false; }

void DmDriver_c::SendReadParam(uint8_t rid) {
    const uint8_t payload[4] = {0, 0, 0, 0};
    send_param_frame(DM_PARAM_READ, rid, payload);
}

void DmDriver_c::SendWriteParamFloat(uint8_t rid, float value) {
    uint8_t payload[4] = {0, 0, 0, 0};
    memcpy(payload, &value, sizeof(float));
    send_param_frame(DM_PARAM_WRITE, rid, payload);
    if (rid == DM_PARAM_RID_TMAX) {
        UpdateTorqueLimit(value);
    }
}

void DmDriver_c::SendStoreParam() {
    const uint8_t payload[4] = {0, 0, 0, 0};
    send_param_frame(DM_PARAM_STORE, 0, payload);
}

void DmDriver_c::SetLimits(float p_max, float v_max, float t_max) {
    Motor_para_set.MAX_POS = p_max;
    Motor_para_set.MAX_VEL = v_max;
    UpdateTorqueLimit(t_max);
}

void DmDriver_c::SetOnlineCallback(std::function<void()> cb) { online_cb_ = cb; }

float DmDriver_c::GetLastReadTmax() const { return last_read_tmax_; }
