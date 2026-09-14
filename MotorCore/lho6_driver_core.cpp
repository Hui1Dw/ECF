#include "MotorCore/lho6_driver_core.hpp"

extern "C" uint32_t HAL_GetTick(void);

/**
 * @brief 初始化单只 LHO6 手的控制/反馈缓存。
 *
 * 默认位置目标初始化为 0xFF，速度、力矩限制和加速度初始化为 0x80。
 */
void Lho6Hand_c::InitCaches() {
    std::memset(&Control_Data, 0, sizeof(Control_Data));
    std::memset(&Feedback_Data, 0, sizeof(Feedback_Data));

    for (uint8_t i = 0; i < LHO6_JOINT_COUNT; ++i) {
        Control_Data.target_position.joint[i] = 0xFF;
        Control_Data.target_torque_limit.joint[i] = 0x80;
        Control_Data.target_speed.joint[i] = 0x80;
        Control_Data.target_acceleration.joint[i] = 0x80;
    }
    for (uint8_t i = 0; i < LHO6_PRESSURE_SENSOR_COUNT; ++i) {
        Control_Data.pressure_matrix_size[i] = 0xA4;
    }
}

/**
 * @brief 解析本手 CAN ID 对应的 LHO6 反馈帧。
 *
 * LHO6 位置写入和位置读取共用 0x01 功能码。写位置后如果收到的数据
 * 与目标位置完全一致，则认为是写回显并清除 position_write_pending_，
 * 不把该帧写入实际位置反馈，避免把控制目标误当作真实位置。
 */
void Lho6Hand_c::Analysis(uint8_t* data, uint32_t id, uint8_t len) {
    if (data == nullptr || id != CAN_ID || len == 0 || len > 8) {
        return;
    }

    bool parsed = false;
    Lho6Cmd_e cmd = static_cast<Lho6Cmd_e>(data[0]);
    switch (cmd) {
        case Lho6Cmd_e::POSITION:
            if (len >= 7) {
                if (position_write_pending_ &&
                    std::memcmp(&data[1],
                                Control_Data.target_position.joint,
                                LHO6_JOINT_COUNT) == 0) {
                    position_write_pending_ = false;
                } else {
                    ParseJointFeedback(cmd, data);
                }
                parsed = true;
            }
            break;
        case Lho6Cmd_e::TORQUE_LIMIT:
        case Lho6Cmd_e::SPEED:
        case Lho6Cmd_e::ACCELERATION:
        case Lho6Cmd_e::TEMPERATURE:
        case Lho6Cmd_e::ERROR_CODE:
        case Lho6Cmd_e::STALL_THRESHOLD:
        case Lho6Cmd_e::STALL_TIME:
        case Lho6Cmd_e::STALL_TORQUE:
            if (len >= 7) {
                ParseJointFeedback(cmd, data);
                parsed = true;
            }
            break;
        case Lho6Cmd_e::PCB_VERSION:
        case Lho6Cmd_e::SOFTWARE_VERSION:
        case Lho6Cmd_e::MECHANICAL_VERSION:
            if (len >= 4) {
                ParseVersionFeedback(cmd, data);
                parsed = true;
            }
            break;
        case Lho6Cmd_e::FACTORY_CODE:
            if (len >= 3) {
                ParseFactoryCodeFrame(data, len);
                parsed = true;
            }
            break;
        case Lho6Cmd_e::PRESSURE_THUMB:
        case Lho6Cmd_e::PRESSURE_INDEX:
        case Lho6Cmd_e::PRESSURE_MIDDLE:
        case Lho6Cmd_e::PRESSURE_RING:
        case Lho6Cmd_e::PRESSURE_LITTLE:
            if (len >= 3) {
                ParsePressureFeedback(cmd, data, len);
                parsed = true;
            }
            break;
        case Lho6Cmd_e::CHANGE_CAN_ID:
            if (len >= 3) {
                Feedback_Data.changed_can_function_code = data[1];
                Feedback_Data.changed_can_id = data[2];
                parsed = true;
            }
            break;
        case Lho6Cmd_e::RESTORE_FACTORY:
            if (len >= 2) {
                Feedback_Data.restore_factory_ack = data[1];
                parsed = true;
            }
            break;
        case Lho6Cmd_e::SAVE_PARAMS:
            if (len >= 2) {
                Feedback_Data.save_params_ack = data[1];
                parsed = true;
            }
            break;
        default:
            break;
    }

    if (parsed) {
        Feedback_Data.online = true;
        Feedback_Data.last_rx_tick_ms = HAL_GetTick();
    }
}

/**
 * @brief 下发六关节位置目标，并标记等待位置写回显。
 */
bool Lho6Hand_c::SetPosition(const Lho6JointData_t& value) {
    Control_Data.target_position = value;
    const bool sent = SendJointCommand(Lho6Cmd_e::POSITION, value);
    if (sent) {
        position_write_pending_ = true;
    }
    return sent;
}

/** @brief 下发六关节力矩限制目标并更新控制缓存。 */
bool Lho6Hand_c::SetTorqueLimit(const Lho6JointData_t& value) {
    Control_Data.target_torque_limit = value;
    return SendJointCommand(Lho6Cmd_e::TORQUE_LIMIT, value);
}

/** @brief 下发六关节速度目标并更新控制缓存。 */
bool Lho6Hand_c::SetSpeed(const Lho6JointData_t& value) {
    Control_Data.target_speed = value;
    return SendJointCommand(Lho6Cmd_e::SPEED, value);
}

/** @brief 下发六关节加速度目标并更新控制缓存。 */
bool Lho6Hand_c::SetAcceleration(const Lho6JointData_t& value) {
    Control_Data.target_acceleration = value;
    return SendJointCommand(Lho6Cmd_e::ACCELERATION, value);
}

/** @brief 下发六关节堵转阈值参数并更新控制缓存。 */
bool Lho6Hand_c::SetStallThreshold(const Lho6JointData_t& value) {
    Control_Data.stall_threshold = value;
    return SendJointCommand(Lho6Cmd_e::STALL_THRESHOLD, value);
}

/** @brief 下发六关节堵转时间参数并更新控制缓存。 */
bool Lho6Hand_c::SetStallTime(const Lho6JointData_t& value) {
    Control_Data.stall_time = value;
    return SendJointCommand(Lho6Cmd_e::STALL_TIME, value);
}

/** @brief 下发六关节堵转力矩参数并更新控制缓存。 */
bool Lho6Hand_c::SetStallTorque(const Lho6JointData_t& value) {
    Control_Data.stall_torque = value;
    return SendJointCommand(Lho6Cmd_e::STALL_TORQUE, value);
}

/** @brief 发送位置读取命令。 */
bool Lho6Hand_c::ReadPosition() { return SendReadCommand(Lho6Cmd_e::POSITION); }

/** @brief 发送力矩限制读取命令。 */
bool Lho6Hand_c::ReadTorqueLimit() { return SendReadCommand(Lho6Cmd_e::TORQUE_LIMIT); }

/** @brief 发送速度读取命令。 */
bool Lho6Hand_c::ReadSpeed() { return SendReadCommand(Lho6Cmd_e::SPEED); }

/** @brief 发送加速度读取命令。 */
bool Lho6Hand_c::ReadAcceleration() { return SendReadCommand(Lho6Cmd_e::ACCELERATION); }

/** @brief 发送温度读取命令。 */
bool Lho6Hand_c::ReadTemperature() { return SendReadCommand(Lho6Cmd_e::TEMPERATURE); }

/** @brief 发送错误码读取命令。 */
bool Lho6Hand_c::ReadErrorCode() { return SendReadCommand(Lho6Cmd_e::ERROR_CODE); }

/** @brief 发送 PCB 版本读取命令。 */
bool Lho6Hand_c::ReadPcbVersion() { return SendReadCommand(Lho6Cmd_e::PCB_VERSION); }

/** @brief 发送软件版本读取命令。 */
bool Lho6Hand_c::ReadSoftwareVersion() { return SendReadCommand(Lho6Cmd_e::SOFTWARE_VERSION); }

/** @brief 发送机械版本读取命令。 */
bool Lho6Hand_c::ReadMechanicalVersion() { return SendReadCommand(Lho6Cmd_e::MECHANICAL_VERSION); }

/**
 * @brief 清空工厂编码缓存后发送工厂编码读取命令。
 */
bool Lho6Hand_c::ReadFactoryCode() {
    std::memset(Feedback_Data.factory_code, 0, sizeof(Feedback_Data.factory_code));
    Feedback_Data.factory_code_len = 0;
    return SendReadCommand(Lho6Cmd_e::FACTORY_CODE);
}

/**
 * @brief 发送指定手指压力矩阵读取命令。
 *
 * matrix_size 的高 4 bit 为行数，低 4 bit 为列数。函数会拒绝非法手指索引
 * 或空矩阵尺寸，并在发起读取前清空该手指的有效压力数据长度。
 */
bool Lho6Hand_c::ReadPressure(uint8_t finger_index, uint8_t matrix_size) {
    const uint8_t rows = static_cast<uint8_t>(matrix_size >> 4U);
    const uint8_t columns = static_cast<uint8_t>(matrix_size & 0x0FU);
    if (finger_index >= LHO6_PRESSURE_SENSOR_COUNT || rows == 0 || columns == 0) {
        return false;
    }
    Control_Data.pressure_matrix_size[finger_index] = matrix_size;
    Feedback_Data.pressure_valid_count[finger_index] = 0;
    uint8_t data[2] = {static_cast<uint8_t>(0xB1U + finger_index), matrix_size};
    return SendRaw(data, sizeof(data));
}

/** @brief 发送堵转阈值读取命令。 */
bool Lho6Hand_c::ReadStallThreshold() { return SendReadCommand(Lho6Cmd_e::STALL_THRESHOLD); }

/** @brief 发送堵转时间读取命令。 */
bool Lho6Hand_c::ReadStallTime() { return SendReadCommand(Lho6Cmd_e::STALL_TIME); }

/** @brief 发送堵转力矩读取命令。 */
bool Lho6Hand_c::ReadStallTorque() { return SendReadCommand(Lho6Cmd_e::STALL_TORQUE); }

/**
 * @brief 发送保存参数命令。
 *
 * 协议要求 8 字节均为 0xCF。发送前清空保存确认值，等待反馈帧重新写入。
 */
bool Lho6Hand_c::SaveParams() {
    Feedback_Data.save_params_ack = 0;
    uint8_t data[8] = {0xCF, 0xCF, 0xCF, 0xCF, 0xCF, 0xCF, 0xCF, 0xCF};
    return SendRaw(data, sizeof(data));
}

/**
 * @brief 发送恢复出厂命令。
 *
 * 协议要求 8 字节均为 0xCE。发送前清空恢复确认值，等待反馈帧重新写入。
 */
bool Lho6Hand_c::RestoreFactory() {
    Feedback_Data.restore_factory_ack = 0;
    uint8_t data[8] = {0xCE, 0xCE, 0xCE, 0xCE, 0xCE, 0xCE, 0xCE, 0xCE};
    return SendRaw(data, sizeof(data));
}

/**
 * @brief 发送修改 CAN ID 命令。
 *
 * function_code 和 new_id 按 LHO6 协议原样发送，反馈会写入
 * changed_can_function_code 和 changed_can_id。
 */
bool Lho6Hand_c::ChangeCanId(uint8_t function_code, uint8_t new_id) {
    Feedback_Data.changed_can_function_code = 0;
    Feedback_Data.changed_can_id = 0;
    uint8_t data[3] = {static_cast<uint8_t>(Lho6Cmd_e::CHANGE_CAN_ID), function_code, new_id};
    return SendRaw(data, sizeof(data));
}

/** @brief 返回本手控制目标缓存的只读引用。 */
const Lho6ControlData_t& Lho6Hand_c::GetControlData() const { return Control_Data; }

/** @brief 返回本手反馈缓存的只读引用。 */
const Lho6FeedbackData_t& Lho6Hand_c::GetFeedbackData() const { return Feedback_Data; }

/** @brief 返回本手使用的 CAN 标准帧 ID。 */
uint32_t Lho6Hand_c::GetCanId() const { return CAN_ID; }

/** @brief 返回本实例对应的左右手标识。 */
Lho6HandSide_e Lho6Hand_c::GetSide() const { return side_; }

/**
 * @brief 根据最后接收时间维护在线状态。
 */
void Lho6Hand_c::UpdateOnlineStatus(uint32_t now_ms, uint32_t timeout_ms) {
    if (Feedback_Data.online && (now_ms - Feedback_Data.last_rx_tick_ms) > timeout_ms) {
        Feedback_Data.online = false;
    }
}

/**
 * @brief 将 LHO6 原始错误字节解析为布尔错误状态。
 */
Lho6ErrorState_t Lho6Hand_c::DecodeError(uint8_t raw) {
    Lho6ErrorState_t err{};
    err.voltage = (raw & 0x01U) != 0;
    err.encoder = (raw & 0x02U) != 0;
    err.temperature = (raw & 0x04U) != 0;
    err.current = (raw & 0x08U) != 0;
    err.overload = (raw & 0x20U) != 0;
    return err;
}

/**
 * @brief 通过通信对象发送一帧经典 CAN 数据。
 */
bool Lho6Hand_c::SendRaw(const uint8_t* data, uint8_t len) {
    if (data == nullptr || len == 0 || len > 8 || !send_frame_) {
        return false;
    }
    return send_frame_(CAN_ID, data, len);
}

/**
 * @brief 发送 1 字节功能码加 6 字节关节数据的写命令。
 */
bool Lho6Hand_c::SendJointCommand(Lho6Cmd_e cmd, const Lho6JointData_t& value) {
    uint8_t data[7] = {static_cast<uint8_t>(cmd)};
    std::memcpy(&data[1], value.joint, LHO6_JOINT_COUNT);
    return SendRaw(data, sizeof(data));
}

/**
 * @brief 发送 1 字节功能码的读取命令。
 */
bool Lho6Hand_c::SendReadCommand(Lho6Cmd_e cmd) {
    uint8_t data[1] = {static_cast<uint8_t>(cmd)};
    return SendRaw(data, sizeof(data));
}

/**
 * @brief 根据功能码把六关节类反馈写入对应缓存字段。
 */
void Lho6Hand_c::ParseJointFeedback(Lho6Cmd_e cmd, const uint8_t* data) {
    switch (cmd) {
        case Lho6Cmd_e::POSITION:
            std::memcpy(Feedback_Data.position.joint, &data[1], LHO6_JOINT_COUNT);
            break;
        case Lho6Cmd_e::TORQUE_LIMIT:
            std::memcpy(Feedback_Data.torque_limit.joint, &data[1], LHO6_JOINT_COUNT);
            break;
        case Lho6Cmd_e::SPEED:
            std::memcpy(Feedback_Data.speed.joint, &data[1], LHO6_JOINT_COUNT);
            break;
        case Lho6Cmd_e::ACCELERATION:
            std::memcpy(Feedback_Data.acceleration.joint, &data[1], LHO6_JOINT_COUNT);
            break;
        case Lho6Cmd_e::STALL_THRESHOLD:
            std::memcpy(Feedback_Data.stall_threshold.joint, &data[1], LHO6_JOINT_COUNT);
            break;
        case Lho6Cmd_e::STALL_TIME:
            std::memcpy(Feedback_Data.stall_time.joint, &data[1], LHO6_JOINT_COUNT);
            break;
        case Lho6Cmd_e::STALL_TORQUE:
            std::memcpy(Feedback_Data.stall_torque.joint, &data[1], LHO6_JOINT_COUNT);
            break;
        case Lho6Cmd_e::TEMPERATURE:
            std::memcpy(Feedback_Data.temperature, &data[1], LHO6_JOINT_COUNT);
            break;
        case Lho6Cmd_e::ERROR_CODE:
            std::memcpy(Feedback_Data.error_raw, &data[1], LHO6_JOINT_COUNT);
            for (uint8_t i = 0; i < LHO6_JOINT_COUNT; ++i) {
                Feedback_Data.error_state[i] = DecodeError(Feedback_Data.error_raw[i]);
            }
            break;
        default:
            break;
    }
}

/**
 * @brief 解析 PCB、软件或机械版本反馈。
 */
void Lho6Hand_c::ParseVersionFeedback(Lho6Cmd_e cmd, const uint8_t* data) {
    Lho6Version_t version{data[1], data[2], data[3]};
    switch (cmd) {
        case Lho6Cmd_e::PCB_VERSION:
            Feedback_Data.pcb_version = version;
            break;
        case Lho6Cmd_e::SOFTWARE_VERSION:
            Feedback_Data.software_version = version;
            break;
        case Lho6Cmd_e::MECHANICAL_VERSION:
            Feedback_Data.mechanical_version = version;
            break;
        default:
            break;
    }
}

/**
 * @brief 解析工厂编码分片反馈。
 *
 * data[1] 为分片行号，每行最多携带 6 字节有效内容。函数按行号写入
 * factory_code 缓存，并维护当前已接收的最长有效长度。
 */
void Lho6Hand_c::ParseFactoryCodeFrame(const uint8_t* data, uint8_t len) {
    if (len < 3) {
        return;
    }
    uint8_t row = data[1];
    uint8_t offset = static_cast<uint8_t>(row * 6U);
    if (offset >= sizeof(Feedback_Data.factory_code)) {
        return;
    }
    uint8_t copy_len = static_cast<uint8_t>(len - 2U);
    if (offset + copy_len > sizeof(Feedback_Data.factory_code)) {
        copy_len = static_cast<uint8_t>(sizeof(Feedback_Data.factory_code) - offset);
    }
    std::memcpy(&Feedback_Data.factory_code[offset], &data[2], copy_len);
    uint8_t end = static_cast<uint8_t>(offset + copy_len);
    if (end > Feedback_Data.factory_code_len) {
        Feedback_Data.factory_code_len = end;
    }
}

/**
 * @brief 解析压力矩阵分片反馈。
 *
 * 功能码决定手指索引，data[1] 的高 4 bit 为行号、低 4 bit 为列号。
 * 偏移按最近一次 ReadPressure() 记录的列数计算。
 */
void Lho6Hand_c::ParsePressureFeedback(Lho6Cmd_e cmd, const uint8_t* data, uint8_t len) {
    const uint8_t finger_index =
        static_cast<uint8_t>(static_cast<uint8_t>(cmd) - static_cast<uint8_t>(Lho6Cmd_e::PRESSURE_THUMB));
    if (finger_index >= LHO6_PRESSURE_SENSOR_COUNT) {
        return;
    }

    const uint8_t columns = static_cast<uint8_t>(Control_Data.pressure_matrix_size[finger_index] & 0x0FU);
    if (columns == 0) {
        return;
    }

    const uint8_t row = static_cast<uint8_t>(data[1] >> 4U);
    const uint8_t column = static_cast<uint8_t>(data[1] & 0x0FU);
    uint16_t offset = static_cast<uint16_t>(row) * columns + column;
    uint8_t copy_len = static_cast<uint8_t>(len - 2U);
    if (offset >= LHO6_PRESSURE_MAX_CELLS) {
        return;
    }
    if (offset + copy_len > LHO6_PRESSURE_MAX_CELLS) {
        copy_len = static_cast<uint8_t>(LHO6_PRESSURE_MAX_CELLS - offset);
    }

    std::memcpy(&Feedback_Data.pressure[finger_index][offset], &data[2], copy_len);
    const uint16_t end = static_cast<uint16_t>(offset + copy_len);
    if (end > Feedback_Data.pressure_valid_count[finger_index]) {
        Feedback_Data.pressure_valid_count[finger_index] = end;
    }
}
