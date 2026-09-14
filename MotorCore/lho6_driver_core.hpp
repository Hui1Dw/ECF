#ifndef MOTOR_CORE_LHO6_DRIVER_CORE_HPP_
#define MOTOR_CORE_LHO6_DRIVER_CORE_HPP_

#include <cstdint>
#include <cstring>
#include <functional>

/**
 * @file lho6_driver_core.hpp
 * @brief LHO6 灵巧手 CAN 协议核心。
 *
 * 每个 Lho6Hand_c 实例只管理一只手。驱动依赖通信对象提供
 * SetRxCallBack、SetDLC、Transmit，不再直接使用 STM32 FDCAN HAL。
 */

/** @brief 单只 LHO6 手的关节数量。 */
constexpr uint8_t LHO6_JOINT_COUNT = 6;
/** @brief 协议中支持的触觉压力传感器数量。 */
constexpr uint8_t LHO6_PRESSURE_SENSOR_COUNT = 5;
/** @brief 单个压力传感器矩阵最多缓存的单元数量。 */
constexpr uint16_t LHO6_PRESSURE_MAX_CELLS = 256;
/** @brief 右手默认 CAN 标准帧 ID。 */
constexpr uint32_t LHO6_RIGHT_CAN_ID = 0x27;
/** @brief 左手默认 CAN 标准帧 ID。 */
constexpr uint32_t LHO6_LEFT_CAN_ID = 0x28;

/** @brief LHO6 物理左右手标识。 */
enum class Lho6HandSide_e : uint8_t {
    RIGHT = 0,
    LEFT = 1,
};

/** @brief LHO6 协议功能码。 */
enum class Lho6Cmd_e : uint8_t {
    POSITION = 0x01,
    TORQUE_LIMIT = 0x02,
    SPEED = 0x05,
    ACCELERATION = 0x87,
    TEMPERATURE = 0x33,
    ERROR_CODE = 0x35,
    PRESSURE_THUMB = 0xB1,
    PRESSURE_INDEX = 0xB2,
    PRESSURE_MIDDLE = 0xB3,
    PRESSURE_RING = 0xB4,
    PRESSURE_LITTLE = 0xB5,
    FACTORY_CODE = 0xC0,
    PCB_VERSION = 0xC1,
    SOFTWARE_VERSION = 0xC2,
    CHANGE_CAN_ID = 0xC3,
    MECHANICAL_VERSION = 0xC4,
    STALL_THRESHOLD = 0xC5,
    STALL_TIME = 0xC6,
    STALL_TORQUE = 0xC7,
    RESTORE_FACTORY = 0xCE,
    SAVE_PARAMS = 0xCF,
};

/**
 * @brief 单只 LHO6 手的六关节原始数据。
 *
 * joint[0..5] 使用协议原始 0x00..0xff 数值，顺序与 LHO6 协议表保持一致。
 */
struct Lho6JointData_t {
    uint8_t joint[LHO6_JOINT_COUNT];
};

/** @brief LHO6 版本号三元组。 */
struct Lho6Version_t {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
};

/** @brief 单关节错误状态位解析结果。 */
struct Lho6ErrorState_t {
    bool voltage;
    bool encoder;
    bool temperature;
    bool current;
    bool overload;
};

/**
 * @brief 单只 LHO6 手的控制目标缓存。
 *
 * 这些字段记录上层最近一次下发的目标值，不代表下位机已经执行成功。
 */
struct Lho6ControlData_t {
    Lho6JointData_t target_position;
    Lho6JointData_t target_torque_limit;
    Lho6JointData_t target_speed;
    Lho6JointData_t target_acceleration;
    Lho6JointData_t stall_threshold;
    Lho6JointData_t stall_time;
    Lho6JointData_t stall_torque;
    uint8_t pressure_matrix_size[LHO6_PRESSURE_SENSOR_COUNT];
};

/**
 * @brief 单只 LHO6 手的反馈缓存。
 *
 * Analysis() 在收到本实例 CAN ID 的反馈帧后更新该结构。
 */
struct Lho6FeedbackData_t {
    Lho6JointData_t position;
    Lho6JointData_t torque_limit;
    Lho6JointData_t speed;
    Lho6JointData_t acceleration;
    Lho6JointData_t stall_threshold;
    Lho6JointData_t stall_time;
    Lho6JointData_t stall_torque;
    uint8_t temperature[LHO6_JOINT_COUNT];
    uint8_t error_raw[LHO6_JOINT_COUNT];
    Lho6ErrorState_t error_state[LHO6_JOINT_COUNT];
    Lho6Version_t pcb_version;
    Lho6Version_t software_version;
    Lho6Version_t mechanical_version;
    uint8_t factory_code[32];
    uint8_t factory_code_len;
    uint8_t pressure[LHO6_PRESSURE_SENSOR_COUNT][LHO6_PRESSURE_MAX_CELLS];
    uint16_t pressure_valid_count[LHO6_PRESSURE_SENSOR_COUNT];
    uint8_t changed_can_function_code;
    uint8_t changed_can_id;
    uint8_t restore_factory_ack;
    uint8_t save_params_ack;
    bool online;
    uint32_t last_rx_tick_ms;
};

/**
 * @brief 单只 LHO6 灵巧手 CAN 驱动。
 *
 * 构造时向通信对象注册接收回调，运行时通过 CAN ID 过滤属于本手的反馈帧。
 * 左右手各使用一个实例，控制目标和反馈数据互不共享。
 */
class Lho6Hand_c {
 public:
    /**
     * @brief 传入具体通信对象创建一只 LHO6 手的驱动实例。
     * @param can 必须提供 SetRxCallBack、SetDLC 和 Transmit 的通信对象。
     * @param can_id 本手接收和发送使用的 CAN 标准帧 ID。
     * @param side 左右手标识。
     *
     * @attention 通信对象的生命周期必须长于使用它的 Lho6Hand_c 对象。
     */
    template <typename CanType>
    Lho6Hand_c(CanType& can, uint32_t can_id, Lho6HandSide_e side)
        : CAN_ID(can_id), side_(side), position_write_pending_(false) {
        InitCaches();
        can.SetRxCallBack([this](CanType* instance) {
            this->Analysis(instance->rx_buff_, instance->rx_id_, instance->rx_len_);
        });
        send_frame_ = [&can](uint32_t id, const uint8_t* data, uint8_t length) {
            const uint32_t previous_id = can.tx_id_;
            can.tx_id_ = id;
            can.SetDLC(length);
            std::memcpy(can.tx_buff_, data, length);
            // BSP_n::CAN_OK 的值为 0，因此这里不直接依赖 CAN BSP 头文件。
            const bool success = static_cast<int>(can.Transmit(1.0f)) == 0;
            can.tx_id_ = previous_id;
            return success;
        };
    }

    /**
     * @brief 解析接收帧。
     * @param data 接收数据区。
     * @param id 接收帧 CAN ID。
     * @param len 接收数据长度，默认按经典 CAN 8 字节处理。
     */
    void Analysis(uint8_t* data, uint32_t id, uint8_t len = 8);

    /** @brief 下发六关节位置目标，数值范围为协议原始 0x00..0xff。 */
    bool SetPosition(const Lho6JointData_t& value);
    /** @brief 下发六关节力矩限制目标，数值范围为协议原始 0x00..0xff。 */
    bool SetTorqueLimit(const Lho6JointData_t& value);
    /** @brief 下发六关节速度目标，数值范围为协议原始 0x00..0xff。 */
    bool SetSpeed(const Lho6JointData_t& value);
    /** @brief 下发六关节加速度目标，数值范围为协议原始 0x00..0xff。 */
    bool SetAcceleration(const Lho6JointData_t& value);
    /** @brief 下发六关节堵转阈值参数。 */
    bool SetStallThreshold(const Lho6JointData_t& value);
    /** @brief 下发六关节堵转时间参数。 */
    bool SetStallTime(const Lho6JointData_t& value);
    /** @brief 下发六关节堵转力矩参数。 */
    bool SetStallTorque(const Lho6JointData_t& value);

    /** @brief 查询六关节当前位置。 */
    bool ReadPosition();
    /** @brief 查询六关节力矩限制反馈。 */
    bool ReadTorqueLimit();
    /** @brief 查询六关节速度反馈。 */
    bool ReadSpeed();
    /** @brief 查询六关节加速度反馈。 */
    bool ReadAcceleration();
    /** @brief 查询六关节温度。 */
    bool ReadTemperature();
    /** @brief 查询六关节错误码。 */
    bool ReadErrorCode();
    /** @brief 查询 PCB 版本号。 */
    bool ReadPcbVersion();
    /** @brief 查询软件版本号。 */
    bool ReadSoftwareVersion();
    /** @brief 查询机械版本号。 */
    bool ReadMechanicalVersion();
    /** @brief 查询工厂编码。 */
    bool ReadFactoryCode();
    /**
     * @brief 查询指定手指压力矩阵。
     * @param finger_index 手指索引，范围为 0..LHO6_PRESSURE_SENSOR_COUNT-1。
     * @param matrix_size 压力矩阵尺寸编码，高 4 bit 为行数，低 4 bit 为列数。
     */
    bool ReadPressure(uint8_t finger_index, uint8_t matrix_size);
    /** @brief 查询六关节堵转阈值参数。 */
    bool ReadStallThreshold();
    /** @brief 查询六关节堵转时间参数。 */
    bool ReadStallTime();
    /** @brief 查询六关节堵转力矩参数。 */
    bool ReadStallTorque();

    /** @brief 请求 LHO6 保存当前参数。 */
    bool SaveParams();
    /** @brief 请求 LHO6 恢复出厂参数。 */
    bool RestoreFactory();
    /**
     * @brief 修改 LHO6 CAN ID。
     * @param function_code 协议要求的功能校验码。
     * @param new_id 新 CAN 标准帧 ID。
     */
    bool ChangeCanId(uint8_t function_code, uint8_t new_id);

    /** @brief 获取控制目标缓存。 */
    const Lho6ControlData_t& GetControlData() const;
    /** @brief 获取最近一次反馈缓存。 */
    const Lho6FeedbackData_t& GetFeedbackData() const;
    /** @brief 获取本实例使用的 CAN ID。 */
    uint32_t GetCanId() const;
    /** @brief 获取本实例的左右手标识。 */
    Lho6HandSide_e GetSide() const;
    /**
     * @brief 根据最近接收时间更新在线状态。
     * @param now_ms 当前系统时间，单位 ms。
     * @param timeout_ms 超过该时间未收到反馈则置为离线。
     */
    void UpdateOnlineStatus(uint32_t now_ms, uint32_t timeout_ms);
    /**
     * @brief 将协议错误码解析为错误状态位。
     * @param raw LHO6 原始错误码。
     * @return 解析后的错误状态。
     */
    static Lho6ErrorState_t DecodeError(uint8_t raw);

 private:
    void InitCaches();
    bool SendRaw(const uint8_t* data, uint8_t len);
    bool SendJointCommand(Lho6Cmd_e cmd, const Lho6JointData_t& value);
    bool SendReadCommand(Lho6Cmd_e cmd);
    void ParseJointFeedback(Lho6Cmd_e cmd, const uint8_t* data);
    void ParseVersionFeedback(Lho6Cmd_e cmd, const uint8_t* data);
    void ParseFactoryCodeFrame(const uint8_t* data, uint8_t len);
    void ParsePressureFeedback(Lho6Cmd_e cmd, const uint8_t* data, uint8_t len);

    uint32_t CAN_ID;
    Lho6HandSide_e side_;
    /** @brief 位置写入后等待同功能码回包，用于过滤写回显和真实位置反馈。 */
    bool position_write_pending_;
    Lho6ControlData_t Control_Data;
    Lho6FeedbackData_t Feedback_Data;
    std::function<bool(uint32_t, const uint8_t*, uint8_t)> send_frame_;
};

#endif  // MOTOR_CORE_LHO6_DRIVER_CORE_HPP_
