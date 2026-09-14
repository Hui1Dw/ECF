#include "LHO6Hand/lho6_driver.hpp"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <functional>
#include <utility>

/**
 * @brief 为主机单元测试提供 HAL 时基桩函数。
 * @return 固定测试时刻。
 */
extern "C" uint32_t HAL_GetTick(void) { return 100U; }

class MockCan_c {
 public:
    using MockRxCallback_t = std::function<void(MockCan_c*)>;

    /**
     * @brief 构造模拟 CAN 通信对象。
     */
    MockCan_c() = default;

    /**
     * @brief 设置模拟发送长度。
     * @param length 数据长度。
     */
    void SetDLC(uint32_t length) { tx_length = length; }

    /**
     * @brief 保存一次模拟发送的 ID 与数据。
     * @param timeout_ms 模拟超时时间，本测试不使用。
     * @return 0 表示发送成功，对应 BSP_n::CAN_OK。
     */
    int Transmit(float timeout_ms) {
        (void)timeout_ms;
        transmitted = true;
        transmitted_id = tx_id_;
        std::memcpy(transmitted_data, tx_buff_, sizeof(transmitted_data));
        return 0;
    }

    /**
     * @brief 设置模拟接收回调。
     * @param callback 接收回调函数。
     */
    void SetRxCallBack(MockRxCallback_t callback) { callback_ = std::move(callback); }

    /**
     * @brief 注入一帧模拟接收数据并执行回调。
     * @param id 接收报文 ID。
     * @param data 测试报文。
     * @param len 实际数据长度。
     */
    void Inject(uint32_t id, const uint8_t* data, uint8_t len) {
        rx_id_ = id;
        rx_len_ = len;
        std::memset(rx_buff_, 0, sizeof(rx_buff_));
        std::memcpy(rx_buff_, data, len);
        callback_(this);
    }

    uint32_t tx_id_ = 0U;
    uint32_t rx_id_ = 0U;
    uint8_t rx_len_ = 0U;
    uint8_t rx_buff_[8]{};
    uint8_t tx_buff_[8]{};
    MockRxCallback_t callback_{};
    bool transmitted = false;
    uint32_t transmitted_id = 0U;
    uint32_t tx_length = 0U;
    uint8_t transmitted_data[8]{};
};

/**
 * @brief 验证 LHO6 驱动经通信对象发送和解析报文。
 * @return 0 全部断言通过。
 */
int main() {
    Lho6ErrorState_t err = Lho6Hand_c::DecodeError(0x2F);
    assert(err.voltage);
    assert(err.encoder);
    assert(err.temperature);
    assert(err.current);
    assert(err.overload);

    MockCan_c left_can;
    Lho6Hand_c left(left_can, LHO6_LEFT_CAN_ID, Lho6HandSide_e::LEFT);
    assert(left.GetCanId() == LHO6_LEFT_CAN_ID);
    assert(left.GetSide() == Lho6HandSide_e::LEFT);
    assert(left.GetControlData().target_position.joint[0] == 0xFF);
    assert(left.GetControlData().target_speed.joint[0] == 0x80);

    Lho6JointData_t target_position{{101, 102, 103, 104, 105, 106}};
    assert(left.SetPosition(target_position));
    assert(left_can.transmitted);
    assert(left_can.transmitted_id == LHO6_LEFT_CAN_ID);
    assert(left_can.tx_length == 7U);
    assert(left_can.transmitted_data[0] == 0x01);
    assert(std::memcmp(&left_can.transmitted_data[1], target_position.joint, LHO6_JOINT_COUNT) == 0);
    assert(left.GetControlData().target_position.joint[5] == 106);

    assert(left.ReadSpeed());
    assert(left_can.transmitted_id == LHO6_LEFT_CAN_ID);
    assert(left_can.tx_length == 1U);
    assert(left_can.transmitted_data[0] == 0x05);

    assert(left.SaveParams());
    assert(left_can.tx_length == 8U);
    for (uint8_t i = 0; i < 8U; ++i) {
        assert(left_can.transmitted_data[i] == 0xCF);
    }

    uint8_t pos_frame[8] = {0x01, 1, 2, 3, 4, 5, 6, 0};
    left_can.Inject(LHO6_LEFT_CAN_ID, pos_frame, 8);
    assert(left.GetFeedbackData().position.joint[0] == 1);
    assert(left.GetFeedbackData().position.joint[5] == 6);
    assert(left.GetFeedbackData().online);

    uint8_t pos_read_echo[1] = {0x01};
    left_can.Inject(LHO6_LEFT_CAN_ID, pos_read_echo, sizeof(pos_read_echo));
    assert(left.GetFeedbackData().position.joint[0] == 1);
    assert(left.GetFeedbackData().position.joint[5] == 6);

    uint8_t speed_frame[7] = {0x05, 21, 22, 23, 24, 25, 26};
    left_can.Inject(LHO6_LEFT_CAN_ID, speed_frame, sizeof(speed_frame));
    assert(left.GetFeedbackData().speed.joint[0] == 21);
    assert(left.GetFeedbackData().speed.joint[5] == 26);

    uint8_t torque_limit_frame[7] = {0x02, 31, 32, 33, 34, 35, 36};
    left_can.Inject(LHO6_LEFT_CAN_ID, torque_limit_frame, sizeof(torque_limit_frame));
    assert(left.GetFeedbackData().torque_limit.joint[0] == 31);
    assert(left.GetFeedbackData().torque_limit.joint[5] == 36);

    uint8_t position_write_ack[7] = {0x01, 101, 102, 103, 104, 105, 106};
    left_can.Inject(LHO6_LEFT_CAN_ID, position_write_ack, sizeof(position_write_ack));
    assert(left.GetFeedbackData().position.joint[0] == 1);
    assert(left.GetFeedbackData().position.joint[5] == 6);

    uint8_t wrong_frame[8] = {0x01, 9, 9, 9, 9, 9, 9, 0};
    left_can.Inject(LHO6_RIGHT_CAN_ID, wrong_frame, sizeof(wrong_frame));
    assert(left.GetFeedbackData().position.joint[0] == 1);
    assert(left.GetFeedbackData().position.joint[5] == 6);

    uint8_t stall_frame[7] = {0xC5, 10, 20, 30, 40, 50, 60};
    left_can.Inject(LHO6_LEFT_CAN_ID, stall_frame, sizeof(stall_frame));
    assert(left.GetFeedbackData().stall_threshold.joint[0] == 10);
    assert(left.GetFeedbackData().stall_threshold.joint[5] == 60);

    assert(left.ReadPressure(0, 0xA4));
    assert(left_can.tx_length == 2U);
    assert(left_can.transmitted_data[0] == 0xB1);
    assert(left_can.transmitted_data[1] == 0xA4);
    uint8_t pressure_frame[8] = {0xB1, 0x03, 11, 12, 13, 14, 15, 16};
    left_can.Inject(LHO6_LEFT_CAN_ID, pressure_frame, sizeof(pressure_frame));
    assert(left.GetFeedbackData().pressure[0][3] == 11);
    assert(left.GetFeedbackData().pressure[0][8] == 16);
    assert(left.GetFeedbackData().pressure_valid_count[0] == 9);

    uint8_t factory_frame_0[8] = {0xC0, 0x00, 'L', 'H', 'O', '6', '-', '0'};
    uint8_t factory_frame_1[3] = {0xC0, 0x01, 'A'};
    left_can.Inject(LHO6_LEFT_CAN_ID, factory_frame_0, sizeof(factory_frame_0));
    left_can.Inject(LHO6_LEFT_CAN_ID, factory_frame_1, sizeof(factory_frame_1));
    assert(left.GetFeedbackData().factory_code_len == 7);
    assert(left.GetFeedbackData().factory_code[6] == 'A');
    assert(left.GetFeedbackData().factory_code[7] == 0);

    assert(left.ReadFactoryCode());
    assert(left_can.tx_length == 1U);
    assert(left_can.transmitted_data[0] == 0xC0);
    uint8_t replacement_factory_frame[3] = {0xC0, 0x00, 'X'};
    left_can.Inject(LHO6_LEFT_CAN_ID, replacement_factory_frame, sizeof(replacement_factory_frame));
    assert(left.GetFeedbackData().factory_code_len == 1);
    assert(left.GetFeedbackData().factory_code[0] == 'X');
    assert(left.GetFeedbackData().factory_code[1] == 0);

    left.UpdateOnlineStatus(1000, 500);
    assert(!left.GetFeedbackData().online);

    MockCan_c right_can;
    Lho6Hand_c right(right_can, LHO6_RIGHT_CAN_ID, Lho6HandSide_e::RIGHT);
    uint8_t right_pos_frame[7] = {0x01, 41, 42, 43, 44, 45, 46};
    right_can.Inject(LHO6_RIGHT_CAN_ID, right_pos_frame, sizeof(right_pos_frame));
    assert(right.GetFeedbackData().position.joint[0] == 41);
    assert(right.GetFeedbackData().position.joint[5] == 46);

    return 0;
}
