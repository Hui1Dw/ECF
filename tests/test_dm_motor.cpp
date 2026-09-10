#include "MotorCore/dm_driver_core.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <functional>
#include <utility>

/**
 * @brief 为主机单元测试提供 HAL 时基桩函数。
 * @return 固定测试时刻。
 */
extern "C" uint32_t HAL_GetTick(void) { return 100U; }

class MockCan_c final : public BSP_n::CanBase_c {
 public:
  using MockRxCallback_t = std::function<void(MockCan_c *)>;

  /**
   * @brief 构造模拟 CAN 通信对象。
   */
  MockCan_c() : CanBase_c(0x00EU, 0x10EU) {}

  /**
   * @brief 设置模拟发送长度。
   * @param length 数据长度。
   */
  void SetDLC(uint32_t length) override { tx_length = length; }

  /**
   * @brief 保存一次模拟发送的 ID 与数据。
   * @param timeout_ms 模拟超时时间，本测试不使用。
   * @return CAN_OK 始终发送成功。
   */
  BSP_n::CanState_e Transmit(float timeout_ms) override {
    (void)timeout_ms;
    transmitted = true;
    transmitted_id = tx_id_;
    std::memcpy(transmitted_data, tx_buff_, sizeof(transmitted_data));
    return BSP_n::CAN_OK;
  }

  /**
   * @brief 设置模拟接收回调。
   * @param callback 接收回调函数。
   */
  void SetRxCallBack(MockRxCallback_t callback) {
    callback_ = std::move(callback);
  }

  /**
   * @brief 注入一帧模拟接收数据并执行回调。
   * @param id 接收报文 ID。
   * @param data 八字节测试报文。
   */
  void Inject(uint32_t id, const uint8_t (&data)[8]) {
    rx_id_ = id;
    std::memcpy(rx_buff_, data, sizeof(data));
    callback_(this);
  }

  MockRxCallback_t callback_{};
  bool transmitted = false;
  uint32_t transmitted_id = 0U;
  uint32_t tx_length = 0U;
  uint8_t transmitted_data[8]{};
};

/**
 * @brief 验证 dm_driver_core 经通信对象发送和接收报文。
 * @return 0 全部断言通过。
 */
int main() {
  MockCan_c can;
  DmDriver_c motor(can, 0x00EU, 0x10EU, false, DM4310);

  motor.Enable();
  assert(can.transmitted);
  assert(can.transmitted_id == 0x00EU);
  assert(can.tx_length == 8U);
  for (uint8_t i = 0U; i < 7U; ++i) {
    assert(can.transmitted_data[i] == 0xFFU);
  }
  assert(can.transmitted_data[7] == DM_ENABLE);

  motor.Control_MIT(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  const uint8_t zero_command[8] = {0x7F, 0xFF, 0x7F, 0xF0,
                                    0x00, 0x00, 0x07, 0xFF};
  assert(std::memcmp(can.transmitted_data, zero_command,
                     sizeof(zero_command)) == 0);

  const uint8_t feedback[8] = {
      0x1E, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF6, 20U};
  can.Inject(0x10EU, feedback);
  assert(std::fabs(motor.Motor_Data.Position - 12.5F) < 0.001F);
  assert(std::fabs(motor.Motor_Data.Velocity - 50.0F) < 0.001F);
  assert(std::fabs(motor.Motor_Data.Torque - 10.0F) < 0.001F);
  assert(motor.Motor_Data.T_MOS == -10.0F);
  assert(motor.Motor_Data.T_Rotor == 20.0F);
  assert(motor.Motor_Data.Error_Code == DM_ERR_ENABLED);

  motor.SendReadParam(DM_PARAM_RID_TMAX);
  assert(can.transmitted_id == DM_PARAM_FRAME_ID);
  assert(can.transmitted_data[0] == 0x0EU);
  assert(can.transmitted_data[2] == DM_PARAM_READ);
  assert(can.transmitted_data[3] == DM_PARAM_RID_TMAX);

  const float new_torque_limit = 15.0F;
  uint8_t parameter_feedback[8] = {
      0x0E, 0x00, DM_PARAM_READ, DM_PARAM_RID_TMAX, 0U, 0U, 0U, 0U};
  std::memcpy(&parameter_feedback[4], &new_torque_limit,
              sizeof(new_torque_limit));
  can.Inject(DM_PARAM_FRAME_ID, parameter_feedback);
  assert(motor.GetLastReadTmax() == new_torque_limit);
  return 0;
}
