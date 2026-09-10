#include "mcp2518fd.hpp"

#include <cstring>
#include <utility>

namespace BSP_n {

Mcp2518fd_c *Mcp2518fd_c::instances_[MX_SPI_BUS_SLAVE_CNT]{};
uint8_t Mcp2518fd_c::instance_count_ = 0U;

namespace {
constexpr uint32_t kModeTimeoutMs = 20U;
constexpr uint8_t kCmdReset = 0x00U;
constexpr uint8_t kCmdWrite = 0x02U;
constexpr uint8_t kCmdRead = 0x03U;
constexpr uint16_t kRegC1Con = 0x000U;
constexpr uint16_t kRegC1NbtCfg = 0x004U;
constexpr uint16_t kRegC1Int = 0x01CU;
constexpr uint16_t kRegC1FifoCon = 0x050U;
constexpr uint16_t kRegC1FltCon = 0x1D0U;
constexpr uint16_t kRegC1FltObj = 0x1F0U;
constexpr uint16_t kRegC1Mask = 0x1F4U;
constexpr uint16_t kRegOsc = 0xE00U;
constexpr uint16_t kRegEccCon = 0xE0CU;
constexpr uint16_t kRamStart = 0x400U;
constexpr uint16_t kRamSize = 2048U;
constexpr uint8_t kFifoRx = 1U;
constexpr uint8_t kFifoTx = 2U;
constexpr uint16_t kFifoStride = 12U;
constexpr uint16_t kFifoStatus = 4U;
constexpr uint16_t kFifoUa = 8U;
constexpr uint32_t kOscReady = 1U << 10;
constexpr uint8_t kEccEnable = 0x01U;
constexpr uint8_t kRxNotEmpty = 0x01U;
constexpr uint8_t kFifoUinc = 0x01U;
constexpr uint8_t kFifoTxReq = 0x02U;
constexpr uint8_t kModeClassic = 0x06U;
}  // namespace

/**
 * @brief 构造并初始化一个 MCP2518FD 通信对象。
 */
Mcp2518fd_c::Mcp2518fd_c(SPI_HandleTypeDef *spi_handle,
                         GPIO_TypeDef *cs_port, uint16_t cs_pin,
                         GPIO_TypeDef *int_port, uint16_t int_pin,
                         uint32_t tx_id, uint32_t rx_id,
                         Mcp2518fdRxCallback_t callback)
    : CanBase_c(tx_id, rx_id),
      spi_(SPI_Init_Config_s{spi_handle, cs_port, cs_pin, SPI_BLOCK_MODE,
                            &Mcp2518fd_c::SpiCompleteCallback}),
      spi_handle_(spi_handle),
      int_port_(int_port),
      int_pin_(int_pin),
      callback_(std::move(callback))
{
  if (instance_count_ < MX_SPI_BUS_SLAVE_CNT) {
    instances_[instance_count_++] = this;
  }
  ready_ = spi_handle != nullptr && cs_port != nullptr && int_port != nullptr &&
           Initialize();
}

/**
 * @brief 设置发送数据长度。
 * @param length 数据长度，范围为 0～8 字节。
 */
void Mcp2518fd_c::SetDLC(uint32_t length)
{
  if (length <= 8U) {
    tx_len_ = static_cast<uint8_t>(length);
  }
}

/**
 * @brief 发送 tx_buff_ 中的经典 CAN 标准帧。
 * @param timeout_ms 接口兼容参数，SPI BSP 内部使用固定有限超时。
 * @return CAN_OK 报文提交成功。
 * @return CAN_ERROR 外设未就绪或发送失败。
 */
CanState_e Mcp2518fd_c::Transmit(float timeout_ms)
{
  (void)timeout_ms;
  if (!ready_ || rx_state_ != RxState_e::IDLE) {
    return CAN_ERROR;
  }
  const bool success =
      SendFrame(static_cast<uint16_t>(tx_id_), tx_buff_, tx_len_);
  if (interrupt_pending_) {
    ProcessPending();
  }
  return success ? CAN_OK : CAN_ERROR;
}

/**
 * @brief 将 HAL GPIO 外部中断按引脚分发给已注册的 MCP2518FD 实例。
 */
void Mcp2518fd_c::ExtiCallback(uint16_t pin)
{
  for (uint8_t i = 0U; i < instance_count_; ++i) {
    Mcp2518fd_c *instance = instances_[i];
    if (instance->int_pin_ == pin &&
        HAL_GPIO_ReadPin(instance->int_port_, instance->int_pin_) ==
            GPIO_PIN_RESET) {
      instance->HandleInterrupt();
    }
  }
}

/**
 * @brief 设置当前对象的接收回调。
 * @param callback 收到报文时执行的实例回调，由上层协议筛选 ID。
 */
void Mcp2518fd_c::SetRxCallBack(Mcp2518fdRxCallback_t callback)
{
  callback_ = std::move(callback);
}

/**
 * @brief 启动 SPI DMA 接收，完成后由 SPI 回调解析并分发报文。
 */
void Mcp2518fd_c::ProcessPending()
{
  if (!ready_ || rx_state_ != RxState_e::IDLE ||
      HAL_SPI_GetState(spi_handle_) != HAL_SPI_STATE_READY) {
    interrupt_pending_ = true;
    return;
  }
  interrupt_pending_ = false;
  spi_.SetMode(SPI_DMA_MODE);
  const uint16_t fifo = kRegC1FifoCon + kFifoRx * kFifoStride;
  StartAsyncRead(fifo + kFifoStatus, 1U, RxState_e::READ_STATUS);
}

/**
 * @brief 记录 MCP INT 事件并在 SPI 空闲时启动接收。
 */
void Mcp2518fd_c::HandleInterrupt()
{
  interrupt_pending_ = true;
  ProcessPending();
}

/**
 * @brief 启动一次异步 SPI 读取事务。
 */
void Mcp2518fd_c::StartAsyncRead(uint16_t address, uint8_t size,
                                 RxState_e next_state)
{
  std::memset(async_tx_, 0, sizeof(async_tx_));
  std::memset(async_rx_, 0, sizeof(async_rx_));
  async_tx_[0] = static_cast<uint8_t>((kCmdRead << 4) |
                                      ((address >> 8) & 0x0FU));
  async_tx_[1] = static_cast<uint8_t>(address);
  rx_state_ = next_state;
  spi_.TransRecv(async_rx_, async_tx_, static_cast<uint8_t>(size + 2U));
}

/**
 * @brief 启动一次异步 SPI 写入事务。
 */
void Mcp2518fd_c::StartAsyncWrite(uint16_t address, const uint8_t *data,
                                  uint8_t size, RxState_e next_state)
{
  std::memset(async_tx_, 0, sizeof(async_tx_));
  std::memset(async_rx_, 0, sizeof(async_rx_));
  async_tx_[0] = static_cast<uint8_t>((kCmdWrite << 4) |
                                      ((address >> 8) & 0x0FU));
  async_tx_[1] = static_cast<uint8_t>(address);
  std::memcpy(&async_tx_[2], data, size);
  rx_state_ = next_state;
  spi_.TransRecv(async_rx_, async_tx_, static_cast<uint8_t>(size + 2U));
}

/**
 * @brief 处理 SPI DMA 完成事件并推进 MCP 接收流程。
 */
void Mcp2518fd_c::HandleSpiComplete()
{
  const uint16_t fifo = kRegC1FifoCon + kFifoRx * kFifoStride;
  switch (rx_state_) {
    case RxState_e::READ_STATUS:
      if ((async_rx_[2] & kRxNotEmpty) == 0U) {
        FinishAsyncReceive();
      } else {
        StartAsyncRead(fifo + kFifoUa, 4U, RxState_e::READ_ADDRESS);
      }
      break;

    case RxState_e::READ_ADDRESS: {
      const uint32_t address = static_cast<uint32_t>(async_rx_[2]) |
                               (static_cast<uint32_t>(async_rx_[3]) << 8) |
                               (static_cast<uint32_t>(async_rx_[4]) << 16) |
                               (static_cast<uint32_t>(async_rx_[5]) << 24);
      StartAsyncRead(static_cast<uint16_t>(kRamStart + (address & 0x0FFFU)),
                     16U, RxState_e::READ_OBJECT);
      break;
    }

    case RxState_e::READ_OBJECT: {
      const uint8_t *object = &async_rx_[2];
      async_frame_.id = static_cast<uint16_t>(object[0]) |
                        (static_cast<uint16_t>(object[1] & 0x07U) << 8);
      async_frame_.dlc = object[4] & 0x0FU;
      async_frame_valid_ = async_frame_.dlc <= 8U &&
                           (object[4] & 0x30U) == 0U;
      if (async_frame_valid_) {
        std::memcpy(async_frame_.data, &object[8], async_frame_.dlc);
      }
      const uint8_t update = kFifoUinc;
      StartAsyncWrite(fifo + 1U, &update, 1U, RxState_e::ACKNOWLEDGE);
      break;
    }

    case RxState_e::ACKNOWLEDGE: {
      const bool frame_valid = async_frame_valid_;
      FinishAsyncReceive();
      if (frame_valid) {
        const uint32_t configured_rx_id = rx_id_;
        rx_id_ = async_frame_.id;
        rx_len_ = async_frame_.dlc;
        std::memcpy(rx_buff_, async_frame_.data, rx_len_);
        if (callback_) {
          callback_(this);
        }
        rx_id_ = configured_rx_id;
      }
      ProcessPending();
      break;
    }

    case RxState_e::IDLE:
    default:
      break;
  }
}

/**
 * @brief 结束异步接收并恢复阻塞 SPI 模式。
 */
void Mcp2518fd_c::FinishAsyncReceive()
{
  rx_state_ = RxState_e::IDLE;
  async_frame_valid_ = false;
  spi_.SetMode(SPI_BLOCK_MODE);
}

/**
 * @brief 将 SPI BSP 完成回调分发到所属 MCP2518FD 对象。
 */
void Mcp2518fd_c::SpiCompleteCallback(SPI_c *spi)
{
  for (uint8_t i = 0U; i < instance_count_; ++i) {
    if (&instances_[i]->spi_ == spi) {
      instances_[i]->HandleSpiComplete();
      return;
    }
  }
}

/**
 * @brief 查询 MCP2518FD 是否初始化成功。
 * @return true 对象可以正常通信。
 * @return false SPI 或控制器初始化失败。
 */
bool Mcp2518fd_c::IsReady() const
{
  return ready_;
}

/**
 * @brief 执行一次 SPI 全双工事务。
 */
bool Mcp2518fd_c::Transfer(uint8_t *tx, uint8_t *rx, uint16_t size)
{
  if (tx == nullptr || rx == nullptr || size == 0U || size > UINT8_MAX) {
    return false;
  }
  spi_.TransRecv(rx, tx, static_cast<uint8_t>(size));
  return true;
}

/**
 * @brief 通过 SPI 指令复位 MCP2518FD。
 */
bool Mcp2518fd_c::Reset()
{
  uint8_t tx[2] = {static_cast<uint8_t>(kCmdReset << 4), 0U};
  uint8_t rx[2]{};
  return Transfer(tx, rx, sizeof(tx));
}

/**
 * @brief 连续读取 MCP2518FD 寄存器或报文 RAM。
 */
bool Mcp2518fd_c::Read(uint16_t address, void *data, uint16_t size)
{
  uint8_t tx[34]{};
  uint8_t rx[34]{};
  if (data == nullptr || size > sizeof(tx) - 2U) {
    return false;
  }
  tx[0] = static_cast<uint8_t>((kCmdRead << 4) | ((address >> 8) & 0x0FU));
  tx[1] = static_cast<uint8_t>(address);
  if (!Transfer(tx, rx, static_cast<uint16_t>(size + 2U))) {
    return false;
  }
  std::memcpy(data, &rx[2], size);
  return true;
}

/**
 * @brief 连续写入 MCP2518FD 寄存器或报文 RAM。
 */
bool Mcp2518fd_c::Write(uint16_t address, const void *data, uint16_t size)
{
  uint8_t tx[34]{};
  uint8_t rx[34]{};
  if (data == nullptr || size > sizeof(tx) - 2U) {
    return false;
  }
  tx[0] = static_cast<uint8_t>((kCmdWrite << 4) | ((address >> 8) & 0x0FU));
  tx[1] = static_cast<uint8_t>(address);
  std::memcpy(&tx[2], data, size);
  return Transfer(tx, rx, static_cast<uint16_t>(size + 2U));
}

/**
 * @brief 读取一个小端格式的 32 位寄存器。
 */
bool Mcp2518fd_c::ReadU32(uint16_t address, uint32_t *value)
{
  uint8_t bytes[4]{};
  if (value == nullptr || !Read(address, bytes, sizeof(bytes))) {
    return false;
  }
  *value = static_cast<uint32_t>(bytes[0]) |
           (static_cast<uint32_t>(bytes[1]) << 8) |
           (static_cast<uint32_t>(bytes[2]) << 16) |
           (static_cast<uint32_t>(bytes[3]) << 24);
  return true;
}

/**
 * @brief 写入一个小端格式的 32 位寄存器。
 */
bool Mcp2518fd_c::WriteU32(uint16_t address, uint32_t value)
{
  const uint8_t bytes[4] = {
      static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
      static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 24)};
  return Write(address, bytes, sizeof(bytes));
}

/**
 * @brief 等待控制器进入指定工作模式。
 */
bool Mcp2518fd_c::WaitMode(uint8_t mode)
{
  const uint32_t started = HAL_GetTick();
  uint8_t status = 0U;
  do {
    if (!Read(kRegC1Con + 2U, &status, 1U)) {
      return false;
    }
    if (((status >> 5) & 0x07U) == mode) {
      return true;
    }
  } while ((HAL_GetTick() - started) < kModeTimeoutMs);
  return false;
}

/**
 * @brief 等待 MCP2518FD 外部晶振稳定。
 */
bool Mcp2518fd_c::WaitOscillator()
{
  const uint32_t started = HAL_GetTick();
  uint32_t osc = 0U;
  do {
    if (!ReadU32(kRegOsc, &osc)) {
      return false;
    }
    if (osc != UINT32_MAX && (osc & kOscReady) != 0U) {
      return true;
    }
  } while ((HAL_GetTick() - started) < kModeTimeoutMs);
  return false;
}

/**
 * @brief 请求并等待控制器切换工作模式。
 */
bool Mcp2518fd_c::SelectMode(uint8_t mode)
{
  uint8_t request = 0U;
  if (!Read(kRegC1Con + 3U, &request, 1U)) {
    return false;
  }
  request = static_cast<uint8_t>((request & ~0x07U) | mode);
  return Write(kRegC1Con + 3U, &request, 1U) && WaitMode(mode);
}

/**
 * @brief 启用 ECC 并初始化全部报文 RAM。
 */
bool Mcp2518fd_c::InitRam()
{
  uint8_t fill[32];
  std::memset(fill, 0xFF, sizeof(fill));
  const uint8_t ecc_enable = kEccEnable;
  if (!Write(kRegEccCon, &ecc_enable, 1U)) {
    return false;
  }
  for (uint16_t offset = 0U; offset < kRamSize; offset += sizeof(fill)) {
    if (!Write(static_cast<uint16_t>(kRamStart + offset), fill,
               sizeof(fill))) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 初始化 MCP2518FD 控制器。
 */
bool Mcp2518fd_c::Initialize()
{
  if (!Reset()) {
    return false;
  }
  HAL_Delay(2U);
  if (!WaitOscillator() || !WaitMode(4U) || !InitRam()) {
    return false;
  }

  uint32_t c1con = 0U;
  if (!ReadU32(kRegC1Con, &c1con)) {
    return false;
  }
  c1con |= 1U << 5;
  c1con &= ~(1U << 19);
  if (!WriteU32(kRegC1Con, c1con) ||
      !WriteU32(kRegC1NbtCfg, 0x000E0303U) ||
      !WriteU32(kRegC1FifoCon + kFifoTx * kFifoStride, 0x07600480U) ||
      !WriteU32(kRegC1FifoCon + kFifoRx * kFifoStride, 0x0F600400U) ||
      !WriteU32(kRegC1FltObj, 0U) ||
      !WriteU32(kRegC1Mask, 1UL << 30)) {
    return false;
  }

  const uint8_t filter_control = 0x80U | kFifoRx;
  const uint8_t rx_event = kRxNotEmpty;
  const uint8_t module_rx_event[2] = {0x02U, 0x00U};
  return Write(kRegC1FltCon, &filter_control, 1U) &&
         Write(kRegC1FifoCon + kFifoRx * kFifoStride, &rx_event, 1U) &&
         Write(kRegC1Int + 2U, module_rx_event, sizeof(module_rx_event)) &&
         SelectMode(kModeClassic);
}

/**
 * @brief 发送一个经典 CAN 标准数据帧。
 */
bool Mcp2518fd_c::SendFrame(uint16_t id, const uint8_t *data, uint8_t length)
{
  const uint16_t fifo = kRegC1FifoCon + kFifoTx * kFifoStride;
  uint8_t status = 0U;
  uint32_t ua = 0U;
  uint8_t object[16]{};
  if (id > 0x7FFU || length > 8U || (length > 0U && data == nullptr) ||
      !Read(fifo + kFifoStatus, &status, 1U) || (status & 0x01U) == 0U ||
      !ReadU32(fifo + kFifoUa, &ua)) {
    return false;
  }

  object[0] = static_cast<uint8_t>(id);
  object[1] = static_cast<uint8_t>(id >> 8);
  object[4] = length;
  if (length > 0U) {
    std::memcpy(&object[8], data, length);
  }
  const uint8_t object_size =
      static_cast<uint8_t>(8U + ((length + 3U) & ~3U));
  if (!Write(static_cast<uint16_t>(kRamStart + (ua & 0x0FFFU)), object,
             object_size)) {
    return false;
  }
  const uint8_t update = kFifoUinc | kFifoTxReq;
  return Write(fifo + 1U, &update, 1U);
}

}  // namespace BSP_n
