#ifndef CLAI_ECF_MCP2518FD_HPP
#define CLAI_ECF_MCP2518FD_HPP

/**
 * @file mcp2518fd.hpp
 * @brief 基于 SPI 的 MCP2518FD 通信驱动。
 *
 * @verbatim
 * BSP_n::Mcp2518fd_c spi_can(
 *     &hspi1, MCP_CS_GPIO_Port, MCP_CS_Pin,
 *     MCP_INT_GPIO_Port, MCP_INT_Pin, 0x0E, 0x10E);
 *
 * void HAL_GPIO_EXTI_Callback(uint16_t pin)
 * {
 *     BSP_n::Mcp2518fd_c::ExtiCallback(pin);
 * }
 * @endverbatim
 *
 * @attention MCP INT 引脚需配置为下降沿外部中断，SPI 接收和发送需配置
 * DMA 通道及对应中断。后续接收和协议解析由 SPI DMA 完成回调自动执行。
 */

#include "CAN/bsp_can_template.hpp"
#include "SPI/bsp_spi.hpp"

#include <cstdint>
#include <functional>

namespace BSP_n {

class Mcp2518fd_c;
using Mcp2518fdRxCallback_t = std::function<void(Mcp2518fd_c *)>;

/**
 * @brief 基于 SPI 和 MCP2518FD 的 CAN 通信对象。
 */
class Mcp2518fd_c final : public CanBase_c {
 public:
  /**
   * @brief 构造并初始化一个 MCP2518FD 通信对象。
   * @param spi_handle STM32 HAL SPI 句柄。
   * @param cs_port 片选信号 GPIO 端口。
   * @param cs_pin 片选信号 GPIO 引脚。
   * @param int_port 中断信号 GPIO 端口。
   * @param int_pin 中断信号 GPIO 引脚。
   * @param tx_id 发送报文 ID。
   * @param rx_id 接收报文 ID。
   * @param callback 收到匹配报文时执行的实例回调。
   */
  Mcp2518fd_c(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_port,
              uint16_t cs_pin, GPIO_TypeDef *int_port, uint16_t int_pin,
              uint32_t tx_id, uint32_t rx_id,
              Mcp2518fdRxCallback_t callback = {});

  /**
   * @brief 将 HAL GPIO 外部中断按引脚分发给已注册的 MCP2518FD 实例。
   * @param pin 触发中断的 GPIO 引脚。
   */
  static void ExtiCallback(uint16_t pin);

  /**
   * @brief 设置发送数据长度。
   * @param length 数据长度，范围为 0～8 字节。
   */
  void SetDLC(uint32_t length) override;

  /**
   * @brief 发送 tx_buff_ 中的经典 CAN 标准帧。
   * @param timeout_ms 接口兼容参数，SPI BSP 内部使用固定有限超时。
   * @return CAN_OK 报文提交成功。
   * @return CAN_ERROR 外设未就绪或发送失败。
   */
  CanState_e Transmit(float timeout_ms) override;

  /**
   * @brief 设置当前对象的接收回调。
   * @param callback 收到报文时执行的实例回调，由上层协议筛选 ID。
   */
  void SetRxCallBack(Mcp2518fdRxCallback_t callback);

  /**
   * @brief 查询 MCP2518FD 是否初始化成功。
   * @return true 对象可以正常通信。
   * @return false SPI 或控制器初始化失败。
   */
  bool IsReady() const;

 private:
  enum class RxState_e : uint8_t {
    IDLE,
    READ_STATUS,
    READ_ADDRESS,
    READ_OBJECT,
    ACKNOWLEDGE
  };

  struct Frame_s {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
  };

  /** @brief 初始化 MCP2518FD 控制器。 */
  bool Initialize();
  /** @brief 执行一次 SPI 全双工事务。 */
  bool Transfer(uint8_t *tx, uint8_t *rx, uint16_t size);
  /** @brief 通过 SPI 指令复位 MCP2518FD。 */
  bool Reset();
  /** @brief 连续读取 MCP2518FD 寄存器或报文 RAM。 */
  bool Read(uint16_t address, void *data, uint16_t size);
  /** @brief 连续写入 MCP2518FD 寄存器或报文 RAM。 */
  bool Write(uint16_t address, const void *data, uint16_t size);
  /** @brief 读取一个小端格式的 32 位寄存器。 */
  bool ReadU32(uint16_t address, uint32_t *value);
  /** @brief 写入一个小端格式的 32 位寄存器。 */
  bool WriteU32(uint16_t address, uint32_t value);
  /** @brief 等待控制器进入指定工作模式。 */
  bool WaitMode(uint8_t mode);
  /** @brief 等待 MCP2518FD 外部晶振稳定。 */
  bool WaitOscillator();
  /** @brief 请求并等待控制器切换工作模式。 */
  bool SelectMode(uint8_t mode);
  /** @brief 启用 ECC 并初始化全部报文 RAM。 */
  bool InitRam();
  /** @brief 发送一个经典 CAN 标准数据帧。 */
  bool SendFrame(uint16_t id, const uint8_t *data, uint8_t length);
  /** @brief 启动一次异步 SPI 读取事务。 */
  void StartAsyncRead(uint16_t address, uint8_t size, RxState_e next_state);
  /** @brief 启动一次异步 SPI 写入事务。 */
  void StartAsyncWrite(uint16_t address, const uint8_t *data, uint8_t size,
                       RxState_e next_state);
  /** @brief 处理 SPI DMA 完成事件并推进 MCP 接收流程。 */
  void HandleSpiComplete();
  /** @brief 记录 MCP INT 事件并在 SPI 空闲时启动接收。 */
  void HandleInterrupt();
  /** @brief 启动 SPI DMA 接收，完成后由 SPI 回调解析并分发报文。 */
  void ProcessPending();
  /** @brief 结束异步接收并恢复阻塞 SPI 模式。 */
  void FinishAsyncReceive();
  /** @brief 将 SPI BSP 完成回调分发到所属 MCP2518FD 对象。 */
  static void SpiCompleteCallback(SPI_c *spi);

  SPI_c spi_;
  SPI_HandleTypeDef *spi_handle_;
  GPIO_TypeDef *int_port_;
  uint16_t int_pin_;
  Mcp2518fdRxCallback_t callback_{};
  uint8_t tx_len_ = 8U;
  bool ready_ = false;
  RxState_e rx_state_ = RxState_e::IDLE;
  uint8_t async_tx_[34]{};
  uint8_t async_rx_[34]{};
  Frame_s async_frame_{};
  bool async_frame_valid_ = false;
  volatile bool interrupt_pending_ = false;
  static Mcp2518fd_c *instances_[MX_SPI_BUS_SLAVE_CNT];
  static uint8_t instance_count_;
};

}  // namespace BSP_n

#endif
