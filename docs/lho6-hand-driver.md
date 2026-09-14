# LHO6 灵巧手驱动接入记录

这份文档会随实现逐步追加，不把所有内容一次写完。
当前对应 `clai_ecf` 中的 LHO6 协议驱动，不覆盖 `MotorCtrl` 的 USB 和 50 Hz 调度。

## 原则

- 驱动本身用新文件，不把灵巧手协议塞进 `dm_driver_core`。
- 现有文件只改接入点，不改达妙电机逻辑。
- USB `0x17/0x18`、左右手编排、50 Hz 错峰仍留在 `MotorCtrl`。
- 关节数值保持协议原值 `0x00..0xFF`，库内不做物理量换算。

## 步骤总览

- [x] 第 1 步：新增协议骨架和入口头文件
- [x] 第 2 步：发送路径走抽象通信对象
- [x] 第 3 步：接收解析与写回显过滤
- [ ] 第 4 步：接入 `MotorCtrl`（下一轮再补）

## 现有文件改动

本轮只动了这些旧文件：

| 文件 | 为什么改 |
| --- | --- |
| `Bsp/CAN/bsp_can_template.hpp` | 把 `rx_len_` 放到 public，并默认初始化为 0。LHO6 读命令是 1 字节、写关节是 7 字节，解析必须知道真实 DLC。 |
| `CMakeLists.txt` | 把 `lho6_driver_core.cpp` 和 `LHO6Hand` 头文件路径加进库目标。 |
| `README.md` | 补目录说明。 |
| `docs/decoupled-architecture-learning.md` | 加一份指向本文的入口。 |

没有修改：

- `MotorCore/dm_driver_core.*`
- `DMMotor/dm_driver.hpp`
- `MotorCtrl` 工程里的旧 `LHO6_Hand`

## 第 1 步：协议骨架

新增文件：

```text
LHO6Hand/lho6_driver.hpp
MotorCore/lho6_driver_core.hpp
MotorCore/lho6_driver_core.cpp
```

入口头文件只做转发，和达妙一样：

```cpp
#include "MotorCore/lho6_driver_core.hpp"
```

`Lho6Hand_c` 的构造函数按达妙方式吃通信对象：

```cpp
template <typename CanType>
Lho6Hand_c(CanType& can, uint32_t can_id, Lho6HandSide_e side);
```

通信对象需要提供：

```text
SetRxCallBack()
SetDLC()
Transmit()
tx_id_ / rx_id_ / tx_buff_ / rx_buff_ / rx_len_
```

LHO6 收发共用同一个标准帧 ID：

| 手 | CAN ID |
| --- | --- |
| 右手 | `0x27` |
| 左手 | `0x28` |

因此创建 `Fdcan_c` 时 `tx_id == rx_id`。

## 第 2 步：发送路径

旧实现直接调用：

```cpp
HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx, payload);
```

新实现改成内部发送闭包：

```text
SetPosition / ReadSpeed / SaveParams
        ↓
SendRaw()
        ↓
send_frame_(can_id, data, len)
        ↓
can.SetDLC(len)
can.Transmit()
```

当前发送约定：

| API | DLC | payload |
| --- | --- | --- |
| `SetPosition()` | 7 | `0x01` + 6 关节 |
| `ReadSpeed()` | 1 | `0x05` |
| `ReadPressure(i, size)` | 2 | `0xB1+i` + `size` |
| `SaveParams()` | 8 | 8 个 `0xCF` |

`SetPosition()` 发送成功后会置位 `position_write_pending_`，供下一步过滤写回显。

## 第 3 步：接收解析

`Analysis()` 只处理 `id == CAN_ID` 的帧，并按真实 `len` 解析。

已经覆盖：

- 位置 / 速度 / 力矩限制 / 堵转参数
- 写位置回显过滤：回包若与目标完全一致，不覆盖真实位置
- 压力矩阵分片
- 工厂编码分片
- `UpdateOnlineStatus()` 超时离线

本轮没有把 `PackLho6HandsFeedback()` 放进库。那是 USB 36 字节打包，属于 `MotorCtrl` 通信层。

## 如何验证

主机单测不依赖 STM32 HAL，也不依赖真实 CAN：

```bash
g++ -std=c++17 -I. tests/test_lho6_hand.cpp MotorCore/lho6_driver_core.cpp -o /tmp/test_lho6_hand
/tmp/test_lho6_hand
```

测试文件：`tests/test_lho6_hand.cpp`。

## 第 4 步：下一步

下一轮再补这些内容，并追加到本文：

1. `MotorCtrl` 用 `Fdcan_c` 替换 `new Lho6Hand_c(&hfdcan1, ...)`。
2. `MotorManage` 继续负责左右手编排和 50 Hz 调度。
3. USB `LHO6_HAND_INFO` / `LHO6_HAND_CONTROL` 仍留在 `CommunicationFifo`。
4. 确认 `Fdcan_c` 过滤器能收到 `0x27` / `0x28`。

`MotorCtrl` 侧预期用法：

```cpp
static BSP_n::Fdcan_c left_hand_can(&hfdcan1, LHO6_LEFT_CAN_ID, LHO6_LEFT_CAN_ID, FDCAN_STANDARD_ID);
LeftHand = new Lho6Hand_c(left_hand_can, LHO6_LEFT_CAN_ID, Lho6HandSide_e::LEFT);
```
