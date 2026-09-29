# HaiTai UART 电机驱动（haitai_uart）

海泰（HaiTai）HT-J-2E 系列电机串口驱动（GDZ 驱动器 ZE300）。

## 数据链路

```
飞控 UART ──> UART转CAN 桥接模块 ──> CAN 总线 ──> 海泰电机
```

- CAN ID 保存在 UART转CAN 桥接模块中，本驱动仅通过 UART 发送 CAN 数据场
- 每帧最多 8 字节（数据场首字节为命令码，多字节字段小端字节序）
- 协议参考：《自定义CAN通信协议_V3.09b0》（海泰GDZ驱动器ZE300 用户资料包）

## 支持的命令码

| 命令码 | 方法 | 说明 | 应答 |
|---|---|---|---|
| 0x00 | `restart()` | 重启从机 | 无 |
| 0xA0 | `readVersion()` | 读 Boot/软件/硬件/协议版本 | 有 |
| 0xA1 | `readQAxisCurrent()` | 读实时 Q 轴电流 [A] | 有 |
| 0xA2 | `readSpeed()` | 读实时旋转速度 [RPM] | 有 |
| 0xA4 | `readStatus()` | 读温度/Q 轴电流/速度/单圈角度 | 有 |
| 0xAE | `readRuntimeStatus()` | 读母线电压/电流/温度/模式/故障码 | 有 |
| 0xAF | `clearFault()` | 清除故障，返回剩余故障码 | 有 |
| 0xB0 | `readMotorParams()` | 读力矩常数 [N/A] | 有 |
| 0xB1 | `setOriginHere()` | 设置当前位置为原点 | 有 |
| 0xC0 | `setTorque()` / `sendTorque()` | Q 轴电流（力矩）控制 | 有 |
| 0xC4 | `goHome()` | 最短距离回原点（≤180°） | 有 |

## 启动方式

### 方式一：命令行（调试）

```bash
# NSH / MAVLink console 中
haitai_uart start /dev/ttyS1                # 默认波特率 115200
haitai_uart start /dev/ttyS1 SER_TEL1_BAUD   # 波特率从串口参数读取
haitai_uart status                           # 查看事务统计与最近读取的电机状态
haitai_uart stop                             # 停止（注意：先停调用方模块）
```

支持的波特率：9600 / 19200 / 38400 / 57600 / 115200 / 230400 / 460800 / 921600。
启动时同步初始化串口并探测链路（读版本）；串口打开失败则拒绝启动。

### 方式二：串口功能自动启动（正式部署）

module.yaml 定义了串口功能参数 `HTU_SER_CFG`（参数组 HaiTai）。将飞控某串口
（如 TELEMETRY 1）的 `SER_TEL1_CONFIG` 设为该功能后，启动脚本自动执行
`haitai_uart start ${SERIAL_DEV} ${BAUD_PARAM}`。

> 实机部署需在目标板 px4board 中启用 `CONFIG_DRIVERS_HAITAI_UART=y`。

## 驱动架构：纯被动服务

- **驱动自身不做任何周期性 UART 轮询**。`Run()` 仅以 100ms 心跳维护
  `stop` 命令响应，所有命令事务均由调用方在其上下文中发起。
- 由此 UART 访问天然串行，无需加锁；但须保证 **同一时刻只有一个上下文
  调用命令接口**（典型：单一控制环先 `sendTorque()` 再低频 `readStatus()`），
  多模块并发调用会破坏应答帧同步。
- 帧同步机制：每个事务开头 `tcflush` 丢弃残留数据（含 fire-and-forget
  迟到的应答），应答首字节须为命令码回显，否则重新同步。

## 其它模块集成

控制器模块（如 `head_pitch_controller`）通过 `HaiTaiUART::get_instance()`
获取运行中的驱动实例：

```cpp
#include "HaiTaiUART.hpp"   // 需在模块 CMakeLists 中 DEPENDS drivers__haitai_uart

HaiTaiUART *motor = HaiTaiUART::get_instance();
```

### 高频控制（400Hz 及以上）推荐用法

控制器挂高优先级工作队列（如 `rate_ctrl`），`ScheduleOnInterval(2500)`：

```cpp
// 启动时：按链路实测往返延时缩短同步应答超时（默认 20ms）
motor->setResponseTimeoutUs(3000);

// Run() 中（400Hz）：
motor->sendTorque(u);            // fire-and-forget，仅一次 UART 写
                                 // 115200 波特率下 5 字节约 0.43ms

if (++_counter % 20 == 0) {      // 反馈低频同步读取（20Hz 足够）
	HaiTaiUART::StatusInfo st{};
	motor->readStatus(st);       // 返回值 PX4_ERROR 表示本轮读取失败，不影响控制
}
```

接口分两类：

| 接口 | 行为 | 适用场景 |
|---|---|---|
| `sendTorque(target_a)` | 发送后不等待应答，立即返回 | 高频控制环 |
| `setTorque(target_a, measured_a)` | 同步等待应答（可读回实测电流） | 低频调用/诊断 |
| 其余 `read*` / `clearFault` 等 | 同步等待应答，阻塞至超时 | 低频/初始化 |

### 时序估算（115200 波特率）

- 0xC0 帧 5 字节 ≈ 0.43ms 发送时间；提高波特率（460800/921600）可降至 0.1ms 级
- CAN 总线占用：400Hz × 约 240 bits @ 1Mbps ≈ 10%，安全
- 同步读取一次的往返时间 ≈ 发送 + 桥接转发 + 电机处理 + 应答返回，实测后据此
  设置 `setResponseTimeoutUs()`

## 停止顺序

先停止调用方模块（控制器），再 `haitai_uart stop`。
驱动被删除时若调用方仍在访问 UART，会产生未定义行为。

## 参考

- 协议：《自定义CAN通信协议_3.09b0》（海泰GDZ驱动器ZE300 用户资料包，
  `2.自定义CAN通信协议` 目录）
- 架构参考：`src/drivers/roboclaw`（ModuleBase + ScheduledWorkItem 模式）
- 注意：从机检测到故障（通信故障除外）时会以 200ms 周期主动上报 0xAE 内容；
  纯被动模式下这些上报帧无人接收，会被事务开头的 `tcflush` 丢弃。
