/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file haitai_uart.h
 *
 * Haitai ZE300 GDZ 驱动器 UART 透传驱动
 *
 * 架构：micoair h743-v2 → UART(/dev/ttyS3) → UART-CAN 桥接模块 → CAN 总线 → Haitai 电机
 *
 * UART-CAN 桥接模块工作模式：直接 8 字节透传
 *   - haitai_uart 通过 UART 发送 ≤ 8 字节 = CAN Data 字段
 *   - StdID 由桥接模块内固定配置（驱动不感知）
 *
 * 协议版本：自定义 CAN 通信协议 V3.09b0 (2026.03.31)
 *   - 所有 CAN 数据帧为标准帧（11bit StdID）
 *   - 字节序：小端
 *
 * 实现的命令码：
 *   B 系列（参数读写，发送后等待应答）：
 *     0xB1 设置当前位置为原点 → 应答机械角度偏移 (2u)
 *     0xB2 位置模式最大速度 读/设 (4u, 0.01Rpm)
 *     0xB3 最大Q轴电流 读/设 (4u, 0.001A)
 *     0xB6 位置环 Kp 读/设 (4f)
 *     0xB7 位置环 Ki 读/设 (4f)
 *     0xB8 速度环 Kp 读/设 (4f)
 *     0xB9 速度环 Ki 读/设 (4f)
 *   C 系列（高速控制，不等应答）：
 *     0xC2 绝对位置控制 (4s, Count, 16384/圈)
 *     0xC4 最短距离回原点
 *     0xCF 关闭电机输出（电机进入自由态）
 *
 * 跨模块调用约定：
 *   通过 extern "C" __EXPORT 导出 C 接口，外部模块（如 head_pitch_controller）
 *   只需 #include <drivers/haitai_uart/haitai_uart.h> 即可调用。
 *
 * 内部实现位于 namespace haitai_uart。
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  测试入口（保留，用于 UART 通路自检）
 * ============================================================ */

/**
 * 测试入口：向指定 UART 输出一次 "this is haitai_uart driver"
 * @return 成功返回写入字节数，失败返回 -1
 */
int haitai_uart_write_test(const char *device);

/**
 * 测试入口：从指定 UART 轮询读取 5 秒，给 PC 端发数据的时间窗口
 * @return 成功返回实际读取字节数，失败或超时返回 -1
 */
int haitai_uart_read_test(const char *device, char *buf, int buf_len);

/* ============================================================
 *  B 系列：参数读写命令（发送后等待应答）
 *  所有函数返回 0 成功，< 0 失败
 * ============================================================ */

/**
 * 0xB1: 设置当前位置为原点（断电保存）
 * 应答约 35ms 延迟（若启用第二编码器）
 * @param[out] mech_offset  机械角度偏移（uint16, 小端）
 */
int haitai_uart_cmd_b1_zero_origin(const char *device, uint16_t *mech_offset);

/**
 * 0xB2: 读取位置模式旋转最大速度
 * @param[out] value  速度值（uint32, 单位 0.01Rpm）
 */
int haitai_uart_cmd_b2_max_speed_get(const char *device, uint32_t *value);

/**
 * 0xB2: 设置位置模式旋转最大速度（断电不保存）
 * @param[in]  value  速度值（uint32, 单位 0.01Rpm），例如 3000 = 30.00 Rpm
 */
int haitai_uart_cmd_b2_max_speed_set(const char *device, uint32_t value);

/**
 * 0xB3: 读取位置/速度模式最大 Q 轴电流
 * @param[out] value  电流值（uint32, 单位 0.001A）
 */
int haitai_uart_cmd_b3_max_qcurrent_get(const char *device, uint32_t *value);

/**
 * 0xB3: 设置位置/速度模式最大 Q 轴电流（断电不保存）
 * @param[in] value  电流值（uint32, 单位 0.001A），例如 5000 = 5.000 A
 */
int haitai_uart_cmd_b3_max_qcurrent_set(const char *device, uint32_t value);

/**
 * 0xB6: 读取位置控制闭环 Kp
 * @param[out] value  Kp（float, 小端）
 */
int haitai_uart_cmd_b6_pos_kp_get(const char *device, float *value);

/**
 * 0xB6: 设置位置控制闭环 Kp（断电不保存）
 */
int haitai_uart_cmd_b6_pos_kp_set(const char *device, float value);

/**
 * 0xB7: 读取位置控制闭环 Ki
 */
int haitai_uart_cmd_b7_pos_ki_get(const char *device, float *value);

/**
 * 0xB7: 设置位置控制闭环 Ki（断电不保存）
 */
int haitai_uart_cmd_b7_pos_ki_set(const char *device, float value);

/**
 * 0xB8: 读取速度控制闭环 Kp
 */
int haitai_uart_cmd_b8_vel_kp_get(const char *device, float *value);

/**
 * 0xB8: 设置速度控制闭环 Kp（断电不保存）
 */
int haitai_uart_cmd_b8_vel_kp_set(const char *device, float value);

/**
 * 0xB9: 读取速度控制闭环 Ki
 */
int haitai_uart_cmd_b9_vel_ki_get(const char *device, float *value);

/**
 * 0xB9: 设置速度控制闭环 Ki（断电不保存）
 */
int haitai_uart_cmd_b9_vel_ki_set(const char *device, float value);

/* ============================================================
 *  C 系列：高速控制命令（不等应答）
 *  返回 0 = UART 发送成功；不保证电机实际收到/执行
 * ============================================================ */

/**
 * 0xC2: 绝对值位置控制
 * @param position  目标绝对位置（int32, Count, 电机一圈 = 16384 Count）
 */
int haitai_uart_cmd_c2_abs_pos(const char *device, int32_t position);

/**
 * 0xC4: 最短距离回原点（旋转角度 ≤ 180°）
 */
int haitai_uart_cmd_c4_return_origin(const char *device);

/**
 * 0xCF: 关闭电机输出（电机进入自由态，上电后默认即此状态）
 */
int haitai_uart_cmd_cf_disable(const char *device);

/* ============================================================
 *  快速路径：持久 fd（供 400Hz 流式控制使用）
 *
 * 上面各命令 API 每次调用都 open → 发送 →（等应答）→ close，
 * 且发送后有 2ms 排空等待，无法满足 400Hz 循环调用。
 * 高频调用方（如 head_pitch_controller）应使用这组接口：
 *   haitai_uart_start()        打开一次，持有 fd
 *   haitai_uart_send_c2_fast() 每周期只做一次 5 字节 write
 *   haitai_uart_stop()         停止时释放
 * ============================================================ */

/**
 * 打开 UART 并完成 termios 配置（115200 8N1，CLOCAL，非阻塞）
 * @return 成功返回 fd（>= 0），失败返回负 errno
 */
int haitai_uart_start(const char *device);

/**
 * 快速发送 0xC2 绝对位置命令（5 字节 = DLC5：0xC2 + int32 小端）
 * 只做一次 write，无 sleep、无 open/close，适合 400Hz 循环调用。
 * 115200 波特下 5 字节约 0.43ms 由 TX 中断排空，2.5ms 周期内不会积压。
 * @return 0 成功，< 0 失败（fd 无效或写入失败）
 */
int haitai_uart_send_c2_fast(int fd, int32_t position);

/**
 * 关闭持久 fd（内部先等 TX FIFO 排空再 close）
 * @return 0 成功，< 0 失败
 */
int haitai_uart_stop(int fd);

#ifdef __cplusplus
}
#endif

/* ============================================================
 *  C++ 命名空间内部接口（驱动自身使用，外部模块不应直接调用）
 * ============================================================ */

namespace haitai_uart
{

/* Haitai CAN 自定义协议命令码 */
constexpr uint8_t CMD_B1_ZERO_ORIGIN      = 0xB1;
constexpr uint8_t CMD_B2_MAX_SPEED       = 0xB2;
constexpr uint8_t CMD_B3_MAX_QCURRENT    = 0xB3;
constexpr uint8_t CMD_B6_POS_KP          = 0xB6;
constexpr uint8_t CMD_B7_POS_KI          = 0xB7;
constexpr uint8_t CMD_B8_VEL_KP          = 0xB8;
constexpr uint8_t CMD_B9_VEL_KI          = 0xB9;
constexpr uint8_t CMD_C2_ABS_POS         = 0xC2;
constexpr uint8_t CMD_C4_RETURN_ORIGIN  = 0xC4;
constexpr uint8_t CMD_CF_DISABLE         = 0xCF;

/* UART 基础设施 */
int initialise_uart(const char *const device, int &uart_fd);
int deinitialise_uart(int &uart_fd);
int write_test(int uart_fd);
int read_test(int uart_fd, char *buf, int buf_len);

/* CAN 协议层工具函数 */

/**
 * 发送一个 CAN 数据帧到 UART
 * 按协议 DLC 实际字节数发送（1 + data_len，不补 0 到 8 字节）：
 *   无数据命令（B1/C4/CF、读 B2-B9）发 1 字节；带 4 字节参数命令发 5 字节。
 * 内部在 write 后有 2ms 排空等待，不适合高频循环调用（高频场景用快速路径 API）。
 * @param uart_fd   已打开的 UART fd
 * @param cmd       命令码（放在 [0]）
 * @param data      数据载荷指针（[1..N]），无数据传 nullptr
 * @param data_len  数据载荷字节数（0~7）
 * @return 实际写入 UART 的字节数，< 0 失败
 */
int send_frame(int uart_fd, uint8_t cmd, const uint8_t *data, uint8_t data_len);

/**
 * 等待电机应答帧
 * @param uart_fd         已打开的 UART fd（必须是非阻塞模式）
 * @param expected_cmd    期望的应答命令码（应答帧 [0] 必须等于此值）
 * @param resp_data       输出缓冲区，存放应答数据字段 [1..N]
 * @param resp_data_len   期望的应答数据字节数（不含 [0] 命令字节）
 *                        B1 = 2，B2/B3/B6/B7/B8/B9 = 4
 * @param timeout_ms      超时毫秒，B1 推荐 100ms，其它 50ms
 * @return 0 成功（resp_data 已填充），< 0 失败/超时
 */
int wait_response(int uart_fd, uint8_t expected_cmd, uint8_t *resp_data,
		  uint8_t resp_data_len, int timeout_ms);

} /* namespace haitai_uart */
