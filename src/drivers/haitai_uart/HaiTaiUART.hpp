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
 * @file HaiTaiUART.hpp
 *
 * 海泰（HaiTai）HT-J-2E 系列电机串口驱动
 *
 * 数据链路：飞控 UART -> UART转CAN 桥接模块 -> CAN 总线 -> 海泰电机
 * CAN ID 已保存在 UART转CAN 桥接模块中，本驱动仅通过 UART 发送 CAN 数据场
 * （每帧最多 8 字节，小端字节序），无需关心 CAN ID。
 *
 * 协议参考：《自定义CAN通信协议_V3.09b0》（海泰GDZ驱动器ZE300）
 *
 * 组织方式参考 src/drivers/roboclaw（ModuleBase 提供命令行入口）。
 *
 * 纯被动服务设计：
 *   - 驱动自身不做任何周期性 UART 轮询，Run() 仅以低频心跳维护
 *     stop 命令响应；所有命令事务均由调用方在其上下文中发起；
 *   - 由此 UART 访问天然串行，无需加锁；但须保证同一时刻只有一个
 *     上下文调用命令接口（典型：单一控制环先 sendTorque() 再低频
 *     readStatus()），多模块并发调用会破坏应答帧同步；
 *   - 高频控制（如 400Hz）：使用 sendTorque()（发送后不等应答，
 *     迟到的应答由下个事务开头的 tcflush 清除），反馈用低频同步
 *     读取，并可按需通过 setResponseTimeoutUs() 缩短应答超时。
 *
 * 其它模块（如 head_pitch_controller）可通过 HaiTaiUART::get_instance()
 * 获取运行中的驱动实例，直接调用下述命令接口。
 * 注意停止顺序：先停调用方模块，再 haitai_uart stop。
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <sys/select.h>
#include <termios.h>

#include <stdint.h>

class HaiTaiUART : public ModuleBase<HaiTaiUART>, public px4::ScheduledWorkItem
{
public:
	// ============================ 数据结构定义 ============================

	/** 版本信息（0xA0 应答） */
	struct VersionInfo {
		uint16_t boot_version;
		uint16_t app_version;
		uint16_t hardware_version;
		uint8_t  protocol_version;
	};

	/** 电机实时状态（0xA4 应答） */
	struct StatusInfo {
		uint8_t temperature_c;   // 工作温度 [℃]
		float   q_current_a;     // Q 轴电流 [A]
		float   speed_rpm;       // 旋转速度 [RPM]
		float   angle_deg;       // 单圈绝对值角度 [deg]
	};

	/** 电机运行状态（0xAE 应答） */
	struct RuntimeStatusInfo {
		float    bus_voltage_v;  // 母线电压 [V]
		float    bus_current_a;  // 母线电流 [A]
		uint8_t  temperature_c;  // 工作温度 [℃]
		uint8_t  run_mode;       // 0:关闭 1:电压 2:Q轴电流 3:速度 4:位置
		uint8_t  fault_code;     // 故障码（位定义见 printFault）
	};

	/** 电机参数（0xB0 应答） */
	struct MotorParamsInfo {
		float torque_constant_na;  // 力矩常数 [N/A]
	};

	/**
	 * @param device_name          串口设备路径，如 "/dev/ttyS1"
	 * @param baud_rate_parameter  保存波特率的参数名（如 "SER_TEL1_BAUD"），
	 *                             传 nullptr 时使用默认波特率 115200
	 */
	HaiTaiUART(const char *device_name, const char *baud_rate_parameter);
	~HaiTaiUART() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	// ============================== 协议命令接口 ==============================

	/** 重启从机（命令码 0x00，从机不应答） */
	int restart();

	/** 读取 Boot/软件/硬件/CAN 协议版本（命令码 0xA0） */
	int readVersion(VersionInfo &version);

	/** 读取实时 Q 轴电流 [A]（命令码 0xA1，应答单位 0.001A） */
	int readQAxisCurrent(float &current_a);

	/** 读取实时旋转速度 [RPM]（命令码 0xA2，应答单位 0.01Rpm） */
	int readSpeed(float &speed_rpm);

	/** 读取温度/Q 轴电流/速度/单圈绝对值角度（命令码 0xA4） */
	int readStatus(StatusInfo &status);

	/**
	 * 读取母线电压/母线电流/温度/运行模式/故障码（命令码 0xAE）
	 * 注：从机检测到故障（除通信故障外）时也会以 200ms 周期主动上报本应答内容
	 */
	int readRuntimeStatus(RuntimeStatusInfo &runtime_status);

	/** 清除故障（命令码 0xAF），remaining_fault 返回清除后剩余故障码 */
	int clearFault(uint8_t &remaining_fault);

	/** 读取力矩常数等电机参数（命令码 0xB0，力矩 = 力矩常数 * Q 轴电流） */
	int readMotorParams(MotorParamsInfo &params);

	/** 设置当前位置为原点（命令码 0xB1），mechanical_offset 返回机械角度偏移 */
	int setOriginHere(uint16_t &mechanical_offset);

	/**
	 * Q 轴电流（力矩）控制（命令码 0xC0）
	 * @param target_a    目标 Q 轴电流 [A]
	 * @param measured_a  可选，返回应答中的实测 Q 轴电流 [A]
	 * 注：同步等待应答，适合低频调用
	 */
	int setTorque(float target_a, float *measured_a = nullptr);

	/**
	 * Q 轴电流（力矩）控制，发送后不等待应答（命令码 0xC0）
	 * 高频控制（如 400Hz）专用：单次调用仅占用一次 UART 写时间
	 * （115200 波特率下 5 字节约 0.43ms）；迟到的应答由下个事务
	 * 开头的 tcflush 丢弃，不会破坏帧同步
	 */
	int sendTorque(float target_a);

	/**
	 * 设置同步应答超时（默认 20ms）。400Hz 控制环内做低频反馈读取时，
	 * 建议按链路实际往返延时缩短（如 2~5ms），避免超时阻塞拖慢控制周期
	 */
	void setResponseTimeoutUs(uint32_t timeout_us) { _response_timeout_us = timeout_us; }

	/**
	 * 最短距离回原点，旋转角度不大于 180 度（命令码 0xC4）
	 * 应答返回单圈/多圈绝对值角度 [deg]
	 */
	int goHome(float &single_turn_deg, float &multi_turn_deg);

private:
	/** 心跳：仅处理 stop 命令，不做任何 UART 事务（纯被动设计） */
	void Run() override;

	/** 协议命令码 */
	enum class Command : uint8_t {
		Restart     = 0x00, // 重启从机（不应答）
		ReadVersion = 0xA0, // 读 Boot/软件/硬件/CAN 协议版本
		ReadCurrent = 0xA1, // 读实时 Q 轴电流
		ReadSpeed   = 0xA2, // 读实时旋转速度
		ReadStatus  = 0xA4, // 读温度/Q 轴电流/速度/单圈角度
		ReadRuntime = 0xAE, // 读母线电压/电流/温度/模式/故障码
		ClearFault  = 0xAF, // 清除故障
		ReadParams  = 0xB0, // 读力矩常数
		SetOrigin   = 0xB1, // 设置当前位置为原点
		SetTorque   = 0xC0, // Q 轴电流（力矩）控制
		GoHome      = 0xC4, // 最短距离回原点
	};

	static constexpr uint8_t CAN_FRAME_MAX_LEN = 8;      // CAN 数据场最大长度
	static constexpr uint32_t DEFAULT_RESPONSE_TIMEOUT_US = 20000; // 同步应答默认超时 20ms
	static constexpr uint64_t HEARTBEAT_INTERVAL_US = 100000;      // 心跳周期（仅维护 stop 响应）
	static constexpr int DEFAULT_BAUD_RATE = 115200;    // 默认波特率

	// ---- 协议收发底层 ----
	int initializeUART();

	/**
	 * 发送一帧命令（请求-应答式事务）
	 * @param payload       命令码之后的数据场（可为 nullptr）
	 * @param payload_len   数据场长度（不含命令码，<= 7）
	 * @param response      应答缓冲区（>= 8 字节）；传 nullptr 时读取并丢弃应答
	 * @param wait_response true 同步等待并读取应答；false 发送后立即返回
	 * @return 等待应答时返回应答字节数（失败为 PX4_ERROR，即 -1）；
	 *         不等待应答时成功返回发送字节数（>0），失败返回 PX4_ERROR
	 */
	int transaction(Command cmd, const uint8_t *payload, uint8_t payload_len, uint8_t *response,
			bool wait_response = true);

	/** 读取并同步应答帧（首字节为命令码回显），返回实际读取长度 */
	int readResponse(uint8_t cmd, uint8_t *response, uint8_t expected_len);

	/** 各命令码对应的应答帧长度 */
	static uint8_t expectedResponseLength(Command cmd);

	/** 打印故障码位含义 */
	static void printFault(uint8_t fault);

	// ---- 小端编解码 ----
	static void     encodeInt32(uint8_t *buf, int32_t value);
	static uint16_t decodeUint16(const uint8_t *buf);
	static int16_t  decodeInt16(const uint8_t *buf);
	static int32_t  decodeInt32(const uint8_t *buf);
	static float    decodeFloat(const uint8_t *buf);

	char _device_name[32];
	char _baud_rate_parameter[24];

	// ---- UART ----
	int _uart_fd{-1};
	fd_set _uart_fd_set;

	// ---- 调用统计与最近一次读取缓存（供 print_status 查看） ----
	// 注：由调用方上下文更新，status 命令读取，仅用于诊断
	uint32_t _response_timeout_us{DEFAULT_RESPONSE_TIMEOUT_US};
	StatusInfo _last_status{};
	RuntimeStatusInfo _last_runtime{};
	bool _last_status_valid{false};
	bool _last_runtime_valid{false};
	uint32_t _transaction_count{0};
	uint32_t _transaction_failures{0};
};
