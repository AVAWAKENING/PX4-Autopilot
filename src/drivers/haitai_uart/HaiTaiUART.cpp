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
 * @file HaiTaiUART.cpp
 *
 * 海泰（HaiTai）HT-J-2E 系列电机串口驱动实现
 *
 * 协议参考：《自定义CAN通信协议_V3.09b0》（海泰GDZ驱动器ZE300）
 * 帧格式：命令码在数据场首字节，多字节字段小端字节序；
 * 应答帧首字节为命令码回显（0x00 重启命令除外，无应答）。
 */

#include "HaiTaiUART.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <parameters/param.h>

#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

using namespace time_literals;

// 心跳挂在低优先级工作队列（高频控制事务由调用方在自己队列的上下文中发起）
HaiTaiUART::HaiTaiUART(const char *device_name, const char *baud_rate_parameter) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	strncpy(_device_name, device_name, sizeof(_device_name) - 1);
	_device_name[sizeof(_device_name) - 1] = '\0';

	if (baud_rate_parameter != nullptr) {
		strncpy(_baud_rate_parameter, baud_rate_parameter, sizeof(_baud_rate_parameter) - 1);
		_baud_rate_parameter[sizeof(_baud_rate_parameter) - 1] = '\0';

	} else {
		_baud_rate_parameter[0] = '\0';
	}
}

HaiTaiUART::~HaiTaiUART()
{
	ScheduleClear();

	if (_uart_fd >= 0) {
		close(_uart_fd);
		_uart_fd = -1;
	}
}

// ============================== 命令接口实现 ==============================

int HaiTaiUART::restart()
{
	// 0x00: 重启从机，[1]-[7] 固定填充 FF 00 FF 00 FF 00 FF，不应答
	const uint8_t payload[7] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF};
	return transaction(Command::Restart, payload, sizeof(payload), nullptr, false);
}

int HaiTaiUART::readVersion(VersionInfo &version)
{
	uint8_t response[CAN_FRAME_MAX_LEN] {};

	if (transaction(Command::ReadVersion, nullptr, 0, response) != 8) {
		return PX4_ERROR;
	}

	version.boot_version     = decodeUint16(&response[1]);
	version.app_version      = decodeUint16(&response[3]);
	version.hardware_version = decodeUint16(&response[5]);
	version.protocol_version = response[7];

	return PX4_OK;
}

int HaiTaiUART::readQAxisCurrent(float &current_a)
{
	uint8_t response[CAN_FRAME_MAX_LEN] {};

	if (transaction(Command::ReadCurrent, nullptr, 0, response) != 5) {
		return PX4_ERROR;
	}

	current_a = decodeInt32(&response[1]) * 0.001f;

	return PX4_OK;
}

int HaiTaiUART::readSpeed(float &speed_rpm)
{
	uint8_t response[CAN_FRAME_MAX_LEN] {};

	if (transaction(Command::ReadSpeed, nullptr, 0, response) != 5) {
		return PX4_ERROR;
	}

	speed_rpm = decodeInt32(&response[1]) * 0.01f;

	return PX4_OK;
}

int HaiTaiUART::readStatus(StatusInfo &status)
{
	uint8_t response[CAN_FRAME_MAX_LEN] {};

	if (transaction(Command::ReadStatus, nullptr, 0, response) != 8) {
		return PX4_ERROR;
	}

	status.temperature_c = response[1];
	status.q_current_a    = decodeInt16(&response[2]) * 0.001f;
	status.speed_rpm      = decodeInt16(&response[4]) * 0.01f;
	status.angle_deg     = decodeUint16(&response[6]) * (360.0f / 16384.0f);

	// 缓存最近一次成功读取，供 print_status 查看
	_last_status = status;
	_last_status_valid = true;

	return PX4_OK;
}

int HaiTaiUART::readRuntimeStatus(RuntimeStatusInfo &runtime_status)
{
	uint8_t response[CAN_FRAME_MAX_LEN] {};

	if (transaction(Command::ReadRuntime, nullptr, 0, response) != 8) {
		return PX4_ERROR;
	}

	runtime_status.bus_voltage_v = decodeUint16(&response[1]) * 0.01f;
	runtime_status.bus_current_a  = decodeUint16(&response[3]) * 0.01f;
	runtime_status.temperature_c = response[5];
	runtime_status.run_mode      = response[6];
	runtime_status.fault_code   = response[7];

	// 缓存最近一次成功读取，供 print_status 查看
	_last_runtime = runtime_status;
	_last_runtime_valid = true;

	return PX4_OK;
}

int HaiTaiUART::clearFault(uint8_t &remaining_fault)
{
	uint8_t response[CAN_FRAME_MAX_LEN] {};

	if (transaction(Command::ClearFault, nullptr, 0, response) != 2) {
		return PX4_ERROR;
	}

	remaining_fault = response[1];

	return PX4_OK;
}

int HaiTaiUART::readMotorParams(MotorParamsInfo &params)
{
	uint8_t response[CAN_FRAME_MAX_LEN] {};

	if (transaction(Command::ReadParams, nullptr, 0, response) != 7) {
		return PX4_ERROR;
	}

	params.torque_constant_na = decodeFloat(&response[2]);

	return PX4_OK;
}

int HaiTaiUART::setOriginHere(uint16_t &mechanical_offset)
{
	uint8_t response[CAN_FRAME_MAX_LEN] {};

	if (transaction(Command::SetOrigin, nullptr, 0, response) != 3) {
		return PX4_ERROR;
	}

	mechanical_offset = decodeUint16(&response[1]);

	return PX4_OK;
}

int HaiTaiUART::setTorque(float target_a, float *measured_a)
{
	// 0xC0: 目标 Q 轴电流 4s，单位 0.001A（力矩 = Q 轴电流 * 力矩常数）
	const int32_t raw_current = (int32_t)lrintf(target_a / 0.001f);

	uint8_t payload[4];
	encodeInt32(payload, raw_current);

	uint8_t response[CAN_FRAME_MAX_LEN] {};

	// 应答内容与 0xA1 一致：实测 Q 轴电流
	if (transaction(Command::SetTorque, payload, sizeof(payload), response) != 5) {
		return PX4_ERROR;
	}

	if (measured_a != nullptr) {
		*measured_a = decodeInt32(&response[1]) * 0.001f;
	}

	return PX4_OK;
}

int HaiTaiUART::sendTorque(float target_a)
{
	// 0xC0: 目标 Q 轴电流 4s，单位 0.001A；发送后不等待应答（高频控制专用）
	const int32_t raw_current = (int32_t)lrintf(target_a / 0.001f);

	uint8_t payload[4];
	encodeInt32(payload, raw_current);

	// 迟到的应答由下个事务开头的 tcflush 丢弃，不会破坏帧同步
	return (transaction(Command::SetTorque, payload, sizeof(payload), nullptr, false) < 0)
	       ? PX4_ERROR : PX4_OK;
}

int HaiTaiUART::goHome(float &single_turn_deg, float &multi_turn_deg)
{
	uint8_t response[CAN_FRAME_MAX_LEN] {};

	// 应答内容与 0xA3 一致：单圈/多圈绝对值角度
	if (transaction(Command::GoHome, nullptr, 0, response) != 7) {
		return PX4_ERROR;
	}

	single_turn_deg = decodeUint16(&response[1]) * (360.0f / 16384.0f);
	multi_turn_deg   = decodeInt32(&response[3]) * (360.0f / 16384.0f);

	return PX4_OK;
}

// ============================== 调度与串口底层 ==============================

void HaiTaiUART::Run()
{
	// 纯被动设计：心跳不做任何 UART 事务，仅处理 stop 命令；
	// 所有命令事务由调用方（如 head_pitch_controller）在其上下文中发起
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}
}

int HaiTaiUART::initializeUART()
{
	int32_t baud_rate = DEFAULT_BAUD_RATE;

	// 从串口配置参数读取波特率（如 SER_TEL1_BAUD）
	if (_baud_rate_parameter[0] != '\0') {
		param_t param = param_find(_baud_rate_parameter);

		if (param == PARAM_INVALID) {
			PX4_ERR("parameter %s not found", _baud_rate_parameter);
			return PX4_ERROR;
		}

		param_get(param, &baud_rate);
	}

	int32_t baud_rate_posix{0};

	switch (baud_rate) {
	case 9600:   baud_rate_posix = B9600;   break;
	case 19200:  baud_rate_posix = B19200;  break;
	case 38400:  baud_rate_posix = B38400;  break;
	case 57600:  baud_rate_posix = B57600;  break;
	case 115200: baud_rate_posix = B115200; break;
	case 230400: baud_rate_posix = B230400; break;
	case 460800: baud_rate_posix = B460800; break;
	case 921600: baud_rate_posix = B921600; break;

	default:
		PX4_ERR("unsupported baud rate %" PRId32, baud_rate);
		return PX4_ERROR;
	}

	_uart_fd = open(_device_name, O_RDWR | O_NOCTTY);

	if (_uart_fd < 0) {
		PX4_ERR("could not open %s", _device_name);
		return PX4_ERROR;
	}

	struct termios uart_config {};

	if (tcgetattr(_uart_fd, &uart_config) < 0) {
		PX4_ERR("failed to get attr");
		close(_uart_fd);
		_uart_fd = -1;
		return PX4_ERROR;
	}

	uart_config.c_oflag &= ~ONLCR;   // 不做 LF->CR/LF 转换
	uart_config.c_cflag &= ~CRTSCTS; // 关闭硬件流控

	if (cfsetispeed(&uart_config, baud_rate_posix) < 0
	    || cfsetospeed(&uart_config, baud_rate_posix) < 0) {
		PX4_ERR("failed to set speed");
		close(_uart_fd);
		_uart_fd = -1;
		return PX4_ERROR;
	}

	if (tcsetattr(_uart_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("failed to set attr");
		close(_uart_fd);
		_uart_fd = -1;
		return PX4_ERROR;
	}

	PX4_INFO("opened %s @ %" PRId32, _device_name, baud_rate);

	// 探测链路：读取电机版本（在 start 上下文执行，此时实例尚未发布，无并发风险；
	// UART转CAN 桥接模块与电机可能尚未上电，失败不阻止启动）
	VersionInfo version{};

	if (readVersion(version) == PX4_OK) {
		PX4_INFO("motor found: boot %u.%u, app %u.%u, hw %u.%u, protocol %u",
			 version.boot_version >> 8, version.boot_version & 0xFF,
			 version.app_version >> 8, version.app_version & 0xFF,
			 version.hardware_version >> 8, version.hardware_version & 0xFF,
			 version.protocol_version);

	} else {
		PX4_WARN("no motor response (bridge/motor powered?)");
	}

	return PX4_OK;
}

int HaiTaiUART::transaction(Command cmd, const uint8_t *payload, uint8_t payload_len,
			     uint8_t *response, bool wait_response)
{
	if (_uart_fd < 0) {
		return PX4_ERROR;
	}

	if (payload_len > CAN_FRAME_MAX_LEN - 1) {
		return PX4_ERROR;
	}

	// 组帧：数据场首字节为命令码（CAN ID 由 UART转CAN 模块添加）
	uint8_t frame[CAN_FRAME_MAX_LEN] {};
	frame[0] = static_cast<uint8_t>(cmd);

	if (payload != nullptr && payload_len > 0) {
		memcpy(&frame[1], payload, payload_len);
	}

	const uint8_t frame_len = 1 + payload_len;

	// 丢弃残留的未读数据（含 fire-and-forget 迟到的应答）
	tcflush(_uart_fd, TCIFLUSH);

	const int written = ::write(_uart_fd, frame, frame_len);

	if (written != frame_len) {
		PX4_ERR("write failed (%d/%u bytes)", written, frame_len);
		return PX4_ERROR;
	}

	_transaction_count++;

	// fire-and-forget：发送后立即返回，迟到的应答由下个事务的 tcflush 丢弃
	if (!wait_response) {
		return frame_len;
	}

	// 调用方不关心应答内容时读取并丢弃，避免残留数据影响后续帧同步
	uint8_t discard[CAN_FRAME_MAX_LEN];
	uint8_t *response_buf = (response != nullptr) ? response : discard;

	const int ret = readResponse(static_cast<uint8_t>(cmd), response_buf, expectedResponseLength(cmd));

	if (ret < 0) {
		_transaction_failures++;
	}

	return ret;
}

int HaiTaiUART::readResponse(uint8_t cmd, uint8_t *response, uint8_t expected_len)
{
	uint8_t total = 0;

	struct timeval timeout {};
	timeout.tv_sec = _response_timeout_us / 1000000;
	timeout.tv_usec = _response_timeout_us % 1000000;

	while (total < expected_len) {
		// select 会修改 fd_set，每次调用前需重新设置
		FD_ZERO(&_uart_fd_set);
		FD_SET(_uart_fd, &_uart_fd_set);

		int select_status = select(_uart_fd + 1, &_uart_fd_set, nullptr, nullptr, &timeout);

		if (select_status <= 0) {
			PX4_ERR("response timeout (cmd 0x%02X, %u/%u bytes)", cmd, total, expected_len);
			return PX4_ERROR;
		}

		uint8_t chunk[CAN_FRAME_MAX_LEN];
		const int ret = ::read(_uart_fd, chunk, sizeof(chunk));

		if (ret <= 0) {
			PX4_ERR("read failed");
			return PX4_ERROR;
		}

		for (int i = 0; i < ret; i++) {
			if (total == 0) {
				// 首字节须为命令码回显，否则丢弃杂散字节（帧重新同步）
				if (chunk[i] == cmd) {
					response[total++] = chunk[i];
				}

			} else if (total < expected_len) {
				response[total++] = chunk[i];
			}
		}
	}

	return total;
}

uint8_t HaiTaiUART::expectedResponseLength(Command cmd)
{
	switch (cmd) {
	case Command::ReadVersion: return 8; // 命令码 + Boot/软件/硬件版本各 2u + 协议版本 1u
	case Command::ReadCurrent: return 5; // 命令码 + 电流 4s
	case Command::ReadSpeed:   return 5; // 命令码 + 速度 4s
	case Command::ReadStatus:  return 8; // 命令码 + 温度 1u + 电流 2s + 速度 2s + 角度 2u
	case Command::ReadRuntime: return 8; // 命令码 + 电压 2u + 电流 2u + 温度 1u + 模式 1u + 故障 1u
	case Command::ClearFault:  return 2; // 命令码 + 故障码 1u
	case Command::ReadParams:  return 7; // 命令码 + 空 1u + 力矩常数 4f + 空 1u
	case Command::SetOrigin:   return 3; // 命令码 + 机械角度偏移 2u
	case Command::SetTorque:   return 5; // 命令码 + 实测电流 4s（同 0xA1）
	case Command::GoHome:      return 7; // 命令码 + 单圈角度 2u + 多圈角度 4s（同 0xA3）
	default:                   return 0;
	}
}

void HaiTaiUART::printFault(uint8_t fault)
{
	if (fault == 0) {
		PX4_INFO_RAW("fault: none\n");
		return;
	}

	PX4_INFO_RAW("fault: 0x%02X (", fault);

	if (fault & (1 << 0)) { PX4_INFO_RAW("voltage ");  }
	if (fault & (1 << 1)) { PX4_INFO_RAW("current ");  }
	if (fault & (1 << 2)) { PX4_INFO_RAW("temperature "); }
	if (fault & (1 << 3)) { PX4_INFO_RAW("encoder ");  }
	if (fault & (1 << 5)) { PX4_INFO_RAW("communication "); }
	if (fault & (1 << 6)) { PX4_INFO_RAW("hardware ");  }
	if (fault & (1 << 7)) { PX4_INFO_RAW("software ");  }

	PX4_INFO_RAW(")\n");
}

// ============================== 小端编解码 ==============================

void HaiTaiUART::encodeInt32(uint8_t *buf, int32_t value)
{
	memcpy(buf, &value, sizeof(value));
}

uint16_t HaiTaiUART::decodeUint16(const uint8_t *buf)
{
	uint16_t value = 0;
	memcpy(&value, buf, sizeof(value));
	return value;
}

int16_t HaiTaiUART::decodeInt16(const uint8_t *buf)
{
	int16_t value = 0;
	memcpy(&value, buf, sizeof(value));
	return value;
}

int32_t HaiTaiUART::decodeInt32(const uint8_t *buf)
{
	int32_t value = 0;
	memcpy(&value, buf, sizeof(value));
	return value;
}

float HaiTaiUART::decodeFloat(const uint8_t *buf)
{
	float value = 0.0f;
	memcpy(&value, buf, sizeof(value));
	return value;
}

// ============================== 模块入口 ==============================

int HaiTaiUART::task_spawn(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_ERR("missing device name, e.g. haitai_uart start /dev/ttyS1");
		return PX4_ERROR;
	}

	const char *device_name = argv[1];
	const char *baud_rate_parameter = (argc >= 3) ? argv[2] : nullptr;

	HaiTaiUART *instance = new HaiTaiUART(device_name, baud_rate_parameter);

	if (instance) {
		// 同步初始化串口（含链路探测），失败则拒绝启动
		if (instance->initializeUART() != PX4_OK) {
			delete instance;
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;
		// 低频心跳：仅维护 stop 命令响应，不做 UART 事务（纯被动设计）
		instance->ScheduleOnInterval(HEARTBEAT_INTERVAL_US);
		return PX4_OK;
	}

	PX4_ERR("alloc failed");

	return PX4_ERROR;
}

int HaiTaiUART::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HaiTaiUART::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

HaiTai HT-J-2E motor driver over a transparent UART-to-CAN bridge.

The driver sends CAN data fields (up to 8 bytes per frame, little endian) over a
flight controller UART to a UART-to-CAN bridge module. The CAN ID is stored in
the bridge and must not be part of the UART payload.

Protocol reference: HaiTai GDZ driver ZE300 custom CAN protocol V3.09b0.
Supported commands: 0x00, 0xA0, 0xA1, 0xA2, 0xA4, 0xAE, 0xAF, 0xB0, 0xB1,
0xC0 and 0xC4.

The driver is a passive service: it does not poll the motor by itself.
All command transactions are issued from the caller's context (e.g. a
controller module at up to 400 Hz via `sendTorque()`), which serializes
the UART access without locks. Other modules can access the running
instance via `HaiTaiUART::get_instance()`. Stop the caller module before
stopping this driver.

### Examples

Start the driver on a serial port:
$ haitai_uart start /dev/ttyS1
$ haitai_uart start /dev/ttyS1 SER_TEL1_BAUD

Check the driver status:
$ haitai_uart status

Stop the driver:
$ haitai_uart stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("haitai_uart", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("<device> [<baud_param>]", "UART device (e.g. /dev/ttyS1), optional baud rate parameter (e.g. SER_TEL1_BAUD, default 115200)", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int HaiTaiUART::print_status()
{
	PX4_INFO_RAW("device: %s (%s)\n", _device_name, (_uart_fd >= 0) ? "open" : "closed");
	PX4_INFO_RAW("transactions: %" PRIu32 ", failures: %" PRIu32 ", response timeout: %" PRIu32 " us\n",
		     _transaction_count, _transaction_failures, _response_timeout_us);

	if (_last_status_valid) {
		PX4_INFO_RAW("motor (last read): temp %d C, Iq %.3f A, speed %.2f RPM, angle %.1f deg\n",
			     _last_status.temperature_c, (double)_last_status.q_current_a,
			     (double)_last_status.speed_rpm, (double)_last_status.angle_deg);
	}

	if (_last_runtime_valid) {
		PX4_INFO_RAW("runtime (last read): bus %.2f V / %.2f A, temp %d C, run mode %d\n",
			     (double)_last_runtime.bus_voltage_v, (double)_last_runtime.bus_current_a,
			     _last_runtime.temperature_c, _last_runtime.run_mode);
		printFault(_last_runtime.fault_code);
	}

	return 0;
}

extern "C" __EXPORT int haitai_uart_main(int argc, char *argv[])
{
	return HaiTaiUART::main(argc, argv);
}
