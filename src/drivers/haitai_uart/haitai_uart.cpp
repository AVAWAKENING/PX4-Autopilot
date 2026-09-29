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
 * @file haitai_uart.cpp
 *
 * Haitai ZE300 GDZ 驱动器 UART 透传驱动实现
 *
 * 架构：micoair h743-v2 → /dev/ttySx → UART-CAN 桥接模块 → CAN → Haitai 电机
 * 桥接模块为 8 字节透传：本驱动发送的 8 字节 = CAN Data 字段，StdID 由模块内置。
 *
 * 实现命令码（V3.09b0 协议）：
 *   B 系列（参数读写，发后等待应答）：
 *     0xB1 设置原点；0xB2 位置最大速度；0xB3 最大Q轴电流；
 *     0xB6 位置Kp；0xB7 位置Ki；0xB8 速度Kp；0xB9 速度Ki
 *   C 系列（高速控制，不等应答）：
 *     0xC2 绝对位置；0xC4 最短距离回原点；0xCF 关闭电机输出
 *
 * NSH 入口（举例）：
 *   haitai_uart b1_zero -d /dev/ttyS3
 *   haitai_uart b2_get -d /dev/ttyS3
 *   haitai_uart b2_set -d /dev/ttyS3 -v 3000
 *   haitai_uart c2_abs -d /dev/ttyS3 -v 8192
 *   haitai_uart cf_disable -d /dev/ttyS3
 */

#include "haitai_uart.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

/* 默认波特率：115200。如需其它速率可改此宏（如 B9600、B57600、B921600）。
 * 注意：必须与 UART-CAN 桥接模块的 UART 侧波特率一致。
 */
#ifndef HAITAI_UART_BAUD
#define HAITAI_UART_BAUD B115200
#endif

/* 测试输出字符串 */
static const char kTestMessage[] = "this is haitai_uart driver\n";
static const int  kTestMessageLen = sizeof(kTestMessage) - 1;

namespace haitai_uart
{

int initialise_uart(const char *const device, int &uart_fd)
{
	if (device == nullptr) {
		PX4_ERR("haitai_uart: device is null");
		return -1;
	}

	uart_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (uart_fd < 0) {
		PX4_ERR("haitai_uart: failed to open %s (errno=%d)", device, errno);
		return -1;
	}

	struct termios uart_config;
	bzero(&uart_config, sizeof(uart_config));
	tcgetattr(uart_fd, &uart_config);

	if (cfsetspeed(&uart_config, HAITAI_UART_BAUD) < 0) {
		PX4_ERR("haitai_uart: cfsetspeed failed for %s", device);
		close(uart_fd);
		uart_fd = -1;
		return -1;
	}

	/* c_cflag 完全覆盖为 8N1 + CLOCAL + CREAD
	 * CLOCAL 必须置位，否则 NuttX 会启用 modem 控制（等 DCD）门控 TX
	 */
	uart_config.c_cflag = CS8 | CLOCAL | CREAD;
	uart_config.c_iflag = 0;
	uart_config.c_oflag = 0;
	uart_config.c_lflag = 0;

	if (tcsetattr(uart_fd, TCSANOW, &uart_config) != 0) {
		PX4_ERR("haitai_uart: tcsetattr failed for %s", device);
		close(uart_fd);
		uart_fd = -1;
		return -1;
	}

	return 0;
}

int deinitialise_uart(int &uart_fd)
{
	if (uart_fd < 0) {
		return -1;
	}

	int ret = close(uart_fd);

	if (ret == 0) {
		uart_fd = -1;
	}

	return ret;
}

int write_test(int uart_fd)
{
	if (uart_fd < 0) {
		PX4_ERR("haitai_uart: write_test invalid fd");
		return -1;
	}

	int ret = ::write(uart_fd, kTestMessage, kTestMessageLen);

	if (ret != kTestMessageLen) {
		PX4_WARN("haitai_uart: write_test short write: ret=%d errno=%d", ret, errno);
	}

	return ret;
}

int read_test(int uart_fd, char *buf, int buf_len)
{
	if (uart_fd < 0) {
		PX4_ERR("haitai_uart: read_test invalid fd");
		return -1;
	}

	if (buf == nullptr || buf_len <= 0) {
		PX4_ERR("haitai_uart: read_test invalid buf");
		return -1;
	}

	/* 非阻塞模式下 read() 无数据立即返回 -1/EAGAIN，单次读必空，
	 * 这里轮询 5 秒给 PC 端发数据的时间窗口。
	 */
	const int kPollPeriodUs = 10000;
	const int kPollTotalUs  = 5 * 1000 * 1000;
	int total = 0;

	for (int elapsed = 0; elapsed < kPollTotalUs; elapsed += kPollPeriodUs) {
		int len = ::read(uart_fd, buf + total, buf_len - 1 - total);

		if (len > 0) {
			total += len;

			if (total >= buf_len - 1) {
				break;
			}

			usleep(1000);
			continue;
		}

		if (len < 0 && errno != EAGAIN) {
			PX4_WARN("haitai_uart: read_test read error: errno=%d", errno);
			break;
		}

		usleep(kPollPeriodUs);
	}

	if (total > 0) {
		buf[total] = '\0';
	}

	return total;
}

/* ============================================================
 *  CAN 协议层工具函数
 * ============================================================ */

int send_frame(int uart_fd, uint8_t cmd, const uint8_t *data, uint8_t data_len)
{
	if (uart_fd < 0) {
		return -EBADF;
	}

	if (data_len > 7) {
		return -EINVAL;
	}

	/* 按协议 DLC 实际字节数发送，不补 0 到 8 字节
	 * DLC = 1（命令码）+ data_len（数据字段）
	 *   无数据命令 (B1/C4/CF/读B2-B9)：DLC=1，只发命令码
	 *   带数据命令 (设B2-B9, C2)：DLC=5，发命令码+4字节参数
	 */
	uint8_t frame[8];
	frame[0] = cmd;
	int frame_len = 1;

	if (data && data_len > 0) {
		memcpy(frame + 1, data, data_len);
		frame_len = 1 + data_len;
	}

	int ret = ::write(uart_fd, frame, frame_len);

	if (ret != frame_len) {
		PX4_WARN("haitai_uart: send_frame cmd=0x%02X write ret=%d errno=%d", cmd, ret, errno);
		return -errno;
	}

	/* 等 TX 真正出线再返回。最长 5 字节 @115200 ≈ 0.43ms，留 2ms 余量 */
	usleep(2000);
	return frame_len;
}

int wait_response(int uart_fd, uint8_t expected_cmd, uint8_t *resp_data,
		  uint8_t resp_data_len, int timeout_ms)
{
	if (uart_fd < 0) {
		return -EBADF;
	}

	if (resp_data == nullptr || resp_data_len == 0 || resp_data_len > 7) {
		return -EINVAL;
	}

	const int total_expected = 1 + resp_data_len;  /* [0]=cmd + [1..N]=data */
	uint8_t buf[8];
	int got = 0;

	const int poll_period_us = 1000;   /* 1ms */
	const int total_us = timeout_ms * 1000;

	for (int elapsed = 0; elapsed < total_us && got < total_expected; elapsed += poll_period_us) {
		int len = ::read(uart_fd, buf + got, total_expected - got);

		if (len > 0) {
			/* 首字节必须是期望的命令码，否则视为同步丢失，丢弃重收 */
			if (got == 0 && buf[0] != expected_cmd) {
				/* 极端情况下可能丢命令字节，整体重启接收 */
				got = 0;
				continue;
			}

			got += len;
			continue;
		}

		if (len < 0 && errno != EAGAIN) {
			PX4_WARN("haitai_uart: wait_response read error: errno=%d", errno);
			return -errno;
		}

		usleep(poll_period_us);
	}

	if (got < total_expected) {
		return -ETIMEDOUT;
	}

	memcpy(resp_data, buf + 1, resp_data_len);
	return 0;
}

/* 小端字节序辅助：把 4 字节组/拆成 32 位整型 */
static inline uint32_t le4_to_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void u32_to_le4(uint8_t *p, uint32_t v)
{
	p[0] = v & 0xFF;
	p[1] = (v >> 8) & 0xFF;
	p[2] = (v >> 16) & 0xFF;
	p[3] = (v >> 24) & 0xFF;
}

} /* namespace haitai_uart */

/* ============================================================
 *  extern "C" 测试入口（保留供 UART 通路自检）
 * ============================================================ */

extern "C" __EXPORT int haitai_uart_write_test(const char *device)
{
	if (device == nullptr) {
		return -1;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -1;
	}

	int ret = haitai_uart::write_test(fd);
	usleep(10000);
	haitai_uart::deinitialise_uart(fd);
	return ret;
}

extern "C" __EXPORT int haitai_uart_read_test(const char *device, char *buf, int buf_len)
{
	if (device == nullptr || buf == nullptr || buf_len <= 0) {
		return -1;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -1;
	}

	int ret = haitai_uart::read_test(fd, buf, buf_len);
	haitai_uart::deinitialise_uart(fd);
	return ret;
}

/* ============================================================
 *  extern "C" 快速路径 API（持久 fd，供 400Hz 流式控制）
 * ============================================================ */

extern "C" __EXPORT int haitai_uart_start(const char *device)
{
	if (device == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	return fd;
}

extern "C" __EXPORT int haitai_uart_send_c2_fast(int fd, int32_t position)
{
	if (fd < 0) {
		return -EBADF;
	}

	/* DLC5：0xC2 + 4 字节小端位置。平台为小端，直接 memcpy */
	uint8_t frame[5];
	frame[0] = haitai_uart::CMD_C2_ABS_POS;
	memcpy(frame + 1, &position, 4);

	int ret = ::write(fd, frame, 5);

	if (ret != 5) {
		return (ret < 0) ? -errno : -EIO;
	}

	return 0;
}

extern "C" __EXPORT int haitai_uart_stop(int fd)
{
	if (fd < 0) {
		return -EBADF;
	}

	/* 等 TX FIFO 排空再 close（5 字节 @115200 ≈ 0.43ms，2ms 足够） */
	usleep(2000);
	return close(fd);
}

/* ============================================================
 *  extern "C" B 系列命令 API（参数读写，等应答）
 * ============================================================ */

/* 0xB1: 设置当前位置为原点。应答约 35ms，给 100ms 超时 */
extern "C" __EXPORT int haitai_uart_cmd_b1_zero_origin(const char *device, uint16_t *mech_offset)
{
	if (device == nullptr || mech_offset == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B1_ZERO_ORIGIN, nullptr, 0);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[2];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B1_ZERO_ORIGIN, resp, 2, 100);
	haitai_uart::deinitialise_uart(fd);

	if (ret < 0) {
		return ret;
	}

	*mech_offset = (uint16_t)resp[0] | ((uint16_t)resp[1] << 8);
	return 0;
}

/* 0xB2: 读取位置模式最大速度（单位 0.01Rpm） */
extern "C" __EXPORT int haitai_uart_cmd_b2_max_speed_get(const char *device, uint32_t *value)
{
	if (device == nullptr || value == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B2_MAX_SPEED, nullptr, 0);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B2_MAX_SPEED, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	if (ret < 0) {
		return ret;
	}

	*value = haitai_uart::le4_to_u32(resp);
	return 0;
}

/* 0xB2: 设置位置模式最大速度（断电不保存） */
extern "C" __EXPORT int haitai_uart_cmd_b2_max_speed_set(const char *device, uint32_t value)
{
	if (device == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	uint8_t data[4];
	haitai_uart::u32_to_le4(data, value);

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B2_MAX_SPEED, data, 4);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B2_MAX_SPEED, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	return ret;
}

/* 0xB3: 读取最大Q轴电流（单位 0.001A） */
extern "C" __EXPORT int haitai_uart_cmd_b3_max_qcurrent_get(const char *device, uint32_t *value)
{
	if (device == nullptr || value == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B3_MAX_QCURRENT, nullptr, 0);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B3_MAX_QCURRENT, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	if (ret < 0) {
		return ret;
	}

	*value = haitai_uart::le4_to_u32(resp);
	return 0;
}

/* 0xB3: 设置最大Q轴电流（断电不保存） */
extern "C" __EXPORT int haitai_uart_cmd_b3_max_qcurrent_set(const char *device, uint32_t value)
{
	if (device == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	uint8_t data[4];
	haitai_uart::u32_to_le4(data, value);

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B3_MAX_QCURRENT, data, 4);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B3_MAX_QCURRENT, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	return ret;
}

/* 0xB6: 读取位置环 Kp (4f 浮点，小端) */
extern "C" __EXPORT int haitai_uart_cmd_b6_pos_kp_get(const char *device, float *value)
{
	if (device == nullptr || value == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B6_POS_KP, nullptr, 0);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B6_POS_KP, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	if (ret < 0) {
		return ret;
	}

	memcpy(value, resp, 4);  /* 平台为小端，直接 memcpy */
	return 0;
}

/* 0xB6: 设置位置环 Kp */
extern "C" __EXPORT int haitai_uart_cmd_b6_pos_kp_set(const char *device, float value)
{
	if (device == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	uint8_t data[4];
	memcpy(data, &value, 4);

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B6_POS_KP, data, 4);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B6_POS_KP, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	return ret;
}

/* 0xB7: 读取位置环 Ki */
extern "C" __EXPORT int haitai_uart_cmd_b7_pos_ki_get(const char *device, float *value)
{
	if (device == nullptr || value == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B7_POS_KI, nullptr, 0);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B7_POS_KI, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	if (ret < 0) {
		return ret;
	}

	memcpy(value, resp, 4);
	return 0;
}

/* 0xB7: 设置位置环 Ki */
extern "C" __EXPORT int haitai_uart_cmd_b7_pos_ki_set(const char *device, float value)
{
	if (device == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	uint8_t data[4];
	memcpy(data, &value, 4);

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B7_POS_KI, data, 4);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B7_POS_KI, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	return ret;
}

/* 0xB8: 读取速度环 Kp */
extern "C" __EXPORT int haitai_uart_cmd_b8_vel_kp_get(const char *device, float *value)
{
	if (device == nullptr || value == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B8_VEL_KP, nullptr, 0);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B8_VEL_KP, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	if (ret < 0) {
		return ret;
	}

	memcpy(value, resp, 4);
	return 0;
}

/* 0xB8: 设置速度环 Kp */
extern "C" __EXPORT int haitai_uart_cmd_b8_vel_kp_set(const char *device, float value)
{
	if (device == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	uint8_t data[4];
	memcpy(data, &value, 4);

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B8_VEL_KP, data, 4);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B8_VEL_KP, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	return ret;
}

/* 0xB9: 读取速度环 Ki */
extern "C" __EXPORT int haitai_uart_cmd_b9_vel_ki_get(const char *device, float *value)
{
	if (device == nullptr || value == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B9_VEL_KI, nullptr, 0);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B9_VEL_KI, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	if (ret < 0) {
		return ret;
	}

	memcpy(value, resp, 4);
	return 0;
}

/* 0xB9: 设置速度环 Ki */
extern "C" __EXPORT int haitai_uart_cmd_b9_vel_ki_set(const char *device, float value)
{
	if (device == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	uint8_t data[4];
	memcpy(data, &value, 4);

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_B9_VEL_KI, data, 4);

	if (ret < 0) {
		haitai_uart::deinitialise_uart(fd);
		return ret;
	}

	uint8_t resp[4];
	ret = haitai_uart::wait_response(fd, haitai_uart::CMD_B9_VEL_KI, resp, 4, 50);
	haitai_uart::deinitialise_uart(fd);

	return ret;
}

/* ============================================================
 *  extern "C" C 系列命令 API（高速控制，不等应答）
 * ============================================================ */

/* 0xC2: 绝对位置控制。position 单位 Count，一圈 = 16384 */
extern "C" __EXPORT int haitai_uart_cmd_c2_abs_pos(const char *device, int32_t position)
{
	if (device == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	uint8_t data[4];
	memcpy(data, &position, 4);

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_C2_ABS_POS, data, 4);
	haitai_uart::deinitialise_uart(fd);
	return ret;
}

/* 0xC4: 最短距离回原点（≤ 180°） */
extern "C" __EXPORT int haitai_uart_cmd_c4_return_origin(const char *device)
{
	if (device == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_C4_RETURN_ORIGIN, nullptr, 0);
	haitai_uart::deinitialise_uart(fd);
	return ret;
}

/* 0xCF: 关闭电机输出，电机进入自由态 */
extern "C" __EXPORT int haitai_uart_cmd_cf_disable(const char *device)
{
	if (device == nullptr) {
		return -EINVAL;
	}

	int fd = -1;

	if (haitai_uart::initialise_uart(device, fd) != 0) {
		return -EIO;
	}

	int ret = haitai_uart::send_frame(fd, haitai_uart::CMD_CF_DISABLE, nullptr, 0);
	haitai_uart::deinitialise_uart(fd);
	return ret;
}

/* ============================================================
 *  NSH 入口
 * ============================================================ */

static void print_usage()
{
	PX4_INFO_RAW("usage:\n");
	PX4_INFO_RAW("  haitai_uart <command> -d <device> [-v <value>]\n");
	PX4_INFO_RAW("\nUART self-test:\n");
	PX4_INFO_RAW("  test_write   -d /dev/ttyS3           # 输出测试字符串\n");
	PX4_INFO_RAW("  test_read    -d /dev/ttyS3           # 读取 5s 打印\n");
	PX4_INFO_RAW("  status\n");
	PX4_INFO_RAW("\nB-series (param read/write, awaits response):\n");
	PX4_INFO_RAW("  b1_zero      -d <dev>                 # 0xB1 set current pos as origin\n");
	PX4_INFO_RAW("  b2_get       -d <dev>                 # 0xB2 read max speed (0.01Rpm)\n");
	PX4_INFO_RAW("  b2_set       -d <dev> -v <u32>        # 0xB2 set max speed\n");
	PX4_INFO_RAW("  b3_get       -d <dev>                 # 0xB3 read max Q-current (0.001A)\n");
	PX4_INFO_RAW("  b3_set       -d <dev> -v <u32>        # 0xB3 set max Q-current\n");
	PX4_INFO_RAW("  b6_get       -d <dev>                 # 0xB6 read position Kp\n");
	PX4_INFO_RAW("  b6_set       -d <dev> -v <float>      # 0xB6 set position Kp\n");
	PX4_INFO_RAW("  b7_get       -d <dev>                 # 0xB7 read position Ki\n");
	PX4_INFO_RAW("  b7_set       -d <dev> -v <float>      # 0xB7 set position Ki\n");
	PX4_INFO_RAW("  b8_get       -d <dev>                 # 0xB8 read velocity Kp\n");
	PX4_INFO_RAW("  b8_set       -d <dev> -v <float>      # 0xB8 set velocity Kp\n");
	PX4_INFO_RAW("  b9_get       -d <dev>                 # 0xB9 read velocity Ki\n");
	PX4_INFO_RAW("  b9_set       -d <dev> -v <float>      # 0xB9 set velocity Ki\n");
	PX4_INFO_RAW("\nC-series (high-speed control, no wait):\n");
	PX4_INFO_RAW("  c2_abs       -d <dev> -v <i32>        # 0xC2 absolute pos (16384/turn)\n");
	PX4_INFO_RAW("  c4_origin    -d <dev>                 # 0xC4 return to origin (<=180deg)\n");
	PX4_INFO_RAW("  cf_disable   -d <dev>                 # 0xCF disable motor output\n");
}

/* 从 argv 解析 -d <device> 和 -v <value> */
static void parse_args(int argc, char *argv[], const char *&device, const char *&value_str)
{
	for (int i = 2; i < argc - 1; ++i) {
		if (strcmp(argv[i], "-d") == 0) {
			device = argv[i + 1];

		} else if (strcmp(argv[i], "-v") == 0) {
			value_str = argv[i + 1];
		}
	}
}

extern "C" __EXPORT int haitai_uart_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return PX4_OK;
	}

	const char *command = argv[1];
	const char *device = nullptr;
	const char *value_str = nullptr;
	parse_args(argc, argv, device, value_str);

	/* -- 通用：测试与状态 -- */
	if (strcmp(command, "status") == 0) {
		PX4_INFO("haitai_uart: Haitai ZE300 GDZ driver, 11 cmds (B1-B9, C2/C4/CF)");
		return PX4_OK;

	} else if (strcmp(command, "test_write") == 0) {
		if (device == nullptr) {
			PX4_ERR("test_write requires -d <device>");
			return PX4_ERROR;
		}

		int ret = haitai_uart_write_test(device);
		PX4_INFO("test_write -> %s ret=%d", device, ret);
		return (ret >= 0) ? PX4_OK : PX4_ERROR;

	} else if (strcmp(command, "test_read") == 0) {
		if (device == nullptr) {
			PX4_ERR("test_read requires -d <device>");
			return PX4_ERROR;
		}

		char buf[128];
		int ret = haitai_uart_read_test(device, buf, sizeof(buf));
		PX4_INFO("test_read -> %s ret=%d", device, ret);

		if (ret > 0) {
			PX4_INFO_RAW("  data: %s\n", buf);
		}

		return (ret >= 0) ? PX4_OK : PX4_ERROR;
	}

	/* -- B 系列：所有命令都需要 -d -- */
	if (device == nullptr) {
		PX4_ERR("command '%s' requires -d <device>", command);
		return PX4_ERROR;
	}

	/* 0xB1 */
	if (strcmp(command, "b1_zero") == 0) {
		uint16_t offset = 0;
		int ret = haitai_uart_cmd_b1_zero_origin(device, &offset);
		PX4_INFO("b1_zero -> ret=%d, mech_offset=%u", ret, (unsigned)offset);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	/* 0xB2 读/设 */
	if (strcmp(command, "b2_get") == 0) {
		uint32_t v = 0;
		int ret = haitai_uart_cmd_b2_max_speed_get(device, &v);
		PX4_INFO("b2_get -> ret=%d, max_speed=%" PRIu32 " (0.01Rpm) = %.2f Rpm", ret, v, (double)(v / 100.0f));
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	if (strcmp(command, "b2_set") == 0) {
		if (value_str == nullptr) {
			PX4_ERR("b2_set requires -v <u32>");
			return PX4_ERROR;
		}

		uint32_t v = (uint32_t)strtoul(value_str, nullptr, 0);
		int ret = haitai_uart_cmd_b2_max_speed_set(device, v);
		PX4_INFO("b2_set -> ret=%d, max_speed=%" PRIu32 " (0.01Rpm)", ret, v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	/* 0xB3 读/设 */
	if (strcmp(command, "b3_get") == 0) {
		uint32_t v = 0;
		int ret = haitai_uart_cmd_b3_max_qcurrent_get(device, &v);
		PX4_INFO("b3_get -> ret=%d, max_qcurrent=%" PRIu32 " (0.001A) = %.3f A", ret, v, (double)(v / 1000.0f));
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	if (strcmp(command, "b3_set") == 0) {
		if (value_str == nullptr) {
			PX4_ERR("b3_set requires -v <u32>");
			return PX4_ERROR;
		}

		uint32_t v = (uint32_t)strtoul(value_str, nullptr, 0);
		int ret = haitai_uart_cmd_b3_max_qcurrent_set(device, v);
		PX4_INFO("b3_set -> ret=%d, max_qcurrent=%" PRIu32 " (0.001A)", ret, v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	/* 0xB6 读/设 */
	if (strcmp(command, "b6_get") == 0) {
		float v = 0.0f;
		int ret = haitai_uart_cmd_b6_pos_kp_get(device, &v);
		PX4_INFO("b6_get -> ret=%d, pos_kp=%f", ret, (double)v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	if (strcmp(command, "b6_set") == 0) {
		if (value_str == nullptr) {
			PX4_ERR("b6_set requires -v <float>");
			return PX4_ERROR;
		}

		float v = strtof(value_str, nullptr);
		int ret = haitai_uart_cmd_b6_pos_kp_set(device, v);
		PX4_INFO("b6_set -> ret=%d, pos_kp=%f", ret, (double)v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	/* 0xB7 读/设 */
	if (strcmp(command, "b7_get") == 0) {
		float v = 0.0f;
		int ret = haitai_uart_cmd_b7_pos_ki_get(device, &v);
		PX4_INFO("b7_get -> ret=%d, pos_ki=%f", ret, (double)v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	if (strcmp(command, "b7_set") == 0) {
		if (value_str == nullptr) {
			PX4_ERR("b7_set requires -v <float>");
			return PX4_ERROR;
		}

		float v = strtof(value_str, nullptr);
		int ret = haitai_uart_cmd_b7_pos_ki_set(device, v);
		PX4_INFO("b7_set -> ret=%d, pos_ki=%f", ret, (double)v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	/* 0xB8 读/设 */
	if (strcmp(command, "b8_get") == 0) {
		float v = 0.0f;
		int ret = haitai_uart_cmd_b8_vel_kp_get(device, &v);
		PX4_INFO("b8_get -> ret=%d, vel_kp=%f", ret, (double)v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	if (strcmp(command, "b8_set") == 0) {
		if (value_str == nullptr) {
			PX4_ERR("b8_set requires -v <float>");
			return PX4_ERROR;
		}

		float v = strtof(value_str, nullptr);
		int ret = haitai_uart_cmd_b8_vel_kp_set(device, v);
		PX4_INFO("b8_set -> ret=%d, vel_kp=%f", ret, (double)v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	/* 0xB9 读/设 */
	if (strcmp(command, "b9_get") == 0) {
		float v = 0.0f;
		int ret = haitai_uart_cmd_b9_vel_ki_get(device, &v);
		PX4_INFO("b9_get -> ret=%d, vel_ki=%f", ret, (double)v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	if (strcmp(command, "b9_set") == 0) {
		if (value_str == nullptr) {
			PX4_ERR("b9_set requires -v <float>");
			return PX4_ERROR;
		}

		float v = strtof(value_str, nullptr);
		int ret = haitai_uart_cmd_b9_vel_ki_set(device, v);
		PX4_INFO("b9_set -> ret=%d, vel_ki=%f", ret, (double)v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	/* 0xC2 绝对位置 */
	if (strcmp(command, "c2_abs") == 0) {
		if (value_str == nullptr) {
			PX4_ERR("c2_abs requires -v <i32>");
			return PX4_ERROR;
		}

		int32_t v = (int32_t)strtol(value_str, nullptr, 0);
		int ret = haitai_uart_cmd_c2_abs_pos(device, v);
		PX4_INFO("c2_abs -> ret=%d, position=%d (Count)", ret, (int)v);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	/* 0xC4 回原点 */
	if (strcmp(command, "c4_origin") == 0) {
		int ret = haitai_uart_cmd_c4_return_origin(device);
		PX4_INFO("c4_origin -> ret=%d", ret);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	/* 0xCF 关闭电机 */
	if (strcmp(command, "cf_disable") == 0) {
		int ret = haitai_uart_cmd_cf_disable(device);
		PX4_INFO("cf_disable -> ret=%d", ret);
		return (ret == 0) ? PX4_OK : PX4_ERROR;
	}

	print_usage();
	return PX4_ERROR;
}
