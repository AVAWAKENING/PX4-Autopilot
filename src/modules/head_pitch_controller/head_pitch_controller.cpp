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

#include "head_pitch_controller.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <matrix/matrix/math.hpp>

using namespace time_literals;

/* haitai_uart 驱动连接的 UART 设备
 * （micoair h743-v2 UART4 → UART-CAN 桥接模块 → Haitai 电机） */
static constexpr char kUartDevice[] = "/dev/ttyS3";

HeadPitchController::HeadPitchController() :
	ModuleParams(nullptr),
	/* 与 DsmRc/SbusRc 等串口工作队列模块一致：由设备名映射对应串口队列 */
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(kUartDevice))
{
}

HeadPitchController::~HeadPitchController()
{
	ScheduleClear();

	/* fd 不在此处关闭：析构运行在 stop 调用者（nsh）上下文，而 fd 属于
	 * 工作队列任务；退出关闭由 Run() 的 should_exit 分支在 wq 上下文完成 */
}

bool HeadPitchController::init()
{
	/* 1. 下发电机参数：B 系列阻塞式调用，各助手内部 open/write/close
	 *    均在当前（nsh/rcS）上下文内闭环，仅启动阶段执行一次 */
	apply_motor_params();

	/* 2. 400Hz 周期调度；UART fd 由 Run() 在工作队列上下文内懒打开 */
	ScheduleOnInterval(2500_us);

	PX4_INFO("initialized: %s @ 400Hz, cnt/rad=%.3f",
		 kUartDevice, (double)_param_htp_cnt_per_rad.get());

	return true;
}

void HeadPitchController::apply_motor_params()
{
	/* 电机侧参数断电不保存，每次模块启动重新下发。
	 * 参数值为负（默认 -1）表示跳过该条不下发，避免不需要
	 * 模块配置电机时白白阻塞等待应答。
	 * 全部为阻塞调用（每条最多 ~50ms 应答超时），只在 init 阶段执行，
	 * 不会进入 400Hz 控制环。任一条失败只告警计数、不中止启动：
	 * 电机未上电时仍可先验证 UART 数据通路。
	 */
	int ret;

	const int32_t max_speed = _param_htp_max_speed.get();

	if (max_speed >= 0) {
		_param_apply_tried++;

		ret = haitai_uart_cmd_b2_max_speed_set(kUartDevice, (uint32_t)max_speed);

		if (ret != 0) {
			PX4_ERR("0xB2 max_speed set failed: %d", ret);
			_param_apply_fail++;
		}
	}

	const int32_t max_qcur = _param_htp_max_qcur.get();

	if (max_qcur >= 0) {
		_param_apply_tried++;

		ret = haitai_uart_cmd_b3_max_qcurrent_set(kUartDevice, (uint32_t)max_qcur);

		if (ret != 0) {
			PX4_ERR("0xB3 max_qcurrent set failed: %d", ret);
			_param_apply_fail++;
		}
	}

	const float pos_kp = _param_htp_pos_kp.get();

	if (pos_kp >= 0.0f) {
		_param_apply_tried++;

		ret = haitai_uart_cmd_b6_pos_kp_set(kUartDevice, pos_kp);

		if (ret != 0) {
			PX4_ERR("0xB6 pos_kp set failed: %d", ret);
			_param_apply_fail++;
		}
	}

	const float pos_ki = _param_htp_pos_ki.get();

	if (pos_ki >= 0.0f) {
		_param_apply_tried++;

		ret = haitai_uart_cmd_b7_pos_ki_set(kUartDevice, pos_ki);

		if (ret != 0) {
			PX4_ERR("0xB7 pos_ki set failed: %d", ret);
			_param_apply_fail++;
		}
	}

	const float vel_kp = _param_htp_vel_kp.get();

	if (vel_kp >= 0.0f) {
		_param_apply_tried++;

		ret = haitai_uart_cmd_b8_vel_kp_set(kUartDevice, vel_kp);

		if (ret != 0) {
			PX4_ERR("0xB8 vel_kp set failed: %d", ret);
			_param_apply_fail++;
		}
	}

	const float vel_ki = _param_htp_vel_ki.get();

	if (vel_ki >= 0.0f) {
		_param_apply_tried++;

		ret = haitai_uart_cmd_b9_vel_ki_set(kUartDevice, vel_ki);

		if (ret != 0) {
			PX4_ERR("0xB9 vel_ki set failed: %d", ret);
			_param_apply_fail++;
		}
	}

	if (_param_apply_fail > 0) {
		PX4_WARN("motor param apply failures: %u/%u (电机未上电或应答超时)",
			 (unsigned)_param_apply_fail, (unsigned)_param_apply_tried);

	} else if (_param_apply_tried == 0) {
		PX4_INFO("motor params all skipped (all < 0, 不下发)");

	} else {
		PX4_INFO("motor params applied OK: tried %u, skipped %u",
			 (unsigned)_param_apply_tried, 6u - (unsigned)_param_apply_tried);
	}
}

void HeadPitchController::Run()
{
	if (should_exit()) {
		ScheduleClear();

		/* fd 在工作队列上下文内关闭（此处即 wq:ttyS3 线程），
		 * haitai_uart_stop 内部先排空 TX FIFO 再 close */
		if (_uart_fd >= 0) {
			haitai_uart_stop(_uart_fd);
			_uart_fd = -1;
		}

		exit_and_cleanup();
		return;
	}

	/* 参数更新（1Hz 限流）：HTP_CNT_PER_RAD 立即生效；
	 * B 系列电机参数需 stop/start 重新下发 */
	parameter_update_s param_update;

	if (_parameter_update_sub.update(&param_update)) {
		updateParams();
	}

	vehicle_attitude_s att{};

	if (_vehicle_attitude_sub.update(&att)) {
		/* pitch = 姿态四元数对应的欧拉角 theta，单位 rad（保持 uORB 原有单位） */
		const matrix::Eulerf euler(matrix::Quatf(att.q));
		_pitch_rad = euler.theta();
		_attitude_valid = true;
	}

	vehicle_attitude_setpoint_s att_sp{};

	if (_vehicle_attitude_setpoint_sub.update(&att_sp)) {
		/* 本版本 vehicle_attitude_setpoint 只有期望四元数 q_d[4]，
		 * pitch_setpoint = q_d 对应欧拉角 theta，单位 rad（保持 uORB 原有单位） */
		_pitch_sp_rad = matrix::Eulerf(matrix::Quatf(att_sp.q_d)).theta();
	}

	/* 首个 vehicle_attitude 到达前不下发，避免用 0 位置命令驱动电机 */
	if (!_attitude_valid) {
		return;
	}

	/* fd 懒打开：必须在工作队列上下文内 open/close（与 DsmRc/SbusRc 等
	 * 串口工作队列模块一致 —— NuttX fd 表按任务组隔离，init() 所在的
	 * nsh/rcS 上下文打开的 fd 对 Run() 不可见）。失败仅计数，下周期重试 */
	if (_uart_fd < 0) {
		_uart_fd = haitai_uart_start(kUartDevice);

		if (_uart_fd < 0) {
			_tx_errors++;
			_tx_last_error = _uart_fd;
			return;
		}

		if (_fd_reopens > 0) {
			PX4_INFO("reopened %s: fd=%d (reopens=%" PRIu32 ")", kUartDevice, _uart_fd, _fd_reopens);
		}
	}

	/* 单位转换：pitch (rad) → Count（电机一圈 = 2π rad = 16384 Count） */
	_cmd_counts = (int32_t)lrintf(_pitch_rad * _param_htp_cnt_per_rad.get());

	const int send_ret = haitai_uart_send_c2_fast(_uart_fd, _cmd_counts);

	if (send_ret == 0) {
		_tx_count++;

	} else {
		_tx_errors++;
		_tx_last_error = send_ret; /* 负 errno，status 可见，便于定位 */

		/* fd 级错误（EACCES/EBADF）：在 wq 上下文关闭本上下文打开的 fd，
		 * 下周期 Run 内重开恢复 —— 与 DsmRc 的 close+reopen 恢复方式一致 */
		if (send_ret == -EACCES || send_ret == -EBADF) {
			haitai_uart_stop(_uart_fd);
			_uart_fd = -1;
			_fd_reopens++;
			PX4_ERR("uart fd error %d, closed (will reopen)", send_ret);
		}
	}
}

int HeadPitchController::print_status()
{
	PX4_INFO_RAW("Running: %s\n", is_running() ? "YES" : "NO");
	PX4_INFO_RAW("  queue: wq:ttyS3 @ 400Hz\n");
	PX4_INFO_RAW("  device: %s (fd=%d)\n", kUartDevice, _uart_fd);
	PX4_INFO_RAW("  pitch: %.3f rad (%.1f deg), pitch_setpoint: %.3f rad\n",
		     (double)_pitch_rad, (double)math::degrees(_pitch_rad), (double)_pitch_sp_rad);
	PX4_INFO_RAW("  cmd: %" PRId32 " Count\n", _cmd_counts);
	PX4_INFO_RAW("  tx: %" PRIu32 ", tx_errors: %" PRIu32 ", last_err: %d, fd_reopens: %" PRIu32 "\n",
		     _tx_count, _tx_errors, _tx_last_error, _fd_reopens);
	PX4_INFO_RAW("  motor params at start: tried %u, failed %u, skipped %u (参数<0 不下发, 修改后需 stop/start)\n",
		     (unsigned)_param_apply_tried, (unsigned)_param_apply_fail, 6u - (unsigned)_param_apply_tried);

	return 0;
}

int HeadPitchController::task_spawn(int argc, char *argv[])
{
	HeadPitchController *instance = new HeadPitchController();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;

	} else {
		PX4_ERR("alloc failed");
	}

	return PX4_ERROR;
}

int HeadPitchController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HeadPitchController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
头部俯仰控制器：经 haitai_uart 驱动（/dev/ttyS3 → UART-CAN 桥 → Haitai 电机）控制头部俯仰。

启动阶段将 HTP_* 参数经 0xB2/B3/B6/B7/B8/B9 下发到电机（阻塞式，仅一次；
值为 -1（默认）的条目跳过不下发；电机侧断电不保存，故每次启动重新下发），随后在 wq:ttyS3 队列以 400Hz 运行：
订阅 vehicle_attitude（pitch）与 vehicle_attitude_setpoint（pitch_body），
将 pitch（rad）换算为 Count（16384/圈，系数 HTP_CNT_PER_RAD）后
经 0xC2 绝对位置命令发送。

### Examples
$ head_pitch_controller start
$ head_pitch_controller status
$ head_pitch_controller stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("head_pitch_controller", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int head_pitch_controller_main(int argc, char *argv[])
{
	return HeadPitchController::main(argc, argv);
}
