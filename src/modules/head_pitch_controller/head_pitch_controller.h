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
 * @file head_pitch_controller.h
 *
 * 头部俯仰控制器（head pitch controller）
 *
 * 挂串口专属工作队列（由 px4::serial_port_to_wq 按设备名映射到
 * wq:ttyS3；该队列本板无其它使用者，400Hz 节拍干净，优先级低于姿态
 * 控制队列，故障时不会拖累飞控关键路径），控制周期 400Hz。
 *
 * 启动阶段（init，阻塞式，仅执行一次）：
 *   将 HTP_* 参数（值 >= 0 的条目；-1 = 不下发，默认全部 -1）经
 *   haitai_uart B 系列命令（0xB2/B3/B6/B7/B8/B9）下发到 Haitai 电机。
 *   电机侧断电不保存，故每次启动重新下发。
 *
 * 周期阶段（Run，400Hz）：
 *   1. 订阅 vehicle_attitude（pitch, rad）与 vehicle_attitude_setpoint
 *      （pitch_body, rad），保持 uORB 原有单位。
 *   2. 将 pitch（rad）乘以 HTP_CNT_PER_RAD 换算为 Count（电机一圈
 *      16384 Count）后，经 haitai_uart 0xC2 绝对位置命令发送。
 *
 * UART fd 生命周期全部在 Run()（工作队列上下文）内闭环 —— 懒打开 /
 * fd 级错误时关闭重开 / should_exit 时关闭。NuttX fd 表按任务组隔离，
 * init() 所在的 nsh/rcS 上下文打开的 fd 对工作队列任务不可见。
 * 此为 DsmRc/SbusRc 等串口工作队列模块的标准做法。
 *
 * 组织方式参考 src/modules/payload_deliverer：
 *   ModuleBase + ModuleParams（module.yaml 定义参数）+ ScheduledWorkItem
 */

#pragma once

#include <math.h>
#include <stdint.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

/* haitai_uart 驱动 extern "C" 接口（含快速路径 API） */
#include <drivers/haitai_uart/haitai_uart.h>

using namespace time_literals;

class HeadPitchController : public ModuleBase<HeadPitchController>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	HeadPitchController();
	~HeadPitchController() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	/** 启动阶段：将 HTP_* 参数经 B 系列命令下发到电机（阻塞式，仅一次） */
	void apply_motor_params();

	/* 参数更新订阅（1Hz 限流）。HTP_CNT_PER_RAD 运行中修改立即生效；
	 * 电机侧参数（B 系列）修改后需 stop/start 重新下发 */
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};

	float _pitch_rad{NAN};       /* 机体俯仰角（rad，uORB 原有单位） */
	float _pitch_sp_rad{NAN};    /* 俯仰设定值 pitch_body（rad，uORB 原有单位） */
	int32_t _cmd_counts{0};      /* 最近一次下发的 0xC2 绝对位置（Count） */
	bool _attitude_valid{false}; /* 收到首个 vehicle_attitude 前不下发 */

	int _uart_fd{-1};            /* 持久 UART fd（Run 内懒打开，wq 上下文所有） */
	uint32_t _tx_count{0};
	uint32_t _tx_errors{0};
	int _tx_last_error{0};         /* 最近一次发送/打开失败返回值（负 errno） */
	uint32_t _fd_reopens{0};       /* fd 级错误后关闭待重开的次数 */
	uint8_t _param_apply_tried{0}; /* 启动时实际下发的 B 系列条数（0~6） */
	uint8_t _param_apply_fail{0};  /* 其中失败的条数 */

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::HTP_MAX_SPEED>) _param_htp_max_speed,
		(ParamInt<px4::params::HTP_MAX_QCUR>) _param_htp_max_qcur,
		(ParamFloat<px4::params::HTP_POS_KP>) _param_htp_pos_kp,
		(ParamFloat<px4::params::HTP_POS_KI>) _param_htp_pos_ki,
		(ParamFloat<px4::params::HTP_VEL_KP>) _param_htp_vel_kp,
		(ParamFloat<px4::params::HTP_VEL_KI>) _param_htp_vel_ki,
		(ParamFloat<px4::params::HTP_CNT_PER_RAD>) _param_htp_cnt_per_rad
	)
};
