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

#include "blackbox_data.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <inttypes.h>

// Logger module entry point
extern "C" __EXPORT int logger_main(int argc, char *argv[]);

using namespace time_literals;

BlackboxData::BlackboxData() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

BlackboxData::~BlackboxData()
{
	ScheduleClear();
}

bool BlackboxData::init()
{
	ScheduleOnInterval(100_ms);

	PX4_INFO("BlackboxData initialized");
	return true;
}

int BlackboxData::print_status()
{
	PX4_INFO_RAW("Blackbox Data Module Status:\n");
	PX4_INFO_RAW("  Running: %s\n", is_running() ? "YES" : "NO");
	PX4_INFO_RAW("  Publish rate: %.2f Hz\n", (double)_publish_rate);

	sensor_gps_s gps{};
	vehicle_global_position_s global_pos{};
	vehicle_local_position_s lpos{};
	battery_status_s battery{};
	home_position_s home{};

	bool has_gps = _sensor_gps_sub.copy(&gps);
	bool has_global_pos = _vehicle_global_position_sub.copy(&global_pos);
	bool has_local_pos = _vehicle_local_position_sub.copy(&lpos);
	bool has_battery = _battery_status_sub.copy(&battery);
	bool has_home = _home_position_sub.copy(&home);

	PX4_INFO_RAW("Subscriptions:\n");
	PX4_INFO_RAW("  GPS: %s\n", has_gps ? "valid" : "no data");
	if (has_gps) {
		PX4_INFO_RAW("          Fix type: %d, Satellites: %d\n", gps.fix_type, gps.satellites_used);
	}

	PX4_INFO_RAW("  Global Position: %s\n", has_global_pos ? "valid" : "no data");
	if (has_global_pos) {
		PX4_INFO_RAW("          Lat: %.7f, Lon: %.7f, Alt: %.2f m\n",
			 (double)global_pos.lat, (double)global_pos.lon, (double)global_pos.alt);
	}

	PX4_INFO_RAW("  Local Position: %s\n", has_local_pos ? "valid" : "no data");
	if (has_local_pos) {
		PX4_INFO_RAW("          X: %.2f, Y: %.2f, Z: %.2f m\n",
			 (double)lpos.x, (double)lpos.y, (double)lpos.z);
		PX4_INFO_RAW("          Vx: %.2f, Vy: %.2f, Vz: %.2f m/s\n",
			 (double)lpos.vx, (double)lpos.vy, (double)lpos.vz);
	}

	PX4_INFO_RAW("  Battery: %s\n", has_battery ? "valid" : "no data");
	if (has_battery) {
		PX4_INFO_RAW("          Remaining: %.1f%%, Voltage: %.2f V\n",
			 (double)(battery.remaining * 100.0f), (double)battery.voltage_v);
	}

	PX4_INFO_RAW("  Home Position: %s\n", has_home ? "valid" : "no data");
	if (has_home) {
		PX4_INFO_RAW("          Lat: %.7f, Lon: %.7f, Alt: %.2f m\n",
			 (double)home.lat, (double)home.lon, (double)home.alt);
	}

	black_box_low_bandwidth_s &msg = _black_box_low_bw_pub.get();
	if (msg.timestamp > 0) {
		PX4_INFO_RAW("Last published message:\n");
		PX4_INFO_RAW("  Timestamp: %" PRIu64 " us\n", msg.timestamp);
		PX4_INFO_RAW("  Position: Lat=%.7f, Lon=%.7f, Alt=%.2f m\n",
			 (double)msg.latitude_deg, (double)msg.longitude_deg, (double)msg.altitude_msl_m);
		PX4_INFO_RAW("  Relative Alt: %.2f m\n", (double)msg.relative_alt_m);
		PX4_INFO_RAW("  Velocity NED: %.2f, %.2f, %.2f m/s\n",
			 (double)msg.vn_m_s, (double)msg.ve_m_s, (double)msg.vd_m_s);
		PX4_INFO_RAW("  Heading: %.2f rad\n", (double)msg.heading_rad);
		PX4_INFO_RAW("  Satellites: %d, Fix type: %d\n", msg.satellites_visible, msg.fix_type);
		PX4_INFO_RAW("  Battery: %d%%, RSSI: %d\n", msg.battery_remaining, msg.rssi);
	}

	return 0;
}

void BlackboxData::Run()
{
	update_and_publish();
}

void BlackboxData::update_and_publish()
{
	sensor_gps_s gps{};
	vehicle_global_position_s global_pos{};
	vehicle_local_position_s lpos{};
	battery_status_s battery{};
	home_position_s home{};

	// 先获取必要的消息数据
	bool has_gps = _sensor_gps_sub.copy(&gps);
	bool has_local_pos = _vehicle_local_position_sub.update(&lpos);
	bool has_battery = _battery_status_sub.copy(&battery);

	// 必须有 GPS、local position 和 battery 数据才能继续
	if (!has_gps || !has_local_pos || !has_battery) {
		return;
	}

	_home_position_sub.copy(&home);

	black_box_low_bandwidth_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.time_utc_usec = gps.time_utc_usec;

	// 先用 GPS 消息中的字段填充
	msg.latitude_deg = gps.latitude_deg;
	msg.longitude_deg = gps.longitude_deg;
	msg.altitude_msl_m = gps.altitude_msl_m;
	msg.altitude_ellipsoid_m = gps.altitude_ellipsoid_m;

	// 如果有 global_pos 消息，则覆盖这四个字段
	if (_vehicle_global_position_sub.update(&global_pos)) {
		if (global_pos.lat_lon_valid && global_pos.alt_valid) {
			msg.latitude_deg = global_pos.lat;
			msg.longitude_deg = global_pos.lon;
			msg.altitude_msl_m = global_pos.alt;
			msg.altitude_ellipsoid_m = global_pos.alt_ellipsoid;
		}
	}

	if (home.valid_alt) {
		msg.relative_alt_m = -(lpos.z - home.z);
	} else {
		msg.relative_alt_m = 0.0f;
	}

	msg.vn_m_s = gps.vel_n_m_s;
	msg.ve_m_s = gps.vel_e_m_s;
	msg.vd_m_s = gps.vel_d_m_s;

	msg.heading_rad = lpos.heading;

	msg.satellites_visible = gps.satellites_used;
	msg.fix_type = gps.fix_type;

	msg.battery_remaining = static_cast<uint8_t>(battery.remaining * 100.0f);
	msg.rssi = 0;

	_black_box_low_bw_pub.update(msg);

	// 首次 fix_type >= 3 时启动日志记录
	if (!_logger_started && msg.fix_type >= 3) {
		PX4_INFO("GPS fix type >= 3, starting logger with -f flag");

		// 调用 logger 模块的入口函数
		const char *argv[] = {"logger", "start", "-f", nullptr};
		int ret = logger_main(3, (char **)argv);

		if (ret != 0) {
			PX4_ERR("Failed to start logger (ret=%d), will retry next time", ret);
		} else {
			PX4_INFO("Logger started successfully in boot_until_shutdown mode");
			_logger_started = true;  // 只有成功后才标记为已启动
		}
	}

	_publish_count++;
	_last_publish_time = msg.timestamp;

	const hrt_abstime now = hrt_absolute_time();
	constexpr hrt_abstime rate_measurement_interval = 5000_ms;

	if (now - _last_rate_measurement_time >= rate_measurement_interval) {
		const float dt = (now - _last_rate_measurement_time) / 1e6f;
		_publish_rate = _publish_count / dt;
		_publish_count = 0;
		_last_rate_measurement_time = now;
	}
}

int BlackboxData::task_spawn(int argc, char *argv[])
{
	BlackboxData *instance = new BlackboxData();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int BlackboxData::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BlackboxData::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Blackbox data module for publishing black box low bandwidth data.

Subscribes to sensor_gps and vehicle_local_position topics and publishes
black_box_low_bandwidth at 10Hz.

### Examples
$ blackbox_data start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("blackbox_data", "logger");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int blackbox_data_main(int argc, char *argv[])
{
	return BlackboxData::main(argc, argv);
}
