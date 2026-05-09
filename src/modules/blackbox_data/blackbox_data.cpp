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

	if (_sensor_gps_sub.update(&gps) && _vehicle_global_position_sub.update(&global_pos) && _vehicle_local_position_sub.update(&lpos) && _battery_status_sub.update(&battery)) {
		if (!global_pos.lat_lon_valid || !global_pos.alt_valid) {
			return;
		}

		_home_position_sub.copy(&home);

		black_box_low_bandwidth_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.time_utc_usec = gps.time_utc_usec;

		msg.latitude_deg = global_pos.lat;
		msg.longitude_deg = global_pos.lon;
		msg.altitude_msl_m = global_pos.alt;
		msg.altitude_ellipsoid_m = global_pos.alt_ellipsoid;

		if (home.valid_alt) {
			msg.relative_alt_m = -(lpos.z - home.z);
		} else {
			msg.relative_alt_m = 0.0f;
		}

		msg.vn_m_s = lpos.vx;
		msg.ve_m_s = lpos.vy;
		msg.vd_m_s = lpos.vz;

		msg.heading_rad = lpos.heading;

		msg.satellites_visible = gps.satellites_used;
		msg.fix_type = gps.fix_type;

		msg.battery_remaining = static_cast<uint8_t>(battery.remaining * 100.0f);
		msg.rssi = 0;

		_black_box_low_bw_pub.publish(msg);
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
