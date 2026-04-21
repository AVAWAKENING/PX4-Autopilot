/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef GNSS_LOW_BANDWIDTH_POSITION_HPP
#define GNSS_LOW_BANDWIDTH_POSITION_HPP

#include <uORB/topics/battery_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamGnssLowBandwidthPosition : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGnssLowBandwidthPosition(mavlink); }

	static constexpr const char *get_name_static() { return "GNSS_LOW_BANDWIDTH_POSITION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GNSS_LOW_BANDWIDTH_POSITION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_GNSS_LOW_BANDWIDTH_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamGnssLowBandwidthPosition(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	bool send() override
	{
		sensor_gps_s gps;
		vehicle_local_position_s lpos;

		if (_gps_sub.update(&gps) && _lpos_sub.update(&lpos)) {

			mavlink_gnss_low_bandwidth_position_t msg{};

			msg.lat = gps.latitude_deg * 1e7;
			msg.lon = gps.longitude_deg * 1e7;
			msg.alt = gps.altitude_msl_m * 1000;
			msg.altitude_ellipsoid_mm = static_cast<int32_t>(gps.altitude_ellipsoid_m * 1000.0);

			if (lpos.z_valid) {
				home_position_s home{};
				_home_sub.copy(&home);

				if (home.valid_alt) {
					msg.relative_alt = -(lpos.z - home.z) * 1000;

				} else {
					msg.relative_alt = -lpos.z * 1000;
				}

			} else {
				msg.relative_alt = 0;
			}

			msg.vn = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy) * 100.0f; // 地速：水平速度大小
		msg.ve = lpos.vz * 100.0f;                                      // 垂直速度：向下速度
		msg.vd = -lpos.vx * 100.0f;                                     // 北向速度：NED 北向速度

		msg.heading = static_cast<uint16_t>(math::degrees(lpos.heading) * 100.0f);

			msg.satellites_visible = gps.satellites_used;
			msg.fix_type = gps.fix_type;

			battery_status_s battery;
			if (_battery_sub.copy(&battery) && battery.remaining >= 0.0f) {
				msg.battery_remaining = static_cast<uint8_t>(battery.remaining * 100.0f);
			} else {
				msg.battery_remaining = UINT8_MAX;
			}

			mavlink_msg_gnss_low_bandwidth_position_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // GNSS_LOW_BANDWIDTH_POSITION_HPP
