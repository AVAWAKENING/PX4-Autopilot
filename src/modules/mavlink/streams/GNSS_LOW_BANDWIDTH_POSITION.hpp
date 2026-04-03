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

#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>

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
		return _gps_sub.advertised() ? MAVLINK_MSG_ID_GNSS_LOW_BANDWIDTH_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamGnssLowBandwidthPosition(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};

	bool send() override
	{
		sensor_gps_s gps;
		vehicle_local_position_s lpos;
		vehicle_global_position_s gpos;

		if (_gps_sub.update(&gps) && _lpos_sub.update(&lpos) && _gpos_sub.update(&gpos)) {

			mavlink_gnss_low_bandwidth_position_t msg{};

			msg.lat = gps.latitude_deg * 1e7;
			msg.lon = gps.longitude_deg * 1e7;
			msg.alt = gps.altitude_msl_m * 1000;

			if (gpos.terrain_alt_valid) {
				msg.relative_alt = ((float)gps.altitude_msl_m - (float)gpos.terrain_alt) * 1000;

			} else if (lpos.dist_bottom_valid) {
				msg.relative_alt = lpos.dist_bottom * 1000;

			} else {
				msg.relative_alt = 0;
			}

			msg.vn = fabsf(lpos.vx) * 100.0f;
			msg.ve = fabsf(lpos.vy) * 100.0f;
			msg.vd = fabsf(lpos.vz) * 100.0f;

			msg.heading = static_cast<uint16_t>(math::degrees(matrix::wrap_2pi(lpos.heading)) * 100.0f);

			msg.satellites_visible = gps.satellites_used;
			msg.fix_type = gps.fix_type;

			mavlink_msg_gnss_low_bandwidth_position_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // GNSS_LOW_BANDWIDTH_POSITION_HPP
