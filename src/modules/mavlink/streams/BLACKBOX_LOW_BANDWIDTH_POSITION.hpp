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

#ifndef BLACKBOX_LOW_BANDWIDTH_POSITION_HPP
#define BLACKBOX_LOW_BANDWIDTH_POSITION_HPP

#include <uORB/topics/black_box_low_bandwidth.h>
#include <mathlib/mathlib.h>

class MavlinkStreamBlackboxLowBandwidthPosition : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamBlackboxLowBandwidthPosition(mavlink); }

	static constexpr const char *get_name_static() { return "BLACKBOX_LOW_BANDWIDTH_POSITION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_BLACKBOX_LOW_BANDWIDTH_POSITION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _blackbox_sub.advertised() ? MAVLINK_MSG_ID_BLACKBOX_LOW_BANDWIDTH_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamBlackboxLowBandwidthPosition(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _blackbox_sub{ORB_ID(black_box_low_bandwidth)};

	bool send() override
	{
		black_box_low_bandwidth_s blackbox;

		if (_blackbox_sub.update(&blackbox)) {

			mavlink_blackbox_low_bandwidth_position_t msg{};

			msg.lat = (int32_t)(blackbox.latitude_deg * 1e7);
			msg.lon = (int32_t)(blackbox.longitude_deg * 1e7);
			msg.altitude_amsl = (int32_t)(blackbox.altitude_msl_m * 1000.0f);
			msg.altitude_ellipsoid = (int32_t)(blackbox.altitude_ellipsoid_m * 1000.0f);
			msg.relative_alt = (int32_t)(blackbox.relative_alt_m * 1000.0f);

			float ground_speed = sqrtf(blackbox.vn_m_s * blackbox.vn_m_s + blackbox.ve_m_s * blackbox.ve_m_s);
			msg.ground_velocity = (int16_t)(ground_speed * 100.0f);
			msg.vertical_velocity = (int16_t)(blackbox.vd_m_s * 100.0f);

			float south_velocity = -blackbox.vn_m_s;
			msg.south_velocity = (int16_t)(south_velocity * 100.0f);

			float heading_deg = math::degrees(blackbox.heading_rad);
			if (heading_deg < 0.0f) {
				heading_deg += 360.0f;
			}
			msg.heading = (uint16_t)(heading_deg * 100.0f);

			mavlink_msg_blackbox_low_bandwidth_position_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // BLACKBOX_LOW_BANDWIDTH_POSITION_HPP
