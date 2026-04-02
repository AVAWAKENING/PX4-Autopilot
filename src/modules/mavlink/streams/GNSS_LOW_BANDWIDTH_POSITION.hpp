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

#ifndef GNSS_LOW_BANDWIDTH_POSITION_HPP
#define GNSS_LOW_BANDWIDTH_POSITION_HPP

#include <uORB/topics/gnss_low_bandwidth_position.h>

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
		return _gnss_low_bandwidth_sub.advertised() ? MAVLINK_MSG_ID_GNSS_LOW_BANDWIDTH_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamGnssLowBandwidthPosition(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _gnss_low_bandwidth_sub{ORB_ID(gnss_low_bandwidth_position)};

	bool send() override
	{
		gnss_low_bandwidth_position_s gnss_data;

		if (_gnss_low_bandwidth_sub.update(&gnss_data)) {
			mavlink_gnss_low_bandwidth_position_t msg{};

			msg.lat = gnss_data.lat;
			msg.lon = gnss_data.lon;
			msg.alt = gnss_data.alt;
			msg.vn = gnss_data.vn;
			msg.ve = gnss_data.ve;
			msg.vd = gnss_data.vd;
			msg.heading = gnss_data.heading;
			msg.satellites_visible = gnss_data.satellites;
			msg.fix_type = gnss_data.fix_type;

			mavlink_msg_gnss_low_bandwidth_position_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // GNSS_LOW_BANDWIDTH_POSITION_HPP
