#pragma once
#include "core/connection/public/udp.hpp"
#include "core/messages/public/data/helpers.hpp"
#include "simulation/runner/public/data/types.hpp"

namespace runner
{

	struct FlightGearAdapter {
		// initialize udp out cache
		messages::ProcessedFlightGearMessageOut cached_msg_out{};

		connection::UDPOut udp_out;
		connection::UDPIn udp_in;

		FlightGearAdapter();

		/**
		 * @brief Receives FlightGear data and returns the current wind vectors.
		 *
		 * The most recent successfully received message remains cached. When wind is
		 * disabled, both returned wind vectors are zero.
		 *
		 * @param[in] input Aircraft frame data and wind-enable flag.
		 * @return Wind expressed in the inertial and body frames.
		 */
		FlightGearAdapterOutput receive(const FlightGearAdapterInput& input);

		/**
		 * @brief Sends the current simulation position and attitude to FlightGear.
		 *
		 * @param[in] input Geographic state and NED-relative Euler angles to send.
		 */
		void send(const FlightGearAdapterSendInput& input);
	};

} // namespace runner
