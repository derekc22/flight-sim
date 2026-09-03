#pragma once
#include "core/connection/public/udp.hpp"
#include "core/messages/public/data/helpers.hpp"
#include "simulation/runner/public/data/types.hpp"

namespace runner {

	struct FlightGearAdapter {
		// initialize udp out cache
		messages::ProcessedFlightGearMessageOut cached_msg_out{};

		connection::UDPOut udp_out;
		connection::UDPIn udp_in;

		FlightGearAdapter();
		FlightGearAdapterOutput receive(const FlightGearAdapterInput& input);
		void send(const FlightGearAdapterSendInput& input);
	};

} // namespace runner
