#pragma once
#include "core/messages/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"

namespace messages {

	FlightGearMessageIn process_in_msg(const geography::GeographicState& geo, const dynamics::EulerAngles& attitude);
	ProcessedFlightGearMessageOut process_out_msg(const FlightGearMessageOut& out_msg);

} // namespace messages
