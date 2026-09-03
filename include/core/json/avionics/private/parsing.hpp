#pragma once
#include <nlohmann/json.hpp>
#include "simulation/avionics/public/components/ahrs.hpp"
#include "simulation/avionics/public/manager.hpp"

namespace json {

	avionics::AttitudeHeadingReferenceSystem parse_attitude_heading_reference_system(const nlohmann::json& ahrs_json);
	avionics::AvionicsManager parse_avionics_manager(const nlohmann::json& config);

} // namespace json
