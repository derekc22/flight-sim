#include <nlohmann/json.hpp>
#include "core/json/avionics/private/parsing.hpp"
#include "core/json/avionics/private/validation.hpp"
#include "simulation/avionics/public/components/ahrs.hpp"
#include "simulation/avionics/public/manager.hpp"

namespace json {

	avionics::AttitudeHeadingReferenceSystem parse_attitude_heading_reference_system(
	    const nlohmann::json& ahrs_json)
	{
		validate_attitude_heading_reference_system_json(ahrs_json);

		return {.Kp = ahrs_json.at("Kp").get<double>(),
		    .Ki = ahrs_json.at("Ki").get<double>(),
		    .fB_tol_scale = ahrs_json.at("fB_tol_scale").get<double>()};
	}

	avionics::AvionicsManager parse_avionics_manager(
	    const nlohmann::json& config)
	{
		const auto& ahrs_json = config.at("attitude_heading_reference_system");

		avionics::Avionics avionics = {.AHRS = parse_attitude_heading_reference_system(ahrs_json)};

		return {.avionics = avionics};
	}

} // namespace json
