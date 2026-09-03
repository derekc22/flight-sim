#include <stdexcept>
#include <nlohmann/json.hpp>
#include "core/json/avionics/private/validation.hpp"

namespace json {

	void validate_attitude_heading_reference_system_json(
	    const nlohmann::json& ahrs_json)
	{
		if (!ahrs_json.is_object()) {
			throw std::runtime_error("json::validate_attitude_heading_reference_system_json expected "
			                         "attitude_heading_reference_system object");
		}
		if (!ahrs_json.contains("Kp")) {
			throw std::runtime_error("json::validate_attitude_heading_reference_system_json: Kp not present");
		}
		if (!ahrs_json.contains("Ki")) {
			throw std::runtime_error("json::validate_attitude_heading_reference_system_json: Ki not present");
		}
		if (!ahrs_json.contains("fB_tol_scale")) {
			throw std::runtime_error("json::validate_attitude_heading_reference_system_json: fB_tol_scale not present");
		}
	}

} // namespace json
