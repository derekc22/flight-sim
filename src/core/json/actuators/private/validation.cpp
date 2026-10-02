#include "core/json/actuators/private/validation.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>

namespace json
{

	void validate_actuator_json(
		const nlohmann::json& actuator_json)
	{
		if (!actuator_json.contains("limit_max")) {
			throw std::runtime_error("json::validate_actuator_json: actuator maximum limit not present");
		}
		if (!actuator_json.contains("limit_min")) {
			throw std::runtime_error("json::validate_actuator_json: actuator minimum limit not present");
		}
		if (!actuator_json.contains("wn")) {
			throw std::runtime_error("json::validate_actuator_json: actuator wn not present");
		}
		if (!actuator_json.contains("zeta")) {
			throw std::runtime_error("json::validate_actuator_json: actuator zeta not present");
		}

		double limit_max = actuator_json.at("limit_max").get<double>();
		double limit_min = actuator_json.at("limit_min").get<double>();
		double wn = actuator_json.at("wn").get<double>();
		double zeta = actuator_json.at("zeta").get<double>();

		if (limit_max < limit_min) {
			throw std::runtime_error(
				"json::validate_actuator_json: actuator maximum limit must be greater than or equal to minimum limit");
		}
		if (wn <= 0.0) {
			throw std::runtime_error("json::validate_actuator_json: actuator wn must be positive");
		}
		if (zeta < 0.0) {
			throw std::runtime_error("json::validate_actuator_json: actuator zeta must be non-negative");
		}
	}

	void validate_propulsor_actuator_json(
		const nlohmann::json& propulsor_actuator_json)
	{
		validate_actuator_json(propulsor_actuator_json);

		double limit_min = propulsor_actuator_json.at("limit_min").get<double>();
		if (limit_min < 0.0) {
			throw std::runtime_error(
				"json::validate_propulsor_actuator_json: propulsor limit_min must be non-negative");
		}
	}

} // namespace json
