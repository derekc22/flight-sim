#pragma once
#include <nlohmann/json.hpp>

namespace json
{

	void validate_actuator_json(const nlohmann::json& actuator_json);
	void validate_propulsor_actuator_json(const nlohmann::json& propulsor_actuator_json);

} // namespace json
