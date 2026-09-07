#pragma once
#include <nlohmann/json.hpp>

namespace json
{

	void validate_surfaces_json(const nlohmann::json& surfaces_json);
	void validate_surface_effectors_json(const nlohmann::json& effectors_json);

} // namespace json
