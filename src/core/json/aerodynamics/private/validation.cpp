#include "core/json/aerodynamics/private/validation.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>

namespace json
{

	void validate_surfaces_json(
		const nlohmann::json& surfaces_json)
	{
		if (!surfaces_json.is_array()) {
			throw std::runtime_error("json::parse_aerodynamics_manager expected 'surfaces' to be an array");
		}
	}

} // namespace json
