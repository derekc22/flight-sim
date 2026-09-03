#pragma once
#include <nlohmann/json.hpp>
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/aerodynamics/public/manager.hpp"

namespace json {

	aerodynamics::DynamicDerivatives parse_dynamic_derivatives(const nlohmann::json& dyn_json);
	aerodynamics::ControlDerivatives parse_control_derivatives(const nlohmann::json& ctrl_json);
	aerodynamics::AerodynamicsManager parse_aerodynamics_manager(const nlohmann::json& config);

} // namespace json
