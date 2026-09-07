#pragma once
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/aerodynamics/public/manager.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace json
{

	aerodynamics::DynamicDerivative parse_dynamic_derivative(const nlohmann::json& derivative_json);
	aerodynamics::DynamicDerivatives parse_dynamic_derivatives(const nlohmann::json& derivatives_json);
	actuators::SurfaceActuatorID parse_surface_actuator_id(const std::string& actuator_id);
	aerodynamics::SurfaceEffector parse_surface_effector(const nlohmann::json& effector_json);
	std::vector<aerodynamics::SurfaceEffector> parse_surface_effectors(const nlohmann::json& effectors_json);
	aerodynamics::AerodynamicsManager parse_aerodynamics_manager(const nlohmann::json& config);

} // namespace json
