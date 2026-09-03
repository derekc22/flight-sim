#pragma once
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "simulation/actuators/public/manager.hpp"
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/structural/public/manager.hpp"

namespace json {

	actuators::PropellerAssembly parse_propellers(const nlohmann::json& propellers_json,
	    structural::StructuralManager& structural_manager,
	    const Eigen::Vector3d& n_prop);

	template <typename SurfaceActuatorType>
	SurfaceActuatorType parse_surface_actuator(const nlohmann::json& config, const std::string& key);

	template <typename PropulsorActuatorType>
	PropulsorActuatorType parse_propulsor_actuator(const nlohmann::json& config,
	    const std::string& key,
	    structural::StructuralManager& structural_manager);

	actuators::ActuatorManager parse_actuator_manager(const nlohmann::json& config,
	    structural::StructuralManager& structural_manager);
} // namespace json

#include "core/json/actuators/private/parsing.tpp"
