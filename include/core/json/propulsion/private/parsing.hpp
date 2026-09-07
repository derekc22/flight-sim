#pragma once
#include "simulation/propulsion/public/manager.hpp"
#include "simulation/structural/public/manager.hpp"

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <string>

namespace json
{

	propulsion::PropellerAssembly parse_propellers(const nlohmann::json& propellers_json,
		structural::StructuralManager& structural_manager,
		const Eigen::Vector3d& n_prop);

	propulsion::PropulsorEffector parse_propulsor_effector(const nlohmann::json& config,
		const std::string& key,
		structural::StructuralManager& structural_manager);

	propulsion::PropulsionManager parse_propulsion_manager(const nlohmann::json& config,
		structural::StructuralManager& structural_manager);

} // namespace json
