#pragma once
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <string>

namespace json
{

	void validate_propulsor_effector_json(const nlohmann::json& propulsor_effector_json);
	void validate_propulsor_effector_placement(const Eigen::Vector3d& p_propulsor, const std::string& key);
	void validate_propellers_json(const nlohmann::json& propellers_json);

} // namespace json
