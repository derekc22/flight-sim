#include "core/json/propulsion/private/parsing.hpp"

#include "core/json/propulsion/private/validation.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/propulsion/public/manager.hpp"
#include "simulation/structural/public/manager.hpp"
#include "simulation/transforms/public/detail/so3.hpp"

#include <algorithm>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace json
{

	propulsion::PropellerAssembly parse_propellers(
		const nlohmann::json& propellers_json,
		structural::StructuralManager& structural_manager,
		const Eigen::Vector3d& n_prop)
	{
		validate_propellers_json(propellers_json);

		std::vector<std::string> geometry_ids = propellers_json.at("geometry_ids").get<std::vector<std::string>>();
		double y_min = std::numeric_limits<double>::infinity();
		double y_max = -std::numeric_limits<double>::infinity();
		double spin_inertia = 0.0;

		for (const std::string& geometry_id : geometry_ids) {
			const structural::Geometry& geom = structural_manager.get_geometry(geometry_id);
			y_min = std::min(y_min, geom.pB_geomB(1) - 0.5 * geom.y_size);
			y_max = std::max(y_max, geom.pB_geomB(1) + 0.5 * geom.y_size);
			spin_inertia += structural_manager.compute_spin_inertia(geom, n_prop);
		}

		const double diameter = y_max - y_min;
		if (diameter < constants::eps) {
			throw std::runtime_error("json::parse_propellers: computed propeller diameter must be positive");
		}
		if (spin_inertia < constants::eps) {
			throw std::runtime_error("json::parse_propellers: computed propeller spin inertia must be positive");
		}

		return {
			.spin_sign = propellers_json.at("spin_sign").get<double>(),
			.thrust_coeff = propellers_json.at("thrust_coeff").get<double>(),
			.torque_coeff = propellers_json.at("torque_coeff").get<double>(),
			.diameter = diameter,
			.spin_inertia = spin_inertia
		};
	}

	propulsion::PropulsorEffector parse_propulsor_effector(
		const nlohmann::json& config,
		const std::string& key,
		structural::StructuralManager& structural_manager)
	{
		if (config.at(key).is_null()) {
			return {};
		}

		const auto& propulsor_effector_json = config.at(key);
		validate_propulsor_effector_json(propulsor_effector_json);

		std::string geometry_id = propulsor_effector_json.at("geometry_id").get<std::string>();
		const structural::Geometry& geom = structural_manager.get_geometry(geometry_id);
		validate_propulsor_effector_placement(geom.pB_geomB, key);

		const double inclination_angle = propulsor_effector_json.at("inclination_angle").get<double>();
		const double toe_angle = propulsor_effector_json.at("toe_angle").get<double>();
		const Eigen::Matrix3d RBP =
			transforms::eul_to_R(toe_angle, inclination_angle, 0.0, transforms::EulerOrder::ZYX);

		propulsion::PropulsorEffector propulsor_effector{.pB_propB = geom.pB_geomB, .n_prop = RBP * constants::ei};

		if (propulsor_effector_json.contains("propellers")) {
			propulsor_effector.propellers = parse_propellers(
				propulsor_effector_json.at("propellers"), structural_manager, propulsor_effector.n_prop);
		}

		return propulsor_effector;
	}

	propulsion::PropulsionManager parse_propulsion_manager(
		const nlohmann::json& config,
		structural::StructuralManager& structural_manager)
	{
		const auto& effectors_json = config.at("effectors");

		return {
			.propulsor_effectors = {
				.front_propulsor = parse_propulsor_effector(effectors_json, "front_propulsor", structural_manager),
				.left_propulsor = parse_propulsor_effector(effectors_json, "left_propulsor", structural_manager),
				.right_propulsor = parse_propulsor_effector(effectors_json, "right_propulsor", structural_manager),
			}
		};
	}

} // namespace json
