#include "core/json/propulsion/private/parsing.hpp"

#include "core/json/propulsion/private/validation.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/propulsion/public/manager.hpp"
#include "simulation/structural/public/manager.hpp"

#include <algorithm>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace json
{

	propulsion::PropellerAssembly parse_propellers(
		const nlohmann::json& propellers_json,
		structural::StructuralManager& structural_manager,
		const Eigen::Vector3d& pB_propB,
		const Eigen::Vector3d& n_prop)
	{
		validate_propellers_json(propellers_json);

		std::vector<std::string> geometry_ids = propellers_json.at("geometry_ids").get<std::vector<std::string>>();
		double radius = 0.0;
		double spin_inertia = 0.0;

		for (const std::string& geometry_id : geometry_ids) {
			structural::Geometry& geom = structural_manager.get_geometry(geometry_id);
			// Find the maximum radial extent of the geometry about the propeller spin axis
			Eigen::Vector3d half_size{0.5 * geom.x_size, 0.5 * geom.y_size, 0.5 * geom.z_size};
			for (double x_sign : {-1.0, 1.0}) {
				for (double y_sign : {-1.0, 1.0}) {
					for (double z_sign : {-1.0, 1.0}) {
						Eigen::Vector3d cornerL{x_sign * half_size(0), y_sign * half_size(1), z_sign * half_size(2)};
						Eigen::Vector3d cornerB = geom.pB_geomB + geom.CBL.transpose() * cornerL;
						Eigen::Vector3d offsetB = cornerB - pB_propB;
						radius = std::max(radius, offsetB.cross(n_prop).norm());
					}
				}
			}
			Eigen::Vector3d center_offsetB = geom.pB_geomB - pB_propB;
			spin_inertia += geom.compute_spin_inertia(n_prop) + geom.mass * center_offsetB.cross(n_prop).squaredNorm();
		}

		const double diameter = 2.0 * radius;
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

		propulsion::PropulsorEffector propulsor_effector{
			.pB_propB = geom.pB_geomB, .n_prop = geom.CBL.transpose() * constants::ei
		};

		if (propulsor_effector_json.contains("propellers")) {
			propulsor_effector.propellers = parse_propellers(propulsor_effector_json.at("propellers"),
				structural_manager,
				propulsor_effector.pB_propB,
				propulsor_effector.n_prop);
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
