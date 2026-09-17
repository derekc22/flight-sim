#pragma once
#include "simulation/structural/public/components/geometry.hpp"
#include "simulation/structural/public/data/types.hpp"

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace structural
{

	struct StructuralManager {
		std::vector<Geometry> geometries;
		std::unordered_map<std::string, size_t> geometry_id_map;

		StructuralManager(const std::vector<Geometry>& geoms);

		/**
		 * @brief Builds a map from geometry identifiers to vector indices.
		 *
		 * If identifiers are duplicated, the last matching geometry determines the stored index.
		 *
		 * @return Map from each geometry identifier to its index in `geometries`.
		 */
		std::unordered_map<std::string, size_t> build_geometry_id_map();

		/**
		 * @brief Returns mutable access to a geometry selected by identifier.
		 *
		 * @param[in] id Geometry identifier.
		 * @return Mutable reference to the selected geometry.
		 */
		Geometry& get_geometry(const std::string& id);

		/**
		 * @brief Computes total mass from all component geometries.
		 *
		 * @return Sum of component masses [kg].
		 */
		double compute_mass();

		/**
		 * @brief Computes the combined center of gravity in body axes.
		 *
		 * @param[in] mass Total mass used to normalize the mass-weighted positions [kg].
		 * @return Position of the combined center of gravity relative to the body origin, expressed in body axes [m].
		 */
		Eigen::Vector3d compute_CG(const dynamics::Mass& mass);

		/**
		 * @brief Computes the combined inertia tensor about the center of gravity.
		 *
		 * Rotates each local inertia tensor into body axes and applies the parallel-axis theorem.
		 *
		 * @param[in] pB_GB Combined center-of-gravity position expressed in body axes [m].
		 * @return Combined inertia tensor about the center of gravity, expressed in body axes [kg m^2].
		 */
		Eigen::Matrix3d compute_JB_G(const CenterOfGravity& pB_GB);

		/**
		 * @brief Computes the current structural state.
		 *
		 * @param[in] input Structural manager input; currently unused.
		 * @return Total mass, center-of-gravity position, and inertia tensor.
		 */
		StructuralManagerOutput step(const StructuralManagerInput& input);
	};

} // namespace structural
