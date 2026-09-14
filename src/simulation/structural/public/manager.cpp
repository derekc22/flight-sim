#include "simulation/structural/public/manager.hpp"

#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/dynamics/public/data/types.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace structural
{

	StructuralManager::StructuralManager(
		const std::vector<Geometry>& geoms)
		: geometries(geoms)
	{
		geometry_id_map = build_geometry_id_map();
	}

	Geometry& StructuralManager::get_geometry(
		const std::string& id)
	{
		const auto it = geometry_id_map.find(id);
		if (it == geometry_id_map.end()) {
			throw std::runtime_error("structural::StructuralManager::get_geometry: geometry id not found: " + id);
		}
		return geometries[it->second];
	}

	double StructuralManager::compute_mass()
	{
		double m = 0.0;
		for (const Geometry& geom : geometries) {
			m += geom.mass;
		}
		if (m < constants::eps) {
			throw std::runtime_error("structural::StructuralManager::compute_mass: mass must be positive");
		}
		return m;
	}

	Eigen::Vector3d StructuralManager::compute_CG(
		const dynamics::Mass& mass)
	{
		Eigen::Vector3d pB_GB = constants::Zero3;
		for (const Geometry& geom : geometries) {
			pB_GB += geom.mass * geom.pB_geomB;
		}
		pB_GB /= mass.data;
		return pB_GB;
	}

	Eigen::Matrix3d StructuralManager::compute_JB_G(
		const CenterOfGravity& pB_GB)
	{
		Eigen::Matrix3d J = constants::Zero3x3;

		for (Geometry& geom : geometries) {
			Eigen::Matrix3d j_localL = geom.compute_local_J();

			// Express geometry-local inertia tensor in body frame
			Eigen::Matrix3d j_localB = geom.CBL.transpose() * j_localL * geom.CBL;

			// Distance from geometry CG to system CG
			Eigen::Vector3d d = geom.pB_geomB - pB_GB.data;

			// Parallel axis theorem
			J += j_localB + geom.mass * (d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose());
		}

		if (std::abs(J.determinant()) < constants::eps) {
			throw std::runtime_error("structural::StructuralManager::compute_JB_G: Inertia tensor is singular");
		}

		return J;
	}

	std::unordered_map<std::string, std::size_t> StructuralManager::build_geometry_id_map()
	{
		std::unordered_map<std::string, std::size_t> m;
		for (std::size_t i = 0; i < geometries.size(); ++i) {
			m[geometries[i].id] = i;
		}
		return m;
	}

	StructuralManagerOutput StructuralManager::step(
		const StructuralManagerInput&)
	{
		dynamics::Mass mass = dynamics::Mass{compute_mass()};
		CenterOfGravity pB_GB = CenterOfGravity{compute_CG(mass)};
		dynamics::InertiaTensor JB_G = dynamics::InertiaTensor{compute_JB_G(pB_GB)};
		return {.struc_t = {mass, pB_GB, JB_G}};
	}

} // namespace structural
