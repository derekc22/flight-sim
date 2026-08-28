#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <string>
#include <unordered_map>
#include <cstddef>
#include "simulation/structural/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/linalg/public.hpp"

namespace structural {

    StructuralManager::StructuralManager(const std::vector<Geometry>& geoms) : geometries(geoms) {
        geometry_id_map = build_geometry_id_map();
    }

    Geometry& StructuralManager::get_geometry(const std::string& id) {
        const auto it = geometry_id_map.find(id);
        if (it == geometry_id_map.end()) { 
            throw std::runtime_error("structural::StructuralManager::get_geometry: geometry id not found: " + id);
        }
        return geometries[it->second];
    }

    double StructuralManager::compute_mass() {
        double m = 0.0;
        for (const Geometry& geom : geometries) {
            m += geom.mass;
        }
        if (m < constants::eps) { 
            throw std::runtime_error("structural::StructuralManager::compute_mass: mass must be positive");
        }
        return m;
    }

    Eigen::Vector3d StructuralManager::compute_CG(const dynamics::Mass& mass) {
        Eigen::Vector3d pB_GB = constants::Zero3;
        for (const Geometry& geom : geometries) {
            pB_GB += geom.mass * geom.pB_geomB;
        }
        pB_GB /= mass.data;
        return pB_GB;
    }

    Eigen::Matrix3d StructuralManager::compute_JB(const CenterOfGravity& pB_GB) {
        Eigen::Matrix3d j = constants::Zero3x3;

        for (const Geometry& geom : geometries) {
            double m = geom.mass;
            Eigen::Matrix3d j_local = compute_local_JB(geom);

            // Distance from geometry CG to system CG
            double dx = geom.pB_geomB(0) - pB_GB.data(0);
            double dy = geom.pB_geomB(1) - pB_GB.data(1);
            double dz = geom.pB_geomB(2) - pB_GB.data(2);

            // Parallel axis theorem
            j(0, 0) += j_local(0, 0) + m * (dy * dy + dz * dz);   // Jxx
            j(1, 1) += j_local(1, 1) + m * (dx * dx + dz * dz);   // Jyy
            j(2, 2) += j_local(2, 2) + m * (dx * dx + dy * dy);   // Jzz

            // Off-diagonal terms (products of inertia)
            j(0, 1) += -m * dx * dy;
            j(0, 2) += -m * dx * dz;
            j(1, 2) += -m * dy * dz;
        }

        // Symmetric
        j(1, 0) = j(0, 1);
        j(2, 0) = j(0, 2);
        j(2, 1) = j(1, 2);

        double detj = j.determinant();
        if (std::abs(detj) < constants::eps) { 
            throw std::runtime_error("structural::StructuralManager::compute_JB: Inertia tensor is singular");
        }

        return j;
    }

    Eigen::Matrix3d StructuralManager::compute_local_JB(const Geometry& geom) {
        double m = geom.mass;
        double lx = geom.x_size;
        double ly = geom.y_size;
        double lz = geom.z_size;

        // Moments of inertia of rectangular prism about its own center
        Eigen::Matrix3d j = constants::Zero3x3;
        j(0, 0) = (1.0 / 12.0) * m * (ly * ly + lz * lz);   // Jxx
        j(1, 1) = (1.0 / 12.0) * m * (lx * lx + lz * lz);   // Jyy
        j(2, 2) = (1.0 / 12.0) * m * (lx * lx + ly * ly);   // Jzz    
        return j;
    }

    double StructuralManager::compute_spin_inertia(const Geometry& geom, const Eigen::Vector3d& axis) {
        Eigen::Vector3d axis_hat = util::norm(axis);
        if (axis_hat.norm() < constants::eps) { 
            throw std::runtime_error("structural::StructuralManager::compute_spin_inertia: spin axis cannot be zero");
        }
        return axis_hat.dot(compute_local_JB(geom) * axis_hat);
    }

    std::unordered_map<std::string, std::size_t> StructuralManager::build_geometry_id_map() {
        std::unordered_map<std::string, std::size_t> m;
        for (std::size_t i = 0; i < geometries.size(); ++i) {
            m[geometries[i].id] = i;
        }
        return m;
    }

    StructuralState StructuralManager::compute_structural_state() {
        dynamics::Mass mass = dynamics::Mass{ compute_mass() };
        CenterOfGravity pB_GB = CenterOfGravity{ compute_CG(mass) };
        dynamics::InertiaTensor JB = dynamics::InertiaTensor{ compute_JB(pB_GB) };
        return { mass, pB_GB, JB };
    }

}
