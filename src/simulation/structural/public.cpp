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
#include "simulation/util/public.hpp"

namespace structural {

    StructuralProperties::StructuralProperties(const std::vector<Geometry>& g) : geometries(g) {
        mass = dynamics::Mass{ compute_mass() };
        CG = dynamics::CenterOfGravity{ compute_CG() };
        JB = dynamics::InertiaTensor{ compute_JB() };
        geometryIDs = build_IDs();
    }

    Geometry& StructuralProperties::get_geometry(const std::string& id) {
        const auto it = geometryIDs.find(id);
        if (it == geometryIDs.end()) { 
            throw std::runtime_error("structural::StructuralProperties::get_geometry: geometry id not found: " + id); 
        }
        return geometries[it->second];
    }

    double StructuralProperties::compute_mass() {
        double m = 0.0;
        for (const Geometry& geom : geometries) {
            m += geom.mass;
        }
        if (m < constants::eps) { 
            throw std::runtime_error("structural::StructuralProperties::compute_mass: mass must be positive"); 
        }
        return m;
    }

    Eigen::Vector3d StructuralProperties::compute_CG() {
        Eigen::Vector3d cg = constants::Zero3;
        for (const Geometry& geom : geometries) {
            cg(0) += geom.mass * geom.x_loc;
            cg(1) += geom.mass * geom.y_loc;
            cg(2) += geom.mass * geom.z_loc;
        }
        cg /= mass.data;
        return cg;
    }

    Eigen::Matrix3d StructuralProperties::compute_JB() {
        Eigen::Matrix3d j = constants::Zero3x3;

        for (const Geometry& geom : geometries) {
            double m = geom.mass;
            Eigen::Matrix3d j_local = compute_local_JB(geom);

            // Distance from geometry CG to system CG
            double dx = geom.x_loc - CG.data(0);
            double dy = geom.y_loc - CG.data(1);
            double dz = geom.z_loc - CG.data(2);

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
            throw std::runtime_error("structural::StructuralProperties::compute_JB: Inertia tensor is singular"); 
        }

        return j;
    }

    Eigen::Matrix3d StructuralProperties::compute_local_JB(const Geometry& geom) {
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

    double StructuralProperties::compute_spin_inertia(const Geometry& geom, const Eigen::Vector3d& axis) {
        Eigen::Vector3d axis_hat = util::norm(axis);
        if (axis_hat.norm() < constants::eps) { 
            throw std::runtime_error("structural::StructuralProperties::compute_spin_inertia: spin axis cannot be zero"); 
        }
        return axis_hat.dot(compute_local_JB(geom) * axis_hat);
    }

    std::unordered_map<std::string, size_t> StructuralProperties::build_IDs() {
        std::unordered_map<std::string, size_t> m;
        for (size_t i = 0; i < geometries.size(); ++i) {
            m[geometries[i].id] = i;
        }
        return m;
    }

}
