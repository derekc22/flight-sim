#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <string>
#include <unordered_map>
#include "simulation/structural/structural.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"

namespace structural {

    StructuralProperties::StructuralProperties(std::vector<Geometry> g) : geometries(g) {
        Mass = dynamics::Mass{ compute_Mass() };
        CG = dynamics::CenterOfGravity{ compute_CG() };
        J = dynamics::InertiaTensor{ compute_J() };
        geometryIDs = build_IDs();
    }

    double StructuralProperties::compute_Mass() {
        double m = 0.0;
        for (const Geometry& geom : geometries) {
            m += geom.mass;
        }
        if (m < global::eps) { throw std::runtime_error("structural::StructuralProperties::compute_Mass: Mass must be positive"); }
        return m;
    }

    Eigen::Vector3d StructuralProperties::compute_CG() {
        Eigen::Vector3d cg = Eigen::Vector3d::Zero();
        for (const Geometry& geom : geometries) {
            cg(0) += geom.mass * geom.x_loc;
            cg(1) += geom.mass * geom.y_loc;
            cg(2) += geom.mass * geom.z_loc;
        }
        cg /= Mass.data;
        return cg;
    }

    Eigen::Matrix3d StructuralProperties::compute_J() {
        Eigen::Matrix3d j = Eigen::Matrix3d::Zero();

        for (const Geometry& geom : geometries) {
            double m = geom.mass;
            double lx = geom.x_size;
            double ly = geom.y_size;
            double lz = geom.z_size;

            // Moments of inertia of rectangular prism about its own center
            double Ixx_local = (1.0 / 12.0) * m * (ly * ly + lz * lz);
            double Iyy_local = (1.0 / 12.0) * m * (lx * lx + lz * lz);
            double Izz_local = (1.0 / 12.0) * m * (lx * lx + ly * ly);

            // Distance from geometry CG to system CG
            double dx = geom.x_loc - CG.data(0);
            double dy = geom.y_loc - CG.data(1);
            double dz = geom.z_loc - CG.data(2);

            // Parallel axis theorem
            j(0, 0) += Ixx_local + m * (dy * dy + dz * dz);
            j(1, 1) += Iyy_local + m * (dx * dx + dz * dz);
            j(2, 2) += Izz_local + m * (dx * dx + dy * dy);

            // Off-diagonal terms (products of inertia)
            j(0, 1) += -m * dx * dy;
            j(0, 2) += -m * dx * dz;
            j(1, 2) += -m * dy * dz;
        }

        // Symmetric
        j(1, 0) = j(0, 1);
        j(2, 0) = j(0, 2);
        j(2, 1) = j(1, 2);

        const double detj = j.determinant();
        if (std::abs(detj) < global::eps) { throw std::runtime_error("structural::StructuralProperties::compute_j: Inertia tensor is singular"); }

        return j;
    }

    std::unordered_map<std::string, size_t> StructuralProperties::build_IDs() {
        std::unordered_map<std::string, size_t> m;
        for (size_t i = 0; i < geometries.size(); ++i){
            m[geometries[i].id] = i;
        }
        return m;
    }

}