#pragma once
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>
#include "simulation/structural/public/data/types.hpp"

namespace structural {

    struct StructuralManager {
        std::vector<Geometry> geometries;
        std::unordered_map<std::string, size_t> geometry_id_map;

        StructuralManager(const std::vector<Geometry>& geoms);

        std::unordered_map<std::string, size_t> build_geometry_id_map();
        Geometry& get_geometry(const std::string& id);

        double compute_mass();
        Eigen::Vector3d compute_CG(const dynamics::Mass& mass);
        Eigen::Matrix3d compute_JB(const CenterOfGravity& pB_GB);
        Eigen::Matrix3d compute_local_JB(const Geometry& geom);

        double compute_spin_inertia(const Geometry& geom, const Eigen::Vector3d& axis);

        StructuralManagerOutput step(const StructuralManagerInput& input);
    };


}
