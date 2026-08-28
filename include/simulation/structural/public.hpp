#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <vector>
#include "simulation/dynamics/public.hpp"
#include "simulation/constants/public.hpp"

namespace structural {

    struct Geometry {
        std::string id;
        double mass;
        double x_size;
        double y_size;
        double z_size;
        Eigen::Vector3d pB_geomB;
    };

    struct CenterOfGravity {
        Eigen::Vector3d data;   // pB_GB [m]
    };

    struct StructuralState {
        dynamics::Mass mass;
        CenterOfGravity pB_GB;
        dynamics::InertiaTensor JB;
    };

    template <typename T>
    struct StructuralState_T {
        T mass = T(0.0);
        constants::Vector3_T<T> pB_GB = constants::Zero3_T<T>;
        constants::Matrix3_T<T> JB = constants::Zero3x3_T<T>;
    };

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

        StructuralState compute_structural_state();
    };


}
