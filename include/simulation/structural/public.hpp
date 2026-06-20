#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <vector>
#include "simulation/dynamics/public.hpp"

namespace structural {

    struct Geometry {
        std::string id;

        double mass;
        double x_size;
        double y_size;
        double z_size;
        Eigen::Vector3d p_ref;
    };

    struct StructuralProperties {
        std::vector<Geometry> geometries;
        std::unordered_map<std::string, size_t> geometry_id_map;

        dynamics::Mass mass;
        dynamics::CenterOfGravity p_cg;
        dynamics::InertiaTensor JB;

        StructuralProperties(const std::vector<Geometry>& g);

        Geometry& get_geometry(const std::string& id);
        double compute_mass();
        Eigen::Vector3d compute_CG();
        Eigen::Matrix3d compute_JB();
        Eigen::Matrix3d compute_local_JB(const Geometry& geom);
        double compute_spin_inertia(const Geometry& geom, const Eigen::Vector3d& axis);
        std::unordered_map<std::string, size_t> build_geometry_id_map();

    };


}
