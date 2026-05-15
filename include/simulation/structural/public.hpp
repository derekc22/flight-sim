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
        double x_loc;
        double y_loc;
        double z_loc;
    };

    struct StructuralProperties {
        std::vector<Geometry> geometries;
        std::unordered_map<std::string, size_t> geometryIDs;

        dynamics::Mass Mass;
        dynamics::CenterOfGravity CG;
        dynamics::InertiaTensor J;

        StructuralProperties(const std::vector<Geometry>& g);

        Geometry& get_geometry(const std::string& id);
        double compute_Mass();
        Eigen::Vector3d compute_CG();
        Eigen::Matrix3d compute_J();
        Eigen::Matrix3d compute_local_J(const Geometry& geom);
        double compute_spin_inertia(const Geometry& geom, const Eigen::Vector3d& axis);
        std::unordered_map<std::string, size_t> build_IDs();

    };


}