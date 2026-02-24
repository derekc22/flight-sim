#pragma once
#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <string>
#include <unordered_map>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"


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

        StructuralProperties(std::vector<Geometry> g);

        double compute_Mass();
        Eigen::Vector3d compute_CG();
        Eigen::Matrix3d compute_J();
        std::unordered_map<std::string, size_t> build_IDs();

    };


}