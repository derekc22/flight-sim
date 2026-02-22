#pragma once
#include <Eigen/Dense>
#include "simulation/frames/frames.hpp"



namespace vehicles {

    struct Properties {
        dynamics::Mass mass;
        dynamics::InertiaTensor J;
    };

    struct Aircraft {
        frames::NEDFrameECEF NEDFrame;
        frames::FRDFrameNED BODYFrameNED;
        frames::FRDFrameECEF BODYFrameECEF;
        // frames::GenericFrame WindFrame;
        // frames::GenericFrame CustomFrame1;
        // frames::GenericFrame CustomFrame2;
        Properties properties;


        // const double detJ = J.data.determinant();
        // if (std::abs(detJ) < global::eps) { throw std::runtime_error("dynamics::ddtB_wB_BI: inertia tensor is singular"); }

        // if (mass < global::eps) { throw std::runtime_error("dynamics::step_rigid_body: mass must be positive"); }


    };




}