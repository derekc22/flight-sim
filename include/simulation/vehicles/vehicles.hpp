#pragma once
#include <Eigen/Dense>
#include "simulation/frames/frames.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"



namespace vehicles {

    struct Aircraft {
        frames::NEDFrameECEF NEDFrame;
        frames::FRDFrameNED BODYFrameNED;
        frames::FRDFrameECEF BODYFrameECEF;
        // frames::GenericFrame WindFrame;
        // frames::GenericFrame CustomFrame1;
        // frames::GenericFrame CustomFrame2;
        structural::StructuralProperties structural;
        aerodynamics::AerodynamicProperties aerodynamic;


        // const double detJ = J.data.determinant();
        // if (std::abs(detJ) < global::eps) { throw std::runtime_error("dynamics::ddtB_wB_BI: inertia tensor is singular"); }

        // if (mass < global::eps) { throw std::runtime_error("dynamics::step_rigid_body: mass must be positive"); }


    };




}