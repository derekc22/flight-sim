#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <cmath>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"
#include "simulation/transforms/transforms.hpp"

namespace frames {

    struct Frame {
        const Frame& Parent;
        dynamics::HomogenousFrameTransformationMatrix H; // {SELF} -> {PARENT}
    };

    struct ECEFFrame {};

    struct NEDFrame {
        dynamics::HomogenousFrameTransformationMatrix HEN; // {ECEF} -> {NED}
    };

    struct FRDFrame {  // FRD <=> BODY

        // {ECEF} -> {BODY}
        dynamics::HomogenousFrameTransformationMatrix HEB; 

        // {NED} -> {BODY}
        dynamics::HomogenousFrameTransformationMatrix HNB; 
        dynamics::OrientationMatrixRate CNB_dot;
        dynamics::OrientationQuaternion qNB;
        dynamics::OrientationQuaternionRate qB_BN_dot;
        dynamics::BodyAxisRates wB_BN;
        dynamics::LinearVelocity vB_BN;
        dynamics::LinearAcceleration aB_BN;
        dynamics::EulerAngles eul;
        dynamics::EulerAngleRates eul_dot;

    };

    Eigen::Matrix3d CEN_from_lat_lon(double lat, double lon);

    std::array<double, 3> lat_lon_alt_from_xECEF(const Eigen::Vector3d& xECEF);


    namespace common {
        Eigen::Vector3d gECEF(Eigen::Vector3d pE_BE);
    }


}
