#pragma once
#include <Eigen/Dense>
#include "simulation/frames/public.hpp"

namespace frames {

    /** @brief Obtains the rotation matrix from the root (ECEFFrame) to the frame F */
    Eigen::Matrix3d CRF(const Frame& F);
    Eigen::Matrix4d HRF(const Frame& F);
    Eigen::Vector3d pRF(const Frame& F);

}
