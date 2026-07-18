#pragma once
#include "simulation/transforms/shared/public.hpp"
#include <Eigen/Dense>

namespace transforms {

    Eigen::Quaterniond eul_to_quatR(double a, double b, double c, EulerOrder order);
    Eigen::Quaterniond eul_to_quatC(double a, double b, double c, EulerOrder order, RotationType type);

    Eigen::Vector3d quatR_to_eul(const Eigen::Quaterniond& qR, EulerOrder order);
    Eigen::Vector3d quatC_to_eul(const Eigen::Quaterniond& qC, EulerOrder order, RotationType type);

    Eigen::Matrix3d quat_to_rot(const Eigen::Quaterniond& q);

    Eigen::Quaterniond normalize_and_canonicalize(Eigen::Quaterniond q);
}
