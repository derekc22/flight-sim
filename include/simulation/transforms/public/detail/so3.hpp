#pragma once
#include "simulation/transforms/public/data/types.hpp"
#include <Eigen/Dense>

namespace transforms {

    Eigen::Matrix3d C_from_R(const Eigen::Matrix3d& R);
    Eigen::Matrix3d R_from_C(const Eigen::Matrix3d& C);

    Eigen::Matrix3d eul_to_R(double a, double b, double c, EulerOrder order);
    Eigen::Matrix3d eul_to_C(double a, double b, double c, EulerOrder order, RotationType type);

    Eigen::Vector3d R_to_eul(const Eigen::Matrix3d& R, EulerOrder order);
    Eigen::Vector3d C_to_eul(const Eigen::Matrix3d& C, EulerOrder order, RotationType type);

    Eigen::Quaterniond rot_to_quat(const Eigen::Matrix3d& rot);
}
