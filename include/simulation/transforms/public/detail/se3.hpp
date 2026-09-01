#pragma once
#include <Eigen/Dense>
#include "simulation/transforms/public/data/types.hpp"

namespace transforms {

    Eigen::Matrix3d C_from_H(const Eigen::Matrix4d& H);
    Eigen::Matrix3d R_from_H(const Eigen::Matrix4d& H);
    Eigen::Vector3d d_from_H(const Eigen::Matrix4d& H);
    Eigen::Vector3d p_from_H(const Eigen::Matrix4d& H);

    Eigen::Matrix4d make_HR(const Eigen::Matrix3d& R, const Eigen::Vector3d& d, TransformationOrder order);
    Eigen::Matrix4d make_HC(const Eigen::Matrix3d& C, const Eigen::Vector3d& d, TransformationOrder order);
    Eigen::Matrix4d make_Hinv(const Eigen::Matrix4d& H);

    Eigen::Vector3d apply_H(const Eigen::Matrix4d& H, const Eigen::Vector3d& v);

}
