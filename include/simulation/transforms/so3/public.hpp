#pragma once
#include <string>
#include <Eigen/Dense>

namespace transforms {

    Eigen::Matrix3d C_from_R(const Eigen::Matrix3d& R);
    Eigen::Matrix3d R_from_C(const Eigen::Matrix3d& C);

    Eigen::Matrix3d eul_to_R(double a, double b, double c, const std::string& order);
    Eigen::Matrix3d eul_to_C(double a, double b, double c, const std::string& order, const std::string& type);

    Eigen::Vector3d R_to_eul(const Eigen::Matrix3d& R, const std::string& order);
    Eigen::Vector3d C_to_eul(const Eigen::Matrix3d& C, const std::string& order, const std::string& type);

    Eigen::Quaterniond rot_to_quat(const Eigen::Matrix3d& rot);
}
