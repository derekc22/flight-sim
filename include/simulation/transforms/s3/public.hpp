#pragma once
#include <string>
#include <Eigen/Dense>

namespace transforms {

    Eigen::Matrix3d quat_to_rot(const Eigen::Quaterniond& q);

    Eigen::Quaterniond normalize_and_canonicalize(Eigen::Quaterniond q);

    Eigen::Quaterniond eul_to_quatR(double a, double b, double c, const std::string& order, const std::string& type);
    Eigen::Quaterniond eul_to_quatC(double a, double b, double c, const std::string& order, const std::string& type);
    Eigen::Vector3d quatC_to_eul(const Eigen::Quaterniond& qC, const std::string& order, const std::string& type);

}
