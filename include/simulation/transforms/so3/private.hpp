#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace transforms {

    Eigen::Matrix3d Rx(double phi);
    Eigen::Matrix3d Ry(double theta);
    Eigen::Matrix3d Rz(double psi);

    Eigen::Matrix3d eul_to_R_extr(double roll, double pitch, double yaw, const std::string& order);
    Eigen::Matrix3d eul_to_R_intr(double roll, double pitch, double yaw, const std::string& order);

    Eigen::Matrix3d chain_rot_post(const std::vector<Eigen::Matrix3d>& rot_list);
    Eigen::Matrix3d chain_rot_pre(const std::vector<Eigen::Matrix3d>& rot_list);

    Eigen::Matrix3d eul_to_C_extr(double a, double b, double c, const std::string& order);
    Eigen::Matrix3d eul_to_C_intr(double a, double b, double c, const std::string& order);


}
