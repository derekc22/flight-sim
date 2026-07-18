#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "simulation/transforms/shared/public.hpp"

namespace transforms {

    Eigen::Matrix3d Rx(double phi);
    Eigen::Matrix3d Ry(double theta);
    Eigen::Matrix3d Rz(double psi);

    Eigen::Matrix3d eul_to_R_extr(double roll, double pitch, double yaw, EulerOrder order);
    Eigen::Matrix3d eul_to_R_intr(double roll, double pitch, double yaw, EulerOrder order);
    Eigen::Matrix3d eul_to_C_extr(double a, double b, double c, EulerOrder order);
    Eigen::Matrix3d eul_to_C_intr(double a, double b, double c, EulerOrder order);

    Eigen::Vector3d R_to_eul_extr(const Eigen::Matrix3d& R, EulerOrder order);
    Eigen::Vector3d R_to_eul_intr(const Eigen::Matrix3d& R, EulerOrder order);
    Eigen::Vector3d C_to_eul_extr(const Eigen::Matrix3d& C, EulerOrder order);
    Eigen::Vector3d C_to_eul_intr(const Eigen::Matrix3d& C, EulerOrder order);

    Eigen::Matrix3d chain_rot_post(const std::vector<Eigen::Matrix3d>& rot_list);
    Eigen::Matrix3d chain_rot_pre(const std::vector<Eigen::Matrix3d>& rot_list);

}
