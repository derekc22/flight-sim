#pragma once
#include <vector>
#include <Eigen/Dense>

namespace transforms {

    Eigen::Matrix4d make_HR_translate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);
    Eigen::Matrix4d make_HR_rotate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);

    Eigen::Matrix4d make_HC_translate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d);
    Eigen::Matrix4d make_HC_rotate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d);

    Eigen::Matrix4d chain_hom_post(const std::vector<Eigen::Matrix4d>& H_list);
    Eigen::Matrix4d chain_hom_pre(const std::vector<Eigen::Matrix4d>& H_list);

}
