#pragma once

#include <Eigen/Dense>

namespace util {

    Eigen::Matrix3d hat(const Eigen::Vector3d& v);
    Eigen::Vector3d vee(const Eigen::Matrix3d& S);
    Eigen::Vector3d norm(const Eigen::Vector3d& v);

}
