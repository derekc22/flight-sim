#include <cmath>
#include "simulation/constants/constants.hpp"
#include "simulation/util/linalg.hpp"

namespace util {

    Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
        Eigen::Matrix3d S;
        S << 0, -v.z(), v.y(),
             v.z(), 0, -v.x(),
             -v.y(), v.x(), 0;
        return S;
    }

    Eigen::Vector3d vee(const Eigen::Matrix3d& S) {
        return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
    }

    Eigen::Vector3d norm(const Eigen::Vector3d& v) {
        const double n = v.norm();
        if (n < constants::eps) return constants::Zero3;
        return v / n;
    }

    double vector_norm(const Eigen::Vector3d& v) {
        return std::sqrt(v.dot(v));
    }

}
