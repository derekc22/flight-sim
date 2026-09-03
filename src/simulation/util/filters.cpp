#include <cmath>
#include "simulation/util/public/filters.hpp"

namespace util {

    double first_order_lag(double val, double prev_val, double tau, double dt) {
        if (tau <= 0.0) { return val; }

        double alpha = std::exp(-dt / tau);
        return (1-alpha) * val + alpha * prev_val;
    }

    Eigen::Vector3d first_order_lag(const Eigen::Vector3d& val, const Eigen::Vector3d& prev_val, double tau, double dt) {
        if (tau <= 0.0) { return val; }

        double alpha = std::exp(-dt / tau);
        return (1-alpha) * val + alpha * prev_val;
    }

    Eigen::Quaterniond first_order_lag(const Eigen::Quaterniond& val, const Eigen::Quaterniond& prev_val, double tau, double dt) {
        if (tau <= 0.0) { return val; }

        double alpha = std::exp(-dt / tau);
        Eigen::Quaterniond val_adjusted = val;
        if (prev_val.coeffs().dot(val.coeffs()) < 0.0) val_adjusted.coeffs() *= -1.0;

        return prev_val.slerp(1-alpha, val_adjusted);
    }

}
