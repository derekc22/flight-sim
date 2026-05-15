#include <algorithm>
#include <cmath>
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"

namespace util {

    double clamp(double x, double min_val, double max_val) {
        if (max_val <= min_val) return min_val;
        if (x > max_val) return max_val;
        if (x < min_val) return min_val;
        return x;
    }

    double clamp_symmetric(double x, double max_abs) {
        if (max_abs <= 0.0) return 0.0;
        return std::clamp(x, -max_abs, max_abs);
    }

    double clamp_positive(double x, double max_val) {
        if (max_val <= 0.0) return 0.0;
        return std::clamp(x, 0.0, max_val);
    }

    double clamp_to_1(double x) {
        // clamps to [-1, 1]
        return std::clamp(x, -1.0, 1.0);
    }

    double clamp_inside_1(double x) {
        // clamps to [-1+eps, 1-eps]
        return std::clamp(x, -1.0 + constants::eps, 1.0 - constants::eps);
    }

    double wrap_to_pi(double x) {
        // maps to (-pi, pi]
        return std::remainder(x, 2.0 * constants::pi);
    }

    double sqrt(double x) {
        return std::sqrt(x);
    }

    double abs(double x) {
        return std::abs(x);
    }

    double smooth_abs(double x) {
        return sqrt(x * x + constants::eps * constants::eps) - constants::eps;
    }

    double first_order_lag(double val, double prev_val, double tau) {
        if (tau <= 0.0) { return val; }

        double alpha = std::exp(-constants::dt / tau);
        return (1-alpha) * val + alpha * prev_val;
    }

    Eigen::Vector3d first_order_lag(const Eigen::Vector3d& val, const Eigen::Vector3d& prev_val, double tau) {
        if (tau <= 0.0) { return val; }

        double alpha = std::exp(-constants::dt / tau);
        return (1-alpha) * val + alpha * prev_val;
    }

    Eigen::Quaterniond first_order_lag(const Eigen::Quaterniond& val, const Eigen::Quaterniond& prev_val, double tau) {
        if (tau <= 0.0) { return val; }

        double alpha = std::exp(-constants::dt / tau);
        Eigen::Quaterniond val_adjusted = val;
        if (prev_val.coeffs().dot(val.coeffs()) < 0.0) val_adjusted.coeffs() *= -1.0;

        return prev_val.slerp(1-alpha, val_adjusted);
    }

}
