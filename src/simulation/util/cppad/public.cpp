#include "simulation/constants/public.hpp"
#include "simulation/util/cppad/public.hpp"

namespace util {

    CppAD::AD<double> sin(const CppAD::AD<double>& x) {
        return CppAD::sin(x);
    }

    CppAD::AD<double> cos(const CppAD::AD<double>& x) {
        return CppAD::cos(x);
    }

    CppAD::AD<double> tan(const CppAD::AD<double>& x) {
        return CppAD::tan(x);
    }

    CppAD::AD<double> asin(const CppAD::AD<double>& x) {
        return CppAD::asin(x);
    }

    CppAD::AD<double> atan2(const CppAD::AD<double>& y, const CppAD::AD<double>& x) {
        return CppAD::atan2(y, x);
    }

    CppAD::AD<double> sqrt(const CppAD::AD<double>& x) {
        return CppAD::sqrt(x);
    }

    CppAD::AD<double> abs(const CppAD::AD<double>& x) {
        return CppAD::abs(x);
    }

    CppAD::AD<double> smooth_abs(const CppAD::AD<double>& x) {
        const CppAD::AD<double> eps_t(constants::eps);
        return sqrt(x * x + eps_t * eps_t) - eps_t;
    }

    CppAD::AD<double> clamp(const CppAD::AD<double>& x, double min_value, double max_value) {
        if (max_value <= min_value) return CppAD::AD<double>(min_value);
        const CppAD::AD<double> min_t(min_value);
        const CppAD::AD<double> max_t(max_value);
        if (x > max_t) return max_t;
        if (x < min_t) return min_t;
        return x;
    }

    CppAD::AD<double> clamp_symmetric(const CppAD::AD<double>& x, double max_abs) {
        if (max_abs <= 0.0) return CppAD::AD<double>(0.0);
        const CppAD::AD<double> max_t(max_abs);
        if (x > max_t) return max_t;
        if (x < -max_t) return -max_t;
        return x;
    }

    CppAD::AD<double> clamp_positive(const CppAD::AD<double>& x, double max_value) {
        if (max_value <= 0.0) return CppAD::AD<double>(0.0);
        const CppAD::AD<double> max_t(max_value);
        if (x < CppAD::AD<double>(0.0)) return CppAD::AD<double>(0.0);
        if (x > max_t) return max_t;
        return x;
    }

    CppAD::AD<double> clamp_to_1(const CppAD::AD<double>& x) {
        if (x > CppAD::AD<double>(1.0)) return CppAD::AD<double>(1.0);
        if (x < CppAD::AD<double>(-1.0)) return CppAD::AD<double>(-1.0);
        return x;
    }

    CppAD::AD<double> vector_norm(const Eigen::Matrix<CppAD::AD<double>, 3, 1>& v) {
        return sqrt(v.dot(v));
    }

    Eigen::Matrix<CppAD::AD<double>, 3, 1> norm(const Eigen::Matrix<CppAD::AD<double>, 3, 1>& v) {
        const CppAD::AD<double> n = vector_norm(v);
        if (n < CppAD::AD<double>(constants::eps)) return Eigen::Matrix<CppAD::AD<double>, 3, 1>::Zero();
        return v / n;
    }

}
