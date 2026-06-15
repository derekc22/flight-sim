#pragma once

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

namespace util {

    CppAD::AD<double> csc(CppAD::AD<double> x);
    CppAD::AD<double> sec(CppAD::AD<double> x);
    CppAD::AD<double> cot(CppAD::AD<double> x);
    CppAD::AD<double> sin(const CppAD::AD<double>& x);
    CppAD::AD<double> cos(const CppAD::AD<double>& x);
    CppAD::AD<double> tan(const CppAD::AD<double>& x);
    CppAD::AD<double> asin(const CppAD::AD<double>& x);
    CppAD::AD<double> acos(const CppAD::AD<double>& x);
    CppAD::AD<double> atan2(const CppAD::AD<double>& y, const CppAD::AD<double>& x);

    CppAD::AD<double> sqrt(const CppAD::AD<double>& x);
    CppAD::AD<double> abs(const CppAD::AD<double>& x);
    CppAD::AD<double> smooth_abs(const CppAD::AD<double>& x);

    CppAD::AD<double> clamp(const CppAD::AD<double>& x, double min_value, double max_value);
    CppAD::AD<double> clamp_symmetric(const CppAD::AD<double>& x, double max_abs);
    CppAD::AD<double> clamp_positive(const CppAD::AD<double>& x, double max_value);
    CppAD::AD<double> clamp_to_1(const CppAD::AD<double>& x);

    double vector_norm(const Eigen::Vector3d& v);
    CppAD::AD<double> vector_norm(const Eigen::Matrix<CppAD::AD<double>, 3, 1>& v);

    Eigen::Matrix<CppAD::AD<double>, 3, 1> norm(const Eigen::Matrix<CppAD::AD<double>, 3, 1>& v);

}
