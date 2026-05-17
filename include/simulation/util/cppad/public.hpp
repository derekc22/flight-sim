#pragma once

#include <cstddef>
#include <stdexcept>
#include <Eigen/Dense>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>

namespace util {

    CppAD::AD<double> sin(const CppAD::AD<double>& x);
    CppAD::AD<double> cos(const CppAD::AD<double>& x);
    CppAD::AD<double> tan(const CppAD::AD<double>& x);
    CppAD::AD<double> asin(const CppAD::AD<double>& x);
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

    template <typename T, std::size_t rows>
    Eigen::Matrix<T, rows, 1> eigen_vector_from_cppad_vector(const CppAD::eigen_vector<T>& x);

    template <typename T, int rows>
    CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(const Eigen::Matrix<T, rows, 1>& x);

    template <int rows>
    CppAD::eigen_vector<CppAD::AD<double>> start_autodiff_tracking(const Eigen::Matrix<double, rows, 1>& x);

    template <int output_rows, int input_rows>
    Eigen::Matrix<double, output_rows, input_rows> compute_jac(CppAD::ADFun<double>& f, const Eigen::Matrix<double, input_rows, 1>& x);

}

#include "simulation/util/cppad/public.tpp"
