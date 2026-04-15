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
    Eigen::Matrix<T, rows, 1> eigen_vector_from_cppad_vector(const CppAD::eigen_vector<T>& x) {
        if (x.size() != rows) throw std::invalid_argument("util::eigen_vector_from_cppad_vector: vector has incorrect size");

        Eigen::Matrix<T, rows, 1> out;
        for (std::size_t i = 0; i < rows; ++i) {
            out(static_cast<Eigen::Index>(i)) = x[i];
        }
        return out;
    }

    template <typename T, int rows>
    CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(const Eigen::Matrix<T, rows, 1>& x) {
        CppAD::eigen_vector<T> out(static_cast<std::size_t>(rows));
        for (int i = 0; i < rows; ++i) {
            out[i] = x(static_cast<Eigen::Index>(i));
        }
        return out;
    }

    template <int rows>
    CppAD::eigen_vector<CppAD::AD<double>> start_autodiff_tracking(const Eigen::Matrix<double, rows, 1>& x) {
        CppAD::eigen_vector<CppAD::AD<double>> x_t = cppad_vector_from_eigen_vector(Eigen::Matrix<CppAD::AD<double>, rows, 1>(x.template cast<CppAD::AD<double>>()));
        CppAD::Independent(x_t);
        return x_t;
    }

    template <int output_rows, int input_rows>
    Eigen::Matrix<double, output_rows, input_rows> compute_jac(CppAD::ADFun<double>& f, const Eigen::Matrix<double, input_rows, 1>& x) {
        const CppAD::eigen_vector<double> x_eval = cppad_vector_from_eigen_vector(x);
        const CppAD::eigen_vector<double> jac_flat = f.Jacobian(x_eval);
        const Eigen::Map<const Eigen::Matrix<double, output_rows, input_rows, Eigen::RowMajor>> jac_map(jac_flat.data());
        return Eigen::Matrix<double, output_rows, input_rows>(jac_map);
    }

}
