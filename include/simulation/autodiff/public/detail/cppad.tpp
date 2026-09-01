#pragma once
#include <cstddef>
#include <stdexcept>
#include <Eigen/Dense>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>
#include "simulation/autodiff/public/detail/cppad.hpp"

namespace autodiff {

    template <typename T, std::size_t rows>
    constants::MatrixX_T<T, rows, 1> eigen_vector_from_cppad_vector(const CppAD::eigen_vector<T>& x) {
        if (x.size() != rows) {
            throw std::invalid_argument("autodiff::eigen_vector_from_cppad_vector: vector has incorrect size");
        }

        constants::MatrixX_T<T, rows, 1> out;
        for (std::size_t i = 0; i < rows; ++i) {
            out(static_cast<Eigen::Index>(i)) = x[i];
        }
        return out;
    }

    template <typename T, int rows>
    CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(const constants::MatrixX_T<T, rows, 1>& x) {
        CppAD::eigen_vector<T> out(static_cast<std::size_t>(rows));
        for (int i = 0; i < rows; ++i) {
            out[i] = x(static_cast<Eigen::Index>(i));
        }
        return out;
    }

    template <int rows>
    constants::MatrixX_T<CppAD::AD<double>, rows, 1> double_to_cppad_double(const constants::MatrixX_T<double, rows, 1>& x) {
        return x.template cast<CppAD::AD<double>>();
    }

    template <int rows>
    CppAD::eigen_vector<CppAD::AD<double>> start_autodiff_tracking(const constants::MatrixX_T<double, rows, 1>& x) {
        CppAD::eigen_vector<CppAD::AD<double>> x_tracked = cppad_vector_from_eigen_vector(double_to_cppad_double(x));
        CppAD::Independent(x_tracked);
        return x_tracked;
    }

    template <int output_rows, int input_rows>
    constants::MatrixX_T<double, output_rows, 1> evaluate_tracked_vector(CppAD::ADFun<double>& f, const constants::MatrixX_T<double, input_rows, 1>& x) {
        const CppAD::eigen_vector<double> x_eval = cppad_vector_from_eigen_vector(x);
        const CppAD::eigen_vector<double> y_eval = f.Forward(0, x_eval);
        return eigen_vector_from_cppad_vector<double, output_rows>(y_eval);
    }

    template <int output_rows, int input_rows>
    constants::MatrixX_T<double, output_rows, input_rows> compute_jac(CppAD::ADFun<double>& f, const constants::MatrixX_T<double, input_rows, 1>& x) {
        const CppAD::eigen_vector<double> x_eval = cppad_vector_from_eigen_vector(x);
        const CppAD::eigen_vector<double> jac_flat = f.Jacobian(x_eval);
        const Eigen::Map<const Eigen::Matrix<double, output_rows, input_rows, Eigen::RowMajor>> jac_map(jac_flat.data());
        return constants::MatrixX_T<double, output_rows, input_rows>(jac_map);
    }

}
