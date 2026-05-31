#pragma once
#include <cstddef>
#include <stdexcept>
#include <Eigen/Dense>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>
#include "simulation/util/cppad/public.hpp"

namespace util {

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
        CppAD::eigen_vector<CppAD::AD<double>> x_tracked = cppad_vector_from_eigen_vector(Eigen::Matrix<CppAD::AD<double>, rows, 1>(x.template cast<CppAD::AD<double>>()));
        CppAD::Independent(x_tracked);
        return x_tracked;
    }

    template <int output_rows, int input_rows>
    Eigen::Matrix<double, output_rows, input_rows> compute_jac(CppAD::ADFun<double>& f, const Eigen::Matrix<double, input_rows, 1>& x) {
        const CppAD::eigen_vector<double> x_eval = cppad_vector_from_eigen_vector(x);
        const CppAD::eigen_vector<double> jac_flat = f.Jacobian(x_eval);
        const Eigen::Map<const Eigen::Matrix<double, output_rows, input_rows, Eigen::RowMajor>> jac_map(jac_flat.data());
        return Eigen::Matrix<double, output_rows, input_rows>(jac_map);
    }

}
