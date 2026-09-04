#pragma once
#include "simulation/constants/public/linalg.hpp"

#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <cstddef>

namespace autodiff
{

	template <typename T, std::size_t rows>
	constants::MatrixX_T<T, rows, 1> eigen_vector_from_cppad_vector(const CppAD::eigen_vector<T>& x);

	template <typename T, int rows>
	CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(const constants::MatrixX_T<T, rows, 1>& x);

	template <int rows>
	constants::MatrixX_T<CppAD::AD<double>, rows, 1> double_to_cppad_double(
		const constants::MatrixX_T<double, rows, 1>& x);

	template <int rows>
	CppAD::eigen_vector<CppAD::AD<double>> start_autodiff_tracking(const constants::MatrixX_T<double, rows, 1>& x);

	template <int output_rows, int input_rows>
	constants::MatrixX_T<double, output_rows, 1> evaluate_tracked_vector(CppAD::ADFun<double>& f,
		const constants::MatrixX_T<double, input_rows, 1>& x);

	template <int output_rows, int input_rows>
	constants::MatrixX_T<double, output_rows, input_rows> compute_jac(CppAD::ADFun<double>& f,
		const constants::MatrixX_T<double, input_rows, 1>& x);

} // namespace autodiff

#include "simulation/autodiff/public/detail/cppad.tpp"
