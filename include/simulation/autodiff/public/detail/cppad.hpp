#pragma once
#include "simulation/constants/public/linalg.hpp"

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

namespace autodiff
{

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
