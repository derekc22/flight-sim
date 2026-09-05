#pragma once
#include "simulation/autodiff/public/data/helpers.hpp"
#include "simulation/autodiff/public/detail/cppad.hpp"
#include "simulation/constants/public/linalg.hpp"

#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

namespace autodiff
{

	template <int rows>
	CppAD::eigen_vector<CppAD::AD<double>> start_autodiff_tracking(
		const constants::MatrixX_T<double, rows, 1>& x)
	{
		CppAD::eigen_vector<CppAD::AD<double>> x_tracked = cppad_vector_from_eigen_vector(double_to_cppad_double(x));
		CppAD::Independent(x_tracked);
		return x_tracked;
	}

	template <int output_rows, int input_rows>
	constants::MatrixX_T<double, output_rows, 1> evaluate_tracked_vector(
		CppAD::ADFun<double>& f,
		const constants::MatrixX_T<double, input_rows, 1>& x)
	{
		const CppAD::eigen_vector<double> x_eval = cppad_vector_from_eigen_vector(x);
		const CppAD::eigen_vector<double> y_eval = f.Forward(0, x_eval);
		return eigen_vector_from_cppad_vector<double, output_rows>(y_eval);
	}

	template <int output_rows, int input_rows>
	constants::MatrixX_T<double, output_rows, input_rows> compute_jac(
		CppAD::ADFun<double>& f,
		const constants::MatrixX_T<double, input_rows, 1>& x)
	{
		const CppAD::eigen_vector<double> x_eval = cppad_vector_from_eigen_vector(x);
		const CppAD::eigen_vector<double> jac_flat = f.Jacobian(x_eval);
		const Eigen::Map<const Eigen::Matrix<double, output_rows, input_rows, Eigen::RowMajor>> jac_map(
			jac_flat.data());
		return constants::MatrixX_T<double, output_rows, input_rows>(jac_map);
	}

} // namespace autodiff
