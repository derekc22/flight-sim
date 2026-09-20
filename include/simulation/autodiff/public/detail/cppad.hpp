#pragma once
#include "simulation/constants/public/linalg.hpp"

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

namespace autodiff
{

	/**
	 * @brief Starts CppAD recording for a fixed-size input vector.
	 *
	 * @tparam rows Number of independent variables.
	 * @param[in] x Initial independent-variable values.
	 * @return Tracked CppAD independent-variable vector.
	 */
	template <int rows>
	CppAD::eigen_vector<CppAD::AD<double>> start_autodiff_tracking(const constants::MatrixX_T<double, rows, 1>& x);

	/**
	 * @brief Evaluates a recorded vector function at zero order.
	 *
	 * @tparam output_rows Number of dependent variables.
	 * @tparam input_rows Number of independent variables.
	 * @param[in,out] f Recorded CppAD function evaluated by a forward sweep.
	 * @param[in] x Independent-variable values at which to evaluate the function.
	 * @return Evaluated dependent-variable vector.
	 */
	template <int output_rows, int input_rows>
	constants::MatrixX_T<double, output_rows, 1> evaluate_tracked_vector(CppAD::ADFun<double>& f,
		const constants::MatrixX_T<double, input_rows, 1>& x);

	/**
	 * @brief Computes the dense Jacobian of a recorded vector function.
	 *
	 * @tparam output_rows Number of dependent variables.
	 * @tparam input_rows Number of independent variables.
	 * @param[in,out] f Recorded CppAD function whose Jacobian is evaluated.
	 * @param[in] x Independent-variable values at which to evaluate the Jacobian.
	 * @return Dense output-by-input Jacobian matrix.
	 */
	template <int output_rows, int input_rows>
	constants::MatrixX_T<double, output_rows, input_rows> compute_jac(CppAD::ADFun<double>& f,
		const constants::MatrixX_T<double, input_rows, 1>& x);

} // namespace autodiff

#include "simulation/autodiff/public/detail/cppad.tpp"
