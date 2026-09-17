#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/constants/public/linalg.hpp"

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <cstddef>

namespace vehicles
{
	struct Aircraft;
} // namespace vehicles

namespace autodiff
{

	/**
	 * @brief Converts a CppAD Eigen vector to a fixed-size Eigen column vector.
	 *
	 * @tparam T Scalar type stored by the vectors.
	 * @tparam rows Required number of vector rows.
	 * @param[in] x CppAD Eigen vector to convert.
	 * @return Fixed-size Eigen column vector with the same coefficients.
	 */
	template <typename T, std::size_t rows>
	constants::MatrixX_T<T, rows, 1> eigen_vector_from_cppad_vector(const CppAD::eigen_vector<T>& x);

	/**
	 * @brief Converts a fixed-size Eigen column vector to a CppAD Eigen vector.
	 *
	 * @tparam T Scalar type stored by the vectors.
	 * @tparam rows Number of vector rows.
	 * @param[in] x Fixed-size Eigen column vector to convert.
	 * @return CppAD Eigen vector with the same coefficients.
	 */
	template <typename T, int rows>
	CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(const constants::MatrixX_T<T, rows, 1>& x);

	/**
	 * @brief Converts a double vector to a CppAD scalar vector.
	 *
	 * @tparam rows Number of vector rows.
	 * @param[in] x Double-precision vector to convert.
	 * @return Vector containing equivalent `CppAD::AD<double>` values.
	 */
	template <int rows>
	constants::MatrixX_T<CppAD::AD<double>, rows, 1> double_to_cppad_double(
		const constants::MatrixX_T<double, rows, 1>& x);

	/**
	 * @brief Builds the model used for automatic-differentiation evaluations.
	 *
	 * Copies structural state, actuator limits, and fixed controls while retaining references to the aircraft's
	 * aerodynamic and propulsion managers. The aircraft must outlive the returned model.
	 *
	 * @param[in] aircraft Aircraft providing managers, actuator limits, and fixed controls.
	 * @param[in] struc_t Structural state copied into the model.
	 * @return Automatic-differentiation model referencing the aircraft managers.
	 */
	AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft, const structural::StructuralState& struc_t);

} // namespace autodiff

#include "simulation/autodiff/public/data/helpers.tpp"
