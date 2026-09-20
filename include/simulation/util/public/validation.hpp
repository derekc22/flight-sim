#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <string>

namespace util
{

	/**
	 * @brief Validates that an Eigen expression has the expected dimensions.
	 *
	 * @tparam T Eigen dense-expression type.
	 * @param[in] M Matrix or vector to validate.
	 * @param[in] rows Expected row count.
	 * @param[in] cols Expected column count.
	 * @param[in] context Calling context included in diagnostics.
	 * @param[in] name Value name included in diagnostics.
	 */
	template <typename T>
	void validate_shape(const Eigen::DenseBase<T>& M,
		Eigen::Index rows,
		Eigen::Index cols,
		const std::string& context,
		const std::string& name);

	/**
	 * @brief Validates that an Eigen expression is square.
	 *
	 * @tparam T Eigen dense-expression type.
	 * @param[in] M Matrix to validate.
	 * @param[in] context Calling context included in diagnostics.
	 * @param[in] name Matrix name included in diagnostics.
	 */
	template <typename T>
	void validate_square(const Eigen::DenseBase<T>& M, const std::string& context, const std::string& name);

	/**
	 * @brief Validates that an Eigen vector or matrix has unit norm.
	 *
	 * @tparam T Eigen matrix-expression type.
	 * @param[in] v Value to validate.
	 * @param[in] context Calling context included in diagnostics.
	 * @param[in] name Value name included in diagnostics.
	 */
	template <typename T>
	void validate_unit(const Eigen::MatrixBase<T>& v, const std::string& context, const std::string& name);

	/**
	 * @brief Validates that a matrix is a proper orthonormal matrix.
	 *
	 * @c constants::eps of positive one.
	 *
	 * @tparam T Eigen matrix-expression type.
	 * @param[in] M Matrix to validate.
	 * @param[in] context Calling context included in diagnostics.
	 * @param[in] name Matrix name included in diagnostics.
	 */
	template <typename T>
	void validate_orthonormal(const Eigen::MatrixBase<T>& M, const std::string& context, const std::string& name);

	/**
	 * @brief Validates that two scalar values are exactly equal.
	 *
	 * @tparam T Comparable value type supported by @c std::to_string.
	 * @param[in] a First value.
	 * @param[in] b Second value.
	 * @param[in] context Calling context included in diagnostics.
	 * @param[in] name_a First value name included in diagnostics.
	 * @param[in] name_b Second value name included in diagnostics.
	 */
	template <typename T>
	void validate_value(T a, T b, const std::string& context, const std::string& name_a, const std::string& name_b);

} // namespace util

#include "simulation/util/public/validation.tpp"
