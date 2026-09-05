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

	template <typename T, std::size_t rows>
	constants::MatrixX_T<T, rows, 1> eigen_vector_from_cppad_vector(const CppAD::eigen_vector<T>& x);

	template <typename T, int rows>
	CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(const constants::MatrixX_T<T, rows, 1>& x);

	template <int rows>
	constants::MatrixX_T<CppAD::AD<double>, rows, 1> double_to_cppad_double(
		const constants::MatrixX_T<double, rows, 1>& x);

	AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft, const structural::StructuralState& struc_t);

} // namespace autodiff

#include "simulation/autodiff/public/data/helpers.tpp"
