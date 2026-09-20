#pragma once
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"

namespace allocator
{

	using EffectivenessMatrix = constants::MatrixX_T<double, constants::nv, constants::nu>;
	using ActuatorIncrementVector = constants::MatrixX_T<double, constants::nu, 1>;
	using AllocatorHessian = constants::MatrixX_T<double, constants::nu, constants::nu>;
	using AllocatorGradient = constants::MatrixX_T<double, constants::nu, 1>;

} // namespace allocator
