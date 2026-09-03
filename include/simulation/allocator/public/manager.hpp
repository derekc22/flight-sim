#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/allocator/public/data/types.hpp"
#include "simulation/qp/public/solver.hpp"

namespace allocator {

	struct AllocatorManager {
		constants::MatrixX_T<double, constants::virtual_input_dim, constants::virtual_input_dim> Q;
		constants::MatrixX_T<double, constants::input_dim, constants::input_dim> R;
		qp::Solver solver{constants::input_dim};

		AllocatorManagerOutput step(const AllocatorManagerInput& input);
	};

} // namespace allocator
