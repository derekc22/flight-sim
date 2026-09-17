#pragma once
#include "simulation/allocator/public/data/types.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/qp/public/solver.hpp"

namespace allocator
{

	struct AllocatorManager {
		constants::MatrixX_T<double, constants::virtual_input_dim, constants::virtual_input_dim> Q;
		constants::MatrixX_T<double, constants::input_dim, constants::input_dim> R;
		qp::Solver solver{constants::input_dim};

		/**
		 * @brief Allocates a virtual-wrench command to bounded actuator inputs.
		 *
		 * Linearizes the net wrench with respect to actuator inputs, masks inactive
		 * wrench components, and solves a weighted quadratic program. An optional
		 * preferred input supplies the target for inactive actuators and adds a trim
		 * tracking penalty. The returned allocation residual is the effectiveness
		 * matrix multiplied by the constrained-minus-unconstrained actuator input
		 * when an active actuator limit binds; otherwise, it is zero.
		 *
		 * @param[in] input Command, masks, operating point, conditions, and aircraft model.
		 * @return Allocated actuator command and virtual-wrench allocation residual.
		 */
		AllocatorManagerOutput step(const AllocatorManagerInput& input);
	};

} // namespace allocator
