#pragma once
#include "simulation/allocator/public/data/types.hpp"
#include "simulation/constants/public/dimensions.hpp"

namespace allocator
{

	/**
	 * @brief Builds an allocator input from simulation and control data.
	 *
	 * Packs @p Zt and @p u_actual_t_1 into the allocator operating point and
	 * converts the commanded wrench into force-then-moment vector form.
	 *
	 * @param[in] mu_cmd Commanded body-frame force [N] and moment [N m].
	 * @param[in] active_mask Mask identifying commanded virtual-wrench components.
	 * @param[in] actuator_mask Mask identifying actuators available to the allocator.
	 * @param[in] Zt Current rigid-body state.
	 * @param[in] u_actual_t_1 Previous-step actual actuator inputs.
	 * @param[in] u_preferred Optional preferred actuator inputs.
	 * @param[in] conditions Current atmospheric and wind conditions.
	 * @param[in] model Differentiable aircraft model and actuator configuration.
	 * @return Fully assembled allocator-manager input.
	 */
	AllocatorManagerInput build_allocator_input(const control::VirtualControlOutput& mu_cmd,
		const std::array<bool, constants::virtual_input_dim>& active_mask,
		const std::array<bool, constants::input_dim>& actuator_mask,
		const dynamics::RigidBodyState& Zt,
		const control::ControlOutput& u_actual_t_1,
		const std::optional<control::ControlOutput>& u_preferred,
		const operating::OperatingConditions& conditions,
		const autodiff::AutoDiffModel& model);

} // namespace allocator
