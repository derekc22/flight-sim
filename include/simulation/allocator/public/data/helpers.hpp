#pragma once
#include "simulation/allocator/public/data/types.hpp"

namespace allocator {

    AllocatorManagerInput build_allocator_input(
        const control::VirtualControlOutput& mu_cmd,
        const std::array<bool, constants::virtual_input_dim>& active_mask,
        const std::array<bool, constants::input_dim>& actuator_mask,
        const dynamics::RigidBodyState& Zt,
        const control::ControlOutput& u_actual_t_1,
        const std::optional<control::ControlOutput>& u_preferred,
        const operating::OperatingConditions& conditions, 
        autodiff::AutoDiffModel& model
    );

}
