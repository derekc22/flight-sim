#include "simulation/allocator/public/data/helpers.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"

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
    ) {
        return {
            .virtual_control = dynamics::unpack_wrench(mu_cmd),
            .active_mask = active_mask,
            .actuator_mask = actuator_mask,
            .operating_point = {
                .state = dynamics::pack_state(Zt),
                .input = u_actual_t_1
            },
            .preferred_actuator_command = u_preferred,
            .conditions = conditions,
            .model = model
        };
    }

}
