#pragma once
#include "simulation/propulsion/public/data/helpers.hpp"
#include "simulation/propulsion/public/detail/loads.hpp"
#include "simulation/propulsion/public/detail/state.hpp"
#include "simulation/propulsion/public/manager.hpp"

namespace propulsion {

    template <typename T>
    PropulsionManagerOutput_T<T> PropulsionManager::step_T(const PropulsionManagerInput_T<T>& input) {
        const PropellerOmegaStateSet_T<T> propeller_state = compute_propeller_omega_state_set_T<T>(input.propulsors, state, input.actuator_inputs, input.atmosphere.rho, input.dt, input.steady_state);

        return {
            .propulsive_wrench = compute_propulsive_loads_T<T>(input.propulsors, input.center_of_gravity, input.vehicle_twist, input.atmosphere, input.actuator_inputs, propeller_state),
            .next_state = make_propulsion_state_T<T>(input.propulsors, propeller_state)
        };
    }

}
