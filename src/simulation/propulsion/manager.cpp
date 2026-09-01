#include "simulation/propulsion/public/data/helpers.hpp"
#include "simulation/propulsion/public/detail/loads.hpp"
#include "simulation/propulsion/public/detail/state.hpp"
#include "simulation/propulsion/public/manager.hpp"

namespace propulsion {

    PropulsionManagerOutput PropulsionManager::step(const PropulsionManagerInput& input) {
        const PropellerOmegaStateSet_T<double> propeller_state = compute_propeller_omega_state_set(input.propulsors, state, input.actuator_inputs, input.atmosphere.rho, input.dt, input.steady_state);

        return {
            .propulsive_wrench = compute_propulsive_loads(input.propulsors, input.center_of_gravity, input.vehicle_twist, input.atmosphere, input.actuator_inputs, propeller_state),
            .next_state = make_propulsion_state(input.propulsors, propeller_state)
        };
    }

    void PropulsionManager::commit(const PropulsionState& next_state) {
        // only the runtime/non-autodiff path should enter this branch because
        // 1) a type error will occur if this line attempts to assign a CppAD::AD<double> to a double field
        // 2) autodiff/trim/linearization should have no side effects; they should only evaluate simulation state, not mutate it
        state = next_state;
    }

}
