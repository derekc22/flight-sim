#include "simulation/propulsion/public/detail/state.hpp"

namespace propulsion {

    PropellerOmegaStateSet_T<double> compute_propeller_omega_state_set(const actuators::PropulsorActuators& propulsors, const PropulsionState& previous_state, const actuators::PropulsorActuatorInputs_T<double>& actuator_inputs, const atmospheric::AirDensity& rho, double dt, bool steady_state) {
        return compute_propeller_omega_state_set_T<double>(propulsors, previous_state, actuator_inputs, rho, dt, steady_state);
    }

}
