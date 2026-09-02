#include "simulation/propulsion/public/detail/state.hpp"

namespace propulsion {

    PropellerOmegaStateSet_T<double> compute_propeller_omega_state_set(const actuators::PropulsorActuators& propulsors, const PropulsionState& prev_state, const actuators::PropulsorActuatorInputs_T<double>& u, const atmospheric::AirDensity& rho, double dt, bool steady_state) {
        return compute_propeller_omega_state_set_T<double>(propulsors, prev_state, u, rho, dt, steady_state);
    }

}
