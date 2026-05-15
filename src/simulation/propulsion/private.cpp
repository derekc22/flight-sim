#include "simulation/propulsion/private.hpp"
#include "simulation/constants/public.hpp"

namespace propulsion {

    double step_propeller_omega_dot(actuators::PropulsorActuator& propulsor, double thrust, const atmospheric::StaticAtmosphericState& static_atm_state) {
        if (!propulsor.propellers) { return 0.0; }
        const atmospheric::AirDensity& rho = static_atm_state.rho;
        double omega = compute_propeller_omega_T<double>(propulsor, thrust, rho);
        double prev_omega = propulsor.propellers->prev_omega.value_or(omega);
        propulsor.propellers->prev_omega = omega;
        return (omega - prev_omega) / constants::dt;
    }

}
