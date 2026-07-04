#include "simulation/propulsion/private.hpp"

namespace propulsion {

    double step_propeller_omega_dot(actuators::PropulsorActuator& propulsor, double thrust, const atmospheric::StaticAtmosphericState& atm, double dt) {
        if (!propulsor.propellers) { return 0.0; }

        const atmospheric::AirDensity& rho = atm.rho;
        double omega = compute_propeller_omega_T<double>(propulsor, thrust, rho);
        double prev_omega = propulsor.propellers->prev_omega.value_or(omega);

        propulsor.propellers->prev_omega = omega;

        return (omega - prev_omega) / dt;
    }

}
