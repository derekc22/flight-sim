#pragma once
#include "simulation/constants/public.hpp"
#include "simulation/propulsion/public/detail/state.hpp"
#include "simulation/util/cppad/public.hpp"
#include "simulation/util/public.hpp"

namespace propulsion {

    template <typename T>
    T compute_propeller_omega_T(const actuators::PropulsorActuator& propulsor, const T& thrust, const atmospheric::AirDensity& rho) {
        if (!propulsor.propellers) { 
            return T(0.0); 
        }
        const actuators::PropellerAssembly& propellers = propulsor.propellers.value();
        const double denom = propellers.thrust_coeff * rho.data * propellers.diameter * propellers.diameter * propellers.diameter * propellers.diameter;
        const T n = util::sqrt((thrust + T(constants::eps)) / T(denom));
        return T(2.0 * constants::pi) * n;
    }

    template <typename T>
    PropellerOmegaState_T<T> compute_propeller_omega_state_T(const actuators::PropulsorActuator& propulsor, T thrust, const atmospheric::AirDensity& rho, const std::optional<double>& previous_omega, T dt, bool steady_state) {
        if (!propulsor.propellers.has_value()) {
            return {}; 
        }

        T omega = compute_propeller_omega_T<T>(propulsor, thrust, rho);
        T omega_dot = !steady_state && previous_omega.has_value() ? (omega - T(previous_omega.value())) / dt : T(0.0);
        return {
            .omega = omega,
            .omega_dot = omega_dot
        };
    }

    template <typename T>
    PropellerOmegaStateSet_T<T> compute_propeller_omega_state_set_T(const actuators::PropulsorActuators& propulsors, const PropulsionState& previous_state, const actuators::PropulsorActuatorInputs_T<T>& actuator_inputs, const atmospheric::AirDensity& rho, T dt, bool steady_state) {
        return {
            .front_propulsor = compute_propeller_omega_state_T<T>(propulsors.front_propulsor, actuator_inputs.front_propulsor_cmd, rho, previous_state.front_propulsor_omega, dt, steady_state),
            .left_propulsor = compute_propeller_omega_state_T<T>(propulsors.left_propulsor, actuator_inputs.left_propulsor_cmd, rho, previous_state.left_propulsor_omega, dt, steady_state),
            .right_propulsor = compute_propeller_omega_state_T<T>(propulsors.right_propulsor, actuator_inputs.right_propulsor_cmd, rho, previous_state.right_propulsor_omega, dt, steady_state)
        };
    }

}
