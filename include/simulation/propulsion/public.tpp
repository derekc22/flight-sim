#pragma once
#include "simulation/constants/public.hpp"
#include "simulation/propulsion/public.hpp"
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
    T compute_propeller_torque_T(const actuators::PropulsorActuator& propulsor, const T& omega, const atmospheric::AirDensity& rho) {
        if (!propulsor.propellers) { 
            return T(0.0); 
        }
        const actuators::PropellerAssembly& propellers = propulsor.propellers.value();
        const T n = omega / T(2.0 * constants::pi);
        return T(propellers.torque_coeff * rho.data * propellers.diameter * propellers.diameter * propellers.diameter * propellers.diameter * propellers.diameter) * n * n;
    }

    template <typename T>
    dynamics::Wrench_T<T> step_propulsor_forces_moments_T(const actuators::PropulsorActuator& propulsor, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const T& thrust, const T& omega_dot) {
        dynamics::Wrench_T<T> out;

        const constants::Vector3_T<T>& wB_BI = twist.w;

        const constants::Vector3_T<T> n_prop = propulsor.n_prop.cast<T>();
        const constants::Vector3_T<T> r_prop = propulsor.p_propulsor_cg.cast<T>();
        const atmospheric::AirDensity& rho = atm.rho;

        out.F = n_prop * thrust;
        out.M = r_prop.cross(out.F);

        if (propulsor.propellers.has_value()) {
            const actuators::PropellerAssembly& propellers = propulsor.propellers.value();
            const T omega = compute_propeller_omega_T<T>(propulsor, thrust, rho);
            const T q_prop = compute_propeller_torque_T<T>(propulsor, omega, rho);
            const constants::Vector3_T<T> H_prop = n_prop * (T(propellers.spin_inertia * propellers.spin_sign) * omega);

            out.M += -wB_BI.cross(H_prop);
            out.M += -n_prop * (T(propellers.spin_inertia * propellers.spin_sign) * omega_dot);
            out.M += -n_prop * (T(propellers.spin_sign) * q_prop);
        }

        return out;
    }

    template <typename T>
    dynamics::Wrench_T<T> step_propulsive_forces_moments_T(const actuators::PropulsorActuators& propulsor_actuators, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const actuators::PropulsorActuatorInputs_T<T>& u, const PropellerOmegaDotSet_T<T>& propeller_omega_dot_set) {
        dynamics::Wrench_T<T> total;

        const dynamics::Wrench_T<T> front = step_propulsor_forces_moments_T<T>(propulsor_actuators.front_propulsor, twist, atm, u.front_propulsor_cmd, propeller_omega_dot_set.front_propulsor);
        const dynamics::Wrench_T<T> left = step_propulsor_forces_moments_T<T>(propulsor_actuators.left_propulsor, twist, atm, u.left_propulsor_cmd, propeller_omega_dot_set.left_propulsor);
        const dynamics::Wrench_T<T> right = step_propulsor_forces_moments_T<T>(propulsor_actuators.right_propulsor, twist, atm, u.right_propulsor_cmd, propeller_omega_dot_set.right_propulsor);

        total.F = front.F + left.F + right.F;
        total.M = front.M + left.M + right.M;

        return total;
    }

    template <typename T>
    PropellerOmegaDotSet_T<T> compute_propeller_omega_dot_set_T(const actuators::PropulsorActuators& propulsor_actuators, const actuators::PropulsorActuatorInputs_T<T>& u, const atmospheric::StaticAtmosphericState& atm, T dt) {
        return {
            .front_propulsor = compute_propeller_omega_dot_T(propulsor_actuators.front_propulsor, u.front_propulsor_cmd, atm.rho, dt),
            .left_propulsor = compute_propeller_omega_dot_T(propulsor_actuators.left_propulsor, u.left_propulsor_cmd, atm.rho, dt),
            .right_propulsor = compute_propeller_omega_dot_T(propulsor_actuators.right_propulsor, u.right_propulsor_cmd, atm.rho, dt)
        };
    }

    template <typename T>
    T compute_propeller_omega_dot_T(const actuators::PropulsorActuator& propulsor, T thrust, const atmospheric::AirDensity& rho, T dt) {
        return compute_propeller_omega_state_T<T>(propulsor, thrust, rho, dt).omega_dot;
    }

    template <typename T>
    PropellerOmegaState_T<T> compute_propeller_omega_state_T(const actuators::PropulsorActuator& propulsor, T thrust, const atmospheric::AirDensity& rho, T dt) {
        if (!propulsor.propellers.has_value()) {
            return {}; 
        }

        T omega = compute_propeller_omega_T<T>(propulsor, thrust, rho);
        T prev_omega = propulsor.propellers->prev_omega.has_value() ? T(propulsor.propellers->prev_omega.value()) : omega;
        return {
            .omega = omega,
            .omega_dot = (omega - prev_omega) / dt
        };
    }



}
