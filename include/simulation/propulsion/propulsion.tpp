#pragma once
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/util/util.hpp"

namespace propulsion {

    template <typename T>
    T compute_propeller_omega_T(const actuators::PropulsorActuator& propulsor, const T& thrust, const atmospheric::AirDensity& rho) {
        if (!propulsor.propellers) { return T(0); }
        const actuators::PropellerAssembly& propellers = propulsor.propellers.value();
        const double denom = propellers.thrust_coeff * rho.data * propellers.diameter * propellers.diameter * propellers.diameter * propellers.diameter;
        const T n = util::sqrt((thrust + T(constants::eps)) / T(denom));
        return T(2.0 * constants::pi) * n;
    }

    template <typename T>
    T compute_propeller_torque_T(const actuators::PropulsorActuator& propulsor, const T& omega, const atmospheric::AirDensity& rho) {
        if (!propulsor.propellers) { return T(0); }
        const actuators::PropellerAssembly& propellers = propulsor.propellers.value();
        const T n = omega / T(2.0 * constants::pi);
        return T(propellers.torque_coeff * rho.data * propellers.diameter * propellers.diameter * propellers.diameter * propellers.diameter * propellers.diameter) * n * n;
    }

    template <typename T>
    dynamics::Wrench_T<T> step_propulsor_forces_moments_T(const actuators::PropulsorActuator& propulsor, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atm_state, const T& thrust, const T& omega_dot) {
        dynamics::Wrench_T<T> out;

        const constants::Vector3_T<T>& wB_BI = twist.w;

        const constants::Vector3_T<T> n_prop = propulsor.n_prop.cast<T>();
        const constants::Vector3_T<T> r_prop = propulsor.pB_prop_cg.cast<T>();
        const atmospheric::AirDensity& rho = static_atm_state.rho;

        out.F = n_prop * thrust;
        out.M = r_prop.cross(out.F);

        if (propulsor.propellers.has_value()) {
            const actuators::PropellerAssembly& propellers = propulsor.propellers.value();
            const T omega = compute_propeller_omega_T<T>(propulsor, thrust, rho);
            const T q_prop = compute_propeller_torque_T<T>(propulsor, omega, rho);
            const constants::Vector3_T<T> H_prop = n_prop * (T(propellers.spin_inertia * propellers.spin_sign) * omega);

            out.M += -wB_BI.cross(H_prop);  // Gyroscopic moment
            out.M += -n_prop * (T(propellers.spin_inertia * propellers.spin_sign) * omega_dot); // Spin-up / angular-acceleration reaction moment
            out.M += -n_prop * (T(propellers.spin_sign) * q_prop);  // Aerodynamic reaction torque
        }

        return out;
    }

    template <typename T>
    dynamics::Wrench_T<T> step_propulsive_forces_moments_T(const actuators::PropulsorActuators& propulsor_actuators, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atm_state, const actuators::PropulsorActuatorInputs_T<T>& u, const PropulsorOmegaDot_T<T>& omega_dot) {
        dynamics::Wrench_T<T> total;

        const dynamics::Wrench_T<T> front = step_propulsor_forces_moments_T<T>(propulsor_actuators.front_propulsor, twist, static_atm_state, u.front_propulsor_cmd, omega_dot.front_propulsor);
        const dynamics::Wrench_T<T> left = step_propulsor_forces_moments_T<T>(propulsor_actuators.left_propulsor, twist, static_atm_state, u.left_propulsor_cmd, omega_dot.left_propulsor);
        const dynamics::Wrench_T<T> right = step_propulsor_forces_moments_T<T>(propulsor_actuators.right_propulsor, twist, static_atm_state, u.right_propulsor_cmd, omega_dot.right_propulsor);

        total.F = front.F + left.F + right.F;
        total.M = front.M + left.M + right.M;

        return total;
    }

}
