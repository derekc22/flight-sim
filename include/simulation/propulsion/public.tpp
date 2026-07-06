#pragma once
#include <type_traits>
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
    dynamics::Wrench_T<T> step_propulsive_forces_moments_T(
        actuators::PropulsorActuators& propulsor_actuators, 
        const dynamics::Twist_T<T>& twist, 
        const atmospheric::StaticAtmosphericState& atm, 
        const actuators::PropulsorActuatorInputs_T<T>& u, 
        T dt,
        bool steady_state
    ) {
        dynamics::Wrench_T<T> total;

        // recall, the definition of steady trim is ẋ = f(x, u) = 0
        // that is, for trim, d/dt(·) = 0 must be enforced for all modelled state variables
        // however, even though omega_dot is not part of the modelled aircraft state, x, it too is set to 0
        // this is done because nonzero omega_dot represents a non-steady propeller transient that creates a moment on the aircraft
        // this moment, if unbalanced, can cause ẋ != 0, which violates the condition of steady equilibrium
        // thus, setting omega_dot to 0 allows equilibrium to be enforced
        // note: a nonzero omega_dot only violates the reduced aircraft trim condition if its induced moment is unbalanced and causes the modelled aircraft state derivatives to be nonzero
        // however, for a true steady operating point, omega_dot must still be zero, because otherwise the omitted propeller spin state is changing
        PropulsorOmegaDot_T omega_dot = steady_state ? 
            PropulsorOmegaDot_T<T>{} :
            step_propellers_omega_dot_T<T>(
                propulsor_actuators, 
                u, 
                atm, 
                dt
            );

        const dynamics::Wrench_T<T> front = step_propulsor_forces_moments_T<T>(propulsor_actuators.front_propulsor, twist, atm, u.front_propulsor_cmd, omega_dot.front_propulsor);
        const dynamics::Wrench_T<T> left = step_propulsor_forces_moments_T<T>(propulsor_actuators.left_propulsor, twist, atm, u.left_propulsor_cmd, omega_dot.left_propulsor);
        const dynamics::Wrench_T<T> right = step_propulsor_forces_moments_T<T>(propulsor_actuators.right_propulsor, twist, atm, u.right_propulsor_cmd, omega_dot.right_propulsor);

        total.F = front.F + left.F + right.F;
        total.M = front.M + left.M + right.M;

        return total;
    }

    template <typename T>
    PropulsorOmegaDot_T<T> step_propellers_omega_dot_T(actuators::PropulsorActuators& propulsor_actuators, const actuators::PropulsorActuatorInputs_T<T>& u, const atmospheric::StaticAtmosphericState& atm, T dt) {
        return {
            .front_propulsor = step_propeller_omega_dot_T(propulsor_actuators.front_propulsor, u.front_propulsor_cmd, atm.rho, dt),
            .left_propulsor = step_propeller_omega_dot_T(propulsor_actuators.left_propulsor, u.left_propulsor_cmd, atm.rho, dt),
            .right_propulsor = step_propeller_omega_dot_T(propulsor_actuators.right_propulsor, u.right_propulsor_cmd, atm.rho, dt)
        };
    }

    template <typename T>
    T step_propeller_omega_dot_T(actuators::PropulsorActuator& propulsor, T thrust, const atmospheric::AirDensity& rho, T dt) {
        if (!propulsor.propellers.has_value()) {
            return T(0.0); 
        }

        T omega = compute_propeller_omega_T<T>(propulsor, thrust, rho);
        T prev_omega = propulsor.propellers->prev_omega.has_value() ? T(propulsor.propellers->prev_omega.value()) : omega;

        // only the runtime/non-autodiff path should enter this branch because
        // 1) a type error will occur if this line attempts to assign a CppAD::AD<double> to a double field
        // 2) autodiff/trim/linearization/etc should have no side-effects; it should only evaluate simulation state, not mutate it 
        if constexpr (std::is_same_v<T, double>) {
            propulsor.propellers->prev_omega = omega;
        }

        return (omega - prev_omega) / dt;
    }



}
