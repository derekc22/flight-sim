#pragma once
#include "simulation/propulsion/public/detail/loads.hpp"

namespace propulsion {

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
    dynamics::Wrench_T<T> compute_propulsor_loads_T(const actuators::PropulsorActuator& propulsor, const constants::Vector3_T<T>& center_of_gravity, const dynamics::Twist_T<T>& vehicle_twist, const atmospheric::StaticAtmosphericState& atmosphere, const T& thrust, const PropellerOmegaState_T<T>& propeller_state) {
        dynamics::Wrench_T<T> out;

        const constants::Vector3_T<T>& wB_BI = vehicle_twist.w;

        const constants::Vector3_T<T> n_prop = propulsor.n_prop.cast<T>();
        const constants::Vector3_T<T> pG_propG = propulsor.pB_propB.cast<T>() - center_of_gravity;
        const atmospheric::AirDensity& rho = atmosphere.rho;

        out.F = n_prop * thrust;
        out.M = pG_propG.cross(out.F);

        if (propulsor.propellers.has_value()) {
            const actuators::PropellerAssembly& propellers = propulsor.propellers.value();
            const T q_prop = compute_propeller_torque_T<T>(propulsor, propeller_state.omega, rho);
            const constants::Vector3_T<T> H_prop = n_prop * (T(propellers.spin_inertia * propellers.spin_sign) * propeller_state.omega);

            out.M += -wB_BI.cross(H_prop);
            out.M += -n_prop * (T(propellers.spin_inertia * propellers.spin_sign) * propeller_state.omega_dot);
            out.M += -n_prop * (T(propellers.spin_sign) * q_prop);
        }

        return out;
    }

    template <typename T>
    dynamics::Wrench_T<T> compute_propulsive_loads_T(const actuators::PropulsorActuators& propulsors, const constants::Vector3_T<T>& center_of_gravity, const dynamics::Twist_T<T>& vehicle_twist, const atmospheric::StaticAtmosphericState& atmosphere, const actuators::PropulsorActuatorInputs_T<T>& actuator_inputs, const PropellerOmegaStateSet_T<T>& propeller_state) {
        dynamics::Wrench_T<T> total;

        const dynamics::Wrench_T<T> front = compute_propulsor_loads_T<T>(propulsors.front_propulsor, center_of_gravity, vehicle_twist, atmosphere, actuator_inputs.front_propulsor_cmd, propeller_state.front_propulsor);
        const dynamics::Wrench_T<T> left = compute_propulsor_loads_T<T>(propulsors.left_propulsor, center_of_gravity, vehicle_twist, atmosphere, actuator_inputs.left_propulsor_cmd, propeller_state.left_propulsor);
        const dynamics::Wrench_T<T> right = compute_propulsor_loads_T<T>(propulsors.right_propulsor, center_of_gravity, vehicle_twist, atmosphere, actuator_inputs.right_propulsor_cmd, propeller_state.right_propulsor);

        total.F = front.F + left.F + right.F;
        total.M = front.M + left.M + right.M;

        return total;
    }

}
