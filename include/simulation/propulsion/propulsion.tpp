#pragma once

namespace propulsion {

    template <typename T>
    PropulsiveWrench_T<T> step_propulsive_forces_moments_T(const actuators::PropulsorActuators& propulsor_actuators, const PropulsorActuatorInputs_T<T>& u) {
        PropulsiveWrench_T<T> total;

        const constants::Vector3_T<T> n_prop_f = propulsor_actuators.front_propulsor.n_prop.cast<T>();
        const constants::Vector3_T<T> n_prop_l = propulsor_actuators.left_propulsor.n_prop.cast<T>();
        const constants::Vector3_T<T> n_prop_r = propulsor_actuators.right_propulsor.n_prop.cast<T>();

        const constants::Vector3_T<T> r_front = propulsor_actuators.front_propulsor.pB_prop_cg.cast<T>();
        const constants::Vector3_T<T> r_left = propulsor_actuators.left_propulsor.pB_prop_cg.cast<T>();
        const constants::Vector3_T<T> r_right = propulsor_actuators.right_propulsor.pB_prop_cg.cast<T>();

        const constants::Vector3_T<T> F_front = n_prop_f * u.front_propulsor_cmd;
        const constants::Vector3_T<T> F_left = n_prop_l * u.left_propulsor_cmd;
        const constants::Vector3_T<T> F_right = n_prop_r * u.right_propulsor_cmd;

        const constants::Vector3_T<T> M_front = r_front.cross(F_front);
        const constants::Vector3_T<T> M_left = r_left.cross(F_left);
        const constants::Vector3_T<T> M_right = r_right.cross(F_right);

        total.F = F_front + F_left + F_right;
        total.M = M_front + M_left + M_right;

        return total;
    }

}