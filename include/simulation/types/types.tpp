#pragma once

namespace types {

    template <typename T>
    ActuatorInputsVector_T<T> unpack_actuator_inputs_T(const ActuatorInputs_T<T>& u) {
        ActuatorInputsVector_T<T> out;
        out << u.elevator_cmd, u.aileron_cmd, u.rudder_cmd,
               u.front_propulsor_cmd, u.left_propulsor_cmd, u.right_propulsor_cmd;
        return out;
    }

    template <typename T>
    StateVector_T<T> unpack_state_T(const State_T<T>& x) {
        StateVector_T<T> out;
        out << x.vx, x.vy, x.vz,
               x.p, x.q, x.r,
               x.phi, x.theta;
        return out;
    }

    template <typename T>
    StateDotVector_T<T> unpack_state_dot_T(const StateDot_T<T>& x_dot) {
        StateDotVector_T<T> out;
        out << x_dot.vx_dot, x_dot.vy_dot, x_dot.vz_dot,
               x_dot.p_dot, x_dot.q_dot, x_dot.r_dot,
               x_dot.phi_dot, x_dot.theta_dot;
        return out;
    }





}