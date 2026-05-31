#pragma once
#include "simulation/actuators/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/operating/public.hpp"

namespace operating {

    template <typename T>
    StateInputVector_T<T> unpack_state_input_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u) {
        StateInputVector_T<T> out;
        out << x.vx, x.vy, x.vz,
               x.p, x.q, x.r,
               x.phi, x.theta,
               u.elevator_cmd, u.aileron_cmd, u.rudder_cmd,
               u.front_propulsor_cmd, u.left_propulsor_cmd, u.right_propulsor_cmd;
        return out;
    }

    template <typename T>
    dynamics::State_T<T> pack_state_T(const StateInputVector_T<T>& z) {
        return {
            .vx = z(0),
            .vy = z(1),
            .vz = z(2),
            .p = z(3),
            .q = z(4),
            .r = z(5),
            .phi = z(6),
            .theta = z(7),
        };
    }

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_actuator_inputs_T(const StateInputVector_T<T>& z) {
        return {
            .elevator_cmd = z(8),
            .aileron_cmd = z(9),
            .rudder_cmd = z(10),

            .front_propulsor_cmd = z(11),
            .left_propulsor_cmd = z(12),
            .right_propulsor_cmd = z(13),
        };
    }

}
