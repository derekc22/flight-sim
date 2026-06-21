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
    dynamics::State_T<T> pack_state_T(const StateInputVector_T<T>& xu) {
        return {
            .vx = xu(0),
            .vy = xu(1),
            .vz = xu(2),
            .p = xu(3),
            .q = xu(4),
            .r = xu(5),
            .phi = xu(6),
            .theta = xu(7),
        };
    }

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_actuator_inputs_T(const StateInputVector_T<T>& xu) {
        return {
            .elevator_cmd = xu(8),
            .aileron_cmd = xu(9),
            .rudder_cmd = xu(10),

            .front_propulsor_cmd = xu(11),
            .left_propulsor_cmd = xu(12),
            .right_propulsor_cmd = xu(13),
        };
    }

}
