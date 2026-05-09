#pragma once

namespace actuators {

    template <typename T>
    ActuatorInputs_T<T> pack_actuator_inputs_T(const ActuatorInputsVector_T<T>& u) {
        return {
            .elevator_cmd = u(0),
            .aileron_cmd = u(1),
            .rudder_cmd = u(2),
            .front_propulsor_cmd = u(3),
            .left_propulsor_cmd = u(4),
            .right_propulsor_cmd = u(5),
        };
    }

    template <typename T>
    ActuatorInputsVector_T<T> unpack_actuator_inputs_T(const ActuatorInputs_T<T>& u) {
        ActuatorInputsVector_T<T> out;
        out << u.elevator_cmd, u.aileron_cmd, u.rudder_cmd,
               u.front_propulsor_cmd, u.left_propulsor_cmd, u.right_propulsor_cmd;
        return out;
    }

    template <typename T>
    ActuatorLimits_T<T> pack_actuator_limits_T(const ActuatorLimitsVector_T<T>& limits) {
        const ActuatorInputsVector_T<T> limit_min = limits.col(0);
        const ActuatorInputsVector_T<T> limit_max = limits.col(1);
        return {
            .limit_min = pack_actuator_inputs_T<T>(limit_min),
            .limit_max = pack_actuator_inputs_T<T>(limit_max),
        };
    }

    template <typename T>
    ActuatorLimitsVector_T<T> unpack_actuator_limits_T(const ActuatorLimits_T<T>& limits) {
        ActuatorLimitsVector_T<T> out;
        out.col(0) = unpack_actuator_inputs_T<T>(limits.limit_min);
        out.col(1) = unpack_actuator_inputs_T<T>(limits.limit_max);
        return out;
    }

}
